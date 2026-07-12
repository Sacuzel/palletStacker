"""Export packed pallet layouts to one Gazebo SDF world.

Use from main.py:
    from gazebo_exporter import write_layout_sdf
    sdf_path = write_layout_sdf(pallets)

Default output:
    gazebo_runs/latest/pallet_towers.sdf
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import subprocess
import time
from pathlib import Path
from typing import Any, List, Sequence, Tuple
from xml.sax.saxutils import escape

import config
from algorithm import pack_boxes
from models import Box
from pallet import Pallet

MM_TO_M = 0.001


def safe_name(value: object, fallback: str = "item") -> str:
    raw = str(value or fallback)
    cleaned = re.sub(r"[^A-Za-z0-9_]+", "_", raw).strip("_")
    return cleaned or fallback


def fmt(value: float) -> str:
    return f"{float(value):.9g}"


def mm_to_m(value_mm: float) -> float:
    return float(value_mm) * MM_TO_M


def stable_rgba(seed: str, alpha: float = 1.0) -> Tuple[float, float, float, float]:
    digest = hashlib.md5(seed.encode("utf-8")).digest()
    return (
        (80 + digest[0] % 140) / 255.0,
        (80 + digest[1] % 140) / 255.0,
        (80 + digest[2] % 140) / 255.0,
        alpha,
    )


def rgba_text(rgba: Tuple[float, float, float, float]) -> str:
    return " ".join(fmt(v) for v in rgba)


def box_inertia(mass_kg: float, sx_m: float, sy_m: float, sz_m: float) -> Tuple[float, float, float]:
    mass = max(float(mass_kg), 0.001)
    ixx = mass / 12.0 * (sy_m * sy_m + sz_m * sz_m)
    iyy = mass / 12.0 * (sx_m * sx_m + sz_m * sz_m)
    izz = mass / 12.0 * (sx_m * sx_m + sy_m * sy_m)
    return max(ixx, 1e-9), max(iyy, 1e-9), max(izz, 1e-9)


def surface_xml(mu: float, restitution: float, kp: float, kd: float) -> str:
    mu = max(float(mu), 0.0)
    restitution = max(float(restitution), 0.0)

    return f"""
          <surface>
            <friction>
              <ode>
                <mu>{fmt(mu)}</mu>
                <mu2>{fmt(mu)}</mu2>
              </ode>
            </friction>
            <bounce>
              <restitution_coefficient>{fmt(restitution)}</restitution_coefficient>
              <threshold>100000</threshold>
            </bounce>
            <contact>
              <ode>
                <kp>{fmt(kp)}</kp>
                <kd>{fmt(kd)}</kd>
                <max_vel>1</max_vel>
                <min_depth>0.0005</min_depth>
              </ode>
            </contact>
          </surface>"""


def cuboid_model_xml(
    *,
    model_name: str,
    pose_xyz_m: Tuple[float, float, float],
    size_xyz_m: Tuple[float, float, float],
    mass_kg: float,
    color_rgba: Tuple[float, float, float, float],
    friction: float,
    restitution: float,
    contact_kp: float,
    contact_kd: float,
    static: bool = False,
) -> str:
    sx, sy, sz = size_xyz_m
    px, py, pz = pose_xyz_m
    ixx, iyy, izz = box_inertia(mass_kg, sx, sy, sz)

    size = f"{fmt(sx)} {fmt(sy)} {fmt(sz)}"
    color = rgba_text(color_rgba)
    static_text = "true" if static else "false"

    return f"""
    <model name=\"{escape(safe_name(model_name))}\">
      <static>{static_text}</static>
      <pose>{fmt(px)} {fmt(py)} {fmt(pz)} 0 0 0</pose>
      <link name=\"body\">
        <gravity>true</gravity>
        <inertial>
          <mass>{fmt(mass_kg)}</mass>
          <inertia>
            <ixx>{fmt(ixx)}</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>{fmt(iyy)}</iyy>
            <iyz>0</iyz>
            <izz>{fmt(izz)}</izz>
          </inertia>
        </inertial>

        <collision name=\"collision\">
          <geometry>
            <box>
              <size>{size}</size>
            </box>
          </geometry>
          {surface_xml(friction, restitution, contact_kp, contact_kd)}
        </collision>

        <visual name=\"visual\">
          <geometry>
            <box>
              <size>{size}</size>
            </box>
          </geometry>
          <material>
            <ambient>{color}</ambient>
            <diffuse>{color}</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
      </link>
    </model>"""


def ground_plane_xml(friction: float, restitution: float, contact_kp: float, contact_kd: float) -> str:
    return f"""
    <model name=\"ground_plane\">
      <static>true</static>
      <link name=\"link\">
        <collision name=\"collision\">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>30 30</size>
            </plane>
          </geometry>
          {surface_xml(friction, restitution, contact_kp, contact_kd)}
        </collision>

        <visual name=\"visual\">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>30 30</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.65 0.65 0.65 1</ambient>
            <diffuse>0.65 0.65 0.65 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""


def _prepare_output_path(
    output_path: str | Path | None,
    output_dir: str | Path,
    run_name: str | None,
    overwrite: bool,
) -> Path:
    if output_path is not None:
        path = Path(output_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        return path

    if run_name is None:
        run_name = time.strftime("%Y%m%d_%H%M%S_pallet_towers")

    run_dir = Path(output_dir) / safe_name(run_name)

    if run_dir.exists() and overwrite:
        shutil.rmtree(run_dir)

    run_dir.mkdir(parents=True, exist_ok=True)
    return run_dir / "pallet_towers.sdf"


def _build_models_and_manifest(
    pallets: Sequence[Pallet],
    *,
    pallet_gap_mm: float,
    friction: float,
    pallet_friction: float,
    restitution: float,
    contact_kp: float,
    contact_kd: float,
    settle_lift_mm: float,
) -> Tuple[str, List[dict[str, Any]]]:
    models: List[str] = []
    manifest: List[dict[str, Any]] = []

    pallet_l_m = mm_to_m(config.PALLET_LENGTH)
    pallet_w_m = mm_to_m(config.PALLET_WIDTH)
    pallet_h_m = mm_to_m(config.PALLET_BASE_HEIGHT)

    for pallet_index, pallet in enumerate(pallets, start=1):
        x_offset_mm = (pallet_index - 1) * (pallet.length + pallet_gap_mm)
        x_offset_m = mm_to_m(x_offset_mm)

        pallet_model_name = f"pallet_{pallet_index:02d}_{safe_name(pallet.pallet_id)}"

        models.append(
            cuboid_model_xml(
                model_name=pallet_model_name,
                pose_xyz_m=(
                    x_offset_m + pallet_l_m / 2.0,
                    pallet_w_m / 2.0,
                    pallet_h_m / 2.0 + 0.002,
                ),
                size_xyz_m=(pallet_l_m, pallet_w_m, pallet_h_m),
                mass_kg=float(config.PALLET_WEIGHT_KG),
                color_rgba=(0.50, 0.34, 0.16, 1.0),
                friction=pallet_friction,
                restitution=restitution,
                contact_kp=contact_kp,
                contact_kd=contact_kd,
                static=False,
            )
        )

        manifest.append(
            {
                "type": "pallet",
                "model_name": pallet_model_name,
                "pallet_id": pallet.pallet_id,
                "mass_kg": float(config.PALLET_WEIGHT_KG),
            }
        )

        for box_index, placement in enumerate(pallet.placements, start=1):
            sx = mm_to_m(placement.length)
            sy = mm_to_m(placement.width)
            sz = mm_to_m(placement.height)

            px = x_offset_m + mm_to_m(placement.x) + sx / 2.0
            py = mm_to_m(placement.y) + sy / 2.0
            pz = pallet_h_m + mm_to_m(placement.z + settle_lift_mm) + sz / 2.0

            box_id = safe_name(placement.box.identifier, f"box_{box_index}")
            sku = safe_name(placement.box.sku, "sku")
            model_name = f"box_p{pallet_index:02d}_{box_index:04d}_{box_id}_{sku}"

            color_seed = placement.box.sku or placement.box.identifier or model_name

            models.append(
                cuboid_model_xml(
                    model_name=model_name,
                    pose_xyz_m=(px, py, pz),
                    size_xyz_m=(sx, sy, sz),
                    mass_kg=float(placement.box.weight),
                    color_rgba=stable_rgba(color_seed),
                    friction=friction,
                    restitution=restitution,
                    contact_kp=contact_kp,
                    contact_kd=contact_kd,
                    static=False,
                )
            )

            manifest.append(
                {
                    "type": "box",
                    "model_name": model_name,
                    "pallet_id": pallet.pallet_id,
                    "identifier": placement.box.identifier,
                    "sku": placement.box.sku,
                    "mass_kg": float(placement.box.weight),
                    "local_position_mm": [placement.x, placement.y, placement.z],
                    "size_mm": [placement.length, placement.width, placement.height],
                    "world_position_m": [px, py, pz],
                }
            )

    return "".join(models), manifest


def create_world_sdf(
    pallets: Sequence[Pallet],
    *,
    pallet_gap_mm: float = 500.0,
    friction: float = config.SIM_FRICTION_COEFF,
    pallet_friction: float | None = None,
    floor_friction: float | None = None,
    restitution: float = 0.0,
    contact_kp: float = 1_000_000.0,
    contact_kd: float = 100.0,
    settle_lift_mm: float = 0.0,
    start_paused: bool = True,
) -> Tuple[str, List[dict[str, Any]]]:
    if not pallets:
        raise ValueError("No pallets to export.")

    pallet_friction = friction if pallet_friction is None else pallet_friction
    floor_friction = friction if floor_friction is None else floor_friction

    models_xml, manifest = _build_models_and_manifest(
        pallets,
        pallet_gap_mm=pallet_gap_mm,
        friction=friction,
        pallet_friction=pallet_friction,
        restitution=restitution,
        contact_kp=contact_kp,
        contact_kd=contact_kd,
        settle_lift_mm=settle_lift_mm,
    )

    paused = "true" if start_paused else "false"

    sdf = f"""<?xml version=\"1.0\" ?>
<sdf version=\"1.10\">
  <world name=\"pallet_towers\">
    <gravity>0 0 -9.81</gravity>

    <physics name=\"pallet_physics\" type=\"ignored\">
      <max_step_size>0.005</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>200</real_time_update_rate>
      <max_contacts>30</max_contacts>
    </physics>

    <plugin filename=\"gz-sim-physics-system\" name=\"gz::sim::systems::Physics\" />
    <plugin filename=\"gz-sim-user-commands-system\" name=\"gz::sim::systems::UserCommands\" />
    <plugin filename=\"gz-sim-scene-broadcaster-system\" name=\"gz::sim::systems::SceneBroadcaster\" />

    <light type=\"directional\" name=\"sun\">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.2 -0.9</direction>
    </light>

{ground_plane_xml(floor_friction, restitution, contact_kp, contact_kd)}
{models_xml}
  </world>
</sdf>
"""

    return sdf, manifest


def write_layout_sdf(
    pallets: Sequence[Pallet],
    output_path: str | Path | None = None,
    *,
    output_dir: str | Path = "gazebo_runs",
    run_name: str | None = "latest",
    overwrite: bool = True,
    pallet_gap_mm: float = 500.0,
    friction: float = config.SIM_FRICTION_COEFF,
    pallet_friction: float | None = None,
    floor_friction: float | None = None,
    restitution: float = 0.0,
    contact_kp: float = 1_000_000.0,
    contact_kd: float = 100.0,
    settle_lift_mm: float = 0.0,
    start_paused: bool = True,
) -> Path:
    """Write all packed pallet towers to one grouped Gazebo SDF world.

    This is the function main.py imports.
    Default output is gazebo_runs/latest/pallet_towers.sdf.
    """
    path = _prepare_output_path(output_path, output_dir, run_name, overwrite)

    sdf, manifest = create_world_sdf(
        pallets,
        pallet_gap_mm=pallet_gap_mm,
        friction=friction,
        pallet_friction=pallet_friction,
        floor_friction=floor_friction,
        restitution=restitution,
        contact_kp=contact_kp,
        contact_kd=contact_kd,
        settle_lift_mm=settle_lift_mm,
        start_paused=start_paused,
    )

    path.write_text(sdf, encoding="utf-8")

    manifest_path = path.with_name("manifest.json")
    manifest_path.write_text(
        json.dumps(
            {
                "sdf_file": path.name,
                "pallet_count": len(pallets),
                "box_count": sum(len(p.placements) for p in pallets),
                "items": manifest,
            },
            indent=2,
        ),
        encoding="utf-8",
    )

    return path


def load_boxes_from_json(path: str | Path) -> List[Box]:
    with Path(path).open("r", encoding="utf-8") as handle:
        data = json.load(handle)

    return [
        Box(
            identifier=item["identifier"],
            sku=item["sku"],
            length=float(item["dimensions_mm"][0]),
            width=float(item["dimensions_mm"][1]),
            height=float(item["dimensions_mm"][2]),
            weight=float(item["weight_kg"]),
        )
        for item in data.get("boxes", [])
    ]


def export_from_json(
    json_path: str | Path,
    output_path: str | Path | None = None,
    **kwargs: Any,
) -> Path:
    boxes = load_boxes_from_json(json_path)
    pallets = pack_boxes(boxes)
    return write_layout_sdf(pallets, output_path=output_path, **kwargs)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Export all packed pallets to one Gazebo SDF world.")
    parser.add_argument("json", type=Path, help="Input JSON file")
    parser.add_argument("-o", "--output", type=Path, default=None, help="Exact SDF output path")
    parser.add_argument("--output-dir", type=Path, default=Path("gazebo_runs"))
    parser.add_argument("--run-name", default="latest")
    parser.add_argument("--timestamped", action="store_true")
    parser.add_argument("--friction", type=float, default=config.SIM_FRICTION_COEFF)
    parser.add_argument("--pallet-gap-mm", type=float, default=500.0)
    parser.add_argument("--settle-lift-mm", type=float, default=0.0)
    parser.add_argument("--unpaused", action="store_true")
    parser.add_argument("--run-gazebo", action="store_true")
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()

    run_name = None if args.timestamped else args.run_name

    sdf_path = export_from_json(
        args.json,
        output_path=args.output,
        output_dir=args.output_dir,
        run_name=run_name,
        overwrite=True,
        friction=args.friction,
        pallet_gap_mm=args.pallet_gap_mm,
        settle_lift_mm=args.settle_lift_mm,
        start_paused=not args.unpaused,
    )

    print(f"Wrote Gazebo world: {sdf_path.resolve()}")

    if args.run_gazebo:
        subprocess.run(["gz", "sim", str(sdf_path)], check=False)


if __name__ == "__main__":
    main()