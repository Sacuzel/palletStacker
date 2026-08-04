"""Export packed pallet layouts to one Gazebo SDF world.

Use from main.py:
    from gazebo_exporter import write_layout_sdf
    sdf_path = write_layout_sdf(pallets)

Default output:
    gazebo_runs/latest/pallet_towers.sdf
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import re
import shutil
import subprocess
import time
from pathlib import Path
import xml.etree.ElementTree as ET
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


def _ensure_xml_path(parent: ET.Element, path: str) -> ET.Element:
    """Return an XML descendant, creating missing elements on the path."""
    node = parent
    for tag in path.split("/"):
        child = node.find(tag)
        if child is None:
            child = ET.SubElement(node, tag)
        node = child
    return node


def _project_path(path_value: str | Path) -> Path:
    """Resolve a project-relative resource beside this exporter module."""
    path = Path(path_value)
    if path.is_absolute():
        return path
    return Path(__file__).resolve().parent / path


def dynamic_pallet_model_xml(
    *,
    model_name: str,
    pose_xyz_m: Tuple[float, float, float],
    model_sdf_path: str | Path,
    mass_kg: float,
    friction: float,
    restitution: float,
    contact_kp: float,
    contact_kd: float,
) -> str:
    """Load the standalone pallet model and inline one configured copy.

    Inlining makes the generated world self-contained: the customer does not
    need to set GZ_SIM_RESOURCE_PATH or download a Fuel model. The standalone
    models/euro_pallet/model.sdf file remains directly usable as a Gazebo model.
    """
    source_path = _project_path(model_sdf_path)
    if not source_path.is_file():
        raise FileNotFoundError(f"Euro pallet model not found: {source_path}")

    source_root = ET.parse(source_path).getroot()
    source_model = source_root if source_root.tag == "model" else source_root.find("model")
    if source_model is None:
        raise ValueError(f"No <model> element found in {source_path}")

    model = copy.deepcopy(source_model)
    model.set("name", safe_name(model_name))
    _ensure_xml_path(model, "static").text = "false"

    px, py, pz = pose_xyz_m
    pose = model.find("pose")
    if pose is None:
        pose = ET.Element("pose")
        model.insert(1, pose)
    pose.text = f"{fmt(px)} {fmt(py)} {fmt(pz)} 0 0 0"

    # Preserve the compound mass distribution if PALLET_WEIGHT_KG is tuned.
    inertial = model.find("./link/inertial")
    if inertial is None:
        raise ValueError(f"Pallet model has no link/inertial block: {source_path}")
    mass_node = _ensure_xml_path(inertial, "mass")
    source_mass = float(mass_node.text or 0.0)
    target_mass = max(float(mass_kg), 0.001)
    inertia_scale = target_mass / source_mass if source_mass > 0.0 else 1.0
    mass_node.text = fmt(target_mass)
    inertia = _ensure_xml_path(inertial, "inertia")
    for component in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
        node = _ensure_xml_path(inertia, component)
        node.text = fmt(float(node.text or 0.0) * inertia_scale)

    # Apply the export-time contact parameters to all five primitive members.
    for collision in model.findall("./link/collision"):
        _ensure_xml_path(collision, "surface/friction/ode/mu").text = fmt(max(friction, 0.0))
        _ensure_xml_path(collision, "surface/friction/ode/mu2").text = fmt(max(friction, 0.0))
        _ensure_xml_path(collision, "surface/bounce/restitution_coefficient").text = fmt(
            max(restitution, 0.0)
        )
        _ensure_xml_path(collision, "surface/bounce/threshold").text = "100000"
        _ensure_xml_path(collision, "surface/contact/ode/kp").text = fmt(contact_kp)
        _ensure_xml_path(collision, "surface/contact/ode/kd").text = fmt(contact_kd)
        _ensure_xml_path(collision, "surface/contact/ode/max_vel").text = "1"
        _ensure_xml_path(collision, "surface/contact/ode/min_depth").text = "0.0005"

    ET.indent(model, space="      " , level=0)
    return "\n    " + ET.tostring(model, encoding="unicode").replace("\n", "\n    ")



def _set_box_inertial(
    inertial: ET.Element,
    *,
    mass_kg: float,
    size_xyz_m: Tuple[float, float, float],
    pose_xyz_m: Tuple[float, float, float] | None = None,
) -> None:
    """Configure a box-shaped link inertial block from one source of truth."""
    sx, sy, sz = size_xyz_m
    ixx, iyy, izz = box_inertia(mass_kg, sx, sy, sz)
    _ensure_xml_path(inertial, "mass").text = fmt(max(mass_kg, 0.001))
    inertia = _ensure_xml_path(inertial, "inertia")
    _ensure_xml_path(inertia, "ixx").text = fmt(ixx)
    _ensure_xml_path(inertia, "ixy").text = "0"
    _ensure_xml_path(inertia, "ixz").text = "0"
    _ensure_xml_path(inertia, "iyy").text = fmt(iyy)
    _ensure_xml_path(inertia, "iyz").text = "0"
    _ensure_xml_path(inertia, "izz").text = fmt(izz)
    if pose_xyz_m is not None:
        _ensure_xml_path(inertial, "pose").text = (
            f"{fmt(pose_xyz_m[0])} {fmt(pose_xyz_m[1])} "
            f"{fmt(pose_xyz_m[2])} 0 0 0"
        )


def _configure_collision_surface(
    collision: ET.Element,
    *,
    friction: float,
    restitution: float,
    contact_kp: float,
    contact_kd: float,
    min_depth: float,
) -> None:
    """Apply common contact values to one primitive collision."""
    _ensure_xml_path(collision, "surface/friction/ode/mu").text = fmt(max(friction, 0.0))
    _ensure_xml_path(collision, "surface/friction/ode/mu2").text = fmt(max(friction, 0.0))
    _ensure_xml_path(collision, "surface/bounce/restitution_coefficient").text = fmt(
        max(restitution, 0.0)
    )
    _ensure_xml_path(collision, "surface/bounce/threshold").text = "100000"
    _ensure_xml_path(collision, "surface/contact/ode/kp").text = fmt(contact_kp)
    _ensure_xml_path(collision, "surface/contact/ode/kd").text = fmt(contact_kd)
    _ensure_xml_path(collision, "surface/contact/ode/max_vel").text = "0.5"
    _ensure_xml_path(collision, "surface/contact/ode/min_depth").text = fmt(min_depth)


def dynamic_forklift_model_xml(
    *,
    model_name: str,
    pose_xyz_rpy_m: Tuple[float, float, float, float, float, float],
    model_sdf_path: str | Path,
    restitution: float,
    contact_kp: float,
    contact_kd: float,
) -> str:
    """Load, configure and inline the three-link forklift model.

    All runtime geometry, inertial values, limits and topics are read from
    config.py. The source SDF remains a directly inspectable baseline model,
    while the generated world follows the single-point-of-change convention.
    """
    source_path = _project_path(model_sdf_path)
    if not source_path.is_file():
        raise FileNotFoundError(f"Forklift model not found: {source_path}")

    source_root = ET.parse(source_path).getroot()
    source_model = source_root if source_root.tag == "model" else source_root.find("model")
    if source_model is None:
        raise ValueError(f"No <model> element found in {source_path}")

    model = copy.deepcopy(source_model)
    model.set("name", safe_name(model_name))
    _ensure_xml_path(model, "static").text = "false"

    px, py, pz, roll, pitch, yaw = pose_xyz_rpy_m
    pose = model.find("pose")
    if pose is None:
        pose = ET.Element("pose")
        model.insert(1, pose)
    pose.text = (
        f"{fmt(px)} {fmt(py)} {fmt(pz)} "
        f"{fmt(roll)} {fmt(pitch)} {fmt(yaw)}"
    )

    body_l = float(config.FORKLIFT_BODY_LENGTH_M)
    body_w = float(config.FORKLIFT_BODY_WIDTH_M)
    body_h = float(config.FORKLIFT_BODY_HEIGHT_M)
    body_mass = float(config.FORKLIFT_BODY_MASS_KG)
    fork_l = float(config.FORKLIFT_FORK_LENGTH_M)
    fork_w = float(config.FORKLIFT_FORK_WIDTH_M)
    fork_t = float(config.FORKLIFT_FORK_THICKNESS_M)
    fork_mass = float(config.FORKLIFT_FORK_MASS_KG)
    fork_spacing = float(config.FORKLIFT_FORK_CENTRE_SPACING_M)
    fork_bottom = float(config.FORKLIFT_INITIAL_FORK_BOTTOM_M)
    fork_speed = float(config.FORKLIFT_FORK_SPEED_M_S)

    if min(body_l, body_w, body_h, fork_l, fork_w, fork_t) <= 0.0:
        raise ValueError("Forklift dimensions must all be positive.")
    if fork_bottom < 0.0:
        raise ValueError("FORKLIFT_INITIAL_FORK_BOTTOM_M cannot be negative.")

    body_link = model.find("./link[@name='base_link']")
    left_link = model.find("./link[@name='left_fork_link']")
    right_link = model.find("./link[@name='right_fork_link']")
    if body_link is None or left_link is None or right_link is None:
        raise ValueError(f"Forklift model is missing one of its three required links: {source_path}")

    body_size = (body_l, body_w, body_h)
    body_center_z = body_h / 2.0
    _set_box_inertial(
        _ensure_xml_path(body_link, "inertial"),
        mass_kg=body_mass,
        size_xyz_m=body_size,
        pose_xyz_m=(0.0, 0.0, body_center_z),
    )
    for element_name in ("collision", "visual"):
        element = body_link.find(element_name)
        if element is None:
            raise ValueError(f"Forklift body has no {element_name}: {source_path}")
        _ensure_xml_path(element, "pose").text = f"0 0 {fmt(body_center_z)} 0 0 0"
        _ensure_xml_path(element, "geometry/box/size").text = (
            f"{fmt(body_l)} {fmt(body_w)} {fmt(body_h)}"
        )
    body_collision = body_link.find("collision")
    assert body_collision is not None
    _configure_collision_surface(
        body_collision,
        friction=float(config.FORKLIFT_BODY_FRICTION_COEFF),
        restitution=restitution,
        contact_kp=contact_kp,
        contact_kd=contact_kd,
        min_depth=0.0005,
    )

    fork_x = body_l / 2.0 + fork_l / 2.0
    fork_center_z = fork_bottom + fork_t / 2.0
    fork_size = (fork_l, fork_w, fork_t)
    for link, y_m in ((left_link, fork_spacing / 2.0), (right_link, -fork_spacing / 2.0)):
        _ensure_xml_path(link, "pose").text = (
            f"{fmt(fork_x)} {fmt(y_m)} {fmt(fork_center_z)} 0 0 0"
        )
        _set_box_inertial(
            _ensure_xml_path(link, "inertial"),
            mass_kg=fork_mass,
            size_xyz_m=fork_size,
        )
        collision = link.find("collision")
        visual = link.find("visual")
        if collision is None or visual is None:
            raise ValueError(f"Forklift fork link lacks collision or visual: {source_path}")
        size_text = f"{fmt(fork_l)} {fmt(fork_w)} {fmt(fork_t)}"
        _ensure_xml_path(collision, "geometry/box/size").text = size_text
        _ensure_xml_path(visual, "geometry/box/size").text = size_text
        _configure_collision_surface(
            collision,
            friction=float(config.FORKLIFT_FORK_FRICTION_COEFF),
            restitution=restitution,
            contact_kp=contact_kp,
            contact_kd=contact_kd,
            min_depth=0.0002,
        )

    # Joint zero corresponds to the initial link pose. The top of a fork at
    # maximum travel is exactly equal to the upper face of the body.
    max_fork_travel = body_h - (fork_bottom + fork_t)
    if max_fork_travel < 0.0:
        raise ValueError("Initial fork upper face is above the forklift body upper face.")

    for joint_name in ("left_fork_joint", "right_fork_joint"):
        joint = model.find(f"./joint[@name='{joint_name}']")
        if joint is None:
            raise ValueError(f"Forklift joint not found: {joint_name}")
        _ensure_xml_path(joint, "axis/limit/lower").text = "0"
        _ensure_xml_path(joint, "axis/limit/upper").text = fmt(max_fork_travel)
        _ensure_xml_path(joint, "axis/limit/effort").text = fmt(
            float(config.FORKLIFT_FORK_JOINT_EFFORT_N)
        )
        _ensure_xml_path(joint, "axis/limit/velocity").text = fmt(fork_speed)
        _ensure_xml_path(joint, "axis/dynamics/damping").text = fmt(
            float(config.FORKLIFT_FORK_JOINT_DAMPING)
        )
        _ensure_xml_path(joint, "axis/dynamics/friction").text = fmt(
            float(config.FORKLIFT_FORK_JOINT_FRICTION)
        )

    for plugin in model.findall("plugin"):
        plugin_name = plugin.get("name", "")
        if plugin_name.endswith("VelocityControl"):
            _ensure_xml_path(plugin, "topic").text = str(config.FORKLIFT_CMD_VEL_TOPIC)
        elif plugin_name.endswith("JointController"):
            joint_name = (_ensure_xml_path(plugin, "joint_name").text or "").strip()
            topic = (
                config.FORKLIFT_LEFT_FORK_TOPIC
                if joint_name == "left_fork_joint"
                else config.FORKLIFT_RIGHT_FORK_TOPIC
            )
            _ensure_xml_path(plugin, "topic").text = str(topic)

            # Velocity mode directly tracks the requested joint speed and is
            # the least tuning-sensitive option for this deliberately simple
            # two-prismatic-joint model. Remove stale force-mode PID fields if
            # they are present in an older standalone model file.
            _ensure_xml_path(plugin, "use_force_commands").text = "false"
            for stale_name in ("p_gain", "i_gain", "d_gain", "i_max", "i_min", "cmd_max", "cmd_min"):
                stale = plugin.find(stale_name)
                if stale is not None:
                    plugin.remove(stale)

    ET.indent(model, space="      ", level=0)
    return "\n    " + ET.tostring(model, encoding="unicode").replace("\n", "\n    ")


def forklift_spawn_pose() -> Tuple[float, float, float, float, float, float]:
    """Place fork tips exactly the configured distance from pallet 1.

    Pallet 1 exposes a fork channel on its negative-Y face. The forklift's
    local +X axis is its forward direction, so a +90 degree yaw faces it toward
    positive world Y. The vector form keeps the calculation valid if yaw is
    tuned later.
    """
    yaw = float(config.FORKLIFT_SPAWN_YAW_RAD)
    forward_x = math.cos(yaw)
    forward_y = math.sin(yaw)
    pallet_face_x = mm_to_m(config.PALLET_LENGTH) / 2.0
    pallet_face_y = 0.0
    desired_tip_x = pallet_face_x - float(config.FORKLIFT_SPAWN_DISTANCE_M) * forward_x
    desired_tip_y = pallet_face_y - float(config.FORKLIFT_SPAWN_DISTANCE_M) * forward_y
    tip_from_origin = (
        float(config.FORKLIFT_BODY_LENGTH_M) / 2.0
        + float(config.FORKLIFT_FORK_LENGTH_M)
    )
    origin_x = desired_tip_x - tip_from_origin * forward_x
    origin_y = desired_tip_y - tip_from_origin * forward_y
    return (
        origin_x,
        origin_y,
        float(config.FORKLIFT_SPAWN_CLEARANCE_M),
        0.0,
        0.0,
        yaw,
    )

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
    pallet_spawn_clearance_m = (
        mm_to_m(config.EURO_PALLET_SPAWN_CLEARANCE_MM)
        if config.USE_EURO_PALLET_MODEL
        else 0.005
    )

    for pallet_index, pallet in enumerate(pallets, start=1):
        x_offset_mm = (pallet_index - 1) * (pallet.length + pallet_gap_mm)
        x_offset_m = mm_to_m(x_offset_mm)

        pallet_model_name = f"pallet_{pallet_index:02d}_{safe_name(pallet.pallet_id)}"

        if config.USE_EURO_PALLET_MODEL:
            # The standalone pallet model uses a floor-level origin at the
            # centre of its 800 x 1200 mm footprint. The channels therefore
            # run parallel to world Y and are open from both 1200 mm ends.
            models.append(
                dynamic_pallet_model_xml(
                    model_name=pallet_model_name,
                    pose_xyz_m=(
                        x_offset_m + pallet_l_m / 2.0,
                        pallet_w_m / 2.0,
                        pallet_spawn_clearance_m,
                    ),
                    model_sdf_path=config.EURO_PALLET_MODEL_SDF,
                    mass_kg=float(config.PALLET_WEIGHT_KG),
                    friction=pallet_friction,
                    restitution=restitution,
                    contact_kp=contact_kp,
                    contact_kd=contact_kd,
                )
            )
        else:
            # Backward-compatible fallback for quick exporter diagnostics.
            models.append(
                cuboid_model_xml(
                    model_name=pallet_model_name,
                    pose_xyz_m=(
                        x_offset_m + pallet_l_m / 2.0,
                        pallet_w_m / 2.0,
                        pallet_h_m / 2.0 + 0.005,
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
                "geometry": (
                    str(config.EURO_PALLET_MODEL_SDF)
                    if config.USE_EURO_PALLET_MODEL
                    else "solid_cuboid_fallback"
                ),
            }
        )

        for box_index, placement in enumerate(pallet.placements, start=1):
            sx = mm_to_m(placement.length)
            sy = mm_to_m(placement.width)
            sz = mm_to_m(placement.height)

            px = x_offset_m + mm_to_m(placement.x) + sx / 2.0
            py = mm_to_m(placement.y) + sy / 2.0
            pz = (
                pallet_spawn_clearance_m
                + pallet_h_m
                + mm_to_m(placement.z + settle_lift_mm)
                + sz / 2.0
            )

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

    if config.USE_FORKLIFT_MODEL:
        spawn_pose = forklift_spawn_pose()
        models.append(
            dynamic_forklift_model_xml(
                model_name=config.FORKLIFT_MODEL_NAME,
                pose_xyz_rpy_m=spawn_pose,
                model_sdf_path=config.FORKLIFT_MODEL_SDF,
                restitution=restitution,
                contact_kp=contact_kp,
                contact_kd=contact_kd,
            )
        )
        manifest.append(
            {
                "type": "forklift",
                "model_name": config.FORKLIFT_MODEL_NAME,
                "geometry": str(config.FORKLIFT_MODEL_SDF),
                "world_pose_m_rad": list(spawn_pose),
                "body_size_m": [
                    config.FORKLIFT_BODY_LENGTH_M,
                    config.FORKLIFT_BODY_WIDTH_M,
                    config.FORKLIFT_BODY_HEIGHT_M,
                ],
                "fork_size_m": [
                    config.FORKLIFT_FORK_LENGTH_M,
                    config.FORKLIFT_FORK_WIDTH_M,
                    config.FORKLIFT_FORK_THICKNESS_M,
                ],
                "max_speed_m_s": config.FORKLIFT_MAX_SPEED_M_S,
                "max_acceleration_m_s2": config.FORKLIFT_MAX_ACCEL_M_S2,
                "spawn_distance_reference": "fork_tips_to_first_pallet_open_face",
                "spawn_distance_m": config.FORKLIFT_SPAWN_DISTANCE_M,
                "gui_plugin": config.FORKLIFT_GUI_PLUGIN_FILENAME,
                "controls": {
                    "up": "forward",
                    "down": "reverse",
                    "left": "counterclockwise_turn",
                    "right": "clockwise_turn",
                    "shift_up": "forks_up",
                    "shift_down": "forks_down",
                    "space": "stop",
                },
            }
        )

    return "".join(models), manifest


def forklift_gui_xml(*, start_paused: bool) -> str:
    """Gazebo GUI configuration including the modifier-aware teleop plugin.

    Gazebo Harmonic treats an SDF <gui> block as the complete GUI config, so
    the normal scene and control plugins are included explicitly rather than
    relying on a user-specific default file.
    """
    if not config.USE_FORKLIFT_MODEL:
        return ""

    keys = config.FORKLIFT_KEY_BINDINGS
    paused = "true" if start_paused else "false"
    return f"""
    <gui fullscreen="0">
      <plugin filename="MinimalScene" name="3D View">
        <gz-gui>
          <title>3D View</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property>
        </gz-gui>
        <engine>ogre2</engine>
        <scene>scene</scene>
        <ambient_light>0.4 0.4 0.4</ambient_light>
        <background_color>0.8 0.8 0.8</background_color>
        <camera_pose>-6 -8 6 0 0.45 0.75</camera_pose>
      </plugin>
      <plugin filename="GzSceneManager" name="Scene Manager" />
      <plugin filename="InteractiveViewControl" name="Interactive view control" />
      <plugin filename="CameraTracking" name="Camera Tracking" />
      <plugin filename="MarkerManager" name="Marker manager" />
      <plugin filename="SelectEntities" name="Select entities" />
      <plugin filename="EntityContextMenuPlugin" name="Entity context menu" />
      <plugin filename="VisualizationCapabilities" name="Visualization capabilities" />
      <plugin filename="WorldControl" name="World control">
        <gz-gui>
          <title>World control</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property>
          <property type="double" key="height">72</property>
          <property type="double" key="width">121</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View">
            <line own="left" target="left" />
            <line own="bottom" target="bottom" />
          </anchors>
        </gz-gui>
        <play_pause>true</play_pause>
        <step>true</step>
        <start_paused>{paused}</start_paused>
      </plugin>
      <plugin filename="WorldStats" name="World stats">
        <gz-gui>
          <title>World stats</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property>
          <property type="double" key="height">110</property>
          <property type="double" key="width">290</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View">
            <line own="right" target="right" />
            <line own="bottom" target="bottom" />
          </anchors>
        </gz-gui>
        <sim_time>true</sim_time>
        <real_time>true</real_time>
        <real_time_factor>true</real_time_factor>
        <iterations>true</iterations>
      </plugin>
      <plugin filename="{escape(str(config.FORKLIFT_GUI_PLUGIN_FILENAME))}"
              name="Forklift teleoperation">
        <gz-gui>
          <title>Forklift controls</title>
          <property type="double" key="width">330</property>
          <property type="double" key="height">230</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View">
            <line own="right" target="right" />
            <line own="top" target="top" />
          </anchors>
        </gz-gui>
        <drive_topic>{escape(str(config.FORKLIFT_CMD_VEL_TOPIC))}</drive_topic>
        <left_fork_topic>{escape(str(config.FORKLIFT_LEFT_FORK_TOPIC))}</left_fork_topic>
        <right_fork_topic>{escape(str(config.FORKLIFT_RIGHT_FORK_TOPIC))}</right_fork_topic>
        <world_stats_topic>{escape(str(config.FORKLIFT_WORLD_STATS_TOPIC))}</world_stats_topic>
        <max_linear_speed>{fmt(config.FORKLIFT_MAX_SPEED_M_S)}</max_linear_speed>
        <max_linear_acceleration>{fmt(config.FORKLIFT_MAX_ACCEL_M_S2)}</max_linear_acceleration>
        <max_angular_speed>{fmt(config.FORKLIFT_MAX_ANGULAR_SPEED_RAD_S)}</max_angular_speed>
        <max_angular_acceleration>{fmt(config.FORKLIFT_MAX_ANGULAR_ACCEL_RAD_S2)}</max_angular_acceleration>
        <fork_speed>{fmt(config.FORKLIFT_FORK_SPEED_M_S)}</fork_speed>
        <update_rate_hz>{fmt(config.FORKLIFT_TELEOP_UPDATE_RATE_HZ)}</update_rate_hz>
        <forward_key>{int(keys['forward'])}</forward_key>
        <reverse_key>{int(keys['reverse'])}</reverse_key>
        <left_key>{int(keys['turn_left'])}</left_key>
        <right_key>{int(keys['turn_right'])}</right_key>
        <lift_key>{int(keys['lift'])}</lift_key>
        <lower_key>{int(keys['lower'])}</lower_key>
      </plugin>
    </gui>"""


def create_world_sdf(
    pallets: Sequence[Pallet],
    *,
    pallet_gap_mm: float = config.GAZEBO_PALLET_GAP_MM,
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

    gui_xml = forklift_gui_xml(start_paused=start_paused)

    sdf = f"""<?xml version=\"1.0\" ?>
<sdf version=\"1.10\">
  <world name=\"pallet_towers\">
    <gravity>0 0 -9.81</gravity>

    <physics name=\"pallet_physics\" type=\"ignored\">
      <max_step_size>{fmt(config.GAZEBO_MAX_STEP_SIZE)}</max_step_size>
      <real_time_factor>{fmt(config.GAZEBO_REAL_TIME_FACTOR)}</real_time_factor>
      <real_time_update_rate>{int(config.GAZEBO_REAL_TIME_UPDATE_RATE)}</real_time_update_rate>
      <max_contacts>{int(config.GAZEBO_MAX_CONTACTS)}</max_contacts>
    </physics>

    <plugin filename=\"gz-sim-physics-system\" name=\"gz::sim::systems::Physics\" />
    <plugin filename=\"gz-sim-user-commands-system\" name=\"gz::sim::systems::UserCommands\" />
    <plugin filename=\"gz-sim-scene-broadcaster-system\" name=\"gz::sim::systems::SceneBroadcaster\" />

{gui_xml}

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

    # Keep generated worlds diff-friendly: interpolated model blocks may carry
    # indentation-only blank lines, which are semantically irrelevant XML.
    sdf = "\n".join(line.rstrip() for line in sdf.splitlines()) + "\n"
    return sdf, manifest


def write_layout_sdf(
    pallets: Sequence[Pallet],
    output_path: str | Path | None = None,
    *,
    output_dir: str | Path = "gazebo_runs",
    run_name: str | None = "latest",
    overwrite: bool = True,
    pallet_gap_mm: float = config.GAZEBO_PALLET_GAP_MM,
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
    parser.add_argument(
        "--pallet-gap-mm", type=float, default=config.GAZEBO_PALLET_GAP_MM
    )
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