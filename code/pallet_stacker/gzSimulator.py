"""Generate and optionally launch a Gazebo Harmonic pallet-loading world.

The module consumes the same fully populated :class:`Pallet` objects as the
Plotly visualizer. It does not modify packing results. Instead, it converts each
pallet-local box placement into an SDFormat world-space pose, includes the local
EUR pallet and forklift models, and writes one self-contained world file that
references those models through ``model://`` URIs.

Coordinate mapping
------------------
* Packing coordinates use millimetres and place ``z=0`` on the pallet loading
  surface.
* Gazebo uses metres and the EUR pallet model places ``z=0`` on the floor-contact
  plane.
* Therefore every box centre receives the pallet base height plus half the
  oriented box height.

The simulation has no end-time condition. It runs until the user closes Gazebo.
"""

from __future__ import annotations

import hashlib
import math
import os
import re
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence
from xml.etree import ElementTree
from xml.sax.saxutils import escape

from . import settings
from .box import Box, Dimensions3D, Point3D
from .pallet import Pallet

MM_TO_M = 0.001


class GazeboSimulationError(RuntimeError):
    """Raised when a Gazebo world cannot be generated or launched."""


@dataclass(frozen=True, slots=True)
class GazeboSimulationResult:
    """Files and process information produced by one Gazebo stage."""

    world_path: Path
    launched: bool
    process_id: int | None = None
    return_code: int | None = None


@dataclass(frozen=True, slots=True)
class _PalletWorldPose:
    pallet: Pallet
    center_x_m: float
    center_y_m: float
    base_z_m: float

    @property
    def min_x_m(self) -> float:
        return self.center_x_m - self.pallet.length_mm * MM_TO_M / 2.0

    @property
    def max_x_m(self) -> float:
        return self.center_x_m + self.pallet.length_mm * MM_TO_M / 2.0

    @property
    def min_y_m(self) -> float:
        return self.center_y_m - self.pallet.width_mm * MM_TO_M / 2.0

    @property
    def max_y_m(self) -> float:
        return self.center_y_m + self.pallet.width_mm * MM_TO_M / 2.0


@dataclass(frozen=True, slots=True)
class _SceneBounds:
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    max_z: float


@dataclass(frozen=True, slots=True)
class _BoxInertia:
    ixx: float
    iyy: float
    izz: float


def create_simulation(
    pallets: Sequence[Pallet],
    *,
    output_path: str | Path = settings.GAZEBO_WORLD_FILE,
    launch: bool = settings.GAZEBO_LAUNCH_SIMULATION,
    wait: bool = settings.GAZEBO_WAIT_FOR_SIMULATION_EXIT,
) -> GazeboSimulationResult:
    """Write the world and optionally launch Gazebo.

    ``wait=False`` starts Gazebo as a child process and returns immediately.
    ``wait=True`` blocks until Gazebo exits and records its return code.
    """

    world_path = write_world_sdf(pallets, output_path=output_path)
    if not launch:
        return GazeboSimulationResult(world_path=world_path, launched=False)

    process = launch_gazebo(world_path, wait=wait)
    return GazeboSimulationResult(
        world_path=world_path,
        launched=True,
        process_id=process.pid,
        return_code=process.returncode if wait else None,
    )


def write_world_sdf(
    pallets: Sequence[Pallet],
    output_path: str | Path = settings.GAZEBO_WORLD_FILE,
) -> Path:
    """Generate and write the Gazebo world for the supplied pallet layout."""

    pallet_sequence = tuple(pallets)
    _validate_settings()
    _prepare_local_assets()
    _validate_pallet_instances(pallet_sequence)

    world_text = build_world_sdf(pallet_sequence)
    output = Path(output_path).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(world_text, encoding="utf-8")

    try:
        ElementTree.parse(output)
    except ElementTree.ParseError as exc:
        raise GazeboSimulationError(
            f"Generated Gazebo world is not valid XML: {exc}"
        ) from exc

    return output


def build_world_sdf(pallets: Sequence[Pallet]) -> str:
    """Return the complete SDFormat world as text without writing it."""

    pallet_poses = _arrange_pallets(pallets)
    forklift_pose = _forklift_pose(pallet_poses)
    bounds = _calculate_scene_bounds(pallet_poses, forklift_pose)
    ground_center_x, ground_center_y, ground_size_x, ground_size_y = (
        _ground_geometry(bounds)
    )
    camera_pose = _camera_pose(bounds)

    sections: list[str] = [
        '<?xml version="1.0"?>',
        '<sdf version="1.10">',
        f'  <world name="{escape(settings.GAZEBO_WORLD_NAME)}">',
        _world_physics_xml(),
        _world_system_plugins_xml(),
        _scene_xml(),
        _gui_xml(camera_pose),
        _light_xml(),
        _ground_xml(
            center_x=ground_center_x,
            center_y=ground_center_y,
            size_x=ground_size_x,
            size_y=ground_size_y,
        ),
    ]

    for index, pallet_pose in enumerate(pallet_poses, start=1):
        sections.append(_pallet_include_xml(pallet_pose, index))
        for box_index, box in enumerate(pallet_pose.pallet.boxes, start=1):
            sections.append(_box_model_xml(pallet_pose, box, box_index))

    sections.append(_forklift_include_xml(forklift_pose))
    sections.extend(("  </world>", "</sdf>", ""))
    return "\n".join(sections)


def launch_gazebo(
    world_path: str | Path,
    *,
    wait: bool = settings.GAZEBO_WAIT_FOR_SIMULATION_EXIT,
) -> subprocess.Popen[bytes]:
    """Launch Gazebo Harmonic with the generated world."""

    world = Path(world_path).expanduser().resolve()
    if not world.is_file():
        raise GazeboSimulationError(f"Gazebo world does not exist: {world}")

    executable = shutil.which(settings.GAZEBO_EXECUTABLE)
    if executable is None:
        raise GazeboSimulationError(
            f"Could not find Gazebo executable {settings.GAZEBO_EXECUTABLE!r} "
            "on PATH. Expected a working 'gz sim' installation."
        )

    command = [
        executable,
        "sim",
        "-v",
        str(settings.GAZEBO_VERBOSITY),
    ]
    if not settings.GAZEBO_START_PAUSED:
        command.append("-r")
    command.append(str(world))

    environment = os.environ.copy()
    model_path = str(settings.GAZEBO_MODELS_DIRECTORY.resolve())
    existing_resource_path = environment.get("GZ_SIM_RESOURCE_PATH", "")
    environment["GZ_SIM_RESOURCE_PATH"] = (
        model_path
        if not existing_resource_path
        else os.pathsep.join((model_path, existing_resource_path))
    )

    try:
        process = subprocess.Popen(command, env=environment)
    except OSError as exc:
        raise GazeboSimulationError(f"Could not launch Gazebo: {exc}") from exc

    if wait:
        process.wait()
        if process.returncode not in (0, None):
            raise GazeboSimulationError(
                f"Gazebo exited with return code {process.returncode}."
            )
    return process


def _prepare_local_assets() -> None:
    if settings.GAZEBO_REGENERATE_FORKLIFT_ASSETS:
        try:
            from .generateForkliftModel import write_forklift_assets

            write_forklift_assets()
        except (OSError, ValueError, ElementTree.ParseError) as exc:
            raise GazeboSimulationError(
                f"Could not regenerate forklift assets: {exc}"
            ) from exc

    if not settings.GAZEBO_VALIDATE_LOCAL_MODELS:
        return

    required_files = (
        settings.PALLET_GAZEBO_MODEL_DIRECTORY / "model.sdf",
        settings.PALLET_GAZEBO_MODEL_DIRECTORY / "model.config",
        settings.FORKLIFT_MODEL_DIRECTORY / "model.sdf",
        settings.FORKLIFT_MODEL_DIRECTORY / "model.config",
    )
    missing = [path for path in required_files if not path.is_file()]
    if missing:
        formatted = "\n".join(f"  - {path}" for path in missing)
        raise GazeboSimulationError(
            "Required Gazebo model files are missing:\n" + formatted
        )

    _validate_pallet_model_envelope(
        settings.PALLET_GAZEBO_MODEL_DIRECTORY / "model.sdf"
    )


def _validate_pallet_model_envelope(model_file: Path) -> None:
    """Check the primitive collision envelope of the local pallet model."""

    try:
        root = ElementTree.parse(model_file).getroot()
    except (OSError, ElementTree.ParseError) as exc:
        raise GazeboSimulationError(
            f"Could not parse pallet model {model_file}: {exc}"
        ) from exc

    bounds: list[tuple[float, float, float, float, float, float]] = []
    for link in root.findall(".//link"):
        link_pose = _parse_pose(link.findtext("pose"))
        _require_zero_rotation(link_pose[3:], "pallet link")
        for collision in link.findall("collision"):
            box_size_text = collision.findtext("geometry/box/size")
            if box_size_text is None:
                continue
            size = _parse_vector(box_size_text, 3, "pallet collision box size")
            collision_pose = _parse_pose(collision.findtext("pose"))
            _require_zero_rotation(collision_pose[3:], "pallet collision")
            center_x = link_pose[0] + collision_pose[0]
            center_y = link_pose[1] + collision_pose[1]
            center_z = link_pose[2] + collision_pose[2]
            bounds.append(
                (
                    center_x - size[0] / 2.0,
                    center_x + size[0] / 2.0,
                    center_y - size[1] / 2.0,
                    center_y + size[1] / 2.0,
                    center_z - size[2] / 2.0,
                    center_z + size[2] / 2.0,
                )
            )

    if not bounds:
        raise GazeboSimulationError(
            f"Pallet model {model_file} contains no primitive box collisions."
        )

    actual = (
        min(item[0] for item in bounds),
        max(item[1] for item in bounds),
        min(item[2] for item in bounds),
        max(item[3] for item in bounds),
        min(item[4] for item in bounds),
        max(item[5] for item in bounds),
    )
    expected = (
        -settings.PALLET_LENGTH_MM * MM_TO_M / 2.0,
        settings.PALLET_LENGTH_MM * MM_TO_M / 2.0,
        -settings.PALLET_WIDTH_MM * MM_TO_M / 2.0,
        settings.PALLET_WIDTH_MM * MM_TO_M / 2.0,
        0.0,
        settings.PALLET_BASE_HEIGHT_MM * MM_TO_M,
    )

    tolerance_m = 1e-5
    if any(abs(a - e) > tolerance_m for a, e in zip(actual, expected)):
        raise GazeboSimulationError(
            "The local pallet model collision envelope does not match settings.py. "
            f"Actual bounds are {actual}; expected {expected}. Regenerate or edit "
            "the pallet model before simulation."
        )


def _validate_pallet_instances(pallets: Sequence[Pallet]) -> None:
    for pallet in pallets:
        configured = (
            settings.PALLET_LENGTH_MM,
            settings.PALLET_WIDTH_MM,
            settings.PALLET_BASE_HEIGHT_MM,
        )
        actual = (pallet.length_mm, pallet.width_mm, pallet.base_height_mm)
        if any(
            abs(current - expected) > settings.PLACEMENT_TOLERANCE_MM
            for current, expected in zip(actual, configured)
        ):
            raise GazeboSimulationError(
                f"Pallet {pallet.pallet_id!r} dimensions {actual} do not match the "
                f"configured Gazebo pallet model dimensions {configured}."
            )

        for box in pallet.boxes:
            if not box.is_placed:
                raise GazeboSimulationError(
                    f"Box {box.box_id!r} on pallet {pallet.pallet_id!r} has no placement."
                )


def _arrange_pallets(pallets: Sequence[Pallet]) -> tuple[_PalletWorldPose, ...]:
    poses: list[_PalletWorldPose] = []
    center_x = settings.GAZEBO_FIRST_PALLET_X_M
    for pallet in pallets:
        poses.append(
            _PalletWorldPose(
                pallet=pallet,
                center_x_m=center_x,
                center_y_m=settings.GAZEBO_FIRST_PALLET_Y_M,
                base_z_m=settings.GAZEBO_PALLET_SPAWN_CLEARANCE_M,
            )
        )
        center_x += (
            pallet.length_mm * MM_TO_M
            + settings.GAZEBO_PALLET_GAP_M
        )
    return tuple(poses)


def _forklift_pose(
    pallet_poses: Sequence[_PalletWorldPose],
) -> tuple[float, float, float, float, float, float]:
    if pallet_poses:
        first = pallet_poses[0]
        nearest_pallet_edge_x = first.min_x_m
        y = first.center_y_m
    else:
        nearest_pallet_edge_x = settings.GAZEBO_FIRST_PALLET_X_M
        y = settings.GAZEBO_FIRST_PALLET_Y_M

    # Forklift model origin is the body front face. Its forks extend along +X.
    # Positioning the fork tips on the negative-X side aligns them with the
    # pallet's longitudinal channels and points them toward the pallet.
    body_front_x = (
        nearest_pallet_edge_x
        - settings.GAZEBO_FORKLIFT_FORK_TIP_CLEARANCE_M
        - settings.FORKLIFT_FORK_LENGTH_M
    )
    return (
        body_front_x,
        y,
        settings.GAZEBO_FORKLIFT_SPAWN_CLEARANCE_M,
        0.0,
        0.0,
        0.0,
    )


def _calculate_scene_bounds(
    pallet_poses: Sequence[_PalletWorldPose],
    forklift_pose: tuple[float, float, float, float, float, float],
) -> _SceneBounds:
    min_x_values: list[float] = []
    max_x_values: list[float] = []
    min_y_values: list[float] = []
    max_y_values: list[float] = []
    max_z_values: list[float] = [settings.FORKLIFT_BODY_HEIGHT_M]

    for pose in pallet_poses:
        min_x_values.append(pose.min_x_m)
        max_x_values.append(pose.max_x_m)
        min_y_values.append(pose.min_y_m)
        max_y_values.append(pose.max_y_m)
        max_z_values.append(
            pose.base_z_m
            + pose.pallet.base_height_mm * MM_TO_M
            + pose.pallet.load_height_mm * MM_TO_M
        )

    forklift_body_front_x = forklift_pose[0]
    forklift_rear_x = forklift_body_front_x - settings.FORKLIFT_BODY_LENGTH_M
    forklift_tip_x = forklift_body_front_x + settings.FORKLIFT_FORK_LENGTH_M
    half_width = settings.FORKLIFT_BODY_WIDTH_M / 2.0
    min_x_values.append(forklift_rear_x)
    max_x_values.append(forklift_tip_x)
    min_y_values.append(forklift_pose[1] - half_width)
    max_y_values.append(forklift_pose[1] + half_width)

    return _SceneBounds(
        min_x=min(min_x_values),
        max_x=max(max_x_values),
        min_y=min(min_y_values),
        max_y=max(max_y_values),
        max_z=max(max_z_values),
    )


def _ground_geometry(
    bounds: _SceneBounds,
) -> tuple[float, float, float, float]:
    content_width_x = bounds.max_x - bounds.min_x
    content_width_y = bounds.max_y - bounds.min_y
    size_x = max(
        settings.GAZEBO_GROUND_MIN_SIZE_M,
        content_width_x + 2.0 * settings.GAZEBO_GROUND_MARGIN_M,
    )
    size_y = max(
        settings.GAZEBO_GROUND_MIN_SIZE_M,
        content_width_y + 2.0 * settings.GAZEBO_GROUND_MARGIN_M,
    )
    return (
        (bounds.min_x + bounds.max_x) / 2.0,
        (bounds.min_y + bounds.max_y) / 2.0,
        size_x,
        size_y,
    )


def _camera_pose(
    bounds: _SceneBounds,
) -> tuple[float, float, float, float, float, float]:
    if settings.GAZEBO_CAMERA_POSE is not None:
        return settings.GAZEBO_CAMERA_POSE

    span_x = max(1.0, bounds.max_x - bounds.min_x)
    span_y = max(1.0, bounds.max_y - bounds.min_y)
    center_x = (bounds.min_x + bounds.max_x) / 2.0
    center_y = (bounds.min_y + bounds.max_y) / 2.0
    horizontal_span = max(span_x, span_y)
    return (
        center_x - max(5.0, horizontal_span * 0.75),
        center_y - max(4.0, horizontal_span * 0.55),
        max(4.5, bounds.max_z + horizontal_span * 0.45),
        0.0,
        0.52,
        0.72,
    )


def _world_physics_xml() -> str:
    gravity = _format_values(settings.GAZEBO_GRAVITY_MPS2)
    return (
        f'    <gravity>{gravity}</gravity>\n'
        '    <physics name="pallet_stacker_physics" type="ignored">\n'
        f'      <max_step_size>{_fmt(settings.GAZEBO_PHYSICS_MAX_STEP_SIZE_S)}</max_step_size>\n'
        f'      <real_time_factor>{_fmt(settings.GAZEBO_REAL_TIME_FACTOR)}</real_time_factor>\n'
        '    </physics>'
    )


def _world_system_plugins_xml() -> str:
    return "\n".join(
        (
            '    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>',
            '    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>',
            '    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>',
        )
    )


def _scene_xml() -> str:
    return (
        '    <scene>\n'
        f'      <ambient>{_format_values(settings.GAZEBO_AMBIENT_LIGHT_RGBA)}</ambient>\n'
        f'      <background>{_format_values(settings.GAZEBO_BACKGROUND_RGBA)}</background>\n'
        f'      <shadows>{_bool_text(not settings.GAZEBO_DISABLE_SHADOWS)}</shadows>\n'
        f'      <grid>{_bool_text(settings.GAZEBO_SHOW_GRID)}</grid>\n'
        '    </scene>'
    )


def _gui_xml(
    camera_pose: tuple[float, float, float, float, float, float],
) -> str:
    world = escape(settings.GAZEBO_WORLD_NAME)
    start_paused = _bool_text(settings.GAZEBO_START_PAUSED)
    return f'''    <gui fullscreen="0">
      <plugin filename="MinimalScene" name="3D View">
        <gz-gui>
          <title>3D View</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property>
        </gz-gui>
        <engine>{escape(settings.GAZEBO_RENDER_ENGINE)}</engine>
        <scene>scene</scene>
        <ambient_light>{_format_values(settings.GAZEBO_AMBIENT_LIGHT_RGBA)}</ambient_light>
        <background_color>{_format_values(settings.GAZEBO_BACKGROUND_RGBA)}</background_color>
        <camera_pose>{_format_values(camera_pose)}</camera_pose>
        <camera_clip>
          <near>{_fmt(settings.GAZEBO_CAMERA_NEAR_CLIP_M)}</near>
          <far>{_fmt(settings.GAZEBO_CAMERA_FAR_CLIP_M)}</far>
        </camera_clip>
      </plugin>
      <plugin filename="GzSceneManager" name="Scene Manager"/>
      <plugin filename="WorldControl" name="World control">
        <play_pause>true</play_pause>
        <step>true</step>
        <start_paused>{start_paused}</start_paused>
        <service>/world/{world}/control</service>
        <stats_topic>/world/{world}/stats</stats_topic>
      </plugin>
      <plugin filename="WorldStats" name="World stats">
        <sim_time>true</sim_time>
        <real_time>true</real_time>
        <real_time_factor>true</real_time_factor>
        <iterations>true</iterations>
        <topic>/world/{world}/stats</topic>
      </plugin>
      <plugin filename="EntityTree" name="Entity tree"/>
    </gui>'''


def _light_xml() -> str:
    cast_shadows = _bool_text(not settings.GAZEBO_DISABLE_SHADOWS)
    return f'''    <light type="directional" name="sun">
      <pose>0 0 10 0 0 0</pose>
      <cast_shadows>{cast_shadows}</cast_shadows>
      <direction>-0.5 0.2 -1</direction>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.15 0.15 0.15 1</specular>
    </light>'''


def _ground_xml(
    *,
    center_x: float,
    center_y: float,
    size_x: float,
    size_y: float,
) -> str:
    color = _format_values(settings.GAZEBO_GROUND_RGBA)
    return f'''    <model name="ground_plane">
      <static>true</static>
      <pose>{_fmt(center_x)} {_fmt(center_y)} 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>{_fmt(size_x)} {_fmt(size_y)}</size></plane></geometry>
          <surface>
            <friction><ode><mu>{_fmt(settings.GAZEBO_GROUND_FRICTION)}</mu><mu2>{_fmt(settings.GAZEBO_GROUND_FRICTION)}</mu2></ode></friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>{_fmt(size_x)} {_fmt(size_y)}</size></plane></geometry>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
        </visual>
      </link>
    </model>'''


def _pallet_include_xml(pose: _PalletWorldPose, index: int) -> str:
    model_name = _safe_name(f"pallet_{index}_{pose.pallet.pallet_id}")
    return f'''    <include>
      <uri>model://{escape(settings.PALLET_GAZEBO_MODEL_NAME)}</uri>
      <name>{model_name}</name>
      <pose>{_fmt(pose.center_x_m)} {_fmt(pose.center_y_m)} {_fmt(pose.base_z_m)} 0 0 0</pose>
    </include>'''


def _forklift_include_xml(
    pose: tuple[float, float, float, float, float, float],
) -> str:
    return f'''    <include>
      <uri>model://{escape(settings.FORKLIFT_MODEL_NAME)}</uri>
      <name>{_safe_name(settings.FORKLIFT_MODEL_NAME)}</name>
      <pose>{_format_values(pose)}</pose>
    </include>'''


def _box_model_xml(
    pallet_pose: _PalletWorldPose,
    box: Box,
    box_index: int,
) -> str:
    position, dimensions = _box_geometry(box)
    dimensions_m = Dimensions3D(
        dimensions.x * MM_TO_M,
        dimensions.y * MM_TO_M,
        dimensions.z * MM_TO_M,
    )

    min_corner_x = pallet_pose.min_x_m + position.x * MM_TO_M
    min_corner_y = pallet_pose.min_y_m + position.y * MM_TO_M
    loading_surface_z = (
        pallet_pose.base_z_m + pallet_pose.pallet.base_height_mm * MM_TO_M
    )
    center_x = min_corner_x + dimensions_m.x / 2.0
    center_y = min_corner_y + dimensions_m.y / 2.0
    center_z = (
        loading_surface_z
        + position.z * MM_TO_M
        + dimensions_m.z / 2.0
        + settings.GAZEBO_BOX_SPAWN_CLEARANCE_M
    )

    mass = max(box.weight_kg, settings.GAZEBO_BOX_MIN_MASS_KG)
    inertia = _box_inertia(mass, dimensions_m)
    red, green, blue = _stable_color(box.sku or box.box_id)
    model_name = _safe_name(
        f"box_{pallet_pose.pallet.pallet_id}_{box_index}_{box.box_id}"
    )
    auto_disable = _bool_text(settings.GAZEBO_BOX_AUTO_DISABLE)

    return f'''    <model name="{model_name}">
      <pose>{_fmt(center_x)} {_fmt(center_y)} {_fmt(center_z)} 0 0 0</pose>
      <static>false</static>
      <self_collide>false</self_collide>
      <allow_auto_disable>{auto_disable}</allow_auto_disable>
      <link name="body">
        <gravity>true</gravity>
        <inertial>
          <mass>{_fmt(mass)}</mass>
          <inertia>
            <ixx>{_fmt(inertia.ixx)}</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>{_fmt(inertia.iyy)}</iyy><iyz>0</iyz><izz>{_fmt(inertia.izz)}</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry><box><size>{_fmt(dimensions_m.x)} {_fmt(dimensions_m.y)} {_fmt(dimensions_m.z)}</size></box></geometry>
          {_box_surface_xml()}
        </collision>
        <visual name="visual">
          <geometry><box><size>{_fmt(dimensions_m.x)} {_fmt(dimensions_m.y)} {_fmt(dimensions_m.z)}</size></box></geometry>
          <material>
            <ambient>{_fmt(red * 0.75)} {_fmt(green * 0.75)} {_fmt(blue * 0.75)} 1</ambient>
            <diffuse>{_fmt(red)} {_fmt(green)} {_fmt(blue)} 1</diffuse>
            <specular>0.08 0.08 0.08 1</specular>
          </material>
        </visual>
      </link>
    </model>'''


def _box_surface_xml() -> str:
    friction = _fmt(settings.GAZEBO_BOX_FRICTION)
    return f'''<surface>
            <friction>
              <ode><mu>{friction}</mu><mu2>{friction}</mu2></ode>
              <bullet><friction>{friction}</friction><friction2>{friction}</friction2></bullet>
            </friction>
            <bounce>
              <restitution_coefficient>{_fmt(settings.GAZEBO_BOX_RESTITUTION)}</restitution_coefficient>
              <threshold>100000</threshold>
            </bounce>
            <contact>
              <ode>
                <kp>{_fmt(settings.GAZEBO_BOX_CONTACT_STIFFNESS)}</kp>
                <kd>{_fmt(settings.GAZEBO_BOX_CONTACT_DAMPING)}</kd>
                <max_vel>{_fmt(settings.GAZEBO_CONTACT_MAX_CORRECTING_VELOCITY_MPS)}</max_vel>
                <min_depth>{_fmt(settings.GAZEBO_CONTACT_MIN_DEPTH_M)}</min_depth>
              </ode>
            </contact>
          </surface>'''


def _box_geometry(box: Box) -> tuple[Point3D, Dimensions3D]:
    position = box.position
    dimensions = box.placed_dimensions
    if position is None or dimensions is None:
        raise GazeboSimulationError(f"Box {box.box_id!r} is not placed.")
    return position, dimensions


def _box_inertia(mass: float, dimensions: Dimensions3D) -> _BoxInertia:
    return _BoxInertia(
        ixx=mass * (dimensions.y**2 + dimensions.z**2) / 12.0,
        iyy=mass * (dimensions.x**2 + dimensions.z**2) / 12.0,
        izz=mass * (dimensions.x**2 + dimensions.y**2) / 12.0,
    )


def _stable_color(text: str) -> tuple[float, float, float]:
    digest = hashlib.sha256(text.encode("utf-8")).digest()
    return tuple((70 + value % 150) / 255.0 for value in digest[:3])  # type: ignore[return-value]


def _safe_name(value: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_]+", "_", value).strip("_")
    if not cleaned:
        cleaned = "entity"
    if cleaned[0].isdigit():
        cleaned = "entity_" + cleaned
    return cleaned


def _parse_pose(text: str | None) -> tuple[float, float, float, float, float, float]:
    if text is None or not text.strip():
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    values = _parse_vector(text, 6, "pose")
    return tuple(values)  # type: ignore[return-value]


def _parse_vector(text: str, count: int, description: str) -> tuple[float, ...]:
    parts = text.split()
    if len(parts) != count:
        raise GazeboSimulationError(
            f"Expected {count} values for {description}, received {text!r}."
        )
    try:
        return tuple(float(part) for part in parts)
    except ValueError as exc:
        raise GazeboSimulationError(
            f"Non-numeric value in {description}: {text!r}."
        ) from exc


def _require_zero_rotation(rotation: Iterable[float], description: str) -> None:
    if any(abs(value) > 1e-9 for value in rotation):
        raise GazeboSimulationError(
            f"Cannot validate rotated {description}; expected axis-aligned boxes."
        )


def _validate_settings() -> None:
    positive_values = {
        "GAZEBO_PHYSICS_MAX_STEP_SIZE_S": settings.GAZEBO_PHYSICS_MAX_STEP_SIZE_S,
        "GAZEBO_REAL_TIME_FACTOR": settings.GAZEBO_REAL_TIME_FACTOR,
        "GAZEBO_GROUND_MIN_SIZE_M": settings.GAZEBO_GROUND_MIN_SIZE_M,
        "GAZEBO_BOX_MIN_MASS_KG": settings.GAZEBO_BOX_MIN_MASS_KG,
        "GAZEBO_BOX_CONTACT_STIFFNESS": settings.GAZEBO_BOX_CONTACT_STIFFNESS,
        "GAZEBO_CAMERA_NEAR_CLIP_M": settings.GAZEBO_CAMERA_NEAR_CLIP_M,
        "GAZEBO_CAMERA_FAR_CLIP_M": settings.GAZEBO_CAMERA_FAR_CLIP_M,
    }
    for name, value in positive_values.items():
        if not math.isfinite(value) or value <= 0.0:
            raise GazeboSimulationError(f"settings.{name} must be positive and finite.")

    non_negative_values = {
        "GAZEBO_GROUND_MARGIN_M": settings.GAZEBO_GROUND_MARGIN_M,
        "GAZEBO_PALLET_GAP_M": settings.GAZEBO_PALLET_GAP_M,
        "GAZEBO_BOX_CONTACT_DAMPING": settings.GAZEBO_BOX_CONTACT_DAMPING,
        "GAZEBO_CONTACT_MAX_CORRECTING_VELOCITY_MPS": settings.GAZEBO_CONTACT_MAX_CORRECTING_VELOCITY_MPS,
        "GAZEBO_PALLET_SPAWN_CLEARANCE_M": settings.GAZEBO_PALLET_SPAWN_CLEARANCE_M,
        "GAZEBO_BOX_SPAWN_CLEARANCE_M": settings.GAZEBO_BOX_SPAWN_CLEARANCE_M,
        "GAZEBO_FORKLIFT_FORK_TIP_CLEARANCE_M": settings.GAZEBO_FORKLIFT_FORK_TIP_CLEARANCE_M,
        "GAZEBO_FORKLIFT_SPAWN_CLEARANCE_M": settings.GAZEBO_FORKLIFT_SPAWN_CLEARANCE_M,
        "GAZEBO_BOX_RESTITUTION": settings.GAZEBO_BOX_RESTITUTION,
        "GAZEBO_CONTACT_MIN_DEPTH_M": settings.GAZEBO_CONTACT_MIN_DEPTH_M,
    }
    for name, value in non_negative_values.items():
        if not math.isfinite(value) or value < 0.0:
            raise GazeboSimulationError(
                f"settings.{name} must be non-negative and finite."
            )

    non_negative_contact_values = {
        "GAZEBO_BOX_FRICTION": settings.GAZEBO_BOX_FRICTION,
        "GAZEBO_GROUND_FRICTION": settings.GAZEBO_GROUND_FRICTION,
    }
    for name, value in non_negative_contact_values.items():
        if not math.isfinite(value) or value < 0.0:
            raise GazeboSimulationError(f"settings.{name} must be non-negative.")

    if not 0.0 <= settings.GAZEBO_BOX_RESTITUTION <= 1.0:
        raise GazeboSimulationError(
            "settings.GAZEBO_BOX_RESTITUTION must be in the range [0, 1]."
        )

    if len(settings.GAZEBO_GRAVITY_MPS2) != 3 or any(
        not math.isfinite(value) for value in settings.GAZEBO_GRAVITY_MPS2
    ):
        raise GazeboSimulationError(
            "GAZEBO_GRAVITY_MPS2 must contain three finite values."
        )
    for name, value in (
        ("GAZEBO_FIRST_PALLET_X_M", settings.GAZEBO_FIRST_PALLET_X_M),
        ("GAZEBO_FIRST_PALLET_Y_M", settings.GAZEBO_FIRST_PALLET_Y_M),
    ):
        if not math.isfinite(value):
            raise GazeboSimulationError(f"settings.{name} must be finite.")

    if settings.GAZEBO_CAMERA_FAR_CLIP_M <= settings.GAZEBO_CAMERA_NEAR_CLIP_M:
        raise GazeboSimulationError(
            "GAZEBO_CAMERA_FAR_CLIP_M must exceed GAZEBO_CAMERA_NEAR_CLIP_M."
        )
    if settings.GAZEBO_VERBOSITY not in range(0, 5):
        raise GazeboSimulationError("GAZEBO_VERBOSITY must be between 0 and 4.")
    if not settings.GAZEBO_WORLD_NAME.strip():
        raise GazeboSimulationError("GAZEBO_WORLD_NAME must not be empty.")
    if settings.GAZEBO_RENDER_ENGINE not in {"ogre", "ogre2"}:
        raise GazeboSimulationError(
            "GAZEBO_RENDER_ENGINE must be either 'ogre' or 'ogre2'."
        )

    for name, color in (
        ("GAZEBO_AMBIENT_LIGHT_RGBA", settings.GAZEBO_AMBIENT_LIGHT_RGBA),
        ("GAZEBO_BACKGROUND_RGBA", settings.GAZEBO_BACKGROUND_RGBA),
        ("GAZEBO_GROUND_RGBA", settings.GAZEBO_GROUND_RGBA),
    ):
        if len(color) != 4 or any(
            not math.isfinite(value) or not 0.0 <= value <= 1.0
            for value in color
        ):
            raise GazeboSimulationError(
                f"settings.{name} must contain four values in the range [0, 1]."
            )

    if settings.GAZEBO_CAMERA_POSE is not None:
        if len(settings.GAZEBO_CAMERA_POSE) != 6 or any(
            not math.isfinite(value) for value in settings.GAZEBO_CAMERA_POSE
        ):
            raise GazeboSimulationError(
                "GAZEBO_CAMERA_POSE must be None or six finite numbers."
            )


def _fmt(value: float) -> str:
    return f"{float(value):.12g}"


def _format_values(values: Iterable[float]) -> str:
    return " ".join(_fmt(value) for value in values)


def _bool_text(value: bool) -> str:
    return "true" if value else "false"
