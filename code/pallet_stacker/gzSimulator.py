"""Generate a simulation-ready Gazebo Harmonic pallet-loading world.

The module consumes the same populated :class:`Pallet` objects as the Plotly
visualizer. It writes one SDFormat world containing the ground, GUI, pallets,
boxes, forklift, and—when enabled—an integrated Gazebo GUI plugin for direct
keyboard control.

``main.py`` is a generation program only. The normal workflow is::

    python code/main.py
    gz sim gazebo/worlds/pallet_stacker_world.sdf

The generated world does not require ROS 2, ``ros_gz_bridge``, a teleoperation
terminal, ``GZ_SIM_RESOURCE_PATH``, or ``GZ_GUI_PLUGIN_PATH``. Pallet and
forklift model XML are inlined. The custom GUI plugin is compiled under the
project, installed in Gazebo GUI's user-local plugin directory, and referenced
from the world by the stable name ``ForkliftTeleop``.

Coordinate mapping
------------------
Packing coordinates use millimetres and place ``z=0`` on the pallet loading
surface. Gazebo uses metres and the EUR pallet model places ``z=0`` on its floor
contact plane. Every exported box centre therefore receives the physical pallet
height in addition to its pallet-local position.

No end time is written. The simulation runs until the user closes Gazebo.
"""

from __future__ import annotations

import copy
import hashlib
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Sequence
from xml.etree import ElementTree
from xml.sax.saxutils import escape

from . import settings
from .box import Box, Dimensions3D, Point3D
from .generateForkliftGuiPlugin import (
    ForkliftGuiPluginError,
    ForkliftGuiPluginResult,
    prepare_forklift_gui_plugin,
)
from .pallet import Pallet

MM_TO_M = 0.001


class GazeboSimulationError(RuntimeError):
    """Raised when the Gazebo world or its required assets cannot be prepared."""


@dataclass(frozen=True, slots=True)
class GazeboSimulationResult:
    """Files produced by one Gazebo-generation stage."""

    world_path: Path
    manifest_path: Path | None
    gui_plugin_path: Path | None
    gui_plugin_rebuilt: bool
    gui_plugin_installed: bool


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


@dataclass(frozen=True, slots=True)
class _KeyBinding:
    code: int
    label: str


def create_simulation(
    pallets: Sequence[Pallet],
    *,
    output_path: str | Path = settings.GAZEBO_WORLD_FILE,
    manifest_path: str | Path = settings.GAZEBO_MANIFEST_FILE,
) -> GazeboSimulationResult:
    """Prepare all assets and write the world without launching Gazebo.

    The generated world's ``<gui>`` block loads the installed forklift control
    plugin. That plugin publishes directly through Gazebo Transport and
    optionally unpauses the world, so a later plain ``gz sim <world>`` command
    is sufficient.
    """

    pallet_sequence = tuple(pallets)
    _validate_settings()
    plugin_result = _prepare_local_assets()
    _validate_pallet_instances(pallet_sequence)

    plugin_filename = (
        None if plugin_result is None else plugin_result.gazebo_filename
    )
    world_text, manifest = build_world_sdf(
        pallet_sequence,
        gui_plugin_filename=plugin_filename,
    )

    output = Path(output_path).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    _write_validated_xml(output, world_text)

    written_manifest: Path | None = None
    if settings.GAZEBO_WRITE_MANIFEST:
        written_manifest = Path(manifest_path).expanduser().resolve()
        written_manifest.parent.mkdir(parents=True, exist_ok=True)
        manifest.update(
            {
                "world_file": str(output),
                "gui_plugin_filename": (
                    None
                    if plugin_result is None
                    else plugin_result.gazebo_filename
                ),
                "gui_plugin_library": (
                    None
                    if plugin_result is None
                    else str(plugin_result.library_path)
                ),
                "gui_plugin_build_library": (
                    None
                    if plugin_result is None
                    else str(plugin_result.build_library_path)
                ),
                "models_inlined": settings.GAZEBO_INLINE_LOCAL_MODELS,
                "requires_ros2": False,
            }
        )
        _write_json_atomic(written_manifest, manifest)

    return GazeboSimulationResult(
        world_path=output,
        manifest_path=written_manifest,
        gui_plugin_path=(
            None if plugin_result is None else plugin_result.library_path
        ),
        gui_plugin_rebuilt=(
            False if plugin_result is None else plugin_result.rebuilt
        ),
        gui_plugin_installed=(
            False if plugin_result is None else plugin_result.installed
        ),
    )

def write_world_sdf(
    pallets: Sequence[Pallet],
    output_path: str | Path = settings.GAZEBO_WORLD_FILE,
) -> Path:
    """Compatibility wrapper returning only the generated world path."""

    return create_simulation(pallets, output_path=output_path).world_path


def build_world_sdf(
    pallets: Sequence[Pallet],
    *,
    gui_plugin_filename: str | None = None,
) -> tuple[str, dict[str, Any]]:
    """Return complete world XML and a diagnostic manifest dictionary.

    ``gui_plugin_filename`` is normally supplied by :func:`create_simulation`
    after the custom plugin has been built and installed. Direct callers may
    omit it; when integrated controls are enabled, the stable plugin base name
    is then used.
    """

    if settings.GAZEBO_ENABLE_INTEGRATED_FORKLIFT_CONTROLS:
        if gui_plugin_filename is None:
            gui_plugin_filename = settings.FORKLIFT_GUI_PLUGIN_NAME
    else:
        gui_plugin_filename = None

    pallet_poses = _arrange_pallets(pallets)
    forklift_pose = _forklift_pose(pallet_poses)
    bounds = _calculate_scene_bounds(pallet_poses, forklift_pose)
    ground_center_x, ground_center_y, ground_size_x, ground_size_y = (
        _ground_geometry(bounds)
    )
    camera_pose = _camera_pose(bounds)
    key_bindings = _key_bindings()

    sections: list[str] = [
        '<?xml version="1.0"?>',
        '<sdf version="1.10">',
        f'  <world name="{escape(settings.GAZEBO_WORLD_NAME)}">',
        _world_physics_xml(),
        _world_system_plugins_xml(),
        _scene_xml(),
        _gui_xml(
            camera_pose=camera_pose,
            plugin_filename=gui_plugin_filename,
            keys=key_bindings,
        ),
        _light_xml(),
        _ground_xml(
            center_x=ground_center_x,
            center_y=ground_center_y,
            size_x=ground_size_x,
            size_y=ground_size_y,
        ),
    ]

    manifest_items: list[dict[str, Any]] = []

    for pallet_index, pallet_pose in enumerate(pallet_poses, start=1):
        pallet_name = _safe_name(
            f"pallet_{pallet_index:02d}_{pallet_pose.pallet.pallet_id}"
        )
        sections.append(_pallet_model_xml(pallet_pose, pallet_name))
        manifest_items.append(
            {
                "type": "pallet",
                "model_name": pallet_name,
                "pallet_id": pallet_pose.pallet.pallet_id,
                "world_pose_m_rad": [
                    pallet_pose.center_x_m,
                    pallet_pose.center_y_m,
                    pallet_pose.base_z_m,
                    0.0,
                    0.0,
                    0.0,
                ],
                "box_count": pallet_pose.pallet.box_count,
                "load_kg": pallet_pose.pallet.current_load_kg,
            }
        )

        for box_index, box in enumerate(pallet_pose.pallet.boxes, start=1):
            box_xml, box_manifest = _box_model_xml(
                pallet_pose,
                box,
                pallet_index=pallet_index,
                box_index=box_index,
            )
            sections.append(box_xml)
            manifest_items.append(box_manifest)

    forklift_name = _safe_name(settings.FORKLIFT_MODEL_NAME)
    sections.append(_forklift_model_xml(forklift_pose, forklift_name))
    manifest_items.append(
        {
            "type": "forklift",
            "model_name": forklift_name,
            "world_pose_m_rad": list(forklift_pose),
            "fork_tip_clearance_m": (
                settings.GAZEBO_FORKLIFT_FORK_TIP_CLEARANCE_M
            ),
            "drive_topic": settings.FORKLIFT_CMD_VEL_TOPIC,
            "fork_position_topic": settings.FORKLIFT_FORK_POSITION_TOPIC,
        }
    )

    sections.extend(("  </world>", "</sdf>", ""))
    world_text = "\n".join(sections)

    try:
        ElementTree.fromstring(world_text)
    except ElementTree.ParseError as exc:
        raise GazeboSimulationError(
            f"Generated Gazebo world is not valid XML: {exc}"
        ) from exc

    controls_enabled = gui_plugin_filename is not None
    automatic_start = (
        controls_enabled and settings.GAZEBO_AUTO_START_SIMULATION
    )
    manifest: dict[str, Any] = {
        "world_name": settings.GAZEBO_WORLD_NAME,
        "pallet_count": len(pallets),
        "box_count": sum(pallet.box_count for pallet in pallets),
        "simulation_start": {
            "automatic": automatic_start,
            "mechanism": (
                "ForkliftTeleop GUI plugin requests pause=false"
                if automatic_start
                else "Use the Gazebo Play control"
            ),
            "world_control_service": (
                f"/world/{settings.GAZEBO_WORLD_NAME}/control"
            ),
        },
        "integrated_forklift_controls": {
            "enabled": controls_enabled,
            "gui_plugin_filename": gui_plugin_filename,
            "transport_only": True,
            "forward": key_bindings["forward"].label,
            "reverse": key_bindings["reverse"].label,
            "turn_left": key_bindings["left"].label,
            "turn_right": key_bindings["right"].label,
            "forks_up": key_bindings["lift"].label,
            "forks_down": key_bindings["lower"].label,
            "stop": key_bindings["stop"].label,
        },
        "items": manifest_items,
    }
    return world_text, manifest

def _prepare_local_assets() -> ForkliftGuiPluginResult | None:
    """Regenerate, validate, and build all local assets needed by the world."""

    if settings.GAZEBO_REGENERATE_FORKLIFT_ASSETS:
        try:
            from .generateForkliftModel import write_forklift_assets

            write_forklift_assets()
        except (OSError, ValueError, ElementTree.ParseError) as exc:
            raise GazeboSimulationError(
                f"Could not regenerate forklift assets: {exc}"
            ) from exc

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

    if settings.GAZEBO_VALIDATE_LOCAL_MODELS:
        pallet_model_file = (
            settings.PALLET_GAZEBO_MODEL_DIRECTORY / "model.sdf"
        )
        _validate_pallet_model_envelope(pallet_model_file)
        _validate_fork_channel_clearance(pallet_model_file)
        _validate_forklift_model(
            settings.FORKLIFT_MODEL_DIRECTORY / "model.sdf"
        )

    if not settings.GAZEBO_ENABLE_INTEGRATED_FORKLIFT_CONTROLS:
        return None

    try:
        return prepare_forklift_gui_plugin()
    except (ForkliftGuiPluginError, OSError) as exc:
        raise GazeboSimulationError(
            f"Could not prepare the ForkliftTeleop GUI plugin: {exc}"
        ) from exc

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
    if any(abs(current - target) > tolerance_m for current, target in zip(actual, expected)):
        raise GazeboSimulationError(
            "The local pallet model collision envelope does not match settings.py. "
            f"Actual bounds are {actual}; expected {expected}."
        )


def _validate_fork_channel_clearance(model_file: Path) -> None:
    """Verify that both forks fit through the pallet's collision channels.

    The forklift approaches along pallet X. Consequently, a collision during
    insertion is possible whenever a pallet collision overlaps a fork in both
    Y and Z, regardless of where that member lies along X.
    """

    try:
        root = ElementTree.parse(model_file).getroot()
    except (OSError, ElementTree.ParseError) as exc:
        raise GazeboSimulationError(
            f"Could not parse pallet model {model_file}: {exc}"
        ) from exc

    half_spacing = settings.FORKLIFT_FORK_CENTRE_SPACING_M / 2.0
    half_width = settings.FORKLIFT_FORK_WIDTH_M / 2.0
    half_thickness = settings.FORKLIFT_FORK_THICKNESS_M / 2.0
    fork_center_z = (
        settings.FORKLIFT_FORK_LOW_POSITION_Z_M
        + settings.FORKLIFT_FORK_INITIAL_POSITION_M
    )
    fork_intervals = {
        "left_fork": (
            half_spacing - half_width,
            half_spacing + half_width,
            fork_center_z - half_thickness,
            fork_center_z + half_thickness,
        ),
        "right_fork": (
            -half_spacing - half_width,
            -half_spacing + half_width,
            fork_center_z - half_thickness,
            fork_center_z + half_thickness,
        ),
    }

    interference: list[str] = []
    tolerance_m = 1e-6
    for link in root.findall(".//link"):
        link_pose = _parse_pose(link.findtext("pose"))
        _require_zero_rotation(link_pose[3:], "pallet link")
        for collision in link.findall("collision"):
            size_text = collision.findtext("geometry/box/size")
            if size_text is None:
                continue
            size = _parse_vector(size_text, 3, "pallet collision box size")
            collision_pose = _parse_pose(collision.findtext("pose"))
            _require_zero_rotation(collision_pose[3:], "pallet collision")

            center_y = link_pose[1] + collision_pose[1]
            center_z = link_pose[2] + collision_pose[2]
            collision_y_min = center_y - size[1] / 2.0
            collision_y_max = center_y + size[1] / 2.0
            collision_z_min = center_z - size[2] / 2.0
            collision_z_max = center_z + size[2] / 2.0

            for fork_name, (fork_y_min, fork_y_max, fork_z_min, fork_z_max) in (
                fork_intervals.items()
            ):
                overlap_y = (
                    fork_y_min < collision_y_max - tolerance_m
                    and fork_y_max > collision_y_min + tolerance_m
                )
                overlap_z = (
                    fork_z_min < collision_z_max - tolerance_m
                    and fork_z_max > collision_z_min + tolerance_m
                )
                if overlap_y and overlap_z:
                    collision_name = collision.get("name") or "unnamed_collision"
                    interference.append(f"{fork_name} vs {collision_name}")

    if interference:
        raise GazeboSimulationError(
            "The configured fork spacing / height does not fit the pallet "
            "collision channels: " + ", ".join(interference)
        )


def _validate_forklift_model(model_file: Path) -> None:
    """Check that the generated forklift still matches shared settings."""

    try:
        root = ElementTree.parse(model_file).getroot()
    except (OSError, ElementTree.ParseError) as exc:
        raise GazeboSimulationError(
            f"Could not parse forklift model {model_file}: {exc}"
        ) from exc

    model = _find_model_element(root, model_file)
    links = {link.get("name"): link for link in model.findall("link")}
    expected_link_names = {"body", "left_fork", "right_fork"}
    if set(links) != expected_link_names:
        raise GazeboSimulationError(
            f"Forklift model must contain exactly {sorted(expected_link_names)}; "
            f"found {sorted(name for name in links if name)}."
        )

    body = links["body"]
    if body is None:
        raise GazeboSimulationError("Forklift body link is missing.")
    _validate_body_collision_envelope(body)

    expected_body_mass = (
        settings.FORKLIFT_TOTAL_MASS_KG
        - 2.0 * settings.FORKLIFT_FORK_MASS_KG
    )
    _require_numeric_text_close(
        body.findtext("inertial/mass"),
        expected_body_mass,
        "forklift body mass",
    )

    half_spacing = settings.FORKLIFT_FORK_CENTRE_SPACING_M / 2.0
    expected_fork_poses = {
        "left_fork": (
            settings.FORKLIFT_FORK_LENGTH_M / 2.0,
            half_spacing,
            settings.FORKLIFT_FORK_LOW_POSITION_Z_M,
            0.0,
            0.0,
            0.0,
        ),
        "right_fork": (
            settings.FORKLIFT_FORK_LENGTH_M / 2.0,
            -half_spacing,
            settings.FORKLIFT_FORK_LOW_POSITION_Z_M,
            0.0,
            0.0,
            0.0,
        ),
    }
    expected_fork_size = (
        settings.FORKLIFT_FORK_LENGTH_M,
        settings.FORKLIFT_FORK_WIDTH_M,
        settings.FORKLIFT_FORK_THICKNESS_M,
    )
    for link_name in ("left_fork", "right_fork"):
        link = links[link_name]
        if link is None:
            raise GazeboSimulationError(f"Forklift link {link_name!r} is missing.")
        actual_pose = _parse_pose(link.findtext("pose"))
        _require_vector_close(
            actual_pose,
            expected_fork_poses[link_name],
            f"{link_name} pose",
        )
        _require_numeric_text_close(
            link.findtext("inertial/mass"),
            settings.FORKLIFT_FORK_MASS_KG,
            f"{link_name} mass",
        )
        size_text = link.findtext("collision/geometry/box/size")
        if size_text is None:
            raise GazeboSimulationError(
                f"Forklift link {link_name!r} has no primitive box collision."
            )
        _require_vector_close(
            _parse_vector(size_text, 3, f"{link_name} collision size"),
            expected_fork_size,
            f"{link_name} collision size",
        )

    joints = {joint.get("name"): joint for joint in model.findall("joint")}
    expected_joints = {
        "left_fork_lift_joint": "left_fork",
        "right_fork_lift_joint": "right_fork",
    }
    if set(joints) != set(expected_joints):
        raise GazeboSimulationError(
            "Forklift model must contain exactly the two configured lift joints."
        )
    for joint_name, child_name in expected_joints.items():
        joint = joints[joint_name]
        if joint is None:
            raise GazeboSimulationError(f"Forklift joint {joint_name!r} is missing.")
        if joint.get("type") != "prismatic":
            raise GazeboSimulationError(
                f"Forklift joint {joint_name!r} must be prismatic."
            )
        if joint.findtext("parent") != "body" or joint.findtext("child") != child_name:
            raise GazeboSimulationError(
                f"Forklift joint {joint_name!r} has the wrong parent / child."
            )
        axis_text = joint.findtext("axis/xyz")
        if axis_text is None:
            raise GazeboSimulationError(
                f"Forklift joint {joint_name!r} has no axis."
            )
        _require_vector_close(
            _parse_vector(axis_text, 3, f"{joint_name} axis"),
            (0.0, 0.0, 1.0),
            f"{joint_name} axis",
        )
        expected_values = {
            "axis/limit/lower": settings.FORKLIFT_FORK_MIN_POSITION_M,
            "axis/limit/upper": settings.FORKLIFT_FORK_MAX_POSITION_M,
            "axis/limit/effort": settings.FORKLIFT_FORK_JOINT_MAX_EFFORT_N,
            "axis/dynamics/damping": settings.FORKLIFT_FORK_JOINT_DAMPING,
            "axis/dynamics/friction": settings.FORKLIFT_FORK_JOINT_FRICTION,
        }
        for element_path, expected in expected_values.items():
            _require_numeric_text_close(
                joint.findtext(element_path),
                expected,
                f"{joint_name} {element_path}",
            )

    plugins = {plugin.get("name"): plugin for plugin in model.findall("plugin")}
    velocity_plugin = plugins.get("gz::sim::systems::VelocityControl")
    position_plugin = plugins.get("gz::sim::systems::JointPositionController")
    if velocity_plugin is None or position_plugin is None:
        raise GazeboSimulationError(
            "Forklift model is missing VelocityControl or JointPositionController."
        )
    if velocity_plugin.findtext("topic") != settings.FORKLIFT_CMD_VEL_TOPIC:
        raise GazeboSimulationError(
            "Forklift VelocityControl topic does not match settings.py."
        )
    if position_plugin.findtext("topic") != settings.FORKLIFT_FORK_POSITION_TOPIC:
        raise GazeboSimulationError(
            "Forklift position-controller topic does not match settings.py."
        )
    controller_joints = [
        element.text for element in position_plugin.findall("joint_name")
    ]
    if controller_joints != list(expected_joints):
        raise GazeboSimulationError(
            "Forklift position controller must drive both lift joints in order."
        )
    if position_plugin.findtext("use_velocity_commands") != "true":
        raise GazeboSimulationError(
            "Forklift position controller must use velocity-limited position mode."
        )
    _require_numeric_text_close(
        position_plugin.findtext("initial_position"),
        settings.FORKLIFT_FORK_INITIAL_POSITION_M,
        "fork controller initial position",
    )
    _require_numeric_text_close(
        position_plugin.findtext("cmd_max"),
        settings.FORKLIFT_MAX_FORK_VELOCITY_MPS,
        "fork controller maximum velocity",
    )
    _require_numeric_text_close(
        position_plugin.findtext("cmd_min"),
        -settings.FORKLIFT_MAX_FORK_VELOCITY_MPS,
        "fork controller minimum velocity",
    )


def _validate_body_collision_envelope(body: ElementTree.Element) -> None:
    bounds: list[tuple[float, float, float, float, float, float]] = []
    link_pose = _parse_pose(body.findtext("pose"))
    _require_zero_rotation(link_pose[3:], "forklift body link")
    for collision in body.findall("collision"):
        size_text = collision.findtext("geometry/box/size")
        if size_text is None:
            continue
        size = _parse_vector(size_text, 3, "forklift body collision size")
        pose = _parse_pose(collision.findtext("pose"))
        _require_zero_rotation(pose[3:], "forklift body collision")
        center_x = link_pose[0] + pose[0]
        center_y = link_pose[1] + pose[1]
        center_z = link_pose[2] + pose[2]
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
            "Forklift body contains no primitive box collisions."
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
        -settings.FORKLIFT_BODY_LENGTH_M,
        0.0,
        -settings.FORKLIFT_BODY_WIDTH_M / 2.0,
        settings.FORKLIFT_BODY_WIDTH_M / 2.0,
        0.0,
        settings.FORKLIFT_BODY_HEIGHT_M,
    )
    _require_vector_close(actual, expected, "forklift body collision envelope")


def _require_numeric_text_close(
    text: str | None,
    expected: float,
    description: str,
) -> None:
    if text is None:
        raise GazeboSimulationError(f"Missing {description}.")
    try:
        actual = float(text)
    except ValueError as exc:
        raise GazeboSimulationError(f"Non-numeric {description}: {text!r}.") from exc
    if not math.isclose(actual, expected, rel_tol=1e-9, abs_tol=1e-9):
        raise GazeboSimulationError(
            f"{description} is {actual}; expected {expected}."
        )


def _require_vector_close(
    actual: Iterable[float],
    expected: Iterable[float],
    description: str,
) -> None:
    actual_values = tuple(actual)
    expected_values = tuple(expected)
    if len(actual_values) != len(expected_values) or any(
        not math.isclose(current, target, rel_tol=1e-9, abs_tol=1e-9)
        for current, target in zip(actual_values, expected_values)
    ):
        raise GazeboSimulationError(
            f"{description} is {actual_values}; expected {expected_values}."
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
                f"Pallet {pallet.pallet_id!r} dimensions {actual} do not match "
                f"the configured Gazebo pallet dimensions {configured}."
            )

        for box in pallet.boxes:
            if not box.is_placed:
                raise GazeboSimulationError(
                    f"Box {box.box_id!r} on pallet {pallet.pallet_id!r} "
                    "has no placement."
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
        center_x += pallet.length_mm * MM_TO_M + settings.GAZEBO_PALLET_GAP_M
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

    # The forklift model origin is its body front face and its forks extend in
    # local +X. The pallet stringers also run along X, so this pose aligns the
    # forks with the channels rather than approaching perpendicular to them.
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


def _ground_geometry(bounds: _SceneBounds) -> tuple[float, float, float, float]:
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
    return f'''    <gravity>{_format_values(settings.GAZEBO_GRAVITY_MPS2)}</gravity>
    <physics name="pallet_stacker_physics" type="ignored">
      <max_step_size>{_fmt(settings.GAZEBO_PHYSICS_MAX_STEP_SIZE_S)}</max_step_size>
      <real_time_factor>{_fmt(settings.GAZEBO_REAL_TIME_FACTOR)}</real_time_factor>
    </physics>'''


def _world_system_plugins_xml() -> str:
    return "\n".join(
        (
            '    <plugin filename="gz-sim-physics-system" '
            'name="gz::sim::systems::Physics"/>',
            '    <plugin filename="gz-sim-user-commands-system" '
            'name="gz::sim::systems::UserCommands"/>',
            '    <plugin filename="gz-sim-scene-broadcaster-system" '
            'name="gz::sim::systems::SceneBroadcaster"/>',
        )
    )


def _scene_xml() -> str:
    return f'''    <scene>
      <ambient>{_format_values(settings.GAZEBO_AMBIENT_LIGHT_RGBA)}</ambient>
      <background>{_format_values(settings.GAZEBO_BACKGROUND_RGBA)}</background>
      <shadows>{_bool_text(not settings.GAZEBO_DISABLE_SHADOWS)}</shadows>
      <grid>{_bool_text(settings.GAZEBO_SHOW_GRID)}</grid>
    </scene>'''


def _gui_xml(
    *,
    camera_pose: tuple[float, float, float, float, float, float],
    plugin_filename: str | None,
    keys: dict[str, _KeyBinding],
) -> str:
    """Build the complete Harmonic GUI configuration for the world."""

    world = escape(settings.GAZEBO_WORLD_NAME)
    automatic_start = (
        plugin_filename is not None and settings.GAZEBO_AUTO_START_SIMULATION
    )
    start_paused = _bool_text(not automatic_start)

    forklift_plugin_xml = ""
    if plugin_filename is not None:
        plugin_file = escape(plugin_filename)
        forklift_plugin_xml = f'''
      <plugin filename="{plugin_file}" name="Forklift controls">
        <gz-gui>
          <title>Forklift controls</title>
          <property type="double" key="width">{settings.FORKLIFT_GUI_PANEL_WIDTH_PX}</property>
          <property type="double" key="height">{settings.FORKLIFT_GUI_PANEL_HEIGHT_PX}</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View">
            <line own="right" target="right"/>
            <line own="top" target="top"/>
          </anchors>
        </gz-gui>
        <drive_topic>{escape(settings.FORKLIFT_CMD_VEL_TOPIC)}</drive_topic>
        <fork_topic>{escape(settings.FORKLIFT_FORK_POSITION_TOPIC)}</fork_topic>
        <world_control_service>/world/{world}/control</world_control_service>
        <max_linear_velocity>{_fmt(settings.FORKLIFT_MAX_LINEAR_VELOCITY_MPS)}</max_linear_velocity>
        <max_angular_velocity>{_fmt(settings.FORKLIFT_MAX_TURN_VELOCITY_RAD_S)}</max_angular_velocity>
        <max_linear_acceleration>{_fmt(settings.FORKLIFT_MAX_LINEAR_ACCELERATION_MPS2)}</max_linear_acceleration>
        <max_angular_acceleration>{_fmt(settings.FORKLIFT_MAX_TURN_ACCELERATION_RAD_S2)}</max_angular_acceleration>
        <max_fork_velocity>{_fmt(settings.FORKLIFT_MAX_FORK_VELOCITY_MPS)}</max_fork_velocity>
        <fork_minimum>{_fmt(settings.FORKLIFT_FORK_MIN_POSITION_M)}</fork_minimum>
        <fork_maximum>{_fmt(settings.FORKLIFT_FORK_MAX_POSITION_M)}</fork_maximum>
        <fork_initial>{_fmt(settings.FORKLIFT_FORK_INITIAL_POSITION_M)}</fork_initial>
        <update_rate_hz>{_fmt(settings.FORKLIFT_TELEOP_RATE_HZ)}</update_rate_hz>
        <auto_start>{_bool_text(settings.GAZEBO_AUTO_START_SIMULATION)}</auto_start>
        <auto_start_retry_interval_ms>{settings.GAZEBO_AUTO_START_RETRY_INTERVAL_MS}</auto_start_retry_interval_ms>
        <auto_start_request_timeout_ms>{settings.GAZEBO_AUTO_START_REQUEST_TIMEOUT_MS}</auto_start_request_timeout_ms>
        <auto_start_max_attempts>{settings.GAZEBO_AUTO_START_MAX_ATTEMPTS}</auto_start_max_attempts>
        {_key_xml("forward", keys["forward"])}
        {_key_xml("reverse", keys["reverse"])}
        {_key_xml("left", keys["left"])}
        {_key_xml("right", keys["right"])}
        {_key_xml("lift", keys["lift"])}
        {_key_xml("lower", keys["lower"])}
        {_key_xml("stop", keys["stop"])}
      </plugin>'''

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
      <plugin filename="InteractiveViewControl" name="Interactive view control"/>
      <plugin filename="CameraTracking" name="Camera tracking"/>
      <plugin filename="MarkerManager" name="Marker manager"/>
      <plugin filename="SelectEntities" name="Select entities"/>
      <plugin filename="EntityContextMenuPlugin" name="Entity context menu"/>
      <plugin filename="VisualizationCapabilities" name="Visualization capabilities"/>
      <plugin filename="WorldControl" name="World control">
        <gz-gui>
          <title>World control</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property>
          <property type="double" key="height">72</property>
          <property type="double" key="width">121</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View">
            <line own="left" target="left"/>
            <line own="bottom" target="bottom"/>
          </anchors>
        </gz-gui>
        <play_pause>true</play_pause>
        <step>true</step>
        <start_paused>{start_paused}</start_paused>
        <service>/world/{world}/control</service>
        <stats_topic>/world/{world}/stats</stats_topic>
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
            <line own="right" target="right"/>
            <line own="bottom" target="bottom"/>
          </anchors>
        </gz-gui>
        <sim_time>true</sim_time>
        <real_time>true</real_time>
        <real_time_factor>true</real_time_factor>
        <iterations>true</iterations>
        <topic>/world/{world}/stats</topic>
      </plugin>
      <plugin filename="EntityTree" name="Entity tree"/>{forklift_plugin_xml}
    </gui>'''

def _key_xml(name: str, binding: _KeyBinding) -> str:
    return (
        f"<{name}_key>{binding.code}</{name}_key>\n"
        f"        <{name}_label>{escape(binding.label)}</{name}_label>"
    )


def _light_xml() -> str:
    cast_shadows = _bool_text(not settings.GAZEBO_DISABLE_SHADOWS)
    return f'''    <light type="directional" name="sun">
      <pose>0 0 10 0 0 0</pose>
      <cast_shadows>{cast_shadows}</cast_shadows>
      <direction>{_format_values(settings.GAZEBO_LIGHT_DIRECTION)}</direction>
      <diffuse>{_format_values(settings.GAZEBO_LIGHT_DIFFUSE_RGBA)}</diffuse>
      <specular>{_format_values(settings.GAZEBO_LIGHT_SPECULAR_RGBA)}</specular>
    </light>'''

def _ground_xml(
    *,
    center_x: float,
    center_y: float,
    size_x: float,
    size_y: float,
) -> str:
    color = _format_values(settings.GAZEBO_GROUND_RGBA)
    friction = _fmt(settings.GAZEBO_GROUND_FRICTION)
    return f'''    <model name="ground_plane">
      <static>true</static>
      <pose>{_fmt(center_x)} {_fmt(center_y)} 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
        <max_contacts>{settings.GAZEBO_MAX_CONTACTS_PER_COLLISION}</max_contacts>
        <geometry>
            <plane><normal>0 0 1</normal><size>{_fmt(size_x)} {_fmt(size_y)}</size></plane>
          </geometry>
          <surface>
            <friction>
              <ode><mu>{friction}</mu><mu2>{friction}</mu2></ode>
              <bullet><friction>{friction}</friction><friction2>{friction}</friction2></bullet>
            </friction>
            <bounce>
              <restitution_coefficient>{_fmt(settings.GAZEBO_GROUND_RESTITUTION)}</restitution_coefficient>
              <threshold>100000</threshold>
            </bounce>
            <contact>
              <ode>
                <kp>{_fmt(settings.GAZEBO_GROUND_CONTACT_STIFFNESS)}</kp>
                <kd>{_fmt(settings.GAZEBO_GROUND_CONTACT_DAMPING)}</kd>
                <max_vel>{_fmt(settings.GAZEBO_CONTACT_MAX_CORRECTING_VELOCITY_MPS)}</max_vel>
                <min_depth>{_fmt(settings.GAZEBO_CONTACT_MIN_DEPTH_M)}</min_depth>
              </ode>
            </contact>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane><normal>0 0 1</normal><size>{_fmt(size_x)} {_fmt(size_y)}</size></plane>
          </geometry>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
        </visual>
      </link>
    </model>'''

def _pallet_model_xml(pose: _PalletWorldPose, model_name: str) -> str:
    if not settings.GAZEBO_INLINE_LOCAL_MODELS:
        return f'''    <include>
      <uri>model://{escape(settings.PALLET_GAZEBO_MODEL_NAME)}</uri>
      <name>{escape(model_name)}</name>
      <pose>{_fmt(pose.center_x_m)} {_fmt(pose.center_y_m)} {_fmt(pose.base_z_m)} 0 0 0</pose>
    </include>'''

    model_file = settings.PALLET_GAZEBO_MODEL_DIRECTORY / "model.sdf"
    model = _load_model_copy(model_file)
    model.set("name", model_name)
    _set_direct_child_text(model, "pose", _format_values((
        pose.center_x_m,
        pose.center_y_m,
        pose.base_z_m,
        0.0,
        0.0,
        0.0,
    )))
    _set_direct_child_text(model, "static", "false")
    _set_direct_child_text(
        model,
        "allow_auto_disable",
        _bool_text(settings.GAZEBO_PALLET_AUTO_DISABLE),
    )
    _configure_pallet_mass(model)
    for collision in model.findall("./link/collision"):
        _configure_collision_surface(
            collision,
            friction=settings.GAZEBO_PALLET_FRICTION,
            restitution=settings.GAZEBO_PALLET_RESTITUTION,
            stiffness=settings.GAZEBO_PALLET_CONTACT_STIFFNESS,
            damping=settings.GAZEBO_PALLET_CONTACT_DAMPING,
        )
    return _serialize_world_model(model)


def _forklift_model_xml(
    pose: tuple[float, float, float, float, float, float],
    model_name: str,
) -> str:
    if not settings.GAZEBO_INLINE_LOCAL_MODELS:
        return f'''    <include>
      <uri>model://{escape(settings.FORKLIFT_MODEL_NAME)}</uri>
      <name>{escape(model_name)}</name>
      <pose>{_format_values(pose)}</pose>
    </include>'''

    model_file = settings.FORKLIFT_MODEL_DIRECTORY / "model.sdf"
    model = _load_model_copy(model_file)
    model.set("name", model_name)
    model.set("name", model_name)
    _set_direct_child_text(model, "pose", _format_values(pose))
    _set_direct_child_text(model, "static", "false")

    for collision in model.findall(".//collision"):
        _set_collision_max_contacts(collision)

    return _serialize_world_model(model)


def _box_model_xml(
    pallet_pose: _PalletWorldPose,
    box: Box,
    *,
    pallet_index: int,
    box_index: int,
) -> tuple[str, dict[str, Any]]:
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
        f"box_p{pallet_index:02d}_{box_index:04d}_{box.box_id}"
    )
    auto_disable = _bool_text(settings.GAZEBO_BOX_AUTO_DISABLE)

    xml = f'''    <model name="{escape(model_name)}">
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
            <max_contacts>{settings.GAZEBO_MAX_CONTACTS_PER_COLLISION}</max_contacts>
            <geometry>
                <box><size>{_fmt(dimensions_m.x)} {_fmt(dimensions_m.y)} {_fmt(dimensions_m.z)}</size></box>
            </geometry>
          {_box_surface_xml()}
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>{_fmt(dimensions_m.x)} {_fmt(dimensions_m.y)} {_fmt(dimensions_m.z)}</size></box>
          </geometry>
          <material>
            <ambient>{_fmt(red * 0.75)} {_fmt(green * 0.75)} {_fmt(blue * 0.75)} 1</ambient>
            <diffuse>{_fmt(red)} {_fmt(green)} {_fmt(blue)} 1</diffuse>
            <specular>0.08 0.08 0.08 1</specular>
          </material>
        </visual>
      </link>
    </model>'''

    manifest = {
        "type": "box",
        "model_name": model_name,
        "pallet_id": pallet_pose.pallet.pallet_id,
        "identifier": box.box_id,
        "sku": box.sku,
        "mass_kg": box.weight_kg,
        "simulation_mass_kg": mass,
        "orientation": None if box.orientation is None else box.orientation.value,
        "local_min_corner_mm": [position.x, position.y, position.z],
        "oriented_size_mm": [dimensions.x, dimensions.y, dimensions.z],
        "world_center_m": [center_x, center_y, center_z],
    }
    return xml, manifest


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


def _configure_pallet_mass(model: ElementTree.Element) -> None:
    inertial = model.find("./link/inertial")
    if inertial is None:
        raise GazeboSimulationError("Pallet model has no link/inertial element.")

    mass_element = _ensure_xml_path(inertial, "mass")
    try:
        source_mass = float(mass_element.text or "0")
    except ValueError as exc:
        raise GazeboSimulationError("Pallet source mass is not numeric.") from exc

    target_mass = settings.GAZEBO_PALLET_MASS_KG
    scale = target_mass / source_mass if source_mass > 0.0 else 1.0
    mass_element.text = _fmt(target_mass)

    inertia = _ensure_xml_path(inertial, "inertia")
    for component_name in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
        component = _ensure_xml_path(inertia, component_name)
        try:
            source_value = float(component.text or "0")
        except ValueError as exc:
            raise GazeboSimulationError(
                f"Pallet inertia component {component_name} is not numeric."
            ) from exc
        component.text = _fmt(source_value * scale)

def _set_collision_max_contacts(
    collision: ElementTree.Element,
) -> None:
    """Apply the shared contact-point limit to one collision element."""

    _set_direct_child_text(
        collision,
        "max_contacts",
        str(settings.GAZEBO_MAX_CONTACTS_PER_COLLISION),
    )

def _configure_collision_surface(
    collision: ElementTree.Element,
    *,
    friction: float,
    restitution: float,
    stiffness: float,
    damping: float,
) -> None:
    _set_collision_max_contacts(collision)

    _ensure_xml_path(
        collision,
        "surface/friction/ode/mu",
    ).text = _fmt(friction)

    _ensure_xml_path(
        collision,
        "surface/friction/ode/mu2",
    ).text = _fmt(friction)
    
    _ensure_xml_path(collision, "surface/friction/ode/mu").text = _fmt(friction)
    _ensure_xml_path(collision, "surface/friction/ode/mu2").text = _fmt(friction)
    _ensure_xml_path(collision, "surface/friction/bullet/friction").text = _fmt(
        friction
    )
    _ensure_xml_path(collision, "surface/friction/bullet/friction2").text = _fmt(
        friction
    )
    _ensure_xml_path(
        collision,
        "surface/bounce/restitution_coefficient",
    ).text = _fmt(restitution)
    _ensure_xml_path(collision, "surface/bounce/threshold").text = "100000"
    _ensure_xml_path(collision, "surface/contact/ode/kp").text = _fmt(stiffness)
    _ensure_xml_path(collision, "surface/contact/ode/kd").text = _fmt(damping)
    _ensure_xml_path(collision, "surface/contact/ode/max_vel").text = _fmt(
        settings.GAZEBO_CONTACT_MAX_CORRECTING_VELOCITY_MPS
    )
    _ensure_xml_path(collision, "surface/contact/ode/min_depth").text = _fmt(
        settings.GAZEBO_CONTACT_MIN_DEPTH_M
    )


def _load_model_copy(model_file: Path) -> ElementTree.Element:
    try:
        root = ElementTree.parse(model_file).getroot()
    except (OSError, ElementTree.ParseError) as exc:
        raise GazeboSimulationError(
            f"Could not load local Gazebo model {model_file}: {exc}"
        ) from exc
    return copy.deepcopy(_find_model_element(root, model_file))


def _find_model_element(
    root: ElementTree.Element,
    source: Path,
) -> ElementTree.Element:
    if root.tag == "model":
        return root
    model = root.find("model")
    if model is None:
        raise GazeboSimulationError(f"No <model> element found in {source}.")
    return model


def _serialize_world_model(model: ElementTree.Element) -> str:
    ElementTree.indent(model, space="  ")
    text = ElementTree.tostring(model, encoding="unicode", short_empty_elements=True)
    return "\n".join("    " + line for line in text.splitlines())


def _set_direct_child_text(
    parent: ElementTree.Element,
    tag: str,
    text: str,
) -> None:
    child = parent.find(tag)
    if child is None:
        child = ElementTree.Element(tag)
        parent.insert(0, child)
    child.text = text


def _ensure_xml_path(parent: ElementTree.Element, path: str) -> ElementTree.Element:
    node = parent
    for tag in path.split("/"):
        child = node.find(tag)
        if child is None:
            child = ElementTree.SubElement(node, tag)
        node = child
    return node


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
    values = tuple((70 + value % 150) / 255.0 for value in digest[:3])
    return values  # type: ignore[return-value]


def _safe_name(value: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_]+", "_", value).strip("_")
    if not cleaned:
        cleaned = "entity"
    if cleaned[0].isdigit():
        cleaned = "entity_" + cleaned
    return cleaned


def _key_bindings() -> dict[str, _KeyBinding]:
    configured = {
        "forward": settings.FORKLIFT_KEY_FORWARD,
        "reverse": settings.FORKLIFT_KEY_REVERSE,
        "left": settings.FORKLIFT_KEY_TURN_LEFT,
        "right": settings.FORKLIFT_KEY_TURN_RIGHT,
        "lift": settings.FORKLIFT_KEY_LIFT,
        "lower": settings.FORKLIFT_KEY_LOWER,
        "stop": settings.FORKLIFT_KEY_STOP,
    }
    bindings = {name: _qt_key(value) for name, value in configured.items()}

    codes = [binding.code for binding in bindings.values()]
    if len(codes) != len(set(codes)):
        raise GazeboSimulationError(
            "Forklift key bindings must be unique. Current values are: "
            + ", ".join(f"{name}={value!r}" for name, value in configured.items())
        )
    return bindings


def _qt_key(value: str) -> _KeyBinding:
    normalized = value.strip().upper().replace("_", "").replace(" ", "")
    named_keys = {
        "UP": (0x01000013, "Up"),
        "DOWN": (0x01000015, "Down"),
        "LEFT": (0x01000012, "Left"),
        "RIGHT": (0x01000014, "Right"),
        "SPACE": (0x20, "Space"),
        "ESC": (0x01000000, "Esc"),
        "ESCAPE": (0x01000000, "Esc"),
        "PAGEUP": (0x01000016, "Page Up"),
        "PAGEDOWN": (0x01000017, "Page Down"),
    }
    if normalized in named_keys:
        code, label = named_keys[normalized]
        return _KeyBinding(code=code, label=label)
    if len(normalized) == 1 and normalized.isprintable():
        return _KeyBinding(code=ord(normalized), label=normalized)
    raise GazeboSimulationError(
        f"Unsupported forklift key name {value!r}. Use one printable character "
        "or UP, DOWN, LEFT, RIGHT, SPACE, ESCAPE, PAGEUP or PAGEDOWN."
    )


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
        values = tuple(float(part) for part in parts)
    except ValueError as exc:
        raise GazeboSimulationError(
            f"Non-numeric value in {description}: {text!r}."
        ) from exc
    if any(not math.isfinite(value) for value in values):
        raise GazeboSimulationError(f"Non-finite value in {description}: {text!r}.")
    return values


def _require_zero_rotation(rotation: Iterable[float], description: str) -> None:
    if any(abs(value) > 1e-9 for value in rotation):
        raise GazeboSimulationError(
            f"Cannot validate rotated {description}; expected axis-aligned boxes."
        )


def _validate_settings() -> None:
    """Validate shared settings before creating or compiling any assets."""

    positive_values = {
        "GAZEBO_PHYSICS_MAX_STEP_SIZE_S": settings.GAZEBO_PHYSICS_MAX_STEP_SIZE_S,
        "GAZEBO_GROUND_MIN_SIZE_M": settings.GAZEBO_GROUND_MIN_SIZE_M,
        "GAZEBO_GROUND_CONTACT_STIFFNESS": settings.GAZEBO_GROUND_CONTACT_STIFFNESS,
        "GAZEBO_PALLET_MASS_KG": settings.GAZEBO_PALLET_MASS_KG,
        "GAZEBO_PALLET_CONTACT_STIFFNESS": settings.GAZEBO_PALLET_CONTACT_STIFFNESS,
        "GAZEBO_BOX_MIN_MASS_KG": settings.GAZEBO_BOX_MIN_MASS_KG,
        "GAZEBO_BOX_CONTACT_STIFFNESS": settings.GAZEBO_BOX_CONTACT_STIFFNESS,
        "GAZEBO_CAMERA_NEAR_CLIP_M": settings.GAZEBO_CAMERA_NEAR_CLIP_M,
        "GAZEBO_CAMERA_FAR_CLIP_M": settings.GAZEBO_CAMERA_FAR_CLIP_M,
        "FORKLIFT_BODY_LENGTH_M": settings.FORKLIFT_BODY_LENGTH_M,
        "FORKLIFT_BODY_WIDTH_M": settings.FORKLIFT_BODY_WIDTH_M,
        "FORKLIFT_BODY_HEIGHT_M": settings.FORKLIFT_BODY_HEIGHT_M,
        "FORKLIFT_FORK_LENGTH_M": settings.FORKLIFT_FORK_LENGTH_M,
        "FORKLIFT_FORK_WIDTH_M": settings.FORKLIFT_FORK_WIDTH_M,
        "FORKLIFT_FORK_THICKNESS_M": settings.FORKLIFT_FORK_THICKNESS_M,
        "FORKLIFT_FORK_CENTRE_SPACING_M": settings.FORKLIFT_FORK_CENTRE_SPACING_M,
        "FORKLIFT_TOTAL_MASS_KG": settings.FORKLIFT_TOTAL_MASS_KG,
        "FORKLIFT_FORK_MASS_KG": settings.FORKLIFT_FORK_MASS_KG,
        "FORKLIFT_REAR_TO_FRONT_BODY_MASS_RATIO": settings.FORKLIFT_REAR_TO_FRONT_BODY_MASS_RATIO,
        "FORKLIFT_CONTACT_STIFFNESS": settings.FORKLIFT_CONTACT_STIFFNESS,
        "FORKLIFT_MAX_LINEAR_VELOCITY_MPS": settings.FORKLIFT_MAX_LINEAR_VELOCITY_MPS,
        "FORKLIFT_MAX_TURN_VELOCITY_RAD_S": settings.FORKLIFT_MAX_TURN_VELOCITY_RAD_S,
        "FORKLIFT_MAX_LINEAR_ACCELERATION_MPS2": settings.FORKLIFT_MAX_LINEAR_ACCELERATION_MPS2,
        "FORKLIFT_MAX_TURN_ACCELERATION_RAD_S2": settings.FORKLIFT_MAX_TURN_ACCELERATION_RAD_S2,
        "FORKLIFT_MAX_FORK_VELOCITY_MPS": settings.FORKLIFT_MAX_FORK_VELOCITY_MPS,
        "FORKLIFT_FORK_JOINT_MAX_EFFORT_N": settings.FORKLIFT_FORK_JOINT_MAX_EFFORT_N,
        "FORKLIFT_TELEOP_RATE_HZ": settings.FORKLIFT_TELEOP_RATE_HZ,
    }
    for name, value in positive_values.items():
        if not math.isfinite(value) or value <= 0.0:
            raise GazeboSimulationError(
                f"settings.{name} must be positive and finite."
            )

    non_negative_values = {
        "GAZEBO_REAL_TIME_FACTOR": settings.GAZEBO_REAL_TIME_FACTOR,
        "GAZEBO_GROUND_MARGIN_M": settings.GAZEBO_GROUND_MARGIN_M,
        "GAZEBO_GROUND_FRICTION": settings.GAZEBO_GROUND_FRICTION,
        "GAZEBO_GROUND_CONTACT_DAMPING": settings.GAZEBO_GROUND_CONTACT_DAMPING,
        "GAZEBO_PALLET_GAP_M": settings.GAZEBO_PALLET_GAP_M,
        "GAZEBO_PALLET_SPAWN_CLEARANCE_M": settings.GAZEBO_PALLET_SPAWN_CLEARANCE_M,
        "GAZEBO_BOX_SPAWN_CLEARANCE_M": settings.GAZEBO_BOX_SPAWN_CLEARANCE_M,
        "GAZEBO_PALLET_FRICTION": settings.GAZEBO_PALLET_FRICTION,
        "GAZEBO_PALLET_CONTACT_DAMPING": settings.GAZEBO_PALLET_CONTACT_DAMPING,
        "GAZEBO_BOX_FRICTION": settings.GAZEBO_BOX_FRICTION,
        "GAZEBO_BOX_CONTACT_DAMPING": settings.GAZEBO_BOX_CONTACT_DAMPING,
        "GAZEBO_CONTACT_MAX_CORRECTING_VELOCITY_MPS": settings.GAZEBO_CONTACT_MAX_CORRECTING_VELOCITY_MPS,
        "GAZEBO_CONTACT_MIN_DEPTH_M": settings.GAZEBO_CONTACT_MIN_DEPTH_M,
        "GAZEBO_FORKLIFT_FORK_TIP_CLEARANCE_M": settings.GAZEBO_FORKLIFT_FORK_TIP_CLEARANCE_M,
        "GAZEBO_FORKLIFT_SPAWN_CLEARANCE_M": settings.GAZEBO_FORKLIFT_SPAWN_CLEARANCE_M,
        "FORKLIFT_BODY_FLOOR_FRICTION": settings.FORKLIFT_BODY_FLOOR_FRICTION,
        "FORKLIFT_FORK_CONTACT_FRICTION": settings.FORKLIFT_FORK_CONTACT_FRICTION,
        "FORKLIFT_CONTACT_DAMPING": settings.FORKLIFT_CONTACT_DAMPING,
        "FORKLIFT_FORK_JOINT_DAMPING": settings.FORKLIFT_FORK_JOINT_DAMPING,
        "FORKLIFT_FORK_JOINT_FRICTION": settings.FORKLIFT_FORK_JOINT_FRICTION,
    }
    for name, value in non_negative_values.items():
        if not math.isfinite(value) or value < 0.0:
            raise GazeboSimulationError(
                f"settings.{name} must be non-negative and finite."
            )

    for name, value in (
        ("GAZEBO_GROUND_RESTITUTION", settings.GAZEBO_GROUND_RESTITUTION),
        ("GAZEBO_PALLET_RESTITUTION", settings.GAZEBO_PALLET_RESTITUTION),
        ("GAZEBO_BOX_RESTITUTION", settings.GAZEBO_BOX_RESTITUTION),
    ):
        if not math.isfinite(value) or not 0.0 <= value <= 1.0:
            raise GazeboSimulationError(
                f"settings.{name} must be in the range [0, 1]."
            )

    positive_integer_values = {
        "GAZEBO_MAX_CONTACTS_PER_COLLISION": settings.GAZEBO_MAX_CONTACTS_PER_COLLISION,
        "GAZEBO_AUTO_START_RETRY_INTERVAL_MS": settings.GAZEBO_AUTO_START_RETRY_INTERVAL_MS,
        "GAZEBO_AUTO_START_REQUEST_TIMEOUT_MS": settings.GAZEBO_AUTO_START_REQUEST_TIMEOUT_MS,
        "GAZEBO_AUTO_START_MAX_ATTEMPTS": settings.GAZEBO_AUTO_START_MAX_ATTEMPTS,
        "FORKLIFT_GUI_PANEL_WIDTH_PX": settings.FORKLIFT_GUI_PANEL_WIDTH_PX,
        "FORKLIFT_GUI_PANEL_HEIGHT_PX": settings.FORKLIFT_GUI_PANEL_HEIGHT_PX,
    }
    for name, value in positive_integer_values.items():
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            raise GazeboSimulationError(
                f"settings.{name} must be a positive integer."
            )

    max_contacts = settings.GAZEBO_MAX_CONTACTS_PER_COLLISION

    if (
        isinstance(max_contacts, bool)
        or not isinstance(max_contacts, int)
        or max_contacts <= 0
    ):
        raise ValueError(
            "GAZEBO_MAX_CONTACTS_PER_COLLISION must be a positive integer."
        )

    if (
        isinstance(settings.GAZEBO_GUI_PLUGIN_PARALLEL_JOBS, bool)
        or not isinstance(settings.GAZEBO_GUI_PLUGIN_PARALLEL_JOBS, int)
        or settings.GAZEBO_GUI_PLUGIN_PARALLEL_JOBS < 0
    ):
        raise GazeboSimulationError(
            "GAZEBO_GUI_PLUGIN_PARALLEL_JOBS must be a non-negative integer."
        )

    if settings.GAZEBO_AUTO_START_SIMULATION and not (
        settings.GAZEBO_ENABLE_INTEGRATED_FORKLIFT_CONTROLS
    ):
        raise GazeboSimulationError(
            "GAZEBO_AUTO_START_SIMULATION requires integrated forklift controls."
        )
    if not settings.GAZEBO_GUI_PLUGIN_BUILD_TYPE.strip():
        raise GazeboSimulationError(
            "GAZEBO_GUI_PLUGIN_BUILD_TYPE must not be empty."
        )
    if not settings.GAZEBO_CMAKE_EXECUTABLE.strip():
        raise GazeboSimulationError(
            "GAZEBO_CMAKE_EXECUTABLE must not be empty."
        )

    if len(settings.GAZEBO_GRAVITY_MPS2) != 3 or any(
        not math.isfinite(value) for value in settings.GAZEBO_GRAVITY_MPS2
    ):
        raise GazeboSimulationError(
            "GAZEBO_GRAVITY_MPS2 must contain three finite values."
        )
    if (
        len(settings.GAZEBO_LIGHT_DIRECTION) != 3
        or any(
            not math.isfinite(value)
            for value in settings.GAZEBO_LIGHT_DIRECTION
        )
        or all(
            abs(value) <= 1e-12
            for value in settings.GAZEBO_LIGHT_DIRECTION
        )
    ):
        raise GazeboSimulationError(
            "GAZEBO_LIGHT_DIRECTION must contain a non-zero finite 3D vector."
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
    if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", settings.GAZEBO_WORLD_NAME):
        raise GazeboSimulationError(
            "GAZEBO_WORLD_NAME must start with a letter or underscore and "
            "contain only letters, numbers and underscores."
        )

    for name, topic in (
        ("FORKLIFT_CMD_VEL_TOPIC", settings.FORKLIFT_CMD_VEL_TOPIC),
        (
            "FORKLIFT_FORK_POSITION_TOPIC",
            settings.FORKLIFT_FORK_POSITION_TOPIC,
        ),
    ):
        if (
            not isinstance(topic, str)
            or not topic.startswith("/")
            or any(character.isspace() for character in topic)
        ):
            raise GazeboSimulationError(
                f"settings.{name} must be an absolute topic without whitespace."
            )

    if settings.FORKLIFT_GUI_PLUGIN_NAME != "ForkliftTeleop":
        raise GazeboSimulationError(
            "FORKLIFT_GUI_PLUGIN_NAME must remain 'ForkliftTeleop' unless the "
            "C++ class, QML resource prefix and CMake target are all renamed."
        )

    expected_plugin_library_name = (
        f"lib{settings.FORKLIFT_GUI_PLUGIN_NAME}.so"
    )
    for name, path in (
        (
            "FORKLIFT_GUI_PLUGIN_BUILD_LIBRARY_FILE",
            settings.FORKLIFT_GUI_PLUGIN_BUILD_LIBRARY_FILE,
        ),
        (
            "FORKLIFT_GUI_PLUGIN_LIBRARY_FILE",
            settings.FORKLIFT_GUI_PLUGIN_LIBRARY_FILE,
        ),
    ):
        if path.name != expected_plugin_library_name:
            raise GazeboSimulationError(
                f"settings.{name} must be named "
                f"{expected_plugin_library_name!r}."
            )

    if (
        settings.FORKLIFT_GUI_PLUGIN_LIBRARY_FILE.parent.resolve()
        != settings.FORKLIFT_GUI_PLUGIN_INSTALL_DIRECTORY.resolve()
    ):
        raise GazeboSimulationError(
            "FORKLIFT_GUI_PLUGIN_LIBRARY_FILE must be located directly in "
            "FORKLIFT_GUI_PLUGIN_INSTALL_DIRECTORY."
        )
    if settings.GAZEBO_RENDER_ENGINE not in {"ogre", "ogre2"}:
        raise GazeboSimulationError(
            "GAZEBO_RENDER_ENGINE must be either 'ogre' or 'ogre2'."
        )

    if not 0.0 < settings.FORKLIFT_BODY_FRONT_LENGTH_FRACTION < 1.0:
        raise GazeboSimulationError(
            "FORKLIFT_BODY_FRONT_LENGTH_FRACTION must be in (0, 1)."
        )
    if settings.FORKLIFT_TOTAL_MASS_KG <= 2.0 * settings.FORKLIFT_FORK_MASS_KG:
        raise GazeboSimulationError(
            "FORKLIFT_TOTAL_MASS_KG must exceed the combined fork mass."
        )
    if settings.FORKLIFT_FORK_CENTRE_SPACING_M <= settings.FORKLIFT_FORK_WIDTH_M:
        raise GazeboSimulationError(
            "FORKLIFT_FORK_CENTRE_SPACING_M must exceed the fork width."
        )
    if (
        settings.FORKLIFT_FORK_CENTRE_SPACING_M
        + settings.FORKLIFT_FORK_WIDTH_M
        > settings.FORKLIFT_BODY_WIDTH_M
    ):
        raise GazeboSimulationError(
            "The fork pair is wider than the forklift body."
        )
    if (
        settings.FORKLIFT_FORK_LOW_POSITION_Z_M
        - settings.FORKLIFT_FORK_THICKNESS_M / 2.0
        < 0.0
    ):
        raise GazeboSimulationError(
            "The lowest fork pose extends below the floor plane."
        )
    if (
        settings.FORKLIFT_FORK_MAX_POSITION_M
        <= settings.FORKLIFT_FORK_MIN_POSITION_M
    ):
        raise GazeboSimulationError(
            "FORKLIFT_FORK_MAX_POSITION_M must exceed the minimum."
        )
    if not (
        settings.FORKLIFT_FORK_MIN_POSITION_M
        <= settings.FORKLIFT_FORK_INITIAL_POSITION_M
        <= settings.FORKLIFT_FORK_MAX_POSITION_M
    ):
        raise GazeboSimulationError(
            "FORKLIFT_FORK_INITIAL_POSITION_M is outside its limits."
        )

    for name, color in (
        ("GAZEBO_AMBIENT_LIGHT_RGBA", settings.GAZEBO_AMBIENT_LIGHT_RGBA),
        ("GAZEBO_BACKGROUND_RGBA", settings.GAZEBO_BACKGROUND_RGBA),
        ("GAZEBO_LIGHT_DIFFUSE_RGBA", settings.GAZEBO_LIGHT_DIFFUSE_RGBA),
        ("GAZEBO_LIGHT_SPECULAR_RGBA", settings.GAZEBO_LIGHT_SPECULAR_RGBA),
        ("GAZEBO_GROUND_RGBA", settings.GAZEBO_GROUND_RGBA),
    ):
        if len(color) != 4 or any(
            not math.isfinite(value) or not 0.0 <= value <= 1.0
            for value in color
        ):
            raise GazeboSimulationError(
                f"settings.{name} must contain four values in [0, 1]."
            )

    if settings.GAZEBO_CAMERA_POSE is not None:
        if len(settings.GAZEBO_CAMERA_POSE) != 6 or any(
            not math.isfinite(value) for value in settings.GAZEBO_CAMERA_POSE
        ):
            raise GazeboSimulationError(
                "GAZEBO_CAMERA_POSE must be None or six finite numbers."
            )

    bindings = _key_bindings()
    codes = [binding.code for binding in bindings.values()]
    if len(set(codes)) != len(codes):
        raise GazeboSimulationError(
            "Forklift key bindings must use seven distinct keys."
        )

def _write_validated_xml(path: Path, text: str) -> None:
    temporary = path.with_name(path.name + ".tmp")
    try:
        temporary.write_text(text, encoding="utf-8")
        ElementTree.parse(temporary)
        temporary.replace(path)
    except ElementTree.ParseError as exc:
        temporary.unlink(missing_ok=True)
        raise GazeboSimulationError(
            f"Generated Gazebo world is not valid XML: {exc}"
        ) from exc
    except OSError as exc:
        temporary.unlink(missing_ok=True)
        raise GazeboSimulationError(
            f"Could not write Gazebo world {path}: {exc}"
        ) from exc


def _write_json_atomic(path: Path, data: dict[str, Any]) -> None:
    temporary = path.with_name(path.name + ".tmp")
    try:
        temporary.write_text(
            json.dumps(data, indent=2, sort_keys=False) + "\n",
            encoding="utf-8",
        )
        # Read back before replacement so a previously valid manifest is not
        # replaced by a partial or unserializable file.
        json.loads(temporary.read_text(encoding="utf-8"))
        temporary.replace(path)
    except (OSError, TypeError, ValueError, json.JSONDecodeError) as exc:
        temporary.unlink(missing_ok=True)
        raise GazeboSimulationError(
            f"Could not write Gazebo manifest {path}: {exc}"
        ) from exc

def _fmt(value: float) -> str:
    return f"{float(value):.12g}"


def _format_values(values: Iterable[float]) -> str:
    return " ".join(_fmt(value) for value in values)


def _bool_text(value: bool) -> str:
    return "true" if value else "false"
