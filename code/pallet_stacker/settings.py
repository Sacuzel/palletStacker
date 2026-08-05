"""Central application configuration for the pallet stacker.

Only user-configurable application values belong in this module. Domain model
instances still receive their values explicitly; this keeps ``Box`` and
``Pallet`` reusable and testable while giving the application one source of
configuration truth.

All dimensions are millimetres and all masses are kilograms unless stated
otherwise. Gazebo-specific dimensions are explicitly named with ``_M`` and use
metres, as expected by SDFormat.
"""

from __future__ import annotations

from pathlib import Path

from .box import Orientation


# ============================================================================
# PROJECT PATHS
# ============================================================================

# settings.py -> pallet_stacker package -> code -> palletStackerWS root
PROJECT_ROOT: Path = Path(__file__).resolve().parents[2]
INPUT_DIRECTORY: Path = PROJECT_ROOT / "input"
OUTPUT_DIRECTORY: Path = PROJECT_ROOT / "results"
PLOTLY_OUTPUT_FILE: Path = OUTPUT_DIRECTORY / "pallet_layout.html"

# Create common project directories when main.py starts.
CREATE_MISSING_PROJECT_DIRECTORIES: bool = True

# Set this to a JSON Path to bypass the file-selection dialog.
# Keep it as None to open the dialog whenever main.py starts.
INPUT_FILE_PATH: Path | None = None


# ============================================================================
# PROGRAM FLOW
# ============================================================================

# Loader selected by main.py. Future values may include, for example,
# "heuristic_v1", "layer", or "lookahead" after those loaders exist.
ACTIVE_LOADER: str = "naive"

# Output stages. Both can be enabled simultaneously.
GENERATE_PLOTLY_OUTPUT: bool = True
GENERATE_GAZEBO_OUTPUT: bool = True
PRINT_RUN_SUMMARY: bool = True


# ============================================================================
# INPUT JSON
# ============================================================================

JSON_FORMAT_VERSION: int = 1
JSON_DIMENSION_UNIT: str = "mm"
JSON_WEIGHT_UNIT: str = "kg"

# Orientations assigned to boxes created from the input JSON. These allow
# rotation around Z while keeping every carton upright.
DEFAULT_BOX_ALLOWED_ORIENTATIONS: tuple[Orientation, ...] = (
    Orientation.XYZ,
    Orientation.YXZ,
)

FILE_DIALOG_TITLE: str = "Select box data JSON file"
FILE_DIALOG_FILE_TYPES: tuple[tuple[str, str], ...] = (
    ("JSON files", "*.json"),
    ("All files", "*.*"),
)


# ============================================================================
# PALLET DEFINITION
# ============================================================================

PALLET_ID_PREFIX: str = "PALLET"
PALLET_ID_DIGITS: int = 3
PALLET_NAME: str = "EUR pallet"

# Pallet-local packing axes:
# X = pallet length, Y = pallet width, Z = height above loading surface.
PALLET_LENGTH_MM: float = 1200.0
PALLET_WIDTH_MM: float = 800.0
PALLET_BASE_HEIGHT_MM: float = 144.0

# Maximum box-stack height measured upward from the top loading surface.
PALLET_MAX_HEIGHT_MM: float = 1800.0

# Set to None to disable load-mass checking.
PALLET_MAX_LOAD_KG: float | None = 1000.0

# Numerical tolerance shared by all placement algorithms and pallet geometry
# checks. Algorithm-specific behavior belongs in the respective *Loader module.
PLACEMENT_TOLERANCE_MM: float = 1e-6


# ============================================================================
# PLOTLY VISUALIZATION
# ============================================================================

PLOTLY_TITLE: str = "Pallet loading result"
PLOTLY_SHOW_BOX_LABELS: bool = False
PLOTLY_OPEN_IN_BROWSER: bool = True

# "directory" stores one local plotly.min.js beside the HTML file. Use True to
# embed Plotly in every HTML file or "cdn" to load it from the internet.
PLOTLY_INCLUDE_PLOTLYJS: bool | str = "directory"

PLOTLY_PALLET_GAP_MM: float = 500.0
PLOTLY_EDGE_WIDTH_PX: float = 5.0
PLOTLY_EDGE_EXPAND_MM: float = 1.0


# ============================================================================
# GAZEBO ASSET PATHS
# ============================================================================

GAZEBO_DIRECTORY: Path = PROJECT_ROOT / "gazebo"
GAZEBO_MODELS_DIRECTORY: Path = GAZEBO_DIRECTORY / "models"
GAZEBO_BRIDGE_DIRECTORY: Path = GAZEBO_DIRECTORY / "bridge"
GAZEBO_WORLDS_DIRECTORY: Path = GAZEBO_DIRECTORY / "worlds"
GAZEBO_PLUGINS_DIRECTORY: Path = GAZEBO_DIRECTORY / "plugins"

PALLET_GAZEBO_MODEL_NAME: str = "euro_pallet"
PALLET_GAZEBO_MODEL_DIRECTORY: Path = (
    GAZEBO_MODELS_DIRECTORY / PALLET_GAZEBO_MODEL_NAME
)

FORKLIFT_MODEL_NAME: str = "simple_forklift"
FORKLIFT_MODEL_DIRECTORY: Path = GAZEBO_MODELS_DIRECTORY / FORKLIFT_MODEL_NAME

# These ROS files remain available for optional ROS-based testing. The generated
# pallet world does not need ROS 2, ros_gz_bridge, or forkliftTeleop.py.
FORKLIFT_BRIDGE_CONFIG_FILE: Path = (
    GAZEBO_BRIDGE_DIRECTORY / "forklift_bridge.yaml"
)
FORKLIFT_TEST_WORLD_FILE: Path = GAZEBO_WORLDS_DIRECTORY / "forklift_test.sdf"

GAZEBO_WORLD_NAME: str = "pallet_stacker_world"
GAZEBO_WORLD_FILE: Path = GAZEBO_WORLDS_DIRECTORY / "pallet_stacker_world.sdf"
GAZEBO_MANIFEST_FILE: Path = (
    GAZEBO_WORLDS_DIRECTORY / "pallet_stacker_world_manifest.json"
)

# Integrated Gazebo GUI plugin used for direct forklift keyboard control.
#
# Gazebo GUI uses the plugin ``filename`` both to locate the shared library and
# to locate the embedded QML resource. Consequently the generated world must use
# the stable base name ``ForkliftTeleop`` rather than an absolute .so path.
# ``main.py`` builds the library inside the project and then installs / updates a
# user-local runtime copy in Gazebo GUI's standard plugin directory. This needs
# no sudo, no GZ_GUI_PLUGIN_PATH, and no additional terminal when the world runs.
FORKLIFT_GUI_PLUGIN_NAME: str = "ForkliftTeleop"
FORKLIFT_GUI_PLUGIN_SOURCE_DIRECTORY: Path = (
    GAZEBO_PLUGINS_DIRECTORY / "forklift_teleop"
)
FORKLIFT_GUI_PLUGIN_BUILD_DIRECTORY: Path = (
    FORKLIFT_GUI_PLUGIN_SOURCE_DIRECTORY / "build"
)
FORKLIFT_GUI_PLUGIN_BUILD_LIBRARY_FILE: Path = (
    FORKLIFT_GUI_PLUGIN_BUILD_DIRECTORY / f"lib{FORKLIFT_GUI_PLUGIN_NAME}.so"
)
FORKLIFT_GUI_PLUGIN_INSTALL_DIRECTORY: Path = (
    Path.home() / ".gz" / "gui" / "plugins"
)
FORKLIFT_GUI_PLUGIN_LIBRARY_FILE: Path = (
    FORKLIFT_GUI_PLUGIN_INSTALL_DIRECTORY / f"lib{FORKLIFT_GUI_PLUGIN_NAME}.so"
)


# ============================================================================
# GAZEBO WORLD GENERATION
# ============================================================================

# main.py generates the world but does not launch Gazebo. The generated SDF
# contains the GUI control plugin and inlined models, so the normal workflow is:
#   python code/main.py
#   gz sim gazebo/worlds/pallet_stacker_world.sdf
GAZEBO_WRITE_MANIFEST: bool = True

# Start physics automatically even when the world is launched with plain
# ``gz sim <world>`` rather than ``gz sim -r <world>``. The integrated GUI
# plugin retries the world's control service while the server is starting.
GAZEBO_AUTO_START_SIMULATION: bool = True
GAZEBO_AUTO_START_RETRY_INTERVAL_MS: int = 250
GAZEBO_AUTO_START_REQUEST_TIMEOUT_MS: int = 100
GAZEBO_AUTO_START_MAX_ATTEMPTS: int = 60

# Regenerate the forklift SDF from these settings before creating the world.
GAZEBO_REGENERATE_FORKLIFT_ASSETS: bool = True

# Inline the pallet and forklift model XML into the generated world. This avoids
# requiring GZ_SIM_RESOURCE_PATH when the world is launched manually.
GAZEBO_INLINE_LOCAL_MODELS: bool = True

# Validate required local model files and check that the pallet collision
# envelope matches PALLET_LENGTH_MM / WIDTH_MM / BASE_HEIGHT_MM.
GAZEBO_VALIDATE_LOCAL_MODELS: bool = True

# Build the custom GUI plugin automatically when main.py generates a Gazebo
# world. A rebuild occurs only when source files or build settings change.
GAZEBO_ENABLE_INTEGRATED_FORKLIFT_CONTROLS: bool = True
GAZEBO_BUILD_FORKLIFT_GUI_PLUGIN: bool = True
GAZEBO_FORCE_REBUILD_FORKLIFT_GUI_PLUGIN: bool = False
GAZEBO_GUI_PLUGIN_BUILD_TYPE: str = "Release"
GAZEBO_GUI_PLUGIN_PARALLEL_JOBS: int = 0  # 0 lets CMake choose.
GAZEBO_CMAKE_EXECUTABLE: str = "cmake"

# Harmonic normally uses Ogre 2. Set to "ogre" only for Ogre 1 fallback.
GAZEBO_RENDER_ENGINE: str = "ogre2"
GAZEBO_DISABLE_SHADOWS: bool = True
GAZEBO_SHOW_GRID: bool = True
GAZEBO_AMBIENT_LIGHT_RGBA: tuple[float, float, float, float] = (
    0.65,
    0.65,
    0.65,
    1.0,
)
GAZEBO_BACKGROUND_RGBA: tuple[float, float, float, float] = (
    0.78,
    0.80,
    0.84,
    1.0,
)

# Directional-light settings. Shadows are still controlled separately through
# GAZEBO_DISABLE_SHADOWS, so these values only govern illumination and colour.
GAZEBO_LIGHT_DIRECTION: tuple[float, float, float] = (-0.5, 0.2, -1.0)
GAZEBO_LIGHT_DIFFUSE_RGBA: tuple[float, float, float, float] = (
    0.90,
    0.90,
    0.90,
    1.0,
)
GAZEBO_LIGHT_SPECULAR_RGBA: tuple[float, float, float, float] = (
    0.15,
    0.15,
    0.15,
    1.0,
)

# Use None for an automatically calculated overview camera. Otherwise provide
# (x, y, z, roll, pitch, yaw), all in metres/radians.
GAZEBO_CAMERA_POSE: tuple[float, float, float, float, float, float] | None = None
GAZEBO_CAMERA_NEAR_CLIP_M: float = 0.10
GAZEBO_CAMERA_FAR_CLIP_M: float = 500.0

# No simulation end time is written. Gazebo runs until the user closes it.
GAZEBO_PHYSICS_MAX_STEP_SIZE_S: float = 0.005
GAZEBO_REAL_TIME_FACTOR: float = 1.0
GAZEBO_GRAVITY_MPS2: tuple[float, float, float] = (0.0, 0.0, -9.81)
GAZEBO_MAX_CONTACTS_PER_COLLISION: int = 4

# Ground plane sizing is automatic. These values set its minimum size and clear
# margin around all generated pallets and the forklift.
GAZEBO_GROUND_MIN_SIZE_M: float = 20.0
GAZEBO_GROUND_MARGIN_M: float = 4.0
GAZEBO_GROUND_FRICTION: float = 0.80
GAZEBO_GROUND_RESTITUTION: float = 0.0
GAZEBO_GROUND_CONTACT_STIFFNESS: float = 1_000_000.0
GAZEBO_GROUND_CONTACT_DAMPING: float = 100.0
GAZEBO_GROUND_RGBA: tuple[float, float, float, float] = (
    0.58,
    0.60,
    0.62,
    1.0,
)

# Pallets are laid out in one row along world +X, matching the Plotly view.
# The first pallet is centred at this world position.
GAZEBO_FIRST_PALLET_X_M: float = 0.0
GAZEBO_FIRST_PALLET_Y_M: float = 0.0
GAZEBO_PALLET_GAP_M: float = 0.50

# Small initial clearances avoid starting primitive collisions in penetration.
# Pallets settle onto the ground; the full box stack is translated upward by
# the same box clearance, preserving all box-to-box contacts.
GAZEBO_PALLET_SPAWN_CLEARANCE_M: float = 0.001
GAZEBO_BOX_SPAWN_CLEARANCE_M: float = 0.001

# Pallet mass/contact values are applied to the inlined pallet model. The
# source model remains a readable geometric template.
GAZEBO_PALLET_MASS_KG: float = 25.0
GAZEBO_PALLET_FRICTION: float = 0.65
GAZEBO_PALLET_RESTITUTION: float = 0.0
GAZEBO_PALLET_CONTACT_STIFFNESS: float = 1_000_000.0
GAZEBO_PALLET_CONTACT_DAMPING: float = 100.0
GAZEBO_PALLET_AUTO_DISABLE: bool = True

# Box contact and material behavior. Zero-mass JSON entries receive the minimum
# simulation mass because dynamic SDFormat links require positive mass.
GAZEBO_BOX_MIN_MASS_KG: float = 0.05
GAZEBO_BOX_FRICTION: float = 0.55
GAZEBO_BOX_RESTITUTION: float = 0.0
GAZEBO_BOX_CONTACT_STIFFNESS: float = 1_000_000.0
GAZEBO_BOX_CONTACT_DAMPING: float = 100.0
GAZEBO_CONTACT_MAX_CORRECTING_VELOCITY_MPS: float = 1.0
GAZEBO_CONTACT_MIN_DEPTH_M: float = 0.0005
GAZEBO_BOX_AUTO_DISABLE: bool = True

# The forklift approaches the negative-X end of the first pallet. Its +X forks
# point along the pallet's longitudinal channels. This distance is measured from
# the fork tips to the nearest pallet edge, not from the forklift body.
GAZEBO_FORKLIFT_FORK_TIP_CLEARANCE_M: float = 3.0
GAZEBO_FORKLIFT_SPAWN_CLEARANCE_M: float = 0.001


# ============================================================================
# GAZEBO FORKLIFT MODEL AND TELEOPERATION
# ============================================================================

# Gazebo Transport topics. The custom GUI plugin publishes directly to these
# topics; no ROS bridge or additional terminal is required.
FORKLIFT_CMD_VEL_TOPIC: str = "/forklift/cmd_vel"
FORKLIFT_FORK_POSITION_TOPIC: str = "/forklift/fork_position"

# Simplified geometry based on a compact STILL RX 20-16 electric forklift.
# The body is split into two rigid visual/collision boxes inside one body link.
# X points forward toward the forks, Y points left, and Z points upward.
FORKLIFT_BODY_LENGTH_M: float = 1.944
FORKLIFT_BODY_WIDTH_M: float = 1.099
FORKLIFT_BODY_HEIGHT_M: float = 2.035
FORKLIFT_BODY_FRONT_LENGTH_FRACTION: float = 0.5

# The two uniform, non-tapered fork bars use the reference fork dimensions.
FORKLIFT_FORK_LENGTH_M: float = 0.800
FORKLIFT_FORK_WIDTH_M: float = 0.080
FORKLIFT_FORK_THICKNESS_M: float = 0.040
FORKLIFT_FORK_CENTRE_SPACING_M: float = 0.450

# At the lowest position, each fork is centred in the simplified EUR pallet's
# fork channel. The position-controller command is displacement above this pose.
FORKLIFT_FORK_LOW_POSITION_Z_M: float = 0.050
FORKLIFT_FORK_MIN_POSITION_M: float = 0.0
FORKLIFT_FORK_MAX_POSITION_M: float = 1.8
FORKLIFT_FORK_INITIAL_POSITION_M: float = 0.0

# Mass model. Total mass matches the approximate service mass of the selected
# 1.6 t reference truck. The rear body half is intentionally twice as heavy as
# the front half to shift the centre of mass away from the forks.
FORKLIFT_TOTAL_MASS_KG: float = 3057.0
FORKLIFT_FORK_MASS_KG: float = 25.0
FORKLIFT_REAR_TO_FRONT_BODY_MASS_RATIO: float = 2.0

# Contact parameters. A flat sliding contact cannot exactly reproduce rolling
# tyres; this low coefficient is a rolling-resistance approximation.
FORKLIFT_BODY_FLOOR_FRICTION: float = 0.02
FORKLIFT_FORK_CONTACT_FRICTION: float = 0.50
FORKLIFT_CONTACT_STIFFNESS: float = 1_000_000.0
FORKLIFT_CONTACT_DAMPING: float = 100.0

# Operator limits. The travel limit is deliberately below the reference truck's
# rated maximum so keyboard operation remains controllable in a small scene.
FORKLIFT_MAX_LINEAR_VELOCITY_MPS: float = 3.0
FORKLIFT_MAX_TURN_VELOCITY_RAD_S: float = 0.75
FORKLIFT_MAX_LINEAR_ACCELERATION_MPS2: float = 1.0
FORKLIFT_MAX_TURN_ACCELERATION_RAD_S2: float = 1.5
FORKLIFT_MAX_FORK_VELOCITY_MPS: float = 0.50
FORKLIFT_FORK_JOINT_MAX_EFFORT_N: float = 12_000.0
FORKLIFT_FORK_JOINT_DAMPING: float = 50.0
FORKLIFT_FORK_JOINT_FRICTION: float = 10.0

# Keys understood by the custom Gazebo GUI plugin. Supported names are one
# printable character plus UP, DOWN, LEFT, RIGHT, SPACE, ESCAPE, PAGEUP and
# PAGEDOWN. W/S/A/D and the arrow keys are the normal defaults.
FORKLIFT_KEY_FORWARD: str = "W"
FORKLIFT_KEY_REVERSE: str = "S"
FORKLIFT_KEY_TURN_LEFT: str = "A"
FORKLIFT_KEY_TURN_RIGHT: str = "D"
FORKLIFT_KEY_LIFT: str = "UP"
FORKLIFT_KEY_LOWER: str = "DOWN"
FORKLIFT_KEY_STOP: str = "SPACE"

# The GUI plugin uses actual key-press and key-release events. Chassis commands
# ramp toward zero when the corresponding key is released. The fork position
# target stops changing and remains held at its last value.
FORKLIFT_TELEOP_RATE_HZ: float = 50.0
FORKLIFT_GUI_PANEL_WIDTH_PX: int = 360
FORKLIFT_GUI_PANEL_HEIGHT_PX: int = 245

# Settings used only by the optional ROS-terminal forkliftTeleop.py module.
FORKLIFT_KEY_HOLD_TIMEOUT_S: float = 0.18
FORKLIFT_FORK_KEY_STEP_M: float = 0.025
