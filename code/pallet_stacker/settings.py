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

PALLET_GAZEBO_MODEL_NAME: str = "euro_pallet"
PALLET_GAZEBO_MODEL_DIRECTORY: Path = (
    GAZEBO_MODELS_DIRECTORY / PALLET_GAZEBO_MODEL_NAME
)

FORKLIFT_MODEL_NAME: str = "simple_forklift"
FORKLIFT_MODEL_DIRECTORY: Path = GAZEBO_MODELS_DIRECTORY / FORKLIFT_MODEL_NAME
FORKLIFT_BRIDGE_CONFIG_FILE: Path = (
    GAZEBO_BRIDGE_DIRECTORY / "forklift_bridge.yaml"
)
FORKLIFT_TEST_WORLD_FILE: Path = GAZEBO_WORLDS_DIRECTORY / "forklift_test.sdf"

GAZEBO_WORLD_NAME: str = "pallet_stacker_world"
GAZEBO_WORLD_FILE: Path = GAZEBO_WORLDS_DIRECTORY / "pallet_stacker_world.sdf"


# ============================================================================
# GAZEBO WORLD AND EXECUTION
# ============================================================================

# When enabled, main.py writes the world and starts Gazebo automatically.
# Set GAZEBO_LAUNCH_SIMULATION to False to generate only the SDF world file.
GAZEBO_LAUNCH_SIMULATION: bool = True
GAZEBO_WAIT_FOR_SIMULATION_EXIT: bool = False
GAZEBO_START_PAUSED: bool = False
GAZEBO_EXECUTABLE: str = "gz"
GAZEBO_VERBOSITY: int = 3

# Keep the generated forklift SDF and ROS bridge synchronized with the values in
# this file. Disable only when intentionally hand-editing those generated files.
GAZEBO_REGENERATE_FORKLIFT_ASSETS: bool = True

# Validate that required local model files exist and that the pallet model's
# collision envelope matches PALLET_LENGTH_MM / WIDTH_MM / BASE_HEIGHT_MM.
GAZEBO_VALIDATE_LOCAL_MODELS: bool = True

# Harmonic normally uses Ogre 2. Set to "ogre" only for Ogre 1 fallback.
GAZEBO_RENDER_ENGINE: str = "ogre2"
GAZEBO_DISABLE_SHADOWS: bool = True
GAZEBO_SHOW_GRID: bool = True
GAZEBO_AMBIENT_LIGHT_RGBA: tuple[float, float, float, float] = (0.65, 0.65, 0.65, 1.0)
GAZEBO_BACKGROUND_RGBA: tuple[float, float, float, float] = (0.78, 0.80, 0.84, 1.0)

# Use None for an automatically calculated overview camera. Otherwise provide
# (x, y, z, roll, pitch, yaw), all in metres/radians.
GAZEBO_CAMERA_POSE: tuple[float, float, float, float, float, float] | None = None
GAZEBO_CAMERA_NEAR_CLIP_M: float = 0.10
GAZEBO_CAMERA_FAR_CLIP_M: float = 500.0

# Unlimited simulation duration is achieved by not defining an end time. Gazebo
# continues until the user closes it. These values only govern time stepping.
GAZEBO_PHYSICS_MAX_STEP_SIZE_S: float = 0.001
GAZEBO_REAL_TIME_FACTOR: float = 1.0
GAZEBO_GRAVITY_MPS2: tuple[float, float, float] = (0.0, 0.0, -9.81)

# Ground plane sizing is automatic. These values set its minimum size and clear
# margin around all generated pallets and the forklift.
GAZEBO_GROUND_MIN_SIZE_M: float = 20.0
GAZEBO_GROUND_MARGIN_M: float = 4.0
GAZEBO_GROUND_FRICTION: float = 0.80
GAZEBO_GROUND_RGBA: tuple[float, float, float, float] = (0.58, 0.60, 0.62, 1.0)

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
# therefore point directly into the pallet channels. This distance is measured
# from the fork tips to the nearest pallet edge, not from the forklift centre.
GAZEBO_FORKLIFT_FORK_TIP_CLEARANCE_M: float = 3.0
GAZEBO_FORKLIFT_SPAWN_CLEARANCE_M: float = 0.001


# ============================================================================
# GAZEBO FORKLIFT MODEL AND TELEOPERATION
# ============================================================================

# ROS 2 / Gazebo Transport topics used by the teleop node and ros_gz_bridge.
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
FORKLIFT_FORK_CENTRE_SPACING_M: float = 0.560

# At the lowest position, each fork is centred in the simplified EUR pallet's
# fork channel. The position controller command is displacement above this pose.
FORKLIFT_FORK_LOW_POSITION_Z_M: float = 0.072
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
FORKLIFT_FORK_CONTACT_FRICTION: float = 0.40
FORKLIFT_CONTACT_STIFFNESS: float = 1_000_000.0
FORKLIFT_CONTACT_DAMPING: float = 100.0

# Operator limits. The travel limit is deliberately below the reference truck's
# rated maximum so keyboard operation remains controllable in a small scene.
FORKLIFT_MAX_LINEAR_VELOCITY_MPS: float = 2.0
FORKLIFT_MAX_TURN_VELOCITY_RAD_S: float = 0.75
FORKLIFT_MAX_LINEAR_ACCELERATION_MPS2: float = 1.0
FORKLIFT_MAX_TURN_ACCELERATION_RAD_S2: float = 1.5
FORKLIFT_MAX_FORK_VELOCITY_MPS: float = 0.50
FORKLIFT_FORK_JOINT_MAX_EFFORT_N: float = 12_000.0
FORKLIFT_FORK_JOINT_DAMPING: float = 50.0
FORKLIFT_FORK_JOINT_FRICTION: float = 10.0

# Keyboard teleoperation behavior. Commands return to zero when key-repeat
# messages stop arriving, while the last fork position target remains active.
FORKLIFT_TELEOP_RATE_HZ: float = 20.0
FORKLIFT_KEY_HOLD_TIMEOUT_S: float = 0.18
FORKLIFT_FORK_KEY_STEP_M: float = 0.025
