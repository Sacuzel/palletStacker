"""Central application configuration for the pallet stacker.

Only user-configurable application values belong in this module. Domain model
instances still receive their values explicitly; this keeps ``Box`` and
``Pallet`` reusable and testable while giving the application one source of
configuration truth.

All dimensions are millimetres and all masses are kilograms unless stated
otherwise.
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

# Create the input and output directories when main.py starts.
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

# Output stages. Gazebo remains disabled until its exporter module exists.
GENERATE_PLOTLY_OUTPUT: bool = True
GENERATE_GAZEBO_OUTPUT: bool = False
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
# GAZEBO FORKLIFT MODEL AND TELEOPERATION
# ============================================================================

# Gazebo asset locations. The forklift model generator creates these folders.
GAZEBO_DIRECTORY: Path = PROJECT_ROOT / "gazebo"
GAZEBO_MODELS_DIRECTORY: Path = GAZEBO_DIRECTORY / "models"
GAZEBO_BRIDGE_DIRECTORY: Path = GAZEBO_DIRECTORY / "bridge"
GAZEBO_WORLDS_DIRECTORY: Path = GAZEBO_DIRECTORY / "worlds"

FORKLIFT_MODEL_NAME: str = "simple_forklift"
FORKLIFT_MODEL_DIRECTORY: Path = GAZEBO_MODELS_DIRECTORY / FORKLIFT_MODEL_NAME
FORKLIFT_BRIDGE_CONFIG_FILE: Path = (
    GAZEBO_BRIDGE_DIRECTORY / "forklift_bridge.yaml"
)
FORKLIFT_TEST_WORLD_FILE: Path = GAZEBO_WORLDS_DIRECTORY / "forklift_test.sdf"

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

# The two uniform, non-tapered fork bars use the RX 20 standard fork dimensions.
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
# tyres; this low coefficient is a deliberate rolling-resistance approximation.
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
