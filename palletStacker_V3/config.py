"""=====================================================================
CONFIG  (role: parameter list / "GVL" - Global Variable List)
=====================================================================
All tunable parameters of the pallet-packing program live here.
No logic in this file, only constants. Every other module reads
from this file, nothing writes to it at runtime.

NOTE: gazebo_exporter.py imports PALLET_LENGTH, PALLET_WIDTH,
PALLET_BASE_HEIGHT, PALLET_WEIGHT_KG and SIM_FRICTION_COEFF from
this module, so do not rename those symbols.

All dimensions are in millimetres, all weights in kilograms,
unless the name says otherwise.
====================================================================="""

# ---------------------------------------------------------------
# PALLET GEOMETRY
# ---------------------------------------------------------------
# Standard EUR/EPAL pallet footprint: 1200 mm x 800 mm.
PALLET_LENGTH: float = 800.0          # X direction [mm]
PALLET_WIDTH: float = 1200.0            # Y direction [mm]

# Maximum allowed height of the box stack ON TOP of the pallet deck
# (the wooden pallet itself is NOT included in this number).
# 1800 mm is a common warehouse racking / trailer limit.
MAX_STACK_HEIGHT: float = 1800.0       # [mm]

# Physical wooden pallet, only needed by the Gazebo exporter so the
# simulation has a realistic base under the boxes.
PALLET_BASE_HEIGHT: float = 144.0      # EUR pallet height [mm]
PALLET_WEIGHT_KG: float = 25.0         # EUR pallet mass  [kg]

# Optional payload limit of one pallet. EUR pallets are rated for
# ~1500 kg in motion. Set to None to disable the check.
MAX_PALLET_PAYLOAD_KG: float = None # = 1500.0

# ---------------------------------------------------------------
# DISCRETISATION (heightmap / feasibility map grid)
# ---------------------------------------------------------------
# The pallet top surface is discretised into square cells of this
# size. The stability paper (Gao et al. 2025) does the same thing
# with a camera heightmap; here we maintain the map analytically.
# All box dimensions in the sample data are multiples of 5 mm, so
# 5 mm gives an exact representation. Smaller = more precise but
# slower (cell count grows quadratically).
GRID_RESOLUTION_MM: float = 10.0        # [mm per cell]

# ---------------------------------------------------------------
# STABILITY VALIDATION (LBCP method, Alg. 1 of the paper)
# ---------------------------------------------------------------
# Relative uncertainty of the centre of gravity (delta_CoG in
# Eq. 1 of the paper). The CoG of a box is assumed to be at its
# geometric centre +/- COG_TOLERANCE * dimension along each axis.
# The task statement says CoG is exactly at the geometric centre,
# so 0.0 is correct; raise it (e.g. 0.1) for sloshing/shifting
# loads to make the validator more conservative.
COG_TOLERANCE: float = 0.0             # [-] 0.0 .. 0.5

# Minimum fraction of the box footprint that must rest on
# load-bearable contact cells. The pure LBCP criterion only needs
# the CoG inside the support polygon; a minimum support-area ratio
# is a common extra industrial constraint (cf. Bischoff & Ratcliff
# style full-support rules, Ramos et al. 2016) that avoids fragile
# "bridging on two edges" placements before the physics sim.
# 0.0 disables the check and uses the paper's criterion only.
# NOTE: 0.50 was found empirically to be the sweet spot on the
# grocery dataset - it forbids poorly supported placements early,
# which forces compact towers and REDUCED the pallet count from
# 4 to 3 (overall utilisation 54.9 % -> 73.1 %) while making the
# stacks physically safer at the same time.
MIN_SUPPORT_RATIO: float = 0.0        # [-] 0.0 .. 1.0

# ---------------------------------------------------------------
# PACKING ALGORITHM BEHAVIOUR
# ---------------------------------------------------------------
# Allow rotating boxes 90 degrees around the vertical axis
# (length <-> width swap).
ALLOW_YAW_ROTATION: bool = True

# Allow tipping boxes onto their side/end, i.e. any of the box's
# three dimensions may point upward. Together with yaw rotation
# this yields all 6 axis-aligned orientations of a cuboid.
# Set to False for "this side up" goods (bottles, eggs, open
# crates), which restricts the search to the 2 upright
# orientations. Note: the CoG is assumed to stay at the geometric
# centre in EVERY orientation (per the task assumptions) - for
# real part-filled cases whose contents settle, tipping would make
# that assumption less accurate.
ALLOW_TIPPED_ORIENTATIONS: bool = True

# If True, an incoming box is offered to ALL pallets that are still
# open, in the order they were opened ("first fit" over open
# pallets). If False, only the newest pallet is considered and
# older pallets are treated as closed the moment a new one opens -
# this matches a single-station manual operator most closely, at
# the cost of some utilisation.
FIRST_FIT_OPEN_PALLETS: bool = False

# Scoring weights of the placement heuristic (see algorithm.py).
# The score is minimised. Units are chosen so that 1.0 of weight
# roughly equals 1 mm of height, which makes tuning intuitive.
W_TOP_HEIGHT: float = 1.0      # heightmap-minimisation term
W_SUPPORT: float = 200.0       # reward for high support ratio
W_SNUGNESS: float = 100.0      # reward for touching walls/boxes
W_SAME_SKU_ALIGN: float = 150.0  # reward for perfect same-SKU column

# ---------------------------------------------------------------
# GAZEBO PHYSICS
# ---------------------------------------------------------------
# Larger max step = faster but less accurate simulation.
# Lower real_time_update_rate = less CPU load.
# real_time_update_rate = 0 usually means "run as fast as possible"
# depending on Gazebo / physics backend.
GAZEBO_MAX_STEP_SIZE: float = 0.5
GAZEBO_REAL_TIME_FACTOR: float = 1.0
GAZEBO_REAL_TIME_UPDATE_RATE: int = 0
GAZEBO_MAX_CONTACTS: int = 30

# Friction coefficient used by the Gazebo exporter for box-box,
# box-pallet and box-floor contacts. ~0.4-0.5 is typical for
# cardboard on cardboard.
SIM_FRICTION_COEFF: float = 0.55

# How plotly.js is bundled into the output HTML:
#   "cdn"       - small file, needs internet when opened
#   True        - plotly.js embedded, ~3.5 MB, works offline
#   "directory" - writes plotly.min.js next to the HTML file
PLOTLY_JS_MODE: bool | str = "cdn"

# Output file locations (relative to the working directory).
OUTPUT_HTML_PATH: str = "pallet_layout.html"
GAZEBO_OUTPUT_DIR: str = "gazebo_runs"
GAZEBO_RUN_NAME: str = "latest"