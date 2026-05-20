# --- PHYSICAL CONSTRAINTS ---
PALLET_LENGTH = 1200.0
PALLET_WIDTH = 800.0
PALLET_BASE_HEIGHT = 144.0
PALLET_MAX_HEIGHT_TOTAL = 1800.0
USABLE_HEIGHT = PALLET_MAX_HEIGHT_TOTAL - PALLET_BASE_HEIGHT
PALLET_AREA = PALLET_LENGTH * PALLET_WIDTH

MIN_SUPPORT_RATIO = 0.70
PLATEAU_TOLERANCE = 5.0  

# --- ADVANCED HEURISTIC SETTINGS ---
# A supporting box must cover at least 15% of the candidate's bottom to count as a "bridge" support
MIN_INTERLOCK_COVERAGE_RATIO = 0.15 
# Distance in mm to consider a box as "touching" a wall for lateral support
ADJACENCY_TOLERANCE = 5.0           

# --- HEURISTIC WEIGHTS ---
WEIGHT_CENTER_START = 1000000.0  
WEIGHT_ZDENSITY = 10000.0        # Priority 1: Force boxes to lie flat 
WEIGHT_PLATEAU = 1000.0          # Priority 2: Cluster boxes of same height 
WEIGHT_MODULO = 100.0            # Priority 3: Align to minimize waste space 

# --- FINAL OPTIMIZATION WEIGHTS ---
WEIGHT_INTERLOCK = 10.0          # Priority 4: Reward bridging across multiple boxes
WEIGHT_ADJACENCY = 5.0           # Priority 5: Reward touching adjacent boxes for lateral stability
WEIGHT_GRAVITY = 0.5             # Tie-breaker: Prefer lower Z levels

# --- PHYSICS SIMULATION (PALLET JACK) ---
RUN_SIMULATION = True          # Toggle simulation on/off
SIM_GRAVITY = 9.81             # m/s^2
SIM_FRICTION_COEFF = 0.35      # Static friction coefficient (Cardboard on cardboard/wood)
SIM_SPEED_MS = 1.5             # Fast walking/cornering speed (approx 5.4 km/h)
SIM_TURN_RADIUS_M = 1.0        # Tight corner radius for manual pallet jack
SIM_BRAKE_DECEL = 1.5          # Deceleration during hard/emergency stop (m/s^2)
PALLET_WEIGHT_KG = 25.0        # Standard wooden EUR pallet weight
SIM_JACK_FORK_WIDTH = 540.0    # Standard pallet jack fork width (much narrower than the 800mm pallet!)