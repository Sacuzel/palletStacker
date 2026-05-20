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

# --- NEW FINAL OPTIMIZATION WEIGHTS ---
WEIGHT_INTERLOCK = 10.0          # Priority 4: Reward bridging across multiple boxes
WEIGHT_ADJACENCY = 5.0           # Priority 5: Reward touching adjacent boxes for lateral stability
WEIGHT_GRAVITY = 0.5             # Tie-breaker: Prefer lower Z levels