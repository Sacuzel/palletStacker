# --- PHYSICAL CONSTRAINTS ---
PALLET_LENGTH = 1200.0
PALLET_WIDTH = 800.0
PALLET_BASE_HEIGHT = 144.0
PALLET_MAX_HEIGHT_TOTAL = 1800.0
USABLE_HEIGHT = PALLET_MAX_HEIGHT_TOTAL - PALLET_BASE_HEIGHT
PALLET_AREA = PALLET_LENGTH * PALLET_WIDTH

MIN_SUPPORT_RATIO = 0.70
PLATEAU_TOLERANCE = 5.0  # mm tolerance for considering surfaces to be the "same height"

# --- HEURISTIC WEIGHTS ---
# The scale of these weights mathematically guarantees the priority order:
WEIGHT_CENTER_START = 1000000.0  # Overrides everything for the 1st box
WEIGHT_ZDENSITY = 10000.0        # Priority 1: Force boxes to lie flat (Max ~1000)
WEIGHT_PLATEAU = 1000.0          # Priority 2: Cluster boxes of same height (Max 1000)
WEIGHT_MODULO = 100.0            # Priority 3: Align to minimize waste space (Max 200)
WEIGHT_GRAVITY = 0.5             # Tie-breaker: Prefer lower Z levels