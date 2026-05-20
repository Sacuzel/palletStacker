"""Global configuration and heuristic weights for the pallet loading algorithm."""

# --- PHYSICAL CONSTRAINTS ---
PALLET_LENGTH = 1200.0
PALLET_WIDTH = 800.0
PALLET_BASE_HEIGHT = 144.0
PALLET_MAX_HEIGHT_TOTAL = 1800.0
USABLE_HEIGHT = PALLET_MAX_HEIGHT_TOTAL - PALLET_BASE_HEIGHT
PALLET_AREA = PALLET_LENGTH * PALLET_WIDTH

# A box must have at least this percentage of its bottom resting on solid support
MIN_SUPPORT_RATIO = 0.70
# Allowed vertical variation (in mm) for two adjacent box tops to be considered a flat plateau
PLATEAU_TOLERANCE = 5.0  

# --- HEURISTIC WEIGHTS ---
# The scale of these weights mathematically guarantees the priority order.
# By spacing them out by orders of magnitude, we prevent lower-priority goals from overriding higher ones.

WEIGHT_CENTER_START = 1000000.0  # Overrides everything to ensure the very 1st box goes to the center
WEIGHT_ZDENSITY = 10000.0        # Priority 1: Force boxes to lie flat (Max bonus ~1000)
WEIGHT_PLATEAU = 1000.0          # Priority 2: Cluster boxes of same height (Max bonus ~1000)
WEIGHT_MODULO = 100.0            # Priority 3: Align to minimize waste space against pallet bounds (Max bonus 200)
WEIGHT_GRAVITY = 0.5             # Tie-breaker: Prefer lower Z levels (prevents floating/towering when tied)