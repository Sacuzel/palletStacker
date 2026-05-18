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
# Weights determine how the algorithm prioritizes candidate positions.
WEIGHT_CENTER_START = 1000000.0  # Massive bonus to guarantee first box is centered
WEIGHT_PLATEAU = 10.0            # Multiplier for plateau area (max ~9.6M)
WEIGHT_MODULO = 1000.0           # Penalty multiplier for waste gaps (max ~1M penalty)
WEIGHT_ZDENSITY = 1.0            # Tie-breaker for Z-density (usually evaluates to ~0.1 to 1.0)