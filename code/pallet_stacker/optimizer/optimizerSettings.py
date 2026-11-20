"""Settings of the loader optimizer (single point of change).

Every tunable value of the optimizer package lives here. Two groups of values
are deliberately NOT here, because they already have a home:

* Pallet geometry, height limit, load limit and the placement tolerance come
  from pallet_stacker/settings.py.
* The inner loader's own configuration (stability constraint, number of open
  pallets, tolerances, tie seed and the baseline rule combination) is read
  from the ALGORITHM SETTINGS block of emsLoader.py. The optimizer wraps
  "emsLoader as currently configured"; these are physical inputs or the
  baseline, never search knobs.

Units: millimetres, kilograms, seconds, m/s^2.
"""

from __future__ import annotations


# ============================================================================
# RUN CONTROL
# ============================================================================

# Print progress and the result summary to the console.
VERBOSE: bool = True

# Master seed of the optimizer (BRKGA). The same seed, order and settings
# give the same result when the run stops by the window criterion or the
# generation limit. A stop by TIME_LIMIT_S depends on machine speed.
RANDOM_SEED: int = 0

# Wall-clock budget for the whole optimizer run [s]. None = no time limit.
# The limit is checked between sweep batches and between BRKGA generations,
# so a run can overshoot by one batch or generation.
TIME_LIMIT_S: float | None = 300.0

# Worker processes for trial evaluation. 0 = automatic (CPU cores - 1),
# 1 = no multiprocessing. Results do not depend on this value.
N_WORKERS: int = 0

# Run emsLoader once on copies of the boxes and compare it with the decoder's
# baseline. A mismatch means emsDecoder.py is out of sync with emsLoader.py.
# Only a warning is printed; the check is outside the time budget.
VERIFY_DECODER_AGAINST_EMS_LOADER: bool = True


# ============================================================================
# STAGE 1 - RULE SWEEP
# ============================================================================

RUN_RULE_SWEEP: bool = True

# Rule values to combine (Tables 1 and 2 of the paper, see emsLoader.py).
# The baseline combination from emsLoader is always included.
SWEEP_BIN_RULES: tuple[str, ...] = ("first_fit", "best_fit", "worst_fit", "all_bins")
SWEEP_SPACE_RULES: tuple[int, ...] = (1, 2, 3, 4, 5, 6, 7, 8)
SWEEP_ORIENTATION_RULES: tuple[int, ...] = (1, 2, 3, 4, 5)

# N_EMS_CANDIDATES values to try for orientation rules 4 and 5.
# Empty = keep the value configured in emsLoader.
SWEEP_N_EMS_CANDIDATES: tuple[int, ...] = ()

# Rows of the ranked sweep table printed to the console.
SWEEP_REPORT_TOP: int = 10


# ============================================================================
# STAGE 2 - BRKGA
# ============================================================================

RUN_BRKGA: bool = True

# The best rule combinations of Stage 1 seed the first generation. One gene
# of the chromosome selects which of them a solution is decoded with.
BRKGA_SEED_CONFIGS: int = 5

# Population and the three BRKGA parameters (Goncalves & Resende 2012;
# usual ranges: elite 0.10-0.25, mutants 0.10-0.30, inheritance 0.5-0.8).
BRKGA_POPULATION: int = 40
BRKGA_ELITE_FRACTION: float = 0.20
BRKGA_MUTANT_FRACTION: float = 0.15
BRKGA_ELITE_INHERITANCE: float = 0.70

# Safety stop.
BRKGA_MAX_GENERATIONS: int = 500

# Window stopping criterion: stop when the best solution is not meaningfully
# better than the best solution this many generations ago. "Meaningfully"
# is defined by DENSITY_RESOLUTION and COG_RESOLUTION_MM below.
STALL_WINDOW_GENERATIONS: int = 25


# ============================================================================
# DECODER STEERING (how chromosome keys change emsLoader's decisions)
# ============================================================================

# Each box has three keys in [0, 1): pallet, space, orientation. A key below
# (1 - DEVIATION_RATE) keeps emsLoader's own decision; a key above it picks
# an alternative. This is the expected share of deviating decisions in a
# random chromosome, i.e. the mutation strength.
DEVIATION_RATE: float = 0.03

# A deviating space key skips 1 .. SPACE_ALTERNATIVES feasible EMSs in
# priority order.
SPACE_ALTERNATIVES: int = 3

# May a deviating pallet key open a new pallet although the box fits on an
# open one?
#   "never"           no
#   "below_capacity"  only while fewer pallets are open than emsLoader's
#                     MAX_OPEN_PALLETS (starting the next pallet early
#                     without closing one)
#   "always"          yes; at capacity this closes the oldest open pallet
EARLY_NEW_PALLET: str = "below_capacity"


# ============================================================================
# OBJECTIVE (after the hard gate; lexicographic)
# ============================================================================
#   1. pallet count                        lower is better
#   2. density score                       higher is better
#   3. centre-of-gravity height            lower is better

# Density score = weighted mix of
#   - mean squared fill rate (sum of squared fill rates / pallet count)
#   - utilization of the occupied envelope (box volume / footprint x stack
#     height, summed over all pallets)
# Set one weight to 0 to use the other alone.
DENSITY_WEIGHT_SSF: float = 0.5
DENSITY_WEIGHT_ENVELOPE: float = 0.5

# Density scores closer than this count as equal; then the CoG decides.
# This is also the epsilon of the stopping criterion.
DENSITY_RESOLUTION: float = 0.001

# CoG heights closer than this count as equal [mm].
COG_RESOLUTION_MM: float = 1.0


# ============================================================================
# STABILITY EVALUATOR (analytical tilted-gravity check)
# ============================================================================

GRAVITY_M_S2: float = 9.81

# Acceleration limits of the requirements document.
ACCEL_FORWARD_M_S2: float = 2.0
ACCEL_BRAKING_M_S2: float = 3.0
ACCEL_LATERAL_M_S2: float = 2.0

# Pallet axis the truck travels along: "x" = pallet length (1200 mm, forks
# enter from the 800 mm side). Both directions along this axis are checked
# with the larger of forward and braking, both lateral directions with the
# lateral limit.
TRAVEL_AXIS: str = "x"

# a_crit / a_spec a unit needs to count as stable (1.0 = exactly the spec).
STABILITY_REQUIRED_MARGIN: float = 1.0

# Which rigid unit the gate checks:
#   "load_tree"  each box together with everything resting on it
#   "box"        each box alone on its support polygon
# Both margins are always computed and reported.
STABILITY_UNIT: str = "load_tree"

# Hard stability gate:
#   "spec"      every unit must reach STABILITY_REQUIRED_MARGIN. Raises an
#               error when no solution does.
#   "baseline"  a solution must be at least as stable as plain emsLoader
#               (worst margin not lower, no more units below spec).
#   "auto"      "spec" when plain emsLoader already meets the spec,
#               otherwise "baseline".
#   "off"       stability is reported only.
STABILITY_GATE: str = "auto"