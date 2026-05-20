"""
Late-stage placement refinements: lateral wall support and brick-style interlocking.

These two bonuses run AFTER the primary heuristics (z-density, plateau, modulo,
gravity) have shaped the candidate scores. They are intentionally smaller in
magnitude than the primary terms; their job is not to override fundamental
decisions, but to refine choices between placements that the primary heuristics
consider roughly equivalent (typically between two L/W-rotated flat orientations
at the same Extreme Point, or between adjacent positions at the same z level).

Both rules share two important restrictions:
  1. They never apply to floor placements (z == 0). The bare deck already
     provides 100% bottom support, so:
       - Adjacency: the trade-off between "more bottom area" and "lateral wall
         support to compensate for less bottom area" doesn't exist - there is
         no missing bottom area to compensate for.
       - Interlocking: by definition this is about resting across MULTIPLE
         boxes below. The deck is a single continuous surface, not "boxes".
  2. They never reject placements. They only contribute additive score terms;
     a placement that misses the bonus simply doesn't get it. Hard constraints
     remain the responsibility of geometry.check_bounds / check_intersection /
     check_support.
"""

import config


# Tolerance (in mm) for considering two vertical faces "touching". The extreme-
# point packer uses exact corner coordinates, so 1 mm comfortably catches any
# floating-point noise without admitting real gaps as contact.
_TOUCH_TOLERANCE = 1.0

# Minimum fraction of an underlying box's top area that must be covered by the
# candidate for that underlying box to count as a real interlocking supporter.
# Below this threshold the contact is just a sliver and contributes no real
# structural tie between layers - precisely the case the rule is meant to
# exclude (the "90% on A, sliver on B" pathology).
_INTERLOCK_MIN_COVERAGE = 0.15


def evaluate_adjacency_bonus(cand, pallet) -> float:
    """
    Reward stacked candidates that share vertical face area with neighbors.

    Once a stacked candidate has the required 70% bottom support, additional
    bottom-area coverage offers diminishing structural return. What it actually
    benefits from at that point is a vertical "wall" - a neighbor box at the
    same Z range that the candidate can lean against. Walls resist tipping and
    sliding during forklift acceleration and truck transit.

    The bonus measures the fraction of the candidate's total vertical face area
    (perimeter * height) that is actually in face-to-face contact with other
    placed boxes. Result is in [0, WEIGHT_ADJACENCY].
    """
    # Restriction 1: floor placements are skipped (see module docstring).
    if abs(cand.z) < 1e-4:
        return 0.0

    # Compute the total vertical face area used for normalization.
    cand_perimeter = 2.0 * (cand.length + cand.width)
    total_face_area = cand_perimeter * cand.height
    if total_face_area < 1e-4:
        return 0.0  # Degenerate box - can't normalize, just bail

    # Cache the candidate's six bounding planes for the loop below.
    cand_x0, cand_x1 = cand.x, cand.x + cand.length
    cand_y0, cand_y1 = cand.y, cand.y + cand.width
    cand_z0, cand_z1 = cand.z, cand.z + cand.height

    contact_area = 0.0

    # Iterate over every placed box. We sum face-to-face overlap area wherever
    # a neighbor's face is flush against one of the candidate's four vertical
    # faces.
    for p in pallet.placements:
        p_x0, p_x1 = p.x, p.x + p.length
        p_y0, p_y1 = p.y, p.y + p.width
        p_z0, p_z1 = p.z, p.z + p.height

        # First, the boxes must overlap in Z. A neighbor with no Z overlap is
        # either below or above the candidate, not beside it, and can't act
        # as a wall to lean against.
        dz = max(0.0, min(cand_z1, p_z1) - max(cand_z0, p_z0))
        if dz <= 1e-4:
            continue

        # For each of the candidate's four vertical faces, test whether the
        # neighbor's corresponding face is flush against it. If flush, the
        # face contact area is dz (height overlap) times the orthogonal-axis
        # overlap.

        # +X face of candidate vs -X face of neighbor.
        if abs(p_x0 - cand_x1) < _TOUCH_TOLERANCE:
            dy = max(0.0, min(cand_y1, p_y1) - max(cand_y0, p_y0))
            contact_area += dy * dz
        # -X face of candidate vs +X face of neighbor.
        if abs(p_x1 - cand_x0) < _TOUCH_TOLERANCE:
            dy = max(0.0, min(cand_y1, p_y1) - max(cand_y0, p_y0))
            contact_area += dy * dz
        # +Y face of candidate vs -Y face of neighbor.
        if abs(p_y0 - cand_y1) < _TOUCH_TOLERANCE:
            dx = max(0.0, min(cand_x1, p_x1) - max(cand_x0, p_x0))
            contact_area += dx * dz
        # -Y face of candidate vs +Y face of neighbor.
        if abs(p_y1 - cand_y0) < _TOUCH_TOLERANCE:
            dx = max(0.0, min(cand_x1, p_x1) - max(cand_x0, p_x0))
            contact_area += dx * dz

    # Normalize to a ratio in [0, 1] and apply the weight. The cap at 1.0 is
    # a safety guard against floating-point edge cases - geometrically, a
    # ratio of 1.0 means every square millimeter of the candidate's vertical
    # face area is in contact with another box (extremely rare; would require
    # being engulfed on all four sides by neighbors of at least equal height).
    ratio = min(1.0, contact_area / total_face_area)
    return ratio * config.WEIGHT_ADJACENCY


def evaluate_interlock_bonus(cand, pallet) -> float:
    """
    Reward a flat, stacked candidate that spans 2 or 3 distinct boxes below.

    Stacking columns of identical boxes directly on top of one another is the
    weak pattern: every "seam" between columns runs straight from top to
    bottom of the stack, and the whole tower can pivot at those seams.
    Brick-style interlocking - where a box on top straddles the seam between
    two below - ties the columns together and dramatically improves the
    stack's lateral stability.

    Restrictions:
      1. Only applies when stacked (z > 0). See module docstring.
      2. Only applies in the flattest orientation. The flat-and-low primary
         heuristic must already have selected this orientation; if the box is
         standing on edge there's a more basic problem to fix first, and we
         shouldn't reward interlocking a sub-optimal orientation.
      3. Each candidate counts only underlying boxes it covers by at least
         _INTERLOCK_MIN_COVERAGE (15%) of THEIR OWN top area. This excludes
         the pathological "candidate sits 90% on box A and a sliver on box B"
         case - if anything shifts box B, the candidate loses that supposed
         support, so it shouldn't have been counted as interlocking in the
         first place.
      4. Reward is awarded for exactly 2 or 3 supporters. 1 supporter is
         plain stacking, not interlocking. 4 or more straddles too many
         seams; the user's spec explicitly caps this rule at 3 to avoid
         destabilizing layouts.
    """
    # Restriction 1: floor placements skip.
    if abs(cand.z) < 1e-4:
        return 0.0

    # Restriction 2: only reward the flattest orientation of the box. The
    # flattest is the rotation with the smallest of the three dims as height.
    # We compute the min from the box's natural dimensions, so this works
    # regardless of the order the user supplied dims in.
    min_possible_height = min(cand.box.length, cand.box.width, cand.box.height)
    if abs(cand.height - min_possible_height) > 1e-4:
        return 0.0

    # Walk every placed box and count those that qualify as MEANINGFUL
    # supporters (i.e. directly underneath AND covered >= 15% of their top).
    meaningful_supporters = 0
    for p in pallet.placements:
        # The underlying box's top surface must be flush with the candidate's
        # bottom surface, within plateau tolerance (boxes at nominally the
        # same height may have a few mm of variance after floating-point ops).
        if abs((p.z + p.height) - cand.z) > config.PLATEAU_TOLERANCE:
            continue

        # Compute the 2D overlap rectangle between the candidate's footprint
        # and the underlying box's top.
        dx = max(0.0, min(cand.x + cand.length, p.x + p.length) - max(cand.x, p.x))
        dy = max(0.0, min(cand.y + cand.width, p.y + p.width) - max(cand.y, p.y))
        if dx <= 0.0 or dy <= 0.0:
            continue  # No 2D overlap -> not actually a supporter

        overlap = dx * dy
        p_top_area = p.length * p.width
        if p_top_area < 1e-4:
            continue  # Degenerate underlying box, skip

        # Coverage is measured against the UNDERLYING box's own top area,
        # matching the spec "candidate covers at least 15% of the box below".
        # A 15%+ overlap means the candidate is meaningfully resting on this
        # supporter, not just slivering across it.
        if (overlap / p_top_area) >= _INTERLOCK_MIN_COVERAGE:
            meaningful_supporters += 1

    # Restriction 4: award only for the 2-3 supporter sweet spot.
    # - 0 supporters: cannot physically occur if check_support already passed
    #   (the candidate would have been rejected outright before scoring).
    # - 1 supporter:  plain stacking, no interlock benefit.
    # - 2 or 3:       brick pattern, ties columns together, REWARD.
    # - 4+:           over-spread; user's spec explicitly excludes this so
    #                 the algorithm doesn't try to make every box span four
    #                 underlying ones, which destabilizes more seams than
    #                 it ties.
    if meaningful_supporters == 2 or meaningful_supporters == 3:
        return config.WEIGHT_INTERLOCK

    return 0.0