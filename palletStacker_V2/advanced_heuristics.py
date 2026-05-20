"""Advanced heuristics for fine-tuning pallet stability (Interlocking and Wall Support)."""

import config

def calculate_interlock_score(cand, pallet) -> float:
    """
    Rewards placements that bridge across 2 or 3 distinct boxes, improving stack cohesion.
    To prevent microscopic overlaps from counting, a supporting box must cover at least 
    MIN_INTERLOCK_COVERAGE_RATIO (e.g., 15%) of the candidate's bottom.
    """
    # Interlocking only applies if we are not on the pallet deck
    if abs(cand.z) < 1e-4:
        return 0.0

    cand_base_area = cand.length * cand.width
    supporting_boxes_count = 0

    for p in pallet.placements:
        # Check if placement p is at the exact height to support the candidate
        if abs((p.z + p.height) - cand.z) <= config.PLATEAU_TOLERANCE:
            
            # Calculate 2D intersection area
            dx = max(0, min(cand.x + cand.length, p.x + p.length) - max(cand.x, p.x))
            dy = max(0, min(cand.y + cand.width, p.y + p.width) - max(cand.y, p.y))
            
            if dx > 0 and dy > 0:
                overlap_area = dx * dy
                coverage_ratio = overlap_area / cand_base_area
                
                # Only count it if it holds a meaningful percentage of the box
                if coverage_ratio >= config.MIN_INTERLOCK_COVERAGE_RATIO:
                    supporting_boxes_count += 1

    # Reward bridging. 2 boxes is great. 3 is also great. 
    # More than 3 usually implies a messy, fragmented plateau, so we cap the reward.
    if supporting_boxes_count == 2:
        return 1.0
    elif supporting_boxes_count == 3:
        return 1.0
    
    return 0.0


def calculate_adjacency_score(cand, pallet) -> float:
    """
    Rewards candidates for physically touching (or being very close to) existing boxes 
    on their X or Y axes. This promotes lateral stability.
    """
    # Wall support is mostly useful for boxes not on the pallet floor
    if abs(cand.z) < 1e-4:
        return 0.0

    total_side_area = 2 * (cand.length * cand.height + cand.width * cand.height)
    touching_area = 0.0

    for p in pallet.placements:
        # Check if the boxes share vertical Z-space (they are side-by-side, not above/below each other)
        z_overlap = max(0, min(cand.z + cand.height, p.z + p.height) - max(cand.z, p.z))
        if z_overlap <= 1e-4:
            continue

        # Check for X-axis adjacency (touching the Left or Right face)
        # Distance between the edges on the X axis must be within tolerance
        x_dist = min(abs(cand.x - (p.x + p.length)), abs((cand.x + cand.length) - p.x))
        y_overlap = max(0, min(cand.y + cand.width, p.y + p.width) - max(cand.y, p.y))

        if x_dist <= config.ADJACENCY_TOLERANCE and y_overlap > 1e-4:
            touching_area += y_overlap * z_overlap

        # Check for Y-axis adjacency (touching the Front or Back face)
        y_dist = min(abs(cand.y - (p.y + p.width)), abs((cand.y + cand.width) - p.y))
        x_overlap = max(0, min(cand.x + cand.length, p.x + p.length) - max(cand.x, p.x))

        if y_dist <= config.ADJACENCY_TOLERANCE and x_overlap > 1e-4:
            touching_area += x_overlap * z_overlap

    # Return the percentage of the candidate's side area that is supported by adjacent walls (0.0 to 1.0)
    if total_side_area > 0:
        return touching_area / total_side_area
    return 0.0