"""Physical bounding and collision detection rules."""

import config

def check_bounds(cand) -> bool:
    """Ensure the placement is strictly inside the usable 3D pallet volume."""
    # 1e-4 is used throughout to prevent false negatives caused by floating-point math inaccuracies
    if cand.x < 0 or cand.y < 0 or cand.z < 0:
        return False
    if cand.x + cand.length > config.PALLET_LENGTH + 1e-4:
        return False
    if cand.y + cand.width > config.PALLET_WIDTH + 1e-4:
        return False
    if cand.z + cand.height > config.USABLE_HEIGHT + 1e-4:
        return False
    return True

def check_intersection(cand, placements) -> bool:
    """Standard 3D Axis-Aligned Bounding Box (AABB) intersection check."""
    for p in placements:
        # Calculate the overlapping distance on each axis
        dx = max(0, min(cand.x + cand.length, p.x + p.length) - max(cand.x, p.x))
        dy = max(0, min(cand.y + cand.width, p.y + p.width) - max(cand.y, p.y))
        dz = max(0, min(cand.z + cand.height, p.z + p.height) - max(cand.z, p.z))
        
        # If all 3 axes overlap by more than a microscopic float error, the boxes are intersecting
        if dx > 1e-4 and dy > 1e-4 and dz > 1e-4:
            return True
    return False

def check_support(cand, placements) -> bool:
    """Ensure candidate has >= MIN_SUPPORT_RATIO (e.g., 70%) bottom area supported by existing boxes."""
    if abs(cand.z) < 1e-4:
        return True  # Resting directly on the pallet deck, completely supported

    cand_area = cand.length * cand.width
    supported_area = 0.0

    for p in placements:
        # Check if placement `p` is flush directly underneath the candidate's bottom
        if abs((p.z + p.height) - cand.z) <= config.PLATEAU_TOLERANCE:
            # Calculate the 2D rectangular overlap area between the bottom box top and candidate bottom
            dx = max(0, min(cand.x + cand.length, p.x + p.length) - max(cand.x, p.x))
            dy = max(0, min(cand.y + cand.width, p.y + p.width) - max(cand.y, p.y))
            if dx > 0 and dy > 0:
                supported_area += dx * dy
                
    return (supported_area / cand_area) >= config.MIN_SUPPORT_RATIO