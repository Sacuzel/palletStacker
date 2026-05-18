import config

def check_bounds(cand) -> bool:
    """Ensure the placement is strictly inside the usable pallet volume."""
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
        dx = max(0, min(cand.x + cand.length, p.x + p.length) - max(cand.x, p.x))
        dy = max(0, min(cand.y + cand.width, p.y + p.width) - max(cand.y, p.y))
        dz = max(0, min(cand.z + cand.height, p.z + p.height) - max(cand.z, p.z))
        
        # If all 3 axes overlap by more than a microscopic float error, they intersect
        if dx > 1e-4 and dy > 1e-4 and dz > 1e-4:
            return True
    return False

def check_support(cand, placements) -> bool:
    """Ensure candidate has >= 70% bottom area supported."""
    if abs(cand.z) < 1e-4:
        return True  # Resting directly on the pallet deck

    cand_area = cand.length * cand.width
    supported_area = 0.0

    for p in placements:
        # Check if placement p is flush directly underneath the candidate
        if abs((p.z + p.height) - cand.z) <= config.PLATEAU_TOLERANCE:
            dx = max(0, min(cand.x + cand.length, p.x + p.length) - max(cand.x, p.x))
            dy = max(0, min(cand.y + cand.width, p.y + p.width) - max(cand.y, p.y))
            if dx > 0 and dy > 0:
                supported_area += dx * dy
                
    return (supported_area / cand_area) >= config.MIN_SUPPORT_RATIO