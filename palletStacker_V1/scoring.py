import config

def evaluate_candidate(cand, pallet) -> float:
    """Scores a legally valid candidate placement based on heuristics."""
    score = 0.0
    
    # 1. HARD CENTER START CONSTRAINT
    if len(pallet.placements) == 0:
        if abs(cand.x - config.PALLET_LENGTH / 2) < 1.0 and abs(cand.y - config.PALLET_WIDTH / 2) < 1.0:
            score += config.WEIGHT_CENTER_START

    # 2. MAXIMIZE PLATEAU / SAME-HEIGHT SURFACE
    cand_top = cand.z + cand.height
    plateau_area = cand.length * cand.width
    
    for p in pallet.placements:
        if abs((p.z + p.height) - cand_top) <= config.PLATEAU_TOLERANCE:
            plateau_area += p.length * p.width
            
    # Normalize score relative to pallet size
    score += (plateau_area / config.PALLET_AREA) * config.WEIGHT_PLATEAU

    # 3. MODULO TRICK (Only if resting on > 50% plateau)
    # Calculate base plateau area (the floor this candidate is resting on)
    if abs(cand.z) < 1e-4:
        base_area = config.PALLET_AREA
    else:
        base_area = 0.0
        for p in pallet.placements:
            if abs((p.z + p.height) - cand.z) <= config.PLATEAU_TOLERANCE:
                base_area += p.length * p.width
                
    if base_area > 0.5 * config.PALLET_AREA:
        # Calculate waste based on remaining linear space to the edge
        rem_x = config.PALLET_LENGTH - cand.x
        rem_y = config.PALLET_WIDTH - cand.y
        waste_x = rem_x % cand.length
        waste_y = rem_y % cand.width
        # Penalty: Subtract waste
        score -= (waste_x + waste_y) * config.WEIGHT_MODULO

    # 4. TIE-BREAKER: Z-DENSITY (Minimize CoM)
    # Larger z_density = flatter/denser box orientation
    z_density = cand.box.weight / cand.height
    score += z_density * config.WEIGHT_ZDENSITY
    
    return score