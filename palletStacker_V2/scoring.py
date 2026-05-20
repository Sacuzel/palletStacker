"""Heuristical evaluation to rank potential box placements."""

import config
from advanced_heuristics import calculate_interlock_score, calculate_adjacency_score

def evaluate_candidate(cand, pallet) -> float:
    """Scores a legally valid candidate placement based on heuristic bonuses."""
    score = 0.0
    
    # 1. HARD CENTER START CONSTRAINT
    if len(pallet.placements) == 0:
        if abs(cand.x - config.PALLET_LENGTH / 2) < 1.0 and abs(cand.y - config.PALLET_WIDTH / 2) < 1.0:
            score += config.WEIGHT_CENTER_START

    # 2. MAXIMIZE SAME-HEIGHT SURFACE (PLATEAU)
    cand_top = cand.z + cand.height
    plateau_area = cand.length * cand.width
    
    for p in pallet.placements:
        if abs((p.z + p.height) - cand_top) <= config.PLATEAU_TOLERANCE:
            plateau_area += p.length * p.width
            
    score += (plateau_area / config.PALLET_AREA) * config.WEIGHT_PLATEAU

    # 3. MODULO TRICK (Bonus for clean divisions against available bounds)
    if abs(cand.z) < 1e-4:
        base_area = config.PALLET_AREA
    else:
        base_area = 0.0
        for p in pallet.placements:
            if abs((p.z + p.height) - cand.z) <= config.PLATEAU_TOLERANCE:
                base_area += p.length * p.width
                
    if base_area > 0.5 * config.PALLET_AREA:
        rem_x = config.PALLET_LENGTH - cand.x
        rem_y = config.PALLET_WIDTH - cand.y
        
        if rem_x >= cand.length and rem_y >= cand.width:
            waste_x = rem_x % cand.length
            waste_y = rem_y % cand.width
            
            bonus_x = 1.0 - (waste_x / cand.length)
            bonus_y = 1.0 - (waste_y / cand.width)
            
            score += (bonus_x + bonus_y) * config.WEIGHT_MODULO

    # 4. PRIMARY GOAL: Z-DENSITY (Keep mass as low as possible)
    z_density = cand.box.weight / cand.height
    score += z_density * config.WEIGHT_ZDENSITY
    
    # 5. NEW: INTERLOCKING / BRIDGING BONUS
    # Promotes rotating the box along the Z-axis to lock 2 or 3 lower boxes together.
    score += calculate_interlock_score(cand, pallet) * config.WEIGHT_INTERLOCK

    # 6. NEW: WALL ADJACENCY / LATERAL SUPPORT
    # Rewards packing the box snugly against existing stacks.
    score += calculate_adjacency_score(cand, pallet) * config.WEIGHT_ADJACENCY

    # 7. TIE-BREAKER: GRAVITY PENALTY
    score -= cand.z * config.WEIGHT_GRAVITY
    
    return score