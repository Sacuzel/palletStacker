"""Heuristical evaluation to rank potential box placements."""

import config
# Late-stage refinement bonuses. See interlocking.py for full rationale; both 
# functions silently return 0.0 for placements where their rules don't apply 
# (floor placements, non-flat orientations, etc.), so it is always safe to add 
# their results to the primary score.
from interlocking import evaluate_adjacency_bonus, evaluate_interlock_bonus

def evaluate_candidate(cand, pallet) -> float:
    """Scores a legally valid candidate placement based on heuristic bonuses."""
    score = 0.0
    
    # 1. HARD CENTER START CONSTRAINT
    # If the pallet is empty, massively reward placing the box at the exact center.
    if len(pallet.placements) == 0:
        if abs(cand.x - config.PALLET_LENGTH / 2) < 1.0 and abs(cand.y - config.PALLET_WIDTH / 2) < 1.0:
            score += config.WEIGHT_CENTER_START

    # 2. MAXIMIZE SAME-HEIGHT SURFACE (PLATEAU)
    # Check how much of the surrounding surface aligns with the top of this candidate.
    cand_top = cand.z + cand.height
    plateau_area = cand.length * cand.width
    
    for p in pallet.placements:
        if abs((p.z + p.height) - cand_top) <= config.PLATEAU_TOLERANCE:
            plateau_area += p.length * p.width
            
    # Normalize relative to pallet area, max multiplier is WEIGHT_PLATEAU
    score += (plateau_area / config.PALLET_AREA) * config.WEIGHT_PLATEAU

    # 3. MODULO TRICK (Bonus for clean divisions against available bounds)
    # Calculate the area the box is resting on.
    if abs(cand.z) < 1e-4:
        base_area = config.PALLET_AREA
    else:
        base_area = 0.0
        for p in pallet.placements:
            if abs((p.z + p.height) - cand.z) <= config.PLATEAU_TOLERANCE:
                base_area += p.length * p.width
                
    # We only care about optimizing against the pallet bounds if we are building a primary layer
    if base_area > 0.5 * config.PALLET_AREA:
        # We calculate waste against the remaining pallet dimensions
        rem_x = config.PALLET_LENGTH - cand.x
        rem_y = config.PALLET_WIDTH - cand.y
        
        # Avoid division by zero and negative remainders
        if rem_x >= cand.length and rem_y >= cand.width:
            waste_x = rem_x % cand.length
            waste_y = rem_y % cand.width
            
            # Convert waste into a bonus: 0 waste = max bonus. 
            # (1.0 - ratio) gives a value between 0.0 and 1.0 per axis
            bonus_x = 1.0 - (waste_x / cand.length)
            bonus_y = 1.0 - (waste_y / cand.width)
            
            score += (bonus_x + bonus_y) * config.WEIGHT_MODULO

    # 4. PRIMARY GOAL: Z-DENSITY (Keep mass as low as possible)
    # With WEIGHT_ZDENSITY set very high, this guarantees flat boxes win.
    z_density = cand.box.weight / cand.height
    score += z_density * config.WEIGHT_ZDENSITY
    
    # 5. TIE-BREAKER: GRAVITY PENALTY
    # Forces boxes to prefer the floor over stacking if plateau/density scores are tied.
    score -= cand.z * config.WEIGHT_GRAVITY
    
    # 6. LATE-STAGE REFINEMENT: LATERAL WALL SUPPORT (adjacency)
    # Rewards stacked candidates that touch other placed boxes on their vertical 
    # faces - "having a wall to lean against" beats redundant bottom coverage 
    # once the 70% support threshold is met. Returns 0 at floor level.
    score += evaluate_adjacency_bonus(cand, pallet)
    
    # 7. LATE-STAGE REFINEMENT: BRICK-STYLE INTERLOCKING
    # Rewards flat, stacked candidates whose footprint spans 2 or 3 boxes below 
    # (each underlying box covered >= 15% of its own top). Prevents straight 
    # tower seams. Returns 0 at floor level or for non-flat orientations.
    score += evaluate_interlock_bonus(cand, pallet)
    
    return score