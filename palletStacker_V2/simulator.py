"""
Kinematic simulation for Pallet Jack stability.
Based on standard physics of rigid bodies and EN 12195-1 / EUMOS 40509 standards.
"""

import config
from pallet import Pallet

def simulate_pallet_physics(pallet: Pallet) -> dict:
    """
    Evaluates the pallet stack against realistic kinetic forces.
    Checks 3 failure modes: Sliding, Global Tipping, and Local Column Tipping.
    """
    g = config.SIM_GRAVITY
    a_brake = config.SIM_BRAKE_DECEL
    
    # 1. Centrifugal acceleration (a = v^2 / r)
    a_turn = (config.SIM_SPEED_MS ** 2) / config.SIM_TURN_RADIUS_M
    
    # 2. SLIDING CHECK (Inertia vs Friction)
    # The mass cancels out on both sides of the equation (m*a > u*m*g -> a > u*g)
    a_slide_threshold = config.SIM_FRICTION_COEFF * g
    will_slide = max(a_brake, a_turn) > a_slide_threshold
    
    # 3. GLOBAL PALLET TIPPING CHECK
    # First, calculate the true Center of Mass (CoM) including the 25kg wooden pallet base
    total_mass = config.PALLET_WEIGHT_KG
    sum_mx = config.PALLET_WEIGHT_KG * (config.PALLET_LENGTH / 2)
    sum_my = config.PALLET_WEIGHT_KG * (config.PALLET_WIDTH / 2)
    sum_mz = config.PALLET_WEIGHT_KG * (config.PALLET_BASE_HEIGHT / 2)
    
    for p in pallet.placements:
        m = p.box.weight
        total_mass += m
        sum_mx += m * p.center[0]
        sum_my += m * p.center[1]
        
        # Center Z is relative to the pallet deck. We add the base height to get absolute height from the floor.
        abs_z = p.center[2] + config.PALLET_BASE_HEIGHT
        sum_mz += m * abs_z
        
    global_com_x = sum_mx / total_mass
    global_com_y = sum_my / total_mass
    global_com_z = sum_mz / total_mass
    
    # Calculate distance from CoM to the physical pivot points (wheels).
    # Lateral pivots are the outer edges of the pallet jack forks (540mm), NOT the 800mm pallet edges!
    fork_margin = (config.PALLET_WIDTH - config.SIM_JACK_FORK_WIDTH) / 2.0
    pivot_y_left = fork_margin
    pivot_y_right = config.PALLET_WIDTH - fork_margin
    
    min_dy = min(abs(global_com_y - pivot_y_left), abs(pivot_y_right - global_com_y))
    
    # Longitudinal pivots are the front load wheels and rear steer wheels.
    pivot_x_front = 100.0
    pivot_x_back = config.PALLET_LENGTH - 100.0
    min_dx = min(abs(global_com_x - pivot_x_front), abs(pivot_x_back - global_com_x))
    
    # Threshold for tipping: a = g * (horizontal_distance_to_pivot / height_of_CoM)
    a_tip_global_x = g * (min_dx / global_com_z)
    a_tip_global_y = g * (min_dy / global_com_z)
    
    global_tip_brake = a_brake > a_tip_global_x
    global_tip_turn = a_turn > a_tip_global_y
    
    # 4. LOCAL BOX/COLUMN TIPPING CHECK
    # Checks if higher boxes will snap off and tip over the boxes beneath them.
    local_tip_failures = 0
    for i, p in enumerate(pallet.placements):
        
        # Approximate a physical "column" by grabbing this box and everything structurally above it
        col_mass = p.box.weight
        col_sum_mx = p.box.weight * p.center[0]
        col_sum_my = p.box.weight * p.center[1]
        col_sum_mz = p.box.weight * p.center[2]
        
        for j, upper in enumerate(pallet.placements):
            if i == j: continue
            if upper.z >= (p.z + p.height - 1e-4):
                # Check 2D structural overlap
                dx = max(0, min(p.x + p.length, upper.x + upper.length) - max(p.x, upper.x))
                dy = max(0, min(p.y + p.width, upper.y + upper.width) - max(p.y, upper.y))
                if dx > 0 and dy > 0:
                    col_mass += upper.box.weight
                    col_sum_mx += upper.box.weight * upper.center[0]
                    col_sum_my += upper.box.weight * upper.center[1]
                    col_sum_mz += upper.box.weight * upper.center[2]
                    
        col_com_x = col_sum_mx / col_mass
        col_com_y = col_sum_my / col_mass
        col_com_z = col_sum_mz / col_mass
        
        # Height of column CoM ABOVE the pivot plane (the bottom of this specific box)
        h_cog = col_com_z - p.z
        
        # Distances from column CoM to the edges of THIS box (the pivot edges)
        pivot_dx = min(abs(col_com_x - p.x), abs((p.x + p.length) - col_com_x))
        pivot_dy = min(abs(col_com_y - p.y), abs((p.y + p.width) - col_com_y))
        
        # Tipping thresholds for this specific local sub-stack
        a_tip_local_x = g * (pivot_dx / h_cog)
        a_tip_local_y = g * (pivot_dy / h_cog)
        
        if a_brake > a_tip_local_x or a_turn > a_tip_local_y:
            local_tip_failures += 1

    return {
        "a_turn": a_turn,
        "a_brake": a_brake,
        "a_slide_threshold": a_slide_threshold,
        "will_slide": will_slide,
        "a_tip_global_x": a_tip_global_x,
        "a_tip_global_y": a_tip_global_y,
        "global_tip_brake": global_tip_brake,
        "global_tip_turn": global_tip_turn,
        "local_tip_failures": local_tip_failures
    }