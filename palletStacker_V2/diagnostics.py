"""Diagnostic reporting for the pallet loading algorithm."""

from typing import List
import config
from pallet import Pallet

def print_packing_report(pallets: List[Pallet]):
    """
    Prints a formatted, readable terminal report showing the chronological 
    placement order, dimensions, weight, position, and physics simulation.
    """
    if not pallets:
        print("\n[!] No pallets were generated.")
        return

    print("\n" + "=" * 90)
    print(f"{'PACKING ALGORITHM DIAGNOSTICS REPORT':^90}")
    print("=" * 90)

    for pallet in pallets:
        total_height = pallet.max_stack_height + config.PALLET_BASE_HEIGHT
        
        print(f"\nPALLET {pallet.pallet_id}")
        print(f"Total Boxes:  {len(pallet.placements)}")
        print(f"Max Height:   {total_height:.1f} mm (including pallet base)")
        print("-" * 90)
        
        header = (
            f"{'Seq':<4} | "
            f"{'Identifier':<12} | "
            f"{'SKU':<8} | "
            f"{'Size (L x W x H)':<20} | "
            f"{'Weight':<8} | "
            f"{'Pos (X, Y, Z)':<20}"
        )
        print(header)
        print("-" * 90)
        
        for idx, placement in enumerate(pallet.placements, start=1):
            box = placement.box
            size_str = f"{placement.length:.0f}x{placement.width:.0f}x{placement.height:.0f}"
            weight_str = f"{box.weight:.1f}kg"
            pos_str = f"({placement.x:.0f}, {placement.y:.0f}, {placement.z:.0f})"
            
            row = (
                f"{idx:<4} | "
                f"{box.identifier:<12} | "
                f"{box.sku:<8} | "
                f"{size_str:<20} | "
                f"{weight_str:<8} | "
                f"{pos_str:<20}"
            )
            print(row)
            
        print("-" * 90)

        # --- RUN AND PRINT PHYSICS SIMULATION IF ENABLED ---
        if config.RUN_SIMULATION:
            from simulator import simulate_pallet_physics
            res = simulate_pallet_physics(pallet)
            
            print(f"{'--- KINEMATIC STABILITY SIMULATION (PALLET JACK) ---':^90}")
            print(f" Parameters: Speed = {config.SIM_SPEED_MS} m/s | Turn Radius = {config.SIM_TURN_RADIUS_M} m | Friction = {config.SIM_FRICTION_COEFF}")
            print(f" Induced Forces: Braking = {res['a_brake']:.2f} m/s^2 | Cornering = {res['a_turn']:.2f} m/s^2")
            
            # 1. Sliding
            slide_status = "FAILED (Boxes will slide)" if res['will_slide'] else "PASS (Friction limits hold)"
            print(f" -> Sliding Risk:             {slide_status:<30} (Threshold: {res['a_slide_threshold']:.2f} m/s^2)")
            
            # 2. Global Braking Tipping
            brake_tip_status = "FAILED (Pallet tips forward)" if res['global_tip_brake'] else "PASS"
            print(f" -> Global Tipping (Braking):   {brake_tip_status:<30} (Threshold: {res['a_tip_global_x']:.2f} m/s^2)")
            
            # 3. Global Cornering Tipping
            turn_tip_status = "FAILED (Pallet tips sideways)" if res['global_tip_turn'] else "PASS"
            print(f" -> Global Tipping (Cornering): {turn_tip_status:<30} (Threshold: {res['a_tip_global_y']:.2f} m/s^2)")
            
            # 4. Local Column Tipping
            if res['local_tip_failures'] > 0:
                print(f" -> Local Column Tipping:       FAILED ({res['local_tip_failures']} sub-stacks will tip off the stack)")
            else:
                print(f" -> Local Column Tipping:       PASS (All internal columns stable)")
                
        print("=" * 90)