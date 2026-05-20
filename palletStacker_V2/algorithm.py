"""The core Greedy Online 3D Bin Packing loop."""

import itertools
from typing import List
from models import Box, Placement
from pallet import Pallet
from geometry import check_bounds, check_intersection, check_support
from scoring import evaluate_candidate

def get_rotations(box: Box) -> List[tuple]:
    """Return up to 6 unique L/W/H dimensional rotations for a box."""
    dims = [box.length, box.width, box.height]
    return list(set(itertools.permutations(dims)))

def pack_boxes(boxes: List[Box]) -> List[Pallet]:
    """Takes a 1D stream of boxes and packs them onto 3D Pallets iteratively."""
    pallets = []
    current_pallet = Pallet("1")
    
    for box in boxes:
        placed = False
        
        # Keep trying to place the current box until successful or completely rejected
        while not placed:
            best_score = -float('inf')
            best_placement = None
            rotations = get_rotations(box)
            
            # Test every orientation at every available corner (Extreme Point)
            for ep in current_pallet.extreme_points:
                for rot in rotations:
                    cand = Placement(
                        pallet_id=current_pallet.pallet_id,
                        box=box,
                        x=ep[0], y=ep[1], z=ep[2],
                        length=rot[0], width=rot[1], height=rot[2]
                    )
                    
                    # Hard constraints: If it physically fails, discard this candidate immediately
                    if not check_bounds(cand): continue
                    if check_intersection(cand, current_pallet.placements): continue
                    if not check_support(cand, current_pallet.placements): continue
                    
                    # Soft constraints: Score the valid placement
                    score = evaluate_candidate(cand, current_pallet)
                    if score > best_score:
                        best_score = score
                        best_placement = cand
                        
            # If we found at least one physically valid spot, apply the highest scoring one
            if best_placement is not None:
                current_pallet.add_placement(best_placement)
                placed = True
            else:
                # If the pallet is completely empty but the box still doesn't fit, 
                # the box is larger than the pallet itself. Discard it.
                if len(current_pallet.placements) == 0:
                    print(f"Skipping Box {box.identifier} (too large for pallet).")
                    break 
                
                # Otherwise, the current pallet is full. Save it and start a new empty pallet.
                pallets.append(current_pallet)
                current_pallet = Pallet(str(len(pallets) + 1))
                
    # Append the last pallet if it contains any boxes
    if len(current_pallet.placements) > 0:
        pallets.append(current_pallet)
        
    return pallets