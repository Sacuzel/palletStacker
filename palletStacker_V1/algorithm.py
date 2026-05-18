import itertools
from typing import List
from models import Box, Placement
from pallet import Pallet
from geometry import check_bounds, check_intersection, check_support
from scoring import evaluate_candidate

def get_rotations(box: Box) -> List[tuple]:
    """Return up to 6 unique L/W/H dimensional rotations."""
    dims = [box.length, box.width, box.height]
    return list(set(itertools.permutations(dims)))

def pack_boxes(boxes: List[Box]) -> List[Pallet]:
    pallets = []
    current_pallet = Pallet("1")
    
    for box in boxes:
        placed = False
        
        while not placed:
            best_score = -float('inf')
            best_placement = None
            rotations = get_rotations(box)
            
            for ep in current_pallet.extreme_points:
                for rot in rotations:
                    cand = Placement(
                        pallet_id=current_pallet.pallet_id,
                        box=box,
                        x=ep[0], y=ep[1], z=ep[2],
                        length=rot[0], width=rot[1], height=rot[2]
                    )
                    
                    # Hard constraints
                    if not check_bounds(cand): continue
                    if check_intersection(cand, current_pallet.placements): continue
                    if not check_support(cand, current_pallet.placements): continue
                    
                    # Soft constraints
                    score = evaluate_candidate(cand, current_pallet)
                    if score > best_score:
                        best_score = score
                        best_placement = cand
                        
            if best_placement is not None:
                current_pallet.add_placement(best_placement)
                placed = True
            else:
                # If pallet is completely empty but box doesn't fit, discard box
                if len(current_pallet.placements) == 0:
                    print(f"Skipping Box {box.identifier} (too large for pallet).")
                    break 
                
                # Otherwise, seal current pallet and evaluate on a new one
                pallets.append(current_pallet)
                current_pallet = Pallet(str(len(pallets) + 1))
                
    if len(current_pallet.placements) > 0:
        pallets.append(current_pallet)
        
    return pallets