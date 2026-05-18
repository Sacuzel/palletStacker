from typing import List
import config
from models import Placement

class Pallet:
    def __init__(self, pallet_id: str):
        self.pallet_id = pallet_id
        self.length = config.PALLET_LENGTH
        self.width = config.PALLET_WIDTH
        self.max_height = config.USABLE_HEIGHT
        self.max_stack_height = 0.0
        self.placements: List[Placement] = []
        
        # Seed EPs to cover all 4 quadrants, allowing packing to expand outward 
        # organically from the required starting center point.
        self.extreme_points = [
            (self.length / 2, self.width / 2, 0.0),
            (0.0, 0.0, 0.0),
            (self.length / 2, 0.0, 0.0),
            (0.0, self.width / 2, 0.0)
        ]

    def add_placement(self, p: Placement):
        self.placements.append(p)
        self.max_stack_height = max(self.max_stack_height, p.z + p.height)
        
        # Generate new discrete extreme points around the new box
        new_eps = [
            (p.x + p.length, p.y, p.z),
            (p.x, p.y + p.width, p.z),
            (p.x, p.y, p.z + p.height),
            # Add top bounding corners to allow building complex plateaus
            (p.x + p.length, p.y, p.z + p.height),
            (p.x, p.y + p.width, p.z + p.height),
            (p.x + p.length, p.y + p.width, p.z + p.height)
        ]
        
        self.extreme_points.extend(new_eps)
        self._filter_extreme_points()

    def _filter_extreme_points(self):
        """Removes Extreme Points that are out of bounds or engulfed by boxes."""
        valid_eps = []
        for ep in self.extreme_points:
            # 1. Check bounds
            if ep[0] >= self.length or ep[1] >= self.width or ep[2] >= self.max_height:
                continue
                
            # 2. Check if inside an existing box
            inside = False
            for p in self.placements:
                if (p.x + 1e-4 < ep[0] < p.x + p.length - 1e-4 and
                    p.y + 1e-4 < ep[1] < p.y + p.width - 1e-4 and
                    p.z + 1e-4 < ep[2] < p.z + p.height - 1e-4):
                    inside = True
                    break
            
            if not inside:
                valid_eps.append(ep)
                
        # Deduplicate
        self.extreme_points = list(set(valid_eps))