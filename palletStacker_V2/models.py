"""Data structures representing the physical items in the packing problem."""

from dataclasses import dataclass
from typing import Tuple

@dataclass
class Box:
    """Represents an unplaced grocery box as defined in the input JSON."""
    identifier: str
    sku: str
    length: float
    width: float
    height: float
    weight: float

@dataclass
class Placement:
    """Represents a Box that has been assigned a specific 3D location and rotation on a Pallet."""
    pallet_id: str
    box: Box
    x: float
    y: float
    z: float
    # Length, width, and height here may differ from the Box's base dimensions due to 3D rotation
    length: float
    width: float
    height: float

    @property
    def center(self) -> Tuple[float, float, float]:
        """Calculates the geometric center of the placed box (used for visualization labels)."""
        return (
            self.x + self.length / 2,
            self.y + self.width / 2,
            self.z + self.height / 2
        )