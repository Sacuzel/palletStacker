"""Data structures representing the physical items in the packing problem."""

from dataclasses import dataclass
from typing import Tuple

@dataclass
class Box:
    """Represents an unplaced grocery box as defined in the input JSON."""
    id: str
    sku: str
    length: int
    width: int
    height: int
    weight: int
    support_ids: list = None
    support_count: int = 0
    position: Tuple[int, int, float] = None  # (x, y, z) position on the pallet

    def copy(self) -> "Box":
            return Box(
                id=self.id,
                sku=self.sku,
                length=self.length,
                width=self.width,
                height=self.height,
                weight=self.weight,
                support_ids=list(self.support_ids) if self.support_ids is not None else None,
                support_count=self.support_count,
                position=self.position,
            )