from dataclasses import dataclass
from typing import Tuple

@dataclass
class Box:
    identifier: str
    sku: str
    length: float
    width: float
    height: float
    weight: float

@dataclass
class Placement:
    pallet_id: str
    box: Box
    x: float
    y: float
    z: float
    length: float
    width: float
    height: float

    @property
    def center(self) -> Tuple[float, float, float]:
        return (
            self.x + self.length / 2,
            self.y + self.width / 2,
            self.z + self.height / 2
        )