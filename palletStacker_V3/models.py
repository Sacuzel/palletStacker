"""=====================================================================
MODELS  (role: data types / "DUTs" - Data Unit Types)
=====================================================================
Plain data containers used across the whole program. No algorithm
logic here - only the definition of what a Box and a Placement are,
plus small derived read-only properties.

Both visualization_plotly.py and gazebo_exporter.py consume these
types, so the field names must stay as they are:
    Box:       identifier, sku, length, width, height, weight
    Placement: box, pallet_id, x, y, z, length, width, height, center
====================================================================="""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class Box:
    """One physical box as it arrives on the conveyor.

    length/width/height are the dimensions of the box in its
    ORIGINAL orientation as given in the input JSON [mm].
    The packing algorithm may later yaw-rotate the box; the chosen
    orientation is stored in the Placement, not here.
    """

    identifier: str        # unique serial, e.g. "WINE-003"
    sku: str               # product group, e.g. "WINE"
    length: float          # X size [mm]
    width: float           # Y size [mm]
    height: float          # Z size [mm]
    weight: float          # mass [kg]

    @property
    def volume_mm3(self) -> float:
        """Volume of the box [mm^3]. Used for utilisation statistics."""
        return self.length * self.width * self.height


@dataclass
class Placement:
    """One box that has been committed to a position on a pallet.

    (x, y, z) is the MINIMUM corner of the box in pallet-local
    coordinates: x along pallet length, y along pallet width,
    z measured from the pallet deck upwards. length/width/height
    are the dimensions IN THE CHOSEN ORIENTATION - any of the up
    to 6 axis-aligned orientations of the cuboid, so they may be
    any permutation of the Box's own length/width/height.
    """

    box: Box               # reference to the packed box
    pallet_id: str         # id of the pallet this box sits on
    x: float               # min corner X [mm]
    y: float               # min corner Y [mm]
    z: float               # min corner Z [mm], 0 = pallet deck
    length: float          # oriented X size [mm]
    width: float           # oriented Y size [mm]
    height: float          # Z size [mm] (never changes, no tipping)

    @property
    def center(self) -> Tuple[float, float, float]:
        """Geometric centre of the box [mm].

        The task statement allows assuming the centre of mass
        coincides with the geometric centre, so this point doubles
        as the nominal CoG for stability checks and labels.
        """
        return (
            self.x + self.length / 2.0,
            self.y + self.width / 2.0,
            self.z + self.height / 2.0,
        )

    @property
    def top_z(self) -> float:
        """Height of the top face above the pallet deck [mm]."""
        return self.z + self.height