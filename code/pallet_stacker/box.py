"""Domain model for a single SKU box.

Coordinate convention
---------------------
* Units are millimetres and kilograms.
* A box position is the minimum X/Y/Z corner of its axis-aligned bounds.
* Orientation changes the mapping of the box's original dimensions to X/Y/Z.

This module contains no pallet-packing heuristic. It only stores box data and
provides deterministic geometry/state operations.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from math import isfinite
from typing import Any, Mapping


@dataclass(frozen=True, slots=True)
class Point3D:
    """A point in millimetres."""

    x: float
    y: float
    z: float

    def __post_init__(self) -> None:
        for name, value in (("x", self.x), ("y", self.y), ("z", self.z)):
            if not isfinite(value):
                raise ValueError(f"Point coordinate {name} must be finite.")


@dataclass(frozen=True, slots=True)
class Dimensions3D:
    """Axis-aligned dimensions in millimetres."""

    x: float
    y: float
    z: float

    def __post_init__(self) -> None:
        for name, value in (("x", self.x), ("y", self.y), ("z", self.z)):
            if not isfinite(value) or value <= 0:
                raise ValueError(f"Dimension {name} must be a positive finite number.")

    @property
    def volume_mm3(self) -> float:
        return self.x * self.y * self.z

    @property
    def base_area_mm2(self) -> float:
        return self.x * self.y


class Orientation(str, Enum):
    """Permutation of original length, width and height onto X, Y and Z."""

    XYZ = "xyz"  # X=length, Y=width,  Z=height
    YXZ = "yxz"  # X=width,  Y=length, Z=height
    XZY = "xzy"  # X=length, Y=height, Z=width
    ZXY = "zxy"  # X=height, Y=length, Z=width
    YZX = "yzx"  # X=width,  Y=height, Z=length
    ZYX = "zyx"  # X=height, Y=width,  Z=length


UPRIGHT_ORIENTATIONS: tuple[Orientation, ...] = (
    Orientation.XYZ,
    Orientation.YXZ,
)

ALL_ORIENTATIONS: tuple[Orientation, ...] = tuple(Orientation)


@dataclass(frozen=True, slots=True)
class Placement:
    """The current placement of a box in pallet-local coordinates."""

    position: Point3D
    orientation: Orientation


@dataclass(slots=True)
class Box:
    """A physical SKU box and its optional current placement.

    By default, only upright orientations are allowed. This means the box may
    rotate 90 degrees around the vertical axis, but it is not placed on its side.
    """

    box_id: str
    length_mm: float
    width_mm: float
    height_mm: float
    weight_kg: float = 0.0
    sku: str | None = None
    name: str | None = None
    contents: str | None = None
    allowed_orientations: tuple[Orientation, ...] = UPRIGHT_ORIENTATIONS
    metadata: dict[str, Any] = field(default_factory=dict)
    _placement: Placement | None = field(default=None, init=False, repr=False)

    def __post_init__(self) -> None:
        self.box_id = self.box_id.strip()
        if not self.box_id:
            raise ValueError("box_id must not be empty.")

        # Reuse Dimensions3D validation.
        Dimensions3D(self.length_mm, self.width_mm, self.height_mm)

        if not isfinite(self.weight_kg) or self.weight_kg < 0:
            raise ValueError("weight_kg must be a non-negative finite number.")

        if not self.allowed_orientations:
            raise ValueError("At least one allowed orientation is required.")

        # Remove duplicates while preserving caller order.
        self.allowed_orientations = tuple(dict.fromkeys(self.allowed_orientations))
        if not all(isinstance(item, Orientation) for item in self.allowed_orientations):
            raise TypeError("allowed_orientations must contain Orientation values.")

        # Detach internal state from a dictionary supplied by the caller.
        self.metadata = dict(self.metadata)

    @property
    def original_dimensions(self) -> Dimensions3D:
        return Dimensions3D(self.length_mm, self.width_mm, self.height_mm)

    @property
    def volume_mm3(self) -> float:
        return self.original_dimensions.volume_mm3

    @property
    def volume_m3(self) -> float:
        return self.volume_mm3 / 1_000_000_000.0

    @property
    def placement(self) -> Placement | None:
        return self._placement

    @property
    def is_placed(self) -> bool:
        return self._placement is not None

    @property
    def position(self) -> Point3D | None:
        return None if self._placement is None else self._placement.position

    @property
    def orientation(self) -> Orientation | None:
        return None if self._placement is None else self._placement.orientation

    @property
    def placed_dimensions(self) -> Dimensions3D | None:
        if self._placement is None:
            return None
        return self.oriented_dimensions(self._placement.orientation)

    def oriented_dimensions(self, orientation: Orientation) -> Dimensions3D:
        """Return dimensions mapped to global X/Y/Z for one orientation."""

        if orientation not in self.allowed_orientations:
            raise ValueError(
                f"Orientation {orientation.value!r} is not allowed for box {self.box_id!r}."
            )

        dimensions_by_axis = {
            "x": self.length_mm,
            "y": self.width_mm,
            "z": self.height_mm,
        }
        permutation = orientation.value
        return Dimensions3D(
            dimensions_by_axis[permutation[0]],
            dimensions_by_axis[permutation[1]],
            dimensions_by_axis[permutation[2]],
        )

    def place(self, position: Point3D, orientation: Orientation) -> None:
        """Set placement after the pallet or placement service has validated it."""

        # This also validates that the orientation is allowed.
        self.oriented_dimensions(orientation)
        self._placement = Placement(position=position, orientation=orientation)

    def unplace(self) -> None:
        """Remove the current placement without changing box properties."""

        self._placement = None

    def bounds(
        self,
        position: Point3D | None = None,
        orientation: Orientation | None = None,
    ) -> tuple[Point3D, Point3D]:
        """Return minimum and maximum corners of an explicit or current placement."""

        if position is None or orientation is None:
            if self._placement is None:
                raise ValueError(
                    "Box is not placed; provide both position and orientation explicitly."
                )
            position = self._placement.position if position is None else position
            orientation = self._placement.orientation if orientation is None else orientation

        dimensions = self.oriented_dimensions(orientation)
        maximum = Point3D(
            position.x + dimensions.x,
            position.y + dimensions.y,
            position.z + dimensions.z,
        )
        return position, maximum

    def corners(
        self,
        position: Point3D | None = None,
        orientation: Orientation | None = None,
    ) -> tuple[Point3D, ...]:
        """Return all eight corners of an explicit or current placement."""

        minimum, maximum = self.bounds(position, orientation)
        return tuple(
            Point3D(x, y, z)
            for x in (minimum.x, maximum.x)
            for y in (minimum.y, maximum.y)
            for z in (minimum.z, maximum.z)
        )

    def center(
        self,
        position: Point3D | None = None,
        orientation: Orientation | None = None,
    ) -> Point3D:
        """Return the center point of an explicit or current placement."""

        minimum, maximum = self.bounds(position, orientation)
        return Point3D(
            (minimum.x + maximum.x) / 2.0,
            (minimum.y + maximum.y) / 2.0,
            (minimum.z + maximum.z) / 2.0,
        )

    def update_metadata(self, values: Mapping[str, Any]) -> None:
        """Merge non-geometric source data, for example fields from input JSON."""

        self.metadata.update(values)
