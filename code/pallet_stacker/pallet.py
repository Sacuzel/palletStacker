"""Domain model for a pallet and its placed boxes.

The pallet owns placement state and performs basic deterministic validation:
* footprint and height limits
* maximum load
* duplicate IDs
* axis-aligned box overlap

Support, stability, loading sequence and placement scoring belong to separate
algorithm modules and are intentionally not implemented here.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import isfinite
from typing import Iterable

from .box import Box, Orientation, Point3D


@dataclass(frozen=True, slots=True)
class PlacementCheck:
    """Result of validating one proposed placement."""

    accepted: bool
    reasons: tuple[str, ...] = ()

    def __bool__(self) -> bool:
        return self.accepted


class PlacementError(ValueError):
    """Raised when a box placement violates pallet constraints."""


@dataclass(slots=True)
class Pallet:
    """Pallet geometry, limits and currently placed boxes.

    Coordinate convention
    ---------------------
    The packing coordinate origin is the lower-left corner of the pallet's top
    loading surface. Therefore a box resting directly on the pallet has z=0.
    ``base_height_mm`` describes the physical pallet itself for later Plotly or
    Gazebo export; it is not added to box Z coordinates used by the algorithm.
    """

    pallet_id: str
    length_mm: float
    width_mm: float
    max_height_mm: float
    max_load_kg: float | None = None
    base_height_mm: float = 144.0
    name: str | None = None
    _boxes: dict[str, Box] = field(default_factory=dict, init=False, repr=False)

    def __post_init__(self) -> None:
        self.pallet_id = self.pallet_id.strip()
        if not self.pallet_id:
            raise ValueError("pallet_id must not be empty.")

        for field_name, value in (
            ("length_mm", self.length_mm),
            ("width_mm", self.width_mm),
            ("max_height_mm", self.max_height_mm),
            ("base_height_mm", self.base_height_mm),
        ):
            if not isfinite(value) or value <= 0:
                raise ValueError(f"{field_name} must be a positive finite number.")

        if self.max_load_kg is not None:
            if not isfinite(self.max_load_kg) or self.max_load_kg <= 0:
                raise ValueError("max_load_kg must be positive when provided.")

    @property
    def boxes(self) -> tuple[Box, ...]:
        """Placed boxes as a read-only tuple."""

        return tuple(self._boxes.values())

    @property
    def box_count(self) -> int:
        return len(self._boxes)

    @property
    def current_load_kg(self) -> float:
        return sum(box.weight_kg for box in self._boxes.values())

    @property
    def remaining_load_kg(self) -> float | None:
        if self.max_load_kg is None:
            return None
        return self.max_load_kg - self.current_load_kg

    @property
    def load_height_mm(self) -> float:
        """Highest occupied Z coordinate above the pallet loading surface."""

        if not self._boxes:
            return 0.0
        return max(box.bounds()[1].z for box in self._boxes.values())

    @property
    def usable_volume_mm3(self) -> float:
        return self.length_mm * self.width_mm * self.max_height_mm

    @property
    def occupied_box_volume_mm3(self) -> float:
        return sum(box.volume_mm3 for box in self._boxes.values())

    @property
    def volume_utilization(self) -> float:
        """Nominal box volume divided by the pallet's usable bounding volume."""

        return self.occupied_box_volume_mm3 / self.usable_volume_mm3

    def contains(self, box_id: str) -> bool:
        return box_id in self._boxes

    def get_box(self, box_id: str) -> Box:
        try:
            return self._boxes[box_id]
        except KeyError as exc:
            raise KeyError(f"Box {box_id!r} is not on pallet {self.pallet_id!r}.") from exc

    def check_placement(
        self,
        box: Box,
        position: Point3D,
        orientation: Orientation,
        *,
        check_overlap: bool = True,
        tolerance_mm: float = 1e-6,
    ) -> PlacementCheck:
        """Validate basic pallet constraints without changing any state."""

        if tolerance_mm < 0 or not isfinite(tolerance_mm):
            raise ValueError("tolerance_mm must be a non-negative finite number.")

        reasons: list[str] = []

        if box.box_id in self._boxes:
            reasons.append(f"Box ID {box.box_id!r} is already present on this pallet.")

        try:
            minimum, maximum = box.bounds(position, orientation)
        except (TypeError, ValueError) as exc:
            reasons.append(str(exc))
            return PlacementCheck(False, tuple(reasons))

        if minimum.x < -tolerance_mm:
            reasons.append("Box extends beyond the pallet at X minimum.")
        if minimum.y < -tolerance_mm:
            reasons.append("Box extends beyond the pallet at Y minimum.")
        if minimum.z < -tolerance_mm:
            reasons.append("Box extends below the pallet loading surface.")
        if maximum.x > self.length_mm + tolerance_mm:
            reasons.append("Box extends beyond the pallet length.")
        if maximum.y > self.width_mm + tolerance_mm:
            reasons.append("Box extends beyond the pallet width.")
        if maximum.z > self.max_height_mm + tolerance_mm:
            reasons.append("Box exceeds the maximum allowed load height.")

        if (
            self.max_load_kg is not None
            and self.current_load_kg + box.weight_kg > self.max_load_kg + tolerance_mm
        ):
            reasons.append("Adding the box would exceed the pallet load limit.")

        if check_overlap:
            for existing in self._boxes.values():
                if self._bounds_overlap(
                    candidate_min=minimum,
                    candidate_max=maximum,
                    existing=existing,
                    tolerance_mm=tolerance_mm,
                ):
                    reasons.append(f"Box overlaps existing box {existing.box_id!r}.")

        return PlacementCheck(not reasons, tuple(reasons))

    def place_box(
        self,
        box: Box,
        position: Point3D,
        orientation: Orientation,
        *,
        check_overlap: bool = True,
        tolerance_mm: float = 1e-6,
    ) -> None:
        """Validate, place and register a box atomically."""

        result = self.check_placement(
            box,
            position,
            orientation,
            check_overlap=check_overlap,
            tolerance_mm=tolerance_mm,
        )
        if not result:
            joined = " ".join(result.reasons)
            raise PlacementError(
                f"Cannot place box {box.box_id!r} on pallet {self.pallet_id!r}: {joined}"
            )

        box.place(position, orientation)
        self._boxes[box.box_id] = box

    def remove_box(self, box_id: str) -> Box:
        """Remove a box from the pallet and clear its placement state."""

        box = self.get_box(box_id)
        del self._boxes[box_id]
        box.unplace()
        return box

    def clear(self) -> None:
        """Remove all boxes and clear their placement states."""

        for box in self._boxes.values():
            box.unplace()
        self._boxes.clear()

    def add_prevalidated_boxes(self, boxes: Iterable[Box]) -> None:
        """Register already placed boxes, validating IDs and basic geometry.

        This is mainly useful when rebuilding pallet state from persisted data.
        Each box must already contain a placement.
        """

        for box in boxes:
            if box.placement is None:
                raise PlacementError(f"Box {box.box_id!r} has no placement.")
            self.place_box(
                box,
                box.placement.position,
                box.placement.orientation,
            )

    @staticmethod
    def _bounds_overlap(
        *,
        candidate_min: Point3D,
        candidate_max: Point3D,
        existing: Box,
        tolerance_mm: float,
    ) -> bool:
        existing_min, existing_max = existing.bounds()

        overlap_x = (
            candidate_min.x < existing_max.x - tolerance_mm
            and candidate_max.x > existing_min.x + tolerance_mm
        )
        overlap_y = (
            candidate_min.y < existing_max.y - tolerance_mm
            and candidate_max.y > existing_min.y + tolerance_mm
        )
        overlap_z = (
            candidate_min.z < existing_max.z - tolerance_mm
            and candidate_max.z > existing_min.z + tolerance_mm
        )
        return overlap_x and overlap_y and overlap_z
