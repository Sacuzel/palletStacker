"""Simple sequential tower-and-row pallet loader.

This loader is deliberately unsophisticated. It provides a deterministic
baseline against which later algorithms can be compared.

Policy
------
1. Process boxes in their input order.
2. Start at pallet-local coordinate X=0, Y=0.
3. Stack boxes at the active tower's X/Y origin.
4. Close the tower when the next box cannot be placed there.
5. Move in +X to start another tower.
6. When the row has no usable X space, move in +Y to start another row.
7. When no new row fits, create another pallet.

The loader checks pallet boundaries, height, mass, and overlap through the
``Pallet`` class. It does not check support area, stability, crushing strength,
centre of mass, or robot reachability.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from typing import Iterable

from . import settings
from .box import Box, Dimensions3D, Orientation, Point3D
from .pallet import Pallet


# ============================================================================
# NAIVE LOADER SETTINGS
# ============================================================================

# The loader tries orientations in this order. The input parser currently gives
# boxes the two upright orientations, so boxes are not turned onto their sides.
NAIVE_ORIENTATION_ORDER: tuple[Orientation, ...] = (
    Orientation.XYZ,
    Orientation.YXZ,
)

# Empty space left between adjacent towers and rows. Zero means flush packing.
NAIVE_TOWER_GAP_MM: float = 0.0
NAIVE_ROW_GAP_MM: float = 0.0

# Permit a higher box to have a larger footprint than boxes below it. This is
# intentionally True for the naive baseline and may create unstable overhangs.
# Set False to require every added box to fit inside the current tower footprint.
NAIVE_ALLOW_TOWER_FOOTPRINT_GROWTH: bool = True


class NaiveLoaderError(RuntimeError):
    """Base exception raised by the naive loader."""


class UnplaceableBoxError(NaiveLoaderError):
    """Raised when one box cannot fit on an otherwise empty pallet."""


@dataclass(slots=True)
class _TowerState:
    """Mutable geometry of the currently active tower."""

    origin_x: float
    origin_y: float
    height_mm: float
    footprint_length_mm: float
    footprint_width_mm: float


@dataclass(slots=True)
class _PalletState:
    """Cursor state used while filling one pallet."""

    pallet: Pallet
    cursor_x: float = 0.0
    cursor_y: float = 0.0
    row_depth_mm: float = 0.0
    active_tower: _TowerState | None = None


@dataclass(frozen=True, slots=True)
class _Candidate:
    """One valid box placement candidate."""

    orientation: Orientation
    position: Point3D
    dimensions: Dimensions3D
    resulting_footprint_length_mm: float
    resulting_footprint_width_mm: float


def load_boxes(boxes: Iterable[Box]) -> list[Pallet]:
    """Load all boxes and return newly created pallets.

    The iterable order is preserved. Each box object is placed directly and is
    therefore mutated by receiving a placement. Supplying a box that is already
    placed is treated as a programming error.
    """

    _validate_settings()
    box_sequence = tuple(boxes)

    if not box_sequence:
        return []

    duplicate_ids = _find_duplicate_ids(box_sequence)
    if duplicate_ids:
        joined = ", ".join(sorted(duplicate_ids))
        raise NaiveLoaderError(f"Duplicate box IDs supplied to loader: {joined}")

    already_placed = [box.box_id for box in box_sequence if box.is_placed]
    if already_placed:
        joined = ", ".join(already_placed)
        raise NaiveLoaderError(f"Boxes are already placed: {joined}")

    pallets: list[Pallet] = []
    state = _new_pallet_state(len(pallets) + 1)
    pallets.append(state.pallet)

    for box in box_sequence:
        # A mass limit can force a new pallet even when geometry remains free.
        if not _weight_fits(state.pallet, box) and state.pallet.box_count > 0:
            state = _new_pallet_state(len(pallets) + 1)
            pallets.append(state.pallet)

        candidate = _candidate_for_active_tower(state, box)
        if candidate is not None:
            _commit_candidate(state, box, candidate, starts_new_tower=False)
            continue

        _close_active_tower(state)

        candidate = _candidate_for_new_tower(state, box)
        if candidate is None:
            _start_new_row(state)
            candidate = _candidate_for_new_tower(state, box)

        if candidate is None and state.pallet.box_count == 0:
            # The current pallet is empty, so opening another identical pallet
            # cannot make this box placeable.
            raise UnplaceableBoxError(_unplaceable_message(box, state.pallet))

        if candidate is None:
            state = _new_pallet_state(len(pallets) + 1)
            pallets.append(state.pallet)
            candidate = _candidate_for_new_tower(state, box)

        if candidate is None:
            # This means the box cannot fit on a completely empty pallet.
            raise UnplaceableBoxError(_unplaceable_message(box, state.pallet))

        _commit_candidate(state, box, candidate, starts_new_tower=True)

    return pallets


def _new_pallet_state(pallet_number: int) -> _PalletState:
    pallet_id = (
        f"{settings.PALLET_ID_PREFIX}-"
        f"{pallet_number:0{settings.PALLET_ID_DIGITS}d}"
    )
    pallet = Pallet(
        pallet_id=pallet_id,
        name=settings.PALLET_NAME,
        length_mm=settings.PALLET_LENGTH_MM,
        width_mm=settings.PALLET_WIDTH_MM,
        base_height_mm=settings.PALLET_BASE_HEIGHT_MM,
        max_height_mm=settings.PALLET_MAX_HEIGHT_MM,
        max_load_kg=settings.PALLET_MAX_LOAD_KG,
    )
    return _PalletState(pallet=pallet)


def _candidate_for_active_tower(
    state: _PalletState,
    box: Box,
) -> _Candidate | None:
    tower = state.active_tower
    if tower is None:
        return None

    if not _weight_fits(state.pallet, box):
        return None

    for orientation in _orientation_candidates(box):
        dimensions = box.oriented_dimensions(orientation)

        if NAIVE_ALLOW_TOWER_FOOTPRINT_GROWTH:
            footprint_length = max(tower.footprint_length_mm, dimensions.x)
            footprint_width = max(tower.footprint_width_mm, dimensions.y)
        else:
            if (
                dimensions.x > tower.footprint_length_mm + settings.PLACEMENT_TOLERANCE_MM
                or dimensions.y
                > tower.footprint_width_mm + settings.PLACEMENT_TOLERANCE_MM
            ):
                continue
            footprint_length = tower.footprint_length_mm
            footprint_width = tower.footprint_width_mm

        if (
            tower.origin_x + footprint_length
            > state.pallet.length_mm + settings.PLACEMENT_TOLERANCE_MM
            or tower.origin_y + footprint_width
            > state.pallet.width_mm + settings.PLACEMENT_TOLERANCE_MM
        ):
            continue

        position = Point3D(
            x=tower.origin_x,
            y=tower.origin_y,
            z=tower.height_mm,
        )
        check = state.pallet.check_placement(
            box,
            position,
            orientation,
            tolerance_mm=settings.PLACEMENT_TOLERANCE_MM,
        )
        if check:
            return _Candidate(
                orientation=orientation,
                position=position,
                dimensions=dimensions,
                resulting_footprint_length_mm=footprint_length,
                resulting_footprint_width_mm=footprint_width,
            )

    return None


def _candidate_for_new_tower(
    state: _PalletState,
    box: Box,
) -> _Candidate | None:
    if not _weight_fits(state.pallet, box):
        return None

    for orientation in _orientation_candidates(box):
        dimensions = box.oriented_dimensions(orientation)
        position = Point3D(x=state.cursor_x, y=state.cursor_y, z=0.0)

        check = state.pallet.check_placement(
            box,
            position,
            orientation,
            tolerance_mm=settings.PLACEMENT_TOLERANCE_MM,
        )
        if check:
            return _Candidate(
                orientation=orientation,
                position=position,
                dimensions=dimensions,
                resulting_footprint_length_mm=dimensions.x,
                resulting_footprint_width_mm=dimensions.y,
            )

    return None


def _commit_candidate(
    state: _PalletState,
    box: Box,
    candidate: _Candidate,
    *,
    starts_new_tower: bool,
) -> None:
    state.pallet.place_box(
        box,
        candidate.position,
        candidate.orientation,
        tolerance_mm=settings.PLACEMENT_TOLERANCE_MM,
    )

    if starts_new_tower:
        state.active_tower = _TowerState(
            origin_x=candidate.position.x,
            origin_y=candidate.position.y,
            height_mm=candidate.dimensions.z,
            footprint_length_mm=candidate.resulting_footprint_length_mm,
            footprint_width_mm=candidate.resulting_footprint_width_mm,
        )
    else:
        tower = state.active_tower
        if tower is None:  # Defensive: callers must preserve this invariant.
            raise NaiveLoaderError("Internal error: no active tower to update.")
        tower.height_mm += candidate.dimensions.z
        tower.footprint_length_mm = candidate.resulting_footprint_length_mm
        tower.footprint_width_mm = candidate.resulting_footprint_width_mm

    tower = state.active_tower
    if tower is None:
        raise NaiveLoaderError("Internal error: placement did not create a tower.")
    state.row_depth_mm = max(state.row_depth_mm, tower.footprint_width_mm)


def _close_active_tower(state: _PalletState) -> None:
    tower = state.active_tower
    if tower is None:
        return

    state.cursor_x = (
        tower.origin_x
        + tower.footprint_length_mm
        + NAIVE_TOWER_GAP_MM
    )
    state.row_depth_mm = max(state.row_depth_mm, tower.footprint_width_mm)
    state.active_tower = None


def _start_new_row(state: _PalletState) -> None:
    state.cursor_x = 0.0
    state.cursor_y += state.row_depth_mm + NAIVE_ROW_GAP_MM
    state.row_depth_mm = 0.0
    state.active_tower = None


def _orientation_candidates(box: Box) -> tuple[Orientation, ...]:
    preferred = tuple(
        orientation
        for orientation in NAIVE_ORIENTATION_ORDER
        if orientation in box.allowed_orientations
    )
    remaining = tuple(
        orientation
        for orientation in box.allowed_orientations
        if orientation not in preferred
    )
    return preferred + remaining


def _weight_fits(pallet: Pallet, box: Box) -> bool:
    if pallet.max_load_kg is None:
        return True
    return (
        pallet.current_load_kg + box.weight_kg
        <= pallet.max_load_kg + settings.PLACEMENT_TOLERANCE_MM
    )


def _unplaceable_message(box: Box, pallet: Pallet) -> str:
    orientations = ", ".join(item.value for item in _orientation_candidates(box))
    return (
        f"Box {box.box_id!r} cannot fit on an empty pallet {pallet.pallet_id!r}. "
        f"Box dimensions are {box.length_mm:g} x {box.width_mm:g} x "
        f"{box.height_mm:g} mm, weight is {box.weight_kg:g} kg, and tested "
        f"orientations were [{orientations}]. Pallet limits are "
        f"{pallet.length_mm:g} x {pallet.width_mm:g} x "
        f"{pallet.max_height_mm:g} mm with maximum load "
        f"{pallet.max_load_kg!r} kg."
    )


def _find_duplicate_ids(boxes: tuple[Box, ...]) -> set[str]:
    seen: set[str] = set()
    duplicates: set[str] = set()
    for box in boxes:
        if box.box_id in seen:
            duplicates.add(box.box_id)
        seen.add(box.box_id)
    return duplicates


def _validate_settings() -> None:
    positive_values = {
        "PALLET_LENGTH_MM": settings.PALLET_LENGTH_MM,
        "PALLET_WIDTH_MM": settings.PALLET_WIDTH_MM,
        "PALLET_BASE_HEIGHT_MM": settings.PALLET_BASE_HEIGHT_MM,
        "PALLET_MAX_HEIGHT_MM": settings.PALLET_MAX_HEIGHT_MM,
    }
    for name, value in positive_values.items():
        if not isfinite(value) or value <= 0:
            raise NaiveLoaderError(f"settings.{name} must be positive and finite.")

    non_negative_values = {
        "NAIVE_TOWER_GAP_MM": NAIVE_TOWER_GAP_MM,
        "NAIVE_ROW_GAP_MM": NAIVE_ROW_GAP_MM,
        "PLACEMENT_TOLERANCE_MM": settings.PLACEMENT_TOLERANCE_MM,
    }
    for name, value in non_negative_values.items():
        if not isfinite(value) or value < 0:
            raise NaiveLoaderError(
                f"settings.{name} must be non-negative and finite."
            )

    if settings.PALLET_MAX_LOAD_KG is not None and (
        not isfinite(settings.PALLET_MAX_LOAD_KG)
        or settings.PALLET_MAX_LOAD_KG <= 0
    ):
        raise NaiveLoaderError(
            "settings.PALLET_MAX_LOAD_KG must be positive or None."
        )

    if settings.PALLET_ID_DIGITS <= 0:
        raise NaiveLoaderError("settings.PALLET_ID_DIGITS must be positive.")

    if not NAIVE_ORIENTATION_ORDER:
        raise NaiveLoaderError(
            "NAIVE_ORIENTATION_ORDER must contain at least one orientation."
        )
