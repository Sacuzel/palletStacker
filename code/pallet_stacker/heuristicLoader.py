"""Online heuristic pallet loader using Corner Points, Gravity Drop, and Heuristic Scoring.

Adapted from the Martello, Pisinger & Vigo (2000) corner-point rule with gravity
settling and multi-objective scoring (Wang & Hauser 2019, Bischoff & Ratcliff 1995).

Heuristics
----------
1. Candidate Positions (Corner Points):
   Generates (x, y) coordinates along pallet walls, box edges, and flush offsets.
   Each candidate is dropped under gravity to find its natural resting Z height.
2. Support & Height Validation:
   Computes under-support ratio at the resting level and checks pallet limits.
3. Multi-term Scoring (Lower is better):
   - Top Height (minimisation): Keeps load surface low, implicitly building layers.
   - Support Ratio (maximisation): Rewards fully supported resting surfaces.
   - Lateral Snugness (maximisation): Rewards touching walls and neighbors.
   - Same-SKU Column Bonus: Encourages clean vertical alignment for identical SKUs.
   - Deepest-Bottom-Left: Deterministic tie-breaking favoring low y, then low x.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from typing import Iterable, Sequence

from . import settings
from .box import Box, Dimensions3D, Orientation, Point3D
from .pallet import Pallet


# ============================================================================
# ALGORITHM SETTINGS & SCORING WEIGHTS
# ============================================================================

# Scoring weights (tuned for grocery box palletising)
W_TOP_HEIGHT: float = 1.0        # Penalises higher resting heights [mm]
W_SUPPORT: float = 200.0         # Rewards high under-support ratio [0..1]
W_SNUGNESS: float = 100.0        # Rewards touching walls/neighbor boxes [0..1]
W_SAME_SKU_ALIGN: float = 150.0  # Rewards identical column stacking {0, 1}

# Minimum fraction of box bottom that must be supported (0.0 .. 1.0)
MIN_SUPPORT_RATIO: float = 0.50

# Grid snapping resolution in mm (1.0 = 1mm discrete grid)
GRID_RESOLUTION_MM: float = 1.0

# Set False to strictly close pallets when a box cannot fit (no backtracking).
# Set True to allow first-fit across all currently open pallets.
FIRST_FIT_OPEN_PALLETS: bool = False


class HeuristicLoaderError(RuntimeError):
    """Base exception raised by the heuristic loader."""


class UnplaceableBoxError(HeuristicLoaderError):
    """Raised when a box cannot fit even on an empty pallet."""


@dataclass(frozen=True, slots=True)
class _Candidate:
    """A fully evaluated, valid candidate placement."""

    position: Point3D
    orientation: Orientation
    dimensions: Dimensions3D
    support_ratio: float
    score: float


# ============================================================================
# MAIN ENTRY POINT
# ============================================================================

def load_boxes(boxes: Iterable[Box]) -> list[Pallet]:
    """Load all boxes sequentially using corner points and heuristic scoring."""
    _validate_settings()
    box_sequence = tuple(boxes)

    if not box_sequence:
        return []

    duplicate_ids = _find_duplicate_ids(box_sequence)
    if duplicate_ids:
        joined = ", ".join(sorted(duplicate_ids))
        raise HeuristicLoaderError(f"Duplicate box IDs supplied to loader: {joined}")

    already_placed = [box.box_id for box in box_sequence if box.is_placed]
    if already_placed:
        joined = ", ".join(already_placed)
        raise HeuristicLoaderError(f"Boxes are already placed: {joined}")

    pallets: list[Pallet] = []

    for box in box_sequence:
        placed = False

        # Determine which pallets can be considered
        if FIRST_FIT_OPEN_PALLETS:
            eligible_pallets = pallets
        else:
            eligible_pallets = pallets[-1:] if pallets else []

        # Try to place on eligible existing pallets
        for pallet in eligible_pallets:
            candidate = _find_best_placement(pallet, box)
            if candidate is not None:
                pallet.place_box(
                    box,
                    candidate.position,
                    candidate.orientation,
                    tolerance_mm=settings.PLACEMENT_TOLERANCE_MM,
                )
                placed = True
                break

        # Open a new pallet if not placed
        if not placed:
            new_pallet = _create_new_pallet(len(pallets) + 1)
            pallets.append(new_pallet)

            candidate = _find_best_placement(new_pallet, box)
            if candidate is not None:
                new_pallet.place_box(
                    box,
                    candidate.position,
                    candidate.orientation,
                    tolerance_mm=settings.PLACEMENT_TOLERANCE_MM,
                )
                placed = True
            else:
                # Box cannot fit on a completely clean pallet
                raise UnplaceableBoxError(_unplaceable_message(box, new_pallet))

    return pallets


# ============================================================================
# CANDIDATE SEARCH & SCORING
# ============================================================================

def _find_best_placement(pallet: Pallet, box: Box) -> _Candidate | None:
    """Evaluate all corner-point candidates on the pallet and return the best one."""
    if not _weight_fits(pallet, box):
        return None

    best: _Candidate | None = None
    tol = settings.PLACEMENT_TOLERANCE_MM

    for orientation, length_mm, width_mm, height_mm in _orientations(box):
        xs, ys = _candidate_coordinates(pallet, length_mm, width_mm)

        for x_mm in xs:
            for y_mm in ys:
                eval_result = _evaluate_placement(
                    pallet, box, x_mm, y_mm, orientation, length_mm, width_mm, height_mm
                )
                if eval_result is None:
                    continue

                support_z, support_ratio = eval_result
                top_height = support_z + height_mm

                # Calculate composite heuristic score (lower is better)
                score = W_TOP_HEIGHT * top_height
                score -= W_SUPPORT * support_ratio
                score -= W_SNUGNESS * _snugness(pallet, x_mm, y_mm, length_mm, width_mm, support_z)

                if _is_same_sku_column(pallet, box, x_mm, y_mm, length_mm, width_mm, support_z):
                    score -= W_SAME_SKU_ALIGN

                # Deepest-bottom-left deterministic tie-breaker
                score += 1e-3 * y_mm + 1e-6 * x_mm

                if best is None or score < best.score:
                    best = _Candidate(
                        position=Point3D(x=x_mm, y=y_mm, z=support_z),
                        orientation=orientation,
                        dimensions=Dimensions3D(x=length_mm, y=width_mm, z=height_mm),
                        support_ratio=support_ratio,
                        score=score,
                    )

    return best


# ============================================================================
# GEOMETRY, GRAVITY & CORNER POINTS
# ============================================================================

def _orientations(box: Box) -> list[tuple[Orientation, float, float, float]]:
    """Return unique (orientation, length_x, width_y, height_z) configurations."""
    unique: list[tuple[Orientation, float, float, float]] = []
    seen_dims: set[tuple[float, float, float]] = set()

    for orientation in box.allowed_orientations:
        dim = box.oriented_dimensions(orientation)
        key = (round(dim.x, 2), round(dim.y, 2), round(dim.z, 2))
        if key not in seen_dims:
            seen_dims.add(key)
            unique.append((orientation, dim.x, dim.y, dim.z))

    return unique


def _candidate_coordinates(
    pallet: Pallet,
    length_mm: float,
    width_mm: float,
) -> tuple[list[float], list[float]]:
    """Build candidate X and Y coordinates based on walls and existing box edges."""
    xs: set[float] = {0.0, pallet.length_mm - length_mm}
    ys: set[float] = {0.0, pallet.width_mm - width_mm}

    for placed in pallet.boxes:
        p_min, p_max = placed.bounds()
        xs.update((p_min.x, p_max.x, p_max.x - length_mm))
        ys.update((p_min.y, p_max.y, p_max.y - width_mm))

    grid = GRID_RESOLUTION_MM
    tol = settings.PLACEMENT_TOLERANCE_MM
    max_x = pallet.length_mm - length_mm + tol
    max_y = pallet.width_mm - width_mm + tol

    xs_ok = sorted(
        {round(v / grid) * grid for v in xs if -tol <= v <= max_x}
    )
    ys_ok = sorted(
        {round(v / grid) * grid for v in ys if -tol <= v <= max_y}
    )
    return xs_ok, ys_ok


def _evaluate_placement(
    pallet: Pallet,
    box: Box,
    x_mm: float,
    y_mm: float,
    orientation: Orientation,
    length_mm: float,
    width_mm: float,
    height_mm: float,
) -> tuple[float, float] | None:
    """Drop the box under gravity at (x, y) and calculate under-support.

    Returns (support_z, support_ratio) or None if placement violates limits.
    """
    tol = settings.PLACEMENT_TOLERANCE_MM
    cand_x0 = x_mm
    cand_x1 = x_mm + length_mm
    cand_y0 = y_mm
    cand_y1 = y_mm + width_mm

    # Find the natural resting level: max top Z of all boxes overlapping in XY
    support_z = 0.0
    overlapping_boxes: list[Box] = []

    for placed in pallet.boxes:
        b_min, b_max = placed.bounds()
        if (
            cand_x0 < b_max.x - tol
            and cand_x1 > b_min.x + tol
            and cand_y0 < b_max.y - tol
            and cand_y1 > b_min.y + tol
        ):
            overlapping_boxes.append(placed)
            if b_max.z > support_z:
                support_z = b_max.z

    # Check max load height limit
    if support_z + height_mm > pallet.max_height_mm + tol:
        return None

    # Calculate support ratio at the resting level
    cand_area = length_mm * width_mm
    if cand_area <= tol:
        return None

    if support_z <= tol:
        # Resting directly on the pallet floor
        support_ratio = 1.0
    else:
        supported_area = 0.0
        for placed in overlapping_boxes:
            b_min, b_max = placed.bounds()
            # Supporting boxes must have their top face flush with support_z
            if abs(b_max.z - support_z) <= max(tol, 0.5):
                ox = max(0.0, min(cand_x1, b_max.x) - max(cand_x0, b_min.x))
                oy = max(0.0, min(cand_y1, b_max.y) - max(cand_y0, b_min.y))
                supported_area += ox * oy
        support_ratio = supported_area / cand_area

    # Enforce minimum support requirement
    if support_ratio < MIN_SUPPORT_RATIO - tol:
        return None

    # Validate boundaries, load capacity, and 3D collision via Pallet API
    position = Point3D(x=x_mm, y=y_mm, z=support_z)
    check = pallet.check_placement(box, position, orientation, tolerance_mm=tol)
    if not check:
        return None

    return support_z, support_ratio


# ============================================================================
# SNUGNESS & COLUMN INTERLOCK
# ============================================================================

def _snugness(
    pallet: Pallet,
    x_mm: float,
    y_mm: float,
    length_mm: float,
    width_mm: float,
    support_z: float,
) -> float:
    """Fraction (0..1) of the 4 lateral side faces supported by walls or neighbors."""
    tol = settings.PLACEMENT_TOLERANCE_MM
    touching = 0

    cand_x0 = x_mm
    cand_x1 = x_mm + length_mm
    cand_y0 = y_mm
    cand_y1 = y_mm + width_mm

    # -X side: pallet wall or neighbor box
    if cand_x0 <= tol:
        touching += 1
    elif any(
        abs(b.bounds()[1].x - cand_x0) <= tol
        and b.bounds()[1].y > cand_y0 + tol
        and b.bounds()[0].y < cand_y1 - tol
        and b.bounds()[1].z > support_z + tol
        for b in pallet.boxes
    ):
        touching += 1

    # +X side: pallet wall or neighbor box
    if cand_x1 >= pallet.length_mm - tol:
        touching += 1
    elif any(
        abs(b.bounds()[0].x - cand_x1) <= tol
        and b.bounds()[1].y > cand_y0 + tol
        and b.bounds()[0].y < cand_y1 - tol
        and b.bounds()[1].z > support_z + tol
        for b in pallet.boxes
    ):
        touching += 1

    # -Y side: pallet wall or neighbor box
    if cand_y0 <= tol:
        touching += 1
    elif any(
        abs(b.bounds()[1].y - cand_y0) <= tol
        and b.bounds()[1].x > cand_x0 + tol
        and b.bounds()[0].x < cand_x1 - tol
        and b.bounds()[1].z > support_z + tol
        for b in pallet.boxes
    ):
        touching += 1

    # +Y side: pallet wall or neighbor box
    if cand_y1 >= pallet.width_mm - tol:
        touching += 1
    elif any(
        abs(b.bounds()[0].y - cand_y1) <= tol
        and b.bounds()[1].x > cand_x0 + tol
        and b.bounds()[0].x < cand_x1 - tol
        and b.bounds()[1].z > support_z + tol
        for b in pallet.boxes
    ):
        touching += 1

    return touching / 4.0


def _is_same_sku_column(
    pallet: Pallet,
    box: Box,
    x_mm: float,
    y_mm: float,
    length_mm: float,
    width_mm: float,
    support_z: float,
) -> bool:
    """True if candidate sits flush on top of an identical same-SKU box."""
    tol = settings.PLACEMENT_TOLERANCE_MM
    box_sku = getattr(box, "sku", None) or getattr(box, "sku_id", None)

    for placed in pallet.boxes:
        p_min, p_max = placed.bounds()
        p_len = p_max.x - p_min.x
        p_wid = p_max.y - p_min.y

        if (
            abs(p_min.x - x_mm) <= tol
            and abs(p_min.y - y_mm) <= tol
            and abs(p_len - length_mm) <= tol
            and abs(p_wid - width_mm) <= tol
            and abs(p_max.z - support_z) <= tol
        ):
            placed_sku = getattr(placed, "sku", None) or getattr(placed, "sku_id", None)
            if box_sku is not None and placed_sku is not None:
                if box_sku == placed_sku:
                    return True
            else:
                # Fallback to matching nominal dimensions
                if (
                    abs(placed.length_mm - box.length_mm) <= tol
                    and abs(placed.width_mm - box.width_mm) <= tol
                    and abs(placed.height_mm - box.height_mm) <= tol
                ):
                    return True

    return False


# ============================================================================
# HELPERS & VALIDATION
# ============================================================================

def _create_new_pallet(pallet_number: int) -> Pallet:
    pallet_id = (
        f"{settings.PALLET_ID_PREFIX}-"
        f"{pallet_number:0{settings.PALLET_ID_DIGITS}d}"
    )
    return Pallet(
        pallet_id=pallet_id,
        name=settings.PALLET_NAME,
        length_mm=settings.PALLET_LENGTH_MM,
        width_mm=settings.PALLET_WIDTH_MM,
        base_height_mm=settings.PALLET_BASE_HEIGHT_MM,
        max_height_mm=settings.PALLET_MAX_HEIGHT_MM,
        max_load_kg=settings.PALLET_MAX_LOAD_KG,
    )


def _weight_fits(pallet: Pallet, box: Box) -> bool:
    if pallet.max_load_kg is None:
        return True
    return (
        pallet.current_load_kg + box.weight_kg
        <= pallet.max_load_kg + settings.PLACEMENT_TOLERANCE_MM
    )


def _unplaceable_message(box: Box, pallet: Pallet) -> str:
    return (
        f"Box {box.box_id!r} cannot fit on an empty pallet {pallet.pallet_id!r}. "
        f"Box dimensions are {box.length_mm:g} x {box.width_mm:g} x "
        f"{box.height_mm:g} mm, weight is {box.weight_kg:g} kg. Pallet limits are "
        f"{pallet.length_mm:g} x {pallet.width_mm:g} x "
        f"{pallet.max_height_mm:g} mm with maximum load {pallet.max_load_kg!r} kg."
    )


def _find_duplicate_ids(boxes: Sequence[Box]) -> set[str]:
    seen: set[str] = set()
    duplicates: set[str] = set()
    for box in boxes:
        if box.box_id in seen:
            duplicates.add(box.box_id)
        seen.add(box.box_id)
    return duplicates


def _validate_settings() -> None:
    for name, value in (
        ("PALLET_LENGTH_MM", settings.PALLET_LENGTH_MM),
        ("PALLET_WIDTH_MM", settings.PALLET_WIDTH_MM),
        ("PALLET_BASE_HEIGHT_MM", settings.PALLET_BASE_HEIGHT_MM),
        ("PALLET_MAX_HEIGHT_MM", settings.PALLET_MAX_HEIGHT_MM),
    ):
        if not isfinite(value) or value <= 0:
            raise HeuristicLoaderError(f"settings.{name} must be positive and finite.")

    if settings.PALLET_MAX_LOAD_KG is not None and (
        not isfinite(settings.PALLET_MAX_LOAD_KG) or settings.PALLET_MAX_LOAD_KG <= 0
    ):
        raise HeuristicLoaderError("settings.PALLET_MAX_LOAD_KG must be positive or None.")

    if settings.PLACEMENT_TOLERANCE_MM < 0 or not isfinite(settings.PLACEMENT_TOLERANCE_MM):
        raise HeuristicLoaderError("settings.PLACEMENT_TOLERANCE_MM must be non-negative.")