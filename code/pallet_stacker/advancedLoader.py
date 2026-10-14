"""Online advanced pallet loader: heightmap scoring with optional LBCP stability.

Successor of the old ``algorithm.py`` and ``stability.py``, rebuilt for the
modular pallet_stacker pipeline (``Box`` / ``Pallet`` domain model,
``settings.py`` and the common loader interface
``load_boxes(boxes) -> list[Pallet]``).

Loading model
-------------
* Boxes arrive one at a time in input order (grouped by SKU) and every
  decision is final: no look-ahead, no backtracking.
* Exactly one pallet is open. When the open pallet cannot take the next box,
  it is closed for good and a new pallet is opened.
* Boxes are loaded from above. A box drops vertically onto the highest surface
  under its footprint (heightmap), so nothing lies above a box at the moment
  it is placed and every placement is reachable from the top.
* Every box is packed. A box that does not fit even on an empty pallet stops
  the run with ``UnplaceableBoxError`` instead of being skipped.

Per box
-------
1. Orientations    allowed orientations with distinct footprints.
2. Candidates      x/y positions aligned with the pallet edges and with the
                   edges of placed boxes: min edge, max edge, and max edge
                   minus the box size (right/back alignment on top of a box).
3. Gravity drop    resting height = highest heightmap value under the
                   footprint.
4. Feasibility     height limit, load limit, MIN_SUPPORT_RATIO and, when
                   USE_LBCP_STABILITY is True, the LBCP stability check.
5. Scoring         lower is better:
                       W_TOP_HEIGHT     * resulting top height   [mm]
                     - W_SUPPORT        * support ratio          [0..1]
                     - W_SNUGNESS       * snugness               [0..1]
                     - W_SAME_SKU_ALIGN * same-SKU column bonus  {0, 1}
                     + TIE_BREAK_Y * y + TIE_BREAK_X * x         [mm]
6. Commit          the cheapest candidate; heightmap and feasibility map are
                   updated.

Scoring terms
-------------
* Top height keeps the load surface low, so the pallet is built in layers.
* Support ratio prefers well-supported positions. With LBCP enabled it counts
  only load-bearable contact; otherwise all contact at the resting level.
* Snugness is the fraction of the four side faces backed by a neighbouring
  column higher than the box bottom. Pallet edges do not count: the pallet is
  open, so a box at the edge leans on nothing. It reduces gaps.
* The same-SKU column bonus applies when the candidate sits exactly on an
  identical same-SKU box. Because boxes arrive in SKU groups, each group turns
  into aligned columns with full-area support.
* Low y, then low x, breaks remaining ties deterministically.

LBCP stability (USE_LBCP_STABILITY)
-----------------------------------
Load Bearable Convex Polygon method of Gao, Wang, Kong, Chong: "Online 3D Bin
Packing with Fast Stability Validation and Stable Rearrangement Planning",
arXiv:2507.09123 (2025). Only the stability part is used:

* Heightmap (HM): top height of the load in every grid cell [mm].
* Feasibility map (FM): True where the top surface is load-bearable, i.e. can
  carry any vertical load without anything below toppling. The pallet deck is
  load-bearable everywhere, so FM starts all True.
* Algorithm 1 (check): the support polygon is the convex hull of the contact
  cells that are also True in FM. The placement is stable when the box's
  centre of gravity, widened by COG_TOLERANCE, lies inside that polygon.
* Algorithm 2 (update): after commit, the footprint is raised in HM; in FM it
  becomes non-bearable except for the cells whose centres lie inside the
  support polygon.

The paper's heightmap is a camera distance field; this heightmap measures
height upward from the deck, so the paper's "min HM" corresponds to "max HM".
The support polygon is computed with Andrew's monotone-chain convex hull.

Grid
----
Heightmap and feasibility map are cell grids. The grid must represent every
footprint exactly, otherwise neighbouring boxes would appear to share cells.
With GRID_RESOLUTION_MM = None the resolution is the greatest common divisor
of the pallet size and all box footprint dimensions in the input (whole
millimetres). Only the set of dimensions is used, solely to choose this grid;
the arrival order is never looked at. A small divisor (e.g. 1 mm) is exact
but slower than 5 or 10 mm.
"""

from __future__ import annotations

from collections.abc import Iterable, Sequence
from dataclasses import dataclass
from functools import reduce
from math import gcd, isfinite
from typing import Any

import numpy as np
from numpy.typing import NDArray

from . import settings
from .box import Box, Orientation, Point3D
from .pallet import Pallet

FloatArray = NDArray[np.float64]
AnyFloatArray = NDArray[np.floating[Any]]
BoolArray = NDArray[np.bool_]
Point2D = tuple[float, float]


# ============================================================================
# ALGORITHM SETTINGS
# ============================================================================

# --- Stability --------------------------------------------------------------

# True: only LBCP-stable placements are scored (Algorithms 1 and 2).
# False: only MIN_SUPPORT_RATIO is checked. Keep MIN_SUPPORT_RATIO above 0.5
# in that case; at or below 0.5 the centre of gravity is no longer guaranteed
# to lie inside the contact area.
USE_LBCP_STABILITY: bool = True

# Relative centre-of-gravity uncertainty (0.0 .. 0.5). 0.0 = CoG exactly at
# the geometric centre. Raise (e.g. 0.1) for loads whose contents can shift.
# Used only when USE_LBCP_STABILITY is True.
COG_TOLERANCE: float = 0.0

# Minimum fraction of the footprint that must rest on (load-bearable) contact.
# 0.0 disables the check and relies on LBCP alone.
MIN_SUPPORT_RATIO: float = 0.51

# --- Scoring weights (score is minimised; 1.0 ~ 1 mm of height) -------------

W_TOP_HEIGHT: float = 1.0        # heightmap minimisation
W_SUPPORT: float = 200.0         # reward for high support ratio
W_SNUGNESS: float = 100.0        # reward for touching walls / neighbours
W_SAME_SKU_ALIGN: float = 150.0  # reward for an exact same-SKU column

# Deterministic tie-break per mm of position: prefer low y, then low x.
TIE_BREAK_Y: float = 1e-3
TIE_BREAK_X: float = 1e-6

# --- Grid ---------------------------------------------------------------------

# Cell size [mm]. None = automatic exact resolution (see module docstring).
# A fixed value must divide the pallet size and every footprint dimension.
GRID_RESOLUTION_MM: float | None = None

# --- Numeric tolerances -------------------------------------------------------

CONTACT_HEIGHT_TOL_MM: float = 1e-6  # cells this close to the resting height touch
WEIGHT_EPS_KG: float = 1e-6          # weight-limit tolerance [kg]
_GRID_EPS: float = 1e-6              # divisibility tolerance [cells]


class AdvancedLoaderError(RuntimeError):
    """Base exception raised by the advanced loader."""


class UnplaceableBoxError(AdvancedLoaderError):
    """Raised when a box cannot fit even on an empty pallet."""


# ============================================================================
# INTERNAL TYPES
# ============================================================================

@dataclass(frozen=True, slots=True)
class _Orient:
    """One distinct orientation of the current box, in grid cells."""

    orientation: Orientation
    nx: int               # footprint along X [cells]
    ny: int               # footprint along Y [cells]
    height_mm: float      # vertical size [mm]


@dataclass(frozen=True, slots=True)
class _Evaluation:
    """Algorithm 1 output for one feasible candidate."""

    support_z: float                      # resting height of the box bottom [mm]
    support_ratio: float                  # (bearable) contact cells / footprint cells
    support_polygon: tuple[Point2D, ...]  # LBCP support polygon (empty if LBCP off)


@dataclass(frozen=True, slots=True)
class _Candidate:
    """One fully evaluated candidate placement."""

    ix: int
    iy: int
    orient: _Orient
    evaluation: _Evaluation
    score: float          # lower = better


@dataclass(frozen=True, slots=True)
class _PlacedRecord:
    """Grid footprint of a placed box (candidate edges and column detection)."""

    column_key: tuple[object, ...]
    ix: int
    iy: int
    nx: int
    ny: int
    top_z: float


# ============================================================================
# MAIN ENTRY POINT
# ============================================================================

def load_boxes(boxes: Iterable[Box]) -> list[Pallet]:
    """Load all boxes one by one in arrival order onto one open pallet at a time."""
    _validate_settings()
    box_sequence = tuple(boxes)

    if not box_sequence:
        return []

    duplicate_ids = _find_duplicate_ids(box_sequence)
    if duplicate_ids:
        joined = ", ".join(sorted(duplicate_ids))
        raise AdvancedLoaderError(f"Duplicate box IDs supplied to loader: {joined}")

    already_placed = [box.box_id for box in box_sequence if box.is_placed]
    if already_placed:
        joined = ", ".join(already_placed)
        raise AdvancedLoaderError(f"Boxes are already placed: {joined}")

    grid_mm = _grid_resolution(box_sequence)

    pallets: list[Pallet] = []
    state: _PalletState | None = None

    for box in box_sequence:
        orients = _orientations(box, grid_mm)

        # ---- try the open pallet ---------------------------------------------
        candidate = None
        if state is not None:
            candidate = _find_best_placement(state, box, orients)

        # ---- close it and open a new pallet if it cannot take the box --------
        if state is None or candidate is None:
            new_pallet = _create_new_pallet(len(pallets) + 1)
            pallets.append(new_pallet)
            state = _PalletState(new_pallet, grid_mm)
            candidate = _find_best_placement(state, box, orients)
            if candidate is None:
                raise UnplaceableBoxError(_unplaceable_message(box, new_pallet))

        # ---- commit (online: irrevocable) -----------------------------------
        state.commit(box, candidate)

    return pallets


# ============================================================================
# CANDIDATE SEARCH AND SCORING
# ============================================================================

def _find_best_placement(
    state: _PalletState,
    box: Box,
    orients: Sequence[_Orient],
) -> _Candidate | None:
    """Evaluate every candidate position and orientation on the open pallet and
    return the cheapest feasible one, or None if the pallet cannot take the box."""
    if not _weight_fits(state.pallet, box):
        return None

    res = state.grid_mm
    column_key = _column_key(box)
    best: _Candidate | None = None

    for orient in orients:
        if orient.nx > state.nx or orient.ny > state.ny:
            continue
        xs, ys = _candidate_cells(state, orient)

        for ix in xs:
            for iy in ys:
                evaluation = _validate_placement(state, ix, iy, orient)
                if evaluation is None:
                    continue  # too high / not enough support / unstable

                support_z = evaluation.support_z
                score = W_TOP_HEIGHT * (support_z + orient.height_mm)
                score -= W_SUPPORT * evaluation.support_ratio
                score -= W_SNUGNESS * _snugness(state, ix, iy, orient.nx, orient.ny, support_z)
                if _is_same_sku_column(state, column_key, ix, iy, orient, support_z):
                    score -= W_SAME_SKU_ALIGN
                score += TIE_BREAK_Y * iy * res + TIE_BREAK_X * ix * res

                if best is None or score < best.score:
                    best = _Candidate(ix, iy, orient, evaluation, score)

    return best


def _candidate_cells(state: _PalletState, orient: _Orient) -> tuple[list[int], list[int]]:
    """Candidate X and Y cell indices for one oriented footprint.

    Per axis: both pallet edges, and for every placed box its min edge (stack
    exactly on top / align with it), its max edge (place flush next to it) and
    its max edge minus the box size (align the far sides on top of it).
    """
    xs = {0, state.nx - orient.nx}
    ys = {0, state.ny - orient.ny}
    for placed in state.placed:
        xs.update((placed.ix, placed.ix + placed.nx, placed.ix + placed.nx - orient.nx))
        ys.update((placed.iy, placed.iy + placed.ny, placed.iy + placed.ny - orient.ny))

    max_ix = state.nx - orient.nx
    max_iy = state.ny - orient.ny
    return (
        sorted(v for v in xs if 0 <= v <= max_ix),
        sorted(v for v in ys if 0 <= v <= max_iy),
    )


def _snugness(
    state: _PalletState,
    ix: int,
    iy: int,
    nx: int,
    ny: int,
    support_z: float,
) -> float:
    """Fraction (0..1) of the four side faces backed by a neighbouring column
    higher than the box bottom. Pallet edges give no lateral support (open
    pallet, not a container), so a face on the pallet boundary is unbacked."""
    hm = state.heightmap
    level = support_z + CONTACT_HEIGHT_TOL_MM
    touching = 0

    if ix > 0 and float(hm[ix - 1, iy:iy + ny].max()) > level:                 # -X
        touching += 1
    if ix + nx < state.nx and float(hm[ix + nx, iy:iy + ny].max()) > level:    # +X
        touching += 1
    if iy > 0 and float(hm[ix:ix + nx, iy - 1].max()) > level:                 # -Y
        touching += 1
    if iy + ny < state.ny and float(hm[ix:ix + nx, iy + ny].max()) > level:    # +Y
        touching += 1

    return touching / 4.0


def _is_same_sku_column(
    state: _PalletState,
    column_key: tuple[object, ...],
    ix: int,
    iy: int,
    orient: _Orient,
    support_z: float,
) -> bool:
    """True if the candidate sits exactly on top of an identical same-SKU box
    (same footprint, same x/y, top face at the resting height)."""
    return any(
        placed.column_key == column_key
        and placed.ix == ix
        and placed.iy == iy
        and placed.nx == orient.nx
        and placed.ny == orient.ny
        and abs(placed.top_z - support_z) <= CONTACT_HEIGHT_TOL_MM
        for placed in state.placed
    )


def _column_key(box: Box) -> tuple[object, ...]:
    """Identifies boxes that may form same-SKU columns."""
    if box.sku:
        return ("sku", box.sku)
    return ("dims", round(box.length_mm, 6), round(box.width_mm, 6), round(box.height_mm, 6))


# ============================================================================
# PER-PALLET STATE (heightmap + feasibility map)
# ============================================================================

class _PalletState:
    """Loader-internal model of the open pallet."""

    def __init__(self, pallet: Pallet, grid_mm: float) -> None:
        self.pallet = pallet
        self.grid_mm = grid_mm
        self.nx = _cells(pallet.length_mm, grid_mm)
        self.ny = _cells(pallet.width_mm, grid_mm)
        self.max_height_mm = pallet.max_height_mm

        # HM: top height of the load per cell [mm].
        self.heightmap: FloatArray = np.zeros((self.nx, self.ny), dtype=np.float64)
        # FM: True where the top surface is load-bearable. The deck is one
        # load-bearable polygon, so the map starts all True.
        self.feasibility_map: BoolArray = np.ones((self.nx, self.ny), dtype=bool)
        self.placed: list[_PlacedRecord] = []

    def commit(self, box: Box, candidate: _Candidate) -> None:
        """Place the box on the pallet and update HM and FM (Algorithm 2)."""
        res = self.grid_mm
        orient = candidate.orient
        support_z = candidate.evaluation.support_z

        # Pallet.place_box re-validates bounds, height, load and overlap.
        self.pallet.place_box(
            box,
            Point3D(x=candidate.ix * res, y=candidate.iy * res, z=support_z),
            orient.orientation,
            tolerance_mm=settings.PLACEMENT_TOLERANCE_MM,
        )

        top_z = support_z + orient.height_mm
        _update_feasibility_map(
            self, candidate.ix, candidate.iy, orient, top_z,
            candidate.evaluation.support_polygon,
        )
        self.placed.append(
            _PlacedRecord(
                column_key=_column_key(box),
                ix=candidate.ix,
                iy=candidate.iy,
                nx=orient.nx,
                ny=orient.ny,
                top_z=top_z,
            )
        )


# ============================================================================
# LBCP ALGORITHM 1 - STRUCTURAL STABILITY CHECK
# ============================================================================

def _validate_placement(
    state: _PalletState,
    ix: int,
    iy: int,
    orient: _Orient,
) -> _Evaluation | None:
    """Gravity drop, height limit, support ratio and (optionally) LBCP check.

    Returns the evaluation for a feasible placement, otherwise None.
    """
    window_hm = state.heightmap[ix:ix + orient.nx, iy:iy + orient.ny]

    # Resting height of the box bottom (gravity drop from above).
    support_z = float(window_hm.max())
    if support_z + orient.height_mm > state.max_height_mm + settings.PLACEMENT_TOLERANCE_MM:
        return None

    # Contact cells at the resting level.
    contact = window_hm >= support_z - CONTACT_HEIGHT_TOL_MM

    # Load-bearable contact: contact cells that belong to an LBCP.
    if USE_LBCP_STABILITY:
        window_fm = state.feasibility_map[ix:ix + orient.nx, iy:iy + orient.ny]
        bearable = contact & window_fm
    else:
        bearable = contact

    if not bearable.any():
        return None

    support_ratio = float(bearable.sum()) / float(bearable.size)
    if support_ratio < MIN_SUPPORT_RATIO - 1e-12:
        return None

    if not USE_LBCP_STABILITY:
        return _Evaluation(support_z, support_ratio, ())

    # Support polygon = convex hull of the load-bearable contact cells.
    res = state.grid_mm
    hull = _convex_hull(_mask_corner_points(bearable, ix, iy, res))

    # Centre-of-gravity uncertainty rectangle; its four corners are enough
    # because both the rectangle and the polygon are convex.
    cx = (ix + orient.nx / 2.0) * res
    cy = (iy + orient.ny / 2.0) * res
    dx = COG_TOLERANCE * orient.nx * res
    dy = COG_TOLERANCE * orient.ny * res
    cog_x = np.array([cx - dx, cx + dx, cx + dx, cx - dx])
    cog_y = np.array([cy - dy, cy - dy, cy + dy, cy + dy])
    if not _points_in_convex_polygon(cog_x, cog_y, hull).all():
        return None

    return _Evaluation(support_z, support_ratio, tuple(hull))


# ============================================================================
# LBCP ALGORITHM 2 - STRUCTURAL STABILITY UPDATE
# ============================================================================

def _update_feasibility_map(
    state: _PalletState,
    ix: int,
    iy: int,
    orient: _Orient,
    top_z: float,
    support_polygon: Sequence[Point2D],
) -> None:
    """Raise HM over the footprint. With LBCP enabled, the footprint in FM
    becomes non-bearable except above the support polygon, which is itself an
    LBCP on the new top face."""
    window = (slice(ix, ix + orient.nx), slice(iy, iy + orient.ny))
    state.heightmap[window] = top_z

    if not USE_LBCP_STABILITY:
        return

    res = state.grid_mm
    centres_x = (np.arange(ix, ix + orient.nx) + 0.5) * res
    centres_y = (np.arange(iy, iy + orient.ny) + 0.5) * res
    grid_x, grid_y = np.meshgrid(centres_x, centres_y, indexing="ij")
    state.feasibility_map[window] = _points_in_convex_polygon(
        grid_x.ravel(), grid_y.ravel(), support_polygon
    ).reshape(orient.nx, orient.ny)


# ============================================================================
# GEOMETRY HELPERS
# ============================================================================

def _mask_corner_points(mask: BoolArray, ix0: int, iy0: int, res: float) -> list[Point2D]:
    """Corners of the first and last True cell in every grid row [mm].

    Only these cells can contribute to the convex hull, which keeps the hull
    input proportional to the number of rows instead of the number of cells.
    """
    rows = np.nonzero(mask.any(axis=0))[0]
    first = np.argmax(mask[:, rows], axis=0)
    last = mask.shape[0] - 1 - np.argmax(mask[::-1, rows], axis=0)

    points: list[Point2D] = []
    for row, a, b in zip(rows.tolist(), first.tolist(), last.tolist(), strict=True):
        y0 = (iy0 + row) * res
        y1 = y0 + res
        for column in {a, b}:
            x0 = (ix0 + column) * res
            x1 = x0 + res
            points.extend(((x0, y0), (x1, y0), (x0, y1), (x1, y1)))
    return points


def _convex_hull(points: Sequence[Point2D]) -> list[Point2D]:
    """Andrew's monotone chain; counter-clockwise vertices, collinear removed."""
    pts = sorted(set(points))
    if len(pts) <= 2:
        return list(pts)

    def cross(o: Point2D, a: Point2D, b: Point2D) -> float:
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower: list[Point2D] = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper: list[Point2D] = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def _points_in_convex_polygon(
    px: AnyFloatArray,
    py: AnyFloatArray,
    hull: Sequence[Point2D],
) -> BoolArray:
    """Vectorised inclusion test for a CCW hull; the border counts as inside."""
    if len(hull) < 3:
        return np.zeros(np.shape(px), dtype=bool)
    inside = np.ones(np.shape(px), dtype=bool)
    for k in range(len(hull)):
        ax, ay = hull[k]
        bx, by = hull[(k + 1) % len(hull)]
        inside &= (bx - ax) * (py - ay) - (by - ay) * (px - ax) >= -1e-9
    return inside


# ============================================================================
# GRID AND ORIENTATIONS
# ============================================================================

def _grid_resolution(boxes: Sequence[Box]) -> float:
    """Cell size that represents the pallet and every footprint exactly."""
    values = [settings.PALLET_LENGTH_MM, settings.PALLET_WIDTH_MM]
    for box in boxes:
        for orientation in box.allowed_orientations:
            d = box.oriented_dimensions(orientation)
            values.extend((d.x, d.y))

    if GRID_RESOLUTION_MM is not None:
        res = float(GRID_RESOLUTION_MM)
        bad = sorted({v for v in values if abs(v / res - round(v / res)) > _GRID_EPS})
        if bad:
            shown = ", ".join(f"{v:g}" for v in bad[:5])
            raise AdvancedLoaderError(
                f"GRID_RESOLUTION_MM={res:g} does not divide footprint dimension(s) "
                f"{shown} mm. Use a divisor of all dimensions or None (automatic)."
            )
        return res

    non_integer = sorted({v for v in values if abs(v - round(v)) > _GRID_EPS})
    if non_integer:
        shown = ", ".join(f"{v:g}" for v in non_integer[:5])
        raise AdvancedLoaderError(
            f"Automatic grid needs whole-millimetre footprints; found {shown} mm. "
            "Set GRID_RESOLUTION_MM to a value that divides all dimensions."
        )
    return float(reduce(gcd, (int(round(v)) for v in values)))


def _orientations(box: Box, grid_mm: float) -> list[_Orient]:
    """Allowed orientations with distinct oriented dimensions."""
    unique: list[_Orient] = []
    seen: set[tuple[int, int, float]] = set()
    for orientation in box.allowed_orientations:
        d = box.oriented_dimensions(orientation)
        nx = _cells(d.x, grid_mm)
        ny = _cells(d.y, grid_mm)
        key = (nx, ny, round(d.z, 6))
        if key not in seen:
            seen.add(key)
            unique.append(_Orient(orientation, nx, ny, d.z))
    return unique


def _cells(value_mm: float, grid_mm: float) -> int:
    return int(round(value_mm / grid_mm))


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
    return pallet.current_load_kg + box.weight_kg <= pallet.max_load_kg + WEIGHT_EPS_KG


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
            raise AdvancedLoaderError(f"settings.{name} must be positive and finite.")

    if settings.PALLET_MAX_LOAD_KG is not None and (
        not isfinite(settings.PALLET_MAX_LOAD_KG) or settings.PALLET_MAX_LOAD_KG <= 0
    ):
        raise AdvancedLoaderError("settings.PALLET_MAX_LOAD_KG must be positive or None.")

    if settings.PLACEMENT_TOLERANCE_MM < 0 or not isfinite(settings.PLACEMENT_TOLERANCE_MM):
        raise AdvancedLoaderError("settings.PLACEMENT_TOLERANCE_MM must be non-negative.")

    if not 0.0 <= COG_TOLERANCE <= 0.5:
        raise AdvancedLoaderError("COG_TOLERANCE must be within 0.0..0.5.")
    if not 0.0 <= MIN_SUPPORT_RATIO <= 1.0:
        raise AdvancedLoaderError("MIN_SUPPORT_RATIO must be within 0.0..1.0.")

    if GRID_RESOLUTION_MM is not None and (
        not isfinite(GRID_RESOLUTION_MM) or GRID_RESOLUTION_MM <= 0
    ):
        raise AdvancedLoaderError("GRID_RESOLUTION_MM must be positive or None.")

    for name, value in (
        ("W_TOP_HEIGHT", W_TOP_HEIGHT),
        ("W_SUPPORT", W_SUPPORT),
        ("W_SNUGNESS", W_SNUGNESS),
        ("W_SAME_SKU_ALIGN", W_SAME_SKU_ALIGN),
        ("TIE_BREAK_Y", TIE_BREAK_Y),
        ("TIE_BREAK_X", TIE_BREAK_X),
        ("CONTACT_HEIGHT_TOL_MM", CONTACT_HEIGHT_TOL_MM),
    ):
        if not isfinite(value) or value < 0:
            raise AdvancedLoaderError(f"{name} must be non-negative and finite.")