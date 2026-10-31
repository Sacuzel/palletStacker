"""Experimental loader: the advanced loader with several open pallets and
selectable space rules.

Identical to ``advancedLoader.py`` (heightmap scoring with optional LBCP
stability), extended with two options of ``emsLoader.py``:

* pallet policy: several pallets can stay open, so a later box may fill a gap
  on an earlier pallet (MAX_OPEN_PALLETS, BIN_SELECTION_RULE);
* space selection: the filling order can follow one of the eight space rules
  of Ali, Ramos & Oliveira (2025) instead of the weighted score
  (SPACE_SELECTION_RULE).

With MAX_OPEN_PALLETS = 1 and SPACE_SELECTION_RULE = None this loader behaves
exactly like advancedLoader.

Loading model
-------------
* Boxes arrive one at a time in input order (grouped by SKU) and every
  decision is final: no look-ahead, no backtracking.
* Up to MAX_OPEN_PALLETS pallets are open (None = all). BIN_SELECTION_RULE
  decides which open pallet gets the box. When no open pallet can take it,
  a new pallet is opened; with a limit, the oldest open pallet is then
  closed for good.
* Boxes are loaded from above. A box drops vertically onto the highest surface
  under its footprint (heightmap), so nothing lies above a box at the moment
  it is placed and every placement is reachable from the top.
* Every box is packed. A box that does not fit even on an empty pallet stops
  the run with ``UnplaceableBoxError`` instead of being skipped.

Per box
-------
1. Pallet          open pallets in BIN_SELECTION_RULE order.
2. Orientations    allowed orientations with distinct footprints.
3. Candidates      x/y positions aligned with the pallet edges and with the
                   edges of placed boxes: min edge, max edge, and max edge
                   minus the box size (right/back alignment on top of a box).
4. Gravity drop    resting height = highest heightmap value under the
                   footprint.
5. Feasibility     height limit, load limit, MIN_SUPPORT_RATIO and, when
                   USE_LBCP_STABILITY is True, the LBCP stability check.
6. Score           lower is better:
                       W_TOP_HEIGHT     * resulting top height   [mm]
                     - W_SUPPORT        * support ratio          [0..1]
                     - W_SNUGNESS       * snugness               [0..1]
                     - W_SAME_SKU_ALIGN * same-SKU column bonus  {0, 1}
                     + TIE_BREAK_Y * y + TIE_BREAK_X * x         [mm]
7. Ranking         SPACE_SELECTION_RULE = None: the lowest score wins.
                   SPACE_SELECTION_RULE = 1..8: candidates are ranked by the
                   space key first; the score only decides between candidates
                   with an equal space key (e.g. orientations at one corner).
8. Commit          the best candidate on the chosen pallet; that pallet's
                   heightmap, feasibility map and owner map are updated.

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

Space selection (SPACE_SELECTION_RULE)
--------------------------------------
Space rules of Table 1 in S. Ali, A. G. Ramos, J. F. Oliveira: "Static
stability versus packing efficiency in online three-dimensional packing
problems: A new approach and a computational study", Computers & Operations
Research 178 (2025) 107005. The paper ranks Empty Maximal Spaces; here every
candidate defines its own space, translated to the heightmap model:

* Corner: the box's back-bottom-left corner (x, y, z) at its resting height.
  X is depth from the back edge (x = 0), Y runs left to right, Z is up.
* Back-bottom pallet corners: (0, 0, 0) on the left, (0, width, 0) on the
  right. The distance of a candidate is measured from its own back-bottom-left
  or back-bottom-right corner, whichever is nearer its pallet corner.
* Volume: the free space around the footprint, open to the top. Starting from
  the footprint, the rectangle is grown one cell row at a time (-X, +X, -Y,
  +Y, repeated) over cells no higher than the resting level; its area times
  the free height up to the pallet height limit is the volume. Spaces are
  open to the top because boxes are loaded from above: there are no cavities
  under overhangs as in the paper's container model.
* Generated earlier: a space exists since the newest of the boxes it rests on
  was placed (the deck is oldest). Every rule ends with this criterion.

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
Heightmap, feasibility map and owner map are cell grids. The grid must
represent every footprint exactly, otherwise neighbouring boxes would appear
to share cells. With GRID_RESOLUTION_MM = None the resolution is the greatest
common divisor of the pallet size and all box footprint dimensions in the
input (whole millimetres). Only the set of dimensions is used, solely to
choose this grid; the arrival order is never looked at. A small divisor
(e.g. 1 mm) is exact but slower than 5 or 10 mm.
"""

from __future__ import annotations

from collections.abc import Iterable, Sequence
from dataclasses import dataclass
from functools import reduce
from math import gcd, hypot, isfinite
from typing import Any

import numpy as np
from numpy.typing import NDArray

from . import settings
from .box import Box, Orientation, Point3D
from .pallet import Pallet

FloatArray = NDArray[np.float64]
AnyFloatArray = NDArray[np.floating[Any]]
BoolArray = NDArray[np.bool_]
IntArray = NDArray[np.int64]
Point2D = tuple[float, float]


# ============================================================================
# ALGORITHM SETTINGS
# ============================================================================

# --- Pallet policy ------------------------------------------------------------

# How many pallets stay open. None = every pallet stays open until the end of
# the run. 1 = only the newest pallet is open (identical to advancedLoader).
# k = the k most recently opened pallets are open; older ones are closed.
MAX_OPEN_PALLETS: int | None = 5

# Which open pallet gets the box:
#   "first_fit"  oldest open pallet that can take the box
#   "best_fit"   fullest open pallet (packed volume) that can take the box
#   "worst_fit"  emptiest open pallet that can take the box
#   "all_bins"   every open pallet is evaluated; the best-ranked candidate
#                wins (equal rank: the older pallet)
BIN_SELECTION_RULE: str = "first_fit"

# --- Space selection ------------------------------------------------------------

# None: candidates are ranked by the weighted score alone (advancedLoader).
# 1..8: candidates are ranked by the space key first (Table 1 of Ali, Ramos &
# Oliveira 2025, as in emsLoader); the score only breaks ties.
#   1  min x, min z, min y
#   2  min of (x, y, z), next min, next min
#   3  min z, min of (x, y), next min
#   4  smallest space volume
#   5  min x, min z, nearest back-bottom pallet corner (left or right)
#   6  nearest back-bottom pallet corner, largest space volume
#   7  nearest back-bottom pallet corner, smallest space volume
#   8  max distance from the box's back-bottom-left corner to the pallet's
#      front-top-right corner
# Every rule ends with "space generated earlier".
SPACE_SELECTION_RULE: int | None = 3

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
W_SNUGNESS: float = 100.0        # reward for touching neighbours
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

_BIN_RULES = ("first_fit", "best_fit", "worst_fit", "all_bins")


class ExperimentalLoaderError(RuntimeError):
    """Base exception raised by the experimental loader."""


class UnplaceableBoxError(ExperimentalLoaderError):
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
    """One fully evaluated candidate placement on one pallet."""

    state: _PalletState
    ix: int
    iy: int
    orient: _Orient
    evaluation: _Evaluation
    score: float                  # weighted score, lower = better
    rank: tuple[float, ...]       # space key + (score,); lower = better


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
    """Load all boxes one by one in arrival order onto the open pallets."""
    _validate_settings()
    box_sequence = tuple(boxes)

    if not box_sequence:
        return []

    duplicate_ids = _find_duplicate_ids(box_sequence)
    if duplicate_ids:
        joined = ", ".join(sorted(duplicate_ids))
        raise ExperimentalLoaderError(f"Duplicate box IDs supplied to loader: {joined}")

    already_placed = [box.box_id for box in box_sequence if box.is_placed]
    if already_placed:
        joined = ", ".join(already_placed)
        raise ExperimentalLoaderError(f"Boxes are already placed: {joined}")

    grid_mm = _grid_resolution(box_sequence)

    pallets: list[Pallet] = []
    states: list[_PalletState] = []

    for box in box_sequence:
        orients = _orientations(box, grid_mm)

        # ---- 1. pallet selection over the open pallets -----------------------
        open_states = states if MAX_OPEN_PALLETS is None else states[-MAX_OPEN_PALLETS:]
        candidate = _select_pallet(open_states, box, orients)

        # ---- open a new pallet if no open pallet can take the box ------------
        if candidate is None:
            new_pallet = _create_new_pallet(len(pallets) + 1)
            pallets.append(new_pallet)
            new_state = _PalletState(new_pallet, grid_mm, index=len(states))
            states.append(new_state)
            candidate = _find_best_placement(new_state, box, orients)
            if candidate is None:
                raise UnplaceableBoxError(_unplaceable_message(box, new_pallet))

        # ---- commit (online: irrevocable) -----------------------------------
        candidate.state.commit(box, candidate)

    return pallets


# ============================================================================
# PALLET SELECTION
# ============================================================================

def _select_pallet(
    states: Sequence[_PalletState],
    box: Box,
    orients: Sequence[_Orient],
) -> _Candidate | None:
    """Return the chosen placement over the open pallets, or None."""
    if BIN_SELECTION_RULE == "all_bins":
        best: _Candidate | None = None
        for state in states:  # opening order: equal rank keeps the older pallet
            candidate = _find_best_placement(state, box, orients)
            if candidate is not None and (best is None or candidate.rank < best.rank):
                best = candidate
        return best

    if BIN_SELECTION_RULE == "first_fit":       # oldest open pallet first
        ordered = list(states)
    elif BIN_SELECTION_RULE == "best_fit":      # fullest open pallet first
        ordered = sorted(states, key=lambda s: (-s.loaded_volume_mm3, s.index))
    else:                                       # worst fit: emptiest first
        ordered = sorted(states, key=lambda s: (s.loaded_volume_mm3, s.index))

    for state in ordered:
        candidate = _find_best_placement(state, box, orients)
        if candidate is not None:
            return candidate
    return None


# ============================================================================
# CANDIDATE SEARCH, SCORING AND RANKING
# ============================================================================

def _find_best_placement(
    state: _PalletState,
    box: Box,
    orients: Sequence[_Orient],
) -> _Candidate | None:
    """Evaluate every candidate position and orientation on one pallet and
    return the best-ranked feasible one, or None if the pallet cannot take
    the box."""
    if not _weight_fits(state.pallet, box):
        return None

    res = state.grid_mm
    column_key = _column_key(box)
    lowest_surface_mm = float(state.heightmap.min())
    best: _Candidate | None = None

    for orient in orients:
        if orient.nx > state.nx or orient.ny > state.ny:
            continue
        # Fast reject: the box rests at least on the lowest surface cell, so a
        # box that is too tall there is too tall everywhere on this pallet.
        if (
            lowest_surface_mm + orient.height_mm
            > state.max_height_mm + settings.PLACEMENT_TOLERANCE_MM
        ):
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

                rank = _space_key(state, ix, iy, orient, support_z) + (score,)
                if best is None or rank < best.rank:
                    best = _Candidate(state, ix, iy, orient, evaluation, score, rank)

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
# SPACE SELECTION (Table 1 of Ali, Ramos & Oliveira 2025)
# ============================================================================

def _space_key(
    state: _PalletState,
    ix: int,
    iy: int,
    orient: _Orient,
    support_z: float,
) -> tuple[float, ...]:
    """Space-rule sort key of one candidate (lower = higher priority).

    Empty when SPACE_SELECTION_RULE is None, so the score alone ranks.
    """
    rule = SPACE_SELECTION_RULE
    if rule is None:
        return ()

    res = state.grid_mm
    x = ix * res
    y = iy * res
    z = support_z

    if rule == 1:
        key: tuple[float, ...] = (x, z, y)
    elif rule == 2:
        key = tuple(sorted((x, y, z)))
    elif rule == 3:
        key = (z, min(x, y), max(x, y))
    elif rule == 4:
        key = (_space_volume(state, ix, iy, orient, support_z),)
    elif rule == 5:
        key = (x, z, _back_corner_distance(state, x, y, z, orient.ny * res))
    elif rule == 6:
        key = (
            _back_corner_distance(state, x, y, z, orient.ny * res),
            -_space_volume(state, ix, iy, orient, support_z),
        )
    elif rule == 7:
        key = (
            _back_corner_distance(state, x, y, z, orient.ny * res),
            _space_volume(state, ix, iy, orient, support_z),
        )
    else:  # rule 8
        key = (
            -hypot(
                state.pallet.length_mm - x,
                state.pallet.width_mm - y,
                state.max_height_mm - z,
            ),
        )

    # "Generated earlier" as the final space criterion.
    return key + (float(_space_age(state, ix, iy, orient, support_z)),)


def _back_corner_distance(
    state: _PalletState,
    x: float,
    y: float,
    z: float,
    width_mm: float,
) -> float:
    """Distance to the nearer back-bottom pallet corner, measured from the
    box's back-bottom-left corner to (0, 0, 0) or from its back-bottom-right
    corner to (0, width, 0)."""
    left = hypot(x, y, z)
    right = hypot(x, state.pallet.width_mm - (y + width_mm), z)
    return min(left, right)


def _space_volume(
    state: _PalletState,
    ix: int,
    iy: int,
    orient: _Orient,
    support_z: float,
) -> float:
    """Volume of the open-top free space around the footprint [mm^3].

    The rectangle grows from the footprint one cell row at a time in the order
    -X, +X, -Y, +Y (repeated) while the new row is no higher than the resting
    level. The result is a maximal rectangle; its area times the free height
    up to the pallet height limit is the volume.
    """
    hm = state.heightmap
    limit = support_z + CONTACT_HEIGHT_TOL_MM
    x0, x1 = ix, ix + orient.nx
    y0, y1 = iy, iy + orient.ny

    grown = True
    while grown:
        grown = False
        if x0 > 0 and float(hm[x0 - 1, y0:y1].max()) <= limit:
            x0 -= 1
            grown = True
        if x1 < state.nx and float(hm[x1, y0:y1].max()) <= limit:
            x1 += 1
            grown = True
        if y0 > 0 and float(hm[x0:x1, y0 - 1].max()) <= limit:
            y0 -= 1
            grown = True
        if y1 < state.ny and float(hm[x0:x1, y1].max()) <= limit:
            y1 += 1
            grown = True

    area_mm2 = (x1 - x0) * (y1 - y0) * state.grid_mm * state.grid_mm
    return area_mm2 * (state.max_height_mm - support_z)


def _space_age(
    state: _PalletState,
    ix: int,
    iy: int,
    orient: _Orient,
    support_z: float,
) -> int:
    """Placement index of the newest box the candidate rests on (-1 = deck).
    Lower means the space was generated earlier."""
    window = (slice(ix, ix + orient.nx), slice(iy, iy + orient.ny))
    contact = state.heightmap[window] >= support_z - CONTACT_HEIGHT_TOL_MM
    return int(state.owner_map[window][contact].max())


# ============================================================================
# PER-PALLET STATE (heightmap + feasibility map + owner map)
# ============================================================================

class _PalletState:
    """Loader-internal model of one pallet."""

    def __init__(self, pallet: Pallet, grid_mm: float, index: int) -> None:
        self.pallet = pallet
        self.grid_mm = grid_mm
        self.index = index                    # opening order (first fit, ties)
        self.nx = _cells(pallet.length_mm, grid_mm)
        self.ny = _cells(pallet.width_mm, grid_mm)
        self.max_height_mm = pallet.max_height_mm
        self.loaded_volume_mm3 = 0.0          # packed box volume (best/worst fit)
        shape = (self.nx, self.ny)

        # HM: top height of the load per cell [mm].
        self.heightmap: FloatArray = np.zeros(shape, dtype=np.float64)
        # FM: True where the top surface is load-bearable. The deck is one
        # load-bearable polygon, so the map starts all True.
        self.feasibility_map: BoolArray = np.ones(shape, dtype=bool)
        # Index into self.placed of the box forming the surface; -1 = deck.
        self.owner_map: IntArray = np.full(shape, -1, dtype=np.int64)
        self.placed: list[_PlacedRecord] = []

    def commit(self, box: Box, candidate: _Candidate) -> None:
        """Place the box on this pallet and update HM and FM (Algorithm 2)
        and the owner map."""
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
        window = (
            slice(candidate.ix, candidate.ix + orient.nx),
            slice(candidate.iy, candidate.iy + orient.ny),
        )
        self.owner_map[window] = len(self.placed)
        self.loaded_volume_mm3 += box.volume_mm3
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
            raise ExperimentalLoaderError(
                f"GRID_RESOLUTION_MM={res:g} does not divide footprint dimension(s) "
                f"{shown} mm. Use a divisor of all dimensions or None (automatic)."
            )
        return res

    non_integer = sorted({v for v in values if abs(v - round(v)) > _GRID_EPS})
    if non_integer:
        shown = ", ".join(f"{v:g}" for v in non_integer[:5])
        raise ExperimentalLoaderError(
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
            raise ExperimentalLoaderError(f"settings.{name} must be positive and finite.")

    if settings.PALLET_MAX_LOAD_KG is not None and (
        not isfinite(settings.PALLET_MAX_LOAD_KG) or settings.PALLET_MAX_LOAD_KG <= 0
    ):
        raise ExperimentalLoaderError("settings.PALLET_MAX_LOAD_KG must be positive or None.")

    if settings.PLACEMENT_TOLERANCE_MM < 0 or not isfinite(settings.PLACEMENT_TOLERANCE_MM):
        raise ExperimentalLoaderError("settings.PLACEMENT_TOLERANCE_MM must be non-negative.")

    if BIN_SELECTION_RULE not in _BIN_RULES:
        raise ExperimentalLoaderError(f"BIN_SELECTION_RULE must be one of {_BIN_RULES}.")
    if MAX_OPEN_PALLETS is not None and MAX_OPEN_PALLETS < 1:
        raise ExperimentalLoaderError("MAX_OPEN_PALLETS must be None or at least 1.")
    if SPACE_SELECTION_RULE is not None and SPACE_SELECTION_RULE not in range(1, 9):
        raise ExperimentalLoaderError("SPACE_SELECTION_RULE must be None or within 1..8.")

    if not 0.0 <= COG_TOLERANCE <= 0.5:
        raise ExperimentalLoaderError("COG_TOLERANCE must be within 0.0..0.5.")
    if not 0.0 <= MIN_SUPPORT_RATIO <= 1.0:
        raise ExperimentalLoaderError("MIN_SUPPORT_RATIO must be within 0.0..1.0.")

    if GRID_RESOLUTION_MM is not None and (
        not isfinite(GRID_RESOLUTION_MM) or GRID_RESOLUTION_MM <= 0
    ):
        raise ExperimentalLoaderError("GRID_RESOLUTION_MM must be positive or None.")

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
            raise ExperimentalLoaderError(f"{name} must be non-negative and finite.")