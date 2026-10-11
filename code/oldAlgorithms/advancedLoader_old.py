"""Online advanced pallet loader based on Ali, Ramos & Oliveira (2025).

Reference
---------
S. Ali, A. G. Ramos, J. F. Oliveira: "Static stability versus packing
efficiency in online three-dimensional packing problems: A new approach and a
computational study", Computers & Operations Research 178 (2025) 107005.

The paper builds online heuristics from three independent selection rules plus
one static-stability constraint (Sections 3 and 4, Appendix A). This module
implements the complete rule set, so each of the paper's 4 x 8 x 5 = 160
heuristics under each of the 4 stability constraints can be selected through
the settings block below. Heuristic names in the paper read <bin><space>
<orientation>, e.g. F52 = first fit, space rule 5, orientation rule 2.

Per arriving box (Appendix A)
-----------------------------
1. Bin selection       choose an open pallet (first / best / worst fit, all bins)
2. Space selection     order the pallet's Empty Maximal Spaces (Table 1)
3. Orientation fit     allowed orientations that fit inside the EMS
4. Stability           static stability constraint (Section 3)
5. Orientation select  choose among the feasible placements (Table 2)
6. Placement           commit to the Pallet (re-validated by Pallet.place_box)
7. Space update        difference and elimination process (Lai & Chan 1997)
When a step finds nothing feasible, the next orientation, EMS and pallet are
tried. When every open pallet fails, a new pallet is opened.

Coordinate convention (same as the paper)
-----------------------------------------
X = depth (0 = back wall, length_mm = entrance), Y = width (left -> right),
Z = height above the loading surface. The deepest-bottom-left corner of an EMS
is its minimum corner (x1, y1, z1); the front-top-right corner is (x2, y2, z2).

Interpretations where the paper leaves details open
---------------------------------------------------
* Convex hull: Andrew's monotone chain instead of Jarvis' gift wrapping. Both
  return the same hull with collinear points removed.
* "Random selection" ties of Table 2 use a seeded generator (RANDOM_SEED) so
  that runs are reproducible.
* Orientation rules 4 and 5 consider the first N_EMS_CANDIDATES EMSs, in space
  priority order, that contain at least one feasible placement.
* "All bins": every open pallet proposes its best placement; the pallet whose
  chosen EMS ranks best under the space rule wins (then orientation score,
  then pallet age).
* Margin (orientation rules 1 and 4): smallest clearance between the oriented
  box and the EMS over the three axes.
* Blocking (Section 4.2) models loading through a container door at
  X = length_mm. Pallets are normally loaded from above, so it is disabled by
  default (ENFORCE_BLOCKING).
"""

from __future__ import annotations

import random
from dataclasses import dataclass
from math import hypot, isfinite
from collections.abc import Iterable, Sequence

from . import settings
from .box import Box, Orientation, Point3D
from .pallet import Pallet

Dims = tuple[float, float, float]
Point2D = tuple[float, float]


# ============================================================================
# ALGORITHM SETTINGS
# ============================================================================

# --- Heuristic selection (Section 4.1) ---------------------------------------
# Defaults = paper's most promising online heuristic F52 (first fit, space
# rule 5, orientation rule 2) with the recommended partial-base polygon support.

# "first_fit" | "best_fit" | "worst_fit" | "all_bins"
BIN_SELECTION_RULE: str = "first_fit"

# 1..8, Table 1:
#   1  min x1, min z1, min y1
#   2  min of (x1, y1, z1), next min, next min
#   3  min z1, min of (x1, y1), next min
#   4  smallest volume, generated earlier
#   5  min x1, min z1, nearest back-bottom bin corner (left or right)
#   6  nearest back-bottom bin corner, largest volume, generated earlier
#   7  nearest back-bottom bin corner, smallest volume, generated earlier
#   8  max distance from EMS deepest-bottom-left corner to bin
#      front-top-right corner, generated earlier
# Rules 5, 6 and 7 place the box in the back-bottom-left OR back-bottom-right
# corner of the EMS (whichever is nearer its bin corner); all others use the
# deepest-bottom-left corner.
SPACE_SELECTION_RULE: int = 3

# 1..5, Table 2:
#   1  (1 EMS) minimum margin, random tie
#   2  (1 EMS) largest base area, smallest X occupancy of the EMS
#   3  (1 EMS) largest base area, greatest X occupancy of the EMS
#   4  (n EMS) per EMS minimum margin; then highest fill ratio, random tie
#   5  (n EMS) max distance from box front-top-right to bin front-top-right,
#      random tie
ORIENTATION_SELECTION_RULE: int = 2

# Number of EMSs considered by orientation rules 4 and 5.
N_EMS_CANDIDATES: int = 3

# --- Static stability (Section 3) --------------------------------------------
# "full_base"            100 % of the base supported
# "partial_base"         PARTIAL_BASE_SUPPORT_RATIO of the base supported
# "cog_polygon"          centre of gravity inside the support polygon
# "partial_base_polygon" CoG inside the support polygon AND polygon area
#                        > POLYGON_MIN_AREA_RATIO of the base (paper's proposal)
STABILITY_CONSTRAINT: str = "partial_base_polygon"
PARTIAL_BASE_SUPPORT_RATIO: float = 0.80
POLYGON_MIN_AREA_RATIO: float = 0.50

# Top faces within this distance of a box bottom count as supporting [mm].
# Keep equal to layoutAnalyzer.ANALYSIS_SUPPORT_FLUSH_TOL_MM.
SUPPORT_FLUSH_TOL_MM: float = 0.5

# --- Pallet policy -------------------------------------------------------------
# None = every pallet stays open (paper). 1 = only the newest pallet is open
# (single manual station). k = the k newest pallets are open.
MAX_OPEN_PALLETS: int | None = 1

# Blocking constraint for door loading along +X (Section 4.2): newly generated
# EMSs that do not reach the entrance (x2 < length) are discarded.
ENFORCE_BLOCKING: bool = False

# --- Empty maximal spaces -------------------------------------------------------
# EMSs thinner than this on any axis are discarded [mm]. Performance setting;
# keep it below the smallest box dimension you expect.
EMS_MIN_SIDE_MM: float = 10.0

# --- Determinism / numerics ---------------------------------------------------
RANDOM_SEED: int | None = 0       # seed for "random selection" ties
SCORE_REL_EPS: float = 1e-9       # relative tolerance when comparing scores
WEIGHT_EPS_KG: float = 1e-6       # weight-limit tolerance [kg]

_BIN_RULES = ("first_fit", "best_fit", "worst_fit", "all_bins")
_STABILITY_RULES = ("full_base", "partial_base", "cog_polygon", "partial_base_polygon")


class AdvancedLoaderError(RuntimeError):
    """Base exception raised by the advanced loader."""


class UnplaceableBoxError(AdvancedLoaderError):
    """Raised when a box cannot fit even on an empty pallet."""


# ============================================================================
# INTERNAL TYPES
# ============================================================================

@dataclass(slots=True)
class _Space:
    """One Empty Maximal Space: [x1, x2] x [y1, y2] x [z1, z2] is empty."""

    x1: float
    y1: float
    z1: float
    x2: float
    y2: float
    z2: float
    created: int          # generation counter ("generated earlier" criterion)

    @property
    def dx(self) -> float:
        return self.x2 - self.x1

    @property
    def dy(self) -> float:
        return self.y2 - self.y1

    @property
    def dz(self) -> float:
        return self.z2 - self.z1

    @property
    def volume(self) -> float:
        return self.dx * self.dy * self.dz


@dataclass(frozen=True, slots=True)
class _Solid:
    """Bounds of one placed box."""

    x1: float
    y1: float
    z1: float
    x2: float
    y2: float
    z2: float


@dataclass(frozen=True, slots=True)
class _Placement:
    """One feasible (fitting and stable) placement of the current box."""

    space: _Space
    orientation: Orientation
    dims: Dims
    position: Dims


@dataclass(frozen=True, slots=True)
class _Choice:
    """The placement a pallet proposes, plus the scores used by 'all bins'."""

    state: _PalletState
    placement: _Placement
    space_key: tuple[float, ...]
    orientation_key: tuple[float, ...]


# ============================================================================
# MAIN ENTRY POINT
# ============================================================================

def load_boxes(boxes: Iterable[Box]) -> list[Pallet]:
    """Load all boxes one by one in arrival order (online, no backtracking)."""
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

    rng = random.Random(RANDOM_SEED)
    pallets: list[Pallet] = []
    states: list[_PalletState] = []

    for box in box_sequence:
        orientations = _orientations(box)

        # ---- 1. bin selection over the open pallets -------------------
        open_states = states if MAX_OPEN_PALLETS is None else states[-MAX_OPEN_PALLETS:]
        choice = _select_pallet(open_states, box, orientations, rng)

        # ---- open a new pallet if no open pallet can take the box -------
        if choice is None:
            new_pallet = _create_new_pallet(len(pallets) + 1)
            pallets.append(new_pallet)
            states.append(_PalletState(new_pallet, index=len(states)))
            choice = _best_in_pallet(states[-1], box, orientations, rng)
            if choice is None:
                raise UnplaceableBoxError(_unplaceable_message(box, new_pallet))

        # ---- 6./7. placement and space update ---------------------------
        _commit(choice, box)

    return pallets


# ============================================================================
# 1. BIN SELECTION
# ============================================================================

def _select_pallet(
    states: Sequence[_PalletState],
    box: Box,
    orientations: list[tuple[Orientation, Dims]],
    rng: random.Random,
) -> _Choice | None:
    """Return the chosen placement over all open pallets, or None."""
    if BIN_SELECTION_RULE == "all_bins":
        choices = [
            choice
            for state in states
            if (choice := _best_in_pallet(state, box, orientations, rng)) is not None
        ]
        if not choices:
            return None
        return min(
            choices,
            key=lambda c: (c.space_key, c.orientation_key, c.state.index),
        )

    if BIN_SELECTION_RULE == "first_fit":       # leftmost (oldest) pallet first
        ordered = list(states)
    elif BIN_SELECTION_RULE == "best_fit":      # fullest pallet first
        ordered = sorted(states, key=lambda s: (-s.loaded_volume_mm3, s.index))
    else:                                       # worst fit: emptiest first
        ordered = sorted(states, key=lambda s: (s.loaded_volume_mm3, s.index))

    for state in ordered:
        choice = _best_in_pallet(state, box, orientations, rng)
        if choice is not None:
            return choice
    return None


# ============================================================================
# 2.-5. SPACE, FIT, STABILITY AND ORIENTATION ON ONE PALLET
# ============================================================================

def _best_in_pallet(
    state: _PalletState,
    box: Box,
    orientations: list[tuple[Orientation, Dims]],
    rng: random.Random,
) -> _Choice | None:
    """Best feasible placement of ``box`` on one pallet, or None."""
    if not _weight_fits(state.pallet, box):
        return None

    tol = settings.PLACEMENT_TOLERANCE_MM
    needed = N_EMS_CANDIDATES if ORIENTATION_SELECTION_RULE in (4, 5) else 1

    # ---- 2. space selection: EMSs in priority order (Table 1) ------------
    ordered = sorted(state.spaces, key=lambda s: _space_key(state, s))

    groups: list[tuple[_Space, list[_Placement]]] = []
    for space in ordered:
        options: list[_Placement] = []
        for orientation, (l, w, h) in orientations:
            # ---- 3. orientation fit evaluation ---------------------------
            if l > space.dx + tol or w > space.dy + tol or h > space.dz + tol:
                continue
            x, y, z = _anchor(state, space, l, w)
            # ---- 4. stability assessment ---------------------------------
            if _is_stable(state, x, y, z, l, w):
                options.append(_Placement(space, orientation, (l, w, h), (x, y, z)))
        if options:
            groups.append((space, options))
            if len(groups) >= needed:
                break

    if not groups:
        return None

    # ---- 5. orientation selection (Table 2) ---------------------------------
    placement, orientation_key = _select_orientation(state, groups, rng)
    return _Choice(
        state=state,
        placement=placement,
        space_key=_space_key(state, placement.space),
        orientation_key=orientation_key,
    )


def _space_key(state: _PalletState, s: _Space) -> tuple[float, ...]:
    """Sort key of Table 1 (lower = higher priority)."""
    rule = SPACE_SELECTION_RULE
    if rule == 1:
        key: tuple[float, ...] = (s.x1, s.z1, s.y1)
    elif rule == 2:
        key = tuple(sorted((s.x1, s.y1, s.z1)))
    elif rule == 3:
        key = (s.z1, min(s.x1, s.y1), max(s.x1, s.y1))
    elif rule == 4:
        key = (s.volume,)
    elif rule == 5:
        key = (s.x1, s.z1, _back_corner(state, s)[0])
    elif rule == 6:
        key = (_back_corner(state, s)[0], -s.volume)
    elif rule == 7:
        key = (_back_corner(state, s)[0], s.volume)
    else:  # rule 8
        key = (-hypot(state.length - s.x1, state.width - s.y1, state.height - s.z1),)
    # "Generated earlier" as the final, deterministic tie-breaker.
    return key + (float(s.created),)


def _back_corner(state: _PalletState, s: _Space) -> tuple[float, bool]:
    """Distance of the EMS to the nearer back-bottom bin corner and whether
    that is the right corner (0, width, 0) rather than the left (0, 0, 0)."""
    left = hypot(s.x1, s.y1, s.z1)
    right = hypot(s.x1, state.width - s.y2, s.z1)
    return (right, True) if right < left else (left, False)


def _anchor(state: _PalletState, s: _Space, l: float, w: float) -> Dims:
    """Position of the box inside the EMS (deepest-bottom-left by default)."""
    if SPACE_SELECTION_RULE in (5, 6, 7) and _back_corner(state, s)[1]:
        return (s.x1, s.y2 - w, s.z1)          # back-bottom-right
    return (s.x1, s.y1, s.z1)                  # back-bottom-left


def _select_orientation(
    state: _PalletState,
    groups: list[tuple[_Space, list[_Placement]]],
    rng: random.Random,
) -> tuple[_Placement, tuple[float, ...]]:
    """Table 2. Return the chosen placement and its score (lower = better)."""
    rule = ORIENTATION_SELECTION_RULE

    if rule == 1:
        return _pick(groups[0][1], lambda p: (_margin(p),), rng, random_ties=True)

    if rule == 2:
        return _pick(
            groups[0][1],
            lambda p: (-p.dims[0] * p.dims[1], p.dims[0] / p.space.dx),
            rng,
            random_ties=False,
        )

    if rule == 3:
        return _pick(
            groups[0][1],
            lambda p: (-p.dims[0] * p.dims[1], -p.dims[0] / p.space.dx),
            rng,
            random_ties=False,
        )

    if rule == 4:
        # (i) per EMS the minimum-margin orientation, (ii) highest fill ratio.
        per_space = [
            _pick(options, lambda p: (_margin(p),), rng, random_ties=True)[0]
            for _, options in groups
        ]
        return _pick(
            per_space,
            lambda p: (-(p.dims[0] * p.dims[1] * p.dims[2]) / p.space.volume,),
            rng,
            random_ties=True,
        )

    # rule 5: max distance from box front-top-right to bin front-top-right.
    candidates = [p for _, options in groups for p in options]
    return _pick(
        candidates,
        lambda p: (
            -hypot(
                state.length - (p.position[0] + p.dims[0]),
                state.width - (p.position[1] + p.dims[1]),
                state.height - (p.position[2] + p.dims[2]),
            ),
        ),
        rng,
        random_ties=True,
    )


def _margin(p: _Placement) -> float:
    """Smallest clearance between the oriented box and its EMS."""
    return min(p.space.dx - p.dims[0], p.space.dy - p.dims[1], p.space.dz - p.dims[2])


def _pick(
    candidates: list[_Placement],
    key,
    rng: random.Random,
    *,
    random_ties: bool,
) -> tuple[_Placement, tuple[float, ...]]:
    """Return the candidate with the lowest key; ties random or first."""
    keys = [key(c) for c in candidates]
    best = min(keys)
    tied = [c for c, k in zip(candidates, keys, strict=True) if _same_score(k, best)]
    chosen = rng.choice(tied) if random_ties and len(tied) > 1 else tied[0]
    return chosen, best


def _same_score(a: tuple[float, ...], b: tuple[float, ...]) -> bool:
    return all(
        abs(x - y) <= SCORE_REL_EPS * max(1.0, abs(x), abs(y))
        for x, y in zip(a, b, strict=True)
    )


# ============================================================================
# 4. STATIC STABILITY (Section 3)
# ============================================================================

def _is_stable(
    state: _PalletState,
    x: float,
    y: float,
    z: float,
    l: float,
    w: float,
) -> bool:
    """Apply STABILITY_CONSTRAINT to a box with footprint (x, y, l, w) at z."""
    tol = settings.PLACEMENT_TOLERANCE_MM

    # Condition 1 of every constraint: resting on the pallet deck.
    if z <= tol:
        return True

    # Intersections of the base with the top faces of supporting boxes.
    # Each rectangle is (x1, y1, x2, y2); its corners are v_j1..v_j4.
    rects: list[tuple[float, float, float, float]] = []
    for b in state.solids:
        if abs(b.z2 - z) > SUPPORT_FLUSH_TOL_MM:
            continue
        rx1, rx2 = max(x, b.x1), min(x + l, b.x2)
        ry1, ry2 = max(y, b.y1), min(y + w, b.y2)
        if rx2 - rx1 > tol and ry2 - ry1 > tol:
            rects.append((rx1, ry1, rx2, ry2))

    if not rects:
        return False

    base_area = l * w
    rule = STABILITY_CONSTRAINT

    # ---- 3.1 / 3.2 area-based support -----------------------------------
    if rule in ("full_base", "partial_base"):
        supported = sum((x2 - x1) * (y2 - y1) for x1, y1, x2, y2 in rects)
        required = 1.0 if rule == "full_base" else PARTIAL_BASE_SUPPORT_RATIO
        return supported >= required * base_area * (1.0 - 1e-9)

    # ---- 3.3 / 3.4 polygon-based support --------------------------------
    cx = x + l / 2.0
    cy = y + w / 2.0

    # Condition 2: CoG directly above one supporting box.
    direct = any(
        x1 - tol <= cx <= x2 + tol and y1 - tol <= cy <= y2 + tol
        for x1, y1, x2, y2 in rects
    )
    if rule == "cog_polygon" and direct:
        return True

    # Condition 3: CoG inside or on the support polygon SP.
    points = [
        corner
        for x1, y1, x2, y2 in rects
        for corner in ((x1, y1), (x2, y1), (x1, y2), (x2, y2))
    ]
    hull = _convex_hull(points)
    if not (direct or _point_in_convex_polygon((cx, cy), hull)):
        return False
    if rule == "cog_polygon":
        return True

    # Partial-base polygon: SP area must exceed the ratio of the base area.
    return _polygon_area(hull) > POLYGON_MIN_AREA_RATIO * base_area


def _convex_hull(points: Sequence[Point2D]) -> list[Point2D]:
    """Andrew's monotone chain; counter-clockwise, collinear points removed."""
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


def _point_in_convex_polygon(pt: Point2D, hull: Sequence[Point2D]) -> bool:
    """Vector-product test of the paper; the boundary counts as inside."""
    if len(hull) < 3:
        return False
    px, py = pt
    for i in range(len(hull)):
        ax, ay = hull[i]
        bx, by = hull[(i + 1) % len(hull)]
        if (bx - ax) * (py - ay) - (by - ay) * (px - ax) < -1e-9:
            return False
    return True


def _polygon_area(hull: Sequence[Point2D]) -> float:
    """Shoelace formula."""
    if len(hull) < 3:
        return 0.0
    total = 0.0
    for i in range(len(hull)):
        ax, ay = hull[i]
        bx, by = hull[(i + 1) % len(hull)]
        total += ax * by - bx * ay
    return abs(total) / 2.0


# ============================================================================
# 6./7. COMMIT AND PER-PALLET STATE (EMS difference & elimination)
# ============================================================================

def _commit(choice: _Choice, box: Box) -> None:
    p = choice.placement
    # Pallet.place_box re-validates bounds, load and overlap before committing.
    choice.state.pallet.place_box(
        box,
        Point3D(x=p.position[0], y=p.position[1], z=p.position[2]),
        p.orientation,
        tolerance_mm=settings.PLACEMENT_TOLERANCE_MM,
    )
    choice.state.commit(box)


class _PalletState:
    """Loader-internal model of one pallet: placed solids and the EMS list."""

    def __init__(self, pallet: Pallet, index: int) -> None:
        self.pallet = pallet
        self.index = index                       # opening order (first fit)
        self.length = pallet.length_mm
        self.width = pallet.width_mm
        self.height = pallet.max_height_mm
        self.solids: list[_Solid] = []
        self.loaded_volume_mm3 = 0.0
        # An empty pallet has one EMS: the whole loading envelope.
        self.spaces: list[_Space] = [
            _Space(0.0, 0.0, 0.0, self.length, self.width, self.height, created=0)
        ]
        self._next_id = 1

        # Support pallets that already carry boxes.
        for box in pallet.boxes:
            self.commit(box)

    def commit(self, box: Box) -> None:
        """Register a placed box and update the EMS list."""
        lo, hi = box.bounds()
        solid = _Solid(lo.x, lo.y, lo.z, hi.x, hi.y, hi.z)
        self.solids.append(solid)
        self.loaded_volume_mm3 += box.volume_mm3
        self._subtract(solid)

    def _subtract(self, b: _Solid) -> None:
        """Difference process: split every EMS the box intersects into up to
        six maximal pieces; elimination: drop pieces contained in another
        EMS, too thin pieces and (optionally) blocked pieces."""
        tol = settings.PLACEMENT_TOLERANCE_MM
        kept: list[_Space] = []
        pieces: list[tuple[float, float, float, float, float, float]] = []

        for s in self.spaces:
            intersects = (
                s.x1 < b.x2 - tol and s.x2 > b.x1 + tol
                and s.y1 < b.y2 - tol and s.y2 > b.y1 + tol
                and s.z1 < b.z2 - tol and s.z2 > b.z1 + tol
            )
            if not intersects:
                kept.append(s)
                continue
            if b.x1 > s.x1:   # behind the box (-X)
                pieces.append((s.x1, s.y1, s.z1, b.x1, s.y2, s.z2))
            if b.x2 < s.x2:   # in front of the box (+X)
                pieces.append((b.x2, s.y1, s.z1, s.x2, s.y2, s.z2))
            if b.y1 > s.y1:   # left of the box (-Y)
                pieces.append((s.x1, s.y1, s.z1, s.x2, b.y1, s.z2))
            if b.y2 < s.y2:   # right of the box (+Y)
                pieces.append((s.x1, b.y2, s.z1, s.x2, s.y2, s.z2))
            if b.z1 > s.z1:   # below the box (-Z)
                pieces.append((s.x1, s.y1, s.z1, s.x2, s.y2, b.z1))
            if b.z2 < s.z2:   # above the box (+Z)
                pieces.append((s.x1, s.y1, b.z2, s.x2, s.y2, s.z2))

        new: list[_Space] = []
        for x1, y1, z1, x2, y2, z2 in pieces:
            if min(x2 - x1, y2 - y1, z2 - z1) < EMS_MIN_SIDE_MM:
                continue
            if ENFORCE_BLOCKING and x2 < self.length - tol:
                continue  # does not reach the entrance: blocked
            new.append(_Space(x1, y1, z1, x2, y2, z2, created=self._next_id))
            self._next_id += 1

        survivors: list[_Space] = []
        for i, n in enumerate(new):
            if any(_contains(k, n) for k in kept):
                continue
            if any(
                j != i and _contains(m, n) and (not _contains(n, m) or j < i)
                for j, m in enumerate(new)
            ):
                continue  # contained in another piece (equal pieces: keep first)
            survivors.append(n)

        self.spaces = kept + survivors


def _contains(outer: _Space, inner: _Space) -> bool:
    tol = settings.PLACEMENT_TOLERANCE_MM
    return (
        outer.x1 <= inner.x1 + tol and outer.x2 >= inner.x2 - tol
        and outer.y1 <= inner.y1 + tol and outer.y2 >= inner.y2 - tol
        and outer.z1 <= inner.z1 + tol and outer.z2 >= inner.z2 - tol
    )


# ============================================================================
# HELPERS & VALIDATION
# ============================================================================

def _orientations(box: Box) -> list[tuple[Orientation, Dims]]:
    """Allowed orientations with distinct oriented dimensions."""
    unique: list[tuple[Orientation, Dims]] = []
    seen: set[tuple[float, float, float]] = set()
    for orientation in box.allowed_orientations:
        d = box.oriented_dimensions(orientation)
        key = (round(d.x, 6), round(d.y, 6), round(d.z, 6))
        if key not in seen:
            seen.add(key)
            unique.append((orientation, (d.x, d.y, d.z)))
    return unique


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

    if BIN_SELECTION_RULE not in _BIN_RULES:
        raise AdvancedLoaderError(f"BIN_SELECTION_RULE must be one of {_BIN_RULES}.")
    if SPACE_SELECTION_RULE not in range(1, 9):
        raise AdvancedLoaderError("SPACE_SELECTION_RULE must be within 1..8.")
    if ORIENTATION_SELECTION_RULE not in range(1, 6):
        raise AdvancedLoaderError("ORIENTATION_SELECTION_RULE must be within 1..5.")
    if STABILITY_CONSTRAINT not in _STABILITY_RULES:
        raise AdvancedLoaderError(f"STABILITY_CONSTRAINT must be one of {_STABILITY_RULES}.")

    if not 0.0 < PARTIAL_BASE_SUPPORT_RATIO <= 1.0:
        raise AdvancedLoaderError("PARTIAL_BASE_SUPPORT_RATIO must be within (0, 1].")
    if not 0.0 <= POLYGON_MIN_AREA_RATIO < 1.0:
        raise AdvancedLoaderError("POLYGON_MIN_AREA_RATIO must be within [0, 1).")
    if N_EMS_CANDIDATES < 1:
        raise AdvancedLoaderError("N_EMS_CANDIDATES must be at least 1.")
    if MAX_OPEN_PALLETS is not None and MAX_OPEN_PALLETS < 1:
        raise AdvancedLoaderError("MAX_OPEN_PALLETS must be None or at least 1.")

    for name, value in (
        ("SUPPORT_FLUSH_TOL_MM", SUPPORT_FLUSH_TOL_MM),
        ("EMS_MIN_SIDE_MM", EMS_MIN_SIDE_MM),
    ):
        if not isfinite(value) or value < 0:
            raise AdvancedLoaderError(f"{name} must be non-negative and finite.")