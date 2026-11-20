"""Steerable EMS decoder for the loader optimizer.

This module is a fork of emsLoader.py (Ali, Ramos & Oliveira 2025). It runs
the same seven steps per box, but differs from emsLoader in three ways:

1. Configuration is an argument (DecoderConfig), not module-level constants.
   The optimizer can therefore try many rule combinations without touching
   any global setting.

2. It works on light, read-only data (BoxSpec) and returns a Layout record.
   No Box or Pallet object is created or modified during a trial, so
   thousands of trials can run without copying or resetting anything. The
   optimizer replays only the winning Layout on the real Box objects.

3. Every box has three optional steering keys in [0, 1):

       key 0  pallet       choose another open pallet, or open a new one
       key 1  space        skip the first feasible EMS(s) in priority order
       key 2  orientation  choose another feasible orientation in the EMS

   A key below (1 - deviation_rate) means "do what emsLoader would do".
   Only keys in the top band deviate; the position inside the band selects
   the alternative. With keys=None, or with all keys at 0.0, the decoder
   reproduces emsLoader exactly: same pallets, positions and orientations.

The box order is never changed.

Functions keep the names they have in emsLoader.py (_select_pallet,
_best_in_pallet, _select_orientation, _is_stable, _subtract ...), so the two
files can be compared side by side. Lines that exist only for steering or
speed are marked with "DECODER:".

Speed-ups that do not change the result
---------------------------------------
* EMS sort keys are computed once, when an EMS is created.
* The sorted EMS list of a pallet is cached until that pallet changes.
* A box shape that found no feasible placement on a pallet is remembered
  until that pallet changes (boxes arrive grouped by SKU, so a full pallet
  is not scanned again for every identical box).
"""

from __future__ import annotations

import random
from collections.abc import Callable, Iterable, Sequence
from dataclasses import dataclass
from math import hypot, isfinite
from operator import attrgetter

from ..box import Box, Orientation
from .geometry import convex_hull, point_in_convex_polygon, polygon_area

Dims = tuple[float, float, float]
Point2D = tuple[float, float]

# Bounds of one placed box: (x1, y1, z1, x2, y2, z2).
Solid = tuple[float, float, float, float, float, float]

BIN_RULES: tuple[str, ...] = ("first_fit", "best_fit", "worst_fit", "all_bins")
STABILITY_RULES: tuple[str, ...] = (
    "full_base", "partial_base", "cog_polygon", "partial_base_polygon",
)
EARLY_NEW_PALLET_MODES: tuple[str, ...] = ("never", "below_capacity", "always")

# Steering keys per box: pallet, space, orientation.
KEYS_PER_BOX: int = 3

# Returned by the steered pallet selection when the key asks for a new pallet.
_OPEN_NEW_PALLET = object()

# "No deviation" marker for a decoded steering key.
_NO_DEVIATION: float = -1.0


class DecoderError(RuntimeError):
    """Base exception raised by the EMS decoder."""


class UnplaceableBoxError(DecoderError):
    """Raised when a box cannot fit even on an empty pallet."""


# ============================================================================
# PUBLIC DATA TYPES
# ============================================================================

@dataclass(frozen=True, slots=True)
class DecoderConfig:
    """Everything emsLoader keeps in its ALGORITHM SETTINGS block.

    The defaults equal the defaults of emsLoader.py. The optimizer normally
    builds this from emsLoader's live settings (see loaderOptimizer.py).
    """

    bin_rule: str = "first_fit"
    space_rule: int = 3
    orientation_rule: int = 2
    n_ems_candidates: int = 3
    stability_constraint: str = "partial_base_polygon"
    partial_base_support_ratio: float = 0.80
    polygon_min_area_ratio: float = 0.50
    support_flush_tol_mm: float = 0.5
    max_open_pallets: int | None = 2
    enforce_blocking: bool = False
    guard_pass_through: bool = True
    ems_min_side_mm: float = 10.0
    random_seed: int = 0
    score_rel_eps: float = 1e-9
    weight_eps_kg: float = 1e-6
    placement_tol_mm: float = 1e-6

    @property
    def label(self) -> str:
        """Heuristic name in the style of the paper, e.g. F32 or A52."""
        return f"{self.bin_rule[0].upper()}{self.space_rule}{self.orientation_rule}"

    def validate(self) -> None:
        if self.bin_rule not in BIN_RULES:
            raise DecoderError(f"bin_rule must be one of {BIN_RULES}.")
        if self.space_rule not in range(1, 9):
            raise DecoderError("space_rule must be within 1..8.")
        if self.orientation_rule not in range(1, 6):
            raise DecoderError("orientation_rule must be within 1..5.")
        if self.stability_constraint not in STABILITY_RULES:
            raise DecoderError(f"stability_constraint must be one of {STABILITY_RULES}.")
        if not 0.0 < self.partial_base_support_ratio <= 1.0:
            raise DecoderError("partial_base_support_ratio must be within (0, 1].")
        if not 0.0 <= self.polygon_min_area_ratio < 1.0:
            raise DecoderError("polygon_min_area_ratio must be within [0, 1).")
        if self.n_ems_candidates < 1:
            raise DecoderError("n_ems_candidates must be at least 1.")
        if self.max_open_pallets is not None and self.max_open_pallets < 1:
            raise DecoderError("max_open_pallets must be None or at least 1.")
        for name in ("support_flush_tol_mm", "ems_min_side_mm", "placement_tol_mm",
                     "score_rel_eps", "weight_eps_kg"):
            value = getattr(self, name)
            if not isfinite(value) or value < 0:
                raise DecoderError(f"{name} must be non-negative and finite.")
        if not isinstance(self.random_seed, int):
            raise DecoderError("random_seed must be an integer.")


@dataclass(frozen=True, slots=True)
class PalletSpec:
    """Loading envelope of one pallet (same values for every pallet)."""

    length_mm: float
    width_mm: float
    max_height_mm: float
    max_load_kg: float | None

    @property
    def usable_volume_mm3(self) -> float:
        return self.length_mm * self.width_mm * self.max_height_mm


@dataclass(frozen=True, slots=True)
class BoxSpec:
    """Read-only data of one box, extracted once from the Box object."""

    index: int                                   # position in the box sequence
    box_id: str
    weight_kg: float
    volume_mm3: float
    # Allowed orientations with distinct oriented dimensions (l, w, h).
    orientations: tuple[tuple[Orientation, Dims], ...]
    # Boxes with identical orientation lists share one shape_id.
    shape_id: int


@dataclass(frozen=True, slots=True)
class SteeringParams:
    """How steering keys are translated into deviations."""

    deviation_rate: float = 0.03       # width of the deviating key band
    space_alternatives: int = 3        # max. feasible EMSs a space key can skip
    early_new_pallet: str = "below_capacity"

    def validate(self) -> None:
        if not 0.0 <= self.deviation_rate <= 1.0:
            raise DecoderError("deviation_rate must be within [0, 1].")
        if self.space_alternatives < 0:
            raise DecoderError("space_alternatives must be non-negative.")
        if self.early_new_pallet not in EARLY_NEW_PALLET_MODES:
            raise DecoderError(
                f"early_new_pallet must be one of {EARLY_NEW_PALLET_MODES}."
            )


@dataclass(frozen=True, slots=True)
class PlacedBox:
    """Final placement of one box in pallet-local coordinates [mm]."""

    box_index: int
    pallet_index: int
    orientation: Orientation
    x: float
    y: float
    z: float
    l: float          # extent along X
    w: float          # extent along Y
    h: float          # extent along Z


@dataclass(frozen=True, slots=True)
class Layout:
    """Result of one decoder run; placements are in box-sequence order."""

    placements: tuple[PlacedBox, ...]
    pallet_count: int


# ============================================================================
# PUBLIC FUNCTIONS
# ============================================================================

def make_box_specs(boxes: Iterable[Box]) -> tuple[BoxSpec, ...]:
    """Extract decoder data from Box objects without modifying them."""
    specs: list[BoxSpec] = []
    shape_ids: dict[tuple[tuple[Orientation, Dims], ...], int] = {}
    for index, box in enumerate(boxes):
        orientations = _orientations(box)
        shape_id = shape_ids.setdefault(orientations, len(shape_ids))
        specs.append(
            BoxSpec(
                index=index,
                box_id=box.box_id,
                weight_kg=box.weight_kg,
                volume_mm3=box.volume_mm3,
                orientations=orientations,
                shape_id=shape_id,
            )
        )
    return tuple(specs)


def decode(
    box_specs: Sequence[BoxSpec],
    pallet: PalletSpec,
    cfg: DecoderConfig,
    keys: Sequence[float] | None = None,
    steering: SteeringParams | None = None,
) -> Layout:
    """Load all boxes in sequence order and return the resulting layout.

    keys      None -> pure emsLoader behaviour. Otherwise KEYS_PER_BOX values
              in [0, 1) per box, laid out as
              [pallet_0, space_0, orientation_0, pallet_1, ...].
    steering  how keys are interpreted; required when keys are given.
    """
    if keys is not None:
        if steering is None:
            raise DecoderError("steering parameters are required when keys are given.")
        if len(keys) != KEYS_PER_BOX * len(box_specs):
            raise DecoderError(
                f"Expected {KEYS_PER_BOX * len(box_specs)} steering keys, "
                f"got {len(keys)}."
            )
    return _Run(pallet, cfg, keys, steering).load(box_specs)


# ============================================================================
# INTERNAL TYPES
# ============================================================================

class _Space:
    """One Empty Maximal Space: [x1, x2] x [y1, y2] x [z1, z2] is empty."""

    __slots__ = (
        "x1", "y1", "z1", "x2", "y2", "z2", "created",
        "dx", "dy", "dz", "volume",
        "back_dist", "back_right",   # nearest back-bottom corner (rules 5-7)
        "key",                       # DECODER: Table 1 sort key, cached
    )

    def __init__(
        self,
        x1: float, y1: float, z1: float,
        x2: float, y2: float, z2: float,
        created: int,
    ) -> None:
        self.x1 = x1
        self.y1 = y1
        self.z1 = z1
        self.x2 = x2
        self.y2 = y2
        self.z2 = z2
        self.created = created       # generation counter ("generated earlier")
        self.dx = x2 - x1
        self.dy = y2 - y1
        self.dz = z2 - z1
        self.volume = self.dx * self.dy * self.dz
        self.back_dist = 0.0
        self.back_right = False
        self.key: tuple[float, ...] = ()


class _Placement:
    """One feasible (fitting and stable) placement of the current box."""

    __slots__ = ("space", "orientation", "dims", "position")

    def __init__(
        self,
        space: _Space,
        orientation: Orientation,
        dims: Dims,
        position: Dims,
    ) -> None:
        self.space = space
        self.orientation = orientation
        self.dims = dims
        self.position = position


class _Choice:
    """The placement a pallet proposes, plus the scores used by 'all bins'."""

    __slots__ = ("state", "placement", "space_key", "orientation_key")

    def __init__(
        self,
        state: _PalletState,
        placement: _Placement,
        space_key: tuple[float, ...],
        orientation_key: tuple[float, ...],
    ) -> None:
        self.state = state
        self.placement = placement
        self.space_key = space_key
        self.orientation_key = orientation_key


_SPACE_SORT_KEY = attrgetter("key")


# ============================================================================
# ONE DECODER RUN
# ============================================================================

class _Run:
    """State and rule logic of one pass over the box sequence."""

    def __init__(
        self,
        pallet: PalletSpec,
        cfg: DecoderConfig,
        keys: Sequence[float] | None,
        steering: SteeringParams | None,
    ) -> None:
        self.pallet = pallet
        self.cfg = cfg
        self.rng = random.Random(cfg.random_seed)
        self.tol = cfg.placement_tol_mm
        # Number of feasible EMSs the orientation rule looks at.
        self.needed = cfg.n_ems_candidates if cfg.orientation_rule in (4, 5) else 1

        # DECODER: steering set-up.
        self.keys = keys
        rate = steering.deviation_rate if steering is not None else 0.0
        if keys is None or rate <= 0.0:
            self.keys = None
            self.rate = 0.0
            self.threshold = 2.0                 # never reached
        else:
            self.rate = rate
            self.threshold = 1.0 - rate
        self.space_alternatives = steering.space_alternatives if steering else 0
        self.early_new_pallet = steering.early_new_pallet if steering else "never"

    # ------------------------------------------------------------------
    # MAIN LOOP (emsLoader.load_boxes)
    # ------------------------------------------------------------------
    def load(self, box_specs: Sequence[BoxSpec]) -> Layout:
        cfg = self.cfg
        keys = self.keys
        states: list[_PalletState] = []
        placements: list[PlacedBox] = []

        for i, spec in enumerate(box_specs):
            # DECODER: decode the three steering keys of this box.
            if keys is None:
                kp = ks = ko = _NO_DEVIATION
            else:
                base = KEYS_PER_BOX * i
                kp = self._deviation(keys[base])
                ks = self._deviation(keys[base + 1])
                ko = self._deviation(keys[base + 2])

            # ---- 1. bin selection over the open pallets -------------------
            open_states = (
                states if cfg.max_open_pallets is None
                else states[-cfg.max_open_pallets:]
            )
            if kp < 0.0:
                choice = self._select_pallet(open_states, spec, ks, ko)
            else:
                choice = self._select_pallet_steered(open_states, spec, kp, ks, ko)

            # ---- open a new pallet if no open pallet can take the box -----
            # DECODER: ... or if the pallet key asked for a new pallet.
            if not isinstance(choice, _Choice):
                states.append(_PalletState(self, index=len(states)))
                choice = self._best_in_pallet(states[-1], spec, ks, ko)
                if choice is None:
                    raise UnplaceableBoxError(self._unplaceable_message(spec))

            # ---- 6./7. placement and space update -------------------------
            self._commit(choice, spec, placements)

        return Layout(placements=tuple(placements), pallet_count=len(states))

    def _deviation(self, key: float) -> float:
        """DECODER: translate one key. Returns _NO_DEVIATION or a fraction in
        [0, 1) that selects among the alternatives of the decision."""
        if key < self.threshold:
            return _NO_DEVIATION
        fraction = (key - self.threshold) / self.rate
        return fraction if fraction < 1.0 else 0.999999999

    # ------------------------------------------------------------------
    # 1. BIN SELECTION
    # ------------------------------------------------------------------
    def _select_pallet(
        self,
        states: Sequence[_PalletState],
        spec: BoxSpec,
        ks: float,
        ko: float,
    ) -> _Choice | None:
        """Return the chosen placement over all open pallets, or None."""
        rule = self.cfg.bin_rule

        if rule == "all_bins":
            choices = [
                choice
                for state in states
                if (choice := self._best_in_pallet(state, spec, ks, ko)) is not None
            ]
            if not choices:
                return None
            return min(
                choices,
                key=lambda c: (c.space_key, c.orientation_key, c.state.index),
            )

        for state in self._bin_order(states):
            choice = self._best_in_pallet(state, spec, ks, ko)
            if choice is not None:
                return choice
        return None

    def _bin_order(self, states: Sequence[_PalletState]) -> list[_PalletState]:
        """Order in which first / best / worst fit try the open pallets."""
        rule = self.cfg.bin_rule
        if rule == "best_fit":      # fullest pallet first
            return sorted(states, key=lambda s: (-s.loaded_volume_mm3, s.index))
        if rule == "worst_fit":     # emptiest pallet first
            return sorted(states, key=lambda s: (s.loaded_volume_mm3, s.index))
        return list(states)         # first fit (and all bins): oldest first

    def _select_pallet_steered(
        self,
        states: Sequence[_PalletState],
        spec: BoxSpec,
        kp: float,
        ks: float,
        ko: float,
    ) -> _Choice | object | None:
        """DECODER: pallet selection when the pallet key deviates.

        Every open pallet proposes its placement. Index 0 of the candidate
        list is what the bin rule would choose; the key selects one of the
        other candidates or, when allowed, a new pallet.
        """
        candidates = [
            choice
            for state in self._bin_order(states)
            if (choice := self._best_in_pallet(state, spec, ks, ko)) is not None
        ]
        if not candidates:
            return None             # a new pallet is the only option anyway
        if self.cfg.bin_rule == "all_bins":
            candidates.sort(
                key=lambda c: (c.space_key, c.orientation_key, c.state.index)
            )

        other_pallets = len(candidates) - 1
        mode = self.early_new_pallet
        allow_new = mode == "always" or (
            mode == "below_capacity"
            and (
                self.cfg.max_open_pallets is None
                or len(states) < self.cfg.max_open_pallets
            )
        )
        alternatives = other_pallets + (1 if allow_new else 0)
        if alternatives == 0:
            return candidates[0]

        pick = 1 + int(kp * alternatives)          # 1 .. alternatives
        if pick <= other_pallets:
            return candidates[pick]
        return _OPEN_NEW_PALLET

    # ------------------------------------------------------------------
    # 2.-5. SPACE, FIT, STABILITY AND ORIENTATION ON ONE PALLET
    # ------------------------------------------------------------------
    def _best_in_pallet(
        self,
        state: _PalletState,
        spec: BoxSpec,
        ks: float,
        ko: float,
    ) -> _Choice | None:
        """Best feasible placement of the box on one pallet, or None."""
        if not self._weight_fits(state, spec):
            return None
        # DECODER: this shape already failed on this (unchanged) pallet.
        if spec.shape_id in state.no_fit:
            return None

        tol = self.tol
        # DECODER: a deviating space key skips the first feasible EMS(s).
        skip = 0
        if ks >= 0.0 and self.space_alternatives > 0:
            skip = 1 + int(ks * self.space_alternatives)
        target = self.needed + skip

        # ---- 2. space selection: EMSs in priority order (Table 1) ---------
        groups: list[tuple[_Space, list[_Placement]]] = []
        for space in state.ordered_spaces():
            options: list[_Placement] = []
            for orientation, (l, w, h) in spec.orientations:
                # ---- 3. orientation fit evaluation ------------------------
                if l > space.dx + tol or w > space.dy + tol or h > space.dz + tol:
                    continue
                x, y, z = self._anchor(space, w)
                # ---- 4. stability assessment ------------------------------
                # ---- 4. stability assessment + pass-through guard ---------
                if self._is_stable(state, x, y, z, l, w) and (
                    not self.cfg.guard_pass_through
                    or self._is_reachable(state, x, y, z, l, w)
                ):
                    options.append(_Placement(space, orientation, (l, w, h), (x, y, z)))
            if options:
                groups.append((space, options))
                if len(groups) >= target:
                    break

        if not groups:
            state.no_fit.add(spec.shape_id)
            return None

        if skip:
            # Fewer feasible EMSs than the key wanted to skip: use the last.
            groups = groups[skip:] if len(groups) > skip else groups[-1:]

        # ---- 5. orientation selection (Table 2) ---------------------------
        placement, orientation_key = self._select_orientation(state, groups)

        # DECODER: a deviating orientation key takes another feasible
        # orientation in the same EMS (the score of the default is kept).
        if ko >= 0.0:
            for space, options in groups:
                if space is placement.space:
                    others = [p for p in options if p is not placement]
                    if others:
                        placement = others[int(ko * len(others))]
                    break

        return _Choice(
            state=state,
            placement=placement,
            space_key=placement.space.key,
            orientation_key=orientation_key,
        )

    def _anchor(self, s: _Space, w: float) -> Dims:
        """Position of the box inside the EMS (deepest-bottom-left by default)."""
        if s.back_right and self.cfg.space_rule in (5, 6, 7):
            return (s.x1, s.y2 - w, s.z1)          # back-bottom-right
        return (s.x1, s.y1, s.z1)                  # back-bottom-left

    def _select_orientation(
        self,
        state: _PalletState,
        groups: list[tuple[_Space, list[_Placement]]],
    ) -> tuple[_Placement, tuple[float, ...]]:
        """Table 2. Return the chosen placement and its score (lower = better)."""
        rule = self.cfg.orientation_rule

        if rule == 1:
            return self._pick(groups[0][1], lambda p: (_margin(p),), random_ties=True)

        if rule == 2:
            return self._pick(
                groups[0][1],
                lambda p: (-p.dims[0] * p.dims[1], p.dims[0] / p.space.dx),
                random_ties=False,
            )

        if rule == 3:
            return self._pick(
                groups[0][1],
                lambda p: (-p.dims[0] * p.dims[1], -p.dims[0] / p.space.dx),
                random_ties=False,
            )

        if rule == 4:
            # (i) per EMS the minimum-margin orientation, (ii) highest fill ratio.
            per_space = [
                self._pick(options, lambda p: (_margin(p),), random_ties=True)[0]
                for _, options in groups
            ]
            return self._pick(
                per_space,
                lambda p: (-(p.dims[0] * p.dims[1] * p.dims[2]) / p.space.volume,),
                random_ties=True,
            )

        # rule 5: max distance from box front-top-right to bin front-top-right.
        candidates = [p for _, options in groups for p in options]
        return self._pick(
            candidates,
            lambda p: (
                -hypot(
                    state.length - (p.position[0] + p.dims[0]),
                    state.width - (p.position[1] + p.dims[1]),
                    state.height - (p.position[2] + p.dims[2]),
                ),
            ),
            random_ties=True,
        )

    def _pick(
        self,
        candidates: list[_Placement],
        key: Callable[[_Placement], tuple[float, ...]],
        *,
        random_ties: bool,
    ) -> tuple[_Placement, tuple[float, ...]]:
        """Return the candidate with the lowest key; ties random or first."""
        keys = [key(c) for c in candidates]
        best = min(keys)
        tied = [c for c, k in zip(candidates, keys, strict=True) if self._same_score(k, best)]
        chosen = self.rng.choice(tied) if random_ties and len(tied) > 1 else tied[0]
        return chosen, best

    def _same_score(self, a: tuple[float, ...], b: tuple[float, ...]) -> bool:
        eps = self.cfg.score_rel_eps
        return all(
            abs(x - y) <= eps * max(1.0, abs(x), abs(y))
            for x, y in zip(a, b, strict=True)
        )

    # ------------------------------------------------------------------
    # 4. STATIC STABILITY (Section 3 of the paper)
    # ------------------------------------------------------------------
    def _is_stable(
        self,
        state: _PalletState,
        x: float,
        y: float,
        z: float,
        l: float,
        w: float,
    ) -> bool:
        """Apply the stability constraint to a box footprint (x, y, l, w) at z."""
        cfg = self.cfg
        tol = self.tol

        # Condition 1 of every constraint: resting on the pallet deck.
        if z <= tol:
            return True

        # Intersections of the base with the top faces of supporting boxes.
        # Each rectangle is (x1, y1, x2, y2); its corners are v_j1..v_j4.
        flush = cfg.support_flush_tol_mm
        xl = x + l
        yw = y + w
        rects: list[tuple[float, float, float, float]] = []
        for bx1, by1, _bz1, bx2, by2, bz2 in state.solids:
            if abs(bz2 - z) > flush:
                continue
            rx1, rx2 = max(x, bx1), min(xl, bx2)
            ry1, ry2 = max(y, by1), min(yw, by2)
            if rx2 - rx1 > tol and ry2 - ry1 > tol:
                rects.append((rx1, ry1, rx2, ry2))

        if not rects:
            return False

        base_area = l * w
        rule = cfg.stability_constraint

        # ---- 3.1 / 3.2 area-based support ---------------------------------
        if rule in ("full_base", "partial_base"):
            supported = sum((x2 - x1) * (y2 - y1) for x1, y1, x2, y2 in rects)
            required = 1.0 if rule == "full_base" else cfg.partial_base_support_ratio
            return supported >= required * base_area * (1.0 - 1e-9)

        # ---- 3.3 / 3.4 polygon-based support ------------------------------
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
        hull = convex_hull(points)
        if not (direct or point_in_convex_polygon((cx, cy), hull)):
            return False
        if rule == "cog_polygon":
            return True

        # Partial-base polygon: SP area must exceed the ratio of the base area.
        return polygon_area(hull) > cfg.polygon_min_area_ratio * base_area

    def _is_reachable(
        self,
        state: _PalletState,
        x: float,
        y: float,
        z: float,
        l: float,
        w: float,
    ) -> bool:
        """Pass-through guard (see emsLoader._is_reachable)."""
        tol = self.tol

        # Side access: a box face flush with a pallet edge.
        if (
            x <= tol or y <= tol
            or x + l >= state.length - tol
            or y + w >= state.width - tol
        ):
            return True

        # Overhang over the footprint = solid in the drop path.
        xl = x + l
        yw = y + w
        for bx1, by1, bz1, bx2, by2, _bz2 in state.solids:
            if (
                bz1 > z + tol
                and bx1 < xl - tol and bx2 > x + tol
                and by1 < yw - tol and by2 > y + tol
            ):
                return False
        return True

    # ------------------------------------------------------------------
    # 6. COMMIT
    # ------------------------------------------------------------------
    def _commit(
        self,
        choice: _Choice,
        spec: BoxSpec,
        placements: list[PlacedBox],
    ) -> None:
        p = choice.placement
        x, y, z = p.position
        l, w, h = p.dims
        # DECODER: record the placement instead of calling Pallet.place_box.
        placements.append(
            PlacedBox(
                box_index=spec.index,
                pallet_index=choice.state.index,
                orientation=p.orientation,
                x=x, y=y, z=z, l=l, w=w, h=h,
            )
        )
        choice.state.commit((x, y, z, x + l, y + w, z + h), spec)

    # ------------------------------------------------------------------
    # HELPERS
    # ------------------------------------------------------------------
    def _weight_fits(self, state: _PalletState, spec: BoxSpec) -> bool:
        if state.max_load_kg is None:
            return True
        return state.load_kg + spec.weight_kg <= state.max_load_kg + self.cfg.weight_eps_kg

    def _unplaceable_message(self, spec: BoxSpec) -> str:
        dims = " / ".join(
            f"{l:g} x {w:g} x {h:g}" for _, (l, w, h) in spec.orientations
        )
        pallet = self.pallet
        return (
            f"Box {spec.box_id!r} cannot fit on an empty pallet. Allowed oriented "
            f"dimensions are {dims} mm, weight is {spec.weight_kg:g} kg. Pallet "
            f"limits are {pallet.length_mm:g} x {pallet.width_mm:g} x "
            f"{pallet.max_height_mm:g} mm with maximum load {pallet.max_load_kg!r} kg."
        )


# ============================================================================
# PER-PALLET STATE (EMS difference & elimination, Lai & Chan 1997)
# ============================================================================

class _PalletState:
    """Decoder-internal model of one pallet: placed solids and the EMS list."""

    def __init__(self, run: _Run, index: int) -> None:
        cfg = run.cfg
        self.index = index                       # opening order (first fit)
        self.length = run.pallet.length_mm
        self.width = run.pallet.width_mm
        self.height = run.pallet.max_height_mm
        self.max_load_kg = run.pallet.max_load_kg
        self.solids: list[Solid] = []
        self.loaded_volume_mm3 = 0.0
        self.load_kg = 0.0                       # DECODER: replaces Pallet.current_load_kg

        self._tol = cfg.placement_tol_mm
        self._space_rule = cfg.space_rule
        self._ems_min_side = cfg.ems_min_side_mm
        self._enforce_blocking = cfg.enforce_blocking
        self._next_id = 1

        # DECODER: caches, both invalidated by commit().
        self._ordered: list[_Space] | None = None
        self.no_fit: set[int] = set()

        # An empty pallet has one EMS: the whole loading envelope.
        self.spaces: list[_Space] = [
            self._make_space(0.0, 0.0, 0.0, self.length, self.width, self.height, 0)
        ]

    def ordered_spaces(self) -> list[_Space]:
        """EMSs in Table 1 priority order (cached until the pallet changes)."""
        if self._ordered is None:
            self._ordered = sorted(self.spaces, key=_SPACE_SORT_KEY)
        return self._ordered

    def commit(self, solid: Solid, spec: BoxSpec) -> None:
        """Register a placed box and update the EMS list."""
        self.solids.append(solid)
        self.loaded_volume_mm3 += spec.volume_mm3
        self.load_kg += spec.weight_kg
        self._subtract(solid)
        self._ordered = None
        self.no_fit.clear()

    def _make_space(
        self,
        x1: float, y1: float, z1: float,
        x2: float, y2: float, z2: float,
        created: int,
    ) -> _Space:
        """Create an EMS and compute its Table 1 sort key (lower = higher
        priority). 'Generated earlier' is the final, deterministic tie-breaker."""
        s = _Space(x1, y1, z1, x2, y2, z2, created)
        rule = self._space_rule

        if rule in (5, 6, 7):
            # Distance to the nearer back-bottom bin corner and whether that
            # is the right corner (0, width, 0) rather than the left (0, 0, 0).
            left = hypot(x1, y1, z1)
            right = hypot(x1, self.width - y2, z1)
            if right < left:
                s.back_dist, s.back_right = right, True
            else:
                s.back_dist, s.back_right = left, False

        if rule == 1:
            key: tuple[float, ...] = (x1, z1, y1)
        elif rule == 2:
            key = tuple(sorted((x1, y1, z1)))
        elif rule == 3:
            key = (z1, min(x1, y1), max(x1, y1))
        elif rule == 4:
            key = (s.volume,)
        elif rule == 5:
            key = (x1, z1, s.back_dist)
        elif rule == 6:
            key = (s.back_dist, -s.volume)
        elif rule == 7:
            key = (s.back_dist, s.volume)
        else:  # rule 8
            key = (-hypot(self.length - x1, self.width - y1, self.height - z1),)
        s.key = key + (float(created),)
        return s

    def _subtract(self, b: Solid) -> None:
        """Difference process: split every EMS the box intersects into up to
        six maximal pieces; elimination: drop pieces contained in another
        EMS, too thin pieces and (optionally) blocked pieces."""
        tol = self._tol
        bx1, by1, bz1, bx2, by2, bz2 = b
        kept: list[_Space] = []
        pieces: list[tuple[float, float, float, float, float, float]] = []

        for s in self.spaces:
            intersects = (
                s.x1 < bx2 - tol and s.x2 > bx1 + tol
                and s.y1 < by2 - tol and s.y2 > by1 + tol
                and s.z1 < bz2 - tol and s.z2 > bz1 + tol
            )
            if not intersects:
                kept.append(s)
                continue
            if bx1 > s.x1:   # behind the box (-X)
                pieces.append((s.x1, s.y1, s.z1, bx1, s.y2, s.z2))
            if bx2 < s.x2:   # in front of the box (+X)
                pieces.append((bx2, s.y1, s.z1, s.x2, s.y2, s.z2))
            if by1 > s.y1:   # left of the box (-Y)
                pieces.append((s.x1, s.y1, s.z1, s.x2, by1, s.z2))
            if by2 < s.y2:   # right of the box (+Y)
                pieces.append((s.x1, by2, s.z1, s.x2, s.y2, s.z2))
            if bz1 > s.z1:   # below the box (-Z)
                pieces.append((s.x1, s.y1, s.z1, s.x2, s.y2, bz1))
            if bz2 < s.z2:   # above the box (+Z)
                pieces.append((s.x1, s.y1, bz2, s.x2, s.y2, s.z2))

        new: list[_Space] = []
        for x1, y1, z1, x2, y2, z2 in pieces:
            if min(x2 - x1, y2 - y1, z2 - z1) < self._ems_min_side:
                continue
            if self._enforce_blocking and x2 < self.length - tol:
                continue  # does not reach the entrance: blocked
            new.append(self._make_space(x1, y1, z1, x2, y2, z2, self._next_id))
            self._next_id += 1

        survivors: list[_Space] = []
        for i, n in enumerate(new):
            if any(_contains(k, n, tol) for k in kept):
                continue
            if any(
                j != i and _contains(m, n, tol) and (not _contains(n, m, tol) or j < i)
                for j, m in enumerate(new)
            ):
                continue  # contained in another piece (equal pieces: keep first)
            survivors.append(n)

        self.spaces = kept + survivors


# ============================================================================
# MODULE-LEVEL HELPERS
# ============================================================================

def _contains(outer: _Space, inner: _Space, tol: float) -> bool:
    return (
        outer.x1 <= inner.x1 + tol and outer.x2 >= inner.x2 - tol
        and outer.y1 <= inner.y1 + tol and outer.y2 >= inner.y2 - tol
        and outer.z1 <= inner.z1 + tol and outer.z2 >= inner.z2 - tol
    )


def _margin(p: _Placement) -> float:
    """Smallest clearance between the oriented box and its EMS."""
    return min(p.space.dx - p.dims[0], p.space.dy - p.dims[1], p.space.dz - p.dims[2])


def _orientations(box: Box) -> tuple[tuple[Orientation, Dims], ...]:
    """Allowed orientations with distinct oriented dimensions."""
    unique: list[tuple[Orientation, Dims]] = []
    seen: set[tuple[float, float, float]] = set()
    for orientation in box.allowed_orientations:
        d = box.oriented_dimensions(orientation)
        key = (round(d.x, 6), round(d.y, 6), round(d.z, 6))
        if key not in seen:
            seen.add(key)
            unique.append((orientation, (d.x, d.y, d.z)))
    return tuple(unique)