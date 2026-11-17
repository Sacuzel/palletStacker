"""Scoring of decoder layouts for the loader optimizer.

evaluate() turns one Layout into a Metrics record. ranking_key() turns the
metrics into a sortable tuple (lower = better) that implements the objective
of the plan memo:

    0. hard gate            geometry and stability (see StabilityGate)
    1. pallet count         lower is better
    2. density score        higher is better; weighted mix of
                              - sum of squared fill rates (per pallet mean)
                              - utilization of the occupied envelope
    3. centre-of-gravity    lower is better (tie-breaker)

Density and CoG are quantized (DENSITY_RESOLUTION, COG_RESOLUTION_MM in
optimizerSettings.py) before they are compared. Differences smaller than the
resolution count as a tie, which is what lets the next tier decide and what
"meaningful improvement" means for the stopping criterion.

Dynamic stability (analytical, no physics engine)
-------------------------------------------------
A constant horizontal acceleration a is equivalent to tilting gravity by
atan(a / g). A rigid unit resting on a support polygon tips when the tilted
projection of its centre of gravity leaves the polygon. For a unit with CoG
height h_c above its support plane and distance d from the CoG projection to
the polygon edge in the direction of the inertial force:

    a_crit = g * d / h_c            margin = a_crit / a_spec  (>= 1 passes)

Checked directions: both directions along the travel axis with the larger of
forward acceleration and braking (the truck drives both ways), and both
lateral directions with the turning acceleration.

Two units are evaluated for every box:

    box        the box alone on its own support polygon
    load tree  the box plus everything resting on it:
               * A box with ONE supporter is rigidly part of that
                 supporter's unit (a column tips as a whole), so its mass
                 keeps its real position.
               * A box with SEVERAL supporters bridges them. It cannot tip
                 with any single one of them, so it passes its weight (and
                 the weight it carries) down as loads acting on the top face
                 of each supporter, at the centre of the contact patch,
                 shared in proportion to the contact areas. The bridging
                 box itself is checked on the hull of all its contact
                 patches, together with everything standing on it.

Intermediate stacks are covered: boxes are replayed in placement order, and
each time a box lands, the box itself and every box in its downward support
chain are re-checked. The reported margin of a unit is its worst value over
all intermediate states.

Known simplification: neighbours, friction and stretch film are ignored, so
every unit is treated as free-standing. The load-tree margin is therefore
conservative for tall column stacks.
"""

from __future__ import annotations

import heapq
from collections.abc import Sequence
from dataclasses import dataclass
from math import isfinite

from .emsDecoder import BoxSpec, Layout, PalletSpec
from .geometry import Edge, axis_exit_distances, convex_hull, polygon_edges

STABILITY_UNITS: tuple[str, ...] = ("load_tree", "box")
GATE_MODES: tuple[str, ...] = ("auto", "spec", "baseline", "off")
TRAVEL_AXES: tuple[str, ...] = ("x", "y")

# Numerical slack when a margin is compared with a threshold.
_MARGIN_EPS: float = 1e-9
# Mass of a box without weight data inside an order that has weight data [kg].
_MIN_MASS_KG: float = 1e-9
# Density used when the whole order has no weight data [kg per mm^3].
_VOLUME_MASS_KG_PER_MM3: float = 1e-6
# Resolution of the margin shortfall in the ranking key of failing solutions.
_VIOLATION_RESOLUTION: float = 0.01
_VIOLATION_CAP: int = 10**9

# Leading elements of the ranking key that hold the quantized objective.
# Two solutions with equal leading elements are "not meaningfully different".
STALL_KEY_LENGTH: int = 6


class EvaluatorError(RuntimeError):
    """Raised for invalid evaluator parameters."""


# ============================================================================
# DATA TYPES
# ============================================================================

@dataclass(frozen=True, slots=True)
class EvalParams:
    """Evaluator inputs (filled from optimizerSettings by loaderOptimizer)."""

    gravity_m_s2: float
    accel_travel_m_s2: float        # along the travel axis, both directions
    accel_lateral_m_s2: float       # across the travel axis, both directions
    travel_axis: str                # "x" (pallet length) or "y"
    required_margin: float          # a_crit / a_spec needed to pass the spec
    stability_unit: str             # which margin the gate uses
    support_flush_tol_mm: float
    placement_tol_mm: float
    weight_ssf: float
    weight_envelope: float
    density_resolution: float
    cog_resolution_mm: float

    def validate(self) -> None:
        for name in ("gravity_m_s2", "accel_travel_m_s2", "accel_lateral_m_s2",
                     "required_margin", "density_resolution", "cog_resolution_mm"):
            value = getattr(self, name)
            if not isfinite(value) or value <= 0:
                raise EvaluatorError(f"{name} must be positive and finite.")
        if self.travel_axis not in TRAVEL_AXES:
            raise EvaluatorError(f"travel_axis must be one of {TRAVEL_AXES}.")
        if self.stability_unit not in STABILITY_UNITS:
            raise EvaluatorError(f"stability_unit must be one of {STABILITY_UNITS}.")
        if self.weight_ssf < 0 or self.weight_envelope < 0:
            raise EvaluatorError("Density weights must be non-negative.")
        if self.weight_ssf + self.weight_envelope <= 0:
            raise EvaluatorError("At least one density weight must be positive.")


@dataclass(frozen=True, slots=True)
class Metrics:
    """Everything the optimizer knows about one layout."""

    geometry_ok: bool               # all boxes placed, inside limits, supported
    pallet_count: int
    fill_rates: tuple[float, ...]   # box volume / usable pallet volume
    ssf: float                      # sum of squared fill rates
    envelope_utilization: float     # box volume / (footprint x stack height)
    density_score: float            # weighted mix of the two, within [0, 1]
    cog_height_mm: float            # mass-weighted mean CoG height, all pallets

    box_margin: float               # worst margin, each box alone
    box_unstable: int               # boxes below the required margin
    tree_margin: float              # worst margin, box + load tree
    tree_unstable: int              # load trees below the required margin

    stability_margin: float         # the pair selected by stability_unit
    unstable_units: int


@dataclass(frozen=True, slots=True)
class StabilityGate:
    """Hard stability gate, resolved once per optimizer run.

    spec      margin >= required margin (no unit below spec)
    baseline  at least as stable as the plain inner loader: the worst margin
              is not lower and no more units are below spec
    off       stability is reported but does not reject anything
    """

    mode: str
    min_margin: float
    max_unstable_units: int

    def passes(self, m: Metrics) -> bool:
        if not m.geometry_ok:
            return False
        if self.mode == "off":
            return True
        return (
            m.stability_margin >= self.min_margin - _MARGIN_EPS
            and m.unstable_units <= self.max_unstable_units
        )


@dataclass(frozen=True, slots=True)
class Trial:
    """One scored solution: how to rebuild it and how good it is."""

    config_index: int                       # index into the optimizer's config list
    keys: tuple[float, ...] | None          # steering keys; None = default decisions
    metrics: Metrics | None                 # None = the decoder could not finish
    key: tuple                              # ranking key, lower = better
    source: str                             # "baseline", "sweep" or "brkga"

    @property
    def stall_key(self) -> tuple:
        """Quantized part of the ranking key (meaningful differences only)."""
        return self.key[:STALL_KEY_LENGTH]

    @property
    def passed(self) -> bool:
        """True when the solution passed the hard gate."""
        return self.key[0] == 0


# ============================================================================
# GATE AND RANKING
# ============================================================================

def resolve_gate(mode: str, baseline: Metrics, params: EvalParams) -> StabilityGate:
    """Turn the requested gate mode into concrete thresholds.

    'auto' uses the spec gate when the baseline itself meets the spec and the
    baseline gate otherwise.
    """
    if mode not in GATE_MODES:
        raise EvaluatorError(f"Stability gate must be one of {GATE_MODES}.")

    spec = StabilityGate("spec", params.required_margin, 0)
    if mode == "off":
        return StabilityGate("off", float("-inf"), _VIOLATION_CAP)
    if mode == "spec" or (mode == "auto" and spec.passes(baseline)):
        return spec
    return StabilityGate(
        "baseline",
        min(params.required_margin, baseline.stability_margin),
        baseline.unstable_units,
    )


def ranking_key(m: Metrics | None, gate: StabilityGate, params: EvalParams) -> tuple:
    """Sortable key of a solution; lower is better.

    (gate flag, violation units, violation margin,      <- hard gate
     pallet count,                                      <- tier 1
     -density bucket,                                   <- tier 2
     CoG bucket,                                        <- tier 3
     -density, CoG)                                     <- exact tie-breakers

    Solutions that fail the gate sort after all passing ones and are ordered
    by how badly they fail, which steers the search back to the feasible side.
    """
    if m is None:
        return (2, 0, 0, 0, 0, 0, 0.0, 0.0)

    if gate.passes(m):
        flag, violation_units, violation_margin = 0, 0, 0
    else:
        flag = 1
        if not m.geometry_ok:
            violation_units, violation_margin = _VIOLATION_CAP, 0
        else:
            violation_units = max(0, m.unstable_units - gate.max_unstable_units)
            shortfall = max(0.0, gate.min_margin - m.stability_margin)
            violation_margin = (
                int(round(shortfall / _VIOLATION_RESOLUTION))
                if shortfall < _VIOLATION_CAP * _VIOLATION_RESOLUTION
                else _VIOLATION_CAP
            )

    density_bucket = int(round(m.density_score / params.density_resolution))
    cog_bucket = int(round(m.cog_height_mm / params.cog_resolution_mm))
    return (
        flag,
        violation_units,
        violation_margin,
        m.pallet_count,
        -density_bucket,
        cog_bucket,
        -m.density_score,
        m.cog_height_mm,
    )


def format_metrics(m: Metrics | None) -> str:
    """One-line summary for log output."""
    if m is None:
        return "decoder failed"
    text = (
        f"{m.pallet_count} pallets | density {m.density_score:.4f} "
        f"(SSF {m.ssf:.4f}, envelope {m.envelope_utilization:.1%}) | "
        f"CoG {m.cog_height_mm:.0f} mm | "
        f"margin box {m.box_margin:.2f} ({m.box_unstable} below spec), "
        f"load tree {m.tree_margin:.2f} ({m.tree_unstable} below spec)"
    )
    return text if m.geometry_ok else text + " | GEOMETRY CHECK FAILED"


# ============================================================================
# EVALUATION
# ============================================================================

def evaluate(
    layout: Layout,
    box_specs: Sequence[BoxSpec],
    pallet: PalletSpec,
    params: EvalParams,
) -> Metrics:
    """Compute all metrics of one layout."""
    tol = params.placement_tol_mm
    n_pallets = layout.pallet_count

    # Mass used for CoG and stability. Orders without any weight data fall
    # back to volume (uniform density); only mass ratios matter.
    by_volume = sum(spec.weight_kg for spec in box_specs) <= 0.0

    # Per pallet, in placement order: (x, y, z, l, w, h, mass).
    items: list[list[tuple[float, float, float, float, float, float, float]]] = [
        [] for _ in range(n_pallets)
    ]
    volumes = [0.0] * n_pallets
    heights = [0.0] * n_pallets
    geometry_ok = len(layout.placements) == len(box_specs)
    mass_sum = 0.0
    mass_height_sum = 0.0

    for p in layout.placements:
        spec = box_specs[p.box_index]
        mass = (
            spec.volume_mm3 * _VOLUME_MASS_KG_PER_MM3 if by_volume
            else max(spec.weight_kg, _MIN_MASS_KG)
        )
        top = p.z + p.h
        if (
            p.x < -tol or p.y < -tol or p.z < -tol
            or p.x + p.l > pallet.length_mm + tol
            or p.y + p.w > pallet.width_mm + tol
            or top > pallet.max_height_mm + tol
        ):
            geometry_ok = False
        items[p.pallet_index].append((p.x, p.y, p.z, p.l, p.w, p.h, mass))
        volumes[p.pallet_index] += spec.volume_mm3
        if top > heights[p.pallet_index]:
            heights[p.pallet_index] = top
        mass_sum += mass
        mass_height_sum += mass * (p.z + p.h / 2.0)

    # ---- density ----------------------------------------------------------
    usable = pallet.usable_volume_mm3
    footprint = pallet.length_mm * pallet.width_mm
    fill_rates = tuple(v / usable for v in volumes)
    ssf = sum(f * f for f in fill_rates)
    envelope_volume = footprint * sum(heights)
    envelope = sum(volumes) / envelope_volume if envelope_volume > 0 else 0.0
    weight_sum = params.weight_ssf + params.weight_envelope
    mean_squared_fill = ssf / n_pallets if n_pallets else 0.0
    density = (
        params.weight_ssf * mean_squared_fill + params.weight_envelope * envelope
    ) / weight_sum

    # ---- centre of gravity -------------------------------------------------
    cog_height = mass_height_sum / mass_sum if mass_sum > 0 else 0.0

    # ---- stability ---------------------------------------------------------
    inf = float("inf")
    box_margin = tree_margin = inf
    box_unstable = tree_unstable = 0
    for pallet_items in items:
        b_margin, b_count, t_margin, t_count, supported = _pallet_stability(
            pallet_items, params
        )
        box_margin = min(box_margin, b_margin)
        tree_margin = min(tree_margin, t_margin)
        box_unstable += b_count
        tree_unstable += t_count
        if not supported:
            geometry_ok = False

    use_tree = params.stability_unit == "load_tree"
    return Metrics(
        geometry_ok=geometry_ok,
        pallet_count=n_pallets,
        fill_rates=fill_rates,
        ssf=ssf,
        envelope_utilization=envelope,
        density_score=density,
        cog_height_mm=cog_height,
        box_margin=box_margin,
        box_unstable=box_unstable,
        tree_margin=tree_margin,
        tree_unstable=tree_unstable,
        stability_margin=tree_margin if use_tree else box_margin,
        unstable_units=tree_unstable if use_tree else box_unstable,
    )


# ============================================================================
# STABILITY OF ONE PALLET
# ============================================================================

def _pallet_stability(
    items: Sequence[tuple[float, float, float, float, float, float, float]],
    params: EvalParams,
) -> tuple[float, int, float, int, bool]:
    """Replay one pallet in placement order.

    Returns (box margin, boxes below spec, load-tree margin, load trees below
    spec, every box supported).
    """
    inf = float("inf")
    n = len(items)
    if n == 0:
        return inf, 0, inf, 0, True

    # margin = (d / h_c) * (g / a_spec); pre-compute g / a_spec per axis.
    if params.travel_axis == "x":
        accel_x, accel_y = params.accel_travel_m_s2, params.accel_lateral_m_s2
    else:
        accel_x, accel_y = params.accel_lateral_m_s2, params.accel_travel_m_s2
    gx = params.gravity_m_s2 / accel_x
    gy = params.gravity_m_s2 / accel_y

    flush = params.support_flush_tol_mm
    tol = params.placement_tol_mm

    x1s = [0.0] * n
    y1s = [0.0] * n
    x2s = [0.0] * n
    y2s = [0.0] * n
    tops = [0.0] * n
    bottoms = [0.0] * n
    edges: list[tuple[Edge, ...]] = [()] * n
    # Supporters of each box as (index, share of the load passed to it,
    # x and y of the contact-patch centre).
    supporters: list[tuple[tuple[int, float, float, float], ...]] = [()] * n
    # Mass and first moments of each load tree (box + what rests on it).
    acc_m = [0.0] * n
    acc_x = [0.0] * n
    acc_y = [0.0] * n
    acc_z = [0.0] * n

    box_ratio = [inf] * n
    tree_worst = [inf] * n
    all_supported = True

    for i, (x, y, z, l, w, h, mass) in enumerate(items):
        x2 = x + l
        y2 = y + w
        x1s[i], y1s[i], x2s[i], y2s[i] = x, y, x2, y2
        bottoms[i] = z
        tops[i] = z + h

        # ---- support polygon and load shares -------------------------------
        if z <= tol:
            # On the pallet deck: the whole base is supported.
            hull = [(x, y), (x2, y), (x2, y2), (x, y2)]
        else:
            rects: list[tuple[float, float, float, float]] = []
            owners: list[int] = []
            for j in range(i):
                if abs(tops[j] - z) > flush:
                    continue
                rx1 = x if x > x1s[j] else x1s[j]
                rx2 = x2 if x2 < x2s[j] else x2s[j]
                ry1 = y if y > y1s[j] else y1s[j]
                ry2 = y2 if y2 < y2s[j] else y2s[j]
                if rx2 - rx1 > tol and ry2 - ry1 > tol:
                    rects.append((rx1, ry1, rx2, ry2))
                    owners.append(j)

            if not rects:
                all_supported = False
                hull = []
            elif len(rects) == 1:
                rx1, ry1, rx2, ry2 = rects[0]
                hull = [(rx1, ry1), (rx2, ry1), (rx2, ry2), (rx1, ry2)]
                supporters[i] = (
                    (owners[0], 1.0, (rx1 + rx2) / 2.0, (ry1 + ry2) / 2.0),
                )
            else:
                hull = convex_hull(
                    [
                        corner
                        for rx1, ry1, rx2, ry2 in rects
                        for corner in ((rx1, ry1), (rx2, ry1), (rx1, ry2), (rx2, ry2))
                    ]
                )
                areas = [(rx2 - rx1) * (ry2 - ry1) for rx1, ry1, rx2, ry2 in rects]
                total_area = sum(areas)
                supporters[i] = tuple(
                    (owner, area / total_area, (rx1 + rx2) / 2.0, (ry1 + ry2) / 2.0)
                    for owner, area, (rx1, ry1, rx2, ry2)
                    in zip(owners, areas, rects, strict=True)
                )
        edges[i] = polygon_edges(hull)

        # ---- the new box alone ----------------------------------------------
        cx = x + l / 2.0
        cy = y + w / 2.0
        cz = z + h / 2.0
        mx, my, mz = mass * cx, mass * cy, mass * cz
        acc_m[i], acc_x[i], acc_y[i], acc_z[i] = mass, mx, my, mz
        ratio = _unit_ratio(cx, cy, h / 2.0, edges[i], gx, gy)
        box_ratio[i] = ratio
        tree_worst[i] = ratio

        # ---- pass the new weight down the support chain ----------------------
        # incoming[a] = mass and first moments that box a receives because
        # of the new box. Boxes are processed from the highest placement
        # index down; every box that passes load to a has a higher index, so
        # incoming[a] is complete when a is taken from the heap.
        if supporters[i]:
            incoming: dict[int, list[float]] = {}
            heap: list[int] = []
            _pass_down(supporters[i], tops, mass, mx, my, mz, incoming, heap)
            while heap:
                a = -heapq.heappop(heap)
                dm, dmx, dmy, dmz = incoming[a]
                acc_m[a] += dm
                acc_x[a] += dmx
                acc_y[a] += dmy
                acc_z[a] += dmz
                # Intermediate-state check of the load tree of a.
                m_a = acc_m[a]
                ratio = _unit_ratio(
                    acc_x[a] / m_a,
                    acc_y[a] / m_a,
                    acc_z[a] / m_a - bottoms[a],
                    edges[a],
                    gx,
                    gy,
                )
                if ratio < tree_worst[a]:
                    tree_worst[a] = ratio
                if supporters[a]:
                    _pass_down(supporters[a], tops, dm, dmx, dmy, dmz, incoming, heap)

    required = params.required_margin - _MARGIN_EPS
    return (
        min(box_ratio),
        sum(1 for r in box_ratio if r < required),
        min(tree_worst),
        sum(1 for r in tree_worst if r < required),
        all_supported,
    )


def _pass_down(
    unit_supporters: Sequence[tuple[int, float, float, float]],
    tops: Sequence[float],
    dm: float,
    dmx: float,
    dmy: float,
    dmz: float,
    incoming: dict[int, list[float]],
    heap: list[int],
) -> None:
    """Hand a load (mass dm with first moments dmx, dmy, dmz) to the
    supporters of one box.

    One supporter:      rigid connection, the load keeps its position.
    Several supporters: the load is split by contact area and acts on each
                        supporter's top face at the contact-patch centre.
    """
    if len(unit_supporters) == 1:
        target = unit_supporters[0][0]
        entry = incoming.get(target)
        if entry is None:
            incoming[target] = [dm, dmx, dmy, dmz]
            heapq.heappush(heap, -target)
        else:
            entry[0] += dm
            entry[1] += dmx
            entry[2] += dmy
            entry[3] += dmz
        return

    for target, share, px, py in unit_supporters:
        part = share * dm
        entry = incoming.get(target)
        if entry is None:
            incoming[target] = [part, part * px, part * py, part * tops[target]]
            heapq.heappush(heap, -target)
        else:
            entry[0] += part
            entry[1] += part * px
            entry[2] += part * py
            entry[3] += part * tops[target]


def _unit_ratio(
    cx: float,
    cy: float,
    cog_height: float,
    unit_edges: Sequence[Edge],
    gx: float,
    gy: float,
) -> float:
    """a_crit / a_spec of one unit: worst of the four horizontal directions.

    cx, cy       CoG projection of the unit [mm]
    cog_height   CoG height above the unit's support plane [mm]
    gx, gy       g / a_spec for the X and Y axes
    """
    if cog_height <= 0.0:
        return float("inf")
    x_pos, x_neg, y_pos, y_neg = axis_exit_distances(cx, cy, unit_edges)
    along_x = x_pos if x_pos < x_neg else x_neg
    along_y = y_pos if y_pos < y_neg else y_neg
    rx = along_x * gx
    ry = along_y * gy
    return (rx if rx < ry else ry) / cog_height