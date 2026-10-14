"""Loader comparison harness for the pallet stacker.

Runs two loaders on the same box-data JSON file and compares the results in
an ASCII table: packing efficiency, static stability and quasi-static
dynamic stability (how hard the pallet can accelerate before a box slides or
tips). Finishes with a weighted verdict and the reasons behind it.

* Which loaders are compared is set here (BASE_LOADER, COMPARISON_LOADER);
  ``settings.ACTIVE_LOADER`` is ignored.
* A loader name ``foo`` refers to the module ``pallet_stacker/fooLoader.py``,
  which must provide ``load_boxes(boxes) -> list[Pallet]``.
* Each loader receives its own freshly parsed boxes, so the runs cannot
  influence each other.
* The only files written are the two Plotly layouts
  ``<results>/test_results/<loaderName>_layout.html``. Plotly JS is embedded
  in each HTML file so no extra asset files are created. No Gazebo output.

Run from the same directory as main.py:
    python loaderTester.py
"""

from __future__ import annotations

import importlib
import sys
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from functools import reduce
from math import ceil, gcd, hypot, inf, isfinite
from pathlib import Path

import numpy as np

from pallet_stacker import settings
from pallet_stacker.pallet import Pallet
from pallet_stacker.processBoxData import BoxDataError, parse_box_json, select_json_file


# ============================================================================
# TESTER SETTINGS
# ============================================================================

# --- Loaders ------------------------------------------------------------------

BASE_LOADER: str = "experimental"            # reference algorithm
COMPARISON_LOADER: str = "advanced"  # candidate algorithm

# --- Input / output -----------------------------------------------------------

# None opens the file dialog. A Path skips the dialog.
INPUT_FILE_PATH: Path | None = None

RESULTS_DIRECTORY: Path = settings.OUTPUT_DIRECTORY / "test_results"
WRITE_PLOTLY_LAYOUTS: bool = True
OPEN_PLOTLY_IN_BROWSER: bool = settings.PLOTLY_OPEN_IN_BROWSER

# --- Physics ------------------------------------------------------------------

GRAVITY_MPS2: float = 9.81

# Static friction coefficients. Published slide-angle results for corrugated
# board are typically around 0.3..0.6 depending on liner, humidity and wear;
# 0.45 is a mid-range assumption for untreated kraft liner.
MU_BOX_ON_BOX: float = 0.45
MU_BOX_ON_PALLET: float = 0.45   # carton on a wooden pallet deck

# Mass used for boxes whose JSON weight is zero (CoG needs positive mass).
MIN_BOX_MASS_KG: float = 0.05

# A side face counts as restrained when at least this fraction of its area is
# in contact with a neighbouring box. A restrained face is assumed not to
# slide or tip in that direction (the neighbour props it).
RESTRAINT_MIN_FACE_FRACTION: float = 0.30

# Reference horizontal accelerations [g] for the "boxes at risk" counts.
# EN 12195-1 road values: 0.8 g forward, 0.5 g sideways/backward. The forklift
# value is an assumed hard stop, not a standard.
REFERENCE_ACCELERATIONS_G: tuple[tuple[str, float], ...] = (
    ("forklift hard stop", 0.30),
    ("road sideways/backward", 0.50),
    ("road forward", 0.80),
)

# Support ratio below which a box is counted as poorly supported.
LOW_SUPPORT_THRESHOLD: float = 0.80

# Gap tolerance for contact detection [mm].
CONTACT_TOL_MM: float = 0.5

# --- Verdict ------------------------------------------------------------------

# Values within this relative difference count as a tie.
TIE_RELATIVE_TOL: float = 0.01

# The verdict is "too close to call" when the weighted point difference is
# below this fraction of all contested weight.
CLEAR_WIN_FRACTION: float = 0.10

# Number of reasons listed for the winner and trade-offs in the conclusion.
VERDICT_REASONS: int = 6
VERDICT_TRADEOFFS: int = 3


# ============================================================================
# REPORT TYPES
# ============================================================================

_DIRECTIONS: tuple[str, ...] = ("+X", "-X", "+Y", "-Y")


@dataclass(frozen=True, slots=True)
class _Body:
    """Axis-aligned placed box [mm, kg]."""

    x0: float
    y0: float
    z0: float
    x1: float
    y1: float
    z1: float
    mass: float

    @property
    def footprint(self) -> float:
        return (self.x1 - self.x0) * (self.y1 - self.y0)

    @property
    def volume(self) -> float:
        return self.footprint * (self.z1 - self.z0)


@dataclass(slots=True)
class PalletReport:
    pallet_id: str
    box_count: int
    mass_kg: float
    load_height_mm: float
    box_volume_mm3: float
    envelope_volume_mm3: float     # L * W * load height
    usable_volume_mm3: float       # L * W * max height
    skyline_volume_mm3: float      # volume under the top surface
    roughness_mm: float
    support_ratios: list[float]
    floating: int
    static_failures: int
    min_static_margin_mm: float
    cog_height_mm: float
    cog_offset_mm: float
    crit_x_g: float
    crit_y_g: float
    crit_g: float
    box_crit_g: list[float]        # per box, worst direction
    exposed: int                   # boxes unrestrained in >= 1 direction


@dataclass(slots=True)
class RunReport:
    name: str
    ok: bool
    error: str = ""
    runtime_s: float = 0.0
    pallets: list[Pallet] = field(default_factory=list)
    pallet_reports: list[PalletReport] = field(default_factory=list)
    metrics: dict[str, float] = field(default_factory=dict)
    layout_path: Path | None = None


@dataclass(frozen=True, slots=True)
class _Metric:
    key: str
    label: str
    section: str
    fmt: Callable[[float], str]
    diff: Callable[[float], str]
    better: str | None    # "higher", "lower" or None (informational)
    weight: float


# ============================================================================
# MAIN
# ============================================================================

def main() -> int:
    _validate_tester_settings()

    path = INPUT_FILE_PATH
    if path is None:
        path = select_json_file(settings.INPUT_DIRECTORY)
        if path is None:
            print("File selection cancelled.")
            return 0

    reference = parse_box_json(path)
    _print_intro(reference.source_path, reference.boxes)

    runs = [_run_loader(name, reference.source_path)
            for name in (BASE_LOADER, COMPARISON_LOADER)]

    if WRITE_PLOTLY_LAYOUTS:
        _write_layouts(runs, reference.source_path)

    metrics = _metric_specs()
    _print_comparison(runs, metrics, reference.box_count)
    for run in runs:
        _print_pallet_table(run)
    _print_verdict(runs, metrics)
    return 0


def _run_loader(name: str, source: Path) -> RunReport:
    """Import, run and analyse one loader on freshly parsed boxes."""
    print(f"Running {name!r} ...", end=" ", flush=True)
    try:
        module = importlib.import_module(f"pallet_stacker.{name}Loader")
        load_boxes = getattr(module, "load_boxes")
    except (ImportError, AttributeError) as exc:
        print("FAILED (import)")
        return RunReport(name=name, ok=False, error=f"import failed: {exc}")

    boxes = parse_box_json(source).boxes
    start = time.perf_counter()
    try:
        pallets = list(load_boxes(boxes))
    except Exception as exc:  # report any loader failure instead of aborting
        runtime = time.perf_counter() - start
        print(f"FAILED after {runtime:.2f} s")
        return RunReport(name=name, ok=False, error=f"{type(exc).__name__}: {exc}",
                         runtime_s=runtime)
    runtime = time.perf_counter() - start
    print(f"done in {runtime:.2f} s")

    run = RunReport(name=name, ok=True, runtime_s=runtime, pallets=pallets)
    run.pallet_reports = [_analyse_pallet(p) for p in pallets]
    run.metrics = _aggregate(run, len(boxes))
    return run


def _write_layouts(runs: Sequence[RunReport], source: Path) -> None:
    from pallet_stacker.plotlyVisualizer import write_layout_html

    for run in runs:
        if not run.ok:
            continue
        run.layout_path = write_layout_html(
            run.pallets,
            RESULTS_DIRECTORY / f"{run.name}_layout.html",
            title=f"{run.name} loader - {source.name}",
            open_in_browser=OPEN_PLOTLY_IN_BROWSER,
            include_plotlyjs=True,
        )


# ============================================================================
# PER-PALLET ANALYSIS
# ============================================================================

def _analyse_pallet(pallet: Pallet) -> PalletReport:
    bodies = _bodies(pallet)
    n = len(bodies)
    length, width = pallet.length_mm, pallet.width_mm
    tol = CONTACT_TOL_MM

    # ---- contacts --------------------------------------------------------------
    # supports[i]: (supporter index or -1 for deck, contact rect, area)
    supports: list[list[tuple[int, tuple[float, float, float, float], float]]] = [
        [] for _ in range(n)
    ]
    side_area = np.zeros((n, 4), dtype=np.float64)

    for i, b in enumerate(bodies):
        if b.z0 <= tol:
            supports[i].append((-1, (b.x0, b.y0, b.x1, b.y1), b.footprint))
        for j, a in enumerate(bodies):
            if i == j:
                continue
            ox = min(a.x1, b.x1) - max(a.x0, b.x0)
            oy = min(a.y1, b.y1) - max(a.y0, b.y0)
            oz = min(a.z1, b.z1) - max(a.z0, b.z0)

            # a supports b from below
            if b.z0 > tol and abs(a.z1 - b.z0) <= tol and ox > tol and oy > tol:
                rect = (max(a.x0, b.x0), max(a.y0, b.y0), min(a.x1, b.x1), min(a.y1, b.y1))
                supports[i].append((j, rect, ox * oy))

            # a touches a side face of b
            if oz > tol:
                if oy > tol and abs(a.x0 - b.x1) <= tol:
                    side_area[i, 0] += oy * oz        # +X face
                if oy > tol and abs(a.x1 - b.x0) <= tol:
                    side_area[i, 1] += oy * oz        # -X face
                if ox > tol and abs(a.y0 - b.y1) <= tol:
                    side_area[i, 2] += ox * oz        # +Y face
                if ox > tol and abs(a.y1 - b.y0) <= tol:
                    side_area[i, 3] += ox * oz        # -Y face

    # ---- load path, static margin, critical accelerations (top-down) --------
    received_m = np.zeros(n)
    received_mx = np.zeros(n)
    received_my = np.zeros(n)
    received_mz = np.zeros(n)

    support_ratios: list[float] = []
    box_crit: list[float] = []
    crit_axis = [inf, inf]           # X, Y
    floating = 0
    static_failures = 0
    min_margin = inf
    exposed = 0

    for i in sorted(range(n), key=lambda k: bodies[k].z0, reverse=True):
        b = bodies[i]
        cx, cy, cz = (b.x0 + b.x1) / 2, (b.y0 + b.y1) / 2, (b.z0 + b.z1) / 2

        total_m = b.mass + received_m[i]
        gx = (b.mass * cx + received_mx[i]) / total_m
        gy = (b.mass * cy + received_my[i]) / total_m
        gz = (b.mass * cz + received_mz[i]) / total_m

        contact_area = sum(area for _, _, area in supports[i])
        support_ratios.append(min(1.0, contact_area / b.footprint))
        on_deck = any(j == -1 for j, _, _ in supports[i])

        corners = [
            (x, y)
            for _, (rx0, ry0, rx1, ry1), _ in supports[i]
            for x in (rx0, rx1)
            for y in (ry0, ry1)
        ]
        hull = _convex_hull(corners)
        margin = _signed_margin(gx, gy, hull)
        min_margin = min(min_margin, margin)
        if not supports[i]:
            floating += 1
        if margin < 0.0:
            static_failures += 1

        # Tipping about the extreme support line in each direction:
        # a_tip / g = horizontal lever / CoG height above the box bottom.
        lever_h = max(gz - b.z0, 1e-9)
        if len(hull) >= 3:
            hx = [p[0] for p in hull]
            hy = [p[1] for p in hull]
            levers = (max(hx) - gx, gx - min(hx), max(hy) - gy, gy - min(hy))
        else:
            levers = (0.0, 0.0, 0.0, 0.0)
        mu = MU_BOX_ON_PALLET if on_deck else MU_BOX_ON_BOX

        x_face = (b.y1 - b.y0) * (b.z1 - b.z0)
        y_face = (b.x1 - b.x0) * (b.z1 - b.z0)
        face = (x_face, x_face, y_face, y_face)

        crit_dirs: list[float] = []
        unrestrained = False
        for k in range(4):
            if side_area[i, k] / face[k] >= RESTRAINT_MIN_FACE_FRACTION:
                crit_dirs.append(inf)
            else:
                unrestrained = True
                crit_dirs.append(min(mu, max(levers[k], 0.0) / lever_h))
        if unrestrained:
            exposed += 1
        box_crit.append(min(crit_dirs))
        crit_axis[0] = min(crit_axis[0], crit_dirs[0], crit_dirs[1])
        crit_axis[1] = min(crit_axis[1], crit_dirs[2], crit_dirs[3])

        # Pass this box and its load to the boxes below, by contact area,
        # applied at each contact rectangle's centroid.
        below = [(j, rect, area) for j, rect, area in supports[i] if j >= 0]
        below_area = sum(area for _, _, area in below)
        for j, (rx0, ry0, rx1, ry1), area in below:
            share = total_m * area / below_area
            received_m[j] += share
            received_mx[j] += share * (rx0 + rx1) / 2
            received_my[j] += share * (ry0 + ry1) / 2
            received_mz[j] += share * gz

    # ---- pallet-level geometry ---------------------------------------------------
    total_mass = sum(b.mass for b in bodies)
    load_height = max((b.z1 for b in bodies), default=0.0)
    if total_mass > 0:
        cog_x = sum(b.mass * (b.x0 + b.x1) / 2 for b in bodies) / total_mass
        cog_y = sum(b.mass * (b.y0 + b.y1) / 2 for b in bodies) / total_mass
        cog_z = sum(b.mass * (b.z0 + b.z1) / 2 for b in bodies) / total_mass
    else:
        cog_x, cog_y, cog_z = length / 2, width / 2, 0.0

    box_volume = sum(b.volume for b in bodies)
    skyline_volume, roughness = _surface_metrics(bodies, length, width)

    return PalletReport(
        pallet_id=pallet.pallet_id,
        box_count=n,
        mass_kg=sum(box.weight_kg for box in pallet.boxes),
        load_height_mm=load_height,
        box_volume_mm3=box_volume,
        envelope_volume_mm3=length * width * load_height,
        usable_volume_mm3=pallet.usable_volume_mm3,
        skyline_volume_mm3=skyline_volume,
        roughness_mm=roughness,
        support_ratios=support_ratios,
        floating=floating,
        static_failures=static_failures,
        min_static_margin_mm=min_margin,
        cog_height_mm=cog_z,
        cog_offset_mm=hypot(cog_x - length / 2, cog_y - width / 2),
        crit_x_g=crit_axis[0],
        crit_y_g=crit_axis[1],
        crit_g=min(crit_axis),
        box_crit_g=box_crit,
        exposed=exposed,
    )


def _bodies(pallet: Pallet) -> list[_Body]:
    bodies: list[_Body] = []
    for box in pallet.boxes:
        lo, hi = box.bounds()
        bodies.append(_Body(lo.x, lo.y, lo.z, hi.x, hi.y, hi.z,
                            max(box.weight_kg, MIN_BOX_MASS_KG)))
    return bodies


def _surface_metrics(bodies: Sequence[_Body], length: float, width: float) -> tuple[float, float]:
    """(volume under the top surface [mm3], mean step between adjacent cells [mm])."""
    if not bodies:
        return 0.0, 0.0

    values = [length, width]
    for b in bodies:
        values.extend((b.x0, b.x1, b.y0, b.y1))
    ints = [int(round(v)) for v in values]
    res = reduce(gcd, ints) if all(abs(v - r) < 1e-6 for v, r in zip(values, ints)) else 1
    res = max(res, 1)

    nx, ny = int(round(length / res)), int(round(width / res))
    hm = np.zeros((nx, ny), dtype=np.float64)
    for b in bodies:
        ix0, ix1 = int(round(b.x0 / res)), int(round(b.x1 / res))
        iy0, iy1 = int(round(b.y0 / res)), int(round(b.y1 / res))
        window = hm[ix0:ix1, iy0:iy1]
        np.maximum(window, b.z1, out=window)

    steps = np.concatenate((np.abs(np.diff(hm, axis=0)).ravel(),
                            np.abs(np.diff(hm, axis=1)).ravel()))
    roughness = float(steps.mean()) if steps.size else 0.0
    return float(hm.sum()) * res * res, roughness


def _convex_hull(points: Sequence[tuple[float, float]]) -> list[tuple[float, float]]:
    """Andrew's monotone chain; counter-clockwise, collinear points removed."""
    pts = sorted(set(points))
    if len(pts) <= 2:
        return list(pts)

    def cross(o: tuple[float, float], a: tuple[float, float], b: tuple[float, float]) -> float:
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower: list[tuple[float, float]] = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper: list[tuple[float, float]] = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def _signed_margin(px: float, py: float, hull: Sequence[tuple[float, float]]) -> float:
    """Distance from the point to the hull border [mm]; negative = outside."""
    if len(hull) < 3:
        return -inf
    margin = inf
    for k in range(len(hull)):
        ax, ay = hull[k]
        bx, by = hull[(k + 1) % len(hull)]
        edge = hypot(bx - ax, by - ay)
        margin = min(margin, ((bx - ax) * (py - ay) - (by - ay) * (px - ax)) / edge)
    return margin


# ============================================================================
# RUN-LEVEL AGGREGATION
# ============================================================================

def _aggregate(run: RunReport, input_count: int) -> dict[str, float]:
    reports = run.pallet_reports
    placed = sum(r.box_count for r in reports)
    m: dict[str, float] = {
        "boxes_placed": float(placed),
        "pallets": float(len(reports)),
        "runtime": run.runtime_s,
        "ms_per_box": 1000.0 * run.runtime_s / max(input_count, 1),
    }
    if not reports or placed == 0:
        return m

    box_vol = sum(r.box_volume_mm3 for r in reports)
    envelope = sum(r.envelope_volume_mm3 for r in reports)
    skyline = sum(r.skyline_volume_mm3 for r in reports)
    supports = [s for r in reports for s in r.support_ratios]
    box_crit = [c for r in reports for c in r.box_crit_g]
    total_mass = sum(r.mass_kg for r in reports)

    m["vol_util"] = box_vol / sum(r.usable_volume_mm3 for r in reports)
    m["fill_density"] = box_vol / envelope if envelope > 0 else 0.0
    m["trapped_void"] = (skyline - box_vol) / skyline if skyline > 0 else 0.0
    m["roughness"] = sum(r.roughness_mm for r in reports) / len(reports)
    m["mean_height"] = sum(r.load_height_mm for r in reports) / len(reports)
    m["mass_density"] = total_mass / (envelope * 1e-9) if envelope > 0 else 0.0

    m["support_mean"] = sum(supports) / len(supports)
    m["support_min"] = min(supports)
    m["low_support"] = float(sum(1 for s in supports if s < LOW_SUPPORT_THRESHOLD))
    m["floating"] = float(sum(r.floating for r in reports))
    m["static_fail"] = float(sum(r.static_failures for r in reports))
    m["static_margin"] = min(r.min_static_margin_mm for r in reports)
    m["cog_ratio"] = sum(
        r.cog_height_mm / r.load_height_mm for r in reports if r.load_height_mm > 0
    ) / len(reports)
    m["cog_height"] = sum(r.cog_height_mm for r in reports) / len(reports)
    m["cog_offset"] = max(r.cog_offset_mm for r in reports)

    m["acc_worst"] = min(r.crit_g for r in reports)
    m["acc_x"] = min(r.crit_x_g for r in reports)
    m["acc_y"] = min(r.crit_y_g for r in reports)
    finite = [r.crit_g for r in reports if isfinite(r.crit_g)]
    m["acc_mean"] = sum(finite) / len(finite) if finite else inf
    for k, (_, ref_g) in enumerate(REFERENCE_ACCELERATIONS_G):
        m[f"risk_{k}"] = float(sum(1 for c in box_crit if c < ref_g))
    m["exposed"] = sum(r.exposed for r in reports) / placed
    return m


# ============================================================================
# METRIC DEFINITIONS
# ============================================================================

def _f_int(v: float) -> str:
    return f"{int(round(v))}"


def _d_int(v: float) -> str:
    return f"{int(round(v)):+d}"


def _f_pct(v: float) -> str:
    return f"{100 * v:.1f} %"


def _d_pct(v: float) -> str:
    return f"{100 * v:+.1f} pp"


def _f_mm(v: float) -> str:
    return "n/a" if not isfinite(v) else f"{v:.0f} mm"


def _d_mm(v: float) -> str:
    return f"{v:+.0f} mm"


def _f_g(v: float) -> str:
    return "no limit" if not isfinite(v) else f"{v:.3f} g"


def _d_g(v: float) -> str:
    return f"{v:+.3f} g"


def _f_s(v: float) -> str:
    return f"{v:.2f} s"


def _d_s(v: float) -> str:
    return f"{v:+.2f} s"


def _f_ms(v: float) -> str:
    return f"{v:.1f} ms"


def _d_ms(v: float) -> str:
    return f"{v:+.1f} ms"


def _f_density(v: float) -> str:
    return f"{v:.0f} kg/m3"


def _d_density(v: float) -> str:
    return f"{v:+.0f} kg/m3"


def _metric_specs() -> list[_Metric]:
    pack, static, dynamic, perf = (
        "PACKING EFFICIENCY", "STATIC STABILITY", "DYNAMIC STABILITY (quasi-static)", "PERFORMANCE"
    )
    specs = [
        _Metric("boxes_placed", "Boxes placed", pack, _f_int, _d_int, "higher", 0.0),
        _Metric("pallets", "Pallets used", pack, _f_int, _d_int, "lower", 6.0),
        _Metric("vol_util", "Volume utilization (max-height volume)", pack, _f_pct, _d_pct, "higher", 4.0),
        _Metric("fill_density", "Fill density of loaded envelope", pack, _f_pct, _d_pct, "higher", 3.0),
        _Metric("trapped_void", "Voids trapped under top surface", pack, _f_pct, _d_pct, "lower", 2.0),
        _Metric("roughness", "Top-surface roughness (mean step)", pack, _f_mm, _d_mm, "lower", 1.0),
        _Metric("mean_height", "Mean load height", pack, _f_mm, _d_mm, None, 0.0),
        _Metric("mass_density", "Load mass density", pack, _f_density, _d_density, None, 0.0),

        _Metric("support_mean", "Mean support ratio", static, _f_pct, _d_pct, "higher", 2.0),
        _Metric("support_min", "Minimum support ratio", static, _f_pct, _d_pct, "higher", 1.0),
        _Metric("low_support", f"Boxes with < {LOW_SUPPORT_THRESHOLD:.0%} support", static,
                _f_int, _d_int, "lower", 1.0),
        _Metric("floating", "Floating boxes (no support)", static, _f_int, _d_int, "lower", 5.0),
        _Metric("static_fail", "Load-path CoG outside support", static, _f_int, _d_int, "lower", 5.0),
        _Metric("static_margin", "Smallest CoG margin to support edge", static, _f_mm, _d_mm, "higher", 1.0),
        _Metric("cog_ratio", "CoG height / load height (mean)", static, _f_pct, _d_pct, "lower", 2.0),
        _Metric("cog_height", "CoG height above deck (mean)", static, _f_mm, _d_mm, None, 0.0),
        _Metric("cog_offset", "Largest horizontal CoG offset", static, _f_mm, _d_mm, "lower", 1.0),

        _Metric("acc_worst", "Critical accel., worst direction", dynamic, _f_g, _d_g, "higher", 4.0),
        _Metric("acc_x", "Critical accel., forklift axis (+-X)", dynamic, _f_g, _d_g, "higher", 2.0),
        _Metric("acc_y", "Critical accel., lateral (+-Y)", dynamic, _f_g, _d_g, "higher", 2.0),
        _Metric("acc_mean", "Critical accel., mean of pallets", dynamic, _f_g, _d_g, "higher", 2.0),
    ]
    risk_weights = (3.0, 2.0, 1.0)
    for k, (label, ref_g) in enumerate(REFERENCE_ACCELERATIONS_G):
        weight = risk_weights[k] if k < len(risk_weights) else 1.0
        specs.append(_Metric(f"risk_{k}", f"At risk @{ref_g:.2f} g ({label})",
                             dynamic, _f_int, _d_int, "lower", weight))
    specs += [
        _Metric("exposed", "Boxes not side-restrained (>= 1 dir.)", dynamic, _f_pct, _d_pct, "lower", 1.0),
        _Metric("runtime", "Loader runtime", perf, _f_s, _d_s, "lower", 0.5),
        _Metric("ms_per_box", "Runtime per box", perf, _f_ms, _d_ms, None, 0.0),
    ]
    return specs


def _winner(metric: _Metric, a: float, b: float) -> int | None:
    """0 = base better, 1 = comparison better, None = tie / informational."""
    if metric.better is None:
        return None
    if a == b or (not isfinite(a) and not isfinite(b)):
        return None
    if isfinite(a) and isfinite(b) and abs(a - b) <= TIE_RELATIVE_TOL * max(abs(a), abs(b)):
        return None
    if metric.better == "higher":
        return 0 if a > b else 1
    return 0 if a < b else 1


# ============================================================================
# CONSOLE OUTPUT
# ============================================================================

def _print_intro(source: Path, boxes: Sequence) -> None:
    usable = settings.PALLET_LENGTH_MM * settings.PALLET_WIDTH_MM * settings.PALLET_MAX_HEIGHT_MM
    total_vol = sum(b.volume_mm3 for b in boxes)
    total_mass = sum(b.weight_kg for b in boxes)
    bound = ceil(total_vol / usable) if usable > 0 else 0
    if settings.PALLET_MAX_LOAD_KG:
        bound = max(bound, ceil(total_mass / settings.PALLET_MAX_LOAD_KG))

    refs = ", ".join(f"{g:.2f} g {name}" for name, g in REFERENCE_ACCELERATIONS_G)
    lines = [
        "=" * 92,
        "PALLET LOADER COMPARISON",
        "=" * 92,
        f"Input           : {source}",
        f"Boxes           : {len(boxes)}   total volume {total_vol * 1e-9:.3f} m3   "
        f"total mass {total_mass:.1f} kg",
        f"Pallet          : {settings.PALLET_LENGTH_MM:g} x {settings.PALLET_WIDTH_MM:g} mm, "
        f"max height {settings.PALLET_MAX_HEIGHT_MM:g} mm, max load {settings.PALLET_MAX_LOAD_KG} kg",
        f"Lower bound     : {bound} pallet(s) (volume / weight only, ignores geometry)",
        f"Loaders         : base = {BASE_LOADER!r}, comparison = {COMPARISON_LOADER!r}",
        "",
        "TESTS",
        "  Packing efficiency",
        "    Volume utilization   box volume / (pallets x L x W x max height).",
        "    Fill density         box volume / (L x W x actual load height).",
        "    Trapped voids        empty space below the top surface (sealed under boxes).",
        "    Roughness            mean height step between neighbouring surface cells.",
        "  Static stability (geometric contact, not the loaders' own checks)",
        "    Support ratio        bottom area resting on a box or deck / bottom area.",
        "    Load-path CoG        every box's own weight plus the load it carries (shared",
        "                         down by contact area) must have its CoG inside the box's",
        "                         support polygon. Margin = distance to the polygon edge.",
        "    CoG height           centre of gravity of the whole load above the deck.",
        "  Dynamic stability (quasi-static rigid bodies under horizontal acceleration)",
        f"    Sliding              interface slides at mu*g (box/box {MU_BOX_ON_BOX}, "
        f"box/pallet {MU_BOX_ON_PALLET}).",
        "    Tipping              box with its carried load tips about its support edge at",
        "                         a = g * lever / CoG height.",
        f"    Restraint            a face with >= {RESTRAINT_MIN_FACE_FRACTION:.0%} contact "
        "to a neighbour is propped",
        "                         and cannot slide or tip that way.",
        "    Critical accel.      lowest sliding/tipping limit of any unrestrained box",
        "                         (1 g = 9.81 m/s2). X is the forklift travel axis.",
        f"    Boxes at risk        unrestrained boxes failing below: {refs}.",
        "",
        "Notes: no lashing or stretch wrap is modelled; boxes are rigid. Results rank",
        "       layouts against each other; they are not a load-securing certificate.",
        "=" * 92,
        "",
    ]
    print("\n".join(lines))


def _render_table(
    columns: Sequence[tuple[str, int, str]],
    rows: Sequence[Sequence[str] | str],
) -> str:
    """columns: (header, width, '<' or '>'); a str row is a full-width section."""
    border = "+" + "+".join("-" * (w + 2) for _, w, _ in columns) + "+"
    inner = len(border) - 4

    def line(cells: Sequence[str]) -> str:
        out = []
        for text, (_, width, align) in zip(cells, columns):
            text = text if len(text) <= width else text[: width - 1] + "~"
            out.append(f"{text:{align}{width}}")
        return "| " + " | ".join(out) + " |"

    out = [border, line([h for h, _, _ in columns]), border.replace("-", "=")]
    for row in rows:
        if isinstance(row, str):
            if out[-1] not in (border, border.replace("-", "=")):
                out.append(border)
            out.append(f"| {row:<{inner}} |")
            out.append(border)
        else:
            out.append(line(row))
    out.append(border)
    return "\n".join(out)


def _print_comparison(runs: Sequence[RunReport], metrics: Sequence[_Metric], input_count: int) -> None:
    base, comp = runs
    for run in runs:
        if not run.ok:
            print(f"\n!! Loader {run.name!r} failed: {run.error}")

    width = max(14, len(base.name), len(comp.name))
    columns = (
        ("Metric", 42, "<"),
        (base.name, width, ">"),
        (comp.name, width, ">"),
        ("Difference", 13, ">"),
        ("Better", width, "<"),
    )

    rows: list[Sequence[str] | str] = []
    section = ""
    for metric in metrics:
        if metric.section != section:
            section = metric.section
            rows.append(section)
        a = base.metrics.get(metric.key)
        b = comp.metrics.get(metric.key)
        va = "FAILED" if not base.ok else ("-" if a is None else metric.fmt(a))
        vb = "FAILED" if not comp.ok else ("-" if b is None else metric.fmt(b))
        if a is None or b is None:
            rows.append((metric.label, va, vb, "-", "-"))
            continue
        diff = metric.diff(b - a) if isfinite(a) and isfinite(b) else "-"
        win = _winner(metric, a, b)
        better = "-" if metric.better is None else ("tie" if win is None else runs[win].name)
        rows.append((metric.label, va, vb, diff, better))

    print()
    print(f"COMPARISON  (input boxes: {input_count}; difference = {comp.name} - {base.name})")
    print(_render_table(columns, rows))
    for run in runs:
        if run.layout_path is not None:
            print(f"Plotly layout ({run.name}): {run.layout_path}")


def _print_pallet_table(run: RunReport) -> None:
    if not run.ok or not run.pallet_reports:
        return
    columns = (
        ("Pallet", 11, "<"),
        ("Boxes", 5, ">"),
        ("Mass kg", 8, ">"),
        ("Height", 7, ">"),
        ("Vol util", 8, ">"),
        ("Fill", 7, ">"),
        ("Voids", 7, ">"),
        ("CoG z/H", 7, ">"),
        ("a_crit X", 9, ">"),
        ("a_crit Y", 9, ">"),
        ("CoG fail", 8, ">"),
    )
    rows: list[Sequence[str] | str] = []
    for r in run.pallet_reports:
        fill = r.box_volume_mm3 / r.envelope_volume_mm3 if r.envelope_volume_mm3 else 0.0
        voids = ((r.skyline_volume_mm3 - r.box_volume_mm3) / r.skyline_volume_mm3
                 if r.skyline_volume_mm3 else 0.0)
        cog = r.cog_height_mm / r.load_height_mm if r.load_height_mm else 0.0
        rows.append((
            r.pallet_id,
            str(r.box_count),
            f"{r.mass_kg:.1f}",
            f"{r.load_height_mm:.0f}",
            _f_pct(r.box_volume_mm3 / r.usable_volume_mm3),
            _f_pct(fill),
            _f_pct(voids),
            _f_pct(cog),
            _f_g(r.crit_x_g),
            _f_g(r.crit_y_g),
            str(r.static_failures),
        ))
    print()
    print(f"PER PALLET - {run.name}")
    print(_render_table(columns, rows))


def _print_verdict(runs: Sequence[RunReport], metrics: Sequence[_Metric]) -> None:
    base, comp = runs
    print()
    print("=" * 92)
    print("VERDICT")
    print("=" * 92)

    # ---- hard gates ------------------------------------------------------------
    if not base.ok and not comp.ok:
        print("Both loaders failed; no comparison possible.")
        return
    if not base.ok or not comp.ok:
        good, bad = (comp, base) if not base.ok else (base, comp)
        print(f"{good.name!r} is better: {bad.name!r} did not complete ({bad.error}).")
        return
    placed_a = base.metrics["boxes_placed"]
    placed_b = comp.metrics["boxes_placed"]
    if placed_a != placed_b:
        good = base if placed_a > placed_b else comp
        print(f"{good.name!r} is better: it placed {int(max(placed_a, placed_b))} boxes "
              f"versus {int(min(placed_a, placed_b))}. Placing every box outweighs all other metrics.")
        return

    # ---- weighted points ----------------------------------------------------------
    points = [0.0, 0.0]
    contested = 0.0
    section_points: dict[str, list[float]] = {}
    decided: list[tuple[float, int, _Metric, float, float]] = []

    for metric in metrics:
        if metric.better is None or metric.weight <= 0:
            continue
        a = base.metrics.get(metric.key)
        b = comp.metrics.get(metric.key)
        if a is None or b is None:
            continue
        contested += metric.weight
        win = _winner(metric, a, b)
        section_points.setdefault(metric.section, [0.0, 0.0])
        if win is None:
            continue
        points[win] += metric.weight
        section_points[metric.section][win] += metric.weight
        decided.append((metric.weight, win, metric, a, b))

    print(f"Weighted points  : {base.name} {points[0]:.1f}  |  {comp.name} {points[1]:.1f}  "
          f"(of {contested:.1f} contested; ties score nothing)")
    for section, (pa, pb) in section_points.items():
        print(f"  {section:<34} {base.name} {pa:5.1f}  |  {comp.name} {pb:5.1f}")
    print()

    gap = abs(points[0] - points[1])
    leader = 0 if points[0] >= points[1] else 1
    clear = contested > 0 and gap >= CLEAR_WIN_FRACTION * contested
    if clear:
        print(f"{runs[leader].name!r} is better than {runs[1 - leader].name!r} on this input.")
    else:
        print(f"Too close to call: the point gap ({gap:.1f}) is below "
              f"{CLEAR_WIN_FRACTION:.0%} of the contested weight. "
              f"{runs[leader].name!r} leads narrowly.")

    def describe(metric: _Metric, a: float, b: float) -> str:
        return (f"{metric.label}: {base.name} {metric.fmt(a)} vs "
                f"{comp.name} {metric.fmt(b)} (weight {metric.weight:g})")

    decided.sort(key=lambda item: item[0], reverse=True)
    wins = [d for d in decided if d[1] == leader][:VERDICT_REASONS]
    losses = [d for d in decided if d[1] != leader][:VERDICT_TRADEOFFS]

    if wins:
        heading = "Why" if clear else "Where it leads,"
        print(f"\n{heading} {runs[leader].name!r}:")
        for _, _, metric, a, b in wins:
            print(f"  + {describe(metric, a, b)}")
    if losses:
        print(f"\nWhere {runs[1 - leader].name!r} is better (trade-offs):")
        for _, _, metric, a, b in losses:
            print(f"  - {describe(metric, a, b)}")
    print()


# ============================================================================
# VALIDATION
# ============================================================================

def _validate_tester_settings() -> None:
    if BASE_LOADER.strip() == COMPARISON_LOADER.strip():
        raise ValueError("BASE_LOADER and COMPARISON_LOADER must be different loaders.")
    for name, value in (("MU_BOX_ON_BOX", MU_BOX_ON_BOX),
                        ("MU_BOX_ON_PALLET", MU_BOX_ON_PALLET),
                        ("GRAVITY_MPS2", GRAVITY_MPS2),
                        ("MIN_BOX_MASS_KG", MIN_BOX_MASS_KG)):
        if not isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be positive and finite.")
    if not 0.0 < RESTRAINT_MIN_FACE_FRACTION <= 1.0:
        raise ValueError("RESTRAINT_MIN_FACE_FRACTION must be within (0, 1].")
    if not 0.0 <= LOW_SUPPORT_THRESHOLD <= 1.0:
        raise ValueError("LOW_SUPPORT_THRESHOLD must be within 0..1.")
    if CONTACT_TOL_MM < 0 or TIE_RELATIVE_TOL < 0 or CLEAR_WIN_FRACTION < 0:
        raise ValueError("Tolerances must be non-negative.")


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (BoxDataError, ValueError, RuntimeError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc