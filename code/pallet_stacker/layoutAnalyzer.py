"""Loader-independent verification of a finished pallet layout.

The module is read-only with respect to the domain model, exactly like
``plotlyVisualizer``: it consumes fully populated :class:`Pallet` objects and
never changes box positions or pallet state. It does not import any loader, so
every loader in ``main.LOADER_REGISTRY`` is checked by the same rules.

Checks (exact, brute force, no grid)
------------------------------------
Successor of the old ``verify_layout.py``. Every box is checked against the
pallet footprint, the height limit, every other box (3D overlap), its allowed
orientations, and resting contact (no floating boxes). Pallet load limits,
boxes placed twice and input boxes that were never packed are also reported.

Metrics
-------
Per pallet: box count, load, stack height, envelope utilisation (same
definition as ``Pallet.volume_utilization``), fill density (box volume over the
volume up to the ACTUAL stack height) and the lowest geometric support ratio.
Plus overall figures.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from math import isfinite
from pathlib import Path
from typing import Any, Sequence

import numpy as np
from numpy.typing import NDArray

from . import settings
from .box import Box
from .pallet import Pallet

FloatArray = NDArray[np.float64]


# ============================================================================
# ANALYSIS SETTINGS
# ============================================================================

# Top faces within this distance of a box bottom count as resting contact.
# Keep equal to the loaders' SUPPORT_FLUSH_TOL_MM, otherwise boxes accepted
# by the loader as supported are reported as floating here.
ANALYSIS_SUPPORT_FLUSH_TOL_MM: float = 0.5

# Raise LayoutAnalysisError when any check fails. Note that raising in
# main.py stage 2b happens before Plotly/Gazebo are written.
ANALYSIS_RAISE_ON_ERROR: bool = False

# JSON report written by main.py.
ANALYSIS_WRITE_JSON: bool = True
ANALYSIS_JSON_FILE: Path = settings.OUTPUT_DIRECTORY / "layout_analysis.json"

# Maximum number of individual findings printed by format_report().
ANALYSIS_PRINT_MAX_FINDINGS: int = 25


class LayoutAnalysisError(RuntimeError):
    """Raised when the analysed layout fails a check (optional)."""


# ============================================================================
# REPORT DATA TYPES
# ============================================================================

@dataclass(frozen=True, slots=True)
class Finding:
    """One failed check."""

    check: str               # short machine-readable check name
    message: str
    pallet_id: str | None = None
    box_id: str | None = None


@dataclass(frozen=True, slots=True)
class PalletMetrics:
    """Aggregated values of one pallet."""

    pallet_id: str
    box_count: int
    load_kg: float
    max_load_kg: float | None
    load_ratio: float | None
    stack_height_mm: float
    envelope_utilization: float   # box volume / (L * W * max height)
    fill_density: float           # box volume / (L * W * actual stack height)
    min_geometric_support: float | None


@dataclass(frozen=True, slots=True)
class LayoutReport:
    """Complete result of :func:`analyze_layout`."""

    pallets: tuple[PalletMetrics, ...]
    findings: tuple[Finding, ...]
    input_box_count: int | None
    placed_box_count: int
    overall_utilization: float
    overall_fill_density: float

    @property
    def pallet_count(self) -> int:
        return len(self.pallets)

    @property
    def error_count(self) -> int:
        return len(self.findings)

    @property
    def ok(self) -> bool:
        """True when every check passed."""
        return not self.findings

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data.update(
            {
                "pallet_count": self.pallet_count,
                "error_count": self.error_count,
                "ok": self.ok,
            }
        )
        return data


# ============================================================================
# MAIN ENTRY POINT
# ============================================================================

def analyze_layout(
    pallets: Sequence[Pallet],
    *,
    input_boxes: Sequence[Box] | None = None,
    raise_on_error: bool = ANALYSIS_RAISE_ON_ERROR,
) -> LayoutReport:
    """Verify and measure a finished layout produced by any loader.

    ``input_boxes`` is optional. When supplied, every input box must appear on
    exactly one pallet; missing boxes are reported as findings.
    """
    _validate_settings()

    findings: list[Finding] = []
    pallet_metrics: list[PalletMetrics] = []

    for pallet in pallets:
        pallet_findings, metrics = _analyze_pallet(pallet)
        findings.extend(pallet_findings)
        pallet_metrics.append(metrics)

    findings.extend(_cross_pallet_findings(pallets, input_boxes))

    placed = sum(pallet.box_count for pallet in pallets)
    box_volume = sum(
        box.volume_mm3 for pallet in pallets for box in pallet.boxes
    )
    usable_volume = sum(pallet.usable_volume_mm3 for pallet in pallets)
    filled_volume = sum(
        pallet.length_mm * pallet.width_mm * pallet.load_height_mm
        for pallet in pallets
    )

    report = LayoutReport(
        pallets=tuple(pallet_metrics),
        findings=tuple(findings),
        input_box_count=None if input_boxes is None else len(input_boxes),
        placed_box_count=placed,
        overall_utilization=box_volume / usable_volume if usable_volume > 0 else 0.0,
        overall_fill_density=box_volume / filled_volume if filled_volume > 0 else 0.0,
    )

    if raise_on_error and not report.ok:
        raise LayoutAnalysisError(
            f"Layout analysis found {report.error_count} error(s); "
            f"first: {report.findings[0].message}"
        )
    return report


# ============================================================================
# PER-PALLET ANALYSIS
# ============================================================================

def _analyze_pallet(pallet: Pallet) -> tuple[list[Finding], PalletMetrics]:
    findings: list[Finding] = []
    boxes = [box for box in pallet.boxes if box.is_placed]

    for box in pallet.boxes:
        if not box.is_placed:
            findings.append(
                Finding("unplaced_on_pallet",
                        f"Box {box.box_id!r} is registered on pallet "
                        f"{pallet.pallet_id!r} but has no placement.",
                        pallet.pallet_id, box.box_id)
            )

    if boxes:
        lo = np.array([_xyz(box.bounds()[0]) for box in boxes], dtype=np.float64)
        hi = np.array([_xyz(box.bounds()[1]) for box in boxes], dtype=np.float64)
    else:
        lo = np.empty((0, 3))
        hi = np.empty((0, 3))

    findings.extend(_geometric_findings(pallet, boxes, lo, hi))
    support = _geometric_support(lo, hi)

    stack = pallet.load_height_mm
    box_volume = sum(box.volume_mm3 for box in boxes)
    filled = pallet.length_mm * pallet.width_mm * stack
    load = pallet.current_load_kg

    metrics = PalletMetrics(
        pallet_id=pallet.pallet_id,
        box_count=pallet.box_count,
        load_kg=load,
        max_load_kg=pallet.max_load_kg,
        load_ratio=None if not pallet.max_load_kg else load / pallet.max_load_kg,
        stack_height_mm=stack,
        envelope_utilization=pallet.volume_utilization,
        fill_density=box_volume / filled if filled > 0 else 0.0,
        min_geometric_support=float(support.min()) if support.size else None,
    )
    return findings, metrics


# ============================================================================
# GEOMETRIC CHECKS (exact, independent of any loader state)
# ============================================================================

def _geometric_findings(
    pallet: Pallet,
    boxes: list[Box],
    lo: FloatArray,
    hi: FloatArray,
) -> list[Finding]:
    tol = settings.PLACEMENT_TOLERANCE_MM
    flush = ANALYSIS_SUPPORT_FLUSH_TOL_MM
    pid = pallet.pallet_id
    findings: list[Finding] = []

    for i, box in enumerate(boxes):
        if (lo[i, 0] < -tol or lo[i, 1] < -tol
                or hi[i, 0] > pallet.length_mm + tol
                or hi[i, 1] > pallet.width_mm + tol):
            findings.append(Finding("overhang",
                f"Box {box.box_id!r} extends beyond the footprint of {pid!r}.",
                pid, box.box_id))
        if lo[i, 2] < -tol:
            findings.append(Finding("below_deck",
                f"Box {box.box_id!r} extends below the loading surface.",
                pid, box.box_id))
        if hi[i, 2] > pallet.max_height_mm + tol:
            findings.append(Finding("too_tall",
                f"Box {box.box_id!r} top {hi[i, 2]:.1f} mm exceeds the "
                f"{pallet.max_height_mm:.1f} mm limit.",
                pid, box.box_id))
        if box.orientation not in box.allowed_orientations:
            findings.append(Finding("orientation_not_allowed",
                f"Box {box.box_id!r} uses orientation {box.orientation!r}, "
                "which is not in its allowed orientations.",
                pid, box.box_id))

    if pallet.max_load_kg is not None and pallet.current_load_kg > pallet.max_load_kg + 1e-6:
        findings.append(Finding("overload",
            f"Pallet {pid!r} carries {pallet.current_load_kg:.2f} kg, limit "
            f"{pallet.max_load_kg:.2f} kg.", pid))

    if not boxes:
        return findings

    # Pairwise 3D overlap (positive penetration on all three axes).
    overlap_len = np.minimum(hi[:, None, :], hi[None, :, :]) - np.maximum(
        lo[:, None, :], lo[None, :, :]
    )
    overlapping = np.all(overlap_len > tol, axis=2)
    for i, j in zip(*np.nonzero(np.triu(overlapping, k=1)), strict=True):
        findings.append(Finding("overlap",
            f"Boxes {boxes[i].box_id!r} and {boxes[j].box_id!r} overlap on {pid!r}.",
            pid, boxes[i].box_id))

    # Floating: above the deck, but no box top flush with the box bottom
    # under a positive XY overlap.
    xy_overlap = np.all(overlap_len[:, :, :2] > tol, axis=2)
    flush_top = np.abs(hi[None, :, 2] - lo[:, None, 2]) <= flush
    np.fill_diagonal(flush_top, False)
    has_support = (xy_overlap & flush_top).any(axis=1)
    for i in np.nonzero((lo[:, 2] > flush) & ~has_support)[0]:
        findings.append(Finding("floating",
            f"Box {boxes[i].box_id!r} at z={lo[i, 2]:.1f} mm has no resting contact.",
            pid, boxes[i].box_id))

    return findings


def _geometric_support(lo: FloatArray, hi: FloatArray) -> FloatArray:
    """Exact flush contact area / footprint area for every box (deck = 1.0)."""
    if lo.shape[0] == 0:
        return np.empty(0)
    flush = ANALYSIS_SUPPORT_FLUSH_TOL_MM
    ox = np.maximum(np.minimum(hi[:, None, 0], hi[None, :, 0])
                    - np.maximum(lo[:, None, 0], lo[None, :, 0]), 0.0)
    oy = np.maximum(np.minimum(hi[:, None, 1], hi[None, :, 1])
                    - np.maximum(lo[:, None, 1], lo[None, :, 1]), 0.0)
    flush_top = np.abs(hi[None, :, 2] - lo[:, None, 2]) <= flush
    np.fill_diagonal(flush_top, False)
    area = (ox * oy * flush_top).sum(axis=1)
    footprint = (hi[:, 0] - lo[:, 0]) * (hi[:, 1] - lo[:, 1])
    support = np.minimum(area / footprint, 1.0)
    return np.where(lo[:, 2] <= flush, 1.0, support)


def _cross_pallet_findings(
    pallets: Sequence[Pallet],
    input_boxes: Sequence[Box] | None,
) -> list[Finding]:
    findings: list[Finding] = []
    seen: dict[str, str] = {}
    for pallet in pallets:
        for box in pallet.boxes:
            if box.box_id in seen:
                findings.append(Finding("placed_twice",
                    f"Box {box.box_id!r} appears on {seen[box.box_id]!r} and "
                    f"{pallet.pallet_id!r}.", pallet.pallet_id, box.box_id))
            else:
                seen[box.box_id] = pallet.pallet_id

    if input_boxes is not None:
        for box in input_boxes:
            if box.box_id not in seen:
                findings.append(Finding("not_packed",
                    f"Input box {box.box_id!r} was not packed on any pallet.",
                    None, box.box_id))
    return findings


# ============================================================================
# REPORT OUTPUT
# ============================================================================

def format_report(report: LayoutReport) -> str:
    """Human-readable console report (HMI-style status page)."""
    lines = ["Layout analysis"]
    if report.input_box_count is not None:
        lines.append(
            f"  Boxes packed: {report.placed_box_count} / {report.input_box_count}"
        )
    lines.append(
        f"  Overall utilization: {report.overall_utilization:.1%} of envelope, "
        f"fill density {report.overall_fill_density:.1%}"
    )

    for m in report.pallets:
        support = (
            "n/a" if m.min_geometric_support is None
            else f"{m.min_geometric_support:.2f}"
        )
        lines.append(
            f"  {m.pallet_id}: fill density {m.fill_density:.1%}, "
            f"min support {support}"
        )

    lines.append(f"  Checks: {'all passed' if report.ok else f'{report.error_count} error(s)'}")
    for item in report.findings[:ANALYSIS_PRINT_MAX_FINDINGS]:
        lines.append(f"    [ERROR] {item.message}")
    hidden = report.error_count - ANALYSIS_PRINT_MAX_FINDINGS
    if hidden > 0:
        lines.append(f"    ... {hidden} more (see JSON report)")
    return "\n".join(lines)


def write_report_json(report: LayoutReport, output_path: str | Path = ANALYSIS_JSON_FILE) -> Path:
    """Write the full report as JSON and return the resolved path."""
    output = Path(output_path).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report.to_dict(), indent=2), encoding="utf-8")
    return output


# ============================================================================
# HELPERS & VALIDATION
# ============================================================================

def _xyz(point: Any) -> tuple[float, float, float]:
    return (point.x, point.y, point.z)


def _validate_settings() -> None:
    if not isfinite(ANALYSIS_SUPPORT_FLUSH_TOL_MM) or ANALYSIS_SUPPORT_FLUSH_TOL_MM < 0:
        raise LayoutAnalysisError(
            "ANALYSIS_SUPPORT_FLUSH_TOL_MM must be non-negative and finite."
        )
    if ANALYSIS_PRINT_MAX_FINDINGS < 0:
        raise LayoutAnalysisError("ANALYSIS_PRINT_MAX_FINDINGS must be non-negative.")