"""Pallet replay viewer.

Replays the result log written by pallet_stacker/resultLogger.py box by box, in
the real loading order, so placements that no human could make become visible
(for example a box lowered through an overhanging box).

Run (from the code/ directory or anywhere else):

    python palletReplayViewer.py

Requires pyvista, pyvistaqt and one Qt binding (PyQt5, PyQt6, PySide2 or
PySide6). The viewer is standalone: it imports nothing from pallet_stacker.
It only reads the chosen log; it writes no files and keeps no state between
runs. Closing the window ends everything.

Program structure (POU style)
-----------------------------
VIEWER SETTINGS     every tunable value (single point of change)
QT COMPATIBILITY    enum access that works with Qt5 and Qt6 bindings
LOG READING         read_replay_log(): JSON -> ReplayLog, with validation
SCENE LAYOUT        SceneLayout: bays, slots, holding square, lift height
PASS-THROUGH CHECK  find_pass_throughs(): boxes lowered through earlier boxes
TIMELINE            tracks that animate one transition k -> k +/- 1
VIEWER WINDOW       ReplayWindow: scene, controls, state machine, timer
ENTRY POINT         main()

Core model
----------
k = number of placed boxes (0 ... N). The box with step k is the next box and
waits on the holding square. For every k the scene is fully determined
(_render_rest). Animation only covers the transition between two neighbouring
rest states: k -> k + 1 (place) or k -> k - 1 (remove, the exact mirror).

World coordinates: metres. X along the bay rows, Y towards the storage bay,
Z up. The log itself is in millimetres, pallet-local, Z from the pallet's
loading surface.
"""

from __future__ import annotations

import hashlib
import html
import json
import sys
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from math import hypot, isfinite, sqrt
from pathlib import Path
from typing import Any

import numpy as np
import pyvista as pv
from qtpy import QtCore, QtGui, QtWidgets
from pyvistaqt import QtInteractor
from vtkmodules.vtkRenderingCore import vtkCellPicker


# ============================================================================
# VIEWER SETTINGS
# ============================================================================

# --- Log file -------------------------------------------------------------------
LOG_FORMAT: str = "pallet_result_log"
LOG_FORMAT_VERSION: int = 1
# Start folder of the file dialog: <project>/results next to the code folder.
DEFAULT_LOG_DIRECTORY: Path = Path(__file__).resolve().parent.parent / "results"

# --- Speeds at 1X -----------------------------------------------------------------
BOX_SPEED_MPS: float = 1.0          # box lift, travel and descent
SLIDE_SPEED_MPS: float = 1.0        # pallet slides and holding-square slides
SPEED_STEPS: tuple[float, ...] = (0.5, 1.0, 2.0, 3.0, 5.0, 50.0)
DEFAULT_SPEED: float = 1.0

# --- Batches --------------------------------------------------------------------
BATCH_SIZES: tuple[int, ...] = (5, 10, 25)

# --- Scene geometry [m] -----------------------------------------------------------
LIFT_MARGIN_M: float = 0.50         # lift height = pallet height + max load height + this
SLOT_GAP_M: float = 0.30            # between two pallets in a bay
BAY_MARGIN_M: float = 0.20          # pallet edge to bay edge
BAY_DISTANCE_M: float = 1.00        # loading bay edge to storage bay edge
SQUARE_MIN_SIDE_M: float = 1.00     # holding square side, at least
SQUARE_CLEARANCE_M: float = 0.20    # added to the largest box footprint side
SQUARE_DISTANCE_M: float = 1.00     # holding square edge to loading bay edge
LANE_LENGTH_M: float = 1.00         # boxes slide in/out this far beyond the square
BOX_VISUAL_GAP_M: float = 0.001     # boxes drawn this much smaller per side
FLOOR_MARGIN_M: float = 1.00        # floor plane extends this far around the scene

# Heights of flat scene elements (avoid z-fighting with the floor).
FLOOR_Z_M: float = -0.002
BAY_FILL_Z_M: float = 0.001
SQUARE_FILL_Z_M: float = 0.0015
OUTLINE_Z_M: float = 0.003

# --- Blinking and ticking ---------------------------------------------------------
BLINK_HOLD_S: float = 5.0           # pass-through blink after landing
CLICK_BLINK_S: float = 5.0          # box blink after a pass-through entry is clicked
ROW_BLINK_S: float = 5.0            # box-list row blink after the same click
BLINK_HZ: float = 2.0               # on/off cycles per second
BLINK_TINT_RGB: tuple[float, float, float] = (1.0, 0.08, 0.08)
BLINK_TINT_STRENGTH: float = 0.7    # 0 = SKU colour, 1 = pure tint
ROW_BLINK_BACKGROUND: str = "#d32f2f"
ROW_BLINK_FOREGROUND: str = "#ffffff"
TRANSLUCENT_OPACITY: float = 0.15   # unticked boxes while any box is ticked

# --- Box info popup (click on a box) --------------------------------------------
CLICK_TOLERANCE_PX: int = 4         # press-release distance that still counts as a click
INFO_POPUP_OFFSET_PX: int = 14      # popup appears this far right/below the cursor
INFO_POPUP_STYLE: str = (
    "QLabel { background: #fdfdf6; color: #1e1e1e; border: 1px solid #8a8a8a; "
    "padding: 6px 8px; }"
)
# --- Pass-through list popup ------------------------------------------------------
PASS_POPUP_WIDTH_PX: int = 560      # fixed; longer entries scroll horizontally
PASS_POPUP_ROWS: int = 10           # visible rows; more entries scroll vertically

# --- Colours (RGB 0..1) -------------------------------------------------------------
BACKGROUND_RGB: tuple[float, float, float] = (0.93, 0.94, 0.95)
FLOOR_RGB: tuple[float, float, float] = (0.80, 0.81, 0.83)
LOADING_BAY_RGB: tuple[float, float, float] = (0.96, 0.78, 0.05)    # yellow
STORAGE_BAY_RGB: tuple[float, float, float] = (0.55, 0.25, 0.78)    # purple
SQUARE_RGB: tuple[float, float, float] = (0.12, 0.42, 0.88)         # blue
SQUARE_DONE_RGB: tuple[float, float, float] = (0.16, 0.68, 0.29)    # green
PALLET_RGB: tuple[float, float, float] = (0.72, 0.55, 0.35)
PALLET_EDGE_RGB: tuple[float, float, float] = (0.40, 0.28, 0.15)
BOX_EDGE_RGB: tuple[float, float, float] = (0.15, 0.15, 0.15)
BAY_FILL_OPACITY: float = 0.18
OUTLINE_WIDTH_PX: float = 4.0

# --- Window, timer and camera -------------------------------------------------------
WINDOW_TITLE: str = "Pallet replay"
WINDOW_SIZE_PX: tuple[int, int] = (1500, 900)
PANEL_WIDTH_PX: int = 370
TICK_INTERVAL_MS: int = 16          # about 60 updates per second
MAX_TICK_DT_S: float = 0.10         # longer stalls (window drag) do not jump
CAMERA_STYLE: str = "terrain"       # "terrain" (turntable, z stays up) or "trackball"
CAMERA_DIRECTION: tuple[float, float, float] = (-0.45, -1.0, 0.65)
CAMERA_DISTANCE_FACTOR: float = 1.25
ENABLE_DEPTH_PEELING: bool = True   # correct overlap of translucent boxes

# Internal: float comparison margin for heights [m].
_EPS_M: float = 1e-9


# ============================================================================
# QT COMPATIBILITY
# ============================================================================

def _qt(owner: Any, scope: str, name: str) -> Any:
    """Enum member that works with flat (Qt5) and scoped (Qt6) enums."""
    scoped = getattr(owner, scope, None)
    if scoped is not None and hasattr(scoped, name):
        return getattr(scoped, name)
    return getattr(owner, name)


_CHECKABLE = _qt(QtCore.Qt, "ItemFlag", "ItemIsUserCheckable")
_CHECKED = _qt(QtCore.Qt, "CheckState", "Checked")
_UNCHECKED = _qt(QtCore.Qt, "CheckState", "Unchecked")
_BACKGROUND_ROLE = _qt(QtCore.Qt, "ItemDataRole", "BackgroundRole")
_FOREGROUND_ROLE = _qt(QtCore.Qt, "ItemDataRole", "ForegroundRole")
_SCROLL_CENTER = _qt(QtWidgets.QAbstractItemView, "ScrollHint", "PositionAtCenter")
_TRANSPARENT = _qt(QtCore.Qt, "GlobalColor", "transparent")
_ANTIALIASING = _qt(QtGui.QPainter, "RenderHint", "Antialiasing")
_NO_PEN = _qt(QtCore.Qt, "PenStyle", "NoPen")
_BUTTON_TEXT_ROLE = _qt(QtGui.QPalette, "ColorRole", "ButtonText")
_FIXED_FONT = _qt(QtGui.QFontDatabase, "SystemFont", "FixedFont")
_TOOLTIP_WINDOW = _qt(QtCore.Qt, "WindowType", "ToolTip")
_RICH_TEXT = _qt(QtCore.Qt, "TextFormat", "RichText")
_ELIDE_NONE = _qt(QtCore.Qt, "TextElideMode", "ElideNone")
_SCROLL_AS_NEEDED = _qt(QtCore.Qt, "ScrollBarPolicy", "ScrollBarAsNeeded")
_MOUSE_PRESS = _qt(QtCore.QEvent, "Type", "MouseButtonPress")
_MOUSE_RELEASE = _qt(QtCore.QEvent, "Type", "MouseButtonRelease")
_LEFT_BUTTON = _qt(QtCore.Qt, "MouseButton", "LeftButton")
_ADJUST_TO_MIN_CONTENTS = _qt(
    QtWidgets.QComboBox, "SizeAdjustPolicy", "AdjustToMinimumContentsLengthWithIcon"
)


# ============================================================================
# LOG READING
# ============================================================================

class LogFormatError(ValueError):
    """The chosen file is not a usable result log."""


@dataclass(frozen=True, slots=True)
class LogPalletSpec:
    length_mm: float
    width_mm: float
    base_height_mm: float
    max_height_mm: float


@dataclass(frozen=True, slots=True)
class LogPallet:
    index: int
    pallet_id: str
    opened_at_step: int
    closed_at_step: int | None      # None: open until the end of the replay


@dataclass(frozen=True, slots=True)
class LogBox:
    step: int
    box_id: str
    sku: str | None
    pallet_index: int
    min_corner_mm: tuple[float, float, float]   # pallet-local
    size_mm: tuple[float, float, float]         # oriented, as placed
    name: str | None = None                     # info popup only
    orientation: str | None = None              # info popup only
    mass_kg: float | None = None                # info popup only


@dataclass(frozen=True, slots=True)
class ReplayLog:
    path: Path
    loader: str
    input_file: str | None
    spec: LogPalletSpec
    tolerance_mm: float
    window: int                                 # K; null in the log -> pallet count
    pallets: tuple[LogPallet, ...]
    boxes: tuple[LogBox, ...]                   # in step order, boxes[i].step == i
    pallet_steps: tuple[tuple[int, ...], ...]   # steps per pallet, ascending


def read_replay_log(path: Path) -> ReplayLog:
    """Read and validate a result log. Raises LogFormatError."""
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise LogFormatError(f"Cannot read {path}: {exc}") from exc
    except ValueError as exc:
        raise LogFormatError(f"{path.name} is not valid JSON: {exc}") from exc

    if not isinstance(raw, dict) or raw.get("format") != LOG_FORMAT:
        raise LogFormatError(
            f"{path.name} is not a pallet result log. Open the file written by "
            f"resultLogger.py (format {LOG_FORMAT!r})."
        )
    if raw.get("format_version") != LOG_FORMAT_VERSION:
        raise LogFormatError(
            f"{path.name} has format version {raw.get('format_version')!r}; this "
            f"viewer reads version {LOG_FORMAT_VERSION}."
        )

    spec_raw = _mapping(raw.get("pallet_spec"), "pallet_spec")
    spec = LogPalletSpec(
        length_mm=_number(spec_raw.get("length_mm"), "pallet_spec.length_mm", positive=True),
        width_mm=_number(spec_raw.get("width_mm"), "pallet_spec.width_mm", positive=True),
        base_height_mm=_number(
            spec_raw.get("base_height_mm"), "pallet_spec.base_height_mm", positive=True
        ),
        max_height_mm=_number(
            spec_raw.get("max_height_mm"), "pallet_spec.max_height_mm", positive=True
        ),
    )
    tolerance = _number(raw.get("placement_tolerance_mm"), "placement_tolerance_mm")
    if tolerance < 0:
        raise LogFormatError("placement_tolerance_mm must not be negative.")

    source = raw.get("source")
    source = source if isinstance(source, dict) else {}
    loader = str(source.get("loader") or "unknown")
    input_file = source.get("input_file")
    input_file = str(input_file) if input_file is not None else None

    pallets = tuple(
        _read_pallet(item, i) for i, item in enumerate(_sequence(raw.get("pallets"), "pallets"))
    )
    boxes = tuple(
        _read_box(item, i) for i, item in enumerate(_sequence(raw.get("boxes"), "boxes"))
    )
    if not boxes:
        raise LogFormatError(f"{path.name} contains no boxes.")
    if not pallets:
        raise LogFormatError(f"{path.name} contains no pallets.")

    max_open = raw.get("max_open_pallets")
    window = len(pallets) if max_open is None else _integer(max_open, "max_open_pallets", 1)

    pallet_steps = _check_log(pallets, boxes, window)
    return ReplayLog(
        path=path,
        loader=loader,
        input_file=input_file,
        spec=spec,
        tolerance_mm=tolerance,
        window=window,
        pallets=pallets,
        boxes=boxes,
        pallet_steps=pallet_steps,
    )


def _read_pallet(item: Any, position: int) -> LogPallet:
    where = f"pallets[{position}]"
    data = _mapping(item, where)
    index = _integer(data.get("index"), f"{where}.index", 0)
    if index != position:
        raise LogFormatError(f"{where}.index is {index}; pallets must be listed in index order.")
    closed = data.get("closed_at_step")
    return LogPallet(
        index=index,
        pallet_id=str(data.get("pallet_id") or f"pallet {index}"),
        opened_at_step=_integer(data.get("opened_at_step"), f"{where}.opened_at_step", 0),
        closed_at_step=None if closed is None else _integer(closed, f"{where}.closed_at_step", 0),
    )


def _read_box(item: Any, position: int) -> LogBox:
    where = f"boxes[{position}]"
    data = _mapping(item, where)
    step = _integer(data.get("step"), f"{where}.step", 0)
    if step != position:
        raise LogFormatError(f"{where}.step is {step}; boxes must be listed in step order.")
    sku = data.get("sku")
    return LogBox(
        step=step,
        box_id=str(data.get("box_id") or f"box {step}"),
        sku=None if sku is None else str(sku),
        pallet_index=_integer(data.get("pallet_index"), f"{where}.pallet_index", 0),
        min_corner_mm=_triple(data.get("min_corner_mm"), f"{where}.min_corner_mm", positive=False),
        size_mm=_triple(data.get("size_mm"), f"{where}.size_mm", positive=True),
        name=None if data.get("name") is None else str(data.get("name")),
        orientation=None if data.get("orientation") is None else str(data.get("orientation")),
        mass_kg=_optional_number(data.get("mass_kg")),
    )


def _check_log(
    pallets: Sequence[LogPallet],
    boxes: Sequence[LogBox],
    window: int,
) -> tuple[tuple[int, ...], ...]:
    """Consistency checks the replay relies on. Returns the steps per pallet."""
    pallet_count = len(pallets)
    steps_per_pallet: list[list[int]] = [[] for _ in range(pallet_count)]
    for box in boxes:
        if not 0 <= box.pallet_index < pallet_count:
            raise LogFormatError(
                f"Box {box.box_id!r} refers to pallet index {box.pallet_index}, "
                f"but the log has {pallet_count} pallets."
            )
        steps_per_pallet[box.pallet_index].append(box.step)

    for pallet, steps in zip(pallets, steps_per_pallet, strict=True):
        if not steps:
            raise LogFormatError(f"Pallet {pallet.pallet_id!r} has no boxes.")
        if pallet.opened_at_step != steps[0]:
            raise LogFormatError(
                f"Pallet {pallet.pallet_id!r}: opened_at_step {pallet.opened_at_step} "
                f"is not the step of its first box ({steps[0]})."
            )
        if pallet.closed_at_step is not None and pallet.closed_at_step <= steps[-1]:
            raise LogFormatError(
                f"Pallet {pallet.pallet_id!r} receives a box at step {steps[-1]} "
                f"but is closed at step {pallet.closed_at_step}."
            )

    for earlier, later in zip(pallets, pallets[1:]):
        if earlier.opened_at_step >= later.opened_at_step:
            raise LogFormatError("Pallets must be listed in opening order.")
        if earlier.closed_at_step is None and later.closed_at_step is not None:
            raise LogFormatError("Pallets must close in opening order (FIFO).")
        if (
            earlier.closed_at_step is not None
            and later.closed_at_step is not None
            and earlier.closed_at_step > later.closed_at_step
        ):
            raise LogFormatError("Pallets must close in opening order (FIFO).")

    # Pallet n and n + K share a loading-bay slot: n must be gone first.
    for n in range(pallet_count - window):
        closed = pallets[n].closed_at_step
        if closed is None or closed > pallets[n + window].opened_at_step:
            raise LogFormatError(
                f"More than max_open_pallets = {window} pallets would be open at "
                f"once (pallet {pallets[n].pallet_id!r} is still open when "
                f"{pallets[n + window].pallet_id!r} opens)."
            )

    return tuple(tuple(steps) for steps in steps_per_pallet)


def _mapping(value: Any, where: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise LogFormatError(f"{where} must be an object.")
    return value


def _sequence(value: Any, where: str) -> list[Any]:
    if not isinstance(value, list):
        raise LogFormatError(f"{where} must be an array.")
    return value


def _number(value: Any, where: str, *, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not isfinite(value):
        raise LogFormatError(f"{where} must be a finite number.")
    if positive and value <= 0:
        raise LogFormatError(f"{where} must be positive.")
    return float(value)


def _optional_number(value: Any) -> float | None:
    """Informative fields: a missing or invalid value is shown as unknown."""
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not isfinite(value):
        return None
    return float(value)


def _integer(value: Any, where: str, minimum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise LogFormatError(f"{where} must be an integer >= {minimum}.")
    return value


def _triple(value: Any, where: str, *, positive: bool) -> tuple[float, float, float]:
    if not isinstance(value, list) or len(value) != 3:
        raise LogFormatError(f"{where} must be a list of three numbers.")
    x, y, z = (_number(v, where, positive=positive) for v in value)
    return (x, y, z)


# ============================================================================
# SCENE LAYOUT
# ============================================================================

@dataclass(frozen=True, slots=True)
class SceneLayout:
    """Fixed geometry of the scene for one log [m]."""

    window: int                 # K = loading-bay slots
    pallet_count: int
    pallet_length_m: float
    pallet_width_m: float
    pallet_height_m: float
    slot_pitch_m: float
    bay_width_m: float
    loading_length_m: float
    storage_y0_m: float         # near edge of the storage bay
    gap_y_m: float              # centre line between the two bays
    square_cx_m: float
    square_cy_m: float
    square_side_m: float
    lane_distance_m: float      # square centre to lane end
    lift_bottom_m: float        # box bottom height during travel

    def bay_length(self, slots: int) -> float:
        return (
            slots * self.pallet_length_m
            + max(0, slots - 1) * SLOT_GAP_M
            + 2.0 * BAY_MARGIN_M
        )

    def loading_center(self, pallet_index: int) -> tuple[float, float]:
        slot = pallet_index % self.window
        x = BAY_MARGIN_M + slot * self.slot_pitch_m + self.pallet_length_m / 2.0
        return (x, self.bay_width_m / 2.0)

    def storage_center(self, pallet_index: int) -> tuple[float, float]:
        # Storage slot = pallet index: FIFO closes pallets in index order.
        x = BAY_MARGIN_M + pallet_index * self.slot_pitch_m + self.pallet_length_m / 2.0
        return (x, self.storage_y0_m + self.bay_width_m / 2.0)

    def storage_path(self, pallet_index: int) -> list[tuple[float, float]]:
        """Loading slot -> gap -> along the gap -> storage slot (no collisions)."""
        lx, ly = self.loading_center(pallet_index)
        sx, sy = self.storage_center(pallet_index)
        return [(lx, ly), (lx, self.gap_y_m), (sx, self.gap_y_m), (sx, sy)]


def build_layout(log: ReplayLog) -> SceneLayout:
    spec = log.spec
    length = spec.length_mm / 1000.0
    width = spec.width_mm / 1000.0
    height = spec.base_height_mm / 1000.0
    bay_width = width + 2.0 * BAY_MARGIN_M
    largest_side = max(max(box.size_mm[0], box.size_mm[1]) for box in log.boxes) / 1000.0
    square_side = max(SQUARE_MIN_SIDE_M, largest_side + SQUARE_CLEARANCE_M)
    loading_length = (
        log.window * length + (log.window - 1) * SLOT_GAP_M + 2.0 * BAY_MARGIN_M
    )
    return SceneLayout(
        window=log.window,
        pallet_count=len(log.pallets),
        pallet_length_m=length,
        pallet_width_m=width,
        pallet_height_m=height,
        slot_pitch_m=length + SLOT_GAP_M,
        bay_width_m=bay_width,
        loading_length_m=loading_length,
        storage_y0_m=bay_width + BAY_DISTANCE_M,
        gap_y_m=bay_width + BAY_DISTANCE_M / 2.0,
        square_cx_m=-SQUARE_DISTANCE_M - square_side / 2.0,
        square_cy_m=bay_width / 2.0,
        square_side_m=square_side,
        lane_distance_m=square_side / 2.0 + LANE_LENGTH_M,
        lift_bottom_m=height + spec.max_height_mm / 1000.0 + LIFT_MARGIN_M,
    )


# ============================================================================
# PASS-THROUGH CHECK
# ============================================================================

@dataclass(frozen=True, slots=True)
class PassThrough:
    """A box whose vertical descent passes through earlier boxes."""

    step: int
    offenders: tuple[int, ...]      # steps of the boxes it passes through
    first_contact_mm: float         # highest offender top, pallet-local


def find_pass_throughs(log: ReplayLog) -> dict[int, PassThrough]:
    """Swept column of every box (its footprint from lift height down to its
    resting z) against every EARLIER box on the SAME pallet.

    An earlier box offends when the footprints overlap by more than the
    tolerance and its top is above the box's resting bottom (face contact does
    not count). The horizontal move needs no check: the lift height clears
    every possible stack.
    """
    tol = log.tolerance_mm
    result: dict[int, PassThrough] = {}

    for steps in log.pallet_steps:
        for position, step in enumerate(steps):
            box = log.boxes[step]
            bx1, by1, bz1 = box.min_corner_mm
            bx2, by2 = bx1 + box.size_mm[0], by1 + box.size_mm[1]

            offenders: list[int] = []
            first_contact = -float("inf")
            for earlier_step in steps[:position]:
                other = log.boxes[earlier_step]
                ox1, oy1, oz1 = other.min_corner_mm
                ox2 = ox1 + other.size_mm[0]
                oy2 = oy1 + other.size_mm[1]
                oz2 = oz1 + other.size_mm[2]
                if (
                    min(bx2, ox2) - max(bx1, ox1) > tol
                    and min(by2, oy2) - max(by1, oy1) > tol
                    and oz2 > bz1 + tol
                ):
                    offenders.append(earlier_step)
                    first_contact = max(first_contact, oz2)

            if offenders:
                result[step] = PassThrough(step, tuple(offenders), first_contact)

    return result


# ============================================================================
# TIMELINE
# ============================================================================

@dataclass(slots=True)
class _Track:
    """One animated (or instant) change inside a transition.

    Times are seconds at 1X; the window advances the timeline by wall time
    multiplied by the selected speed.
    """

    start: float
    duration: float
    apply: Callable[[float], None] | None       # called with fraction 0..1
    on_finish: Callable[[], None] | None        # called once at fraction 1
    done: bool = False


class _Timeline:
    """Tracks of one transition, applied in start order.

    A track is applied only once it has started, and only until it is done,
    so sequential tracks on the same actor never fight each other.
    """

    __slots__ = ("tracks", "time")

    def __init__(self) -> None:
        self.tracks: list[_Track] = []
        self.time = 0.0

    def add(
        self,
        start: float,
        duration: float,
        apply: Callable[[float], None] | None = None,
        on_finish: Callable[[], None] | None = None,
    ) -> float:
        """Add a track; returns its end time."""
        duration = max(0.0, duration)
        self.tracks.append(_Track(start, duration, apply, on_finish))
        return start + duration

    def finalize(self) -> None:
        # Stable sort: tracks with the same start keep their insertion order.
        self.tracks.sort(key=lambda track: track.start)

    def advance(self, dt: float) -> bool:
        """Advance by dt (seconds at 1X). Returns True when every track is done."""
        self.time += dt
        now = self.time
        for track in self.tracks:
            if track.start > now:
                break
            if track.done:
                continue
            if track.duration <= 0.0:
                fraction = 1.0
            else:
                fraction = min(1.0, (now - track.start) / track.duration)
            if track.apply is not None:
                track.apply(fraction)
            if fraction >= 1.0:
                track.done = True
                if track.on_finish is not None:
                    track.on_finish()
        return all(track.done for track in self.tracks)


def _polyline_length(points: Sequence[tuple[float, float]]) -> float:
    return sum(
        hypot(x2 - x1, y2 - y1) for (x1, y1), (x2, y2) in zip(points, points[1:])
    )


def _polyline_point(
    points: Sequence[tuple[float, float]], fraction: float
) -> tuple[float, float]:
    """Point at the given fraction of the polyline's length."""
    remaining = fraction * _polyline_length(points)
    for (x1, y1), (x2, y2) in zip(points, points[1:]):
        segment = hypot(x2 - x1, y2 - y1)
        if segment <= 0.0:
            continue
        if remaining <= segment:
            ratio = remaining / segment
            return (x1 + (x2 - x1) * ratio, y1 + (y2 - y1) * ratio)
        remaining -= segment
    return points[-1]


# ============================================================================
# SMALL HELPERS
# ============================================================================

def _sku_rgb(text: str) -> tuple[float, float, float]:
    """Same rule as gzSimulator._stable_color, so colours match Gazebo."""
    digest = hashlib.sha256(text.encode("utf-8")).digest()
    r, g, b = ((70 + value % 150) / 255.0 for value in digest[:3])
    return (r, g, b)


def _tint(rgb: tuple[float, float, float]) -> tuple[float, float, float]:
    s = BLINK_TINT_STRENGTH
    r, g, b = (c * (1.0 - s) + t * s for c, t in zip(rgb, BLINK_TINT_RGB))
    return (r, g, b)


def _actor_key(actor: Any) -> str:
    """Identity of a VTK object, independent of which Python wrapper holds it."""
    return actor.GetAddressAsString("vtkObject")


def _rect_points(x0: float, y0: float, x1: float, y1: float, z: float) -> np.ndarray:
    return np.array(
        [[x0, y0, z], [x1, y0, z], [x1, y1, z], [x0, y1, z]], dtype=float
    )


def _quad_mesh(points: np.ndarray) -> pv.PolyData:
    return pv.PolyData(points, faces=np.array([4, 0, 1, 2, 3]))


def _outline_mesh(points: np.ndarray) -> pv.PolyData:
    return pv.PolyData(points, lines=np.array([5, 0, 1, 2, 3, 0]))


def _make_icon(kind: str, color: QtGui.QColor, count: int = 1) -> QtGui.QIcon:
    """Small drawn icons: 'triangles' (count of them), 'turtle', 'pause', 'stop'."""
    height = 18
    width = 10 * count + 6 if kind == "triangles" else (28 if kind == "turtle" else 18)
    pixmap = QtGui.QPixmap(width, height)
    pixmap.fill(QtGui.QColor(_TRANSPARENT))
    painter = QtGui.QPainter(pixmap)
    painter.setRenderHint(_ANTIALIASING)
    painter.setPen(_NO_PEN)
    painter.setBrush(color)
    if kind == "triangles":
        for i in range(count):
            x = 3.0 + 10.0 * i
            painter.drawPolygon(
                QtGui.QPolygonF(
                    [QtCore.QPointF(x, 3.0), QtCore.QPointF(x + 9.0, 9.0), QtCore.QPointF(x, 15.0)]
                )
            )
    elif kind == "pause":
        painter.drawRect(QtCore.QRectF(4.0, 3.0, 3.5, 12.0))
        painter.drawRect(QtCore.QRectF(10.5, 3.0, 3.5, 12.0))
    elif kind == "stop":
        painter.drawRect(QtCore.QRectF(4.0, 4.0, 10.0, 10.0))
    elif kind == "turtle":
        painter.drawEllipse(QtCore.QRectF(4.0, 4.0, 15.0, 9.0))     # shell
        painter.drawEllipse(QtCore.QRectF(19.0, 6.0, 6.0, 5.0))     # head
        painter.drawRect(QtCore.QRectF(6.0, 11.0, 3.0, 4.0))        # legs
        painter.drawRect(QtCore.QRectF(14.0, 11.0, 3.0, 4.0))
        painter.drawPolygon(
            QtGui.QPolygonF(
                [QtCore.QPointF(4.5, 8.0), QtCore.QPointF(1.0, 10.5), QtCore.QPointF(5.0, 10.0)]
            )
        )                                                           # tail
    painter.end()
    return QtGui.QIcon(pixmap)


def _event_xy(event: Any) -> tuple[float, float]:
    """Mouse position in widget coordinates (Qt6: position(), Qt5: pos())."""
    point = event.position() if hasattr(event, "position") else event.pos()
    return (point.x(), point.y())


class _ClickFilter(QtCore.QObject):
    """Qt event filter on the 3D widget that reports left clicks.

    VTK's camera style grabs VTK mouse focus on a press, so VTK release
    observers never fire. Qt sees every event first. The filter never consumes
    an event, so the camera keeps working unchanged.
    """

    def __init__(
        self,
        on_press: Callable[[], None],
        on_click: Callable[[float, float], None],
        parent: QtCore.QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self._on_press = on_press
        self._on_click = on_click
        self._press_xy: tuple[float, float] | None = None

    def eventFilter(self, watched: Any, event: Any) -> bool:  # noqa: N802 - Qt API
        kind = event.type()
        if kind == _MOUSE_PRESS and event.button() == _LEFT_BUTTON:
            self._press_xy = _event_xy(event)
            self._on_press()
        elif kind == _MOUSE_RELEASE and event.button() == _LEFT_BUTTON:
            press, self._press_xy = self._press_xy, None
            if press is not None:
                x, y = _event_xy(event)
                if (
                    abs(x - press[0]) <= CLICK_TOLERANCE_PX
                    and abs(y - press[1]) <= CLICK_TOLERANCE_PX
                ):
                    self._on_click(x, y)
        return False


# ============================================================================
# VIEWER WINDOW
# ============================================================================

_IDLE = "idle"
_MOVING = "moving"
_PAUSED = "paused"


class ReplayWindow(QtWidgets.QMainWindow):
    """3D scene, control panel, state machine and animation timer."""

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle(WINDOW_TITLE)
        self.resize(*WINDOW_SIZE_PX)

        # ---- replay data (set when a log is opened) ------------------------
        self.log: ReplayLog | None = None
        self.scene: SceneLayout | None = None
        self.pass_throughs: dict[int, PassThrough] = {}

        # ---- replay state -------------------------------------------------
        self.k = 0                          # placed boxes (rest state)
        self.state = _IDLE
        self.speed = DEFAULT_SPEED
        self.direction = 0                  # +1 placing, -1 removing
        self.batch_remaining = 0            # transitions after the current one
        self.timeline: _Timeline | None = None

        # ---- scene objects (rebuilt per log) --------------------------------
        self.box_actors: list[Any] = []
        self.box_colors: list[tuple[float, float, float]] = []
        self.box_offsets: list[tuple[float, float, float]] = []  # to pallet centre; z absolute
        self.box_half_heights: list[float] = []
        self.box_fade: list[float] = []
        self.pallet_actors: list[Any] = []
        self.pallet_centers: list[tuple[float, float]] = []
        self.storage_fill: pv.PolyData | None = None
        self.storage_outline: pv.PolyData | None = None
        self.storage_fill_actor: Any = None
        self.storage_outline_actor: Any = None
        self.square_fill_actor: Any = None
        self.square_outline_actor: Any = None

        # ---- inspection -----------------------------------------------------
        self.ticked: set[int] = set()
        self.blink_until: dict[int, float] = {}     # box step -> wall time
        self.pass_through_active: set[int] = set()  # boxes inside offenders now
        self.row_blink_until: dict[int, float] = {}
        self._tinted: set[int] = set()              # boxes showing the tint now
        self._rows_tinted: set[int] = set()
        # Pass-through entries in the combobox: filled as the boxes are placed.
        self.listed_pass_throughs: set[int] = set()

        # ---- box info popup (click picking) ---------------------------------
        self._actor_steps: dict[str, int] = {}      # VTK actor address -> box step
        self._picker = vtkCellPicker()
        self._picker.SetTolerance(0.0005)

        self._last_tick = time.monotonic()
        self._needs_render = False

        self._build_ui()
        self._init_plotter()
        self._update_controls()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._on_tick)
        self.timer.start(TICK_INTERVAL_MS)

    # ------------------------------------------------------------------
    # UI CONSTRUCTION
    # ------------------------------------------------------------------
    def _build_ui(self) -> None:
        central = QtWidgets.QWidget(self)
        self.setCentralWidget(central)
        row = QtWidgets.QHBoxLayout(central)
        row.setContentsMargins(6, 6, 6, 6)
        row.setSpacing(8)

        self.plotter = QtInteractor(central)
        row.addWidget(self.plotter.interactor, 1)

        panel = QtWidgets.QWidget(central)
        panel.setFixedWidth(PANEL_WIDTH_PX)
        column = QtWidgets.QVBoxLayout(panel)
        column.setContentsMargins(0, 0, 0, 0)
        column.setSpacing(8)
        row.addWidget(panel)

        icon_color = self.palette().color(_BUTTON_TEXT_ROLE)

        # ---- file and status --------------------------------------------
        self.open_button = QtWidgets.QPushButton("Open result log…")
        self.open_button.clicked.connect(self._on_open_clicked)
        column.addWidget(self.open_button)

        self.status_label = QtWidgets.QLabel("No result log open.")
        self.status_label.setWordWrap(True)
        column.addWidget(self.status_label)

        # ---- place --------------------------------------------------------
        place_group = QtWidgets.QGroupBox("Place")
        place_layout = QtWidgets.QVBoxLayout(place_group)
        place_next = QtWidgets.QPushButton("Place next box")
        place_next.clicked.connect(lambda _=False: self._start_batch(+1, 1))
        place_layout.addWidget(place_next)
        place_row = QtWidgets.QHBoxLayout()
        self.place_buttons: list[QtWidgets.QPushButton] = [place_next]
        for size in (*BATCH_SIZES, None):
            button = QtWidgets.QPushButton("All" if size is None else f"+{size}")
            button.clicked.connect(lambda _=False, n=size: self._start_batch(+1, n))
            place_row.addWidget(button)
            self.place_buttons.append(button)
        place_layout.addLayout(place_row)
        column.addWidget(place_group)

        # ---- remove -------------------------------------------------------
        remove_group = QtWidgets.QGroupBox("Remove")
        remove_layout = QtWidgets.QVBoxLayout(remove_group)
        remove_last = QtWidgets.QPushButton("Remove last box")
        remove_last.clicked.connect(lambda _=False: self._start_batch(-1, 1))
        remove_layout.addWidget(remove_last)
        remove_row = QtWidgets.QHBoxLayout()
        self.remove_buttons: list[QtWidgets.QPushButton] = [remove_last]
        for size in BATCH_SIZES:
            button = QtWidgets.QPushButton(f"\u2212{size}")
            button.clicked.connect(lambda _=False, n=size: self._start_batch(-1, n))
            remove_row.addWidget(button)
            self.remove_buttons.append(button)
        # Restart replaces "remove all": the same end state, without animation.
        restart = QtWidgets.QPushButton("Restart")
        restart.clicked.connect(self._on_restart_clicked)
        remove_row.addWidget(restart)
        self.remove_buttons.append(restart)
        remove_layout.addLayout(remove_row)
        column.addWidget(remove_group)

        # ---- playback -----------------------------------------------------
        playback_group = QtWidgets.QGroupBox("Playback")
        playback_layout = QtWidgets.QVBoxLayout(playback_group)
        control_row = QtWidgets.QHBoxLayout()
        self._pause_icon = _make_icon("pause", icon_color)
        self._resume_icon = _make_icon("triangles", icon_color, 1)
        self.pause_button = QtWidgets.QPushButton(self._pause_icon, "Pause")
        self.pause_button.setCheckable(True)
        self.pause_button.clicked.connect(self._on_pause_clicked)
        self.stop_button = QtWidgets.QPushButton(_make_icon("stop", icon_color), "Stop")
        self.stop_button.clicked.connect(self._on_stop_clicked)
        control_row.addWidget(self.pause_button)
        control_row.addWidget(self.stop_button)
        playback_layout.addLayout(control_row)

        speed_grid = QtWidgets.QGridLayout()
        self.speed_group = QtWidgets.QButtonGroup(self)
        self.speed_group.setExclusive(True)
        self.speed_buttons: dict[float, QtWidgets.QPushButton] = {}
        for i, speed in enumerate(SPEED_STEPS):
            if speed < 1.0:
                icon = _make_icon("turtle", icon_color)
            else:
                icon = _make_icon("triangles", icon_color, min(3, int(speed)))
            button = QtWidgets.QPushButton(icon, f"{speed:g}X")
            button.setCheckable(True)
            button.setIconSize(icon.availableSizes()[0] if icon.availableSizes() else QtCore.QSize(28, 18))
            button.clicked.connect(lambda _=False, s=speed: self._set_speed(s))
            self.speed_group.addButton(button)
            self.speed_buttons[speed] = button
            speed_grid.addWidget(button, i // 3, i % 3)
        playback_layout.addLayout(speed_grid)
        column.addWidget(playback_group)

        # ---- pass-through ---------------------------------------------------
        pass_group = QtWidgets.QGroupBox("Pass-through")
        pass_layout = QtWidgets.QVBoxLayout(pass_group)
        self.pass_combo = QtWidgets.QComboBox()
        self.pass_combo.setSizeAdjustPolicy(_ADJUST_TO_MIN_CONTENTS)
        self.pass_combo.setMinimumContentsLength(20)
        # Some desktop styles open a combobox as a list sized to all its items,
        # up to the full screen height. "combobox-popup: 0" makes Qt honour
        # setMaxVisibleItems; the fixed width plus no eliding gives scrollbars.
        self.pass_combo.setStyleSheet("QComboBox { combobox-popup: 0; }")
        self.pass_combo.setMaxVisibleItems(PASS_POPUP_ROWS)
        pass_view = self.pass_combo.view()
        assert pass_view is not None    # always set for a QComboBox
        pass_view.setMinimumWidth(PASS_POPUP_WIDTH_PX)
        pass_view.setMaximumWidth(PASS_POPUP_WIDTH_PX)
        pass_view.setTextElideMode(_ELIDE_NONE)
        pass_view.setHorizontalScrollBarPolicy(_SCROLL_AS_NEEDED)
        pass_view.setVerticalScrollBarPolicy(_SCROLL_AS_NEEDED)
        self.pass_combo.activated.connect(self._on_pass_through_activated)
        pass_layout.addWidget(self.pass_combo)
        column.addWidget(pass_group)

        # ---- box list ------------------------------------------------------
        boxes_group = QtWidgets.QGroupBox("Boxes (tick to isolate)")
        boxes_layout = QtWidgets.QVBoxLayout(boxes_group)
        self.box_list = QtWidgets.QListWidget()
        self.box_list.setUniformItemSizes(True)
        self.box_list.setFont(QtGui.QFontDatabase.systemFont(_FIXED_FONT))
        self.box_list.itemChanged.connect(self._on_box_item_changed)
        boxes_layout.addWidget(self.box_list)
        column.addWidget(boxes_group, 1)

        # ---- box info popup ---------------------------------------------------
        # A tool-tip window: it does not grab the mouse, so the camera keeps
        # working while it is shown. Hidden on the next press in the 3D view.
        self.info_popup = QtWidgets.QLabel(self, _TOOLTIP_WINDOW)
        self.info_popup.setTextFormat(_RICH_TEXT)
        self.info_popup.setStyleSheet(INFO_POPUP_STYLE)
        self.info_popup.hide()

    def _init_plotter(self) -> None:
        plotter: Any = self.plotter     # pyvista's decorated methods confuse Pylance
        plotter.set_background(BACKGROUND_RGB)
        try:
            plotter.add_axes()
        except Exception:  # noqa: BLE001 - cosmetic only
            pass
        if ENABLE_DEPTH_PEELING:
            try:
                plotter.enable_depth_peeling()
            except Exception:  # noqa: BLE001 - translucency then just sorts less well
                pass
        if CAMERA_STYLE == "terrain":
            try:
                plotter.enable_terrain_style(mouse_wheel_zooms=True, shift_pans=True)
            except TypeError:
                plotter.enable_terrain_style()

        # Click picking through a Qt event filter (see _ClickFilter).
        self._click_filter = _ClickFilter(self._on_view_press, self._on_view_click, self)
        plotter.interactor.installEventFilter(self._click_filter)

    # ------------------------------------------------------------------
    # OPENING A LOG
    # ------------------------------------------------------------------
    def _on_open_clicked(self) -> None:
        if self.state != _IDLE:
            return
        start = DEFAULT_LOG_DIRECTORY if DEFAULT_LOG_DIRECTORY.is_dir() else Path.cwd()
        file_name, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Open result log",
            str(start),
            "Result logs (*.json);;All files (*)",
        )
        if not file_name:
            return
        try:
            log = read_replay_log(Path(file_name))
        except LogFormatError as exc:
            QtWidgets.QMessageBox.critical(self, "Cannot open result log", str(exc))
            return
        self._load_log(log)

    def _load_log(self, log: ReplayLog) -> None:
        self.log = log
        self.scene = build_layout(log)
        self.pass_throughs = find_pass_throughs(log)

        self.k = 0
        self.state = _IDLE
        self.direction = 0
        self.batch_remaining = 0
        self.timeline = None
        self.ticked.clear()
        self.blink_until.clear()
        self.pass_through_active.clear()
        self.row_blink_until.clear()
        self._tinted.clear()
        self._rows_tinted.clear()
        self.listed_pass_throughs.clear()
        self.info_popup.hide()

        self._build_scene()
        self._reset_pass_through_combo()
        self._fill_box_list()
        self._render_rest(0)
        self._frame_camera()
        self.setWindowTitle(f"{WINDOW_TITLE} \u2013 {log.loader} loader \u2013 {log.path.name}")
        self._update_controls()

    def _build_scene(self) -> None:
        log, lay = self.log, self.scene
        assert log is not None and lay is not None
        plotter = self.plotter
        clear = getattr(plotter, "clear_actors", None)
        if callable(clear):
            clear()
        else:
            plotter.clear()

        # ---- floor ----------------------------------------------------------
        x0, x1, y0, y1, _ = self._scene_bounds()
        floor = pv.Plane(
            center=((x0 + x1) / 2.0, (y0 + y1) / 2.0, FLOOR_Z_M),
            direction=(0.0, 0.0, 1.0),
            i_size=x1 - x0,
            j_size=y1 - y0,
            i_resolution=1,
            j_resolution=1,
        )
        plotter.add_mesh(floor, color=FLOOR_RGB, reset_camera=False)

        # ---- loading bay (fixed size: K slots) -----------------------------
        plotter.add_mesh(
            _quad_mesh(_rect_points(0.0, 0.0, lay.loading_length_m, lay.bay_width_m, BAY_FILL_Z_M)),
            color=LOADING_BAY_RGB,
            opacity=BAY_FILL_OPACITY,
            reset_camera=False,
        )
        plotter.add_mesh(
            _outline_mesh(_rect_points(0.0, 0.0, lay.loading_length_m, lay.bay_width_m, OUTLINE_Z_M)),
            color=LOADING_BAY_RGB,
            line_width=OUTLINE_WIDTH_PX,
            reset_camera=False,
        )

        # ---- storage bay (grows one slot at a time) ------------------------
        y_far = lay.storage_y0_m + lay.bay_width_m
        one_slot = lay.bay_length(1)
        self.storage_fill = _quad_mesh(
            _rect_points(0.0, lay.storage_y0_m, one_slot, y_far, BAY_FILL_Z_M)
        )
        self.storage_outline = _outline_mesh(
            _rect_points(0.0, lay.storage_y0_m, one_slot, y_far, OUTLINE_Z_M)
        )
        self.storage_fill_actor = plotter.add_mesh(
            self.storage_fill, color=STORAGE_BAY_RGB, opacity=BAY_FILL_OPACITY, reset_camera=False
        )
        self.storage_outline_actor = plotter.add_mesh(
            self.storage_outline, color=STORAGE_BAY_RGB, line_width=OUTLINE_WIDTH_PX, reset_camera=False
        )

        # ---- holding square ------------------------------------------------
        half = lay.square_side_m / 2.0
        sx0, sx1 = lay.square_cx_m - half, lay.square_cx_m + half
        sy0, sy1 = lay.square_cy_m - half, lay.square_cy_m + half
        self.square_fill_actor = plotter.add_mesh(
            _quad_mesh(_rect_points(sx0, sy0, sx1, sy1, SQUARE_FILL_Z_M)),
            color=SQUARE_RGB,
            opacity=BAY_FILL_OPACITY,
            reset_camera=False,
        )
        self.square_outline_actor = plotter.add_mesh(
            _outline_mesh(_rect_points(sx0, sy0, sx1, sy1, OUTLINE_Z_M)),
            color=SQUARE_RGB,
            line_width=OUTLINE_WIDTH_PX,
            reset_camera=False,
        )

        # ---- pallets ---------------------------------------------------------
        length, width, height = lay.pallet_length_m, lay.pallet_width_m, lay.pallet_height_m
        self.pallet_actors = []
        self.pallet_centers = []
        for pallet in log.pallets:
            mesh = pv.Box(
                bounds=(-length / 2, length / 2, -width / 2, width / 2, -height / 2, height / 2)
            )
            actor = plotter.add_mesh(
                mesh, color=PALLET_RGB, show_edges=True, edge_color=PALLET_EDGE_RGB,
                reset_camera=False,
            )
            actor.SetVisibility(False)
            self.pallet_actors.append(actor)
            self.pallet_centers.append(lay.loading_center(pallet.index))

        # ---- boxes -----------------------------------------------------------
        self.box_actors = []
        self._actor_steps = {}
        self.box_colors = []
        self.box_offsets = []
        self.box_half_heights = []
        self.box_fade = []
        for box in log.boxes:
            sx, sy, sz = (value / 1000.0 for value in box.size_mm)
            mx, my, mz = (value / 1000.0 for value in box.min_corner_mm)
            vx = max(sx / 2.0 - BOX_VISUAL_GAP_M, 0.001)
            vy = max(sy / 2.0 - BOX_VISUAL_GAP_M, 0.001)
            vz = max(sz / 2.0 - BOX_VISUAL_GAP_M, 0.001)
            color = _sku_rgb(box.sku or box.box_id)
            actor = plotter.add_mesh(
                pv.Box(bounds=(-vx, vx, -vy, vy, -vz, vz)),
                color=color,
                show_edges=True,
                edge_color=BOX_EDGE_RGB,
                reset_camera=False,
            )
            actor.SetVisibility(False)
            self._actor_steps[_actor_key(actor)] = box.step
            self.box_actors.append(actor)
            self.box_colors.append(color)
            self.box_offsets.append(
                (
                    mx + sx / 2.0 - length / 2.0,
                    my + sy / 2.0 - width / 2.0,
                    height + mz + sz / 2.0,
                )
            )
            self.box_half_heights.append(sz / 2.0)
            self.box_fade.append(1.0)

    def _scene_bounds(self) -> tuple[float, float, float, float, float]:
        """x0, x1, y0, y1, z_top of everything that can appear [m]."""
        log, lay = self.log, self.scene
        assert log is not None and lay is not None
        x0 = lay.square_cx_m - lay.lane_distance_m - lay.square_side_m / 2.0 - FLOOR_MARGIN_M
        x1 = max(lay.loading_length_m, lay.bay_length(lay.pallet_count)) + FLOOR_MARGIN_M
        y0 = min(0.0, lay.square_cy_m - lay.square_side_m / 2.0) - FLOOR_MARGIN_M
        y1 = lay.storage_y0_m + lay.bay_width_m + FLOOR_MARGIN_M
        tallest = max(box.size_mm[2] for box in log.boxes) / 1000.0
        return (x0, x1, y0, y1, lay.lift_bottom_m + tallest)

    def _frame_camera(self) -> None:
        """Start view covering the loading bay, the holding lane and the storage
        bay at its final size, so the growing storage never leaves the view."""
        x0, x1, y0, y1, z_top = self._scene_bounds()
        focal = ((x0 + x1) / 2.0, (y0 + y1) / 2.0, z_top * 0.25)
        dx, dy, dz = CAMERA_DIRECTION
        norm = sqrt(dx * dx + dy * dy + dz * dz)
        span = max(x1 - x0, y1 - y0, z_top)
        distance = span * CAMERA_DISTANCE_FACTOR
        eye = (
            focal[0] + dx / norm * distance,
            focal[1] + dy / norm * distance,
            focal[2] + dz / norm * distance,
        )
        self.plotter.camera_position = [eye, focal, (0.0, 0.0, 1.0)]
        self.plotter.reset_camera_clipping_range()
        self._needs_render = True

    def _reset_pass_through_combo(self) -> None:
        """Header only. Entries are added as the boxes are placed, at the moment
        a box starts passing through (when it starts blinking)."""
        combo = self.pass_combo
        combo.blockSignals(True)
        combo.clear()
        combo.addItem(self._pass_through_header_text())
        model = combo.model()
        if isinstance(model, QtGui.QStandardItemModel):
            header = model.item(0)
            if header is not None:
                header.setEnabled(False)
        combo.setCurrentIndex(0)
        combo.blockSignals(False)

    def _pass_through_header_text(self) -> str:
        count = len(self.listed_pass_throughs)
        if count == 0:
            return "No pass-through found"
        return f"{count} box{'es' if count != 1 else ''} pass through earlier boxes"

    def _pass_through_entry_text(self, step: int) -> str:
        log = self.log
        assert log is not None
        entry = self.pass_throughs[step]
        box = log.boxes[step]
        pallet_id = log.pallets[box.pallet_index].pallet_id
        through = ", ".join(log.boxes[other].box_id for other in entry.offenders)
        return f"Step {step}: {box.box_id} on {pallet_id}, through {through}"

    def _set_pass_through_listed(self, step: int, listed: bool) -> None:
        """Add or remove one combobox entry; entries stay in step order."""
        if step not in self.pass_throughs or listed == (step in self.listed_pass_throughs):
            return
        combo = self.pass_combo
        combo.blockSignals(True)
        if listed:
            self.listed_pass_throughs.add(step)
            # Index 0 is the header; entries follow in step order.
            position = 1 + sum(1 for other in self.listed_pass_throughs if other < step)
            combo.insertItem(position, self._pass_through_entry_text(step), step)
        else:
            self.listed_pass_throughs.discard(step)
            index = combo.findData(step)
            if index >= 0:
                if combo.currentIndex() == index:
                    combo.setCurrentIndex(0)
                combo.removeItem(index)
        combo.setItemText(0, self._pass_through_header_text())
        combo.blockSignals(False)

    def _sync_listed_pass_throughs(self, target: set[int]) -> None:
        """Make the listed entries equal to target (used by the rest state)."""
        for step in sorted(self.listed_pass_throughs - target):
            self._set_pass_through_listed(step, False)
        for step in sorted(target - self.listed_pass_throughs):
            self._set_pass_through_listed(step, True)

    def _fill_box_list(self) -> None:
        log = self.log
        assert log is not None
        width = len(str(len(log.boxes) - 1))
        box_list = self.box_list
        box_list.blockSignals(True)
        box_list.clear()
        for box in log.boxes:
            pallet_id = log.pallets[box.pallet_index].pallet_id
            sku = box.sku if box.sku is not None else "-"
            item = QtWidgets.QListWidgetItem(
                f"{box.step:>{width}}  {box.box_id}  {sku}  {pallet_id}"
            )
            item.setFlags(item.flags() | _CHECKABLE)
            item.setCheckState(_UNCHECKED)
            box_list.addItem(item)
        box_list.blockSignals(False)

    # ------------------------------------------------------------------
    # REST STATE: the scene as a function of k
    # ------------------------------------------------------------------
    def _render_rest(self, k: int) -> None:
        log, lay = self.log, self.scene
        assert log is not None and lay is not None
        box_count = len(log.boxes)
        complete = k >= box_count

        # ---- pallets: hidden, in the loading bay or in storage --------------
        stored_slots = 0
        for pallet in log.pallets:
            n = pallet.index
            actor = self.pallet_actors[n]
            if pallet.opened_at_step >= k:
                actor.SetVisibility(False)
                self._set_pallet_pose(n, *lay.loading_center(n))
                continue
            closed = pallet.closed_at_step is not None and pallet.closed_at_step < k
            if complete or closed:
                self._set_pallet_pose(n, *lay.storage_center(n))
                stored_slots = max(stored_slots, n + 1)
            else:
                self._set_pallet_pose(n, *lay.loading_center(n))
            actor.SetVisibility(True)

        # ---- boxes: on a pallet, on the holding square or not yet visible ----
        for box in log.boxes:
            step = box.step
            actor = self.box_actors[step]
            self.box_fade[step] = 1.0
            if step < k:
                self._set_box_pose(step, *self._box_on_pallet(step))
                actor.SetVisibility(True)
            elif step == k:
                self._set_box_pose(step, *self._box_on_square(step))
                actor.SetVisibility(True)
            else:
                actor.SetVisibility(False)

        self.k = k
        self.pass_through_active.clear()
        self._sync_listed_pass_throughs({s for s in self.pass_throughs if s < k})
        self._set_storage_slots(stored_slots)
        self._set_square_complete(complete)
        self._update_blinks(time.monotonic())
        self._refresh_all_boxes()
        self._needs_render = True

    # ------------------------------------------------------------------
    # SCENE PRIMITIVES
    # ------------------------------------------------------------------
    def _set_box_pose(self, step: int, x: float, y: float, z: float) -> None:
        self.box_actors[step].SetPosition(x, y, z)

    def _box_on_pallet(self, step: int) -> tuple[float, float, float]:
        """Resting centre of a box on its pallet, wherever the pallet is now."""
        assert self.log is not None
        cx, cy = self.pallet_centers[self.log.boxes[step].pallet_index]
        dx, dy, cz = self.box_offsets[step]
        return (cx + dx, cy + dy, cz)

    def _box_on_loading_slot(self, step: int) -> tuple[float, float, float]:
        """Resting centre of a box on its pallet standing in its loading slot."""
        assert self.log is not None and self.scene is not None
        cx, cy = self.scene.loading_center(self.log.boxes[step].pallet_index)
        dx, dy, cz = self.box_offsets[step]
        return (cx + dx, cy + dy, cz)

    def _box_on_square(self, step: int) -> tuple[float, float, float]:
        assert self.scene is not None
        return (self.scene.square_cx_m, self.scene.square_cy_m, self.box_half_heights[step])

    def _set_pallet_pose(self, n: int, x: float, y: float) -> None:
        assert self.scene is not None
        self.pallet_centers[n] = (x, y)
        self.pallet_actors[n].SetPosition(x, y, self.scene.pallet_height_m / 2.0)

    def _move_pallet_group(self, n: int, x: float, y: float, members: Sequence[int]) -> None:
        """Move a pallet together with the boxes resting on it."""
        self._set_pallet_pose(n, x, y)
        for step in members:
            self._set_box_pose(step, *self._box_on_pallet(step))

    def _show_pallet_in_loading_slot(self, n: int) -> None:
        assert self.scene is not None
        self._set_pallet_pose(n, *self.scene.loading_center(n))
        self.pallet_actors[n].SetVisibility(True)

    def _hide_pallet(self, n: int) -> None:
        self.pallet_actors[n].SetVisibility(False)

    def _set_storage_slots(self, slots: int) -> None:
        lay = self.scene
        assert lay is not None and self.storage_fill is not None and self.storage_outline is not None
        if slots <= 0:
            self.storage_fill_actor.SetVisibility(False)
            self.storage_outline_actor.SetVisibility(False)
        else:
            length = lay.bay_length(slots)
            y0 = lay.storage_y0_m
            y1 = y0 + lay.bay_width_m
            self.storage_fill.points = _rect_points(0.0, y0, length, y1, BAY_FILL_Z_M)
            self.storage_outline.points = _rect_points(0.0, y0, length, y1, OUTLINE_Z_M)
            self.storage_fill_actor.SetVisibility(True)
            self.storage_outline_actor.SetVisibility(True)
        self._needs_render = True

    def _set_square_complete(self, complete: bool) -> None:
        rgb = SQUARE_DONE_RGB if complete else SQUARE_RGB
        fill = self.square_fill_actor.GetProperty()
        fill.SetColor(*rgb)
        fill.SetOpacity(1.0 if complete else BAY_FILL_OPACITY)
        self.square_outline_actor.GetProperty().SetColor(*rgb)
        self._needs_render = True

    def _refresh_box(self, step: int) -> None:
        """Colour (blink tint) and opacity (ticks, fade) of one box."""
        prop = self.box_actors[step].GetProperty()
        base = 1.0 if (not self.ticked or step in self.ticked) else TRANSLUCENT_OPACITY
        prop.SetOpacity(base * self.box_fade[step])
        self.box_actors[step].SetPickable(base >= 1.0)
        rgb = self.box_colors[step]
        prop.SetColor(*(_tint(rgb) if step in self._tinted else rgb))

    def _refresh_all_boxes(self) -> None:
        for step in range(len(self.box_actors)):
            self._refresh_box(step)
        self._needs_render = True

    def _set_pass_through(self, step: int, active: bool) -> None:
        if active:
            self.pass_through_active.add(step)
        else:
            self.pass_through_active.discard(step)

    def _start_hold_blink(self, step: int) -> None:
        until = time.monotonic() + BLINK_HOLD_S
        self.blink_until[step] = max(self.blink_until.get(step, 0.0), until)

    # ------------------------------------------------------------------
    # TRANSITION BUILDING BLOCKS
    # ------------------------------------------------------------------
    def _members(self, n: int, below_step: int) -> tuple[int, ...]:
        """Boxes resting on pallet n at a moment when steps < below_step are placed."""
        assert self.log is not None
        return tuple(step for step in self.log.pallet_steps[n] if step < below_step)

    def _add_pallet_slide(
        self,
        timeline: _Timeline,
        start: float,
        n: int,
        path: list[tuple[float, float]],
        members: tuple[int, ...],
    ) -> float:
        duration = _polyline_length(path) / SLIDE_SPEED_MPS

        def apply(fraction: float) -> None:
            x, y = _polyline_point(path, fraction)
            self._move_pallet_group(n, x, y, members)

        return timeline.add(start, duration, apply)

    def _add_close_to_storage(
        self, timeline: _Timeline, start: float, n: int, members: tuple[int, ...]
    ) -> float:
        """Storage grows by one slot, then the pallet slides into it."""
        timeline.add(start, 0.0, on_finish=lambda: self._set_storage_slots(n + 1))
        assert self.scene is not None
        return self._add_pallet_slide(timeline, start, n, self.scene.storage_path(n), members)

    def _add_return_to_loading(
        self, timeline: _Timeline, start: float, n: int, members: tuple[int, ...]
    ) -> float:
        """The pallet slides back to its loading slot, then storage shrinks."""
        assert self.scene is not None
        path = list(reversed(self.scene.storage_path(n)))
        end = self._add_pallet_slide(timeline, start, n, path, members)
        timeline.add(end, 0.0, on_finish=lambda: self._set_storage_slots(n))
        return end

    def _add_box_vertical(
        self,
        timeline: _Timeline,
        start: float,
        step: int,
        x: float,
        y: float,
        z_from: float,
        z_to: float,
        *,
        monitor: bool = False,
        listing: str | None = None,
        on_finish: Callable[[], None] | None = None,
    ) -> float:
        """Vertical move of a box centre. With monitor=True the box blinks
        while its bottom is below the highest offender top (pass-through).

        listing="enter": the box appears in the pass-through list when it
        starts passing through (descent). listing="exit": it leaves the list
        when its lift clears the offenders (removal, the mirror)."""
        duration = abs(z_to - z_from) / BOX_SPEED_MPS
        contact: float | None = None
        entry = self.pass_throughs.get(step) if monitor else None
        if entry is not None and self.scene is not None:
            contact = self.scene.pallet_height_m + entry.first_contact_mm / 1000.0
        half = self.box_half_heights[step]

        def apply(fraction: float) -> None:
            z = z_from + (z_to - z_from) * fraction
            self._set_box_pose(step, x, y, z)
            if contact is not None:
                inside = (z - half) < contact - _EPS_M and fraction < 1.0
                self._set_pass_through(step, inside)
                if listing == "enter" and inside:
                    self._set_pass_through_listed(step, True)
                elif listing == "exit" and not inside:
                    self._set_pass_through_listed(step, False)

        return timeline.add(start, duration, apply, on_finish)

    def _add_box_travel(
        self,
        timeline: _Timeline,
        start: float,
        step: int,
        xy_from: tuple[float, float],
        xy_to: tuple[float, float],
        z: float,
    ) -> float:
        duration = hypot(xy_to[0] - xy_from[0], xy_to[1] - xy_from[1]) / BOX_SPEED_MPS

        def apply(fraction: float) -> None:
            x = xy_from[0] + (xy_to[0] - xy_from[0]) * fraction
            y = xy_from[1] + (xy_to[1] - xy_from[1]) * fraction
            self._set_box_pose(step, x, y, z)

        return timeline.add(start, duration, apply)

    def _add_lane_slide(
        self, timeline: _Timeline, start: float, step: int, *, incoming: bool
    ) -> float:
        """Next box slides onto the square fading in, or the waiting box
        slides away fading out, along the lane on the square's -X side."""
        lay = self.scene
        assert lay is not None
        square_x = lay.square_cx_m
        lane_x = lay.square_cx_m - lay.lane_distance_m
        y = lay.square_cy_m
        z = self.box_half_heights[step]
        duration = lay.lane_distance_m / SLIDE_SPEED_MPS

        def apply(fraction: float) -> None:
            if incoming:
                x = lane_x + (square_x - lane_x) * fraction
                fade = fraction
            else:
                x = square_x + (lane_x - square_x) * fraction
                fade = 1.0 - fraction
            self.box_actors[step].SetVisibility(True)
            self._set_box_pose(step, x, y, z)
            self.box_fade[step] = fade
            self._refresh_box(step)

        finish = None if incoming else (lambda: self.box_actors[step].SetVisibility(False))
        return timeline.add(start, duration, apply, finish)

    # ------------------------------------------------------------------
    # TRANSITIONS
    # ------------------------------------------------------------------
    def _build_forward(self, k: int) -> _Timeline:
        """Place box k: k -> k + 1."""
        log, lay = self.log, self.scene
        assert log is not None and lay is not None
        step = k
        box_count = len(log.boxes)
        timeline = _Timeline()
        t = 0.0

        # 1. This box closes a pallet: storage grows, the pallet slides in.
        for pallet in log.pallets:
            if pallet.closed_at_step == step:
                n = pallet.index
                t = self._add_close_to_storage(timeline, t, n, self._members(n, step))

        # 2. This box opens a pallet: the empty pallet appears in its slot.
        for pallet in log.pallets:
            if pallet.opened_at_step == step:
                timeline.add(
                    t, 0.0, on_finish=lambda n=pallet.index: self._show_pallet_in_loading_slot(n)
                )

        # 3. Lift from the holding square.
        square_x, square_y = lay.square_cx_m, lay.square_cy_m
        half = self.box_half_heights[step]
        travel_z = lay.lift_bottom_m + half
        lift_end = self._add_box_vertical(timeline, t, step, square_x, square_y, half, travel_z)

        # 4. Meanwhile the next box slides onto the empty square, fading in.
        if step + 1 < box_count:
            self._add_lane_slide(timeline, lift_end, step + 1, incoming=True)

        # 5. Travel at lift height and descend to the resting position.
        tx, ty, tz = self._box_on_loading_slot(step)
        t = self._add_box_travel(timeline, lift_end, step, (square_x, square_y), (tx, ty), travel_z)
        hold = (lambda: self._start_hold_blink(step)) if step in self.pass_throughs else None
        t = self._add_box_vertical(
            timeline, t, step, tx, ty, travel_z, tz, monitor=True, listing="enter", on_finish=hold
        )

        # 6. Last box: the pallets still open go to storage, square turns green.
        if step == box_count - 1:
            for pallet in log.pallets:
                if pallet.closed_at_step is None:
                    n = pallet.index
                    t = self._add_close_to_storage(timeline, t, n, self._members(n, step + 1))
            timeline.add(t, 0.0, on_finish=lambda: self._set_square_complete(True))

        timeline.finalize()
        return timeline

    def _build_reverse(self, k: int) -> _Timeline:
        """Remove box k - 1: k -> k - 1 (the mirror of _build_forward)."""
        log, lay = self.log, self.scene
        assert log is not None and lay is not None
        step = k - 1
        box_count = len(log.boxes)
        timeline = _Timeline()
        t = 0.0

        # 1. From the completed state: square back to blue, the end-of-replay
        #    pallets return to their loading slots in reverse order.
        if k == box_count:
            timeline.add(0.0, 0.0, on_finish=lambda: self._set_square_complete(False))
            for pallet in reversed(log.pallets):
                if pallet.closed_at_step is None:
                    n = pallet.index
                    t = self._add_return_to_loading(timeline, t, n, self._members(n, k))

        # 2. The waiting box slides away fading out, during the lift and travel.
        slide_end = t
        if k < box_count:
            slide_end = self._add_lane_slide(timeline, t, k, incoming=False)

        # 3. Lift from the pallet, travel, descend onto the holding square.
        square_x, square_y = lay.square_cx_m, lay.square_cy_m
        half = self.box_half_heights[step]
        travel_z = lay.lift_bottom_m + half
        sx, sy, sz = self._box_on_loading_slot(step)
        t = self._add_box_vertical(
            timeline, t, step, sx, sy, sz, travel_z, monitor=True, listing="exit"
        )
        t = self._add_box_travel(timeline, t, step, (sx, sy), (square_x, square_y), travel_z)
        t = max(t, slide_end)
        t = self._add_box_vertical(timeline, t, step, square_x, square_y, travel_z, half)

        # 4. This box had opened its pallet: the empty pallet disappears.
        for pallet in log.pallets:
            if pallet.opened_at_step == step:
                timeline.add(t, 0.0, on_finish=lambda n=pallet.index: self._hide_pallet(n))

        # 5. This box had closed a pallet: it returns, then storage shrinks.
        for pallet in reversed(log.pallets):
            if pallet.closed_at_step == step:
                n = pallet.index
                t = self._add_return_to_loading(timeline, t, n, self._members(n, step))

        timeline.finalize()
        return timeline

    def _build_transition(self) -> _Timeline:
        if self.direction > 0:
            return self._build_forward(self.k)
        return self._build_reverse(self.k)

    # ------------------------------------------------------------------
    # STATE MACHINE
    # ------------------------------------------------------------------
    def _start_batch(self, direction: int, count: int | None) -> None:
        """Place (+1) or remove (-1) count boxes one at a time; None = all."""
        if self.log is None or self.state != _IDLE:
            return
        available = len(self.log.boxes) - self.k if direction > 0 else self.k
        n = available if count is None else min(count, available)
        if n <= 0:
            return
        self.direction = direction
        self.batch_remaining = n - 1
        self.timeline = self._build_transition()
        self.state = _MOVING
        self._update_controls()

    def _finish_transition(self) -> None:
        self.timeline = None
        self._render_rest(self.k + self.direction)
        if self.batch_remaining > 0:
            self.batch_remaining -= 1
            self.timeline = self._build_transition()
        else:
            self.state = _IDLE
            self.direction = 0
        self._update_controls()

    def _on_pause_clicked(self) -> None:
        if self.state == _MOVING:
            self.state = _PAUSED
        elif self.state == _PAUSED:
            self.state = _MOVING
        self._update_controls()

    def _on_stop_clicked(self) -> None:
        """The current transition always finishes; the rest of the batch is
        discarded. While paused, Stop also resumes at the highlighted speed."""
        if self.state not in (_MOVING, _PAUSED):
            return
        self.batch_remaining = 0
        if self.state == _PAUSED:
            self.state = _MOVING
        self._update_controls()

    def _set_speed(self, speed: float) -> None:
        self.speed = speed
        self._update_controls()

    def _on_restart_clicked(self) -> None:
        """Empty every pallet at once (no animation) and start over at k = 0."""
        if self.log is None or self.state != _IDLE or self.k == 0:
            return
        self._render_rest(0)
        self._update_controls()

    # ------------------------------------------------------------------
    # INSPECTION TOOLS (view-only: never change k)
    # ------------------------------------------------------------------
    def _on_pass_through_activated(self, index: int) -> None:
        data = self.pass_combo.itemData(index)
        if data is None or self.log is None:
            return
        step = int(data)
        now = time.monotonic()
        if self.box_actors[step].GetVisibility():
            self.blink_until[step] = max(self.blink_until.get(step, 0.0), now + CLICK_BLINK_S)
        self.row_blink_until[step] = now + ROW_BLINK_S
        item = self.box_list.item(step)
        if item is not None:
            self.box_list.scrollToItem(item, _SCROLL_CENTER)
        self._needs_render = True

    def _on_box_item_changed(self, item: QtWidgets.QListWidgetItem) -> None:
        step = self.box_list.row(item)
        if step < 0:
            return
        checked = item.checkState() == _CHECKED
        if checked == (step in self.ticked):
            return
        if checked:
            self.ticked.add(step)
        else:
            self.ticked.discard(step)
        self._refresh_all_boxes()

    def _on_view_press(self) -> None:
        """Any left press in the 3D view closes the info popup."""
        self.info_popup.hide()

    def _on_view_click(self, qx: float, qy: float) -> None:
        """Left click (not a drag) at widget position (qx, qy): pick a box."""
        if self.log is None:
            return
        # Qt logical pixels, origin top-left -> VTK display pixels, origin
        # bottom-left (the same conversion QVTKRenderWindowInteractor does).
        scale = self.plotter.interactor.devicePixelRatioF()
        height = self.plotter.iren.interactor.GetSize()[1]
        x = int(round(qx * scale))
        y = height - int(round(qy * scale)) - 1
        self._picker.Pick(x, y, 0, self.plotter.renderer)
        actor = self._picker.GetActor()
        if actor is None:
            return
        step = self._actor_steps.get(_actor_key(actor))
        if step is not None:
            self._show_box_info(step)

    def _show_box_info(self, step: int) -> None:
        popup = self.info_popup
        popup.setText(self._box_info_html(step))
        popup.adjustSize()
        cursor = QtGui.QCursor.pos()
        x = cursor.x() + INFO_POPUP_OFFSET_PX
        y = cursor.y() + INFO_POPUP_OFFSET_PX
        screen = QtGui.QGuiApplication.screenAt(cursor)
        if screen is not None:                  # keep the popup on screen
            area = screen.availableGeometry()
            x = max(area.left(), min(x, area.right() - popup.width()))
            y = max(area.top(), min(y, area.bottom() - popup.height()))
        popup.move(x, y)
        popup.show()
        popup.raise_()

    def _box_info_html(self, step: int) -> str:
        """Same fields as the Plotly hover text, plus step and status."""
        log = self.log
        assert log is not None
        box = log.boxes[step]
        pallet_id = log.pallets[box.pallet_index].pallet_id
        x, y, z = box.min_corner_mm
        sx, sy, sz = box.size_mm
        esc = html.escape

        lines = [
            f"<b>{esc(box.box_id)}</b>",
            f"Step: {step} ({self._box_status(step)})",
            f"SKU: {esc(box.sku or 'N/A')}",
        ]
        if box.name:
            lines.append(f"Name: {esc(box.name)}")
        lines += [
            f"Pallet: {esc(pallet_id)}",
            f"Orientation: {esc(box.orientation or 'unknown')}",
            f"Position: x={x:.1f}, y={y:.1f}, z={z:.1f} mm",
            f"Placed size: {sx:.1f} \u00d7 {sy:.1f} \u00d7 {sz:.1f} mm",
            "Weight: unknown" if box.mass_kg is None else f"Weight: {box.mass_kg:.2f} kg",
        ]
        # Only once revealed: same rule as the pass-through list.
        if step in self.listed_pass_throughs:
            through = ", ".join(
                esc(log.boxes[other].box_id) for other in self.pass_throughs[step].offenders
            )
            lines.append(f"<span style='color:#c62828'>Passes through: {through}</span>")
        return "<br>".join(lines)

    def _box_status(self, step: int) -> str:
        if self.timeline is not None:
            moving = self.k if self.direction > 0 else self.k - 1
            if step == moving:
                return "moving"
        if step < self.k:
            return "placed"
        if step == self.k:
            return "next box"
        return "not placed yet"

    def _update_blinks(self, now: float) -> None:
        """Box blink tint: pass-through in progress, hold after landing and
        clicked entries. Phase from wall time, so blinking continues paused."""
        for step in [s for s, until in self.blink_until.items() if until <= now]:
            del self.blink_until[step]
        active = set(self.blink_until) | self.pass_through_active
        phase_on = int(now * BLINK_HZ * 2.0) % 2 == 0
        tinted = active if phase_on else set()
        if tinted != self._tinted:
            changed = tinted ^ self._tinted
            self._tinted = tinted
            for step in changed:
                if step < len(self.box_actors):
                    self._refresh_box(step)
            self._needs_render = True

    def _update_row_blinks(self, now: float) -> None:
        for step in [s for s, until in self.row_blink_until.items() if until <= now]:
            del self.row_blink_until[step]
        phase_on = int(now * BLINK_HZ * 2.0) % 2 == 0
        shown = set(self.row_blink_until) if phase_on else set()
        if shown == self._rows_tinted:
            return
        self.box_list.blockSignals(True)
        for step in shown ^ self._rows_tinted:
            item = self.box_list.item(step)
            if item is None:
                continue
            if step in shown:
                item.setBackground(QtGui.QBrush(QtGui.QColor(ROW_BLINK_BACKGROUND)))
                item.setForeground(QtGui.QBrush(QtGui.QColor(ROW_BLINK_FOREGROUND)))
            else:
                item.setData(_BACKGROUND_ROLE, None)
                item.setData(_FOREGROUND_ROLE, None)
        self.box_list.blockSignals(False)
        self._rows_tinted = shown

    # ------------------------------------------------------------------
    # CONTROLS AND STATUS
    # ------------------------------------------------------------------
    def _update_controls(self) -> None:
        loaded = self.log is not None
        box_count = len(self.log.boxes) if self.log is not None else 0
        idle = loaded and self.state == _IDLE
        running = self.state in (_MOVING, _PAUSED)
        paused = self.state == _PAUSED

        for button in self.place_buttons:
            button.setEnabled(idle and self.k < box_count)
        for button in self.remove_buttons:
            button.setEnabled(idle and self.k > 0)
        self.open_button.setEnabled(not running)

        self.pause_button.setEnabled(running)
        self.pause_button.setChecked(paused)
        self.pause_button.setText("Resume" if paused else "Pause")
        self.pause_button.setIcon(self._resume_icon if paused else self._pause_icon)
        self.stop_button.setEnabled(running)

        for speed, button in self.speed_buttons.items():
            button.setChecked(speed == self.speed)

        self.pass_combo.setEnabled(loaded)
        self.box_list.setEnabled(loaded)
        self._update_status()

    def _update_status(self) -> None:
        log = self.log
        if log is None:
            self.status_label.setText("No result log open.")
            return
        box_count = len(log.boxes)
        if self.state == _PAUSED:
            activity = "Paused"
        elif self.state == _MOVING:
            activity = "Placing" if self.direction > 0 else "Removing"
            if self.batch_remaining:
                activity += f", {self.batch_remaining} more in this batch"
        elif self.k >= box_count:
            activity = "All boxes placed"
        else:
            activity = "Ready"
        self.status_label.setText(
            f"{log.path.name}\n"
            f"Loader: {log.loader}\n"
            f"Box {self.k} / {box_count}\n"
            f"{activity}"
        )

    # ------------------------------------------------------------------
    # TIMER
    # ------------------------------------------------------------------
    def _on_tick(self) -> None:
        now = time.monotonic()
        dt = min(now - self._last_tick, MAX_TICK_DT_S)
        self._last_tick = now

        if self.state == _MOVING and self.timeline is not None:
            finished = self.timeline.advance(dt * self.speed)
            self._needs_render = True
            if finished:
                self._finish_transition()

        if self.log is not None:
            self._update_blinks(now)
            self._update_row_blinks(now)

        if self._needs_render:
            self._needs_render = False
            self.plotter.render()

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:  # noqa: N802 - Qt API
        self.timer.stop()
        self.info_popup.hide()
        try:
            self.plotter.close()
        except Exception:  # noqa: BLE001 - shutting down anyway
            pass
        super().closeEvent(event)


# ============================================================================
# ENTRY POINT
# ============================================================================

def main() -> int:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    window = ReplayWindow()
    window.show()
    run = getattr(app, "exec", None) or getattr(app, "exec_")
    return int(run() or 0)


if __name__ == "__main__":
    sys.exit(main())