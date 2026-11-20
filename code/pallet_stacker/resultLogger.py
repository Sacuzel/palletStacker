"""Result logger: writes the replay log of a finished loader run.

Called once from main.py, stage 3 (OUTPUT), like a PLC action:

    logResults(
        pallets,
        max_open_pallets=K,
        loader_name="optimizer",
        source_file=box_data.source_path,
        output_path=settings.RESULT_LOG_FILE,
    )

The log is the only input of palletReplayViewer.py. It holds the FINAL result
of the run in the real loading order, plus the step at which every pallet was
opened and closed. For the optimizer this is the replayed winner, because the
optimizer replaces the loader.

Pallet events (FIFO open-pallet window)
---------------------------------------
emsLoader and the optimizer's emsDecoder keep the K newest pallets open
(``states[-K:]``) and open a new pallet only for the box that is being placed.
With pallets numbered in opening order (= order of the returned list):

    opened_at_step(n) = smallest sequence index on pallet n
    closed_at_step(n) = opened_at_step(n + K)      (None: open until the end)

closed_at_step is the sequence index of the box whose placement pushes the
pallet out of the open window. The derivation is checked before anything is
written (FIFO self-check): a loader that does not close its pallets
first-in-first-out with this K makes the logger fail loudly instead of writing
a misleading file.

Sequence index
--------------
Sequence index of a box = ``box.metadata["source_index"]``, set by
processBoxData.py. The box order is never changed, so it equals the placement
order. The log calls it ``step`` (0 ... N-1).

Side effects
------------
Reads Box and Pallet objects only (never modifies them) and writes exactly one
JSON file, atomically: temporary file -> read back -> replace.
"""

from __future__ import annotations

import json
import os
from collections import Counter
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from . import settings
from .box import Box
from .pallet import Pallet

# Identification of the file format; palletReplayViewer.py checks both values.
LOG_FORMAT: str = "pallet_result_log"
LOG_FORMAT_VERSION: int = 1

# Number of offending items listed in an error message.
_ERROR_EXAMPLES: int = 5


class ResultLoggerError(RuntimeError):
    """Raised when the result log cannot be derived, validated or written."""


# ============================================================================
# INTERNAL DATA TYPES
# ============================================================================

@dataclass(frozen=True, slots=True)
class _BoxRecord:
    """One placed box with its global sequence index."""

    step: int
    pallet_index: int
    box: Box


@dataclass(frozen=True, slots=True)
class _PalletEvents:
    """Opening and closing step of one pallet."""

    opened_at_step: int
    closed_at_step: int | None


# ============================================================================
# PUBLIC ACTION
# ============================================================================

def logResults(  # noqa: N802 - name chosen to read like a PLC action call
    pallets: Sequence[Pallet],
    *,
    max_open_pallets: int | None,
    loader_name: str,
    source_file: str | Path | None,
    output_path: str | Path,
) -> Path:
    """Write the replay log of the finished run and return its path.

    pallets           the list returned by the loader, in opening order
    max_open_pallets  K of the run that produced ``pallets``; None = all open
    loader_name       registry key of the loader (shown by the viewer)
    source_file       the input JSON the boxes came from (informative only)
    output_path       the log file to write
    """
    pallet_list = tuple(pallets)
    if not pallet_list:
        raise ResultLoggerError("Nothing to log: the loader returned no pallets.")
    window = _effective_window(max_open_pallets, len(pallet_list))

    # ------------------------------------------------------------------
    # 1. COLLECT: every placed box with its sequence index
    # ------------------------------------------------------------------
    per_pallet = _collect_boxes(pallet_list)

    # ------------------------------------------------------------------
    # 2. DERIVE: opening and closing step of every pallet
    # ------------------------------------------------------------------
    events = _derive_pallet_events(per_pallet, window)

    # ------------------------------------------------------------------
    # 3. VALIDATE: fail loudly, write nothing
    # ------------------------------------------------------------------
    _validate_log(pallet_list, per_pallet, events, window)

    # ------------------------------------------------------------------
    # 4. WRITE
    # ------------------------------------------------------------------
    document = _build_document(
        pallet_list,
        per_pallet,
        events,
        max_open_pallets=max_open_pallets,
        loader_name=loader_name,
        source_file=source_file,
    )
    path = Path(output_path).expanduser().resolve()
    _write_json_atomic(path, document)
    return path


# ============================================================================
# 1. COLLECT
# ============================================================================

def _collect_boxes(pallets: Sequence[Pallet]) -> list[list[_BoxRecord]]:
    """Per pallet, its boxes in the pallet's own insertion (placement) order."""
    per_pallet: list[list[_BoxRecord]] = []

    for pallet_index, pallet in enumerate(pallets):
        records: list[_BoxRecord] = []
        for box in pallet.boxes:
            if (
                box.position is None
                or box.placed_dimensions is None
                or box.orientation is None
            ):
                raise ResultLoggerError(
                    f"Box {box.box_id!r} on pallet {pallet.pallet_id!r} has no placement."
                )

            step = box.metadata.get("source_index")
            # bool is a subclass of int and must not pass as an index.
            if isinstance(step, bool) or not isinstance(step, int) or step < 0:
                raise ResultLoggerError(
                    f"Box {box.box_id!r} has no valid metadata['source_index'] "
                    f"(found {step!r}). The boxes must come from processBoxData.py."
                )
            records.append(_BoxRecord(step=step, pallet_index=pallet_index, box=box))

        if not records:
            raise ResultLoggerError(
                f"Pallet {pallet.pallet_id!r} holds no boxes, so its opening step "
                f"is undefined."
            )
        per_pallet.append(records)

    return per_pallet


# ============================================================================
# 2. DERIVE
# ============================================================================

def _effective_window(max_open_pallets: int | None, pallet_count: int) -> int:
    """K used for the derivation. None (all pallets open) -> pallet count."""
    if max_open_pallets is None:
        return pallet_count
    if (
        isinstance(max_open_pallets, bool)
        or not isinstance(max_open_pallets, int)
        or max_open_pallets < 1
    ):
        raise ResultLoggerError(
            f"max_open_pallets must be None or an integer >= 1, got {max_open_pallets!r}."
        )
    return max_open_pallets


def _derive_pallet_events(
    per_pallet: Sequence[Sequence[_BoxRecord]],
    window: int,
) -> list[_PalletEvents]:
    """opened(n) = first box on n; closed(n) = opened(n + K) or None."""
    opened = [min(record.step for record in records) for records in per_pallet]
    pallet_count = len(opened)
    return [
        _PalletEvents(
            opened_at_step=opened[n],
            closed_at_step=opened[n + window] if n + window < pallet_count else None,
        )
        for n in range(pallet_count)
    ]


# ============================================================================
# 3. VALIDATE
# ============================================================================

def _validate_log(
    pallets: Sequence[Pallet],
    per_pallet: Sequence[Sequence[_BoxRecord]],
    events: Sequence[_PalletEvents],
    window: int,
) -> None:
    # ---- one pallet geometry for the whole run (the viewer draws one spec) --
    first = pallets[0]
    tolerance = settings.PLACEMENT_TOLERANCE_MM
    for pallet in pallets[1:]:
        for name in ("length_mm", "width_mm", "base_height_mm", "max_height_mm"):
            if abs(getattr(pallet, name) - getattr(first, name)) > tolerance:
                raise ResultLoggerError(
                    f"Pallet {pallet.pallet_id!r} has a different {name} than "
                    f"{first.pallet_id!r}; the log supports one pallet geometry."
                )

    # ---- sequence indices form exactly 0 ... N-1 ----------------------------
    steps = [record.step for records in per_pallet for record in records]
    box_count = len(steps)
    counts = Counter(steps)
    if sorted(counts) != list(range(box_count)) or len(counts) != box_count:
        duplicates = sorted(step for step, count in counts.items() if count > 1)
        missing = sorted(set(range(box_count)) - counts.keys())
        outside = sorted(step for step in counts if step >= box_count)
        raise ResultLoggerError(
            f"Sequence indices are not exactly 0..{box_count - 1}: "
            f"duplicates {duplicates[:_ERROR_EXAMPLES]}, "
            f"missing {missing[:_ERROR_EXAMPLES]}, "
            f"out of range {outside[:_ERROR_EXAMPLES]}."
        )

    # ---- pallets were opened in list order ----------------------------------
    for n in range(len(events) - 1):
        if events[n].opened_at_step >= events[n + 1].opened_at_step:
            raise ResultLoggerError(
                f"Pallet {pallets[n + 1].pallet_id!r} (first box at step "
                f"{events[n + 1].opened_at_step}) was opened before pallet "
                f"{pallets[n].pallet_id!r} (first box at step "
                f"{events[n].opened_at_step}). The loader must return its pallets "
                f"in opening order."
            )

    # ---- boxes within one pallet appear in placement order ------------------
    for pallet, records in zip(pallets, per_pallet, strict=True):
        for earlier, later in zip(records, records[1:]):
            if later.step <= earlier.step:
                raise ResultLoggerError(
                    f"Pallet {pallet.pallet_id!r} lists box {later.box.box_id!r} "
                    f"(step {later.step}) after box {earlier.box.box_id!r} "
                    f"(step {earlier.step}); boxes must be in placement order."
                )

    # ---- FIFO self-check: no box lands on a pallet after it closed ----------
    for pallet, records, event in zip(pallets, per_pallet, events, strict=True):
        if event.closed_at_step is None:
            continue
        late = [record for record in records if record.step >= event.closed_at_step]
        if late:
            raise ResultLoggerError(
                f"FIFO self-check failed: box {late[0].box.box_id!r} (step "
                f"{late[0].step}) lands on pallet {pallet.pallet_id!r} after the "
                f"pallet left the open window at step {event.closed_at_step} "
                f"(K = {window}). The active loader does not close its pallets "
                f"first-in-first-out with this max_open_pallets value."
            )


# ============================================================================
# 4. WRITE
# ============================================================================

def _build_document(
    pallets: Sequence[Pallet],
    per_pallet: Sequence[Sequence[_BoxRecord]],
    events: Sequence[_PalletEvents],
    *,
    max_open_pallets: int | None,
    loader_name: str,
    source_file: str | Path | None,
) -> dict[str, Any]:
    first = pallets[0]

    pallet_entries: list[dict[str, Any]] = []
    for index, (pallet, records, event) in enumerate(
        zip(pallets, per_pallet, events, strict=True)
    ):
        pallet_entries.append(
            {
                "index": index,
                "pallet_id": pallet.pallet_id,
                "box_count": len(records),
                "load_kg": pallet.current_load_kg,
                "opened_at_step": event.opened_at_step,
                "closed_at_step": event.closed_at_step,
            }
        )

    records_in_order = sorted(
        (record for records in per_pallet for record in records),
        key=lambda record: record.step,
    )
    box_entries: list[dict[str, Any]] = []
    for record in records_in_order:
        box = record.box
        # Checked in _collect_boxes; repeated here for the type checker.
        position = box.position
        size = box.placed_dimensions
        orientation = box.orientation
        assert position is not None and size is not None and orientation is not None
        box_entries.append(
            {
                "step": record.step,
                "box_id": box.box_id,
                "sku": box.sku,
                "name": box.name,
                "pallet_index": record.pallet_index,
                "orientation": orientation.value,
                "min_corner_mm": [position.x, position.y, position.z],
                "size_mm": [size.x, size.y, size.z],
                "mass_kg": box.weight_kg,
            }
        )

    return {
        "format": LOG_FORMAT,
        "format_version": LOG_FORMAT_VERSION,
        "units": {"length": "mm", "mass": "kg"},
        "source": {
            "input_file": None if source_file is None else str(source_file),
            "loader": loader_name,
        },
        "pallet_spec": {
            "length_mm": first.length_mm,
            "width_mm": first.width_mm,
            "base_height_mm": first.base_height_mm,
            "max_height_mm": first.max_height_mm,
            "max_load_kg": first.max_load_kg,
        },
        "placement_tolerance_mm": settings.PLACEMENT_TOLERANCE_MM,
        "max_open_pallets": max_open_pallets,
        "box_count": len(box_entries),
        "pallet_count": len(pallet_entries),
        "pallets": pallet_entries,
        "boxes": box_entries,
    }


def _write_json_atomic(path: Path, document: dict[str, Any]) -> None:
    """Write via a temporary file that is read back before it replaces the
    target, so an existing valid log is never left half-written."""
    temporary = path.with_name(f".{path.name}.tmp")
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        text = json.dumps(document, indent=2, allow_nan=False) + "\n"
        temporary.write_text(text, encoding="utf-8")
        json.loads(temporary.read_text(encoding="utf-8"))
        os.replace(temporary, path)
    except (OSError, TypeError, ValueError) as exc:
        try:
            temporary.unlink(missing_ok=True)
        except OSError:
            pass
        raise ResultLoggerError(f"Could not write result log {path}: {exc}") from exc
