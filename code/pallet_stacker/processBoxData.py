"""Select and parse a pallet-stacker box-data JSON file.

This module has two responsibilities:
1. Open a native file-selection dialog.
2. Validate the selected JSON file and convert its entries into ``Box`` objects.

The parser preserves the order of the ``boxes`` array because the packing
algorithm processes boxes sequentially. It does not create a ``Pallet`` because
the input format contains no pallet geometry or capacity information.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from json import JSONDecodeError
from math import isfinite
from pathlib import Path
from typing import Any

from . import settings
from .box import Box


class BoxDataError(ValueError):
    """Raised when a box-data file is missing, malformed, or invalid."""


@dataclass(frozen=True, slots=True)
class ProcessedBoxData:
    """Validated contents of one input file."""

    source_path: Path
    format_version: int
    dimension_unit: str
    weight_unit: str
    boxes: tuple[Box, ...]

    @property
    def box_count(self) -> int:
        return len(self.boxes)


def select_json_file(initial_directory: str | Path | None = None) -> Path | None:
    """Open a file dialog and return the selected JSON path.

    Returns ``None`` when the user cancels the dialog. Tkinter is imported only
    when this function is called, so parsing can be tested in headless systems.
    """

    try:
        import tkinter as tk
        from tkinter import filedialog
    except ImportError as exc:
        raise RuntimeError(
            "Tkinter is not installed. On Ubuntu, install it with: "
            "sudo apt install python3-tk"
        ) from exc

    initial_dir = None
    if initial_directory is not None:
        candidate = Path(initial_directory).expanduser()
        if candidate.is_dir():
            initial_dir = str(candidate.resolve())

    root = tk.Tk()
    root.withdraw()

    try:
        root.attributes("-topmost", True)
        selected = filedialog.askopenfilename(
            parent=root,
            title=settings.FILE_DIALOG_TITLE,
            initialdir=initial_dir,
            filetypes=settings.FILE_DIALOG_FILE_TYPES,
        )
    finally:
        root.destroy()

    if not selected:
        return None

    return Path(selected).expanduser().resolve()


def parse_box_json(file_path: str | Path) -> ProcessedBoxData:
    """Parse one JSON file and create unplaced ``Box`` objects.

    The order of the source ``boxes`` array is retained. The expected units are
    millimetres and kilograms; unsupported units are rejected rather than
    silently converted.
    """

    path = Path(file_path).expanduser().resolve()

    if not path.exists():
        raise BoxDataError(f"Input file does not exist: {path}")
    if not path.is_file():
        raise BoxDataError(f"Input path is not a file: {path}")
    if path.suffix.lower() != ".json":
        raise BoxDataError(f"Input file must have a .json extension: {path.name}")

    try:
        with path.open("r", encoding="utf-8") as json_file:
            raw_data = json.load(json_file)
    except OSError as exc:
        raise BoxDataError(f"Could not read input file {path}: {exc}") from exc
    except JSONDecodeError as exc:
        raise BoxDataError(
            f"Invalid JSON in {path.name} at line {exc.lineno}, "
            f"column {exc.colno}: {exc.msg}"
        ) from exc

    if not isinstance(raw_data, dict):
        raise BoxDataError("The JSON root must be an object.")

    format_version = raw_data.get("format_version")
    if format_version != settings.JSON_FORMAT_VERSION:
        raise BoxDataError(
            f"Unsupported format_version {format_version!r}; "
            f"expected {settings.JSON_FORMAT_VERSION}."
        )

    units = raw_data.get("units")
    if not isinstance(units, dict):
        raise BoxDataError("Field 'units' must be an object.")

    dimension_unit = units.get("dimensions")
    weight_unit = units.get("weight")

    if dimension_unit != settings.JSON_DIMENSION_UNIT:
        raise BoxDataError(
            f"Unsupported dimension unit {dimension_unit!r}; "
            f"expected {settings.JSON_DIMENSION_UNIT!r}."
        )
    if weight_unit != settings.JSON_WEIGHT_UNIT:
        raise BoxDataError(
            f"Unsupported weight unit {weight_unit!r}; "
            f"expected {settings.JSON_WEIGHT_UNIT!r}."
        )

    raw_boxes = raw_data.get("boxes")
    if not isinstance(raw_boxes, list):
        raise BoxDataError("Field 'boxes' must be an array.")

    boxes: list[Box] = []
    seen_identifiers: set[str] = set()

    for index, raw_box in enumerate(raw_boxes):
        location = f"boxes[{index}]"
        if not isinstance(raw_box, dict):
            raise BoxDataError(f"{location} must be an object.")

        identifier = _required_text(raw_box, "identifier", location)
        sku = _required_text(raw_box, "sku", location)

        if identifier in seen_identifiers:
            raise BoxDataError(
                f"Duplicate box identifier {identifier!r} at {location}."
            )
        seen_identifiers.add(identifier)

        dimensions = raw_box.get("dimensions_mm")
        if not isinstance(dimensions, (list, tuple)) or len(dimensions) != 3:
            raise BoxDataError(
                f"{location}.dimensions_mm must contain exactly "
                "[length, width, height]."
            )

        length_mm = _positive_number(dimensions[0], f"{location}.dimensions_mm[0]")
        width_mm = _positive_number(dimensions[1], f"{location}.dimensions_mm[1]")
        height_mm = _positive_number(dimensions[2], f"{location}.dimensions_mm[2]")
        weight_kg = _non_negative_number(
            raw_box.get("weight_kg"),
            f"{location}.weight_kg",
        )

        known_fields = {
            "identifier",
            "sku",
            "dimensions_mm",
            "weight_kg",
        }
        metadata: dict[str, Any] = {
            key: value for key, value in raw_box.items() if key not in known_fields
        }
        metadata["source_index"] = index

        try:
            box = Box(
                box_id=identifier,
                sku=sku,
                length_mm=length_mm,
                width_mm=width_mm,
                height_mm=height_mm,
                weight_kg=weight_kg,
                allowed_orientations=settings.DEFAULT_BOX_ALLOWED_ORIENTATIONS,
                metadata=metadata,
            )
        except (TypeError, ValueError) as exc:
            raise BoxDataError(f"Invalid data at {location}: {exc}") from exc

        boxes.append(box)

    return ProcessedBoxData(
        source_path=path,
        format_version=format_version,
        dimension_unit=dimension_unit,
        weight_unit=weight_unit,
        boxes=tuple(boxes),
    )


def process_box_data(
    file_path: str | Path | None = None,
    *,
    initial_directory: str | Path | None = None,
) -> ProcessedBoxData | None:
    """Select and parse box data through one main entry point.

    When ``file_path`` is omitted, the file-selection dialog is shown. ``None``
    is returned only when the user cancels the dialog. Invalid files raise
    ``BoxDataError`` so that ``main.py`` can decide how to present the error.
    """

    selected_path = (
        Path(file_path).expanduser().resolve()
        if file_path is not None
        else select_json_file(initial_directory)
    )

    if selected_path is None:
        return None

    return parse_box_json(selected_path)


def _required_text(data: dict[str, Any], key: str, location: str) -> str:
    value = data.get(key)
    if not isinstance(value, str) or not value.strip():
        raise BoxDataError(f"{location}.{key} must be a non-empty string.")
    return value.strip()


def _positive_number(value: Any, location: str) -> float:
    number = _finite_number(value, location)
    if number <= 0:
        raise BoxDataError(f"{location} must be greater than zero.")
    return number


def _non_negative_number(value: Any, location: str) -> float:
    number = _finite_number(value, location)
    if number < 0:
        raise BoxDataError(f"{location} must not be negative.")
    return number


def _finite_number(value: Any, location: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise BoxDataError(f"{location} must be a number.")

    number = float(value)
    if not isfinite(number):
        raise BoxDataError(f"{location} must be finite.")
    return number
