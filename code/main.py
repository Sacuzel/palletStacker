"""Entry point for the pallet stacker application."""

from __future__ import annotations

import argparse
import json
import shlex
import sys
import tempfile
from collections.abc import Callable, Sequence
from pathlib import Path
from typing import Any

from pallet_stacker import settings
from pallet_stacker.box import Box
from pallet_stacker.naiveLoader import NaiveLoaderError, load_boxes as naive_load_boxes
from pallet_stacker.heuristicLoader import HeuristicLoaderError, load_boxes as heuristic_load_boxes
from pallet_stacker.advancedLoader import AdvancedLoaderError, load_boxes as advanced_load_boxes
from pallet_stacker.pallet import Pallet
from pallet_stacker.processBoxData import BoxDataError, process_box_data

LoaderFunction = Callable[[Sequence[Box]], list[Pallet]]


# Add future algorithms here after their modules exist. Every loader should use
# the same interface: load_boxes(boxes) -> list[Pallet].
LOADER_REGISTRY: dict[str, LoaderFunction] = {
    "naive": naive_load_boxes,
    "heuristic": heuristic_load_boxes,
    "advanced": advanced_load_boxes,
}


def main(argv: Sequence[str] | None = None) -> int:
    """Execute the application stages in a visible, deterministic order."""
    args = _parse_args(argv)

    # ------------------------------------------------------------------
    # 0. WORKSPACE: establish predictable input/output directories
    # ------------------------------------------------------------------
    if settings.CREATE_MISSING_PROJECT_DIRECTORIES and not args.headless:
        for directory in (
            settings.INPUT_DIRECTORY,
            settings.OUTPUT_DIRECTORY,
            settings.GAZEBO_MODELS_DIRECTORY,
            settings.GAZEBO_BRIDGE_DIRECTORY,
            settings.GAZEBO_WORLDS_DIRECTORY,
            settings.GAZEBO_PLUGINS_DIRECTORY,
        ):
            directory.mkdir(parents=True, exist_ok=True)

    # ------------------------------------------------------------------
    # 1. INPUT: select/parse JSON into Box objects
    # ------------------------------------------------------------------
    if args.headless:
        box_data = _process_stdin_json()
    else:
        box_data = process_box_data(
            settings.INPUT_FILE_PATH,
            initial_directory=settings.INPUT_DIRECTORY,
        )
        if box_data is None:
            print("File selection cancelled.")
            return 0

    # ------------------------------------------------------------------
    # 2. ALGORITHM: create pallets and place all boxes
    # ------------------------------------------------------------------
    loader = _select_loader(settings.ACTIVE_LOADER)
    pallets = loader(box_data.boxes)

    # ------------------------------------------------------------------
    # 3. HEADLESS RESULT: machine-readable JSON to stdout
    # ------------------------------------------------------------------
    if args.headless:
        json.dump(
            _build_result(
                input_box_count=box_data.box_count,
                pallets=pallets,
            ),
            sys.stdout,
            separators=(",", ":"),
        )
        sys.stdout.write("\n")
        return 0

    # ------------------------------------------------------------------
    # 4. OUTPUT: create optional visualizations and simulation assets
    # ------------------------------------------------------------------
    plotly_output: Path | None = None
    if settings.GENERATE_PLOTLY_OUTPUT:
        # Imported only when this output stage is enabled. This keeps the core
        # loader runnable even when Plotly is not installed.
        from pallet_stacker.plotlyVisualizer import write_layout_html

        plotly_output = write_layout_html(
            pallets,
            output_path=settings.PLOTLY_OUTPUT_FILE,
            title=settings.PLOTLY_TITLE,
            show_box_labels=settings.PLOTLY_SHOW_BOX_LABELS,
            open_in_browser=settings.PLOTLY_OPEN_IN_BROWSER,
            include_plotlyjs=settings.PLOTLY_INCLUDE_PLOTLYJS,
            pallet_gap_mm=settings.PLOTLY_PALLET_GAP_MM,
            edge_width_px=settings.PLOTLY_EDGE_WIDTH_PX,
            edge_expand_mm=settings.PLOTLY_EDGE_EXPAND_MM,
        )

    gazebo_world: Path | None = None
    gazebo_manifest: Path | None = None
    gazebo_gui_plugin: Path | None = None
    gazebo_gui_plugin_rebuilt = False
    gazebo_gui_plugin_installed = False

    if settings.GENERATE_GAZEBO_OUTPUT:
        from pallet_stacker.gzSimulator import create_simulation

        gazebo_result = create_simulation(
            pallets,
            output_path=settings.GAZEBO_WORLD_FILE,
            manifest_path=settings.GAZEBO_MANIFEST_FILE,
        )
        gazebo_world = gazebo_result.world_path
        gazebo_manifest = gazebo_result.manifest_path
        gazebo_gui_plugin = gazebo_result.gui_plugin_path
        gazebo_gui_plugin_rebuilt = gazebo_result.gui_plugin_rebuilt
        gazebo_gui_plugin_installed = gazebo_result.gui_plugin_installed

    # ------------------------------------------------------------------
    # 5. STATUS: report the completed run and the next manual command
    # ------------------------------------------------------------------
    if settings.PRINT_RUN_SUMMARY:
        _print_summary(
            source_file=box_data.source_path,
            input_box_count=box_data.box_count,
            pallets=pallets,
            plotly_output=plotly_output,
            gazebo_world=gazebo_world,
            gazebo_manifest=gazebo_manifest,
            gazebo_gui_plugin=gazebo_gui_plugin,
            gazebo_gui_plugin_rebuilt=gazebo_gui_plugin_rebuilt,
            gazebo_gui_plugin_installed=gazebo_gui_plugin_installed,
        )

    return 0


def _parse_args(argv: Sequence[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Read input JSON from stdin and write result JSON to stdout.",
    )
    return parser.parse_args(argv)


def _process_stdin_json():
    """Reuse the existing file parser while exposing JSON-over-stdin externally."""
    input_text = sys.stdin.read()
    if not input_text.strip():
        raise ValueError("Headless mode requires JSON input on stdin.")

    # Validate syntax before creating the temporary input file so malformed JSON
    # produces an immediate, clear error.
    json.loads(input_text)

    with tempfile.NamedTemporaryFile(
        mode="w",
        suffix=".json",
        encoding="utf-8",
        delete=False,
    ) as handle:
        handle.write(input_text)
        input_path = Path(handle.name)

    try:
        box_data = process_box_data(input_path)
        if box_data is None:
            raise ValueError("Headless input could not be processed.")
        return box_data
    finally:
        input_path.unlink(missing_ok=True)


def _build_result(*, input_box_count: int, pallets: Sequence[Pallet]) -> dict[str, Any]:
    """Serialize the algorithm output without changing the packing algorithm."""
    placed_count = sum(pallet.box_count for pallet in pallets)
    total_weight = sum(pallet.current_load_kg for pallet in pallets)

    placement_index = 0
    pallet_results: list[dict[str, Any]] = []

    for pallet in pallets:
        boxes: list[dict[str, Any]] = []
        for box in pallet.boxes:
            placement = box.placement
            if placement is None:
                raise RuntimeError(f"Placed pallet contains box without placement: {box.box_id}")

            boxes.append(
                {
                    "identifier": box.box_id,
                    "sku": box.sku,
                    "placement_index": placement_index,
                    "position_mm": _xyz_values(placement.position),
                    "orientation": _enum_value(placement.orientation),
                    "placed_dimensions_mm": _xyz_values(box.placed_dimensions),
                }
            )
            placement_index += 1

        pallet_results.append(
            {
                "pallet_id": pallet.pallet_id,
                "box_count": pallet.box_count,
                "current_load_kg": pallet.current_load_kg,
                "remaining_load_kg": pallet.remaining_load_kg,
                "load_height_mm": pallet.load_height_mm,
                "volume_utilization": pallet.volume_utilization,
                "boxes": boxes,
            }
        )

    return {
        "format_version": 1,
        "loader": settings.ACTIVE_LOADER,
        "summary": {
            "input_box_count": input_box_count,
            "placed_box_count": placed_count,
            "pallet_count": len(pallets),
            "total_weight_kg": total_weight,
        },
        "pallets": pallet_results,
    }


def _xyz_values(value: Any) -> list[Any]:
    if all(hasattr(value, name) for name in ("x", "y", "z")):
        return [value.x, value.y, value.z]
    return _sequence_values(value)


def _sequence_values(value: Any) -> list[Any]:
    if isinstance(value, (list, tuple)):
        return list(value)
    try:
        return list(value)
    except TypeError as exc:
        raise RuntimeError(f"Expected a sequence-like value, got {value!r}.") from exc


def _enum_value(value: Any) -> Any:
    return getattr(value, "value", value)


def _select_loader(loader_name: str) -> LoaderFunction:
    normalized_name = loader_name.strip().lower()
    try:
        return LOADER_REGISTRY[normalized_name]
    except KeyError as exc:
        available = ", ".join(sorted(LOADER_REGISTRY))
        raise ValueError(
            f"Unknown loader {loader_name!r}. Available loaders: {available}."
        ) from exc


def _print_summary(
    *,
    source_file: Path,
    input_box_count: int,
    pallets: Sequence[Pallet],
    plotly_output: Path | None,
    gazebo_world: Path | None,
    gazebo_manifest: Path | None,
    gazebo_gui_plugin: Path | None,
    gazebo_gui_plugin_rebuilt: bool,
    gazebo_gui_plugin_installed: bool,
) -> None:
    placed_count = sum(pallet.box_count for pallet in pallets)
    total_weight = sum(pallet.current_load_kg for pallet in pallets)

    print()
    print("Pallet stacker run complete")
    print(f"  Input file: {source_file}")
    print(f"  Loader: {settings.ACTIVE_LOADER}")
    print(f"  Input boxes: {input_box_count}")
    print(f"  Placed boxes: {placed_count}")
    print(f"  Pallets created: {len(pallets)}")
    print(f"  Total box weight: {total_weight:.2f} kg")

    for pallet in pallets:
        print(
            f"  {pallet.pallet_id}: {pallet.box_count} boxes, "
            f"{pallet.current_load_kg:.2f} kg, "
            f"height {pallet.load_height_mm:.1f} mm, "
            f"volume utilization {pallet.volume_utilization:.1%}"
        )

    if plotly_output is not None:
        print(f"  Plotly output: {plotly_output}")

    if gazebo_world is not None:
        print(f"  Gazebo world: {gazebo_world}")
        if gazebo_manifest is not None:
            print(f"  Gazebo manifest: {gazebo_manifest}")
        if gazebo_gui_plugin is not None:
            if gazebo_gui_plugin_rebuilt and gazebo_gui_plugin_installed:
                plugin_state = "rebuilt and installed"
            elif gazebo_gui_plugin_rebuilt:
                plugin_state = "rebuilt; installed copy already current"
            elif gazebo_gui_plugin_installed:
                plugin_state = "installed/updated"
            else:
                plugin_state = "current"
            print(f"  Forklift GUI plugin ({plugin_state}): {gazebo_gui_plugin}")
        else:
            print("  Integrated forklift controls: disabled")

        print("  Gazebo launch: not started by main.py; generation is complete")
        display_path = _project_relative_or_absolute(gazebo_world)
        command = f"gz sim {shlex.quote(str(display_path))}"
        print(f"  Run simulation: {command}")
        if gazebo_gui_plugin is not None:
            print(
                "  Forklift controls: W/S drive, A/D turn, "
                "arrows lift/lower, Space stop"
            )


def _project_relative_or_absolute(path: Path) -> Path:
    try:
        return path.resolve().relative_to(settings.PROJECT_ROOT.resolve())
    except ValueError:
        return path.resolve()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (BoxDataError, NaiveLoaderError, HeuristicLoaderError,AdvancedLoaderError,
            ValueError, RuntimeError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
