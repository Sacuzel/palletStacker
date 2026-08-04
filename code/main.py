"""Entry point for the pallet stacker application."""

from __future__ import annotations

import sys
from collections.abc import Callable, Sequence
from pathlib import Path

from pallet_stacker import settings
from pallet_stacker.box import Box
from pallet_stacker.naiveLoader import NaiveLoaderError, load_boxes as naive_load_boxes
from pallet_stacker.pallet import Pallet
from pallet_stacker.processBoxData import BoxDataError, process_box_data

LoaderFunction = Callable[[Sequence[Box]], list[Pallet]]


# Add future algorithms here after their modules exist. Every loader should use
# the same interface: load_boxes(boxes) -> list[Pallet].
LOADER_REGISTRY: dict[str, LoaderFunction] = {
    "naive": naive_load_boxes,
}


def main() -> int:
    """Execute the application stages in a visible, deterministic order."""

    # ------------------------------------------------------------------
    # 0. WORKSPACE: establish predictable input/output directories
    # ------------------------------------------------------------------
    if settings.CREATE_MISSING_PROJECT_DIRECTORIES:
        settings.INPUT_DIRECTORY.mkdir(parents=True, exist_ok=True)
        settings.OUTPUT_DIRECTORY.mkdir(parents=True, exist_ok=True)

    # ------------------------------------------------------------------
    # 1. INPUT: select and parse JSON into Box objects
    # ------------------------------------------------------------------
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
    # 3. OUTPUT: create optional visualizations and exports
    # ------------------------------------------------------------------
    output_path = None
    if settings.GENERATE_PLOTLY_OUTPUT:
        # Imported only when this output stage is enabled. This keeps the core
        # loader runnable even when Plotly is not installed.
        from pallet_stacker.plotlyVisualizer import write_layout_html

        output_path = write_layout_html(
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

    if settings.GENERATE_GAZEBO_OUTPUT:
        raise NotImplementedError(
            "Gazebo output is enabled, but the Gazebo exporter has not been "
            "implemented yet. Set GENERATE_GAZEBO_OUTPUT = False in settings.py."
        )

    # ------------------------------------------------------------------
    # 4. STATUS: report the completed run
    # ------------------------------------------------------------------
    if settings.PRINT_RUN_SUMMARY:
        _print_summary(
            source_file=box_data.source_path,
            input_box_count=box_data.box_count,
            pallets=pallets,
            plotly_output=output_path,
        )

    return 0


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


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (BoxDataError, NaiveLoaderError, ValueError, RuntimeError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
