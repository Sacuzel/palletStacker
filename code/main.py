"""Entry point for the pallet stacker application."""

from __future__ import annotations

import shlex
import sys
from collections.abc import Callable, Sequence
from pathlib import Path

from pallet_stacker import emsLoader, settings
from pallet_stacker.box import Box
from pallet_stacker.naiveLoader import NaiveLoaderError, load_boxes as naive_load_boxes
from pallet_stacker.heuristicLoader import HeuristicLoaderError, load_boxes as heuristic_load_boxes
from pallet_stacker.advancedLoader import AdvancedLoaderError, load_boxes as advanced_load_boxes
from pallet_stacker.experimentalLoader import ExperimentalLoaderError, load_boxes as experimental_load_boxes
from pallet_stacker.emsLoader import EmsLoaderError, load_boxes as ems_load_boxes
from pallet_stacker.optimizer import LoaderOptimizerError, load_boxes as optimizer_load_boxes
from pallet_stacker.optimizer import loaderOptimizer
from pallet_stacker.pallet import Pallet
from pallet_stacker.processBoxData import BoxDataError, process_box_data
from pallet_stacker.resultLogger import ResultLoggerError, logResults

LoaderFunction = Callable[[Sequence[Box]], list[Pallet]]


# Add future algorithms here after their modules exist. Every loader should use
# the same interface: load_boxes(boxes) -> list[Pallet].
LOADER_REGISTRY: dict[str, LoaderFunction] = {
    "naive": naive_load_boxes,
    "heuristic": heuristic_load_boxes,
    "advanced": advanced_load_boxes,
    "experimental": experimental_load_boxes,
    "ems": ems_load_boxes,
    "optimizer": optimizer_load_boxes,
}

# Loaders whose pallets close first-in-first-out through an open-pallet window
# (states[-K:]). Only these can be replayed; see resultLogger.py.
RESULT_LOG_LOADERS: frozenset[str] = frozenset({"ems", "optimizer"})

# The replay viewer lives next to this file.
REPLAY_VIEWER_FILE: Path = Path(__file__).resolve().parent / "palletReplayViewer.py"


def main() -> int:
    """Execute the application stages in a visible, deterministic order."""

    # ------------------------------------------------------------------
    # 0. WORKSPACE: establish predictable input/output directories
    # ------------------------------------------------------------------
    if settings.CREATE_MISSING_PROJECT_DIRECTORIES:
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
    loader_name = _active_loader_name()
    loader = _select_loader(loader_name)
    pallets = loader(box_data.boxes)

    # ------------------------------------------------------------------
    # 2b. ANALYSIS: loader-independent verification, stability and metrics
    # ------------------------------------------------------------------
    layout_report = None
    layout_report_file: Path | None = None
    if settings.RUN_LAYOUT_ANALYSIS:
        from pallet_stacker import layoutAnalyzer

        layout_report = layoutAnalyzer.analyze_layout(
            pallets,
            input_boxes=box_data.boxes,
        )
        if layoutAnalyzer.ANALYSIS_WRITE_JSON:
            layout_report_file = layoutAnalyzer.write_report_json(
                layout_report,
                layoutAnalyzer.ANALYSIS_JSON_FILE,
            )

    # ------------------------------------------------------------------
    # 3. OUTPUT: create optional visualizations and simulation assets
    # ------------------------------------------------------------------
    plotly_output: Path | None = None
    if settings.GENERATE_PLOTLY_OUTPUT:
        # Imported only when this output stage is enabled. This keeps the core
        # loader runnable even when Plotly is not installed.
        from pallet_stacker.plotlyVisualizer import write_layout_html

        plotly_output = write_layout_html(
            pallets,
            output_path=settings.PLOTLY_OUTPUT_FILE,
            title=f"{settings.PLOTLY_TITLE} - {loader_name} loader",
            show_box_labels=settings.PLOTLY_SHOW_BOX_LABELS,
            open_in_browser=settings.PLOTLY_OPEN_IN_BROWSER,
            include_plotlyjs=settings.PLOTLY_INCLUDE_PLOTLYJS,
            pallet_gap_mm=settings.PLOTLY_PALLET_GAP_MM,
            edge_width_px=settings.PLOTLY_EDGE_WIDTH_PX,
            edge_expand_mm=settings.PLOTLY_EDGE_EXPAND_MM,
        )

    # Result log for palletReplayViewer.py: final result, loading order and
    # the step at which each pallet left the open-pallet window.
    result_log: Path | None = None
    result_log_note: str | None = None
    if settings.GENERATE_RESULT_LOG:
        if loader_name in RESULT_LOG_LOADERS:
            result_log = logResults(
                pallets,
                max_open_pallets=_resolve_max_open_pallets(loader_name),
                loader_name=loader_name,
                source_file=box_data.source_path,
                output_path=settings.RESULT_LOG_FILE,
            )
        else:
            result_log_note = f"skipped (not supported for the {loader_name!r} loader)"

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
    # 4. STATUS: report the completed run and the next manual command
    # ------------------------------------------------------------------
    if settings.PRINT_RUN_SUMMARY:
        _print_summary(
            source_file=box_data.source_path,
            input_box_count=box_data.box_count,
            pallets=pallets,
            plotly_output=plotly_output,
            result_log=result_log,
            result_log_note=result_log_note,
            gazebo_world=gazebo_world,
            gazebo_manifest=gazebo_manifest,
            gazebo_gui_plugin=gazebo_gui_plugin,
            gazebo_gui_plugin_rebuilt=gazebo_gui_plugin_rebuilt,
            gazebo_gui_plugin_installed=gazebo_gui_plugin_installed,
        )

        if layout_report is not None:
            from pallet_stacker.layoutAnalyzer import format_report

            print()
            print(format_report(layout_report))
            if layout_report_file is not None:
                print(f"  Analysis report: {layout_report_file}")

    return 0


def _active_loader_name() -> str:
    """Registry key of the active loader (normalized like the registry keys)."""
    return settings.ACTIVE_LOADER.strip().lower()


def _select_loader(loader_name: str) -> LoaderFunction:
    normalized_name = loader_name.strip().lower()
    try:
        return LOADER_REGISTRY[normalized_name]
    except KeyError as exc:
        available = ", ".join(sorted(LOADER_REGISTRY))
        raise ValueError(
            f"Unknown loader {loader_name!r}. Available loaders: {available}."
        ) from exc


def _resolve_max_open_pallets(loader_name: str) -> int | None:
    """Open-pallet window K of the run that just finished. None = all open.

    ems        emsLoader.MAX_OPEN_PALLETS
    optimizer  the winning configuration (robust even if K is ever swept)
    """
    if loader_name == "ems":
        return emsLoader.MAX_OPEN_PALLETS
    if loader_name == "optimizer":
        report = loaderOptimizer.LAST_REPORT
        if report is None:
            raise ResultLoggerError(
                "The optimizer finished without a report (LAST_REPORT is None)."
            )
        return report.configs[report.best.config_index].max_open_pallets
    raise ValueError(f"No result log support for loader {loader_name!r}.")


def _print_summary(
    *,
    source_file: Path,
    input_box_count: int,
    pallets: Sequence[Pallet],
    plotly_output: Path | None,
    result_log: Path | None,
    result_log_note: str | None,
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

    if result_log is not None:
        print(f"  Result log: {result_log}")
        viewer = _project_relative_or_absolute(REPLAY_VIEWER_FILE)
        print(f"  Replay viewer: python {shlex.quote(str(viewer))}")
    elif result_log_note is not None:
        print(f"  Result log: {result_log_note}")

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
    except (BoxDataError, NaiveLoaderError, HeuristicLoaderError, AdvancedLoaderError,
            ExperimentalLoaderError, EmsLoaderError, LoaderOptimizerError,
            ResultLoggerError, ValueError, RuntimeError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc