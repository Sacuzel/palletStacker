"""Loader optimizer: iterates the EMS loader until the load stops improving.

Public interface (same as every other loader)
---------------------------------------------
    load_boxes(boxes) -> list[Pallet]

main.py can register this function in its loader table. The whole order is
known in advance and the box order is never changed; only the decisions of
the inner loader are optimized.

Program flow
------------
0. PROBLEM   read-only box data, pallet envelope, baseline configuration
             (emsLoader's own settings) and all rule combinations
1. BASELINE  plain emsLoader behaviour; fixes the stability gate
2. STAGE 1   rule sweep: every rule combination with default decisions
3. STAGE 2   BRKGA: per-box steering keys on top of the best combinations,
             with an optional genetic extension (GENETIC_EXTENSION: "none",
             "ladder" or "islands"), stopped by the window criterion,
             generation limit or time limit
4. REPLAY    the best solution is decoded once more and placed on the
             ORIGINAL Box objects through Pallet.place_box (normal validation)

Trials never touch Box or Pallet objects. A trial is described completely by
(configuration index, steering keys), so any solution can be rebuilt exactly.

After a run, LAST_REPORT holds the baseline, the ranked sweep, the BRKGA
history and the winning solution.

Modules
-------
    optimizerSettings.py  all tunable values
    emsDecoder.py         steerable fork of emsLoader (one trial = one decode)
    evaluator.py          metrics, objective, stability gate
    geometry.py           hull / polygon code shared by decoder and evaluator
    ruleSweep.py          Stage 1
    brkga.py              Stage 2
"""

from __future__ import annotations

import copy
import multiprocessing
from multiprocessing.context import BaseContext
import os
import time
from collections.abc import Iterable, Sequence
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, field, replace
from math import isfinite

from .. import emsLoader, settings
from ..box import Box, Point3D
from ..pallet import Pallet
from . import optimizerSettings as opt
from .brkga import BrkgaParams, BrkgaResult, Chromosome, Scored, run_brkga
from .emsDecoder import (
    KEYS_PER_BOX,
    DecoderConfig,
    DecoderError,
    BoxSpec,
    Layout,
    PalletSpec,
    SteeringParams,
    decode,
    make_box_specs,
)
from .evaluator import (
    GATE_MODES,
    EvalParams,
    Metrics,
    StabilityGate,
    Trial,
    evaluate,
    format_metrics,
    ranking_key,
    resolve_gate,
)
from .ruleSweep import build_sweep_configs, format_sweep_table, run_rule_sweep

# One trial to evaluate: (configuration index, steering keys or None).
Task = tuple[int, tuple[float, ...] | None]

# DecoderConfig field -> name of the setting in emsLoader.py.
_EMS_LOADER_SETTINGS: dict[str, str] = {
    "bin_rule": "BIN_SELECTION_RULE",
    "space_rule": "SPACE_SELECTION_RULE",
    "orientation_rule": "ORIENTATION_SELECTION_RULE",
    "n_ems_candidates": "N_EMS_CANDIDATES",
    "stability_constraint": "STABILITY_CONSTRAINT",
    "partial_base_support_ratio": "PARTIAL_BASE_SUPPORT_RATIO",
    "polygon_min_area_ratio": "POLYGON_MIN_AREA_RATIO",
    "support_flush_tol_mm": "SUPPORT_FLUSH_TOL_MM",
    "max_open_pallets": "MAX_OPEN_PALLETS",
    "enforce_blocking": "ENFORCE_BLOCKING",
    "guard_pass_through": "GUARD_PASS_THROUGH",
    "ems_min_side_mm": "EMS_MIN_SIDE_MM",
    "random_seed": "RANDOM_SEED",
    "score_rel_eps": "SCORE_REL_EPS",
    "weight_eps_kg": "WEIGHT_EPS_KG",
}

# Sweep configurations per worker between two checks of the time limit.
_SWEEP_BATCH_PER_WORKER: int = 8
# Task chunks per worker inside one batch (smaller chunks balance better).
_CHUNKS_PER_WORKER: int = 4
# A BRKGA progress line is printed at least this often [generations].
_PROGRESS_EVERY_GENERATIONS: int = 10
# Position tolerance of the decoder-vs-emsLoader comparison [mm].
_VERIFY_POSITION_TOL_MM: float = 1e-6


class LoaderOptimizerError(RuntimeError):
    """Raised when the optimizer cannot produce a valid load."""


# ============================================================================
# DATA TYPES
# ============================================================================

@dataclass(frozen=True, slots=True)
class _Problem:
    """Everything a trial needs; identical in the main and worker processes."""

    box_specs: tuple[BoxSpec, ...]
    pallet: PalletSpec
    configs: tuple[DecoderConfig, ...]      # index 0 = baseline
    steering: SteeringParams
    eval_params: EvalParams


@dataclass(slots=True)
class OptimizerReport:
    """Record of the last optimizer run (see LAST_REPORT)."""

    box_count: int
    configs: tuple[DecoderConfig, ...]
    gate: StabilityGate
    baseline: Trial
    best: Trial
    sweep_ranked: list[Trial] = field(default_factory=list)
    sweep_complete: bool = True
    brkga_generations: int = 0
    brkga_stop_reason: str = "not run"
    brkga_extension: str = "none"
    brkga_best_island: int | None = None              # "islands" only
    brkga_history: list[Trial] = field(default_factory=list)
    evaluations: int = 0
    workers: int = 1
    elapsed_s: float = 0.0
    decoder_matches_ems_loader: bool | None = None   # None = not checked


LAST_REPORT: OptimizerReport | None = None


# ============================================================================
# MAIN ENTRY POINT
# ============================================================================

def load_boxes(boxes: Iterable[Box]) -> list[Pallet]:
    """Load all boxes in arrival order and return the optimized pallets."""
    global LAST_REPORT

    _validate_settings()
    box_sequence = tuple(boxes)
    if not box_sequence:
        return []
    _validate_boxes(box_sequence)

    # ------------------------------------------------------------------
    # 0. PROBLEM: read-only data shared by all trials
    # ------------------------------------------------------------------
    base_config = _base_config_from_ems_loader()
    pallet_spec = PalletSpec(
        length_mm=settings.PALLET_LENGTH_MM,
        width_mm=settings.PALLET_WIDTH_MM,
        max_height_mm=settings.PALLET_MAX_HEIGHT_MM,
        max_load_kg=settings.PALLET_MAX_LOAD_KG,
    )
    box_specs = make_box_specs(box_sequence)
    eval_params = _eval_params(base_config)
    steering = _steering_params()
    if opt.RUN_RULE_SWEEP:
        configs = tuple(
            build_sweep_configs(
                base_config,
                opt.SWEEP_BIN_RULES,
                opt.SWEEP_SPACE_RULES,
                opt.SWEEP_ORIENTATION_RULES,
                opt.SWEEP_N_EMS_CANDIDATES,
            )
        )
    else:
        configs = (base_config,)
    problem = _Problem(box_specs, pallet_spec, configs, steering, eval_params)

    open_pallets = (
        "all" if base_config.max_open_pallets is None
        else str(base_config.max_open_pallets)
    )
    _log(
        f"{len(box_specs)} boxes | baseline rule {base_config.label} "
        f"({base_config.bin_rule}, space {base_config.space_rule}, "
        f"orientation {base_config.orientation_rule}) | "
        f"static stability: {base_config.stability_constraint} | "
        f"open pallets: {open_pallets}"
    )

    # ------------------------------------------------------------------
    # 1. BASELINE: plain emsLoader behaviour, fixes the stability gate
    # ------------------------------------------------------------------
    try:
        baseline_layout = decode(box_specs, pallet_spec, base_config)
    except DecoderError as exc:
        raise LoaderOptimizerError(str(exc)) from exc
    baseline_metrics = evaluate(baseline_layout, box_specs, pallet_spec, eval_params)
    gate = resolve_gate(opt.STABILITY_GATE, baseline_metrics, eval_params)
    baseline = _make_trial((0, None), baseline_metrics, gate, eval_params, "baseline")
    _log(f"baseline: {format_metrics(baseline_metrics)}")
    _log(f"stability gate: {_gate_text(gate, eval_params)}")

    decoder_matches: bool | None = None
    if opt.VERIFY_DECODER_AGAINST_EMS_LOADER:
        decoder_matches = _verify_against_ems_loader(box_sequence, baseline_layout)

    # The time budget starts after the one-off verification.
    started = time.monotonic()
    deadline = None if opt.TIME_LIMIT_S is None else started + opt.TIME_LIMIT_S

    runner = _TrialRunner(problem, _worker_count())
    ranked: list[Trial] = [baseline]
    sweep_complete = True
    brkga_result: BrkgaResult | None = None
    best = baseline

    try:
        # --------------------------------------------------------------
        # 2. STAGE 1: rule sweep
        # --------------------------------------------------------------
        if len(configs) > 1:
            _log(
                f"stage 1: rule sweep, {len(configs)} combinations, "
                f"{runner.workers} worker process(es)"
            )

            def evaluate_configs(indices: Sequence[int]) -> list[Trial]:
                tasks: list[Task] = [(index, None) for index in indices]
                return [
                    _make_trial(task, metrics, gate, eval_params, "sweep")
                    for task, metrics in zip(tasks, runner.run(tasks), strict=True)
                ]

            ranked, sweep_complete = run_rule_sweep(
                baseline,
                len(configs),
                evaluate_configs,
                batch_size=runner.workers * _SWEEP_BATCH_PER_WORKER,
                deadline=deadline,
            )
            if not sweep_complete:
                _log(
                    f"stage 1 stopped by the time limit after "
                    f"{len(ranked)} of {len(configs)} combinations"
                )
            for line in format_sweep_table(ranked, configs, top=opt.SWEEP_REPORT_TOP):
                _log("  " + line)
            best = ranked[0]
            _log(f"stage 1 done in {time.monotonic() - started:.1f} s")

        # --------------------------------------------------------------
        # 3. STAGE 2: BRKGA on top of the best rule combinations
        # --------------------------------------------------------------
        if opt.RUN_BRKGA:
            if deadline is not None and time.monotonic() >= deadline:
                _log("stage 2 skipped: time limit reached")
            else:
                brkga_result = _run_stage_two(
                    problem, runner, ranked, gate, deadline, started
                )
                candidate = brkga_result.best
                if isinstance(candidate, Trial) and candidate.key < best.key:
                    best = candidate
    finally:
        runner.close()

    # ------------------------------------------------------------------
    # 4. REPLAY: rebuild the winner on the original Box objects
    # ------------------------------------------------------------------
    if not best.passed:
        raise LoaderOptimizerError(_gate_failure_message(best, gate, eval_params))

    layout = decode(
        box_specs, pallet_spec, configs[best.config_index], best.keys, steering
    )
    if evaluate(layout, box_specs, pallet_spec, eval_params) != best.metrics:
        raise LoaderOptimizerError(
            "Internal error: replaying the best solution gave different metrics."
        )
    pallets = _build_pallets(box_sequence, layout)

    elapsed = time.monotonic() - started
    LAST_REPORT = OptimizerReport(
        box_count=len(box_specs),
        configs=configs,
        gate=gate,
        baseline=baseline,
        best=best,
        sweep_ranked=ranked,
        sweep_complete=sweep_complete,
        brkga_generations=brkga_result.generations if brkga_result else 0,
        brkga_stop_reason=brkga_result.stop_reason if brkga_result else "not run",
        brkga_extension=brkga_result.extension if brkga_result else opt.GENETIC_EXTENSION,
        brkga_best_island=(
            brkga_result.best_island
            if brkga_result and brkga_result.extension == "islands" else None
        ),
        brkga_history=(
            [t for t in brkga_result.history if isinstance(t, Trial)]
            if brkga_result else []
        ),
        evaluations=runner.evaluations + 1,          # + baseline
        workers=runner.workers,
        elapsed_s=elapsed,
        decoder_matches_ems_loader=decoder_matches,
    )
    _log_result(LAST_REPORT)
    return pallets


# ============================================================================
# STAGE 2
# ============================================================================

def _run_stage_two(
    problem: _Problem,
    runner: _TrialRunner,
    ranked: Sequence[Trial],
    gate: StabilityGate,
    deadline: float | None,
    started: float,
) -> BrkgaResult:
    """BRKGA over [configuration gene] + KEYS_PER_BOX keys per box."""
    # Best rule combinations of Stage 1 (failed decodes cannot seed anything).
    seed_trials = [t for t in ranked if t.metrics is not None][: opt.BRKGA_SEED_CONFIGS]
    seed_configs = [t.config_index for t in seed_trials]
    n_configs = len(seed_configs)
    n_box_genes = KEYS_PER_BOX * len(problem.box_specs)
    n_genes = 1 + n_box_genes

    def to_task(chromosome: Chromosome) -> Task:
        # Gene 0 selects the rule combination, the rest steer the boxes.
        selected = min(int(chromosome[0] * n_configs), n_configs - 1)
        return (seed_configs[selected], tuple(chromosome[1:]))

    def evaluate_chromosomes(chromosomes: Sequence[Chromosome]) -> list[Trial]:
        tasks = [to_task(chromosome) for chromosome in chromosomes]
        return [
            _make_trial(task, metrics, gate, problem.eval_params, "brkga")
            for task, metrics in zip(tasks, runner.run(tasks), strict=True)
        ]

    # One seed per rule combination: all box keys 0.0 = default decisions, so
    # generation 0 already contains the best results of Stage 1.
    seeds: list[Chromosome] = [
        [(k + 0.5) / n_configs] + [0.0] * n_box_genes for k in range(n_configs)
    ]

    params = _brkga_params()
    labels = ", ".join(problem.configs[i].label for i in seed_configs)
    _log(
        f"stage 2: BRKGA, {_extension_text(params)}, {n_genes} genes, "
        f"seeded with {labels}, {runner.workers} worker process(es)"
    )

    islands = params.extension == "islands"
    last_logged: list[tuple | None] = [None]

    def on_generation(
        generation: int, best_of_generation: Scored, evaluations: int, island: int
    ) -> None:
        improved = best_of_generation.stall_key != last_logged[0]
        if improved or generation % _PROGRESS_EVERY_GENERATIONS == 0:
            last_logged[0] = best_of_generation.stall_key
            metrics = (
                best_of_generation.metrics
                if isinstance(best_of_generation, Trial)
                else None
            )
            where = f" | island {island}" if islands else ""
            _log(
                f"  gen {generation:>4} | {time.monotonic() - started:>6.1f} s | "
                f"{'new best' if improved and generation else '        '} | "
                f"{format_metrics(metrics)}{where}"
            )

    def on_escalation(generation: int, step: str) -> None:
        what = {
            "shake": f"shake ({opt.SHAKE_BLOCK_BOXES}-box blocks redrawn around the elite)",
            "restart": "restart (best kept, seeds and new random chromosomes)",
        }
        _log(
            f"  gen {generation:>4} | {time.monotonic() - started:>6.1f} s | "
            f"stalled -> {what.get(step, step)}"
        )

    result = run_brkga(
        n_genes,
        evaluate_chromosomes,
        params,
        opt.RANDOM_SEED,
        seeds=seeds,
        fixed_genes=1,              # gene 0 selects the rule combination
        deadline=deadline,
        on_generation=on_generation,
        on_escalation=on_escalation,
    )

    if result.stop_reason == "window":
        if params.extension == "ladder":
            reason = (
                f"no meaningful improvement after {params.max_restarts} "
                f"restart(s) (ladder exhausted)"
            )
        elif islands:
            reason = (
                f"no meaningful improvement in {params.stall_window} "
                f"generations on any island"
            )
        else:
            reason = f"no meaningful improvement in {params.stall_window} generations"
    elif result.stop_reason == "max_generations":
        reason = "generation limit reached"
    elif result.stop_reason == "time_limit":
        reason = "time limit reached"
    else:
        reason = result.stop_reason

    if params.extension == "ladder":
        details = f" ({result.shakes} shake(s), {result.restarts} restart(s))"
    elif islands:
        details = f" (best found on island {result.best_island})"
    else:
        details = ""
    _log(f"stage 2 stopped after {result.generations} generations{details}: {reason}")
    return result


def _extension_text(params: BrkgaParams) -> str:
    """Short description of the active genetic extension for the log."""
    if params.extension == "islands":
        migration = (
            f"migration every {params.migration_interval} generations "
            f"({params.migrants} migrants)"
            if params.migration_interval > 0 and params.islands > 1
            else "no migration"
        )
        return (
            f"islands {params.islands} x {params.island_population} "
            f"(seeds {opt.RANDOM_SEED}..{opt.RANDOM_SEED + params.islands - 1}), "
            f"{migration}"
        )
    if params.extension == "ladder":
        return (
            f"population {params.population}, ladder ({params.shakes_per_restart} "
            f"shake(s) per restart, up to {params.max_restarts} restart(s))"
        )
    return f"population {params.population}"


# ============================================================================
# TRIAL EVALUATION (main process and worker processes)
# ============================================================================

def _run_task(problem: _Problem, task: Task) -> Metrics | None:
    """Decode and score one trial. None = the decoder could not finish."""
    config_index, keys = task
    try:
        layout = decode(
            problem.box_specs,
            problem.pallet,
            problem.configs[config_index],
            keys,
            problem.steering,
        )
    except DecoderError:
        return None
    return evaluate(layout, problem.box_specs, problem.pallet, problem.eval_params)


_WORKER_PROBLEM: _Problem | None = None


def _worker_init(problem: _Problem) -> None:
    """Runs once in every worker process."""
    global _WORKER_PROBLEM
    _WORKER_PROBLEM = problem


def _worker_task(task: Task) -> Metrics | None:
    if _WORKER_PROBLEM is None:
        raise LoaderOptimizerError("Worker process was not initialized.")
    return _run_task(_WORKER_PROBLEM, task)


class _TrialRunner:
    """Evaluates batches of trials, in worker processes when available.

    Results are returned in task order, so the outcome does not depend on the
    number of workers. If the pool cannot be used, evaluation continues in
    the main process.
    """

    def __init__(self, problem: _Problem, workers: int) -> None:
        self.problem = problem
        self.workers = max(1, workers)
        self.evaluations = 0
        self._pool: ProcessPoolExecutor | None = None
        if self.workers > 1:
            try:
                self._pool = ProcessPoolExecutor(
                    max_workers=self.workers,
                    mp_context=_mp_context(),
                    initializer=_worker_init,
                    initargs=(problem,),
                )
            except Exception as exc:  # noqa: BLE001 - any failure -> one process
                _log(f"multiprocessing unavailable ({exc}); using one process")
                self.workers = 1

    def run(self, tasks: Sequence[Task]) -> list[Metrics | None]:
        self.evaluations += len(tasks)
        if self._pool is not None:
            try:
                chunk = max(1, len(tasks) // (self.workers * _CHUNKS_PER_WORKER))
                return list(self._pool.map(_worker_task, tasks, chunksize=chunk))
            except Exception as exc:  # noqa: BLE001 - retry in the main process
                _log(
                    f"worker pool failed ({type(exc).__name__}: {exc}); "
                    f"continuing in one process"
                )
                self.close()
                self.workers = 1
        return [_run_task(self.problem, task) for task in tasks]

    def close(self) -> None:
        if self._pool is not None:
            self._pool.shutdown(wait=False, cancel_futures=True)
            self._pool = None


def _mp_context() -> BaseContext:
    """'fork' where available: workers inherit the loaded modules and the
    calling script (main.py, loaderTester.py) is not imported again."""
    if "fork" in multiprocessing.get_all_start_methods():
        return multiprocessing.get_context("fork")
    return multiprocessing.get_context()


def _worker_count() -> int:
    if opt.N_WORKERS > 0:
        return opt.N_WORKERS
    return max(1, (os.cpu_count() or 1) - 1)


def _make_trial(
    task: Task,
    metrics: Metrics | None,
    gate: StabilityGate,
    eval_params: EvalParams,
    source: str,
) -> Trial:
    return Trial(
        config_index=task[0],
        keys=task[1],
        metrics=metrics,
        key=ranking_key(metrics, gate, eval_params),
        source=source,
    )


# ============================================================================
# CONFIGURATION
# ============================================================================

def _base_config_from_ems_loader() -> DecoderConfig:
    """Baseline decoder configuration = emsLoader's current settings."""
    values: dict[str, object] = {}
    missing: list[str] = []
    for field_name, setting_name in _EMS_LOADER_SETTINGS.items():
        if hasattr(emsLoader, setting_name):
            values[field_name] = getattr(emsLoader, setting_name)
        else:
            missing.append(setting_name)
    if missing:
        _log(
            "WARNING: emsLoader has no setting(s) "
            + ", ".join(missing)
            + "; the decoder defaults are used for them"
        )
    # The decoder needs a concrete seed so that every trial can be replayed.
    if values.get("random_seed") is None:
        values["random_seed"] = opt.RANDOM_SEED

    config = replace(
        DecoderConfig(),
        placement_tol_mm=settings.PLACEMENT_TOLERANCE_MM,
        **values,
    )
    config.validate()
    return config


def _eval_params(base_config: DecoderConfig) -> EvalParams:
    params = EvalParams(
        gravity_m_s2=opt.GRAVITY_M_S2,
        accel_travel_m_s2=max(opt.ACCEL_FORWARD_M_S2, opt.ACCEL_BRAKING_M_S2),
        accel_lateral_m_s2=opt.ACCEL_LATERAL_M_S2,
        travel_axis=opt.TRAVEL_AXIS,
        required_margin=opt.STABILITY_REQUIRED_MARGIN,
        stability_unit=opt.STABILITY_UNIT,
        support_flush_tol_mm=base_config.support_flush_tol_mm,
        placement_tol_mm=base_config.placement_tol_mm,
        weight_ssf=opt.DENSITY_WEIGHT_SSF,
        weight_envelope=opt.DENSITY_WEIGHT_ENVELOPE,
        density_resolution=opt.DENSITY_RESOLUTION,
        cog_resolution_mm=opt.COG_RESOLUTION_MM,
    )
    params.validate()
    return params


def _steering_params() -> SteeringParams:
    params = SteeringParams(
        deviation_rate=opt.DEVIATION_RATE,
        space_alternatives=opt.SPACE_ALTERNATIVES,
        early_new_pallet=opt.EARLY_NEW_PALLET,
    )
    params.validate()
    return params

def _brkga_params() -> BrkgaParams:
    """BRKGA parameters from optimizerSettings (used by validation and Stage 2).
    BrkgaParams.validate() checks the common values and those of the
    selected GENETIC_EXTENSION."""
    return BrkgaParams(
        population=opt.BRKGA_POPULATION,
        elite_fraction=opt.BRKGA_ELITE_FRACTION,
        mutant_fraction=opt.BRKGA_MUTANT_FRACTION,
        elite_inheritance=opt.BRKGA_ELITE_INHERITANCE,
        max_generations=opt.BRKGA_MAX_GENERATIONS,
        stall_window=opt.STALL_WINDOW_GENERATIONS,
        extension=opt.GENETIC_EXTENSION,
        # "ladder"
        shakes_per_restart=opt.LADDER_SHAKES_PER_RESTART,
        max_restarts=opt.LADDER_MAX_RESTARTS,
        shake_block_genes=KEYS_PER_BOX * opt.SHAKE_BLOCK_BOXES,
        shake_intensity=opt.SHAKE_INTENSITY,
        # A key at or above this value makes the decoder deviate.
        shake_threshold=1.0 - opt.DEVIATION_RATE,
        # "islands"
        islands=opt.ISLAND_COUNT,
        island_population=opt.ISLAND_POPULATION,
        migration_interval=opt.ISLAND_MIGRATION_INTERVAL,
        migrants=opt.ISLAND_MIGRANTS,
    )

# ============================================================================
# REPLAY ON THE ORIGINAL BOXES
# ============================================================================

def _build_pallets(box_sequence: Sequence[Box], layout: Layout) -> list[Pallet]:
    """Create the Pallet objects and place the original boxes in sequence
    order. Pallet.place_box re-validates bounds, load and overlap."""
    pallets = [_create_new_pallet(number) for number in range(1, layout.pallet_count + 1)]
    for placed in layout.placements:
        pallets[placed.pallet_index].place_box(
            box_sequence[placed.box_index],
            Point3D(x=placed.x, y=placed.y, z=placed.z),
            placed.orientation,
            tolerance_mm=settings.PLACEMENT_TOLERANCE_MM,
        )
    return pallets


def _create_new_pallet(pallet_number: int) -> Pallet:
    pallet_id = (
        f"{settings.PALLET_ID_PREFIX}-"
        f"{pallet_number:0{settings.PALLET_ID_DIGITS}d}"
    )
    return Pallet(
        pallet_id=pallet_id,
        length_mm=settings.PALLET_LENGTH_MM,
        width_mm=settings.PALLET_WIDTH_MM,
        base_height_mm=settings.PALLET_BASE_HEIGHT_MM,
        max_height_mm=settings.PALLET_MAX_HEIGHT_MM,
        max_load_kg=settings.PALLET_MAX_LOAD_KG,
        name=settings.PALLET_NAME,
    )


# ============================================================================
# DECODER REGRESSION CHECK
# ============================================================================

def _verify_against_ems_loader(
    box_sequence: Sequence[Box],
    baseline_layout: Layout,
) -> bool | None:
    """Compare the decoder's baseline with emsLoader on copies of the boxes.

    Returns True (identical), False (different) or None (not comparable).
    """
    if getattr(emsLoader, "RANDOM_SEED", 0) is None:
        _log("decoder check skipped: emsLoader.RANDOM_SEED is None (not reproducible)")
        return None

    clones = copy.deepcopy(list(box_sequence))
    try:
        reference_pallets = emsLoader.load_boxes(clones)
    except Exception as exc:  # noqa: BLE001 - the check must never stop the run
        _log(f"decoder check skipped: emsLoader failed ({type(exc).__name__}: {exc})")
        return None

    reference: dict[str, tuple[int, object, Point3D | None]] = {}
    for pallet_index, pallet in enumerate(reference_pallets):
        for box in pallet.boxes:
            reference[box.box_id] = (pallet_index, box.orientation, box.position)

    different: list[str] = []
    for placed in baseline_layout.placements:
        box_id = box_sequence[placed.box_index].box_id
        ref = reference.get(box_id)
        same = (
            ref is not None
            and ref[2] is not None
            and ref[0] == placed.pallet_index
            and ref[1] == placed.orientation
            and abs(ref[2].x - placed.x) <= _VERIFY_POSITION_TOL_MM
            and abs(ref[2].y - placed.y) <= _VERIFY_POSITION_TOL_MM
            and abs(ref[2].z - placed.z) <= _VERIFY_POSITION_TOL_MM
        )
        if not same:
            different.append(box_id)

    if not different and len(reference_pallets) == baseline_layout.pallet_count:
        _log("decoder check: baseline is identical to emsLoader")
        return True

    _log(
        f"WARNING: decoder baseline differs from emsLoader "
        f"({len(different)} of {len(box_sequence)} boxes, first: "
        f"{different[0] if different else '-'}; pallets "
        f"{baseline_layout.pallet_count} vs {len(reference_pallets)}). "
        f"emsDecoder.py is out of sync with emsLoader.py; the optimizer "
        f"continues with the decoder's own behaviour."
    )
    return False


# ============================================================================
# VALIDATION
# ============================================================================

def _validate_boxes(box_sequence: Sequence[Box]) -> None:
    seen: set[str] = set()
    duplicates: set[str] = set()
    for box in box_sequence:
        if box.box_id in seen:
            duplicates.add(box.box_id)
        seen.add(box.box_id)
    if duplicates:
        joined = ", ".join(sorted(duplicates))
        raise LoaderOptimizerError(f"Duplicate box IDs supplied to loader: {joined}")

    already_placed = [box.box_id for box in box_sequence if box.is_placed]
    if already_placed:
        joined = ", ".join(already_placed)
        raise LoaderOptimizerError(f"Boxes are already placed: {joined}")


def _validate_settings() -> None:
    for name, value in (
        ("PALLET_LENGTH_MM", settings.PALLET_LENGTH_MM),
        ("PALLET_WIDTH_MM", settings.PALLET_WIDTH_MM),
        ("PALLET_BASE_HEIGHT_MM", settings.PALLET_BASE_HEIGHT_MM),
        ("PALLET_MAX_HEIGHT_MM", settings.PALLET_MAX_HEIGHT_MM),
    ):
        if not isfinite(value) or value <= 0:
            raise LoaderOptimizerError(f"settings.{name} must be positive and finite.")
    if settings.PALLET_MAX_LOAD_KG is not None and (
        not isfinite(settings.PALLET_MAX_LOAD_KG) or settings.PALLET_MAX_LOAD_KG <= 0
    ):
        raise LoaderOptimizerError("settings.PALLET_MAX_LOAD_KG must be positive or None.")

    if opt.TIME_LIMIT_S is not None and (
        not isfinite(opt.TIME_LIMIT_S) or opt.TIME_LIMIT_S <= 0
    ):
        raise LoaderOptimizerError("TIME_LIMIT_S must be positive or None.")
    if opt.N_WORKERS < 0:
        raise LoaderOptimizerError("N_WORKERS must be 0 (automatic) or positive.")
    if not isinstance(opt.RANDOM_SEED, int):
        raise LoaderOptimizerError("RANDOM_SEED must be an integer.")
    if opt.STABILITY_GATE not in GATE_MODES:
        raise LoaderOptimizerError(f"STABILITY_GATE must be one of {GATE_MODES}.")
    if opt.RUN_RULE_SWEEP and not (
        opt.SWEEP_BIN_RULES and opt.SWEEP_SPACE_RULES and opt.SWEEP_ORIENTATION_RULES
    ):
        raise LoaderOptimizerError("The SWEEP_* rule lists must not be empty.")
    if opt.SWEEP_REPORT_TOP < 1:
        raise LoaderOptimizerError("SWEEP_REPORT_TOP must be at least 1.")
    if opt.BRKGA_SEED_CONFIGS < 1:
        raise LoaderOptimizerError("BRKGA_SEED_CONFIGS must be at least 1.")
    for name, value in (
        ("ACCEL_FORWARD_M_S2", opt.ACCEL_FORWARD_M_S2),
        ("ACCEL_BRAKING_M_S2", opt.ACCEL_BRAKING_M_S2),
    ):
        if not isfinite(value) or value <= 0:
            raise LoaderOptimizerError(f"{name} must be positive and finite.")
    if opt.RUN_BRKGA:
        _brkga_params().validate()


# ============================================================================
# CONSOLE OUTPUT
# ============================================================================

def _log(message: str) -> None:
    if opt.VERBOSE:
        print(f"[optimizer] {message}", flush=True)


def _gate_text(gate: StabilityGate, params: EvalParams) -> str:
    unit = "load-tree" if params.stability_unit == "load_tree" else "box"
    if gate.mode == "off":
        return "off (stability is reported only)"
    if gate.mode == "spec":
        return f"spec ({unit} margin >= {gate.min_margin:g} for every unit)"
    text = (
        f"baseline ({unit} margin >= {gate.min_margin:.2f} and at most "
        f"{gate.max_unstable_units} units below spec)"
    )
    if opt.STABILITY_GATE == "auto":
        text += " - plain emsLoader does not meet the spec accelerations"
    return text


def _gate_failure_message(best: Trial, gate: StabilityGate, params: EvalParams) -> str:
    if best.metrics is not None and not best.metrics.geometry_ok:
        return "Internal error: the best solution failed the geometry check."
    unit = "load-tree" if params.stability_unit == "load_tree" else "box"
    margin = best.metrics.stability_margin if best.metrics is not None else float("nan")
    return (
        f"No solution passed the stability gate '{gate.mode}' "
        f"(required {unit} margin {gate.min_margin:g}, best found {margin:.2f}). "
        f"Set STABILITY_GATE to 'auto', 'baseline' or 'off' in "
        f"optimizerSettings.py to get a load anyway."
    )


def _log_result(report: OptimizerReport) -> None:
    best, baseline = report.best, report.baseline
    config = report.configs[best.config_index]
    if best.source == "baseline":
        origin = "baseline (no better solution found)"
    elif best.source == "sweep":
        origin = f"rule sweep, rule {config.label}"
    elif report.brkga_best_island is not None:
        origin = f"BRKGA island {report.brkga_best_island} on rule {config.label}"
    else:
        origin = f"BRKGA on rule {config.label}"

    _log(f"baseline: {format_metrics(baseline.metrics)}")
    _log(f"result  : {format_metrics(best.metrics)}")
    _log(f"source  : {origin}")
    if best.metrics is not None and baseline.metrics is not None:
        b, r = baseline.metrics, best.metrics
        _log(
            f"change  : pallets {b.pallet_count} -> {r.pallet_count}, "
            f"density {r.density_score - b.density_score:+.4f}, "
            f"CoG {r.cog_height_mm - b.cog_height_mm:+.0f} mm"
        )
    _log(
        f"{report.evaluations} evaluations in {report.elapsed_s:.1f} s "
        f"with {report.workers} worker process(es)"
    )