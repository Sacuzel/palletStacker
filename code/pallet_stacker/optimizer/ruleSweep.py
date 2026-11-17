"""Rule sweep, Stage 1 of the optimizer.

The EMS loader of Ali, Ramos & Oliveira (2025) is built from three
independent rules: bin selection (4), space selection (8) and orientation
selection (5). This module lists every combination of the rules enabled in
optimizerSettings.py and ranks the resulting loads. Every combination runs
with default decisions, i.e. exactly as emsLoader would run with those rule
settings.

Only the three selection rules (and optionally N_EMS_CANDIDATES) vary. The
stability constraint, the number of open pallets and all tolerances are
physical inputs and stay as configured in emsLoader.

Outputs
-------
* a ranked list of Trials (baseline included)
* a text table for the log
The best combinations seed the BRKGA of Stage 2.
"""

from __future__ import annotations

import time
from collections.abc import Callable, Sequence
from dataclasses import replace

from .emsDecoder import DecoderConfig
from .evaluator import Trial

# Orientation rules that look at more than one EMS (Table 2, rules 4 and 5).
_MULTI_EMS_ORIENTATION_RULES: tuple[int, ...] = (4, 5)


def build_sweep_configs(
    base: DecoderConfig,
    bin_rules: Sequence[str],
    space_rules: Sequence[int],
    orientation_rules: Sequence[int],
    n_ems_values: Sequence[int] = (),
) -> list[DecoderConfig]:
    """All rule combinations to try. Index 0 is always the baseline.

    n_ems_values  values of N_EMS_CANDIDATES to try for orientation rules 4
                  and 5 (the only rules that use it). Empty = keep the
                  baseline value.
    """
    configs: list[DecoderConfig] = [base]
    seen = {_signature(base)}

    for bin_rule in bin_rules:
        for space_rule in space_rules:
            for orientation_rule in orientation_rules:
                if orientation_rule in _MULTI_EMS_ORIENTATION_RULES and n_ems_values:
                    candidates_values: Sequence[int] = n_ems_values
                else:
                    candidates_values = (base.n_ems_candidates,)
                for n_ems in candidates_values:
                    cfg = replace(
                        base,
                        bin_rule=bin_rule,
                        space_rule=space_rule,
                        orientation_rule=orientation_rule,
                        n_ems_candidates=n_ems,
                    )
                    cfg.validate()
                    signature = _signature(cfg)
                    if signature not in seen:
                        seen.add(signature)
                        configs.append(cfg)
    return configs


def run_rule_sweep(
    baseline: Trial,
    config_count: int,
    evaluate: Callable[[Sequence[int]], Sequence[Trial]],
    *,
    batch_size: int,
    deadline: float | None = None,
) -> tuple[list[Trial], bool]:
    """Evaluate configurations 1 .. config_count-1 and rank them.

    baseline    the already evaluated Trial of configuration 0
    evaluate    scores a batch of configuration indices, results in order
    batch_size  configurations per batch; the deadline is checked in between

    Returns (trials ranked best first, True when every configuration ran).
    The baseline wins ties because it is first in the list (stable sort).
    """
    trials: list[Trial] = [baseline]
    pending = list(range(1, config_count))
    completed = True

    while pending:
        if deadline is not None and time.monotonic() >= deadline:
            completed = False
            break
        batch, pending = pending[:batch_size], pending[batch_size:]
        trials.extend(evaluate(batch))

    return sorted(trials, key=lambda trial: trial.key), completed


def format_sweep_table(
    ranked: Sequence[Trial],
    configs: Sequence[DecoderConfig],
    *,
    top: int,
) -> list[str]:
    """Text table of the best `top` combinations; the baseline is always shown."""
    header = (
        f"{'rank':>4}  {'rule':<4} {'nEMS':>4}  {'pallets':>7}  {'density':>7}  "
        f"{'SSF':>6}  {'envelope':>8}  {'CoG mm':>6}  {'margin':>6}  "
        f"{'below spec':>10}  gate"
    )
    lines = [header, "-" * len(header)]

    for rank, trial in enumerate(ranked, start=1):
        is_baseline = trial.config_index == 0
        if rank > top and not is_baseline:
            continue
        if rank > top + 1 and is_baseline:
            lines.append(f"{'...':>4}")
        cfg = configs[trial.config_index]
        n_ems = (
            str(cfg.n_ems_candidates)
            if cfg.orientation_rule in _MULTI_EMS_ORIENTATION_RULES
            else "-"
        )
        m = trial.metrics
        if m is None:
            lines.append(f"{rank:>4}  {cfg.label:<4} {n_ems:>4}  decoder failed")
            continue
        line = (
            f"{rank:>4}  {cfg.label:<4} {n_ems:>4}  {m.pallet_count:>7}  "
            f"{m.density_score:>7.4f}  {m.ssf:>6.4f}  "
            f"{m.envelope_utilization:>8.1%}  {m.cog_height_mm:>6.0f}  "
            f"{m.stability_margin:>6.2f}  {m.unstable_units:>10}  "
            f"{'ok' if trial.passed else 'FAIL'}"
        )
        lines.append(line + ("   <- baseline" if is_baseline else ""))
    return lines


def _signature(cfg: DecoderConfig) -> tuple[str, int, int, int]:
    """Rule combinations with the same signature produce the same load."""
    n_ems = (
        cfg.n_ems_candidates
        if cfg.orientation_rule in _MULTI_EMS_ORIENTATION_RULES
        else 0
    )
    return (cfg.bin_rule, cfg.space_rule, cfg.orientation_rule, n_ems)