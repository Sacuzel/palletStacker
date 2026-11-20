"""Biased random-key genetic algorithm (BRKGA), Stage 2 of the optimizer.

Reference
---------
J. F. Goncalves, M. G. C. Resende: "A parallel multi-population biased
random-key genetic algorithm for a container loading problem", Computers &
Operations Research 39(2) (2012) 179-190.

This module contains only the genetic algorithm. It knows nothing about
pallets: a chromosome is a list of random keys in [0, 1), and the caller
supplies the function that decodes and scores a batch of chromosomes.

One generation (one population)
-------------------------------
1. Sort the population by fitness (lower key = better).
2. Copy the elite (best fraction) unchanged into the next generation.
3. Add mutants: completely new random chromosomes.
4. Fill the rest with offspring. Each child has one elite parent and one
   non-elite parent, and takes every gene from the elite parent with
   probability elite_inheritance (parametrized uniform crossover).
5. Score only the new chromosomes; the elite keep their scores.

Stall detection (window method)
-------------------------------
The best solution of generation g is compared with the best solution of
generation g - stall_window. If it is not meaningfully better (see
Scored.stall_key), the search has stalled.

Genetic extension (BrkgaParams.extension)
-----------------------------------------
"none"      One population. The first stall stops the run.

"ladder"    One population. A stall does not stop the run at once; the next
            generation is replaced by an escalation step instead:
              shake    The best chromosome stays unchanged. Every other
                       member is a copy of a random elite member with one
                       contiguous block of genes redrawn. Inside the block a
                       gene becomes an active key (>= shake_threshold) with
                       probability shake_intensity, otherwise an inactive one.
              restart  The best chromosome stays unchanged. The seeds
                       return, the rest is new random chromosomes.
            Order: shakes_per_restart shakes, then one restart, repeated.
            The run stops when it stalls again after max_restarts restarts
            without a meaningful improvement. A meaningful improvement resets
            the ladder. After every step the stall window starts again.
            Escalation steps count as generations.

"islands"   Several populations (islands) of island_population chromosomes
            evolve side by side, island i with its own random stream
            (seed + i). Every island starts with all seeds. All new
            chromosomes of one generation are scored in one batch. Every
            migration_interval generations, the best `migrants` chromosomes
            of each island replace the worst of the next island (ring);
            migration_interval = 0 keeps the islands independent (parallel
            multi-start). The stall window watches the best solution over all
            islands; the first stall stops the run.

The best solution is never lost in any extension.

Reproducibility
---------------
All randomness comes from `seed`. With the same seed, the same evaluate
function and a stop by window or generation limit, the result is identical.
A stop by deadline depends on machine speed. "islands" with one island,
island_population = population and no migration gives exactly the "none"
result.
"""

from __future__ import annotations

import random
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from typing import Protocol

Chromosome = list[float]
Member = tuple[Chromosome, "Scored"]

GENETIC_EXTENSIONS: tuple[str, ...] = ("none", "ladder", "islands")

# Kinds of generation in the ladder (passed to on_escalation).
_NORMAL = "normal"
SHAKE = "shake"
RESTART = "restart"


class Scored(Protocol):
    """What the evaluate function must return for each chromosome."""

    @property
    def key(self) -> tuple:
        """Full ranking key; lower is better."""
        ...

    @property
    def stall_key(self) -> tuple:
        """Coarse ranking key; a lower value is a meaningful improvement."""
        ...


class BrkgaError(RuntimeError):
    """Raised for invalid BRKGA parameters."""


@dataclass(frozen=True, slots=True)
class BrkgaParams:
    population: int = 40             # "none" and "ladder"
    elite_fraction: float = 0.20
    mutant_fraction: float = 0.15
    elite_inheritance: float = 0.70
    max_generations: int = 500
    stall_window: int = 25
    extension: str = "none"          # one of GENETIC_EXTENSIONS
    # "ladder"
    shakes_per_restart: int = 1      # shakes tried before each restart
    max_restarts: int = 2            # restarts without improvement before stopping
    shake_block_genes: int = 75      # length of the redrawn gene block
    shake_intensity: float = 0.25    # P(active key) for a redrawn gene
    shake_threshold: float = 0.0     # keys >= this are active for the decoder
    # "islands"
    islands: int = 3                 # number of populations
    island_population: int = 35      # chromosomes per island
    migration_interval: int = 20     # generations between migrations; 0 = never
    migrants: int = 2                # chromosomes copied per island and migration

    def validate(self) -> None:
        """Check the common parameters and those of the active extension."""
        if not 0.0 < self.elite_fraction < 1.0:
            raise BrkgaError("elite_fraction must be within (0, 1).")
        if not 0.0 <= self.mutant_fraction < 1.0:
            raise BrkgaError("mutant_fraction must be within [0, 1).")
        if not 0.5 <= self.elite_inheritance <= 1.0:
            raise BrkgaError("elite_inheritance must be within [0.5, 1].")
        if self.max_generations < 0:
            raise BrkgaError("max_generations must be non-negative.")
        if self.stall_window < 1:
            raise BrkgaError("stall_window must be at least 1.")
        if self.extension not in GENETIC_EXTENSIONS:
            raise BrkgaError(f"extension must be one of {GENETIC_EXTENSIONS}.")

        if self.extension == "islands":
            self._check_size(self.island_population, "island_population")
            if self.islands < 1:
                raise BrkgaError("islands must be at least 1.")
            if self.migration_interval < 0:
                raise BrkgaError("migration_interval must be non-negative.")
            if not 1 <= self.migrants < self.island_population:
                raise BrkgaError("migrants must be within [1, island_population).")
            return

        self._check_size(self.population, "population")
        if self.extension == "ladder":
            if self.shakes_per_restart < 0:
                raise BrkgaError("shakes_per_restart must be non-negative.")
            if self.max_restarts < 0:
                raise BrkgaError("max_restarts must be non-negative.")
            if self.shake_block_genes < 1:
                raise BrkgaError("shake_block_genes must be at least 1.")
            if not 0.0 < self.shake_intensity <= 1.0:
                raise BrkgaError("shake_intensity must be within (0, 1].")
            if not 0.0 <= self.shake_threshold <= 1.0:
                raise BrkgaError("shake_threshold must be within [0, 1].")

    def counts(self, size: int | None = None) -> tuple[int, int]:
        """Number of elite and mutant chromosomes in a population of `size`
        (default: `population`)."""
        n = self.population if size is None else size
        elite = max(1, round(self.elite_fraction * n))
        mutants = round(self.mutant_fraction * n)
        return elite, mutants

    def _check_size(self, size: int, name: str) -> None:
        if size < 4:
            raise BrkgaError(f"{name} must be at least 4.")
        elite, mutants = self.counts(size)
        if elite + mutants >= size:
            raise BrkgaError(
                f"elite_fraction and mutant_fraction leave no room for "
                f"offspring in a {name} of {size}."
            )


@dataclass(slots=True)
class BrkgaResult:
    best_chromosome: Chromosome
    best: Scored
    generations: int                 # generations completed after generation 0
    evaluations: int
    stop_reason: str                 # "window", "max_generations" or "time_limit"
    history: list[Scored] = field(default_factory=list)   # best of each generation
    extension: str = "none"
    shakes: int = 0                  # "ladder": shake steps in the whole run
    restarts: int = 0                # "ladder": restart steps in the whole run
    best_island: int = 0             # "islands": island that found the best


def run_brkga(
    n_genes: int,
    evaluate: Callable[[Sequence[Chromosome]], Sequence[Scored]],
    params: BrkgaParams,
    seed: int,
    *,
    seeds: Sequence[Chromosome] = (),
    fixed_genes: int = 0,
    deadline: float | None = None,
    on_generation: Callable[[int, Scored, int, int], None] | None = None,
    on_escalation: Callable[[int, str], None] | None = None,
) -> BrkgaResult:
    """Run the genetic algorithm and return the best chromosome found.

    n_genes        chromosome length
    evaluate       scores a batch of chromosomes, returns results in order
    params         see BrkgaParams; params.extension selects the extension
    seed           random seed ("islands": island i uses seed + i)
    seeds          chromosomes placed in the initial population (e.g. the
                   default solutions of the best rule combinations); every
                   island gets all of them, a ladder restart brings them back
    fixed_genes    leading genes a shake never redraws (e.g. a gene that
                   selects the rule combination)
    deadline       time.monotonic() value after which no generation starts
    on_generation  callback(generation, best, evaluations, island) for
                   progress output; island = island holding the best (0 for
                   one population)
    on_escalation  "ladder" only: callback(generation, "shake" | "restart")
                   when the window fires and the next generation becomes an
                   escalation step
    """
    params.validate()
    if n_genes < 1:
        raise BrkgaError("n_genes must be at least 1.")
    if not 0 <= fixed_genes < n_genes:
        raise BrkgaError("fixed_genes must be within [0, n_genes).")
    for chromosome in seeds:
        if len(chromosome) != n_genes:
            raise BrkgaError("Every seed chromosome must have n_genes genes.")

    if params.extension == "islands":
        return _run_islands(
            n_genes, evaluate, params, seed, seeds, deadline, on_generation
        )
    return _run_single(
        n_genes, evaluate, params, seed, seeds, fixed_genes, deadline,
        on_generation, on_escalation,
    )


# ============================================================================
# ONE POPULATION ("none" and "ladder")
# ============================================================================

def _run_single(
    n_genes: int,
    evaluate: Callable[[Sequence[Chromosome]], Sequence[Scored]],
    params: BrkgaParams,
    seed: int,
    seeds: Sequence[Chromosome],
    fixed_genes: int,
    deadline: float | None,
    on_generation: Callable[[int, Scored, int, int], None] | None,
    on_escalation: Callable[[int, str], None] | None,
) -> BrkgaResult:
    rng = random.Random(seed)
    size = params.population
    n_elite, n_mutants = params.counts(size)
    ladder = params.extension == "ladder"

    # ---- generation 0: seeds plus random chromosomes ------------------------
    chromosomes = _initial_chromosomes(n_genes, size, seeds, rng)
    scores = list(evaluate(chromosomes))
    evaluations = len(chromosomes)
    population = _sorted_population(chromosomes, scores)

    history: list[Scored] = [population[0][1]]
    if on_generation is not None:
        on_generation(0, population[0][1], evaluations, 0)

    # ---- escalation ladder state ("ladder" only) ------------------------------
    pending = _NORMAL                   # kind of the next generation
    window_floor = 0                    # stall window starts here (last escalation)
    escalation_key: tuple | None = None # best stall_key at the last escalation
    shakes_done = 0                     # since the last restart or improvement
    restarts_done = 0                   # since the last improvement
    total_shakes = 0
    total_restarts = 0

    stop_reason = "max_generations"
    generation = 0
    while generation < params.max_generations:
        if deadline is not None and time.monotonic() >= deadline:
            stop_reason = "time_limit"
            break
        generation += 1

        # ---- build the new chromosomes ------------------------------------------
        if pending == SHAKE:
            elite = population[:n_elite]
            kept = population[:1]                       # the best, unchanged
            new = [
                _shaken_copy(rng.choice(elite)[0], fixed_genes, params, rng)
                for _ in range(size - 1)
            ]
        elif pending == RESTART:
            kept = population[:1]                       # the best, unchanged
            new = [list(chromosome) for chromosome in seeds[: size - 1]]
            while len(new) < size - 1:
                new.append(_random_chromosome(n_genes, rng))
        else:
            kept, new = _breed(population, n_genes, n_elite, n_mutants, params, rng)
        pending = _NORMAL

        new_scores = list(evaluate(new))
        evaluations += len(new)
        population = _merge(kept, new, new_scores)
        best = population[0][1]
        history.append(best)

        # A meaningful improvement since the last escalation resets the ladder.
        if escalation_key is not None and best.stall_key < escalation_key:
            escalation_key = None
            shakes_done = 0
            restarts_done = 0

        if on_generation is not None:
            on_generation(generation, best, evaluations, 0)

        # ---- window stall check -----------------------------------------------
        if _stalled(history, generation, params.stall_window, window_floor):
            if ladder and shakes_done < params.shakes_per_restart:
                pending = SHAKE
                shakes_done += 1
                total_shakes += 1
            elif ladder and restarts_done < params.max_restarts:
                pending = RESTART
                restarts_done += 1
                total_restarts += 1
                shakes_done = 0
            else:
                stop_reason = "window"
                break
            escalation_key = best.stall_key
            window_floor = generation
            if on_escalation is not None:
                on_escalation(generation, pending)

    return BrkgaResult(
        best_chromosome=population[0][0],
        best=population[0][1],
        generations=generation,
        evaluations=evaluations,
        stop_reason=stop_reason,
        history=history,
        extension=params.extension,
        shakes=total_shakes,
        restarts=total_restarts,
    )


# ============================================================================
# SEVERAL POPULATIONS ("islands")
# ============================================================================

def _run_islands(
    n_genes: int,
    evaluate: Callable[[Sequence[Chromosome]], Sequence[Scored]],
    params: BrkgaParams,
    seed: int,
    seeds: Sequence[Chromosome],
    deadline: float | None,
    on_generation: Callable[[int, Scored, int, int], None] | None,
) -> BrkgaResult:
    n_islands = params.islands
    size = params.island_population
    n_elite, n_mutants = params.counts(size)
    rngs = [random.Random(seed + i) for i in range(n_islands)]

    # ---- generation 0: every island gets all seeds; seeds are scored once ----
    seed_list = [list(chromosome) for chromosome in seeds[:size]]
    randoms = [
        _initial_chromosomes(n_genes, size, seed_list, rng)[len(seed_list):]
        for rng in rngs
    ]
    batch = seed_list + [c for island in randoms for c in island]
    scores = list(evaluate(batch))
    evaluations = len(batch)
    seed_scores = scores[: len(seed_list)]
    offset = len(seed_list)
    populations: list[list[Member]] = []
    for island in randoms:
        island_scores = scores[offset: offset + len(island)]
        offset += len(island)
        populations.append(
            _sorted_population(
                [list(c) for c in seed_list] + island, seed_scores + island_scores
            )
        )

    best_island = _best_island(populations)
    best_chromosome, best = populations[best_island][0]
    history: list[Scored] = [best]
    if on_generation is not None:
        on_generation(0, best, evaluations, best_island)

    stop_reason = "max_generations"
    generation = 0
    while generation < params.max_generations:
        if deadline is not None and time.monotonic() >= deadline:
            stop_reason = "time_limit"
            break
        generation += 1

        # ---- breed every island, score all new chromosomes in one batch -------
        parts = [
            _breed(population, n_genes, n_elite, n_mutants, params, rng)
            for population, rng in zip(populations, rngs, strict=True)
        ]
        batch = [c for _, new in parts for c in new]
        scores = list(evaluate(batch))
        evaluations += len(batch)
        offset = 0
        for i, (kept, new) in enumerate(parts):
            populations[i] = _merge(kept, new, scores[offset: offset + len(new)])
            offset += len(new)

        # ---- migration (ring) ------------------------------------------------------
        interval = params.migration_interval
        if interval > 0 and n_islands > 1 and generation % interval == 0:
            _migrate(populations, params.migrants)

        # ---- global best: a copy made by migration never replaces its origin ----
        island = _best_island(populations)
        candidate_chromosome, candidate = populations[island][0]
        if candidate.key < best.key:
            best_chromosome, best, best_island = candidate_chromosome, candidate, island
        history.append(best)
        if on_generation is not None:
            on_generation(generation, best, evaluations, best_island)

        # ---- window stall check on the global best ------------------------------
        if _stalled(history, generation, params.stall_window, 0):
            stop_reason = "window"
            break

    return BrkgaResult(
        best_chromosome=best_chromosome,
        best=best,
        generations=generation,
        evaluations=evaluations,
        stop_reason=stop_reason,
        history=history,
        extension=params.extension,
        best_island=best_island,
    )


def _best_island(populations: Sequence[Sequence[Member]]) -> int:
    """Index of the island holding the best chromosome (ties: lowest index)."""
    best = 0
    for i in range(1, len(populations)):
        if populations[i][0][1].key < populations[best][0][1].key:
            best = i
    return best


def _migrate(populations: list[list[Member]], migrants: int) -> None:
    """Ring migration: the best `migrants` of island i replace the worst
    `migrants` of island i + 1. Migrants keep their scores."""
    emigrants = [population[:migrants] for population in populations]
    n = len(populations)
    for i in range(n):
        target = (i + 1) % n
        survivors = populations[target][: len(populations[target]) - migrants]
        # Residents first: with equal keys the stable sort keeps them ahead.
        populations[target] = sorted(
            survivors + emigrants[i], key=lambda member: member[1].key
        )


# ============================================================================
# SHARED BUILDING BLOCKS
# ============================================================================

def _initial_chromosomes(
    n_genes: int,
    size: int,
    seeds: Sequence[Chromosome],
    rng: random.Random,
) -> list[Chromosome]:
    """Seeds first, then random chromosomes up to `size`."""
    chromosomes: list[Chromosome] = [list(chromosome) for chromosome in seeds[:size]]
    while len(chromosomes) < size:
        chromosomes.append(_random_chromosome(n_genes, rng))
    return chromosomes


def _breed(
    population: Sequence[Member],
    n_genes: int,
    n_elite: int,
    n_mutants: int,
    params: BrkgaParams,
    rng: random.Random,
) -> tuple[list[Member], list[Chromosome]]:
    """One normal generation: (kept elite members, new chromosomes)."""
    n_offspring = len(population) - n_elite - n_mutants
    elite = list(population[:n_elite])
    non_elite = population[n_elite:]
    new: list[Chromosome] = [
        _random_chromosome(n_genes, rng) for _ in range(n_mutants)
    ]
    bias = params.elite_inheritance
    for _ in range(n_offspring):
        elite_parent = rng.choice(elite)[0]
        other_parent = rng.choice(non_elite)[0]
        new.append(
            [
                e if rng.random() < bias else o
                for e, o in zip(elite_parent, other_parent, strict=True)
            ]
        )
    return elite, new


def _merge(
    kept: Sequence[Member],
    new: Sequence[Chromosome],
    new_scores: Sequence[Scored],
) -> list[Member]:
    """Next population, sorted. Kept members first: with equal keys the
    stable sort keeps the older one."""
    return _sorted_population(
        [c for c, _ in kept] + list(new),
        [s for _, s in kept] + list(new_scores),
    )


def _stalled(
    history: Sequence[Scored],
    generation: int,
    window: int,
    window_floor: int,
) -> bool:
    """True when the best of `generation` is not meaningfully better than
    the best `window` generations earlier (never before window_floor)."""
    if generation - window < window_floor:
        return False
    return not history[generation].stall_key < history[generation - window].stall_key


def _random_chromosome(n_genes: int, rng: random.Random) -> Chromosome:
    return [rng.random() for _ in range(n_genes)]


def _shaken_copy(
    parent: Chromosome,
    fixed_genes: int,
    params: BrkgaParams,
    rng: random.Random,
) -> Chromosome:
    """Copy of parent with one contiguous block of free genes redrawn."""
    child = list(parent)
    free = len(child) - fixed_genes
    block = min(params.shake_block_genes, free)
    start = fixed_genes + rng.randrange(free - block + 1)
    low = params.shake_threshold
    for i in range(start, start + block):
        if low < 1.0 and rng.random() < params.shake_intensity:
            child[i] = low + (1.0 - low) * rng.random()     # active key
        else:
            child[i] = low * rng.random()                   # inactive key
    return child


def _sorted_population(
    chromosomes: Sequence[Chromosome],
    scores: Sequence[Scored],
) -> list[Member]:
    if len(chromosomes) != len(scores):
        raise BrkgaError("evaluate() must return one result per chromosome.")
    return sorted(
        zip(chromosomes, scores, strict=True),
        key=lambda pair: pair[1].key,
    )