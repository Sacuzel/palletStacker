"""Biased random-key genetic algorithm (BRKGA), Stage 2 of the optimizer.

Reference
---------
J. F. Goncalves, M. G. C. Resende: "A parallel multi-population biased
random-key genetic algorithm for a container loading problem", Computers &
Operations Research 39(2) (2012) 179-190.

This module contains only the genetic algorithm. It knows nothing about
pallets: a chromosome is a list of random keys in [0, 1), and the caller
supplies the function that decodes and scores a batch of chromosomes.

One generation (single population)
----------------------------------
1. Sort the population by fitness (lower key = better).
2. Copy the elite (best fraction) unchanged into the next generation.
3. Add mutants: completely new random chromosomes.
4. Fill the rest with offspring. Each child has one elite parent and one
   non-elite parent, and takes every gene from the elite parent with
   probability elite_inheritance (parametrized uniform crossover).
5. Score only the new chromosomes; the elite keep their scores.

Stopping (window method)
------------------------
The best solution of generation g is compared with the best solution of
generation g - stall_window. If it is not meaningfully better (see
Scored.stall_key), the run stops. A generation limit and an optional
wall-clock deadline act as safety stops.

Reproducibility
---------------
All randomness comes from the random.Random instance passed in. With the
same seed, the same evaluate function and a stop by window or generation
limit, the result is identical. A stop by deadline depends on machine speed.
"""

from __future__ import annotations

import random
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from typing import Protocol

Chromosome = list[float]


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
    population: int = 40
    elite_fraction: float = 0.20
    mutant_fraction: float = 0.15
    elite_inheritance: float = 0.70
    max_generations: int = 500
    stall_window: int = 25

    def validate(self) -> None:
        if self.population < 4:
            raise BrkgaError("population must be at least 4.")
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
        elite, mutants = self.counts()
        if elite + mutants >= self.population:
            raise BrkgaError(
                "elite_fraction and mutant_fraction leave no room for offspring."
            )

    def counts(self) -> tuple[int, int]:
        """Number of elite and mutant chromosomes per generation."""
        elite = max(1, round(self.elite_fraction * self.population))
        mutants = round(self.mutant_fraction * self.population)
        return elite, mutants


@dataclass(slots=True)
class BrkgaResult:
    best_chromosome: Chromosome
    best: Scored
    generations: int                 # generations completed after generation 0
    evaluations: int
    stop_reason: str                 # "window", "max_generations" or "time_limit"
    history: list[Scored] = field(default_factory=list)   # best of each generation


def run_brkga(
    n_genes: int,
    evaluate: Callable[[Sequence[Chromosome]], Sequence[Scored]],
    params: BrkgaParams,
    rng: random.Random,
    *,
    seeds: Sequence[Chromosome] = (),
    deadline: float | None = None,
    on_generation: Callable[[int, Scored, int], None] | None = None,
) -> BrkgaResult:
    """Run the genetic algorithm and return the best chromosome found.

    n_genes        chromosome length
    evaluate       scores a batch of chromosomes, returns results in order
    seeds          chromosomes placed in the initial population (e.g. the
                   default solutions of the best rule combinations)
    deadline       time.monotonic() value after which no generation starts
    on_generation  callback(generation, best, evaluations) for progress output
    """
    params.validate()
    if n_genes < 1:
        raise BrkgaError("n_genes must be at least 1.")
    for seed in seeds:
        if len(seed) != n_genes:
            raise BrkgaError("Every seed chromosome must have n_genes genes.")

    size = params.population
    n_elite, n_mutants = params.counts()
    n_offspring = size - n_elite - n_mutants

    # ---- generation 0: seeds plus random chromosomes ------------------------
    chromosomes: list[Chromosome] = [list(seed) for seed in seeds[:size]]
    while len(chromosomes) < size:
        chromosomes.append(_random_chromosome(n_genes, rng))
    scores = list(evaluate(chromosomes))
    evaluations = len(chromosomes)
    population = _sorted_population(chromosomes, scores)

    history: list[Scored] = [population[0][1]]
    if on_generation is not None:
        on_generation(0, population[0][1], evaluations)

    stop_reason = "max_generations"
    generation = 0
    while generation < params.max_generations:
        if deadline is not None and time.monotonic() >= deadline:
            stop_reason = "time_limit"
            break
        generation += 1

        elite = population[:n_elite]
        non_elite = population[n_elite:]

        # ---- new chromosomes: mutants and offspring -------------------------
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

        new_scores = list(evaluate(new))
        evaluations += len(new)

        # Elite first: with equal keys the stable sort keeps the older one.
        population = _sorted_population(
            [c for c, _ in elite] + new,
            [s for _, s in elite] + new_scores,
        )
        best = population[0][1]
        history.append(best)
        if on_generation is not None:
            on_generation(generation, best, evaluations)

        # ---- window stopping criterion ---------------------------------------
        window = params.stall_window
        if generation >= window:
            if not best.stall_key < history[generation - window].stall_key:
                stop_reason = "window"
                break

    return BrkgaResult(
        best_chromosome=population[0][0],
        best=population[0][1],
        generations=generation,
        evaluations=evaluations,
        stop_reason=stop_reason,
        history=history,
    )


def _random_chromosome(n_genes: int, rng: random.Random) -> Chromosome:
    return [rng.random() for _ in range(n_genes)]


def _sorted_population(
    chromosomes: Sequence[Chromosome],
    scores: Sequence[Scored],
) -> list[tuple[Chromosome, Scored]]:
    if len(chromosomes) != len(scores):
        raise BrkgaError("evaluate() must return one result per chromosome.")
    return sorted(
        zip(chromosomes, scores, strict=True),
        key=lambda pair: pair[1].key,
    )