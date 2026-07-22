from __future__ import annotations

import argparse
import json
from collections import defaultdict
from pathlib import Path
from typing import Any

import torch


def load_pt_dataset(path: Path) -> list[list[list[int]]]:
    """
    Load and validate a dataset with this expected structure:

        dataset[sequence_index][box_index] = [length, width, height]
    """
    if not path.is_file():
        raise FileNotFoundError(f"Dataset not found: {path}")

    try:
        # The file contains normal Python lists rather than model weights.
        dataset = torch.load(
            path,
            map_location="cpu",
            weights_only=False,
        )
    except TypeError:
        # Compatibility with older PyTorch versions that do not support
        # the weights_only argument.
        dataset = torch.load(path, map_location="cpu")

    if not isinstance(dataset, list):
        raise TypeError(
            f"Expected the top-level object to be a list, "
            f"but received {type(dataset).__name__}"
        )

    for sequence_index, sequence in enumerate(dataset):
        if not isinstance(sequence, list):
            raise TypeError(
                f"Sequence {sequence_index} is not a list: "
                f"{type(sequence).__name__}"
            )

        for box_index, dimensions in enumerate(sequence):
            if not isinstance(dimensions, (list, tuple)):
                raise TypeError(
                    f"Box {box_index} in sequence {sequence_index} "
                    f"is not a list or tuple"
                )

            if len(dimensions) != 3:
                raise ValueError(
                    f"Box {box_index} in sequence {sequence_index} "
                    f"has {len(dimensions)} dimensions instead of 3: "
                    f"{dimensions!r}"
                )

            if not all(
                isinstance(value, int) and value > 0
                for value in dimensions
            ):
                raise ValueError(
                    f"Invalid dimensions at sequence {sequence_index}, "
                    f"box {box_index}: {dimensions!r}"
                )

    return dataset


def calculate_weight(
    dimensions_mm: list[int],
    fixed_weight_kg: float | None,
    density_kg_m3: float | None,
) -> float:
    """
    Determine a box's weight using either:

    1. One fixed weight for every box, or
    2. Box volume multiplied by a fixed density.
    """
    if fixed_weight_kg is not None:
        return round(fixed_weight_kg, 3)

    if density_kg_m3 is None:
        raise ValueError(
            "Either fixed_weight_kg or density_kg_m3 must be provided"
        )

    length_mm, width_mm, height_mm = dimensions_mm

    volume_m3 = (
        length_mm * width_mm * height_mm
    ) / 1_000_000_000

    return round(volume_m3 * density_kg_m3, 3)


def convert_sequence(
    sequence: list[list[int]],
    sequence_index: int,
    grid_unit_mm: int,
    fixed_weight_kg: float | None,
    density_kg_m3: float | None,
) -> dict[str, Any]:
    """
    Convert one packing sequence into the requested JSON structure.

    Boxes with identical dimensions receive the same generated SKU.
    """
    sku_counts: dict[str, int] = defaultdict(int)
    boxes: list[dict[str, Any]] = []

    for grid_dimensions in sequence:
        grid_length, grid_width, grid_height = grid_dimensions

        dimensions_mm = [
            grid_length * grid_unit_mm,
            grid_width * grid_unit_mm,
            grid_height * grid_unit_mm,
        ]

        # The source dataset has no product names. Therefore, dimensions
        # are used to create a deterministic synthetic SKU.
        sku = (
            f"BOX-{grid_length}X{grid_width}X{grid_height}"
        )

        sku_counts[sku] += 1

        identifier = (
            f"{sku}-"
            f"S{sequence_index:04d}-"
            f"{sku_counts[sku]:03d}"
        )

        boxes.append(
            {
                "identifier": identifier,
                "sku": sku,
                "dimensions_mm": dimensions_mm,
                "weight_kg": calculate_weight(
                    dimensions_mm=dimensions_mm,
                    fixed_weight_kg=fixed_weight_kg,
                    density_kg_m3=density_kg_m3,
                ),
            }
        )

    return {
        "format_version": 1,
        "units": {
            "dimensions": "mm",
            "weight": "kg",
        },
        "boxes": boxes,
    }


def write_sequence(
    sequence: list[list[int]],
    sequence_index: int,
    dataset_name: str,
    output_directory: Path,
    grid_unit_mm: int,
    fixed_weight_kg: float | None,
    density_kg_m3: float | None,
) -> Path:
    converted = convert_sequence(
        sequence=sequence,
        sequence_index=sequence_index,
        grid_unit_mm=grid_unit_mm,
        fixed_weight_kg=fixed_weight_kg,
        density_kg_m3=density_kg_m3,
    )

    output_path = (
        output_directory
        / f"{dataset_name}_sequence_{sequence_index:04d}.json"
    )

    with output_path.open("w", encoding="utf-8") as output_file:
        json.dump(
            converted,
            output_file,
            indent=2,
            ensure_ascii=False,
        )
        output_file.write("\n")

    return output_path


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Convert the Online 3D-BPP .pt dataset into "
            "one pallet-stacker JSON file per sequence."
        )
    )

    parser.add_argument(
        "input",
        type=Path,
        help="Path to the input .pt dataset",
    )
    parser.add_argument(
        "output_directory",
        type=Path,
        help="Directory in which JSON files will be written",
    )
    parser.add_argument(
        "--grid-unit-mm",
        type=int,
        required=True,
        help=(
            "Number of millimetres represented by one dataset grid unit. "
            "For example, 100 converts [4, 4, 2] to [400, 400, 200]."
        ),
    )
    parser.add_argument(
        "--sequence-index",
        type=int,
        help=(
            "Convert only this sequence. "
            "By default, all sequences are converted."
        ),
    )

    weight_group = parser.add_mutually_exclusive_group(required=True)

    weight_group.add_argument(
        "--fixed-weight-kg",
        type=float,
        help="Assign this weight to every box",
    )
    weight_group.add_argument(
        "--density-kg-m3",
        type=float,
        help=(
            "Calculate weight as box volume multiplied by this density"
        ),
    )

    arguments = parser.parse_args()

    if arguments.grid_unit_mm <= 0:
        parser.error("--grid-unit-mm must be greater than zero")

    if (
        arguments.fixed_weight_kg is not None
        and arguments.fixed_weight_kg < 0
    ):
        parser.error("--fixed-weight-kg cannot be negative")

    if (
        arguments.density_kg_m3 is not None
        and arguments.density_kg_m3 < 0
    ):
        parser.error("--density-kg-m3 cannot be negative")

    dataset = load_pt_dataset(arguments.input)

    arguments.output_directory.mkdir(
        parents=True,
        exist_ok=True,
    )

    if arguments.sequence_index is not None:
        sequence_index = arguments.sequence_index

        if not 0 <= sequence_index < len(dataset):
            parser.error(
                f"--sequence-index must be between 0 "
                f"and {len(dataset) - 1}"
            )

        indices = [sequence_index]
    else:
        indices = range(len(dataset))

    for sequence_index in indices:
        output_path = write_sequence(
            sequence=dataset[sequence_index],
            sequence_index=sequence_index,
            dataset_name=arguments.input.stem,
            output_directory=arguments.output_directory,
            grid_unit_mm=arguments.grid_unit_mm,
            fixed_weight_kg=arguments.fixed_weight_kg,
            density_kg_m3=arguments.density_kg_m3,
        )

        print(
            f"Wrote sequence {sequence_index}: "
            f"{len(dataset[sequence_index])} boxes -> {output_path}"
        )


if __name__ == "__main__":
    main()