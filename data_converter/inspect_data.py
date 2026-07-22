from pathlib import Path
from typing import Any

import torch


DATASET_PATH = Path(
    "Online-3D-BPP-DRL/dataset/cut_2.pt"
)


def describe(
    value: Any,
    name: str = "root",
    depth: int = 0,
    max_depth: int = 4,
    max_elements: int = 5,
) -> None:
    """Recursively print the structure without dumping the whole dataset."""
    indentation = "  " * depth
    value_type = type(value).__name__

    if isinstance(value, torch.Tensor):
        print(
            f"{indentation}{name}: Tensor("
            f"shape={tuple(value.shape)}, "
            f"dtype={value.dtype}, "
            f"device={value.device})"
        )
        return

    if isinstance(value, dict):
        print(
            f"{indentation}{name}: dict("
            f"length={len(value)}, "
            f"keys={list(value.keys())[:max_elements]})"
        )

        if depth < max_depth:
            for key, child in list(value.items())[:max_elements]:
                describe(
                    child,
                    name=f"[{key!r}]",
                    depth=depth + 1,
                    max_depth=max_depth,
                    max_elements=max_elements,
                )
        return

    if isinstance(value, (list, tuple)):
        print(
            f"{indentation}{name}: "
            f"{value_type}(length={len(value)})"
        )

        if depth < max_depth:
            for index, child in enumerate(value[:max_elements]):
                describe(
                    child,
                    name=f"[{index}]",
                    depth=depth + 1,
                    max_depth=max_depth,
                    max_elements=max_elements,
                )
        return

    if hasattr(value, "shape") and hasattr(value, "dtype"):
        print(
            f"{indentation}{name}: {value_type}("
            f"shape={value.shape}, dtype={value.dtype})"
        )
        return

    print(f"{indentation}{name}: {value_type} = {value!r}")


def find_largest_sequence(
    dataset: list[list[list[int]]],
) -> tuple[int, list[list[int]]]:
    """
    Find the sequence containing the greatest number of boxes.

    Returns:
        A tuple containing:
        - sequence index
        - sequence contents
    """
    if not dataset:
        raise ValueError("The dataset contains no sequences.")

    sequence_index, sequence = max(
        enumerate(dataset),
        key=lambda indexed_sequence: len(indexed_sequence[1]),
    )

    return sequence_index, sequence


def calculate_sequence_volume(
    sequence: list[list[int]],
) -> int:
    """Calculate the total volume in dataset grid units."""
    return sum(
        length * width * height
        for length, width, height in sequence
    )


def main() -> None:
    if not DATASET_PATH.is_file():
        raise FileNotFoundError(
            f"Dataset not found: {DATASET_PATH.resolve()}"
        )

    try:
        dataset = torch.load(
            DATASET_PATH,
            map_location="cpu",
            weights_only=False,
        )
    except TypeError:
        # Compatibility with older PyTorch versions.
        dataset = torch.load(
            DATASET_PATH,
            map_location="cpu",
        )

    if not isinstance(dataset, list):
        raise TypeError(
            "Expected the top-level dataset object to be a list, "
            f"but received {type(dataset).__name__}."
        )

    print(f"File: {DATASET_PATH.resolve()}")
    print(f"Top-level Python type: {type(dataset)}")
    print()

    describe(dataset)

    largest_index, largest_sequence = find_largest_sequence(dataset)
    total_volume = calculate_sequence_volume(largest_sequence)

    print()
    print("Largest sequence")
    print("----------------")
    print(f"Sequence index: {largest_index}")
    print(f"Number of boxes: {len(largest_sequence)}")
    print(f"Total grid-unit volume: {total_volume}")

    print()
    print("First 10 boxes:")
    for box_index, dimensions in enumerate(largest_sequence[:10]):
        print(f"  Box {box_index}: {dimensions}")

    print()
    print("Complete largest sequence:")
    print(largest_sequence)


if __name__ == "__main__":
    main()