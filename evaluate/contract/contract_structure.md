# Packing Algorithm Evaluation Contract

This document defines the output format that every pallet-packing algorithm should produce so that the shared `Evaluator` module can compare different algorithms consistently.

The evaluator treats each algorithm as a black box:

```text
same input boxes + same pallet parameters -> algorithm output JSON -> evaluator metrics
```

The algorithm may use any internal method it wants. The only requirement is that the final output follows this contract.

## Files

- `evaluate_finished.py` — evaluator module.
- `algorithm_output_contract_example.json` — self-contained example payload that can be loaded and evaluated directly.

## Top-level JSON structure

For a self-contained result file, use this top-level structure:

```json
{
  "pallet_parameters": {},
  "algorithm_results": {}
}
```

The evaluator itself is initialized with these two dictionaries:

```python
from evaluate_finished import Evaluator

metrics = Evaluator(
    algorithm_results=payload["algorithm_results"],
    pallet_parameters=payload["pallet_parameters"],
).evaluate()
```

## `pallet_parameters` contract

Use millimetres for all dimensions unless your project explicitly chooses another unit. The important rule is that pallet dimensions, box dimensions, and positions must all use the same unit.

Required canonical fields:

```json
{
  "pallet_base_width": 800,
  "pallet_length": 1200,
  "max_stack_height": 1500
}
```

Meaning:

| Field | Type | Meaning |
|---|---:|---|
| `pallet_base_width` | number | Pallet width along the y-axis. |
| `pallet_length` | number | Pallet length along the x-axis. |
| `max_stack_height` | number | Maximum allowed stack height along the z-axis. |

The evaluator also accepts some fallback field names, such as `width`, `length`, and `max_height`, but new algorithms should use the canonical names above.

## `algorithm_results` contract

Required structure:

```json
{
  "algorithm_name": "example_algorithm",
  "dataset_name": "example_dataset",
  "total_boxes": 5,
  "runtime": 0.012,
  "drops": []
}
```

Fields:

| Field | Required | Type | Meaning |
|---|---:|---:|---|
| `algorithm_name` | No | string | Human-readable algorithm name. Useful when collecting benchmark results. |
| `dataset_name` | No | string | Name of the input dataset used for this result. |
| `total_boxes` | Yes | integer | Number of boxes given to the algorithm before packing. This is used for `boxes_placed_percentage`. |
| `runtime` | No | number | Runtime in seconds. Use wall-clock time unless your benchmark defines another timing method. |
| `drops` | Yes | array | List of successfully placed boxes. One drop equals one placed box. |

A box that was not placed must not appear in `drops`.

## `drops` contract

Each placed box must be represented as one drop:

```json
{
  "box": {
    "id": "box_001",
    "sku": "SKU-A",
    "length": 400,
    "width": 300,
    "height": 200,
    "weight": 8.5
  },
  "position": {
    "x": 0,
    "y": 0,
    "z": 0
  }
}
```

### `box` fields

| Field | Required | Type | Meaning |
|---|---:|---:|---|
| `id` | Yes | string | Unique box instance ID. Do not reuse the same ID for two placed boxes. |
| `sku` | No | string | Product or box-type identifier. Multiple boxes may share the same SKU. |
| `length` | Yes | number | Placed length of the box along the x-axis. |
| `width` | Yes | number | Placed width of the box along the y-axis. |
| `height` | Yes | number | Placed height of the box along the z-axis. |
| `weight` | No | number | Box weight. Not currently used by the evaluator metrics, but useful for later stability metrics. |

If the algorithm rotates a box, the output dimensions must already reflect the chosen orientation. For example, if a `400 x 300 x 200` box is rotated so that `300` is along x and `400` is along y, output:

```json
{
  "length": 300,
  "width": 400,
  "height": 200
}
```

The evaluator does not need to know the original unrotated dimensions to compute the current metrics.

### `position` fields

| Field | Required | Type | Meaning |
|---|---:|---:|---|
| `x` | Yes | number | Lower-left-bottom x-coordinate of the box. |
| `y` | Yes | number | Lower-left-bottom y-coordinate of the box. |
| `z` | Yes | number | Bottom height of the box. `z = 0` means the box is directly on the pallet floor. |

The position is the lower-left-bottom corner of the placed box.

A placed box occupies this rectangular volume:

```text
x: [position.x, position.x + box.length]
y: [position.y, position.y + box.width]
z: [position.z, position.z + box.height]
```

## Coordinate convention

Use a right-handed, axis-aligned pallet coordinate system:

```text
x = pallet length direction
y = pallet width direction
z = vertical height direction
```

The pallet floor starts at:

```text
x = 0
y = 0
z = 0
```

So a valid floor box has `position.z = 0`.

## Validity requirements

The evaluator computes metrics from the given placements. It does not fully repair or optimize invalid outputs. Algorithm outputs should therefore satisfy these rules before evaluation:

1. All placed boxes are inside the pallet footprint.
2. No two boxes overlap in 3D space.
3. A box is either on the pallet floor or has physical support underneath it.
4. Box dimensions are positive numbers.
5. Box positions are non-negative numbers.
6. Each placed box has one unique `box.id`.
7. Units are consistent across pallet dimensions, box dimensions, and positions.

## Metrics computed by the evaluator

The current evaluator computes:

| Metric | Meaning |
|---|---|
| `boxes_placed_percentage` | `len(drops) / total_boxes * 100`. |
| `packed_volume_percentage` | Total placed box volume divided by usable pallet volume. |
| `final_stack_height` | Highest occupied z-coordinate: `max(position.z + box.height)`. |
| `bounding_box_density` | Placed volume divided by the volume of the smallest axis-aligned box containing all placed boxes. |
| `average_support_ratio` | Mean support ratio including boxes on the pallet floor. |
| `average_support_ratio_non_floor` | Mean support ratio for stacked boxes only. |
| `minimum_support_ratio` | Worst support ratio including floor boxes. |
| `minimum_support_ratio_non_floor` | Worst support ratio for stacked boxes only. |
| `runtime` | Runtime copied from `algorithm_results.runtime`. |
| `number_of_stacks` | Number of physically connected box clusters. |

## Support ratio definition

For a single box:

```text
support_ratio = supported_bottom_area / box_bottom_area
```

- Floor boxes, where `position.z = 0`, are counted as fully supported: `support_ratio = 1.0`.
- For non-floor boxes, only boxes whose top face exactly touches the current box bottom face are counted as supporters.
- The evaluator clips supporter top faces to the bottom footprint of the current box.
- It uses the union of those support rectangles, so overlapping supporters are not double-counted.

## Minimal usage example

```python
import json
from evaluate_finished import Evaluator

with open("algorithm_output_contract_example.json", "r", encoding="utf-8") as f:
    payload = json.load(f)

metrics = Evaluator(
    algorithm_results=payload["algorithm_results"],
    pallet_parameters=payload["pallet_parameters"],
).evaluate(output_dir="results", print_to_stdout=True)
```

This writes:

```text
results/evaluation_metrics.csv
```

and returns the metrics as a Python dictionary.

## Notes for algorithm authors

- Do not output candidate placements, rejected boxes, search states, or intermediate states in `drops`. Only final placed boxes belong there.
- Store unplaced boxes elsewhere if needed, for example in an optional `unplaced_box_ids` list.
- If you add extra fields, keep the required fields unchanged. The evaluator ignores unknown fields.
- Keep `box.id` unique per physical box instance. Use `sku` for shared product type.
- Always output dimensions after rotation, not before rotation.
