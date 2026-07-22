"""
This module evaluates the performance of a packing algorithm on a given set of
boxes and a pallet/container.

The evaluator assumes that an algorithm returns placed boxes as ``drops``.
Each drop is expected to look like this:

    {
        "box": {
            "id": "box_001",
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

Coordinate convention
---------------------
Positions are lower-left-bottom corner coordinates, not box-center coordinates.
A placed box occupies:

    x: [position.x, position.x + box.length]
    y: [position.y, position.y + box.width]
    z: [position.z, position.z + box.height]

    
The evaluation metrics include:
    1. Boxes placed %
    2. Packed volume %
    3. Final stack height
    4. Bounding-box density
    5. Average support ratio
    6. Minimum support ratio
    7. Number of separate stacks / connected components

The results can be printed to stdout and saved as a CSV file.
"""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple, Union


Rect = Tuple[float, float, float, float]


class Evaluator:
    """Evaluate one packing algorithm result.

    Parameters:
        algorithm_results:
            Dictionary returned by the packing algorithm. At minimum it should
            contain a ``drops`` list. It may also contain ``total_boxes``
        pallet_parameters:
            Dictionary describing the pallet/container. The common expected keys
            are ``pallet_base_width``, ``pallet_length`` and ``max_stack_height``.
            The evaluator also accepts fallback names such as ``width``,
            ``length`` and ``max_height``.
        eps:
            Tolerance used for floating-point contact checks.

    Notes
    -----
    Every drop must use the nested ``position`` mapping, and ``position.x``,
    ``position.y`` and ``position.z`` are always interpreted as the box's
    lower-left-bottom corner. Box-center coordinates and alternate coordinate
    layouts are not supported.
    """

    def __init__(
        self,
        algorithm_results: Dict[str, Any],
        pallet_parameters: Dict[str, Any],
        eps: float = 1e-6,
    ) -> None:
        self.algorithm_results: Dict[str, Any] = algorithm_results
        self.pallet_parameters: Dict[str, Any] = pallet_parameters
        self.eps = eps

        self.drops: List[Dict[str, Any]] = list(algorithm_results.get("drops", []) or [])
        self._validate_drop_convention()

        self.pallet_width: float = self._first_number(
            pallet_parameters,
            "pallet_base_width",
            "pallet_width",
            "width",
            default=0.0,
        )
        self.pallet_length: float = self._first_number(
            pallet_parameters,
            "pallet_length",
            "length",
            default=0.0,
        )
        self.pallet_max_height: float = self._first_number(
            pallet_parameters,
            "max_stack_height",
            "pallet_max_height",
            "max_height",
            "height",
            default=0.0,
        )

    def evaluate(
        self,
        output_dir: Optional[Union[str, Path]] = None,
        print_to_stdout: bool = False,
    ) -> Dict[str, float]:
        """Compute all evaluation metrics.

        Parameters
        ----------
        output_dir:
            If provided, metrics are saved to
            ``<output_dir>/evaluation_metrics.csv``.
        print_to_stdout:
            If true, metrics are printed to stdout.

        Returns
        -------
        dict
            Dictionary containing the evaluation metrics.
        """

        metrics: Dict[str, float] = {
            "boxes_placed_percentage": self.compute_boxes_placed_percentage(),
            "packed_volume_percentage": self.compute_packed_volume_percentage(),
            "final_stack_height": self.compute_final_stack_height(),
            "bounding_box_density": self.compute_bounding_box_density(),
            "average_support_ratio": self.compute_average_support_ratio(include_floor_boxes=True),
            "average_support_ratio_non_floor": self.compute_average_support_ratio(include_floor_boxes=False),
            "minimum_support_ratio": self.compute_minimum_support_ratio(include_floor_boxes=True),
            "minimum_support_ratio_non_floor": self.compute_minimum_support_ratio(include_floor_boxes=False),
            "number_of_stacks": float(self.compute_number_of_stacks()),
        }

        if print_to_stdout:
            self.print_metrics(metrics)

        if output_dir is not None:
            self.save_metrics_to_csv(metrics, output_dir)

        return metrics

    def save_metrics_to_csv(
        self,
        metrics: Dict[str, float],
        output_dir: Union[str, Path],
        filename: str = "evaluation_metrics.csv",
    ) -> Path:
        """Save metrics as a two-column CSV file.

        Returns the path of the written file.
        """

        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        csv_path = output_path / filename

        with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(["metric", "value"])
            for key, value in metrics.items():
                writer.writerow([key, value])

        return csv_path

    def print_metrics(self, metrics: Dict[str, float]) -> None:
        """Print metrics in a readable format."""

        for key, value in metrics.items():
            print(f"{key}: {value}")

    # ------------------------------------------------------------------
    # Core metrics
    # ------------------------------------------------------------------

    def compute_boxes_placed_percentage(self) -> float:
        """Return the percentage of input boxes that were placed."""

        total_boxes = float(self.algorithm_results.get("total_boxes", 0) or 0)
        placed_boxes = float(len(self.drops))

        if total_boxes <= 0:
            return 0.0

        return (placed_boxes / total_boxes) * 100.0

    def compute_packed_volume_percentage(self) -> float:
        """Return placed-box volume as a percentage of usable pallet volume."""

        total_volume = self.pallet_width * self.pallet_length * self.pallet_max_height

        if total_volume <= 0:
            return 0.0

        packed_volume = sum(self._box_volume(drop) for drop in self.drops)
        return (packed_volume / total_volume) * 100.0

    def compute_final_stack_height(self) -> float:
        """Return the highest occupied z-coordinate of the packed pallet."""

        if not self.drops:
            return 0.0

        return max(self._z(drop) + self._height(drop) for drop in self.drops)

    def compute_bounding_box_density(self) -> float:
        """Return packed volume divided by final stack bounding-box volume.

        This rewards compact outputs. For example, two algorithms may place
        the same total volume, but the one that spreads boxes across a larger
        empty region will have a lower density.
        """

        if not self.drops:
            return 0.0

        min_x = min(self._x(drop) for drop in self.drops)
        max_x = max(self._x(drop) + self._length(drop) for drop in self.drops)
        min_y = min(self._y(drop) for drop in self.drops)
        max_y = max(self._y(drop) + self._width(drop) for drop in self.drops)
        min_z = min(self._z(drop) for drop in self.drops)
        max_z = max(self._z(drop) + self._height(drop) for drop in self.drops)

        bounding_box_volume = (max_x - min_x) * (max_y - min_y) * (max_z - min_z)

        if bounding_box_volume <= 0:
            return 0.0

        packed_volume = sum(self._box_volume(drop) for drop in self.drops)
        return (packed_volume / bounding_box_volume) * 100.0

    def compute_average_support_ratio(self, include_floor_boxes: bool = True) -> float:
        """Return the mean support ratio over placed boxes.

        A support ratio is in the range [0, 1]:

            supported_area_under_box / box_base_area

        Floor boxes are normally considered fully supported. Set
        ``include_floor_boxes=False`` to evaluate only boxes stacked on top of
        other boxes.
        """

        drops = self._support_metric_drops(include_floor_boxes)

        if not drops:
            return 0.0

        support_ratios = [self.compute_support_ratio(drop, self.drops) for drop in drops]
        return sum(support_ratios) / len(support_ratios)

    def compute_minimum_support_ratio(self, include_floor_boxes: bool = True) -> float:
        """Return the worst support ratio among placed boxes."""

        drops = self._support_metric_drops(include_floor_boxes)

        if not drops:
            return 0.0

        return min(self.compute_support_ratio(drop, self.drops) for drop in drops)

    def compute_support_ratio(
        self,
        drop: Dict[str, Any],
        placed_drops: Optional[List[Dict[str, Any]]] = None,
        eps: Optional[float] = None,
    ) -> float:
        """Compute support ratio for one placed box.

        The current box is supported by the pallet floor if ``z == 0``.
        Otherwise, only boxes whose top face touches this box's bottom face
        are considered supporters. Their top faces are clipped to the current
        box's bottom footprint, and the union of those clipped areas is used.

        Using the union area avoids accidentally double-counting support if
        invalid or overlapping supporter boxes exist.
        """

        tolerance = self.eps if eps is None else eps
        placed_drops = self.drops if placed_drops is None else placed_drops

        if self._z(drop) <= tolerance:
            return 1.0

        base_area = self._base_area(drop)

        if base_area <= 0:
            return 0.0

        box_x1, box_x2, box_y1, box_y2 = self._xy_rect(drop)
        support_rectangles: List[Rect] = []

        for other in placed_drops:
            if other is drop:
                continue

            if self._same_placement(drop, other):
                continue

            other_top_z = self._z(other) + self._height(other)

            # Only boxes whose top face touches this box's bottom face can
            # physically support it. Lower boxes with an air gap do not count.
            if abs(other_top_z - self._z(drop)) > tolerance:
                continue

            other_x1, other_x2, other_y1, other_y2 = self._xy_rect(other)

            overlap_x1 = max(box_x1, other_x1)
            overlap_x2 = min(box_x2, other_x2)
            overlap_y1 = max(box_y1, other_y1)
            overlap_y2 = min(box_y2, other_y2)

            if overlap_x2 - overlap_x1 > tolerance and overlap_y2 - overlap_y1 > tolerance:
                support_rectangles.append((overlap_x1, overlap_x2, overlap_y1, overlap_y2))

        supported_area = self._union_area(support_rectangles)
        ratio = supported_area / base_area

        # Clamp protects against tiny floating-point errors.
        return max(0.0, min(1.0, ratio))

    def compute_number_of_stacks(self) -> int:
        """Return the number of connected physical clusters.

        Two boxes are in the same stack/component if they physically touch:
        - one supports the other vertically, or
        - their side faces touch with overlapping height and side length.

        This makes separated towers count as separate stacks, while adjacent
        boxes that touch each other count as one connected stack.
        """

        n = len(self.drops)

        if n == 0:
            return 0

        adjacency: List[List[int]] = [[] for _ in range(n)]

        for i in range(n):
            for j in range(i + 1, n):
                if self._boxes_touch(self.drops[i], self.drops[j]):
                    adjacency[i].append(j)
                    adjacency[j].append(i)

        visited = [False] * n
        components = 0

        for start in range(n):
            if visited[start]:
                continue

            components += 1
            stack = [start]
            visited[start] = True

            while stack:
                node = stack.pop()

                for neighbor in adjacency[node]:
                    if not visited[neighbor]:
                        visited[neighbor] = True
                        stack.append(neighbor)

        return components

    # ------------------------------------------------------------------
    # Geometry helpers
    # ------------------------------------------------------------------

    def _support_metric_drops(self, include_floor_boxes: bool) -> List[Dict[str, Any]]:
        if include_floor_boxes:
            return self.drops

        return [drop for drop in self.drops if self._z(drop) > self.eps]

    def _boxes_touch(self, a: Dict[str, Any], b: Dict[str, Any]) -> bool:
        """Return True if two placed boxes physically touch."""

        ax1, ax2, ay1, ay2 = self._xy_rect(a)
        bx1, bx2, by1, by2 = self._xy_rect(b)
        az1, az2 = self._z(a), self._z(a) + self._height(a)
        bz1, bz2 = self._z(b), self._z(b) + self._height(b)

        x_overlap = self._interval_overlap(ax1, ax2, bx1, bx2)
        y_overlap = self._interval_overlap(ay1, ay2, by1, by2)
        z_overlap = self._interval_overlap(az1, az2, bz1, bz2)

        # Vertical support contact.
        if x_overlap > self.eps and y_overlap > self.eps:
            if abs(az2 - bz1) <= self.eps or abs(bz2 - az1) <= self.eps:
                return True

        # Side contact along x.
        if y_overlap > self.eps and z_overlap > self.eps:
            if abs(ax2 - bx1) <= self.eps or abs(bx2 - ax1) <= self.eps:
                return True

        # Side contact along y.
        if x_overlap > self.eps and z_overlap > self.eps:
            if abs(ay2 - by1) <= self.eps or abs(by2 - ay1) <= self.eps:
                return True

        return False

    @staticmethod
    def _interval_overlap(a1: float, a2: float, b1: float, b2: float) -> float:
        return max(0.0, min(a2, b2) - max(a1, b1))

    def _xy_rect(self, drop: Dict[str, Any]) -> Rect:
        x1 = self._x(drop)
        x2 = x1 + self._length(drop)
        y1 = self._y(drop)
        y2 = y1 + self._width(drop)
        return x1, x2, y1, y2

    def _base_area(self, drop: Dict[str, Any]) -> float:
        return self._length(drop) * self._width(drop)

    def _box_volume(self, drop: Dict[str, Any]) -> float:
        return self._length(drop) * self._width(drop) * self._height(drop)

    def _x(self, drop: Dict[str, Any]) -> float:
        return self._position_value(drop, "x")

    def _y(self, drop: Dict[str, Any]) -> float:
        return self._position_value(drop, "y")

    def _z(self, drop: Dict[str, Any]) -> float:
        return self._position_value(drop, "z")

    def _length(self, drop: Dict[str, Any]) -> float:
        return self._box_value(drop, "length")

    def _width(self, drop: Dict[str, Any]) -> float:
        return self._box_value(drop, "width")

    def _height(self, drop: Dict[str, Any]) -> float:
        return self._box_value(drop, "height")

    @staticmethod
    def _first_number(
        data: Dict[str, Any],
        *keys: str,
        default: float = 0.0,
    ) -> float:
        for key in keys:
            value = data.get(key)
            if value is not None:
                return float(value)
        return default

    def _validate_drop_convention(self) -> None:
        """Require the lower-left-bottom drop schema for every placement.

        Supported placement schema::

            {
                "box": {"length": ..., "width": ..., "height": ...},
                "position": {"x": ..., "y": ..., "z": ...},
            }

        The coordinates are never interpreted as box-center coordinates. No
        direct ``drop["x"]``/``drop["y"]``/``drop["z"]`` fallback is
        accepted.
        """

        for index, drop in enumerate(self.drops):
            if not isinstance(drop, dict):
                raise ValueError(f"drops[{index}] must be a dictionary.")

            box = drop.get("box")
            if not isinstance(box, dict):
                raise ValueError(f"drops[{index}].box must be a dictionary.")

            position = drop.get("position")
            if not isinstance(position, dict):
                raise ValueError(
                    f"drops[{index}].position must be a dictionary containing "
                    "lower-left-bottom x, y and z coordinates."
                )

            for key in ("x", "y", "z"):
                if key not in position:
                    raise ValueError(
                        f"Missing drops[{index}].position.{key}; positions must "
                        "use lower-left-bottom corner coordinates."
                    )
                try:
                    float(position[key])
                except (TypeError, ValueError) as exc:
                    raise ValueError(
                        f"drops[{index}].position.{key} must be numeric."
                    ) from exc

            for key in ("length", "width", "height"):
                if key not in box:
                    raise ValueError(f"Missing drops[{index}].box.{key}.")
                try:
                    value = float(box[key])
                except (TypeError, ValueError) as exc:
                    raise ValueError(
                        f"drops[{index}].box.{key} must be numeric."
                    ) from exc
                if value <= 0:
                    raise ValueError(
                        f"drops[{index}].box.{key} must be positive."
                    )

    @staticmethod
    def _position_value(drop: Dict[str, Any], key: str) -> float:
        """Read a lower-left-bottom coordinate from ``drop['position']``."""

        return float(drop["position"][key])

    @staticmethod
    def _box_value(drop: Dict[str, Any], key: str) -> float:
        """Read a required explicit box dimension from ``drop['box']``."""

        return float(drop["box"][key])

    def _same_placement(self, a: Dict[str, Any], b: Dict[str, Any]) -> bool:
        """Return True if two drop dictionaries appear to describe the same box."""

        if a is b:
            return True

        a_box = a.get("box", {}) or {}
        b_box = b.get("box", {}) or {}

        a_id = a_box.get("id")
        b_id = b_box.get("id")

        if a_id is not None and b_id is not None and a_id == b_id:
            same_position = (
                abs(self._x(a) - self._x(b)) <= self.eps
                and abs(self._y(a) - self._y(b)) <= self.eps
                and abs(self._z(a) - self._z(b)) <= self.eps
            )
            return same_position

        return False

    @staticmethod
    def _union_area(rectangles: Iterable[Rect]) -> float:
        """Compute union area of axis-aligned rectangles.

        Rectangles are represented as ``(x1, x2, y1, y2)``.
        """

        rects = list(rectangles)

        if not rects:
            return 0.0

        x_coordinates = sorted({x for rect in rects for x in (rect[0], rect[1])})
        total_area = 0.0

        for left, right in zip(x_coordinates, x_coordinates[1:]):
            slab_width = right - left

            if slab_width <= 0:
                continue

            y_intervals: List[Tuple[float, float]] = []

            for x1, x2, y1, y2 in rects:
                if x1 < right and x2 > left:
                    y_intervals.append((y1, y2))

            if not y_intervals:
                continue

            y_intervals.sort()
            merged_length = 0.0
            current_start, current_end = y_intervals[0]

            for start, end in y_intervals[1:]:
                if start <= current_end:
                    current_end = max(current_end, end)
                else:
                    merged_length += current_end - current_start
                    current_start, current_end = start, end

            merged_length += current_end - current_start
            total_area += slab_width * merged_length

        return total_area


if __name__ == "__main__":
    # Small sanity-check example. This block is safe to delete if the evaluator
    # is only imported from a benchmark runner.
    example_results = {
        "total_boxes": 2,
        "drops": [
            {
                "box": {"id": "lower", "length": 200, "width": 300, "height": 200},
                "position": {"x": 0, "y": 0, "z": 0},
            },
            {
                "box": {"id": "upper", "length": 400, "width": 300, "height": 100},
                "position": {"x": 0, "y": 0, "z": 200},
            },
        ],
    }

    example_pallet = {
        "pallet_base_width": 800,
        "pallet_length": 1200,
        "max_stack_height": 1500,
    }

    Evaluator(example_results, example_pallet).evaluate(print_to_stdout=True)
