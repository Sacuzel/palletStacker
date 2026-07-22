"""
Validation module for the pallet-packing algorithm output contract.

The validator checks that an algorithm output follows the same contract used by
``evaluate_finished.py``:

    {
        "pallet_parameters": {
            "pallet_base_width": 800,
            "pallet_length": 1200,
            "max_stack_height": 1500
        },
        "algorithm_results": {
            "total_boxes": 5,
            "runtime": 0.012,
            "drops": [
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
            ]
        }
    }

Coordinate convention
---------------------
Positions are lower-left-bottom corner coordinates, not box-center coordinates.
A placed box occupies:

    x: [position.x, position.x + box.length]
    y: [position.y, position.y + box.width]
    z: [position.z, position.z + box.height]

The validator always raises ``ValueError`` when the output violates the contract.
``validate_results()`` returns True when the output is valid and raises ValueError
if the output is invalid and ``fail_on_error`` is True.
"""

from __future__ import annotations

import json
import logging
import math
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple, Union


logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

Rect = Tuple[float, float, float, float]
Point = Tuple[float, float]
Polygon = List[Point]
LBCP = Tuple[Polygon, float, Any]
Drop = Dict[str, Any]


class Validator:
    """Validate one packing algorithm result against the output contract.

    Parameters:
        algorithm_results:
            The ``algorithm_results`` dictionary from the contract. Required keys
            are ``total_boxes`` and ``drops``. ``runtime`` is optional.
        pallet_parameters:
            The ``pallet_parameters`` dictionary from the contract. Required keys
            are ``pallet_base_width``, ``pallet_length`` and ``max_stack_height``.
        fail_on_error:
            If True, raise ValueError on the first validation error. If False, log
            all errors and return False from ``validate_results()``.
        eps:
            Tolerance used for floating-point comparisons.
        minimum_support_ratio:
            Minimum support ratio required for non-floor boxes. The default ``0.0``
            means a non-floor box only needs some positive physical contact below
            it. For stricter validation, use something like ``0.8``.
        cog_uncertainty_ratio:
            Half-size of the horizontal centre-of-gravity uncertainty region as a
            fraction of the box length and width. ``0.0`` checks only the geometric
            centre. For example, ``0.05`` checks a rectangle extending 5% of the
            corresponding box dimension in every horizontal direction.
    """

    REQUIRED_PALLET_FIELDS = ("pallet_base_width", "pallet_length", "max_stack_height")
    REQUIRED_ALGORITHM_FIELDS = ("total_boxes", "drops")
    REQUIRED_BOX_FIELDS = ("id", "length", "width", "height")
    REQUIRED_POSITION_FIELDS = ("x", "y", "z")

    def __init__(
        self,
        algorithm_results: Dict[str, Any],
        pallet_parameters: Dict[str, Any],
        fail_on_error: bool = True,
        eps: float = 1e-6,
        minimum_support_ratio: float = 0.0,
        cog_uncertainty_ratio: float = 0.0,
    ) -> None:
        self.algorithm_results = algorithm_results
        self.pallet_parameters = pallet_parameters
        self.fail_on_error = fail_on_error
        self.eps = eps
        self.minimum_support_ratio = minimum_support_ratio
        self.cog_uncertainty_ratio = self._as_number(
            cog_uncertainty_ratio, "cog_uncertainty_ratio"
        )
        if not 0.0 <= self.cog_uncertainty_ratio <= 0.5:
            raise ValueError(
                "cog_uncertainty_ratio must be between 0.0 and 0.5 inclusive, "
                f"got {self.cog_uncertainty_ratio}."
            )

        # Populated by verify_bottom_up_lbcp() for diagnostics and evaluation.
        self.lbcp_results: List[Dict[str, Any]] = []

        self.drops: List[Drop] = list(algorithm_results.get("drops", []) or [])

        # The contract uses x for pallet length, y for pallet width, z for height.
        self.pallet_width = self._as_number(
            pallet_parameters.get("pallet_base_width"), "pallet_parameters.pallet_base_width"
        )
        self.pallet_length = self._as_number(
            pallet_parameters.get("pallet_length"), "pallet_parameters.pallet_length"
        )
        self.pallet_max_height = self._as_number(
            pallet_parameters.get("max_stack_height"), "pallet_parameters.max_stack_height"
        )

    @classmethod
    def from_payload(
        cls,
        payload: Dict[str, Any],
        eps: float = 1e-6,
        minimum_support_ratio: float = 0.0,
        cog_uncertainty_ratio: float = 0.0,
    ) -> "Validator":
        """Create a validator from a self-contained contract JSON payload."""

        if not isinstance(payload, dict):
            raise ValueError("Top-level payload must be a JSON object/dictionary.")

        cls._require_mapping(payload, "pallet_parameters", "payload")
        cls._require_mapping(payload, "algorithm_results", "payload")

        return cls(
            algorithm_results=payload["algorithm_results"],
            pallet_parameters=payload["pallet_parameters"],
            eps=eps,
            minimum_support_ratio=minimum_support_ratio,
            cog_uncertainty_ratio=cog_uncertainty_ratio,
        )

    @classmethod
    def from_json_file(
        cls,
        path: Union[str, Path],
        eps: float = 1e-6,
        minimum_support_ratio: float = 0.0,
        cog_uncertainty_ratio: float = 0.0,
    ) -> "Validator":
        """Load a self-contained contract JSON file and create a validator."""

        with Path(path).open("r", encoding="utf-8") as file:
            payload = json.load(file)

        return cls.from_payload(
            payload,
            eps=eps,
            minimum_support_ratio=minimum_support_ratio,
            cog_uncertainty_ratio=cog_uncertainty_ratio,
        )

    def validate_results(self) -> bool:
        """Validate structure and placement legality.

        Returns
        -------
        bool
            True if validation is successful.

        Raises
        ------
        ValueError
            If the result violates the contract.
        """

        self.verify_contract_structure()
        self.verify_all_drops_are_legal()
        return True

    def verify_contract_structure(self) -> None:
        """Verify that required contract fields exist and have valid types."""

        if not isinstance(self.pallet_parameters, dict):
            raise ValueError("pallet_parameters must be a dictionary.")
        if not isinstance(self.algorithm_results, dict):
            raise ValueError("algorithm_results must be a dictionary.")

        for field in self.REQUIRED_PALLET_FIELDS:
            if field not in self.pallet_parameters:
                raise ValueError(f"Missing required pallet_parameters.{field!s}.")
            value = self._as_number(self.pallet_parameters[field], f"pallet_parameters.{field}")
            if value <= 0:
                raise ValueError(f"pallet_parameters.{field} must be positive, got {value}.")

        for field in self.REQUIRED_ALGORITHM_FIELDS:
            if field not in self.algorithm_results:
                raise ValueError(f"Missing required algorithm_results.{field!s}.")

        total_boxes = self.algorithm_results["total_boxes"]
        if not isinstance(total_boxes, int) or isinstance(total_boxes, bool):
            raise ValueError("algorithm_results.total_boxes must be an integer.")
        if total_boxes < 0:
            raise ValueError("algorithm_results.total_boxes must be non-negative.")

        if not isinstance(self.drops, list):
            raise ValueError("algorithm_results.drops must be a list.")

        if len(self.drops) > total_boxes:
            raise ValueError(
                "algorithm_results.drops contains more placed boxes than "
                f"algorithm_results.total_boxes ({len(self.drops)} > {total_boxes})."
            )

        seen_ids = set()
        for index, drop in enumerate(self.drops):
            self._validate_drop_structure(drop, index)
            box_id = drop["box"]["id"]
            if box_id in seen_ids:
                raise ValueError(f"Duplicate box.id found in drops: {box_id!r}.")
            seen_ids.add(box_id)

    def _validate_drop_structure(self, drop: Drop, index: int) -> None:
        path = f"algorithm_results.drops[{index}]"

        if not isinstance(drop, dict):
            raise ValueError(f"{path} must be a dictionary.")

        self._require_mapping(drop, "box", path)
        self._require_mapping(drop, "position", path)

        box = drop["box"]
        position = drop["position"]

        for field in self.REQUIRED_BOX_FIELDS:
            if field not in box:
                raise ValueError(f"Missing required {path}.box.{field}.")

        box_id = box["id"]
        if not isinstance(box_id, (str, int)) or isinstance(box_id, bool) or str(box_id) == "":
            raise ValueError(f"{path}.box.id must be a non-empty string or integer.")

        for field in ("length", "width", "height"):
            value = self._as_number(box[field], f"{path}.box.{field}")
            if value <= 0:
                raise ValueError(f"{path}.box.{field} must be positive, got {value}.")

        if "weight" in box:
            weight = self._as_number(box["weight"], f"{path}.box.weight")
            if weight < 0:
                raise ValueError(f"{path}.box.weight must be non-negative, got {weight}.")

        for field in self.REQUIRED_POSITION_FIELDS:
            if field not in position:
                raise ValueError(f"Missing required {path}.position.{field}.")
            value = self._as_number(position[field], f"{path}.position.{field}")
            if value < -self.eps:
                raise ValueError(f"{path}.position.{field} must be non-negative, got {value}.")

    def verify_all_drops_are_legal(self) -> None:
        """Verify bounds, overlaps, physical support, and LBCP stability."""

        self.verify_drops_within_pallet_bounds()
        self.verify_no_overlaps()
        self.verify_boxes_are_supported()
        self.verify_bottom_up_lbcp()

    def verify_drops_within_pallet_bounds(self) -> None:
        """Verify that every placed box is inside the pallet/container.

        Contract convention:
        - x + box.length must fit within pallet_length.
        - y + box.width must fit within pallet_base_width.
        - z + box.height must fit within max_stack_height.
        """

        logging.info("Validating lower-left-bottom corner positions against pallet bounds.")

        for index, drop in enumerate(self.drops):
            x1, x2, y1, y2, z1, z2 = self._box_extents(drop)

            if not (
                x1 >= -self.eps
                and y1 >= -self.eps
                and z1 >= -self.eps
                and x2 <= self.pallet_length + self.eps
                and y2 <= self.pallet_width + self.eps
                and z2 <= self.pallet_max_height + self.eps
            ):
                message = (
                        f"Drop at index {index} is out of pallet bounds. "
                        f"Box extents are x=[{x1}, {x2}], y=[{y1}, {y2}], z=[{z1}, {z2}], "
                        f"but pallet extents are x=[0, {self.pallet_length}], "
                        f"y=[0, {self.pallet_width}], z=[0, {self.pallet_max_height}]."
                    )
                if self.fail_on_error:
                    raise ValueError(message)
                else:
                    logger.warning(message)

    def verify_no_overlaps(self) -> None:
        """Verify that no two boxes overlap in 3D space."""

        logging.info("Verifying no 3D AABB overlaps between drops.")

        for i, drop1 in enumerate(self.drops):
            for j in range(i + 1, len(self.drops)):
                drop2 = self.drops[j]
                if self._boxes_overlap(drop1, drop2):
                    id1 = drop1["box"].get("id", i)
                    id2 = drop2["box"].get("id", j)
                    message = f"Overlap detected between drops {id1!r} and {id2!r}."
                    if self.fail_on_error:
                        raise ValueError(message)
                    else:
                        logger.warning(message)

    def verify_boxes_are_supported(self) -> None:
        """Verify that every non-floor box has physical support underneath it."""

        logging.info("Verifying that every non-floor box has support underneath it.")

        for index, drop in enumerate(self.drops):
            z = self._z(drop)
            if z <= self.eps:
                continue

            ratio = self.compute_support_ratio(drop)
            required_ratio = max(self.minimum_support_ratio, self.eps)

            if ratio < required_ratio:
                box_id = drop["box"].get("id", index)
                message = (
                    f"Drop {box_id!r} at index {index} is unsupported or insufficiently supported. "
                    f"Support ratio is {ratio:.6g}; required at least {required_ratio:.6g}."
                )
                if self.fail_on_error:
                    raise ValueError(message)
                else:
                    logger.warning(message)

    def verify_bottom_up_lbcp(self) -> None:
        """Validate the completed stack bottom-up using LBCPs.

        The pallet floor is the initial LBCP. Floor boxes contribute their
        complete top faces. Every higher box is processed in ascending bottom
        height order. Its candidate LBCP is the convex hull of the portions of
        its footprint that overlap already validated LBCPs at the same contact
        height. The box passes only when its complete horizontal CoG uncertainty
        region lies inside that candidate polygon.

        Successful per-box results are stored in ``self.lbcp_results``.
        """

        logging.info("Running bottom-up Load Bearable Convex Polygon validation.")

        floor_polygon: Polygon = [
            (0.0, 0.0),
            (self.pallet_length, 0.0),
            (self.pallet_length, self.pallet_width),
            (0.0, self.pallet_width),
        ]
        lbcps: List[LBCP] = [(floor_polygon, 0.0, "__pallet_floor__")]
        self.lbcp_results = []

        indexed_drops = list(enumerate(self.drops))
        indexed_drops.sort(key=lambda item: (self._z(item[1]), item[0]))

        for original_index, drop in indexed_drops:
            box_id = drop["box"].get("id", original_index)
            bottom_z = self._z(drop)
            top_z = bottom_z + self._height(drop)
            footprint = self._rect_to_polygon(self._xy_rect(drop))

            if bottom_z <= self.eps:
                support_polygon = footprint
                supporting_ids = ["__pallet_floor__"]
            else:
                support_points: List[Point] = []
                supporting_ids: List[Any] = []

                for polygon, support_height, support_id in lbcps:
                    if abs(support_height - bottom_z) > self.eps:
                        continue

                    clipped = self._clip_polygon_to_rect(polygon, self._xy_rect(drop))
                    if self._polygon_area(clipped) <= self.eps:
                        continue

                    support_points.extend(clipped)
                    supporting_ids.append(support_id)

                support_polygon = self._convex_hull(support_points)

                if self._polygon_area(support_polygon) <= self.eps:
                    message = (
                        f"Drop {box_id!r} at index {original_index} has no positive-area "
                        "load-bearing contact region in the bottom-up LBCP model."
                    )
                    if self.fail_on_error:
                        raise ValueError(message)
                    else:
                        logger.warning(message)

            cog_region = self._cog_uncertainty_polygon(drop)
            if not all(
                self._point_in_convex_polygon(point, support_polygon)
                for point in cog_region
            ):
                message = (
                    f"Drop {box_id!r} at index {original_index} fails bottom-up LBCP "
                    "validation: its centre-of-gravity uncertainty region is not fully "
                    f"contained in the load-bearing support polygon at z={bottom_z}."
                )
                if self.fail_on_error:
                    raise ValueError(message)
                else:
                    logger.warning(message)

            lbcps.append((support_polygon, top_z, box_id))
            self.lbcp_results.append(
                {
                    "box_id": box_id,
                    "drop_index": original_index,
                    "bottom_z": bottom_z,
                    "top_z": top_z,
                    "supporting_ids": supporting_ids,
                    "lbcp": support_polygon,
                    "lbcp_area": self._polygon_area(support_polygon),
                    "cog_region": cog_region,
                }
            )

    def compute_support_ratio(self, drop: Drop, placed_drops: Optional[List[Drop]] = None) -> float:
        """Compute the physical support ratio for one placed box.

        Floor boxes are fully supported. For non-floor boxes, only top faces
        exactly touching the current box bottom face count as support.
        """

        placed_drops = self.drops if placed_drops is None else placed_drops

        if self._z(drop) <= self.eps:
            return 1.0

        base_area = self._length(drop) * self._width(drop)
        if base_area <= 0:
            return 0.0

        box_x1, box_x2, box_y1, box_y2 = self._xy_rect(drop)
        support_rectangles: List[Rect] = []

        for other in placed_drops:
            if other is drop:
                continue

            if self._same_box_id(drop, other):
                continue

            other_top_z = self._z(other) + self._height(other)
            if abs(other_top_z - self._z(drop)) > self.eps:
                continue

            other_x1, other_x2, other_y1, other_y2 = self._xy_rect(other)

            overlap_x1 = max(box_x1, other_x1)
            overlap_x2 = min(box_x2, other_x2)
            overlap_y1 = max(box_y1, other_y1)
            overlap_y2 = min(box_y2, other_y2)

            if overlap_x2 - overlap_x1 > self.eps and overlap_y2 - overlap_y1 > self.eps:
                support_rectangles.append((overlap_x1, overlap_x2, overlap_y1, overlap_y2))

        supported_area = self._union_area(support_rectangles)
        ratio = supported_area / base_area
        return max(0.0, min(1.0, ratio))

    @staticmethod
    def _rect_to_polygon(rect: Rect) -> Polygon:
        x1, x2, y1, y2 = rect
        return [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]

    def _cog_uncertainty_polygon(self, drop: Drop) -> Polygon:
        """Return the four horizontal extreme points of the possible CoG."""

        x1, x2, y1, y2 = self._xy_rect(drop)
        centre_x = (x1 + x2) / 2.0
        centre_y = (y1 + y2) / 2.0
        delta_x = self.cog_uncertainty_ratio * self._length(drop)
        delta_y = self.cog_uncertainty_ratio * self._width(drop)

        if delta_x <= self.eps and delta_y <= self.eps:
            return [(centre_x, centre_y)]

        return [
            (centre_x - delta_x, centre_y - delta_y),
            (centre_x + delta_x, centre_y - delta_y),
            (centre_x + delta_x, centre_y + delta_y),
            (centre_x - delta_x, centre_y + delta_y),
        ]

    def _clip_polygon_to_rect(self, polygon: Polygon, rect: Rect) -> Polygon:
        """Clip a convex polygon against an axis-aligned rectangle."""

        if not polygon:
            return []

        x1, x2, y1, y2 = rect
        result = list(polygon)

        def clip(
            points: Polygon,
            inside: Any,
            intersection: Any,
        ) -> Polygon:
            if not points:
                return []

            output: Polygon = []
            previous = points[-1]
            previous_inside = inside(previous)

            for current in points:
                current_inside = inside(current)
                if current_inside:
                    if not previous_inside:
                        output.append(intersection(previous, current))
                    output.append(current)
                elif previous_inside:
                    output.append(intersection(previous, current))
                previous = current
                previous_inside = current_inside

            return output

        def intersect_vertical(a: Point, b: Point, x_value: float) -> Point:
            dx = b[0] - a[0]
            if abs(dx) <= self.eps:
                return (x_value, a[1])
            t = (x_value - a[0]) / dx
            return (x_value, a[1] + t * (b[1] - a[1]))

        def intersect_horizontal(a: Point, b: Point, y_value: float) -> Point:
            dy = b[1] - a[1]
            if abs(dy) <= self.eps:
                return (a[0], y_value)
            t = (y_value - a[1]) / dy
            return (a[0] + t * (b[0] - a[0]), y_value)

        result = clip(
            result,
            lambda p: p[0] >= x1 - self.eps,
            lambda a, b: intersect_vertical(a, b, x1),
        )
        result = clip(
            result,
            lambda p: p[0] <= x2 + self.eps,
            lambda a, b: intersect_vertical(a, b, x2),
        )
        result = clip(
            result,
            lambda p: p[1] >= y1 - self.eps,
            lambda a, b: intersect_horizontal(a, b, y1),
        )
        result = clip(
            result,
            lambda p: p[1] <= y2 + self.eps,
            lambda a, b: intersect_horizontal(a, b, y2),
        )

        return self._deduplicate_polygon_points(result)

    def _deduplicate_polygon_points(self, points: Polygon) -> Polygon:
        unique: Polygon = []
        for point in points:
            if not any(
                abs(point[0] - other[0]) <= self.eps
                and abs(point[1] - other[1]) <= self.eps
                for other in unique
            ):
                unique.append(point)
        return unique

    def _convex_hull(self, points: Iterable[Point]) -> Polygon:
        """Return the counter-clockwise convex hull of 2D points."""

        unique = sorted(set((float(x), float(y)) for x, y in points))
        if len(unique) <= 1:
            return unique

        def cross(o: Point, a: Point, b: Point) -> float:
            return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

        lower: Polygon = []
        for point in unique:
            while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= self.eps:
                lower.pop()
            lower.append(point)

        upper: Polygon = []
        for point in reversed(unique):
            while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= self.eps:
                upper.pop()
            upper.append(point)

        return lower[:-1] + upper[:-1]

    @staticmethod
    def _polygon_area(polygon: Polygon) -> float:
        if len(polygon) < 3:
            return 0.0
        return abs(
            sum(
                polygon[i][0] * polygon[(i + 1) % len(polygon)][1]
                - polygon[(i + 1) % len(polygon)][0] * polygon[i][1]
                for i in range(len(polygon))
            )
        ) / 2.0

    def _point_in_convex_polygon(self, point: Point, polygon: Polygon) -> bool:
        """Return True when a point is inside or on a convex polygon boundary."""

        if len(polygon) < 3:
            return False

        sign = 0
        px, py = point
        for index, a in enumerate(polygon):
            b = polygon[(index + 1) % len(polygon)]
            cross = (b[0] - a[0]) * (py - a[1]) - (b[1] - a[1]) * (px - a[0])
            if abs(cross) <= self.eps:
                continue
            current_sign = 1 if cross > 0 else -1
            if sign == 0:
                sign = current_sign
            elif current_sign != sign:
                return False
        return True

    def _boxes_overlap(self, a: Drop, b: Drop) -> bool:
        ax1, ax2, ay1, ay2, az1, az2 = self._box_extents(a)
        bx1, bx2, by1, by2, bz1, bz2 = self._box_extents(b)

        return (
            ax1 < bx2 - self.eps
            and ax2 > bx1 + self.eps
            and ay1 < by2 - self.eps
            and ay2 > by1 + self.eps
            and az1 < bz2 - self.eps
            and az2 > bz1 + self.eps
        )

    def _box_extents(self, drop: Drop) -> Tuple[float, float, float, float, float, float]:
        x1 = self._x(drop)
        y1 = self._y(drop)
        z1 = self._z(drop)
        x2 = x1 + self._length(drop)
        y2 = y1 + self._width(drop)
        z2 = z1 + self._height(drop)
        return x1, x2, y1, y2, z1, z2

    def _xy_rect(self, drop: Drop) -> Rect:
        x1 = self._x(drop)
        x2 = x1 + self._length(drop)
        y1 = self._y(drop)
        y2 = y1 + self._width(drop)
        return x1, x2, y1, y2

    def _x(self, drop: Drop) -> float:
        return self._as_number(drop["position"]["x"], "position.x")

    def _y(self, drop: Drop) -> float:
        return self._as_number(drop["position"]["y"], "position.y")

    def _z(self, drop: Drop) -> float:
        return self._as_number(drop["position"]["z"], "position.z")

    def _length(self, drop: Drop) -> float:
        return self._as_number(drop["box"]["length"], "box.length")

    def _width(self, drop: Drop) -> float:
        return self._as_number(drop["box"]["width"], "box.width")

    def _height(self, drop: Drop) -> float:
        return self._as_number(drop["box"]["height"], "box.height")

    @staticmethod
    def _same_box_id(a: Drop, b: Drop) -> bool:
        return a.get("box", {}).get("id") == b.get("box", {}).get("id")

    @staticmethod
    def _require_mapping(data: Dict[str, Any], key: str, path: str) -> None:
        if key not in data:
            raise ValueError(f"Missing required {path}.{key}.")
        if not isinstance(data[key], dict):
            raise ValueError(f"{path}.{key} must be a dictionary.")

    @staticmethod
    def _as_number(value: Any, path: str) -> float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{path} must be a number, got {value!r}.")
        value = float(value)
        if not math.isfinite(value):
            raise ValueError(f"{path} must be finite, got {value!r}.")
        return value

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
