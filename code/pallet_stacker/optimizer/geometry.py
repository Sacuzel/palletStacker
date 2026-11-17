"""Shared 2D geometry for the loader optimizer.

Used by
-------
* emsDecoder.py  - static stability of a candidate placement (support polygon)
* evaluator.py   - dynamic stability margin (tilted-gravity check)

The first three functions are the hull code of emsLoader.py, moved here
unchanged so that the decoder reproduces emsLoader exactly:

    convex_hull               Andrew's monotone chain
    point_in_convex_polygon   vector-product test, boundary counts as inside
    polygon_area              shoelace formula

The last two are new and only used by the evaluator:

    polygon_edges             pre-computes edge data of a support polygon
    axis_exit_distances       how far a point can move along +X, -X, +Y, -Y
                              before it leaves the polygon

All coordinates are millimetres in pallet-local X/Y.
"""

from __future__ import annotations

from collections.abc import Sequence
from math import hypot

Point2D = tuple[float, float]

# One polygon edge as (ax, ay, nx, ny): a point on the edge and the unit
# normal pointing to the inside of the polygon.
Edge = tuple[float, float, float, float]

# A point is still "inside" when it is this far outside an edge. Same value
# as in emsLoader's vector-product test.
BOUNDARY_EPS: float = 1e-9

# Edge normals with a smaller component than this are treated as parallel to
# the axis in axis_exit_distances.
_PARALLEL_EPS: float = 1e-12


# ============================================================================
# HULL CODE (identical to emsLoader.py)
# ============================================================================

def convex_hull(points: Sequence[Point2D]) -> list[Point2D]:
    """Andrew's monotone chain; counter-clockwise, collinear points removed."""
    pts = sorted(set(points))
    if len(pts) <= 2:
        return list(pts)

    def cross(o: Point2D, a: Point2D, b: Point2D) -> float:
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower: list[Point2D] = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper: list[Point2D] = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def point_in_convex_polygon(pt: Point2D, hull: Sequence[Point2D]) -> bool:
    """Vector-product test; the boundary counts as inside."""
    if len(hull) < 3:
        return False
    px, py = pt
    for i in range(len(hull)):
        ax, ay = hull[i]
        bx, by = hull[(i + 1) % len(hull)]
        if (bx - ax) * (py - ay) - (by - ay) * (px - ax) < -BOUNDARY_EPS:
            return False
    return True


def polygon_area(hull: Sequence[Point2D]) -> float:
    """Shoelace formula."""
    if len(hull) < 3:
        return 0.0
    total = 0.0
    for i in range(len(hull)):
        ax, ay = hull[i]
        bx, by = hull[(i + 1) % len(hull)]
        total += ax * by - bx * ay
    return abs(total) / 2.0


# ============================================================================
# SUPPORT-POLYGON MARGINS (evaluator only)
# ============================================================================

def polygon_edges(hull: Sequence[Point2D]) -> tuple[Edge, ...]:
    """Edge data of a counter-clockwise convex polygon.

    Returns one (ax, ay, nx, ny) tuple per edge, where (nx, ny) is the unit
    normal pointing into the polygon. A degenerate polygon (fewer than three
    corners) has no edges.
    """
    if len(hull) < 3:
        return ()
    edges: list[Edge] = []
    for i in range(len(hull)):
        ax, ay = hull[i]
        bx, by = hull[(i + 1) % len(hull)]
        ex, ey = bx - ax, by - ay
        length = hypot(ex, ey)
        if length <= 0.0:
            continue
        # Counter-clockwise polygon: the inside is to the left of the edge.
        edges.append((ax, ay, -ey / length, ex / length))
    return tuple(edges)


def axis_exit_distances(
    px: float,
    py: float,
    edges: Sequence[Edge],
) -> tuple[float, float, float, float]:
    """Distance from (px, py) to the polygon boundary along +X, -X, +Y, -Y.

    * Point inside the polygon: four non-negative distances [mm].
    * Point outside the polygon: all four values are the (negative) distance
      to the violated edge, so the caller sees a negative margin.
    * Degenerate polygon: all zeros (no margin).
    """
    if not edges:
        return (0.0, 0.0, 0.0, 0.0)

    inf = float("inf")
    x_pos = x_neg = y_pos = y_neg = inf
    inside = inf

    for ax, ay, nx, ny in edges:
        # Signed distance of the point to this edge (>= 0 means inside).
        s = nx * (px - ax) + ny * (py - ay)
        if s < inside:
            inside = s
        # Moving along +X changes s by nx per millimetre, and so on.
        if nx < -_PARALLEL_EPS:
            t = s / -nx
            if t < x_pos:
                x_pos = t
        elif nx > _PARALLEL_EPS:
            t = s / nx
            if t < x_neg:
                x_neg = t
        if ny < -_PARALLEL_EPS:
            t = s / -ny
            if t < y_pos:
                y_pos = t
        elif ny > _PARALLEL_EPS:
            t = s / ny
            if t < y_neg:
                y_neg = t

    if inside < -BOUNDARY_EPS:
        return (inside, inside, inside, inside)
    return (x_pos, x_neg, y_pos, y_neg)