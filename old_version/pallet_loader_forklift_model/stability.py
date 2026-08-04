"""=====================================================================
STABILITY  (role: stability validation POU)
=====================================================================
Implements the Load Bearable Convex Polygon (LBCP) stability method
from:

    Gao, Wang, Kong, Chong: "Online 3D Bin Packing with Fast
    Stability Validation and Stable Rearrangement Planning",
    arXiv:2507.09123 (2025)

Two entry points, mirroring the paper:

    validate_placement(...)        -> Algorithm 1 (Structural
                                      Stability Validation)
    update_feasibility_map(...)    -> Algorithm 2 (Structural
                                      Stability Update)

The bin state is represented exactly as in the paper by two 2D
grids aligned with the pallet surface:

    HM (heightmap)        float [mm]  - top height of the stack at
                                        every (x, y) cell
    FM (feasibility map)  bool        - True where the current top
                                        surface belongs to an LBCP,
                                        i.e. can bear ANY vertical
                                        load without toppling

Per Lemma III.1 of the paper the pallet deck itself is one big
LBCP, so FM is initialised to all-True by the Pallet module.

Sign convention difference vs. the paper: the paper's heightmap is
a camera DISTANCE field, so their "min HM" corresponds to OUR
"max HM" (height measured upward from the deck).
====================================================================="""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Sequence, Tuple

import numpy as np

Point = Tuple[float, float]


# ===============================================================
# RESULT TYPE
# ===============================================================
@dataclass
class StabilityResult:
    """Return value of Algorithm 1 - everything the caller needs to
    either reject the placement or commit it and run Algorithm 2."""

    valid: bool                    # final stability flag
    support_z: float               # resting height h_s of the box [mm]
    support_ratio: float           # bearable contact cells / footprint cells
    support_polygon: List[Point]   # convex hull vertices (pallet coords, mm)
    reason: str                    # human-readable reject reason ("" if valid)


# ===============================================================
# GEOMETRY HELPERS (pure functions, no state)
# ===============================================================
def convex_hull(points: Sequence[Point]) -> List[Point]:
    """Andrew's monotone-chain convex hull, O(n log n).

    This is the CH({...}) operator of Eq. 2 in the paper. Returns
    the hull vertices in counter-clockwise order. Degenerate inputs
    (a single point or a straight line) return the input extremes,
    which the caller treats as an unstable/empty polygon for any
    2D CoG region test.
    """
    pts = sorted(set(points))
    if len(pts) <= 2:
        return list(pts)

    def cross(o: Point, a: Point, b: Point) -> float:
        # z-component of (a-o) x (b-o); >0 means left turn
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower: List[Point] = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)

    upper: List[Point] = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)

    return lower[:-1] + upper[:-1]


def point_in_convex_polygon(pt: Point, hull_ccw: Sequence[Point]) -> bool:
    """True if pt lies inside or on the border of a CCW convex hull.

    Implements the 'CoG inside support polygon' test of Alg. 1
    (line 18). Points exactly on an edge count as inside, which is
    consistent with static equilibrium (marginally stable).
    """
    n = len(hull_ccw)
    if n < 3:
        return False  # degenerate hull cannot enclose a 2D region
    px, py = pt
    for i in range(n):
        ax, ay = hull_ccw[i]
        bx, by = hull_ccw[(i + 1) % n]
        # Cross product must be >= 0 for every edge of a CCW hull.
        if (bx - ax) * (py - ay) - (by - ay) * (px - ax) < -1e-9:
            return False
    return True


def _cells_to_corner_points(
    cells_mask: np.ndarray, ix0: int, iy0: int, res: float
) -> List[Point]:
    """Convert True cells of a footprint mask into hull candidate
    points (the 4 physical corners of each cell, in mm).

    Optimisation: only the leftmost and rightmost True cell of each
    grid row can contribute to the convex hull, so only their
    corners are emitted. This keeps the hull input small even for
    large contact areas (O(rows) instead of O(cells))."""
    points: List[Point] = []
    nx, ny = cells_mask.shape
    for iy in range(ny):
        row = cells_mask[:, iy]
        xs = np.flatnonzero(row)
        if xs.size == 0:
            continue
        for ix in (int(xs[0]), int(xs[-1])):
            x_mm = (ix0 + ix) * res
            y_mm = (iy0 + iy) * res
            # all four corners of the square cell
            points.extend(
                [
                    (x_mm, y_mm),
                    (x_mm + res, y_mm),
                    (x_mm, y_mm + res),
                    (x_mm + res, y_mm + res),
                ]
            )
    return points


# ===============================================================
# ALGORITHM 1 - STRUCTURAL STABILITY VALIDATION
# ===============================================================
def validate_placement(
    *,
    heightmap: np.ndarray,       # HM_t  [mm], shape (nx, ny)
    feasibility_map: np.ndarray, # FM_t  bool, shape (nx, ny)
    ix: int,                     # footprint min corner, cell index X
    iy: int,                     # footprint min corner, cell index Y
    nx_cells: int,               # footprint size in cells, X
    ny_cells: int,               # footprint size in cells, Y
    grid_res: float,             # cell size [mm]
    cog_tolerance: float,        # delta_CoG of Eq. 1 (0.0 .. 0.5)
    min_support_ratio: float,    # extra industrial constraint (config)
    height_tol: float = 1e-6,    # cells within this of max count as contact
) -> StabilityResult:
    """Algorithm 1 of the paper, adapted to an upward heightmap.

    Steps (paper line numbers in brackets):
      1. slice the heightmap window under the footprint        [8]
      2. support height h_s = max of the window                [8]
      3. contact cells  = cells at h_s                         [10]
      4. bearable cells = contact cells that are True in FM    [12]
      5. support polygon = convex hull of bearable cells       [14]
      6. CoG uncertainty square from cog_tolerance (Eq. 1)     [16]
      7. stable iff every CoG extreme lies inside the hull     [18]
    """
    window_hm = heightmap[ix : ix + nx_cells, iy : iy + ny_cells]
    window_fm = feasibility_map[ix : ix + nx_cells, iy : iy + ny_cells]

    # --- (2) resting height of the box bottom face -------------
    support_z = float(window_hm.max())

    # --- (3) geometric contact cells ----------------------------
    contact_mask = window_hm >= support_z - height_tol

    # --- (4) keep only load-bearable contact (belongs to an LBCP)
    bearable_mask = contact_mask & window_fm

    footprint_cells = nx_cells * ny_cells
    support_ratio = float(bearable_mask.sum()) / float(footprint_cells)

    if not bearable_mask.any():
        return StabilityResult(False, support_z, 0.0, [], "no load-bearable contact")

    # Extra practical constraint on top of the paper's criterion
    # (disabled when MIN_SUPPORT_RATIO = 0.0 in config).
    if support_ratio < min_support_ratio:
        return StabilityResult(
            False, support_z, support_ratio, [],
            f"support ratio {support_ratio:.2f} < min {min_support_ratio:.2f}",
        )

    # --- (5) support polygon P_new = CH(contact ∩ LBCPs) --------
    hull_points = _cells_to_corner_points(bearable_mask, ix, iy, grid_res)
    hull = convex_hull(hull_points)

    # --- (6) CoG uncertainty set C_new (Eq. 1) ------------------
    # Nominal CoG = geometric centre of the footprint. With
    # tolerance t, the CoG can be anywhere in a rectangle of
    # half-size (t*L, t*W) around it; testing the 4 extreme
    # corners of that rectangle is sufficient because both the
    # rectangle and the support polygon are convex.
    len_mm = nx_cells * grid_res
    wid_mm = ny_cells * grid_res
    cx = ix * grid_res + len_mm / 2.0
    cy = iy * grid_res + wid_mm / 2.0
    dx = cog_tolerance * len_mm
    dy = cog_tolerance * wid_mm
    cog_extremes: List[Point] = [
        (cx - dx, cy - dy),
        (cx + dx, cy - dy),
        (cx + dx, cy + dy),
        (cx - dx, cy + dy),
    ]

    # --- (7) inclusion test -------------------------------------
    for pt in cog_extremes:
        if not point_in_convex_polygon(pt, hull):
            return StabilityResult(
                False, support_z, support_ratio, hull,
                "CoG (incl. tolerance) outside support polygon",
            )

    return StabilityResult(True, support_z, support_ratio, hull, "")


# ===============================================================
# ALGORITHM 2 - STRUCTURAL STABILITY UPDATE
# ===============================================================
def update_feasibility_map(
    *,
    heightmap: np.ndarray,
    feasibility_map: np.ndarray,
    ix: int,
    iy: int,
    nx_cells: int,
    ny_cells: int,
    grid_res: float,
    new_top_z: float,
    support_polygon: Sequence[Point],
) -> None:
    """Algorithm 2 of the paper: commit a placement into HM and FM.

    Both maps are modified IN PLACE (they are the state of the
    owning Pallet, like VAR_IN_OUT in structured text).

    1. HM: the whole footprint is raised to the new top face.
    2. FM: the footprint first becomes non-bearable (the old
       surface is buried), then the region of the NEW top face
       that lies vertically above the support polygon is marked
       bearable again. By Theorem III.2 of the paper that region
       is itself an LBCP: any load whose weight passes through it
       is routed straight down into already-bearable structure.
       For a box sitting flat on an equal-size box (the common
       same-SKU column case) this makes the entire top face
       bearable, exactly like Lemma III.1.
    """
    # --- (1) raise the heightmap --------------------------------
    heightmap[ix : ix + nx_cells, iy : iy + ny_cells] = new_top_z

    # --- (2) rebuild feasibility inside the footprint ------------
    window_fm = feasibility_map[ix : ix + nx_cells, iy : iy + ny_cells]
    window_fm[:, :] = False

    # Rasterise the support polygon onto cell centres.
    for local_ix in range(nx_cells):
        x_mm = (ix + local_ix + 0.5) * grid_res
        for local_iy in range(ny_cells):
            y_mm = (iy + local_iy + 0.5) * grid_res
            if point_in_convex_polygon((x_mm, y_mm), support_polygon):
                window_fm[local_ix, local_iy] = True
