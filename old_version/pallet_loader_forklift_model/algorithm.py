"""=====================================================================
ALGORITHM  (role: packing logic POU / "PRG" of the packing station)
=====================================================================
Online pallet-loading algorithm. Boxes arrive one at a time in the
order given by the input file (grouped by SKU, like totes coming
off a grocery DC conveyor) and every decision is final - there is
no backtracking or rearranging, matching a human operator.

The algorithm is a combination of well-established heuristics from
the container/pallet-loading literature (no machine learning):

  * CANDIDATE POSITIONS - "corner points" (Martello, Pisinger &
    Vigo 2000; generalised by the extreme-point rule of Crainic,
    Perboli & Tadei 2008): a new box is only ever tried at
    positions where its edges align with the pallet walls or with
    the edges of already-placed boxes. Optimal-quality placements
    live on this small candidate set, which keeps the online
    decision fast.

  * ORIENTATIONS - all 6 axis-aligned orientations of the cuboid
    are tried (3 choices of vertical axis x 2 yaw rotations), as
    in general 3D-BPP heuristics (e.g. the EMS methods of Parreno
    et al. 2008, Ha et al. 2017). Both degrees of freedom can be
    restricted in config (ALLOW_TIPPED_ORIENTATIONS,
    ALLOW_YAW_ROTATION) for "this side up" goods.

  * SCORING - a weighted combination of:
      - Heightmap minimisation (Wang & Hauser 2019): prefer the
        placement that keeps the load surface as low as possible.
        This implicitly builds full horizontal layers first, the
        pattern favoured for stability by layer-building methods
        (Elhedhli, Gzara & Yildiz 2019).
      - Support-ratio maximisation: prefer well-supported
        positions (static mechanical equilibrium preference of
        Ramos, Oliveira & Lopes 2016).
      - Snugness / best-match-first (Li & Zhang 2015): prefer
        positions touching walls or existing boxes, which reduces
        fragmented gaps.
      - Same-SKU column building: identical boxes stacked into
        aligned columns give perfect interlock and full-area
        support - the classic column-stacking pattern of grocery
        palletising (Bischoff & Ratcliff 1995 family). Because
        boxes arrive in SKU groups, this bonus turns each group
        into stable towers whenever that does not waste height.
      - Deepest-Bottom-Left tie-breaking (Karabulut & Inceoglu
        2004): deterministic preference for low y, then low x.

  * STABILITY - every candidate is filtered through the LBCP
    validation (Algorithm 1) implemented in stability.py, so ONLY
    provably stable placements are ever scored. This mirrors the
    action-masking role the paper gives the validator inside its
    DRL pipeline, with the learned policy replaced by the
    deterministic scoring above.

  * NEW PALLET RULE - first-fit over open pallets: the box goes to
    the first (oldest) open pallet that can take it stably; if
    none can, a new pallet is opened (classic First Fit from
    one-dimensional bin packing, Johnson 1973). Set
    config.FIRST_FIT_OPEN_PALLETS = False to only ever consider
    the newest pallet.
====================================================================="""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import config
from models import Box
from pallet import Pallet
from stability import StabilityResult


# ===============================================================
# INTERNAL TYPE: one fully evaluated candidate placement
# ===============================================================
@dataclass
class _Candidate:
    x_mm: float                # min corner X of the footprint
    y_mm: float                # min corner Y of the footprint
    length_mm: float           # oriented X size
    width_mm: float            # oriented Y size
    height_mm: float           # oriented Z size
    stability: StabilityResult # Alg. 1 output (contains support_z)
    score: float               # lower = better


# ===============================================================
# ORIENTATION GENERATION
# ===============================================================
def _orientations(box: Box) -> List[Tuple[float, float, float]]:
    """List of unique axis-aligned orientations (length_x, width_y,
    height_z) to try for this box, driven by the config flags:

      ALLOW_TIPPED_ORIENTATIONS  ALLOW_YAW_ROTATION  orientations
      ------------------------------------------------------------
      True                       True                all 6
      True                       False               3 (each dim up,
                                                     no yaw swap)
      False                      True                2 (upright+yaw)
      False                      False               1 (as delivered)

    Duplicate orientations (boxes with two or three equal
    dimensions, e.g. a cube) are removed so no time is wasted
    re-evaluating identical footprints."""
    l, w, h = box.length, box.width, box.height

    if config.ALLOW_TIPPED_ORIENTATIONS:
        # one entry per choice of vertical axis: (x, y, z-up)
        base = [(l, w, h), (l, h, w), (h, w, l)]
    else:
        base = [(l, w, h)]

    oriented: List[Tuple[float, float, float]] = []
    for lx, wy, hz in base:
        oriented.append((lx, wy, hz))
        if config.ALLOW_YAW_ROTATION:
            oriented.append((wy, lx, hz))  # 90 deg yaw: swap x/y

    # de-duplicate while preserving order
    unique: List[Tuple[float, float, float]] = []
    for o in oriented:
        if o not in unique:
            unique.append(o)
    return unique


# ===============================================================
# CANDIDATE POSITION GENERATION (corner / extreme points)
# ===============================================================
def _candidate_coordinates(pallet: Pallet, length_mm: float, width_mm: float) -> Tuple[List[float], List[float]]:
    """Build the sets of candidate x and y coordinates for a box of
    the given oriented footprint.

    For each axis the candidates are:
      * 0                               (pallet wall, low side)
      * pallet_size - box_size          (pallet wall, high side)
      * every existing box min edge     (stack exactly on top /
                                         left-align against it)
      * every existing box max edge     (place flush next to it)
      * every existing box max edge - box_size
                                        (right-align on top of it)
    Duplicates and out-of-bounds values are removed. This is the
    corner-point rule: an optimal axis-aligned packing can always
    be normalised so every box touches a wall or another box on
    its low sides, so nothing useful is lost by this restriction.
    """
    xs = {0.0, pallet.length - length_mm}
    ys = {0.0, pallet.width - width_mm}

    for p in pallet.placements:
        xs.update((p.x, p.x + p.length, p.x + p.length - length_mm))
        ys.update((p.y, p.y + p.width, p.y + p.width - width_mm))

    grid = pallet.grid_res
    # snap to the grid and keep only positions where the box fits
    xs_ok = sorted(
        {round(v / grid) * grid for v in xs if -1e-6 <= v <= pallet.length - length_mm + 1e-6}
    )
    ys_ok = sorted(
        {round(v / grid) * grid for v in ys if -1e-6 <= v <= pallet.width - width_mm + 1e-6}
    )
    return xs_ok, ys_ok


# ===============================================================
# SNUGNESS MEASURE (best-match-first term)
# ===============================================================
def _snugness(
    pallet: Pallet, ix: int, iy: int, nx_cells: int, ny_cells: int, support_z: float
) -> float:
    """Fraction (0..1) of the four side faces that are laterally
    supported at the resting level - either by a pallet wall or by
    neighbouring stack cells at least as high as the box bottom.
    Touching neighbours prevent sliding and avoid unusable slivers
    of space, hence the reward in the score."""
    hm = pallet.heightmap
    touching = 0

    # -X side: wall or neighbouring column height >= support level
    if ix == 0:
        touching += 1
    elif float(hm[ix - 1, iy : iy + ny_cells].max()) > support_z + 1e-6:
        touching += 1
    # +X side
    if ix + nx_cells == pallet.nx:
        touching += 1
    elif float(hm[ix + nx_cells, iy : iy + ny_cells].max()) > support_z + 1e-6:
        touching += 1
    # -Y side
    if iy == 0:
        touching += 1
    elif float(hm[ix : ix + nx_cells, iy - 1].max()) > support_z + 1e-6:
        touching += 1
    # +Y side
    if iy + ny_cells == pallet.ny:
        touching += 1
    elif float(hm[ix : ix + nx_cells, iy + ny_cells].max()) > support_z + 1e-6:
        touching += 1

    return touching / 4.0


# ===============================================================
# SAME-SKU COLUMN DETECTION
# ===============================================================
def _is_same_sku_column(
    pallet: Pallet, box: Box, x_mm: float, y_mm: float,
    length_mm: float, width_mm: float, support_z: float,
) -> bool:
    """True if the candidate sits EXACTLY on top of an identical
    same-SKU box (same footprint, same x/y, top face at support_z).
    Such column stacking gives full-area support and is the
    standard pattern for homogeneous grocery cases."""
    for p in pallet.placements:
        if (
            p.box.sku == box.sku
            and abs(p.x - x_mm) < 1e-6
            and abs(p.y - y_mm) < 1e-6
            and abs(p.length - length_mm) < 1e-6
            and abs(p.width - width_mm) < 1e-6
            and abs(p.top_z - support_z) < 1e-6
        ):
            return True
    return False


# ===============================================================
# BEST PLACEMENT ON ONE PALLET
# ===============================================================
def find_best_placement(pallet: Pallet, box: Box) -> Optional[_Candidate]:
    """Evaluate all candidate positions/orientations of `box` on
    `pallet` and return the best stable one, or None if the pallet
    cannot take the box at all.

    Score (minimised):
        W_TOP_HEIGHT     * resulting top height   [mm]
      - W_SUPPORT        * support ratio          [0..1]
      - W_SNUGNESS       * snugness               [0..1]
      - W_SAME_SKU_ALIGN * same-SKU column bonus  {0,1}
      + tiny * y + tinier * x                     (DBL tie-break)
    """
    # --- orientations to try (up to 6, see _orientations) ---------
    best: Optional[_Candidate] = None

    for length_mm, width_mm, height_mm in _orientations(box):
        xs, ys = _candidate_coordinates(pallet, length_mm, width_mm)
        nx_cells = pallet.mm_to_cells(length_mm)
        ny_cells = pallet.mm_to_cells(width_mm)

        for x_mm in xs:
            for y_mm in ys:
                evaluation = pallet.evaluate_placement(
                    box, x_mm, y_mm, length_mm, width_mm, height_mm
                )
                if evaluation is None:
                    continue  # out of bounds / too high / unstable
                stab, support_z = evaluation

                ix = pallet.mm_to_cells(x_mm)
                iy = pallet.mm_to_cells(y_mm)

                # ---- assemble the heuristic score ----------------
                top_height = support_z + height_mm
                score = config.W_TOP_HEIGHT * top_height
                score -= config.W_SUPPORT * stab.support_ratio
                score -= config.W_SNUGNESS * _snugness(
                    pallet, ix, iy, nx_cells, ny_cells, support_z
                )
                if _is_same_sku_column(
                    pallet, box, x_mm, y_mm, length_mm, width_mm, support_z
                ):
                    score -= config.W_SAME_SKU_ALIGN
                # Deepest-bottom-left deterministic tie-break
                score += 1e-3 * y_mm + 1e-6 * x_mm

                if best is None or score < best.score:
                    best = _Candidate(
                        x_mm, y_mm, length_mm, width_mm, height_mm, stab, score
                    )

    return best


# ===============================================================
# MAIN ENTRY POINT: ONLINE PACKING LOOP
# ===============================================================
def pack_boxes(boxes: Sequence[Box]) -> List[Pallet]:
    """Pack the boxes ONE BY ONE, in arrival order, onto pallets.

    This is the function imported by gazebo_exporter.py and main.py.

    Behaviour:
      * each box is offered to the open pallets (first-fit order,
        see config.FIRST_FIT_OPEN_PALLETS);
      * the best stable placement on the accepting pallet is
        committed immediately (online, no backtracking);
      * if no open pallet can take the box, a new pallet is opened;
      * a box that does not fit even on an empty pallet (too large
        or too heavy) is rejected with a console warning - it
        physically cannot be palletised under the given limits.
    """
    pallets: List[Pallet] = []
    rejected: List[Box] = []

    for box in boxes:
        placed = False

        # ---- which pallets may accept this box -------------------
        if config.FIRST_FIT_OPEN_PALLETS:
            open_pallets = pallets            # first fit over all
        else:
            open_pallets = pallets[-1:]       # only the newest one

        # ---- try existing pallets ---------------------------------
        for pallet in open_pallets:
            candidate = find_best_placement(pallet, box)
            if candidate is not None:
                pallet.commit_placement(
                    box,
                    candidate.x_mm,
                    candidate.y_mm,
                    candidate.length_mm,
                    candidate.width_mm,
                    candidate.height_mm,
                    candidate.stability,
                )
                placed = True
                break

        # ---- open a fresh pallet if needed ------------------------
        if not placed:
            new_pallet = Pallet(pallet_id=f"P{len(pallets) + 1:02d}")
            pallets.append(new_pallet)
            candidate = find_best_placement(new_pallet, box)
            if candidate is not None:
                new_pallet.commit_placement(
                    box,
                    candidate.x_mm,
                    candidate.y_mm,
                    candidate.length_mm,
                    candidate.width_mm,
                    candidate.height_mm,
                    candidate.stability,
                )
                placed = True
            else:
                # Box larger than the pallet / above payload limit.
                print(
                    f"[WARN] Box {box.identifier} "
                    f"({box.length:.0f}x{box.width:.0f}x{box.height:.0f} mm, "
                    f"{box.weight:.1f} kg) cannot be placed on an empty "
                    f"pallet and was REJECTED."
                )
                rejected.append(box)
                # remove the pallet again if it stayed empty
                if not new_pallet.placements:
                    pallets.pop()

    return pallets