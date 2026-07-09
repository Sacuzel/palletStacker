"""=====================================================================
PALLET  (role: stateful pallet POU / "function block")
=====================================================================
One Pallet instance is the equivalent of a function block instance
in structured text: it owns its internal state (the heightmap, the
feasibility map and the list of committed placements) and exposes
methods that read or update that state.

The Pallet knows NOTHING about heuristics or scoring - it can only
answer "would this exact placement be stable here?" (via the
stability POU) and "commit this placement". Choosing WHERE to place
is the job of algorithm.py.

Attribute names pallet_id / length / width / max_height /
max_stack_height / placements are consumed by
visualization_plotly.py and gazebo_exporter.py - do not rename.
====================================================================="""

from __future__ import annotations

from typing import List, Optional

import numpy as np

import config
from models import Box, Placement
from stability import StabilityResult, update_feasibility_map, validate_placement


class Pallet:
    """State + elementary operations of a single pallet."""

    def __init__(self, pallet_id: str) -> None:
        # ------------- static geometry (from config) -------------
        self.pallet_id: str = pallet_id
        self.length: float = float(config.PALLET_LENGTH)      # X [mm]
        self.width: float = float(config.PALLET_WIDTH)        # Y [mm]
        self.max_height: float = float(config.MAX_STACK_HEIGHT)  # Z limit [mm]

        # ------------- grid discretisation ------------------------
        self.grid_res: float = float(config.GRID_RESOLUTION_MM)
        self.nx: int = int(round(self.length / self.grid_res))
        self.ny: int = int(round(self.width / self.grid_res))

        # ------------- internal state (VAR of the FB) -------------
        # HM: current top height of the stack at each cell [mm].
        self.heightmap: np.ndarray = np.zeros((self.nx, self.ny), dtype=np.float64)
        # FM: True where the top surface is load-bearable.
        # Lemma III.1 of the paper: the pallet deck is one big LBCP,
        # so the map starts all-True.
        self.feasibility_map: np.ndarray = np.ones((self.nx, self.ny), dtype=bool)
        # All committed placements, in loading order.
        self.placements: List[Placement] = []

    # ===========================================================
    # READ-ONLY DERIVED VALUES (used by visualisation & stats)
    # ===========================================================
    @property
    def max_stack_height(self) -> float:
        """Current highest point of the load [mm]."""
        return float(self.heightmap.max())

    @property
    def total_weight_kg(self) -> float:
        """Payload mass currently on the pallet (boxes only) [kg]."""
        return sum(p.box.weight for p in self.placements)

    @property
    def used_volume_mm3(self) -> float:
        """Sum of the volumes of all packed boxes [mm^3]."""
        return sum(p.box.volume_mm3 for p in self.placements)

    @property
    def utilization(self) -> float:
        """Packed volume / (footprint * allowed height), 0..1."""
        envelope = self.length * self.width * self.max_height
        return self.used_volume_mm3 / envelope if envelope > 0 else 0.0

    # ===========================================================
    # COORDINATE HELPERS
    # ===========================================================
    def mm_to_cells(self, value_mm: float) -> int:
        """Convert a length in mm to a whole number of grid cells.

        Rounding (instead of truncation) keeps dimensions that are
        exact multiples of the resolution loss-free."""
        return int(round(value_mm / self.grid_res))

    # ===========================================================
    # PLACEMENT FEASIBILITY  (bounds + height + weight + Alg. 1)
    # ===========================================================
    def evaluate_placement(
        self,
        box: Box,
        x_mm: float,
        y_mm: float,
        length_mm: float,
        width_mm: float,
        height_mm: float,
    ) -> Optional[tuple[StabilityResult, float]]:
        """Full feasibility check of one candidate position.

        length_mm / width_mm / height_mm are the dimensions of the
        box IN THE CANDIDATE ORIENTATION (any of the up to 6
        axis-aligned orientations), so they may be any permutation
        of the box's own length/width/height.

        Returns (StabilityResult, support_z) if the placement is
        geometrically possible AND stable, otherwise None.

        Checks performed, in cheap-to-expensive order:
          1. footprint completely inside pallet edges (task rule:
             no overhang beyond pallet dimensions)
          2. payload limit of the pallet (optional, config)
          3. resulting top face below MAX_STACK_HEIGHT
          4. LBCP structural stability (Algorithm 1)
        """
        ix = self.mm_to_cells(x_mm)
        iy = self.mm_to_cells(y_mm)
        nx_cells = self.mm_to_cells(length_mm)
        ny_cells = self.mm_to_cells(width_mm)

        # --- 1. bounds check --------------------------------------
        if ix < 0 or iy < 0 or ix + nx_cells > self.nx or iy + ny_cells > self.ny:
            return None

        # --- 2. payload check -------------------------------------
        if (
            config.MAX_PALLET_PAYLOAD_KG is not None
            and self.total_weight_kg + box.weight > config.MAX_PALLET_PAYLOAD_KG
        ):
            return None

        # --- 3./4. height + stability (Algorithm 1) ---------------
        result = validate_placement(
            heightmap=self.heightmap,
            feasibility_map=self.feasibility_map,
            ix=ix,
            iy=iy,
            nx_cells=nx_cells,
            ny_cells=ny_cells,
            grid_res=self.grid_res,
            cog_tolerance=config.COG_TOLERANCE,
            min_support_ratio=config.MIN_SUPPORT_RATIO,
        )

        if result.support_z + height_mm > self.max_height + 1e-6:
            return None  # would exceed the allowed tower height

        if not result.valid:
            return None

        return result, result.support_z

    # ===========================================================
    # COMMIT  (updates HM + FM via Algorithm 2, appends placement)
    # ===========================================================
    def commit_placement(
        self,
        box: Box,
        x_mm: float,
        y_mm: float,
        length_mm: float,
        width_mm: float,
        height_mm: float,
        stability: StabilityResult,
    ) -> Placement:
        """Irreversibly place the box (online setting: no backtracking).

        length_mm / width_mm / height_mm are the oriented dimensions,
        exactly as passed to evaluate_placement(). The caller must
        have obtained `stability` from evaluate_placement() for the
        SAME coordinates and orientation."""
        ix = self.mm_to_cells(x_mm)
        iy = self.mm_to_cells(y_mm)
        nx_cells = self.mm_to_cells(length_mm)
        ny_cells = self.mm_to_cells(width_mm)
        z_mm = stability.support_z
        new_top = z_mm + height_mm

        # Algorithm 2: update heightmap + feasibility map in place.
        update_feasibility_map(
            heightmap=self.heightmap,
            feasibility_map=self.feasibility_map,
            ix=ix,
            iy=iy,
            nx_cells=nx_cells,
            ny_cells=ny_cells,
            grid_res=self.grid_res,
            new_top_z=new_top,
            support_polygon=stability.support_polygon,
        )

        placement = Placement(
            box=box,
            pallet_id=self.pallet_id,
            x=float(ix * self.grid_res),
            y=float(iy * self.grid_res),
            z=float(z_mm),
            length=float(nx_cells * self.grid_res),
            width=float(ny_cells * self.grid_res),
            height=float(height_mm),
        )
        self.placements.append(placement)
        return placement