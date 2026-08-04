# Online SKU-Group Pallet Packing with LBCP Stability Validation

Online (no-backtracking) pallet-loading program for grocery DC box
streams. Boxes arrive one by one in the order given in the input
JSON, each placement decision is final, and every placement is
proven statically stable before it is committed.

## File structure (POU-style: one file = one role)

| File                       | Role (PLC analogy)                                                    |
|----------------------------|-----------------------------------------------------------------------|
| `config.py`                | Global Variable List — every tunable parameter, no logic              |
| `models.py`                | DUTs — `Box` and `Placement` data types                               |
| `stability.py`             | Stability POU — Algorithm 1 (validation) + Algorithm 2 (map update) of Gao et al., arXiv:2507.09123 |
| `pallet.py`                | Pallet function block — owns heightmap, feasibility map, placements   |
| `algorithm.py`             | Packing POU — candidate generation, heuristic scoring, `pack_boxes()` |
| `main.py`                  | Main program — JSON in → pack → report → Plotly HTML + Gazebo SDF out |
| `verify_layout.py`         | Independent brute-force checker (overlap / floating / overhang / height) |
| `visualization_plotly.py`  | (yours, unchanged) interactive 3D HTML                                |
| `gazebo_exporter.py`       | (yours, unchanged) Gazebo SDF world + manifest                        |

## Usage

```bash
pip install numpy plotly
python main.py groceryBoxesSerialGroups.json          # pack + export
python main.py groceryBoxesSerialGroups.json --labels # box ids in 3D view
python verify_layout.py                               # independent sanity check
gz sim gazebo_runs/latest/pallet_towers.sdf           # physics validation
```

Outputs: `pallet_layout.html` (Plotly) and
`gazebo_runs/latest/pallet_towers.sdf` + `manifest.json` (Gazebo).

## Methods used (all literature-based, no ML)

**Stability — Load Bearable Convex Polygon (LBCP)** from Gao,
Wang, Kong & Chong (2025), Section III-C / Algorithms 1–2. The
pallet keeps a heightmap `HM` and a boolean feasibility map `FM`
marking which surface cells belong to an LBCP (the pallet deck
starts as one big LBCP, Lemma III.1). A candidate placement is
stable iff the CoG (geometric centre, plus the configurable
`COG_TOLERANCE` uncertainty of Eq. 1) lies inside the convex hull
of its *load-bearable* contact cells (Eq. 2 / Corollary III.2.1).
After commit, `FM` is updated so only the part of the new top face
above the support polygon is bearable (Theorem III.2 / Alg. 2).
The DRL policy, rearrangement planner (SRP/MCTS/A*) and unpacking
parts of the paper are intentionally omitted per the task.

**Placement heuristics**
- Corner/extreme-point candidate positions (Martello, Pisinger &
  Vigo 2000; Crainic, Perboli & Tadei 2008)
- All 6 axis-aligned orientations per box (3 vertical axes x 2 yaw
  rotations); restrict via `ALLOW_TIPPED_ORIENTATIONS` /
  `ALLOW_YAW_ROTATION` in config for "this side up" goods
- Heightmap minimisation scoring (Wang & Hauser 2019) → implicit
  layer building (Elhedhli, Gzara & Yildiz 2019)
- Support-ratio preference and minimum support-area constraint
  (Ramos, Oliveira & Lopes 2016; Bischoff & Ratcliff-style)
- Best-match / snugness reward (Li & Zhang 2015)
- Same-SKU column stacking bonus (classic grocery palletising
  pattern, exploits the grouped arrival order)
- Deepest-Bottom-Left tie-break (Karabulut & İnceoğlu 2004)
- First Fit over open pallets (Johnson 1973); new pallet opens only
  when no open pallet can take the box stably

## Result on the provided dataset

120/120 boxes packed, 3 pallets, volume utilisation 76.8 % /
77.4 % / 65.2 % (last pallet still open), overall 73.1 % of the
1200×800×1800 mm envelope. Independent checker reports zero
overlaps, floating boxes, overhangs or height violations.
`MIN_SUPPORT_RATIO = 0.50` was tuned empirically: it both tightens
stability and reduced the pallet count from 4 to 3.
## Gazebo Euro pallet and controllable forklift

The generated world now contains the dynamic simplified Euro pallet and a
three-link forklift (one body plus two vertically sliding forks). Forklift
controls are provided by a small modifier-aware Gazebo GUI plugin.

Build the plugin once:

```bash
./plugins/forklift_teleop/build_and_install.sh
```

Run the packer and simulation:

```bash
python main.py
gz sim gazebo_runs/latest/pallet_towers.sdf
```

Controls: arrows drive and turn, Shift+Up / Shift+Down move the forks, and Space
stops all motion. See `README_FORKLIFT_SDF.md` for geometry, dependencies,
configuration, and validation details.
