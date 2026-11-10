# Algorithm Functional Requirements

## Development Phases

The stacking optimization algorithm will be developed in two phases.

- Phase 1: Simplified Algorithm
  - The first phase focuses on optimizing the placement of rectangular boxes with simplified structural properties.
  - The algorithm must respect the predetermined picking sequence and produce physically accessible, mechanically stable pallet stacks without rearranging previously placed boxes.

- Phase 2: Extended Algorithm
  - The second phase extends the algorithm to support more complex box geometries, orientation-dependent structural limitations, and temporary storage of boxes.
  - All Phase 1 requirements remain applicable unless explicitly modified.

## Box Orientation

### Phase 1

- Boxes must remain upright.
- Only rotations around the vertical axis are permitted, resulting in two possible horizontal orientations for rectangular boxes.

### Phase 2

- Boxes may be rotated into any orientation.
- Each box face has a specified maximum permissible load.
- The algorithm must account for these limits when determining valid orientations and placements.
- Placements that exceed the structural limits of any box are prohibited.

## Picking Sequence and Temporary Storage

### Phase 1

- Boxes must be processed in the predetermined picking sequence.
- Each box must be placed directly into its final position before the next box is processed.
- Previously placed boxes cannot be moved.

### Phase 2

- The picking sequence must remain unchanged.
- An empty pallet may be used to temporarily store up to three boxes for later placement.
- Temporarily stored boxes must eventually be moved to their final positions.
- The algorithm must account for the space occupied by temporarily stored boxes.
- Temporary storage should be minimized to avoid unnecessary handling.

## Pallet Selection and Distribution

### Both phases

- The algorithm must support multiple pallets carried simultaneously by the order-picking truck.
- Each box may be assigned to any available pallet.
- Boxes may be distributed between pallets without requiring one pallet to be completed before another is used.
- The algorithm must track the current arrangement of boxes on each pallet.

## Placement Accessibility

### Both phases

- Every proposed placement must be physically accessible to the picker.
- The algorithm must ensure that a box can be moved into its intended position without intersecting previously placed boxes.
- Sufficient free space must exist along a feasible insertion path.
- A placement is invalid if surrounding boxes prevent the picker from inserting the box, even if the final position is geometrically feasible.

## Stack Stability

### Phase 1

- Every intermediate and final stack must remain stable under the acceleration, braking, and turning forces experienced during transport.
- The algorithm must evaluate stability using specified acceleration limits:
  - Forward acceleration: 2.0 m/s²
  - Braking deceleration: 3.0 m/s²
  - Lateral acceleration during turning: 2.0 m/s²
- Boxes may be partially supported or bridge gaps between supporting boxes, provided that mechanical stability is maintained.
- Interlocking arrangements between boxes are permitted.
- Supporting surfaces must be at approximately the same height. Differences in support height must not exceed a specified tolerance: ±5 mm
- Placements that result in unstable or inadequately supported boxes are prohibited.

### Phase 2

- The algorithm must additionally evaluate the risk of individual boxes sliding relative to their supporting surfaces during acceleration, braking, and turning.
- Friction coefficients between contacting surfaces are assumed to be known.
- Sliding stability must be evaluated using the frictional forces available at each contact interface and the forces acting on the boxes.
- Placements that allow individual boxes to slide under the specified acceleration limits are prohibited.

## Rearrangement and Pallet Completion

### Phase 1

- Boxes placed in their final positions cannot be rearranged.
- Partially filled pallets cannot be unloaded before the order is completed.
- The algorithm must construct the pallet stacks using only the pallets available on the order-picking truck.

### Phase 2

- Moving a box from temporary storage to its final position is permitted and is not considered rearrangement.

## Pallet Constraints

### Both phases

- Standard EUR pallets must be used.
- Pallet dimensions:
  - Length: 1200 mm
  - Width: 800 mm
  - Pallet height: 144 mm
- Maximum permitted loaded pallet height: 1800 mm beginning from the pallet.
- Boxes must remain entirely within the horizontal boundaries of the pallet.
- Overhanging boxes are prohibited.
- Pallet load capacity is excluded from the optimization constraints.

## Box Properties

### Phase 1

- All boxes are assumed to be rectangular cuboids.
- Boxes are assumed to withstand the loads encountered when stacked upright.
- Perfect knowledge of box dimensions, weights, and identities is assumed.
- Box deformation is not considered.

### Phase 2

- The algorithm must support boxes with non-rectangular geometries.
- Boxes may have limited load-bearing capacity.
- Structural limitations must be considered when evaluating placements and orientations.
- Perfect knowledge of box geometries, weights, and structural properties is assumed.
