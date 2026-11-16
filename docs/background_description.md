# Warehouse picking process

A warehouse order picker drives through the warehouse using an order-picking truck. The truck has two forks capable of carrying one to three EUR pallets.

The picker receives an order from the warehouse management system (WMS) through a pick-by-voice device. The device instructs the picker where to stop and how many boxes to collect at each location. The picker follows a predetermined route through the warehouse, stopping only at locations specified by the pick-by-voice system.

Once the pallets are full or the order is complete, the picker lowers the pallets and wraps each stack in plastic film to prevent the boxes from falling. If a stack becomes unstable during picking, the picker may need to stop and partially wrap the pallet before continuing.

Products are arranged along the picking route so that heavier and larger boxes are generally collected before lighter and smaller ones, with weight taking priority over size. Consequently, even relatively simple stacking strategies can produce reasonably stable pallets, as heavier boxes tend to be placed near the bottom.

## Example

In this example, the order-picking truck carries two EUR pallets, and the picker receives an order consisting of 364 boxes.

The picker starts at the beginning of the predetermined route and drives through the warehouse aisles, stopping at locations containing products included in the order. The pick-by-voice system communicates the required locations and quantities.

An example interaction between the system and the picker might be:

- Pick-by-voice: "A1, three boxes."
- Picker: "Three, OK."
- Pick-by-voice: "A4, one box."
- Picker: "One, OK."
- Pick-by-voice: "A9, five boxes."

The picker continues along the route until the pallets are full or the order has been completed.

# Stacking Logic

Most orders contain similar types of boxes but differ in their quantities.

During training, pickers are taught to minimize gaps and maximize stack stability. Experienced pickers may also aim to create large, flat surfaces to support subsequent boxes.

Over time, pickers develop recurring stacking patterns, or "templates", that produce sufficiently good results for common orders. When these templates are unsuitable, they fall back on general stacking principles. Different pickers may develop different templates and strategies.

# Box Orientation

Boxes must generally remain upright due to their structural properties and stability requirements. Therefore, only rotations around the vertical axis are permitted, allowing two horizontal orientations for rectangular boxes.

# Picking Sequence and Temporary Storage

Although boxes can technically be collected in any order, deviating from the predetermined picking route increases travel distance, requires additional turns, and involves time-consuming interactions with the pick-by-voice system. Therefore, the picking sequence should remain unchanged.

In exceptional circumstances, the picker may skip a location if boxes are unavailable or cannot fit onto the existing pallets, returning later with an empty pallet.

Each collected box must be transferred onto the order-picking truck before the picker continues. However, the final placement sequence may differ from the picking sequence. An empty pallet may be used to temporarily store boxes for later placement, although additional handling should be minimized.

# Pallet Selection and Distribution

The picker may place boxes on any available pallet carried by the order-picking truck.

Pickers generally attempt to complete one pallet before starting another. However, for larger orders, they may distribute boxes from the beginning of the picking route across multiple pallets. These early boxes are typically heavier, larger, and well-suited for forming stable base layers. For example, some boxes occupy exactly one-quarter of a EUR pallet's surface.

The algorithm should allow boxes to be distributed between pallets rather than requiring one pallet to be completed before another is used.

# Placement Accessibility

Every placement must be physically accessible to the picker. A box cannot be placed in a location where surrounding boxes obstruct its insertion, even if sufficient space exists.

For example, a deep gap surrounded by tall stacks may be geometrically suitable but physically inaccessible.

# Stack Stability

Stacks must withstand the acceleration, braking, and turning forces experienced during transport.

Boxes may be partially supported or bridge gaps, provided that they remain mechanically stable and structurally supported. Interlocking between layers may improve stability but is not typically considered explicitly by human pickers.

Placing boxes across supporting surfaces of different heights is possible but not recommended, as it results in tilted placements.

Both geometric support and mechanical stability must be considered when evaluating placements.

# Rearrangement and Pallet Completion

Previously stacked boxes may be rearranged to accommodate new boxes, but rearrangement is time-consuming and should be avoided except in exceptional circumstances.

Partially filled pallets cannot be wrapped and unloaded before the order is complete. However, the picker may begin stacking on another available pallet while the others remain partially filled.

# Pallets

Standard EUR pallets are used.

Pallet load capacity is not considered an optimization constraint, as the boxes are not sufficiently dense to exceed it under normal stacking conditions.

The maximum permitted height of a loaded pallet is 180 cm, due to the loading platform doorway. This is a hard constraint.

Boxes must remain entirely within the horizontal boundaries of the pallet. No overhang is permitted.

# Boxes

Most boxes are rectangular cuboids capable of supporting substantial loads when upright. Some exceptions involve irregular shapes or limited load-bearing capacity.

For initial algorithm development, perfect knowledge of each box is assumed, including its dimensions, weight, identity, fragility, and permitted orientations.

The availability and accuracy of this information in the warehouse management system remain to be verified.
