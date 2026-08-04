"""Independent sanity checker: re-derives physical validity of the
final layout WITHOUT using the heightmap (brute force box-vs-box),
so bugs in the maps would be caught here."""
import itertools
import config
from main import load_boxes_from_json
from algorithm import pack_boxes

boxes = load_boxes_from_json("groceryBoxesSerialGroups.json")
pallets = pack_boxes(boxes)

errors = 0
for pallet in pallets:
    ps = pallet.placements
    # bounds + height
    for p in ps:
        if p.x < -1e-6 or p.y < -1e-6 or p.x + p.length > pallet.length + 1e-6 or p.y + p.width > pallet.width + 1e-6:
            print(f"OVERHANG: {p.box.identifier} on {pallet.pallet_id}"); errors += 1
        if p.z + p.height > pallet.max_height + 1e-6:
            print(f"TOO TALL: {p.box.identifier} on {pallet.pallet_id}"); errors += 1
    # pairwise overlap
    for a, b in itertools.combinations(ps, 2):
        ox = min(a.x + a.length, b.x + b.length) - max(a.x, b.x)
        oy = min(a.y + a.width, b.y + b.width) - max(a.y, b.y)
        oz = min(a.z + a.height, b.z + b.height) - max(a.z, b.z)
        if ox > 1e-6 and oy > 1e-6 and oz > 1e-6:
            print(f"OVERLAP: {a.box.identifier} <-> {b.box.identifier} on {pallet.pallet_id}"); errors += 1
    # support: every box rests on floor or on at least one box top exactly at its z
    for p in ps:
        if p.z < 1e-6:
            continue
        supported = any(
            abs(q.z + q.height - p.z) < 1e-6
            and min(p.x + p.length, q.x + q.length) - max(p.x, q.x) > 1e-6
            and min(p.y + p.width, q.y + q.width) - max(p.y, q.y) > 1e-6
            for q in ps if q is not p
        )
        if not supported:
            print(f"FLOATING: {p.box.identifier} at z={p.z} on {pallet.pallet_id}"); errors += 1

print(f"\nCheck finished, {errors} problems found.")
# SKU distribution per pallet, to understand the layout
for pallet in pallets:
    skus = {}
    for p in pallet.placements:
        skus[p.box.sku] = skus.get(p.box.sku, 0) + 1
    print(pallet.pallet_id, dict(sorted(skus.items())))
