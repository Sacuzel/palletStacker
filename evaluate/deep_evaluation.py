"""
Criterion: 

1. Packing and Space Efficiency:

    Volume/Space Utilization:                       100 * (sum of volumes of all boxes in the stack) / (total volume of the pallet).

    Number of Packed Items:                         The total number of items successfully packed onto a single pallet.

    Stack Density:                                  The volume of the items in the stack divided
                                                    by the volume of the stack's bounding box.

    Number of Pallets:                              If multiple pallets are used, this metric counts the number of pallets
                                                    required to pack all items.

2. Physical and Structural Stability:

    Average Box Support:                            The average number of packages supporting each box from below.
                                                    Values greater than 1.5 indicate high stability via interlocks.

    Average Face Contacts:                          The average number of packages that each package touches on all of its faces,
                                                    used as a proxy for stack cohesiveness.

    Load Bearable Convex Polygon (LBCP):            A Boolean metric used to determine
                                                    if an item's center of gravity falls within its support polygon,
                                                    guaranteeing no bin collapse.

    Compression Constraints:                        Measuring whether a layer can sustain the weight of items above it based on the constituent boxes'
                                                    compression indices.


"""