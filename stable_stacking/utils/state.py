import numpy as np

from stable_stacking.box import Box

class State:
    def __init__(self, pallet_discretization=10, max_stack_height=None):
        self._height_map = np.zeros((pallet_discretization, pallet_discretization))
        self._weight_map = np.zeros((pallet_discretization, pallet_discretization))
        self._drop_index_map = np.zeros((pallet_discretization, pallet_discretization))

        self._local_evaluation_score = 0
        self._all_boxes_in_state = []
        self._cell_size = 1.0

        # Temporary default: same as pallet discretization.
        # Better: read this from parameters.yaml.
        self._max_stack_height = (
            max_stack_height
            if max_stack_height is not None
            else pallet_discretization
        )

    # getters
    def get_current_stack_height(self):
        return np.max(self._height_map)

    def get_max_stack_height(self):
        return self._max_stack_height

    def get_height_map(self):
        return self._height_map

    def get_weight_map(self):
        return self._weight_map

    def get_drop_index_map(self):
        return self._drop_index_map

    def get_local_evaluation_score(self):
        return self._local_evaluation_score

    def get_all_boxes_in_state(self):
        return self._all_boxes_in_state

    def get_cell_size(self):
        return self._cell_size

    def get_max_height(self):
        return np.max(self._height_map)

    def initialize_maps(self, pallet_size):
        self._height_map = np.zeros((pallet_size, pallet_size))
        self._weight_map = np.ones((pallet_size, pallet_size)) * 1000  # Assuming a very high initial weight capacity
        self._drop_index_map = np.zeros((pallet_size, pallet_size))

    def get_number_of_boxes_in_area(self, x_start: int, y_start: int, length: int, width: int):
        """
        Counts the number of boxes in a specified area of the drop index map.
        The area is defined by the top-left corner (x_start, y_start) and the dimensions (length, width).
        """
        sub_area = self._drop_index_map[x_start:x_start + length, y_start:y_start + width]
        unique_boxes = np.unique(sub_area[sub_area > 0])  # Exclude zeros which represent empty spaces
        return len(unique_boxes)

    def get_number_of_sides_supported(self, box, drop_position):
        """
        Determines the number of sides of a box that are supported by other boxes at a given drop position.
        A side is considered supported if any box touches any part of that side.
        """
        x_start, y_start = drop_position
        length, width = box.length, box.width

        # Check left side
        left_supported = np.any(self._drop_index_map[x_start:x_start + length, y_start - 1] > 0) if y_start > 0 else False

        # Check right side
        right_supported = np.any(self._drop_index_map[x_start:x_start + length, y_start + width] > 0) if y_start + width < self._drop_index_map.shape[1] else False

        # Check front side
        front_supported = np.any(self._drop_index_map[x_start - 1, y_start:y_start + width] > 0) if x_start > 0 else False

        # Check back side
        back_supported = np.any(self._drop_index_map[x_start + length, y_start:y_start + width] > 0) if x_start + length < self._drop_index_map.shape[0] else False

        return sum([left_supported, right_supported, front_supported, back_supported])

    def calculate_surface_support_area(self, box, drop_position):
        """
        Calculates the percentage of the base and side surfaces of a box that are supported by other boxes or the pallet.
        A surface is considered supported if it touches any part of another box or the pallet.
        """
        x_start, y_start = drop_position
        length, width = box.length, box.width

        # Calculate base support area
        base_area = length * width
        base_supported_area = np.sum(self._drop_index_map[x_start:x_start + length, y_start:y_start + width] > 0)
        base_support_percentage = base_supported_area / base_area if base_area > 0 else 0

        # Calculate side support area (assuming sides are vertical and have the same height as the box)
        side_area = 2 * (length + width) * box.height  # Perimeter * height
        side_supported_area = 0

        # Check left and right sides
        if y_start > 0:
            side_supported_area += np.sum(self._drop_index_map[x_start:x_start + length, y_start - 1] > 0)
        if y_start + width < self._drop_index_map.shape[1]:
            side_supported_area += np.sum(self._drop_index_map[x_start:x_start + length, y_start + width] > 0)

        # Check front and back sides
        if x_start > 0:
            side_supported_area += np.sum(self._drop_index_map[x_start - 1, y_start:y_start + width] > 0)
        if x_start + length < self._drop_index_map.shape[0]:
            side_supported_area += np.sum(self._drop_index_map[x_start + length, y_start:y_start + width] > 0)

        side_support_percentage = side_supported_area / side_area if side_area > 0 else 0

        return (base_support_percentage + side_support_percentage) / 2  # Average support percentage

    def calculate_support_ids_for_drop(self, box, drop_position, support_threshold=0.25, height_tolerance=1e-6):
        """
        Returns the IDs of boxes that support this new box.

        A supporting box is counted only if:
        1. it touches the base of the new box, and
        2. its contact area is at least support_threshold of the new box base area.
        """
        x_start, y_start = drop_position
        length, width = box.length, box.width

        height_area = self._height_map[
            x_start:x_start + length,
            y_start:y_start + width
        ]

        index_area = self._drop_index_map[
            x_start:x_start + length,
            y_start:y_start + width
        ]

        # The box is dropped from above and rests on the highest surface
        # inside its footprint.
        base_z = np.max(height_area)

        # Only cells exactly at the contact height are real supports.
        # Lower cells are gaps underneath the box and should not count.
        contact_mask = np.isclose(height_area, base_z, atol=height_tolerance)

        # Ignore pallet / empty cells with ID 0.
        contact_box_ids = index_area[contact_mask]
        contact_box_ids = contact_box_ids[contact_box_ids > 0]

        if contact_box_ids.size == 0:
            return []

        unique_ids, counts = np.unique(contact_box_ids, return_counts=True)

        base_area = length * width

        support_ids = [
            box_id
            for box_id, count in zip(unique_ids, counts)
            if count / base_area >= support_threshold
        ]

        return support_ids

    def add_box(self, box, drop_position):
        x_start, y_start = drop_position
        length, width = box.length, box.width

        support_ids = self.calculate_support_ids_for_drop(box, drop_position)

        height_area = self._height_map[
            x_start:x_start + length,
            y_start:y_start + width
        ]

        base_z = np.max(height_area)
        top_z = base_z + box.height

        new_box_id = len(self._all_boxes_in_state) + 1

        self._height_map[
            x_start:x_start + length,
            y_start:y_start + width
        ] = top_z

        self._drop_index_map[
            x_start:x_start + length,
            y_start:y_start + width
        ] = new_box_id

        new_box = Box(
            id=new_box_id,
            sku=box.sku,
            length=box.length,
            width=box.width,
            height=box.height,
            weight=box.weight,
            support_ids=support_ids,
            support_count=len(support_ids),
            position=(x_start, y_start, base_z)  # Store the (x, y, z) position of the box
        )

        self._all_boxes_in_state.append(new_box)

    def clone(self):
        new_state = State(
            pallet_discretization=self._height_map.shape[0],
            max_stack_height=self._max_stack_height,
        )
        new_state._height_map = np.copy(self._height_map)
        new_state._weight_map = np.copy(self._weight_map)
        new_state._drop_index_map = np.copy(self._drop_index_map)
        new_state._local_evaluation_score = self._local_evaluation_score
        new_state._all_boxes_in_state = [box.copy() for box in self._all_boxes_in_state]
        return new_state