import numpy as np

class State:
    """
    Represents state of a pallet during the packing process. 
    Contains three feature maps: height map, weight map, and drop index map.
    """
    def __init__(self, pallette_discretization=10):

        # 2D matrix where each cell represents the height of the stack
        self._height_map = np.zeros((pallette_discretization, pallette_discretization))

        # 2D matrix where each cell represents the maximum weight that can be added
        self._weight_map = np.zeros((pallette_discretization, pallette_discretization))

        # 2D matrix where each cell represents the index of the topmost box
        # the index / ID of the topmost box occupying that cell
        self._drop_index_map = np.zeros((pallette_discretization, pallette_discretization))

        self._local_evaluation_score = 0

    # getters
    def get_height_map(self):
        return self._height_map

    def get_weight_map(self):
        return self._weight_map

    def get_drop_index_map(self):
        return self._drop_index_map

    def get_local_evaluation_score(self):
        return self._local_evaluation_score

    def initialize_maps(self, pallette_size):
        self._height_map = np.zeros((pallette_size, pallette_size))
        self._weight_map = np.ones((pallette_size, pallette_size)) * 1000  # Assuming a very high initial weight capacity
        self._drop_index_map = np.zeros((pallette_size, pallette_size))

    def get_number_of_boxes_in_area(self, x_start, y_start, length, width):
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