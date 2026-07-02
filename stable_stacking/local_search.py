from stable_stacking.state import State
from palletStacker_V2_Claude.models import Box

def create_local_k_states(remaining_box_types, current_state, k):
    """
    Create k number of candidate local drops based on the remaining box types.
    Iterates through all possible drops for each box type and calculates the local evaluation score for each drop. 
    Returns the top k drops based on the local evaluation score.

    Args:
        remaining_box_types (list): List of remaining box types (skus) to consider for placement.
        current_state (State): The current state of the pallet.
        k (int): The number of top candidate drops to return.    
    """
    candidate_drops = []

    for box_type in remaining_box_types:
        # for each box type, calculate all possible drops
        # Get all possible drop position for each box
        # Evaluate each drop and calculate local evaluation score
        # Store k best drops based on local evaluation score
        # Create a Box instance for the current box type
        box = Box(identifier=box_type, sku=box_type, length=1.0, width=1.0, height=1.0, weight=1.0)  # Placeholder dimensions and weight
        all_possible_drops = current_state.calculate_possible_drops(box)
        # Evaluate each drop and calculate local evaluation score
        for drop in all_possible_drops:
            x, y = drop
            local_evaluation_score = current_state.calculate_local_evaluation_score(box, (x, y))
            candidate_drops.append((box, (x, y), local_evaluation_score))

    # if any valid drops are possible:
    #   select k number of local drops
    #   create states of each k drop
    
    if not candidate_drops:
        return []  # No valid drops available
    # select k best drops based on local evaluation score
    candidate_drops.sort(key=lambda x: x[2], reverse=True)  # Sort by local evaluation score in descending order
    top_k_drops = candidate_drops[:k]
    return top_k_drops


def can_place_box(self, x, y, length, width):
    """
    Checks if a box of given length and width can be placed at the (x, y) position on the pallet.
    Ensures that the box does not exceed the pallet boundaries and does not overlap with existing boxes.
    """
    # Check if the box exceeds pallet boundaries
    if x + length > self._height_map.shape[0] or y + width > self._height_map.shape[1]:
        return False

    # Check for overlap with existing boxes
    for i in range(int(length)):
        for j in range(int(width)):
            if self._height_map[x + i, y + j] > 0:  # Assuming height > 0 means occupied
                return False

    # Check that the weight capacity is not exceeded
    for i in range(int(length)):
        for j in range(int(width)):
            if self._weight_map[x + i, y + j] <= 0:  # Assuming weight capacity <= 0 means cannot place
                return False

    return True

def calculate_possible_drops(self, box):
    """
    Calculates all possible positions where the given box can be placed on the pallet based on the current height and weight maps.
    A position is calculated from the top-left corner of the box.
    Box can be rotated by 0 degreees or 90 degrees (length and width can be swapped).
    Returns a list of tuples representing the (x, y) coordinates of the top-left corner of the box for each valid drop position.
    """
    possible_drops = []
    box_dimensions = [(box.length, box.width), (box.width, box.length)]  # Consider both orientations

    for length, width in box_dimensions:
        for x in range(self._height_map.shape[0] - int(length) + 1):
            for y in range(self._height_map.shape[1] - int(width) + 1):
                # Check if the box can be placed at (x, y)
                if self.can_place_box(x, y, length, width):
                    possible_drops.append((x, y))

    return possible_drops