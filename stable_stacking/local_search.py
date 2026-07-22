from stable_stacking.utils.local_evaluation_crietrion import calculate_local_evaluation_score
from stable_stacking.utils.state import State
from stable_stacking.box import Box
import logging
import numpy as np

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

def create_local_k_states(box: Box, current_state, k, weights):
    """
    Create k number of candidate local drops based on the remaining box types.
    Iterates through all possible drops for each box type and calculates the local evaluation score for each drop. 
    Returns the top k drops based on the local evaluation score.

    Args:
        box (Box): The box to consider for placement.
        current_state (State): The current state of the pallet.
        k (int): The number of top candidate drops to return.
        weights (dict): Weights for the local evaluation criterion.
    """
    candidate_drops = []

    # for each box type, calculate all possible drops
    # Get all possible drop position for each box
    # Evaluate each drop and calculate local evaluation score
    # Store k best drops based on local evaluation score
    all_possible_drops = calculate_possible_drops(box, current_state)

    for oriented_box, position in all_possible_drops:
        local_evaluation_score = calculate_local_evaluation_score(
            current_state,
            oriented_box,
            position,
            weights,
        )

        candidate_drops.append(
            (oriented_box, position, local_evaluation_score)
        )

    # Remove duplicates based on box type, position, and orientation
    unique_candidate_drops = {}

    for oriented_box, position, score in candidate_drops:
        key = (
            oriented_box.sku,
            position,
            oriented_box.length,
            oriented_box.width,
        )

        if key not in unique_candidate_drops or unique_candidate_drops[key][2] < score:
            unique_candidate_drops[key] = (oriented_box, position, score)

    unique_candidate_drops = list(unique_candidate_drops.values())
    unique_candidate_drops.sort(key=lambda x: x[2], reverse=True)

    top_k_drops = unique_candidate_drops[:k]

    new_k_states = []

    for oriented_box, position, score in top_k_drops:
        new_state = current_state.clone()
        new_state.add_box(oriented_box, position)
        new_k_states.append(new_state)

    return new_k_states

def has_valid_support(
    state,
    x,
    y,
    length,
    width,
    base_z,
    tol=1e-6,
    min_edge_support_ratio=0.75,
    min_base_support_ratio=0.50,
):
    footprint = state._height_map[x:x+length, y:y+width]

    # Ground placement is valid.
    if base_z == 0:
        return True

    contact = np.isclose(footprint, base_z, atol=tol)

    base_support_ratio = contact.sum() / (length * width)

    front_ratio = contact[0, :].sum() / width
    back_ratio = contact[-1, :].sum() / width
    left_ratio = contact[:, 0].sum() / length
    right_ratio = contact[:, -1].sum() / length

    opposite_edges_supported = (
        (front_ratio >= min_edge_support_ratio and back_ratio >= min_edge_support_ratio)
        or
        (left_ratio >= min_edge_support_ratio and right_ratio >= min_edge_support_ratio)
    )

    enough_base_area_supported = base_support_ratio >= min_base_support_ratio

    return opposite_edges_supported and enough_base_area_supported

def can_place_box(state, x, y, length, width, box_height, max_height):
    if x + length > state._height_map.shape[0]:
        return False
    if y + width > state._height_map.shape[1]:
        return False

    footprint = state._height_map[x:x+length, y:y+width]
    base_z = footprint.max()
    top_z = base_z + box_height

    if top_z > max_height:
        return False

    return has_valid_support(state, x, y, length, width, base_z)

def calculate_possible_drops(box: Box, state: State):
    """
    Calculates all possible positions where the given box can be placed.

    Returns:
        list of tuples:
            (oriented_box, (x, y))

    The oriented_box has length/width swapped when rotated.
    """
    possible_drops = []

    orientations = [
        (box.length, box.width, False),
        (box.width, box.length, True),
    ]

    # Avoid duplicate orientation for square boxes
    seen_orientations = set()

    max_height = state.get_max_stack_height()

    for length, width, rotated in orientations:
        orientation_key = (length, width)

        if orientation_key in seen_orientations:
            continue

        seen_orientations.add(orientation_key)

        x_range = state._height_map.shape[0] - int(length) + 1
        y_range = state._height_map.shape[1] - int(width) + 1

        for x in range(x_range):
            for y in range(y_range):
                if can_place_box(
                    state,
                    x,
                    y,
                    length,
                    width,
                    box.height,
                    max_height,
                ):
                    oriented_box = box.copy()
                    oriented_box.length = length
                    oriented_box.width = width

                    # Optional but useful for debugging / visualization later
                    oriented_box.rotated = rotated

                    possible_drops.append((oriented_box, (x, y)))

    return possible_drops