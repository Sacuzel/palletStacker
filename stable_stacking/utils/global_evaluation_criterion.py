import numpy as np

def calculate_global_evaluation_score(state):
    """
    Calculates a global evaluation score for the current state of the pallet.
    
    Criteria:
        • Stack density:        (Volume of stack) / (Volume of bounding box of stack)
        
        • Average box support:  Average number of packages that each package is placed upon.

    """
    stack_density = calculate_stack_density(state)
    average_box_support = calculate_average_box_support(state)
    # The division helps to normalize the average box support,
    # as high values lie around 2.5 and both factors are intended to be weighted equally.
    score = stack_density + (average_box_support/2.5)
    return score

def calculate_stack_density(state):
    """
    Calculates the stack density of the current state of the pallet.
    
    Stack density is defined as the ratio of the volume of the stack to the volume of the bounding box of the stack.

    
    Volume of stack                 =   total volume of all boxes already placed.
    Volume of bounding box of stack =   volume of the rectangular 3D space occupied by the stack as a whole,
                                        including empty gaps inside that space
    
    Args:
        state (State): The current state of the pallet.
    """
    bounding_volume = get_bounding_box_volume(state)

    if bounding_volume == 0:
        return 0.0

    return get_stack_volume(state) / bounding_volume

def get_bounding_box_volume(state):
    """
    Calculates the volume of the smallest axis-aligned bounding box
    around the current stack.
    """
    cell_size = state.get_cell_size()
    height_map = state.get_height_map()
    occupied = height_map > 0

    if not np.any(occupied):
        return 0.0

    x_indices, y_indices = np.where(occupied)

    occupied_length_cells = x_indices.max() - x_indices.min() + 1
    occupied_width_cells = y_indices.max() - y_indices.min() + 1

    occupied_length = occupied_length_cells * cell_size
    occupied_width = occupied_width_cells * cell_size

    max_height = np.max(height_map)

    return occupied_length * occupied_width * max_height

def get_stack_volume(state):
    """
    Calculates the total volume of all boxes currently in the state.
    Assumes _all_boxes_in_state contains one entry per placed box.
    """
    total_volume = 0.0
    cell_size = state.get_cell_size()  # Get the cell size from the state
    all_boxes_in_state_data = state.get_all_boxes_in_state()
    all_boxes = [box for box in all_boxes_in_state_data]

    for box in all_boxes:
        total_volume += (
            box.length * cell_size *
            box.width * cell_size *
            box.height
        )

    return total_volume
  
def calculate_average_box_support(state):
    """
    Calculates the average number of boxes that each box is placed upon in the current state of the pallet.
    
    Args:
        state (State): The current state of the pallet.
    """
    all_boxes_in_state_data = state.get_all_boxes_in_state()
    if not all_boxes_in_state_data:
        return 0.0

    total_supports = sum(
        placed_box.support_count for placed_box in all_boxes_in_state_data
    )

    return total_supports / len(all_boxes_in_state_data)