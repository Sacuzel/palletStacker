from stable_stacking.utils.state import State
from palletStacker_V2_Claude.models import Box

class Algorithm:
    def __init__(self, parameters, box_data: list ):
        self._parameters = parameters
        self._box_data = box_data
        self._global_states = []  # List of Global state objects
        self._remaining_box_types = list(set(box.sku for box in box_data))  # get all unique sku values from box_data

    def run(self):
        """
        current_states = [empty pallet]
        while packages remain and placements are still possible:

            child_states = []

            for each state in current_states:

                candidate_drops = local_search(state)

                for each drop in candidate_drops:
                    child = state + that box placement
                    update child's height map, weight map, drop index map
                    child_states.append(child)

            remove duplicate child states

            score each child state globally

            current_states = best k_global child states

        return best completed or best remaining state
        """
        self.initialize()

    def initialize(self):
        # Start with empty pallet state
        initial_state = State(pallette_discretization=self._parameters['pallette_size'])
        initial_state.initialize_maps(pallette_size=self._parameters['pallette_size'])
        self._global_states.append(initial_state)