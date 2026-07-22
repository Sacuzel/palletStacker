from stable_stacking.utils.state import State
from stable_stacking.box import Box
from stable_stacking.local_search import create_local_k_states
from stable_stacking.utils.global_evaluation_criterion import calculate_global_evaluation_score
import logging

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)


class Algorithm:
    def __init__(self, parameters, box_data: list[Box] ):
        self._parameters = parameters
        self._box_data = box_data
        self._global_states = []  # List of Global state objects
        self._current_states = []  # List of current state objects
        self._remaining_boxes = box_data.copy()  # List of remaining boxes to be placed
        self._k_global = parameters.get('k_global', 5)  # Default to 5 if not provided
        self._k_local = parameters.get('k_local', 5)  # Default to 5 if not provided
        self._local_criterion_weights = parameters.get('local_criterion_weights')

    def run(self) -> State:
        """
        Main algorithm loop for the Stable Stacking problem.
        Returns:
            State: The best state found after processing all boxes.
        """
        logger.info("Starting the Stable Stacking Algorithm...")
        self.initialize()
        logger.info(f"Initial global states: {len(self._global_states)}")

        self._current_states = self._global_states.copy()
        best_state_so_far = self._current_states[0]

        while self._remaining_boxes and self._current_states:
            if len(self._remaining_boxes) % 10 == 0 or len(self._remaining_boxes) < 5:
                logger.info(f"Remaining boxes: {len(self._remaining_boxes)}")

            current_box = self._remaining_boxes[0]
            child_states = []

            for state in self._current_states:
                k_local_states = create_local_k_states(
                    current_box,
                    state,
                    self._k_local,
                    self._local_criterion_weights,
                )
                child_states.extend(k_local_states)

            if not child_states:
                logger.info(
                    f"No valid drops available for box {current_box.sku}. "
                    "Returning best state so far."
                )
                return best_state_so_far

            states_with_scores = [
                (state, calculate_global_evaluation_score(state))
                for state in child_states
            ]

            states_with_scores.sort(key=lambda x: x[1], reverse=True)

            self._current_states = [
                state for state, score in states_with_scores[:self._k_global]
            ]

            best_state_so_far = self._current_states[0]

            self._remaining_boxes.pop(0)

        return best_state_so_far

    def initialize(self):
        pallet_size = self._parameters["pallet_size"]

        max_stack_height = self._parameters.get(
            "max_stack_height",
            pallet_size,
        )

        initial_state = State(
            pallet_discretization=pallet_size,
            max_stack_height=max_stack_height,
        )

        initial_state.initialize_maps(pallet_size=pallet_size)
        self._global_states.append(initial_state)