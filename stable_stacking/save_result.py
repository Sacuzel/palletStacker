"""
This module contains functions to save the results of the stacking model
so that they can be evaluated later. The results are saved in a JSON format following
the contract defined in the `evaluate` module:

{
  "pallet_parameters": {
    "pallet_base_width": 800,
    "pallet_length": 1200,
    "max_stack_height": 1500
  },
  "algorithm_results": {
    "algorithm_name": "example_greedy_lowest_z",
    "dataset_name": "contract_example_dataset",
    "total_boxes": 5,
    "runtime": 0.012,
    "drops": [
      {
        "box": {
          "id": "box_001",
          "sku": "SKU-A",
          "length": 400,
          "width": 300,
          "height": 200,
          "weight": 8.5
        },
        "position": {
          "x": 0,
          "y": 0,
          "z": 0
        }
      },
      {
        "box": {
          "id": "box_002",
          "sku": "SKU-A",
          "length": 400,
          "width": 300,
          "height": 200,
          "weight": 8.5
        },
        "position": {
          "x": 400,
          "y": 0,
          "z": 0
        }
      }
    ]
  }}

The input of the module is a State object and a dictionary of pallet parameters.
class State:
    def __init__(self, pallet_discretization, max_stack_height=None):
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

"""

import json
import logging

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)

def save_results_to_json(state, pallet_parameters, algorithm_name, dataset_name, runtime, output_file_path):
    """
    Saves the results of the stacking model to a JSON file.

    Args:
        state (State): The final state of the stacking model.
        pallet_parameters (dict): Dictionary containing pallet parameters.
        algorithm_name (str): Name of the algorithm used.
        dataset_name (str): Name of the dataset used.
        runtime (float): Runtime of the algorithm in seconds.
        output_file_path (str): Path to save the output JSON file.
    """
    try:
        drops = []
        for box in state.get_all_boxes_in_state():
            drops.append({
                "box": {
                    "id": box.id,
                    "sku": box.sku,
                    "length": box.length,
                    "width": box.width,
                    "height": box.height,
                    "weight": box.weight
                },
                "position": {
                    "x": box.position[0],
                    "y": box.position[1],
                    "z": box.position[2]
                }
            })

        results = {
            "pallet_parameters": pallet_parameters,
            "algorithm_results": {
                "algorithm_name": algorithm_name,
                "dataset_name": dataset_name,
                "total_boxes": len(drops),
                "runtime": runtime,
                "drops": drops
            }
        }

        with open(output_file_path, 'w') as f:
            json.dump(results, f, indent=4)

        logger.info(f"Results saved successfully to {output_file_path}")

    except Exception as e:
        logger.error(f"Error saving results to JSON: {e}")
        raise e