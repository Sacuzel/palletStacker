import logging
import yaml
import json

from stable_stacking.algorithm import Algorithm
from stable_stacking.box import Box
from stable_stacking.save_result import save_results_to_json
from stable_stacking.visualization_state_plotly import write_state_html

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)

DEFAULT_BOX_DATA_PATH = "data_converter/converted_datasets/cut_2/cut_2_sequence_1967.json"
#DEFAULT_BOX_DATA_PATH = "boxJsons/groceryBoxes.json"
#DEFAULT_BOX_DATA_PATH = "boxJsons/sorted_and_discretized/groceryBoxes_discretized_100mm_compat.json"
DEFAULT_PARAMETERS_PATH = "stable_stacking/utils/parameters.yaml"

def load_parameters():
    """
    Loads parameters from the YAML configuration file.
    Returns:
        dict: A dictionary containing the parameters.
    """
    try:
        with open(DEFAULT_PARAMETERS_PATH, 'r') as f:
            parameters = yaml.safe_load(f)
            logger.info("Parameters loaded successfully.")
            return parameters['parameters']
    except Exception as e:
        logger.error(f"Error loading parameters: {e}")
        raise e

def load_box_data() -> list[Box]:
    """
    Loads box data from a JSON file.
    Returns:
        list[Box]: A list of Box objects.
    """
    try:
        with open(DEFAULT_BOX_DATA_PATH, 'r') as f:
            box_data_as_dict = json.load(f)
            logger.info("Box data loaded successfully.")

            boxes = [
            Box(
                id=item["identifier"],
                sku=item["sku"],
                length=int(item["dimensions_mm"][0]/100),
                width=int(item["dimensions_mm"][1]/100),
                height=int(item["dimensions_mm"][2]/100),
                weight=item["weight_kg"],
            )
            for item in box_data_as_dict['boxes']]
            return boxes
                
    except Exception as e:
        logger.error(f"Error loading box data: {e}")
        raise e

def main():
    logger.info("Starting Stable Stacking MVP...")
    # load parameters from YAML
    parameters = load_parameters()
    logger.info(f"Loaded parameters: {parameters}")
    # load box data from JSON
    box_data = load_box_data()
    logger.info(f"Loaded box data: {len(box_data)} boxes")
    logger.info(f"First entry: {box_data[0]}")
    # run the packing algorithm
    algorithm = Algorithm(parameters, box_data)
    best_state = algorithm.run()
    logger.info(f"Best state found!")
    logger.info(f"Saving best state to JSON...")
    save_results_to_json(
        best_state,
        parameters,
        algorithm_name="Stable Stacking MVP",
        dataset_name="Grocery Boxes Mixed",
        runtime=0,  # Placeholder for runtime
        output_file_path="best_state_results.json"
    )
    # generate the 3D visualization
    write_state_html(
        best_state,
        output_path="state_layout.html",
        show_box_labels=True,
        open_in_browser=False,
    )

if __name__ == "__main__":
    main()