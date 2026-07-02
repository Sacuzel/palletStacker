import logging
import yaml
import json

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)

DEFAULT_BOX_DATA_PATH = "boxJsons/groceryBoxes.json"
DEFAULT_PARAMETERS_PATH = "stable_stacking/utils/parameters.yaml"

def load_parameters():
    """Loads parameters from the YAML configuration file."""
    try:
        with open(DEFAULT_PARAMETERS_PATH, 'r') as f:
            parameters = yaml.safe_load(f)
            logger.info("Parameters loaded successfully.")
            return parameters
    except Exception as e:
        logger.error(f"Error loading parameters: {e}")
        raise e

def load_box_data():
    """Loads box data from a JSON file."""
    try:
        with open(DEFAULT_BOX_DATA_PATH, 'r') as f:
            box_data = json.load(f)
            logger.info("Box data loaded successfully.")
            return box_data
    except Exception as e:
        logger.error(f"Error loading box data: {e}")
        raise e

def main():
    logger.info("Starting Stable Stacking MVP...")
    # load parameters from YAML
    parameters = load_parameters()
    # load box data from JSON
    box_data = load_box_data()
    # run the packing algorithm
    # generate the 3D visualization
    pass

if __name__ == "__main__":
    main()