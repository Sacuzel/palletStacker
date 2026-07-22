"""
Main Python module to validate and evaluate the Stable Stacking algorithm.

Run this module with the path to the results JSON file as an argument.
The module will validate the results against the contract defined in the `evaluate` module.
Example usage:
    python main.py path/to/results.json
"""

import argparse
import json
import logging

from evaluate.evaluate import Evaluator
from evaluate.validate import Validator

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

FAIL_ON_ERROR = False  # Set to False to log warnings instead of raising exceptions

def parse_arguments():
    """
    Parses command-line arguments.
    Returns:
        argparse.Namespace: Parsed command-line arguments.
    """
    parser = argparse.ArgumentParser(description="Validate and evaluate the Stable Stacking algorithm.")
    parser.add_argument('results_file', type=str, help="Path to the results JSON file.")
    return parser.parse_args()

def load_json(file_path: str) -> dict:
    """
    Loads a JSON file and returns its content.
    Args:
        file_path (str): Path to the JSON file.
    Returns:
        dict: Content of the JSON file.
    """

    with open(file_path, 'r') as f:
        return json.load(f)

def main():
    args = parse_arguments()
    results = load_json(args.results_file)
    algorithm_results = results.get('algorithm_results', {})
    pallet_parameters = results.get('pallet_parameters', {})

    # 1. Validate the results.
    validator = Validator(algorithm_results, pallet_parameters, fail_on_error=FAIL_ON_ERROR)
    is_valid = validator.validate_results()
    if not is_valid:
        logger.error("Validation failed. Exiting.")
        return
    # 2. Evaluate the results.
    evaluator = Evaluator(algorithm_results, pallet_parameters)
    evaluator.evaluate(print_to_stdout=True)


if __name__ == "__main__":
    main()