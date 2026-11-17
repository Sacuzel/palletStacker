"""Loader optimizer for the pallet stacker.

Wraps the EMS loader in an iterative search (rule sweep + BRKGA) and exposes
the result through the normal loader interface:

    from pallet_stacker.optimizer import load_boxes
    pallets = load_boxes(boxes)

See loaderOptimizer.py for the program flow and optimizerSettings.py for all
tunable values.
"""

from .loaderOptimizer import LoaderOptimizerError, load_boxes

__all__ = ["LoaderOptimizerError", "load_boxes"]