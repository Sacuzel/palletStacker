"""=====================================================================
MAIN  (role: main program / "PLC_PRG" - ties the POUs together)
=====================================================================
Program flow (one scan, top to bottom):

  1. ask the user to choose the input JSON via a file dialog
  2. run the online packing algorithm  (algorithm.pack_boxes)
  3. print a utilisation / stability report to the console
  4. write the interactive Plotly HTML  (visualization_plotly)
  5. write the Gazebo SDF world + manifest (gazebo_exporter)

Usage:
    python main.py                   # a file dialog opens
    python main.py --labels         # show box ids in 3D
    python main.py --open           # open HTML in browser
====================================================================="""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import List, Optional

import config
from algorithm import pack_boxes
from models import Box
from pallet import Pallet


# ===============================================================
# INPUT: file dialog -> Path
# ===============================================================
def ask_json_path() -> Optional[Path]:
    """Open a graphical file dialog and return the chosen JSON path,
    or None if the user cancelled the dialog.

    An invisible Tkinter root window is created only to host the
    dialog and destroyed immediately afterwards, so execution
    returns to the console."""
    import tkinter as tk
    from tkinter import filedialog

    root = tk.Tk()
    root.withdraw()

    file_path = filedialog.askopenfilename(
        title="Select Grocery Boxes JSON",
        filetypes=[("JSON files", "*.json")],
    )

    root.destroy()

    return Path(file_path) if file_path else None


# ===============================================================
# INPUT: JSON -> List[Box]
# ===============================================================
def load_boxes_from_json(path: str | Path) -> List[Box]:
    """Parse the input file into Box objects, preserving order.

    The order of the "boxes" array IS the arrival order on the
    conveyor - the online algorithm sees them exactly in this
    sequence and never looks ahead."""
    with Path(path).open("r", encoding="utf-8") as handle:
        data = json.load(handle)

    boxes: List[Box] = []
    for item in data.get("boxes", []):
        dims = item["dimensions_mm"]
        boxes.append(
            Box(
                identifier=str(item["identifier"]),
                sku=str(item["sku"]),
                length=float(dims[0]),
                width=float(dims[1]),
                height=float(dims[2]),
                weight=float(item["weight_kg"]),
            )
        )
    return boxes


# ===============================================================
# REPORT: console summary of the packing result
# ===============================================================
def print_report(boxes: List[Box], pallets: List[Pallet]) -> None:
    """Human-readable summary, similar to an HMI status page."""
    packed = sum(len(p.placements) for p in pallets)
    print("=" * 64)
    print("PALLET PACKING REPORT")
    print("=" * 64)
    print(f"Boxes in input          : {len(boxes)}")
    print(f"Boxes packed            : {packed}")
    print(f"Boxes rejected          : {len(boxes) - packed}")
    print(f"Pallets used            : {len(pallets)}")
    print(
        f"Pallet envelope         : {config.PALLET_LENGTH:.0f} x "
        f"{config.PALLET_WIDTH:.0f} x {config.MAX_STACK_HEIGHT:.0f} mm"
    )
    print("-" * 64)
    for pallet in pallets:
        print(
            f"  {pallet.pallet_id}: "
            f"{len(pallet.placements):3d} boxes | "
            f"stack height {pallet.max_stack_height:7.0f} mm | "
            f"payload {pallet.total_weight_kg:7.1f} kg | "
            f"volume utilisation {pallet.utilization * 100.0:5.1f} %"
        )
    if pallets:
        total_used = sum(p.used_volume_mm3 for p in pallets)
        total_env = len(pallets) * (
            config.PALLET_LENGTH * config.PALLET_WIDTH * config.MAX_STACK_HEIGHT
        )
        print("-" * 64)
        print(f"Overall volume utilisation: {total_used / total_env * 100.0:5.1f} %")
    print("=" * 64)


# ===============================================================
# MAIN PROGRAM
# ===============================================================
def main() -> None:
    parser = argparse.ArgumentParser(
        description="Online SKU-group pallet packing with LBCP stability validation."
    )
    parser.add_argument(
        "--labels", action="store_true", help="show box identifiers in the 3D view"
    )
    parser.add_argument(
        "--open", action="store_true", help="open the Plotly HTML in a browser"
    )
    args = parser.parse_args()

    # ---- 1. INPUT (file dialog) ---------------------------------
    print("Awaiting file selection...")
    json_path = ask_json_path()
    if json_path is None:
        sys.exit("No file selected. Exiting.")
    if not json_path.exists():
        sys.exit(f"Input file not found: {json_path}")
    boxes = load_boxes_from_json(json_path)
    print(f"Loaded {len(boxes)} boxes from {json_path}")

    # ---- 2. PACKING (the actual algorithm) ----------------------
    pallets = pack_boxes(boxes)

    # ---- 3. REPORT ----------------------------------------------
    print_report(boxes, pallets)

    # ---- 4. PLOTLY VISUALISATION --------------------------------
    # Import here so the packing core stays usable on systems
    # without plotly installed.
    from visualization_plotly import write_layout_html

    html_path = write_layout_html(
        pallets,
        output_path=config.OUTPUT_HTML_PATH,
        title="Pallet loading result (LBCP-stable online packing)",
        show_box_labels=args.labels,
        open_in_browser=args.open,
        include_plotlyjs=config.PLOTLY_JS_MODE,
    )
    print(f"Wrote Plotly visualisation : {html_path}")

    # ---- 5. GAZEBO EXPORT ---------------------------------------
    from gazebo_exporter import write_layout_sdf

    sdf_path = write_layout_sdf(
        pallets,
        output_dir=config.GAZEBO_OUTPUT_DIR,
        run_name=config.GAZEBO_RUN_NAME,
        overwrite=True,
    )
    print(f"Wrote Gazebo world         : {sdf_path.resolve()}")
    print("Run the physics check with : gz sim " + str(sdf_path))


if __name__ == "__main__":
    main()