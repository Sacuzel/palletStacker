"""Entry point for the Pallet Packing application."""

import tkinter as tk
from tkinter import filedialog
import json

from models import Box
from algorithm import pack_boxes
from visualization_plotly import write_layout_html
from diagnostics import print_packing_report
from gazebo_exporter import write_layout_sdf

def load_json():
    """Opens a graphical file dialog to select the grocery boxes JSON file."""
    # Setup invisible Tkinter root to host the file dialog
    root = tk.Tk()
    root.withdraw()
    
    file_path = filedialog.askopenfilename(
        title="Select Grocery Boxes JSON",
        filetypes=[("JSON files", "*.json")]
    )
    
    # Destroy the root to return execution to the console
    root.destroy()
    
    if not file_path:
        return None
        
    with open(file_path, 'r') as f:
        return json.load(f)

def main():
    print("Awaiting file selection...")
    data = load_json()
    if not data:
        print("No file selected. Exiting.")
        return
        
    # Parse JSON input into Box domain objects
    boxes = []
    for b in data.get("boxes", []):
        boxes.append(Box(
            identifier=b["identifier"],
            sku=b["sku"],
            length=b["dimensions_mm"][0],
            width=b["dimensions_mm"][1],
            height=b["dimensions_mm"][2],
            weight=b["weight_kg"]
        ))
        
    print(f"Loaded {len(boxes)} boxes. Initiating MVP algorithm...")
    
    # Run the core packing algorithm
    pallets = pack_boxes(boxes)
    
    print(f"Algorithm finished. Packed {len(boxes)} boxes into {len(pallets)} pallet(s).")

    # --- Print Terminal Diagnostics ---
    print_packing_report(pallets)
    
    # Render the interactive 3D HTML output
    out_file = "pallet_layout.html"
    print(f"Generating 3D interactive layout -> {out_file}")
    write_layout_html(pallets, output_path=out_file, show_box_labels=True, open_in_browser=True)

    # Render the Gazebo physics world
    sdf_path = write_layout_sdf(
        pallets,
        run_name="latest",
        overwrite=True,
    )

    print(f"Generating Gazebo physics world -> {sdf_path}")

if __name__ == "__main__":
    main()