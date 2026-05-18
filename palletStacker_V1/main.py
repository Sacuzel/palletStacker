import tkinter as tk
from tkinter import filedialog
import json

from models import Box
from algorithm import pack_boxes
from visualization_plotly import write_layout_html

def load_json():
    # Setup invisible Tkinter root 
    root = tk.Tk()
    root.withdraw()
    file_path = filedialog.askopenfilename(
        title="Select Grocery Boxes JSON",
        filetypes=[("JSON files", "*.json")]
    )
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
    pallets = pack_boxes(boxes)
    
    print(f"Algorithm finished. Packed {len(boxes)} boxes into {len(pallets)} pallet(s).")
    
    out_file = "pallet_layout.html"
    print(f"Generating 3D interactive layout -> {out_file}")
    write_layout_html(pallets, output_path=out_file, show_box_labels=True, open_in_browser=True)

if __name__ == "__main__":
    main()