"""Interactive 3D Plotly visualization for pallet-loading results.

This module converts the final pallet layout into a browser-viewable HTML file.
It does not affect the loading algorithm. It only reads Pallet and Placement
objects after the algorithm has finished.
"""

from __future__ import annotations

import hashlib
import webbrowser
from pathlib import Path
from typing import List, Sequence, Tuple

import plotly.graph_objects as go

from pallet import Pallet
from models import Placement

Color = str

def _stable_color_from_text(text: str) -> Color:
    digest = hashlib.md5(text.encode("utf-8")).digest()
    r = 80 + digest[0] % 140
    g = 80 + digest[1] % 140
    b = 80 + digest[2] % 140
    return f"rgb({r},{g},{b})"

def _cuboid_vertices(x0: float, y0: float, z0: float, length: float, width: float, height: float) -> Tuple[List[float], List[float], List[float]]:
    x1 = x0 + length
    y1 = y0 + width
    z1 = z0 + height
    xs = [x0, x1, x1, x0, x0, x1, x1, x0]
    ys = [y0, y0, y1, y1, y0, y0, y1, y1]
    zs = [z0, z0, z0, z0, z1, z1, z1, z1]
    return xs, ys, zs

def _cuboid_triangle_indices() -> Tuple[List[int], List[int], List[int]]:
    i = [0, 0, 4, 4, 0, 0, 1, 1, 2, 2, 3, 3]
    j = [1, 2, 6, 7, 5, 4, 6, 5, 7, 6, 4, 7]
    k = [2, 3, 5, 6, 1, 5, 2, 6, 3, 7, 0, 4]
    return i, j, k

def _placement_hover_text(placement: Placement) -> str:
    sku = placement.box.sku or "N/A"
    return (
        f"<b>{placement.box.identifier}</b><br>"
        f"SKU: {sku}<br>"
        f"Pallet: {placement.pallet_id}<br>"
        f"Position: x={placement.x:.0f}, y={placement.y:.0f}, z={placement.z:.0f} mm<br>"
        f"Size: {placement.length:.0f} × {placement.width:.0f} × {placement.height:.0f} mm<br>"
        f"Weight: {placement.box.weight:.1f} kg"
    )

def _make_cuboid_mesh(*, x: float, y: float, z: float, length: float, width: float, height: float, color: Color, name: str, hover_text: str, opacity: float, legend_group: str, show_legend: bool) -> go.Mesh3d:
    xs, ys, zs = _cuboid_vertices(x, y, z, length, width, height)
    i, j, k = _cuboid_triangle_indices()
    return go.Mesh3d(x=xs, y=ys, z=zs, i=i, j=j, k=k, color=color, opacity=opacity, flatshading=True, name=name, legendgroup=legend_group, showlegend=show_legend, hovertext=[hover_text] * 8, hoverinfo="text")

def _make_wireframe(*, x: float, y: float, z: float, length: float, width: float, height: float, color: str, name: str, show_legend: bool, expand_mm: float = 0.0) -> go.Scatter3d:
    e = expand_mm
    xs, ys, zs = _cuboid_vertices(x - e, y - e, z - e, length + 2.0 * e, width + 2.0 * e, height + 2.0 * e)
    edges = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)]
    line_x, line_y, line_z = [], [], []
    for a, b in edges:
        line_x.extend([xs[a], xs[b], None])
        line_y.extend([ys[a], ys[b], None])
        line_z.extend([zs[a], zs[b], None])
    return go.Scatter3d(x=line_x, y=line_y, z=line_z, mode="lines", line=dict(color=color, width=4), name=name, showlegend=show_legend, hoverinfo="skip")

def _make_text_label(placement: Placement, x_offset: float) -> go.Scatter3d:
    cx, cy, cz = placement.center
    return go.Scatter3d(x=[cx + x_offset], y=[cy], z=[cz], mode="text", text=[placement.box.identifier], textposition="middle center", showlegend=False, hoverinfo="skip")

def create_pallet_figure(pallets: Sequence[Pallet], *, title: str = "Pallet loading result", show_box_labels: bool = False, pallet_gap_mm: float = 500.0) -> go.Figure:
    fig = go.Figure()
    seen_legend_groups = set()
    max_x = max_y = max_z = 0.0

    for pallet_index, pallet in enumerate(pallets):
        x_offset = pallet_index * (pallet.length + pallet_gap_mm)
        max_x = max(max_x, x_offset + pallet.length)
        max_y = max(max_y, pallet.width)
        max_z = max(max_z, pallet.max_height)

        pallet_hover = (f"<b>Pallet {pallet.pallet_id}</b><br>Size: {pallet.length:.0f} × {pallet.width:.0f} mm<br>"
                        f"Boxes: {len(pallet.placements)}<br>Stack height: {pallet.max_stack_height:.0f} mm")

        fig.add_trace(_make_cuboid_mesh(x=x_offset, y=0.0, z=-80.0, length=pallet.length, width=pallet.width, height=80.0, color="rgb(120,120,120)", name=f"Pallet {pallet.pallet_id}", hover_text=pallet_hover, opacity=0.12, legend_group=f"PALLET-{pallet.pallet_id}", show_legend=True))
        fig.add_trace(_make_wireframe(x=x_offset, y=0.0, z=0.0, length=pallet.length, width=pallet.width, height=pallet.max_height, color="rgba(80,80,80,0.35)", name=f"Pallet {pallet.pallet_id} height limit", show_legend=False))

        for placement in pallet.placements:
            sku = placement.box.sku or "UNKNOWN"
            legend_group = f"SKU-{sku}"
            color = _stable_color_from_text(sku)
            show_legend = legend_group not in seen_legend_groups
            seen_legend_groups.add(legend_group)

            fig.add_trace(_make_cuboid_mesh(x=x_offset + placement.x, y=placement.y, z=placement.z, length=placement.length, width=placement.width, height=placement.height, color=color, name=sku, hover_text=_placement_hover_text(placement), opacity=1.0, legend_group=legend_group, show_legend=show_legend))
            fig.add_trace(_make_wireframe(x=x_offset + placement.x, y=placement.y, z=placement.z, length=placement.length, width=placement.width, height=placement.height, color="rgba(15,15,15,1.0)", name="Box edges", show_legend=False, expand_mm=1.5))
            if show_box_labels:
                fig.add_trace(_make_text_label(placement, x_offset))

    max_x, max_y, max_z = max(max_x, 1200.0), max(max_y, 800.0), max(max_z, 1800.0)
    fig.update_layout(title=title, scene=dict(xaxis=dict(title="X / pallet length (mm)", range=[-100, max_x + 100]), yaxis=dict(title="Y / pallet width (mm)", range=[-100, max_y + 100]), zaxis=dict(title="Z / height (mm)", range=[-100, max_z + 100]), aspectmode="data", camera=dict(eye=dict(x=1.7, y=-1.9, z=1.25), up=dict(x=0.0, y=0.0, z=1.0))), margin=dict(l=0, r=0, t=45, b=0), legend=dict(title="SKU / object", itemsizing="constant"))
    return fig

def write_layout_html(pallets: Sequence[Pallet], output_path: str | Path = "pallet_layout.html", *, title: str = "Pallet loading result", show_box_labels: bool = False, open_in_browser: bool = False, include_plotlyjs: bool | str = "directory") -> Path:
    output = Path(output_path).resolve()
    fig = create_pallet_figure(pallets, title=title, show_box_labels=show_box_labels)
    config = {"displaylogo": False, "scrollZoom": True, "responsive": True}
    fig.write_html(str(output), include_plotlyjs=include_plotlyjs, full_html=True, config=config)
    if open_in_browser:
        webbrowser.open(output.as_uri())
    return output