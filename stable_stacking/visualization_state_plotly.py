"""Interactive 3D Plotly visualization for stable_stacking State objects.

This module is adapted from the original pallet/placement Plotly visualizer, but
it reads the final `State` object produced by the stable_stacking algorithm.

Expected State interface
------------------------
The visualizer expects a State-like object with:

    state.get_all_boxes_in_state() -> list[Box]
    state.get_height_map() -> numpy.ndarray      # optional but recommended
    state.get_cell_size() -> float               # optional; defaults to 1.0

Each placed Box is expected to have:

    box.id or box.identifier
    box.sku
    box.length
    box.width
    box.height
    box.weight
    box.position = (x, y, z)

where x, y, z, length, width, and height are in the same coordinate system as
used by the State maps. If your maps are discretized grid cells, the figure is
shown in grid units by default.
"""

from __future__ import annotations

import hashlib
import webbrowser
from pathlib import Path
from typing import Any, Iterable, List, Tuple

import plotly.graph_objects as go

Color = str


def _stable_color_from_text(text: str) -> Color:
    """Generate a deterministic RGB color based on the SKU string hash."""
    digest = hashlib.md5(text.encode("utf-8")).digest()
    r = 80 + digest[0] % 140
    g = 80 + digest[1] % 140
    b = 80 + digest[2] % 140
    return f"rgb({r},{g},{b})"


def _cuboid_vertices(
    x0: float,
    y0: float,
    z0: float,
    length: float,
    width: float,
    height: float,
) -> Tuple[List[float], List[float], List[float]]:
    """Calculate the 8 corner vertices for a 3D cuboid."""
    x1 = x0 + length
    y1 = y0 + width
    z1 = z0 + height

    xs = [x0, x1, x1, x0, x0, x1, x1, x0]
    ys = [y0, y0, y1, y1, y0, y0, y1, y1]
    zs = [z0, z0, z0, z0, z1, z1, z1, z1]
    return xs, ys, zs


def _cuboid_triangle_indices() -> Tuple[List[int], List[int], List[int]]:
    """Define 12 triangles, two for each cuboid face."""
    i = [0, 0, 4, 4, 0, 0, 1, 1, 2, 2, 3, 3]
    j = [1, 2, 6, 7, 5, 4, 6, 5, 7, 6, 4, 7]
    k = [2, 3, 5, 6, 1, 5, 2, 6, 3, 7, 0, 4]
    return i, j, k


def _make_cuboid_mesh(
    *,
    x: float,
    y: float,
    z: float,
    length: float,
    width: float,
    height: float,
    color: Color,
    name: str,
    hover_text: str,
    opacity: float,
    legend_group: str,
    show_legend: bool,
) -> go.Mesh3d:
    """Create the solid colored 3D faces for a cuboid."""
    xs, ys, zs = _cuboid_vertices(x, y, z, length, width, height)
    i, j, k = _cuboid_triangle_indices()

    return go.Mesh3d(
        x=xs,
        y=ys,
        z=zs,
        i=i,
        j=j,
        k=k,
        color=color,
        opacity=opacity,
        flatshading=True,
        name=name,
        legendgroup=legend_group,
        showlegend=show_legend,
        hovertext=[hover_text] * 8,
        hoverinfo="text",
    )


def _make_wireframe(
    *,
    x: float,
    y: float,
    z: float,
    length: float,
    width: float,
    height: float,
    color: str,
    name: str,
    show_legend: bool,
    expand: float = 0.0,
) -> go.Scatter3d:
    """Create dark cuboid outlines. expand helps avoid z-fighting."""
    xs, ys, zs = _cuboid_vertices(
        x - expand,
        y - expand,
        z - expand,
        length + 2.0 * expand,
        width + 2.0 * expand,
        height + 2.0 * expand,
    )

    edges = [
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 4),
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
    ]

    line_x: list[float | None] = []
    line_y: list[float | None] = []
    line_z: list[float | None] = []

    for a, b in edges:
        line_x.extend([xs[a], xs[b], None])
        line_y.extend([ys[a], ys[b], None])
        line_z.extend([zs[a], zs[b], None])

    return go.Scatter3d(
        x=line_x,
        y=line_y,
        z=line_z,
        mode="lines",
        line=dict(color=color, width=4),
        name=name,
        showlegend=show_legend,
        hoverinfo="skip",
    )


def _get_state_boxes(state: Any) -> list[Any]:
    """Return boxes from a State-like object."""
    if hasattr(state, "get_all_boxes_in_state"):
        return list(state.get_all_boxes_in_state())

    if hasattr(state, "_all_boxes_in_state"):
        return list(state._all_boxes_in_state)

    raise TypeError(
        "Expected a State-like object with get_all_boxes_in_state() "
        "or _all_boxes_in_state."
    )


def _get_state_shape(state: Any, boxes: Iterable[Any]) -> tuple[float, float]:
    """Infer pallet/map length and width."""
    if hasattr(state, "get_height_map"):
        height_map = state.get_height_map()
        if hasattr(height_map, "shape") and len(height_map.shape) >= 2:
            # State maps are indexed as [x, y], so shape[0] is length and shape[1] is width.
            return float(height_map.shape[0]), float(height_map.shape[1])

    max_x = max_y = 0.0
    for box in boxes:
        if getattr(box, "position", None) is None:
            continue
        x, y, _ = box.position
        max_x = max(max_x, float(x) + float(box.length))
        max_y = max(max_y, float(y) + float(box.width))

    return max(max_x, 1.0), max(max_y, 1.0)


def _get_cell_size(state: Any, override_cell_size: float | None) -> float:
    """Return coordinate scaling factor."""
    if override_cell_size is not None:
        return float(override_cell_size)

    if hasattr(state, "get_cell_size"):
        return float(state.get_cell_size())

    return 1.0


def _box_identifier(box: Any) -> str:
    """Return a stable user-facing box identifier."""
    for attr in ("identifier", "id", "box_id"):
        if hasattr(box, attr):
            value = getattr(box, attr)
            if value is not None:
                return str(value)
    return "UNKNOWN"


def _box_hover_text(box: Any, *, unit_label: str) -> str:
    """Create HTML tooltip text for a placed box."""
    identifier = _box_identifier(box)
    sku = getattr(box, "sku", "UNKNOWN") or "UNKNOWN"
    weight = getattr(box, "weight", None)
    support_ids = getattr(box, "support_ids", None)
    support_count = getattr(box, "support_count", None)

    x, y, z = box.position

    lines = [
        f"<b>{identifier}</b>",
        f"SKU: {sku}",
        f"Position: x={float(x):.2f}, y={float(y):.2f}, z={float(z):.2f} {unit_label}",
        (
            "Size: "
            f"{float(box.length):.2f} × {float(box.width):.2f} × {float(box.height):.2f} "
            f"{unit_label}"
        ),
    ]

    if weight is not None:
        lines.append(f"Weight: {float(weight):.2f} kg")

    if support_count is not None:
        lines.append(f"Support count: {support_count}")

    if support_ids:
        lines.append(f"Support IDs: {list(support_ids)}")

    return "<br>".join(lines)


def _make_text_label(box: Any, *, scale: float = 1.0) -> go.Scatter3d:
    """Create a floating text label at the center of a placed box."""
    x, y, z = box.position
    cx = (float(x) + float(box.length) / 2.0) * scale
    cy = (float(y) + float(box.width) / 2.0) * scale
    cz = (float(z) + float(box.height) / 2.0) * scale

    return go.Scatter3d(
        x=[cx],
        y=[cy],
        z=[cz],
        mode="text",
        text=[_box_identifier(box)],
        textposition="middle center",
        showlegend=False,
        hoverinfo="skip",
    )


def create_state_figure(
    state: Any,
    *,
    title: str = "Stable stacking result",
    show_box_labels: bool = False,
    cell_size: float | None = None,
    unit_label: str = "grid units",
    show_pallet_base: bool = True,
    pallet_base_height: float = 0.15,
) -> go.Figure:
    """Construct a 3D Plotly figure from a stable_stacking State object.

    Parameters
    ----------
    state:
        Final State object produced by your algorithm.
    title:
        Plot title.
    show_box_labels:
        If True, draw each box id in the center of its cuboid.
    cell_size:
        Optional scaling factor. Use 1.0 for grid-cell units. If one cell equals
        100 mm and your heights are also in cells, pass 100.0 and set
        unit_label="mm".
    unit_label:
        Axis and hover-text unit label.
    show_pallet_base:
        If True, draw a translucent base under the state map footprint.
    pallet_base_height:
        Base thickness before scaling.
    """
    boxes = _get_state_boxes(state)
    scale = _get_cell_size(state, cell_size)
    pallet_length, pallet_width = _get_state_shape(state, boxes)

    fig = go.Figure()
    seen_legend_groups: set[str] = set()
    max_x = pallet_length * scale
    max_y = pallet_width * scale
    max_z = 1.0 * scale

    if show_pallet_base:
        pallet_hover = (
            f"<b>Pallet/state grid</b><br>"
            f"Size: {pallet_length:.0f} × {pallet_width:.0f} cells<br>"
            f"Boxes: {len(boxes)}"
        )
        fig.add_trace(
            _make_cuboid_mesh(
                x=0.0,
                y=0.0,
                z=-pallet_base_height * scale,
                length=pallet_length * scale,
                width=pallet_width * scale,
                height=pallet_base_height * scale,
                color="rgb(120,120,120)",
                name="Pallet/state grid",
                hover_text=pallet_hover,
                opacity=0.12,
                legend_group="PALLET",
                show_legend=True,
            )
        )

    missing_position_ids: list[str] = []

    for box in boxes:
        if getattr(box, "position", None) is None:
            missing_position_ids.append(_box_identifier(box))
            continue

        x, y, z = box.position
        x = float(x) * scale
        y = float(y) * scale
        z = float(z) * scale
        length = float(box.length) * scale
        width = float(box.width) * scale
        height = float(box.height) * scale

        sku = getattr(box, "sku", "UNKNOWN") or "UNKNOWN"
        legend_group = f"SKU-{sku}"
        color = _stable_color_from_text(str(sku))
        show_legend = legend_group not in seen_legend_groups
        seen_legend_groups.add(legend_group)

        max_x = max(max_x, x + length)
        max_y = max(max_y, y + width)
        max_z = max(max_z, z + height)

        fig.add_trace(
            _make_cuboid_mesh(
                x=x,
                y=y,
                z=z,
                length=length,
                width=width,
                height=height,
                color=color,
                name=str(sku),
                hover_text=_box_hover_text(box, unit_label=unit_label),
                opacity=1.0,
                legend_group=legend_group,
                show_legend=show_legend,
            )
        )
        fig.add_trace(
            _make_wireframe(
                x=x,
                y=y,
                z=z,
                length=length,
                width=width,
                height=height,
                color="rgba(15,15,15,1.0)",
                name="Box edges",
                show_legend=False,
                expand=0.015 * scale,
            )
        )

        if show_box_labels:
            fig.add_trace(_make_text_label(box, scale=scale))

    if missing_position_ids:
        missing = ", ".join(missing_position_ids[:10])
        suffix = "..." if len(missing_position_ids) > 10 else ""
        raise ValueError(
            "Some boxes do not have box.position set, so they cannot be visualized: "
            f"{missing}{suffix}"
        )

    padding = max(1.0 * scale, 0.05 * max(max_x, max_y, max_z))

    fig.update_layout(
        title=title,
        scene=dict(
            xaxis=dict(
                title=f"X / pallet length ({unit_label})",
                range=[-padding, max_x + padding],
            ),
            yaxis=dict(
                title=f"Y / pallet width ({unit_label})",
                range=[-padding, max_y + padding],
            ),
            zaxis=dict(
                title=f"Z / height ({unit_label})",
                range=[-padding, max_z + padding],
            ),
            aspectmode="data",
            camera=dict(
                eye=dict(x=1.7, y=-1.9, z=1.25),
                up=dict(x=0.0, y=0.0, z=1.0),
            ),
        ),
        margin=dict(l=0, r=0, t=45, b=0),
        legend=dict(title="SKU / object", itemsizing="constant"),
    )

    return fig


def write_state_html(
    state: Any,
    output_path: str | Path = "state_layout.html",
    *,
    title: str = "Stable stacking result",
    show_box_labels: bool = False,
    open_in_browser: bool = False,
    include_plotlyjs: bool | str = "directory",
    cell_size: float | None = None,
    unit_label: str = "grid units",
    show_pallet_base: bool = True,
) -> Path:
    """Generate an interactive HTML visualization for a State object."""
    output = Path(output_path).resolve()

    fig = create_state_figure(
        state,
        title=title,
        show_box_labels=show_box_labels,
        cell_size=cell_size,
        unit_label=unit_label,
        show_pallet_base=show_pallet_base,
    )

    config = {"displaylogo": False, "scrollZoom": True, "responsive": True}
    fig.write_html(
        str(output),
        include_plotlyjs=include_plotlyjs,
        full_html=True,
        config=config,
    )

    if open_in_browser:
        webbrowser.open(output.as_uri())

    return output


# Backwards-friendly alias: lets you use a similar naming style to the old visualizer.
def write_layout_html(*args: Any, **kwargs: Any) -> Path:
    """Alias for write_state_html()."""
    return write_state_html(*args, **kwargs)
