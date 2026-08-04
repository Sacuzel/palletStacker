"""Interactive Plotly visualization for pallet-loading results.

The module is read-only with respect to the domain model. It converts one or
more fully populated :class:`Pallet` objects into a Plotly figure or an HTML
file, but it never changes box positions or pallet state.

Coordinate convention
---------------------
Box coordinates are pallet-local millimetres. The top loading surface of each
pallet is Z=0. The physical pallet base is drawn below that plane using the
pallet's ``base_height_mm`` value.
"""

from __future__ import annotations

import hashlib
import webbrowser
from collections import defaultdict
from pathlib import Path
from typing import Sequence

import plotly.graph_objects as go

from . import settings
from .box import Box, Dimensions3D, Point3D
from .pallet import Pallet

Color = str

# Vertex ordering:
#   0..3 = bottom face, counter-clockwise when viewed from above
#   4..7 = corresponding top vertices
_CUBOID_EDGES: tuple[tuple[int, int], ...] = (
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
)


def _stable_color_from_text(text: str) -> Color:
    """Return a deterministic, moderately bright RGB color for text."""

    digest = hashlib.sha256(text.encode("utf-8")).digest()
    red = 70 + digest[0] % 150
    green = 70 + digest[1] % 150
    blue = 70 + digest[2] % 150
    return f"rgb({red},{green},{blue})"


def _cuboid_vertices(
    *,
    x: float,
    y: float,
    z: float,
    length: float,
    width: float,
    height: float,
    expand_mm: float = 0.0,
) -> tuple[list[float], list[float], list[float]]:
    """Return the eight vertices of an axis-aligned cuboid."""

    if expand_mm < 0:
        raise ValueError("expand_mm must be non-negative.")

    x0 = x - expand_mm
    y0 = y - expand_mm
    z0 = z - expand_mm
    x1 = x + length + expand_mm
    y1 = y + width + expand_mm
    z1 = z + height + expand_mm

    xs = [x0, x1, x1, x0, x0, x1, x1, x0]
    ys = [y0, y0, y1, y1, y0, y0, y1, y1]
    zs = [z0, z0, z0, z0, z1, z1, z1, z1]
    return xs, ys, zs


def _cuboid_triangle_indices() -> tuple[list[int], list[int], list[int]]:
    """Return 12 consistently wound triangles for a closed cuboid."""

    triangles = (
        # Bottom, outward normal -Z
        (0, 2, 1),
        (0, 3, 2),
        # Top, outward normal +Z
        (4, 5, 6),
        (4, 6, 7),
        # Front, outward normal -Y
        (0, 1, 5),
        (0, 5, 4),
        # Right, outward normal +X
        (1, 2, 6),
        (1, 6, 5),
        # Back, outward normal +Y
        (2, 3, 7),
        (2, 7, 6),
        # Left, outward normal -X
        (3, 0, 4),
        (3, 4, 7),
    )
    return (
        [triangle[0] for triangle in triangles],
        [triangle[1] for triangle in triangles],
        [triangle[2] for triangle in triangles],
    )


def _box_geometry(box: Box) -> tuple[Point3D, Dimensions3D]:
    """Return current position and oriented dimensions for a placed box."""

    position = box.position
    dimensions = box.placed_dimensions
    if position is None or dimensions is None or box.orientation is None:
        raise ValueError(f"Box {box.box_id!r} is not placed.")
    return position, dimensions


def _box_hover_text(box: Box, pallet: Pallet) -> str:
    """Build the tooltip shown when hovering over a box."""

    position, dimensions = _box_geometry(box)
    sku = box.sku or "N/A"
    name_line = f"<br>Name: {box.name}" if box.name else ""

    return (
        f"<b>{box.box_id}</b><br>"
        f"SKU: {sku}{name_line}<br>"
        f"Pallet: {pallet.pallet_id}<br>"
        f"Orientation: {box.orientation.value}<br>"
        f"Position: x={position.x:.1f}, y={position.y:.1f}, "
        f"z={position.z:.1f} mm<br>"
        f"Placed size: {dimensions.x:.1f} × {dimensions.y:.1f} × "
        f"{dimensions.z:.1f} mm<br>"
        f"Weight: {box.weight_kg:.2f} kg"
    )


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
    """Create the solid faces of one cuboid."""

    xs, ys, zs = _cuboid_vertices(
        x=x,
        y=y,
        z=z,
        length=length,
        width=width,
        height=height,
    )
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
        lighting={
            "ambient": 0.65,
            "diffuse": 0.75,
            "specular": 0.08,
            "roughness": 0.9,
            "fresnel": 0.05,
        },
        lightposition={"x": 1500, "y": -2000, "z": 3000},
        name=name,
        legendgroup=legend_group,
        showlegend=show_legend,
        hovertext=[hover_text] * 8,
        hoverinfo="text",
    )


def _append_wireframe_coordinates(
    line_x: list[float | None],
    line_y: list[float | None],
    line_z: list[float | None],
    *,
    x: float,
    y: float,
    z: float,
    length: float,
    width: float,
    height: float,
    expand_mm: float,
) -> None:
    """Append one cuboid's 12 edges to combined line-coordinate arrays."""

    xs, ys, zs = _cuboid_vertices(
        x=x,
        y=y,
        z=z,
        length=length,
        width=width,
        height=height,
        expand_mm=expand_mm,
    )

    for start, end in _CUBOID_EDGES:
        line_x.extend((xs[start], xs[end], None))
        line_y.extend((ys[start], ys[end], None))
        line_z.extend((zs[start], zs[end], None))


def _make_wireframe_trace(
    *,
    line_x: list[float | None],
    line_y: list[float | None],
    line_z: list[float | None],
    name: str,
    color: str,
    width_px: float,
    legend_group: str | None = None,
) -> go.Scatter3d:
    """Create one combined 3D line trace from accumulated edge coordinates."""

    return go.Scatter3d(
        x=line_x,
        y=line_y,
        z=line_z,
        mode="lines",
        line={"color": color, "width": width_px},
        name=name,
        legendgroup=legend_group,
        showlegend=False,
        hoverinfo="skip",
    )


def create_pallet_figure(
    pallets: Sequence[Pallet],
    *,
    title: str = settings.PLOTLY_TITLE,
    show_box_labels: bool = settings.PLOTLY_SHOW_BOX_LABELS,
    pallet_gap_mm: float = settings.PLOTLY_PALLET_GAP_MM,
    edge_width_px: float = settings.PLOTLY_EDGE_WIDTH_PX,
    edge_expand_mm: float = settings.PLOTLY_EDGE_EXPAND_MM,
) -> go.Figure:
    """Construct a 3D scene containing all supplied pallets and boxes.

    ``edge_expand_mm`` moves the black wireframe slightly outside each mesh to
    reduce depth-buffer flicker. One millimetre is normally negligible relative
    to carton dimensions while still making shared boundaries easier to see.
    Set it to zero for geometrically exact edge positions.
    """

    if pallet_gap_mm < 0:
        raise ValueError("pallet_gap_mm must be non-negative.")
    if edge_width_px <= 0:
        raise ValueError("edge_width_px must be positive.")
    if edge_expand_mm < 0:
        raise ValueError("edge_expand_mm must be non-negative.")

    fig = go.Figure()
    seen_sku_groups: set[str] = set()
    edge_coordinates_by_sku: dict[
        str,
        tuple[list[float | None], list[float | None], list[float | None]],
    ] = defaultdict(lambda: ([], [], []))

    label_x: list[float] = []
    label_y: list[float] = []
    label_z: list[float] = []
    label_text: list[str] = []

    x_cursor = 0.0
    maximum_x = 0.0
    maximum_y = 0.0
    maximum_z = 0.0
    minimum_z = 0.0

    for pallet in pallets:
        x_offset = x_cursor
        x_cursor += pallet.length_mm + pallet_gap_mm

        maximum_x = max(maximum_x, x_offset + pallet.length_mm)
        maximum_y = max(maximum_y, pallet.width_mm)
        maximum_z = max(maximum_z, pallet.max_height_mm)
        minimum_z = min(minimum_z, -pallet.base_height_mm)

        pallet_hover = (
            f"<b>Pallet {pallet.pallet_id}</b><br>"
            f"Size: {pallet.length_mm:.1f} × {pallet.width_mm:.1f} mm<br>"
            f"Base height: {pallet.base_height_mm:.1f} mm<br>"
            f"Boxes: {pallet.box_count}<br>"
            f"Load: {pallet.current_load_kg:.2f} kg<br>"
            f"Stack height: {pallet.load_height_mm:.1f} mm<br>"
            f"Volume utilization: {pallet.volume_utilization:.1%}"
        )

        pallet_group = f"PALLET-{pallet.pallet_id}"
        fig.add_trace(
            _make_cuboid_mesh(
                x=x_offset,
                y=0.0,
                z=-pallet.base_height_mm,
                length=pallet.length_mm,
                width=pallet.width_mm,
                height=pallet.base_height_mm,
                color="rgb(150,115,75)",
                name=f"Pallet {pallet.pallet_id}",
                hover_text=pallet_hover,
                opacity=0.30,
                legend_group=pallet_group,
                show_legend=True,
            )
        )

        height_x: list[float | None] = []
        height_y: list[float | None] = []
        height_z: list[float | None] = []
        _append_wireframe_coordinates(
            height_x,
            height_y,
            height_z,
            x=x_offset,
            y=0.0,
            z=0.0,
            length=pallet.length_mm,
            width=pallet.width_mm,
            height=pallet.max_height_mm,
            expand_mm=0.0,
        )
        fig.add_trace(
            _make_wireframe_trace(
                line_x=height_x,
                line_y=height_y,
                line_z=height_z,
                name=f"Pallet {pallet.pallet_id} height limit",
                color="rgba(80,80,80,0.28)",
                width_px=2.0,
                legend_group=pallet_group,
            )
        )

        for box in pallet.boxes:
            position, dimensions = _box_geometry(box)
            sku = box.sku or "UNKNOWN"
            legend_group = f"SKU-{sku}"
            color = _stable_color_from_text(sku)
            show_legend = legend_group not in seen_sku_groups
            seen_sku_groups.add(legend_group)

            global_x = x_offset + position.x
            fig.add_trace(
                _make_cuboid_mesh(
                    x=global_x,
                    y=position.y,
                    z=position.z,
                    length=dimensions.x,
                    width=dimensions.y,
                    height=dimensions.z,
                    color=color,
                    name=sku,
                    hover_text=_box_hover_text(box, pallet),
                    opacity=1.0,
                    legend_group=legend_group,
                    show_legend=show_legend,
                )
            )

            edge_x, edge_y, edge_z = edge_coordinates_by_sku[legend_group]
            _append_wireframe_coordinates(
                edge_x,
                edge_y,
                edge_z,
                x=global_x,
                y=position.y,
                z=position.z,
                length=dimensions.x,
                width=dimensions.y,
                height=dimensions.z,
                expand_mm=edge_expand_mm,
            )

            if show_box_labels:
                center = box.center()
                label_x.append(x_offset + center.x)
                label_y.append(center.y)
                label_z.append(center.z)
                label_text.append(box.box_id)

    for legend_group, (edge_x, edge_y, edge_z) in edge_coordinates_by_sku.items():
        fig.add_trace(
            _make_wireframe_trace(
                line_x=edge_x,
                line_y=edge_y,
                line_z=edge_z,
                name="Box edges",
                color="rgba(15,15,15,1.0)",
                width_px=edge_width_px,
                legend_group=legend_group,
            )
        )

    if show_box_labels and label_text:
        fig.add_trace(
            go.Scatter3d(
                x=label_x,
                y=label_y,
                z=label_z,
                mode="text",
                text=label_text,
                textposition="middle center",
                showlegend=False,
                hoverinfo="skip",
                name="Box labels",
            )
        )

    # Useful defaults even when an empty pallet list is supplied.
    maximum_x = max(maximum_x, 1200.0)
    maximum_y = max(maximum_y, 800.0)
    maximum_z = max(maximum_z, 1800.0)
    minimum_z = min(minimum_z, -150.0)

    padding_mm = 100.0
    fig.update_layout(
        title=title,
        scene={
            "xaxis": {
                "title": "X / pallet length (mm)",
                "range": [-padding_mm, maximum_x + padding_mm],
            },
            "yaxis": {
                "title": "Y / pallet width (mm)",
                "range": [-padding_mm, maximum_y + padding_mm],
            },
            "zaxis": {
                "title": "Z / height (mm)",
                "range": [minimum_z - 20.0, maximum_z + padding_mm],
            },
            "aspectmode": "data",
            "dragmode": "orbit",
            "camera": {
                "eye": {"x": 1.7, "y": -1.9, "z": 1.25},
                "up": {"x": 0.0, "y": 0.0, "z": 1.0},
            },
        },
        margin={"l": 0, "r": 0, "t": 45, "b": 0},
        legend={
            "title": {"text": "SKU / object"},
            "itemsizing": "constant",
            "groupclick": "togglegroup",
        },
    )
    return fig


def write_layout_html(
    pallets: Sequence[Pallet],
    output_path: str | Path = settings.PLOTLY_OUTPUT_FILE,
    *,
    title: str = settings.PLOTLY_TITLE,
    show_box_labels: bool = settings.PLOTLY_SHOW_BOX_LABELS,
    open_in_browser: bool = settings.PLOTLY_OPEN_IN_BROWSER,
    include_plotlyjs: bool | str = settings.PLOTLY_INCLUDE_PLOTLYJS,
    pallet_gap_mm: float = settings.PLOTLY_PALLET_GAP_MM,
    edge_width_px: float = settings.PLOTLY_EDGE_WIDTH_PX,
    edge_expand_mm: float = settings.PLOTLY_EDGE_EXPAND_MM,
) -> Path:
    """Write the Plotly scene to HTML and optionally open a browser tab."""

    output = Path(output_path).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)

    figure = create_pallet_figure(
        pallets,
        title=title,
        show_box_labels=show_box_labels,
        pallet_gap_mm=pallet_gap_mm,
        edge_width_px=edge_width_px,
        edge_expand_mm=edge_expand_mm,
    )
    config = {
        "displaylogo": False,
        "scrollZoom": True,
        "responsive": True,
    }
    figure.write_html(
        str(output),
        include_plotlyjs=include_plotlyjs,
        full_html=True,
        config=config,
    )

    if open_in_browser:
        webbrowser.open_new_tab(output.as_uri())

    return output
