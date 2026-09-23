"""
grid_rasterizer.py

"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import yaml

# in meter
DEFAULT_CELL_SIZE_M = 0.05

FREE = 0
OCCUPIED = 1


@dataclass
class Grid:

    width_cells: int
    height_cells: int
    cell_size_m: float
    origin_x: float
    origin_y: float
    cells: list = field(repr=False)  

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        col = int((x - self.origin_x) / self.cell_size_m)
        row = int((y - self.origin_y) / self.cell_size_m)
        return col, row

    def cell_to_world(self, col: int, row: int) -> tuple[float, float]:
        x = self.origin_x + (col + 0.5) * self.cell_size_m
        y = self.origin_y + (row + 0.5) * self.cell_size_m
        return x, y

    def in_bounds(self, col: int, row: int) -> bool:
        return 0 <= col < self.width_cells and 0 <= row < self.height_cells

    def is_free(self, col: int, row: int) -> bool:
        if not self.in_bounds(col, row):
            return False
        return self.cells[row][col] == FREE

    def set_occupied(self, col: int, row: int) -> None:
        if self.in_bounds(col, row):
            self.cells[row][col] = OCCUPIED



# Map extent resolution: boundary polygon -> explicit YAML dims -> bbox


def _bbox_of_points(points_lists):
    xs, ys = [], []
    for points in points_lists:
        for x, y in points:
            xs.append(x)
            ys.append(y)
    if not xs:
        return None
    return min(xs), min(ys), max(xs), max(ys)


def _resolve_map_extent(data: dict) -> tuple[float, float, float, float]:

    polygons = data.get("polygons", [])
    boundary = next((p for p in polygons if p.get("type") == "boundary"), None)

    if boundary is not None:
        pts = boundary["points"]
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        return min_x, min_y, max_x - min_x, max_y - min_y

    map_section = data.get("map", {})
    if "width_m" in map_section and "height_m" in map_section:
        origin = map_section.get("origin", [0.0, 0.0])
        return origin[0], origin[1], map_section["width_m"], map_section["height_m"]

    # Fallback: bounding box over every feature in the map.
    point_lists = [p["points"] for p in polygons]
    for obj in data.get("objects", {}).values():
        point_lists.append([obj["position"]])

    bbox = _bbox_of_points(point_lists)
    if bbox is None:
        raise ValueError(
            "Cannot determine map extent: missing boundary polygon, missing "
            "width_m/height_m in YAML, and missing  features to "
            " recognize a bounding box from."
        )

    min_x, min_y, max_x, max_y = bbox

    pad = 0.5
    return min_x - pad, min_y - pad, (max_x - min_x) + 2 * pad, (max_y - min_y) + 2 * pad



# Rasterization


def _rasterize_polygon_edges(grid: Grid, points: list[tuple[float, float]], thickness_cells: int = 1):

    n = len(points)
    for i in range(n):
        x1, y1 = points[i]
        x2, y2 = points[(i + 1) % n]
        c1, r1 = grid.world_to_cell(x1, y1)
        c2, r2 = grid.world_to_cell(x2, y2)

        steps = max(abs(c2 - c1), abs(r2 - r1), 1)
        for s in range(steps + 1):
            t = s / steps
            col = round(c1 + (c2 - c1) * t)
            row = round(r1 + (r2 - r1) * t)
            for dc in range(-thickness_cells, thickness_cells + 1):
                for dr in range(-thickness_cells, thickness_cells + 1):
                    grid.set_occupied(col + dc, row + dr)


def _point_in_polygon(x: float, y: float, points: list[tuple[float, float]]) -> bool:
    #ray-casting point-in-polygon.
    inside = False
    n = len(points)
    j = n - 1
    for i in range(n):
        xi, yi = points[i]
        xj, yj = points[j]
        if ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi
        ):
            inside = not inside
        j = i
    return inside


def _mark_outside_boundary_occupied(grid: Grid, boundary_points: list[tuple[float, float]]):
    for row in range(grid.height_cells):
        for col in range(grid.width_cells):
            wx, wy = grid.cell_to_world(col, row)
            if not _point_in_polygon(wx, wy, boundary_points):
                grid.set_occupied(col, row)


def rasterize(data: dict, cell_size_m: float = DEFAULT_CELL_SIZE_M) -> Grid:
    #to build an occupancy Grid from a parsed map-editor YAML.

    min_x, min_y, width_m, height_m = _resolve_map_extent(data)

    width_cells = max(1, math.ceil(width_m / cell_size_m))
    height_cells = max(1, math.ceil(height_m / cell_size_m))

    cells = [[FREE for _ in range(width_cells)] for _ in range(height_cells)]
    grid = Grid(
        width_cells=width_cells,
        height_cells=height_cells,
        cell_size_m=cell_size_m,
        origin_x=min_x,
        origin_y=min_y,
        cells=cells,
    )

    polygons = data.get("polygons", [])
    boundary = next((p for p in polygons if p.get("type") == "boundary"), None)
    if boundary is not None:
        # Outside the boundary is unreachable.
        boundary_pts = [tuple(p) for p in boundary["points"]]
        _mark_outside_boundary_occupied(grid, boundary_pts)

    for poly in polygons:
        ptype = poly.get("type", "obstacle")
        if ptype == "boundary":
            continue  # handled above: the boundary itself is not an obstacle
        points = [tuple(p) for p in poly["points"]]
        # Obstacles get filled solid; walls are edges only 
        if ptype == "obstacle":
            for row in range(grid.height_cells):
                for col in range(grid.width_cells):
                    wx, wy = grid.cell_to_world(col, row)
                    if _point_in_polygon(wx, wy, points):
                        grid.set_occupied(col, row)
        else:  # for example thin wall
            _rasterize_polygon_edges(grid, points, thickness_cells=1)


    return grid


def load_grid(yaml_path: str, cell_size_m: float = DEFAULT_CELL_SIZE_M) -> Grid:
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f) or {}
    return rasterize(data, cell_size_m=cell_size_m)


def load_named_object_position(yaml_path: str, name: str) -> tuple[float, float] | None:
    #returns the world-space (x, y) of a named object from the map YAML, or None if it
    #does not exist. Ignores z / orientation
  
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f) or {}
    obj = data.get("objects", {}).get(name)
    if obj is None:
        return None
    pos = obj["position"]
    return float(pos[0]), float(pos[1])
