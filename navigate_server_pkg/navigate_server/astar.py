"""
astar.py

8-directional A* path planner for a mecanum drive base
"""


from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from typing import Callable, Optional


# (dcol, drow, cost) for all 8 directions. Orthogonal = 1.0, diagonal =
# sqrt(2). This is the standard 8-connected grid cost and is the correct
# one for a holonomic base (no turn-in-place penalty needed).
_NEIGHBORS = [
    (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
    (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
    (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)),
]


@dataclass(order=True)
class _PQItem:
    f_score: float
    counter: int
    cell: tuple[int, int] = field(compare=False)


def _heuristic(a: tuple[int, int], b: tuple[int, int]) -> float:
    #Octile distance -- the admissible heuristic for 8-directional grids

    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)


def _no_corner_cutting(grid, col, row, dcol, drow) -> bool:
    #For diagonal moves, both orthogonal cells adjacent to the move must
    #also be free -- otherwise the path would cut through the corner of a
    #blocked cell

    if dcol != 0 and drow != 0:
        if not grid.is_free(col + dcol, row):
            return False
        if not grid.is_free(col, row + drow):
            return False
    return True


def find_path(
    grid,
    start_cell: tuple[int, int],
    goal_cell: tuple[int, int],
    abort_check: Optional[Callable[[], bool]] = None,
    feedback_callback: Optional[Callable[[int], None]] = None,
    feedback_every_n: int = 200,
) -> Optional[list[tuple[int, int]]]:
    #A* over an 8-connected grid.

    #Returns a list of (col, row) cells from start to goal inclusive, or


    if not grid.is_free(*start_cell) or not grid.is_free(*goal_cell):
        return None

    if start_cell == goal_cell:
        return [start_cell]

    open_heap: list[_PQItem] = []
    counter = 0

    g_score = {start_cell: 0.0}
    came_from: dict[tuple[int, int], tuple[int, int]] = {}

    heapq.heappush(open_heap, _PQItem(_heuristic(start_cell, goal_cell), counter, start_cell))
    closed: set[tuple[int, int]] = set()

    cells_expanded = 0

    while open_heap:
        if abort_check is not None and abort_check():
            return None

        current = heapq.heappop(open_heap).cell

        if current in closed:
            continue
        closed.add(current)

        cells_expanded += 1
        if feedback_callback is not None and cells_expanded % feedback_every_n == 0:
            feedback_callback(cells_expanded)

        if current == goal_cell:
            return _reconstruct_path(came_from, current)

        col, row = current
        for dcol, drow, step_cost in _NEIGHBORS:
            neighbor = (col + dcol, row + drow)

            if not grid.is_free(*neighbor):
                continue
            if not _no_corner_cutting(grid, col, row, dcol, drow):
                continue
            if neighbor in closed:
                continue

            tentative_g = g_score[current] + step_cost

            if tentative_g < g_score.get(neighbor, math.inf):
                g_score[neighbor] = tentative_g
                came_from[neighbor] = current
                f = tentative_g + _heuristic(neighbor, goal_cell)
                counter += 1
                heapq.heappush(open_heap, _PQItem(f, counter, neighbor))

    return None 


def _reconstruct_path(came_from, current) -> list[tuple[int, int]]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def _collinear_simplify(path: list[tuple[int, int]]) -> list[tuple[int, int]]:

    if len(path) <= 2:
        return path[:]

    simplified = [path[0]]

    def direction(a, b):
        dx, dy = b[0] - a[0], b[1] - a[1]
        m = math.hypot(dx, dy)
        return (dx / m, dy / m) if m else (0.0, 0.0)

    prev_dir = direction(path[0], path[1])
    for i in range(1, len(path) - 1):
        cur_dir = direction(path[i], path[i + 1])
        if cur_dir != prev_dir:
            simplified.append(path[i])
        prev_dir = cur_dir

    simplified.append(path[-1])
    return simplified


def _has_line_of_sight(grid, a: tuple[int, int], b: tuple[int, int]) -> bool:
    #True if the straight line between cells a and b passes through no
    #occupied cell and never cuts a blocked corner. 
    
    x1, y1 = a
    x2, y2 = b
    dist = math.hypot(x2 - x1, y2 - y1)
    steps = max(int(dist * 4), 1)  # 4 samples per cell-length, conservative

    prev_cell = a
    for s in range(steps + 1):
        t = s / steps
        x = x1 + (x2 - x1) * t
        y = y1 + (y2 - y1) * t
        col, row = round(x), round(y)

        if not grid.is_free(col, row):
            return False

        # corner-cutting along the sampled line forbidden, same rule as
        # the search itself, so string-pulling can't approve a shortcut
        # the original search would have refused to take.
        dcol, drow = col - prev_cell[0], row - prev_cell[1]
        if abs(dcol) == 1 and abs(drow) == 1:
            if not _no_corner_cutting(grid, prev_cell[0], prev_cell[1], dcol, drow):
                return False
        prev_cell = (col, row)

    return True


def _string_pull(grid, path: list[tuple[int, int]]) -> list[tuple[int, int]]:
    #skip ahead to the farthest waypoint still in direct line-of-sight. 
    #Cleans up the zig-zag staircase artifact
    
    if len(path) <= 2:
        return path[:]

    pulled = [path[0]]
    anchor_idx = 0

    while anchor_idx < len(path) - 1:
        farthest = anchor_idx + 1
        for candidate in range(len(path) - 1, anchor_idx, -1):
            if _has_line_of_sight(grid, path[anchor_idx], path[candidate]):
                farthest = candidate
                break
        pulled.append(path[farthest])
        anchor_idx = farthest

    return pulled


def simplify_path(path: list[tuple[int, int]], grid=None) -> list[tuple[int, int]]:
    #Simplifies a raw A* cell path into a small set of waypoints.


    collinear = _collinear_simplify(path)
    if grid is None:
        return collinear
    return _string_pull(grid, collinear)
