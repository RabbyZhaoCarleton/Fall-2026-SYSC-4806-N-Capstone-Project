"""Shortest paths on the cost grid.

The global planner needs the travel cost between every pair of places in the
mission (robot start, each pickup, each exit drop-off). Instead of asking Nav2
for each pair (n^2 planner calls), it runs Dijkstra's algorithm once from each
place: one run gives the cheapest route from that place to *every* cell, so it
covers all the other places at once.

Edge cost (proposal 2.3.2, the same model Nav2's Smac 2D planner uses):

    c(u, v) = d(u, v) * (1 + mu * kappa(v))

    d(u, v)   distance between the cell centres: r straight, r*sqrt(2) diagonal
    mu        cost_travel_multiplier: how strongly routes keep away from walls
    kappa(v)  normalised costmap cost of the cell being entered (0..1)

Dijkstra (not A*) is used here because there is no single goal to aim a
heuristic at: we want the costs to several targets from one search. It stops as
soon as every target has been reached.
"""

import heapq
import math

SQRT2 = math.sqrt(2.0)


def dijkstra(grid, source, mu, targets=None):
    """Cheapest-route search from one cell.

    Args:
        grid: a CostGrid.
        source: index of the start cell (must be free).
        mu: cost_travel_multiplier.
        targets: optional iterable of cell indexes. The search stops once all of
            them have their final cost. None searches the whole reachable grid.

    Returns:
        (cost, parent): two lists over all cells. cost[i] is the cheapest cost
        from source to i (math.inf if unreachable); parent[i] is the previous
        cell on that route (-1 for the source and unreached cells).
    """
    width, height = grid.width, grid.height
    blocked, kappa = grid.blocked, grid.kappa
    r = grid.resolution
    n = width * height

    cost = [math.inf] * n
    parent = [-1] * n
    done = [False] * n
    remaining = set(targets) if targets is not None else None
    if remaining is not None:
        remaining.discard(source)

    cost[source] = 0.0
    heap = [(0.0, source)]
    while heap:
        c, i = heapq.heappop(heap)
        if done[i]:
            continue  # an old, more expensive entry for a cell already settled
        done[i] = True
        if remaining is not None:
            remaining.discard(i)
            if not remaining:
                break  # every target settled: nothing more to learn

        cx, cy = i % width, i // width
        # 8 neighbours. A diagonal move is only allowed if both cells it cuts
        # past are free, so routes never squeeze through the corner of a wall.
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                       (1, 1), (1, -1), (-1, 1), (-1, -1)):
            nx, ny = cx + dx, cy + dy
            if nx < 0 or ny < 0 or nx >= width or ny >= height:
                continue
            j = ny * width + nx
            if blocked[j] or done[j]:
                continue
            if dx and dy:
                if blocked[cy * width + nx] or blocked[ny * width + cx]:
                    continue
                step = r * SQRT2
            else:
                step = r
            nc = c + step * (1.0 + mu * kappa[j])
            if nc < cost[j]:
                cost[j] = nc
                parent[j] = i
                heapq.heappush(heap, (nc, j))
    return cost, parent


def extract_path(parent, source, target):
    """Cells from source to target (inclusive) following the parent links.

    Returns None if target was not reached by the search that made parent.
    """
    if target == source:
        return [source]
    if parent[target] < 0:
        return None
    cells = [target]
    while cells[-1] != source:
        p = parent[cells[-1]]
        if p < 0:
            return None
        cells.append(p)
    cells.reverse()
    return cells


def smooth_path(grid, cells, max_kappa):
    """Turn a cell-by-cell route into a short list of straight segments.

    A grid route zig-zags (it can only move in 8 directions), which overstates
    both its length and how much the robot turns. Nav2's planner and controller
    produce a smoother route, so to estimate driving time realistically the
    route is "string-pulled": from each corner, jump to the furthest later cell
    that can be seen in a straight line through cells no costlier than
    max_kappa. The result is a list of (x, y) points in the map frame.
    """
    if not cells:
        return []
    points = [cells[0]]
    anchor = 0
    while anchor < len(cells) - 1:
        # Walk forward from the anchor while the straight line stays clear; the
        # last cell that could still be seen becomes the next corner.
        nxt = anchor + 1
        while (nxt + 1 < len(cells)
               and grid.line_of_sight(cells[anchor], cells[nxt + 1], max_kappa)):
            nxt += 1
        points.append(cells[nxt])
        anchor = nxt
    return [grid.index_to_world(i) for i in points]


def polyline_length(points):
    """Total length of a list of (x, y) points, in metres."""
    return sum(math.hypot(x1 - x0, y1 - y0)
               for (x0, y0), (x1, y1) in zip(points, points[1:]))
