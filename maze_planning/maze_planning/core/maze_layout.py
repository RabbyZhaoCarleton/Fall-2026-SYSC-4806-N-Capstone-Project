"""Mazes drawn in the team's ASCII format, turned into costmaps.

The format is the one maze_sim/scripts/make_maze.py turns into Gazebo worlds
(and maze_sim/layouts/*.txt on the newer branches):

    +--+--+--+--+        "+--" a wall on a cell's top side, "+  " an opening
    |     |              "|" a wall on a cell's left side
    +  +  +  +--+        a missing wall on the outer boundary is an exit
    ...                  lines starting with "#" are comments

Cells are 0.5 m. The origin is the maze's top-left corner, x points right and
y points up, so the rows go *down* (row r is at y between -r*0.5 and -(r+1)*0.5).
This matches the Gazebo worlds, so the same file can drive the offline
simulator, the lightweight ROS simulator and the Gazebo simulation.

`layout_costmap` builds what Nav2's global costmap would look like once the
whole maze has been explored: walls lethal, inflation around them like Nav2's
inflation layer, and everything outside the maze unknown (the lidar sees no
return through an exit, so the outside is never mapped). Exits are therefore
openings into unknown space, just as on the real robot.
"""

import math
import random
from dataclasses import dataclass, field

import numpy as np

from .grid import INSCRIBED, LETHAL, UNKNOWN, CostGrid

CELL = 0.5      # m, maze cell size (maze_sim/scripts/make_maze.py)
THICK = 0.02    # m, wall thickness


@dataclass
class MazeLayout:
    rows: int
    cols: int
    cell: float = CELL
    # Wall segments as ((x0, y0), (x1, y1)) in the map frame.
    walls: list = field(default_factory=list)
    # Exits as (x, y): the middle of each gap in the outer boundary.
    exits: list = field(default_factory=list)
    text: str = ''

    def cell_centre(self, row, col):
        return (col * self.cell + self.cell / 2, -(row * self.cell + self.cell / 2))


def parse_layout(text, cell=CELL):
    """Read an ASCII maze. Returns a MazeLayout."""
    lines = [ln.rstrip('\n') for ln in text.splitlines()
             if ln.strip() and not ln.lstrip().startswith('#')]
    if len(lines) < 3 or len(lines) % 2 == 0:
        raise ValueError('a maze needs an odd number (3 or more) of drawing lines')
    cols = (len(lines[0].rstrip()) - 1) // 3
    rows = (len(lines) - 1) // 2
    layout = MazeLayout(rows=rows, cols=cols, cell=cell, text=text)

    for i, line in enumerate(lines):
        r = i // 2
        if i % 2 == 0:
            # Horizontal walls: characters 3c+1..3c+2 are "--" for a wall.
            y = -r * cell
            for c in range(cols):
                seg = line[3 * c + 1:3 * c + 3]
                if seg == '--':
                    layout.walls.append(((c * cell, y), ((c + 1) * cell, y)))
                elif r in (0, rows):
                    layout.exits.append(((c + 0.5) * cell, y))
        else:
            # Vertical walls: character 3c is "|" for a wall.
            y0, y1 = -r * cell, -(r + 1) * cell
            for c in range(cols + 1):
                is_wall = 3 * c < len(line) and line[3 * c] == '|'
                if is_wall:
                    layout.walls.append(((c * cell, y0), (c * cell, y1)))
                elif c in (0, cols):
                    layout.exits.append((c * cell, (y0 + y1) / 2))
    return layout


def load_layout(path, cell=CELL):
    with open(path) as f:
        return parse_layout(f.read(), cell)


def layout_costmap(layout, resolution=0.05, robot_radius=0.11, inflation_radius=0.2,
                   cost_scaling_factor=5.0, margin=0.5):
    """Costmap (as a CostGrid plus the raw values) of a fully explored maze.

    Mirrors Nav2's inflation layer (nav2_maze.yaml: robot_radius 0.11,
    inflation_radius 0.2, cost_scaling_factor 5.0):
        distance to wall <= robot_radius:   99 (inscribed)
        up to inflation_radius:             98 * exp(-factor * (d - robot_radius))
        further:                            0

    Returns (grid, values, info) where values is the width*height list of cell
    values (for publishing as an OccupancyGrid) and info has width, height,
    resolution, origin_x, origin_y.
    """
    width_m = layout.cols * layout.cell
    height_m = layout.rows * layout.cell
    origin_x = -margin
    origin_y = -height_m - margin
    w = int(math.ceil((width_m + 2 * margin) / resolution))
    h = int(math.ceil((height_m + 2 * margin) / resolution))

    # Cell centres.
    xs = origin_x + (np.arange(w) + 0.5) * resolution
    ys = origin_y + (np.arange(h) + 0.5) * resolution
    gx, gy = np.meshgrid(xs, ys)   # gy[row, col], row = cy

    # Distance from every cell centre to the nearest wall segment.
    dist = np.full((h, w), np.inf)
    for (x0, y0), (x1, y1) in layout.walls:
        sx, sy = x1 - x0, y1 - y0
        seg2 = sx * sx + sy * sy
        t = np.clip(((gx - x0) * sx + (gy - y0) * sy) / seg2, 0.0, 1.0)
        d = np.hypot(gx - (x0 + t * sx), gy - (y0 + t * sy))
        dist = np.minimum(dist, d)

    values = np.zeros((h, w), dtype=np.int16)
    # A cell is a wall if the wall passes through it.
    lethal = dist <= THICK / 2 + resolution / 2
    inscribed = dist <= robot_radius
    inflated = (dist <= inflation_radius) & ~inscribed
    values[inflated] = np.round(
        98.0 * np.exp(-cost_scaling_factor * (dist[inflated] - robot_radius))).astype(np.int16)
    values[inscribed] = INSCRIBED
    values[lethal] = LETHAL

    # Outside the maze is never mapped.
    outside = (gx < 0) | (gx > width_m) | (gy > 0) | (gy < -height_m)
    values[outside] = UNKNOWN

    flat = values.reshape(-1).tolist()
    grid = CostGrid(w, h, resolution, origin_x, origin_y, flat)
    info = dict(width=w, height=h, resolution=resolution,
                origin_x=origin_x, origin_y=origin_y)
    return grid, flat, info


def random_layout(rows, cols, n_exits, rng=None, extra_openings=0):
    """A random perfect maze (depth-first search) with n_exits boundary exits.

    extra_openings removes that many more inner walls, which creates loops
    (several routes between places), the harder case for route optimisation.
    Exits are never on the top-left cell, where the robot starts.
    """
    rng = rng or random.Random()
    # right[r][c] / down[r][c]: wall on the right / bottom side of cell (r, c).
    right = [[True] * cols for _ in range(rows)]
    down = [[True] * cols for _ in range(rows)]
    seen = [[False] * cols for _ in range(rows)]
    stack = [(0, 0)]
    seen[0][0] = True
    while stack:
        r, c = stack[-1]
        nbrs = [(r + dr, c + dc) for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1))
                if 0 <= r + dr < rows and 0 <= c + dc < cols and not seen[r + dr][c + dc]]
        if not nbrs:
            stack.pop()
            continue
        nr, nc = rng.choice(nbrs)
        if nr == r:
            right[r][min(c, nc)] = False
        else:
            down[min(r, nr)][c] = False
        seen[nr][nc] = True
        stack.append((nr, nc))

    inner = [('r', r, c) for r in range(rows) for c in range(cols - 1) if right[r][c]] + \
            [('d', r, c) for r in range(rows - 1) for c in range(cols) if down[r][c]]
    for kind, r, c in rng.sample(inner, min(extra_openings, len(inner))):
        (right if kind == 'r' else down)[r][c] = False

    # Boundary openings: (side, index).
    sides = [('top', c) for c in range(1, cols)] + [('bottom', c) for c in range(cols)] + \
            [('left', r) for r in range(1, rows)] + [('right', r) for r in range(rows)]
    exits = set(rng.sample(sides, n_exits))

    lines = []
    for r in range(rows + 1):
        h = '+'
        for c in range(cols):
            if r == 0:
                wall = ('top', c) not in exits
            elif r == rows:
                wall = ('bottom', c) not in exits
            else:
                wall = down[r - 1][c]
            h += ('--' if wall else '  ') + '+'
        lines.append(h)
        if r == rows:
            break
        v = '|' if ('left', r) not in exits else ' '
        for c in range(cols):
            last = c == cols - 1
            wall = ('right', r) not in exits if last else right[r][c]
            v += '  ' + ('|' if wall else ' ')
        lines.append(v)
    return '\n'.join(lines) + '\n'
