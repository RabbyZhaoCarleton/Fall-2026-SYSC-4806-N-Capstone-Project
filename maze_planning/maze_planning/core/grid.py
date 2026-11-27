"""The cost grid the global planner searches.

The global planner plans on the same grid Nav2 plans on: the global costmap
(``/global_costmap/costmap``, a ``nav_msgs/OccupancyGrid``). Using the costmap
instead of the raw SLAM map matters because the costmap already contains

* the inflation around walls (cells near walls are expensive, cells closer than
  the robot's radius are blocked), and
* the keepout zones (exit openings and collision areas, proposal 2.3.1) once the
  team adds Nav2's Keepout Filter.

So a route the global planner thinks is possible is also one Nav2 can drive.

This module has no ROS imports, so it can be unit tested (and used by the
offline simulator) on any computer.

Cell values (as Nav2 publishes them in an OccupancyGrid):
    -1      unknown (never seen by the lidar)
    0       free, far from walls
    1..98   free but near a wall (higher = closer)
    99      "inscribed": the robot's centre here would touch a wall
    100     lethal: a wall
"""

import math
from collections import deque

import numpy as np

UNKNOWN = -1
INSCRIBED = 99
LETHAL = 100


class CostGrid:
    """A 2-D grid of costs in the map frame.

    Cells are stored row by row, like an OccupancyGrid: cell (cx, cy) has index
    ``cy * width + cx``; cx grows with x and cy grows with y; cell (0, 0) has its
    lower-left corner at (origin_x, origin_y).
    """

    def __init__(self, width, height, resolution, origin_x, origin_y, data,
                 blocked_threshold=INSCRIBED, allow_unknown=False):
        """
        Args:
            width, height: grid size in cells.
            resolution: cell size in metres (0.05 in this project).
            origin_x, origin_y: map-frame position of the lower-left corner of cell (0, 0).
            data: width * height cell values (-1 or 0..100), row by row.
            blocked_threshold: cells with this value or more cannot be entered.
                99 (inscribed) matches Nav2: the robot's centre may not go there.
            allow_unknown: if False (proposal 2.3.2: allow_unknown = false),
                unknown cells cannot be entered. Exits lead into unknown space,
                so this also keeps the planner from routing outside the maze.
        """
        self.width = int(width)
        self.height = int(height)
        # OccupancyGrid stores the resolution as a 32-bit float (0.05 arrives as
        # 0.05000000074505806). Rounding to a micrometre gives back the intended
        # value, so a point on a cell boundary (x = 0.25) lands in the same cell
        # whether the grid came from a ROS message or not.
        self.resolution = round(float(resolution), 6)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)

        values = np.asarray(data, dtype=np.int16).reshape(-1)
        if values.size != self.width * self.height:
            raise ValueError(f'grid data has {values.size} cells, expected '
                             f'{self.width} x {self.height}')
        self.values = values

        # blocked[i] is True if the robot's centre may not be in cell i.
        blocked = values >= blocked_threshold
        if not allow_unknown:
            blocked |= values == UNKNOWN
        # kappa[i] is the normalised cost of cell i in [0, 1] (proposal 2.3.2's
        # kappa(v)): 0 far from walls, 1 right next to the inscribed zone.
        # Unknown cells (if allowed at all) count as mid-cost.
        kappa = np.clip(values, 0, 98).astype(np.float64) / 98.0
        kappa[values == UNKNOWN] = 0.5

        # Plain Python lists: indexing a list is much faster than indexing a
        # numpy array element by element, which is what the search does.
        self.blocked = blocked.tolist()
        # wall[i]: cell i is an actual obstacle (lethal), not just close to one.
        self.wall = (values >= LETHAL).tolist()
        self.kappa = kappa.tolist()

    # ------------------------------------------------------------ coordinates
    def index(self, cx, cy):
        return cy * self.width + cx

    def cell_of_index(self, idx):
        return idx % self.width, idx // self.width

    def in_bounds(self, cx, cy):
        return 0 <= cx < self.width and 0 <= cy < self.height

    def world_to_cell(self, x, y):
        """Cell containing map point (x, y), or None if it is outside the grid."""
        # The tiny epsilon puts points that are on a boundary up to rounding error
        # (0.25 / 0.05 = 4.999999999) consistently into the upper cell.
        cx = int(math.floor((x - self.origin_x) / self.resolution + 1e-9))
        cy = int(math.floor((y - self.origin_y) / self.resolution + 1e-9))
        return (cx, cy) if self.in_bounds(cx, cy) else None

    def cell_to_world(self, cx, cy):
        """Map position of the centre of cell (cx, cy)."""
        return (self.origin_x + (cx + 0.5) * self.resolution,
                self.origin_y + (cy + 0.5) * self.resolution)

    def index_to_world(self, idx):
        return self.cell_to_world(*self.cell_of_index(idx))

    def is_free(self, cx, cy):
        return self.in_bounds(cx, cy) and not self.blocked[self.index(cx, cy)]

    # ------------------------------------------------------------ helpers
    def nearest_free(self, x, y, max_distance):
        """Index of the free cell nearest to (x, y), searching up to max_distance metres.

        Used for the robot's own position: the robot can be a little inside the
        inflated zone near a wall (the costmap marks it "inscribed"), and a search
        that starts in a blocked cell would find nothing.
        Returns None if no free cell is that close.
        """
        start = self.world_to_cell(x, y)
        if start is None:
            return None
        if self.is_free(*start):
            return self.index(*start)
        max_cells = int(math.ceil(max_distance / self.resolution))
        # Breadth-first search outwards, then keep the closest free cell found.
        seen = {start}
        queue = deque([start])
        best, best_d2 = None, None
        while queue:
            cx, cy = queue.popleft()
            for nx, ny in ((cx + 1, cy), (cx - 1, cy), (cx, cy + 1), (cx, cy - 1)):
                if (nx, ny) in seen or not self.in_bounds(nx, ny):
                    continue
                if abs(nx - start[0]) > max_cells or abs(ny - start[1]) > max_cells:
                    continue
                seen.add((nx, ny))
                if self.is_free(nx, ny):
                    d2 = (nx - start[0]) ** 2 + (ny - start[1]) ** 2
                    if best_d2 is None or d2 < best_d2:
                        best, best_d2 = (nx, ny), d2
                else:
                    queue.append((nx, ny))
        if best is None or best_d2 > max_cells ** 2:
            return None
        return self.index(*best)

    def line_of_sight(self, a, b, max_kappa=1.0, walls_only=False):
        """True if the straight line between cells a and b is clear.

        a and b are cell indexes.
        Default (for the robot's own route): every cell on the line must be free,
        and no costlier than max_kappa, so a shortcut does not hug a wall more
        than the grid route did.
        walls_only (for "can the robot see / reach that object or exit from
        here?"): only real walls (lethal cells) block the line. The inflation
        zone does not: an object next to a wall is inside it, but the arm can
        still reach over it, and an exit opening leads into unknown space.
        The target cell b itself is not checked in this mode: an object touching
        a wall shares the wall's 5 cm cell.
        """
        x0, y0 = self.cell_of_index(a)
        x1, y1 = self.cell_of_index(b)

        def bad(x, y):
            i = self.index(x, y)
            if walls_only:
                return self.wall[i] and (x, y) != (x1, y1)
            return self.blocked[i] or self.kappa[i] > max_kappa

        # Walk every cell the line between the two cell centres passes through
        # ("grid walking"). At each step move one cell in x or in y, whichever
        # boundary the line crosses first. If it passes exactly through a corner,
        # both side cells are checked too, so the line can never slip diagonally
        # between two blocked cells.
        nx, ny = abs(x1 - x0), abs(y1 - y0)
        sx = 1 if x1 > x0 else -1
        sy = 1 if y1 > y0 else -1
        x, y = x0, y0
        ix = iy = 0
        if bad(x, y):
            return False
        while ix < nx or iy < ny:
            # Compare (0.5 + ix) / nx with (0.5 + iy) / ny without dividing.
            lhs = (1 + 2 * ix) * ny
            rhs = (1 + 2 * iy) * nx
            if lhs < rhs:
                x += sx
                ix += 1
            elif lhs > rhs:
                y += sy
                iy += 1
            else:  # exactly through a corner
                if bad(x + sx, y) or bad(x, y + sy):
                    return False
                x += sx
                y += sy
                ix += 1
                iy += 1
            if bad(x, y):
                return False
        return True
