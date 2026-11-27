"""Random maze generation from a seed (AR3, AR4, AR8, AR10, AR11, AR12).

A maze is a grid of `columns` x `rows` cells. Row 0 is the top row; rows go
toward -y, as in make_maze.py (2.2, item 7). Walls are stored per grid line:

    h[r][c]  horizontal wall on grid line r (0 = top boundary, rows = bottom),
             above row r, spanning column c
    v[r][c]  vertical wall on grid line c (0 = left boundary, columns = right),
             left of column c, spanning row r

Generation is a randomized depth-first search (a perfect maze: one path
between any two cells), then optional loops, then exits. Everything uses one
random.Random seeded with the maze's seed, so the same parameters and seed give
the same maze (AR8). Reachability is checked afterwards by a breadth-first
search on the cell graph; it is never assumed from the algorithm (AR4).
"""

import hashlib
import random
from collections import deque

from .params import (allowed_exit_segments, blank_row_conflict, boundary_distance,
                     boundary_segments)


class GenerationError(RuntimeError):
    pass


def derive_seed(set_seed, index):
    """Seed of maze `index` of a set (AR8): stable across Python versions and machines."""
    digest = hashlib.sha256(f'maze-experiments:{int(set_seed)}:{int(index)}'.encode()).digest()
    return int.from_bytes(digest[:6], 'big')


class Maze:
    def __init__(self, params, seed, h, v, exits, start):
        self.params = params
        self.seed = seed
        self.h = h
        self.v = v
        self.exits = exits      # list of (side, column, row), in boundary order
        self.start = start      # (column, row)

    @property
    def columns(self):
        return len(self.h[0])

    @property
    def rows(self):
        return len(self.v)

    # ---------------------------------------------------------------- graph
    def open_neighbours(self, c, r):
        if r > 0 and not self.h[r][c]:
            yield c, r - 1
        if r < self.rows - 1 and not self.h[r + 1][c]:
            yield c, r + 1
        if c > 0 and not self.v[r][c]:
            yield c - 1, r
        if c < self.columns - 1 and not self.v[r][c + 1]:
            yield c + 1, r

    def distances(self, origin=None):
        """Cell steps from origin (default: the start) to every reachable cell (BFS)."""
        origin = origin or self.start
        dist = {origin: 0}
        q = deque([origin])
        while q:
            cell = q.popleft()
            for n in self.open_neighbours(*cell):
                if n not in dist:
                    dist[n] = dist[cell] + 1
                    q.append(n)
        return dist

    def check_reachable(self):
        """Problems with reachability (AR4); empty when every cell and exit is reachable."""
        dist = self.distances()
        problems = []
        unreachable = self.columns * self.rows - len(dist)
        if unreachable:
            problems.append(f'{unreachable} cell(s) cannot be reached from the start')
        for side, c, r in self.exits:
            if (c, r) not in dist:
                problems.append(f'exit {side} of cell ({c}, {r}) cannot be reached')
            if not self.is_gap(side, c, r):
                problems.append(f'exit {side} of cell ({c}, {r}) has a wall')
        return problems

    def is_gap(self, side, c, r):
        if side == 'top':
            return not self.h[0][c]
        if side == 'bottom':
            return not self.h[self.rows][c]
        if side == 'left':
            return not self.v[r][0]
        return not self.v[r][self.columns]

    def wall_count(self):
        return sum(map(sum, self.h)) + sum(map(sum, self.v))


def _carve(columns, rows, rng):
    h = [[True] * columns for _ in range(rows + 1)]
    v = [[True] * (columns + 1) for _ in range(rows)]
    visited = [[False] * columns for _ in range(rows)]
    start = (rng.randrange(columns), rng.randrange(rows))
    stack = [start]
    visited[start[1]][start[0]] = True
    while stack:
        c, r = stack[-1]
        options = []
        if r > 0 and not visited[r - 1][c]:
            options.append((c, r - 1))
        if r < rows - 1 and not visited[r + 1][c]:
            options.append((c, r + 1))
        if c > 0 and not visited[r][c - 1]:
            options.append((c - 1, r))
        if c < columns - 1 and not visited[r][c + 1]:
            options.append((c + 1, r))
        if not options:
            stack.pop()
            continue
        nc, nr = options[rng.randrange(len(options))]
        if nr != r:
            h[max(r, nr)][c] = False
        else:
            v[r][max(c, nc)] = False
        visited[nr][nc] = True
        stack.append((nc, nr))
    return h, v


def inner_walls(h, v):
    columns, rows = len(h[0]), len(v)
    out = []
    for r in range(1, rows):
        for c in range(columns):
            if h[r][c]:
                out.append(('h', r, c))
    for r in range(rows):
        for c in range(1, columns):
            if v[r][c]:
                out.append(('v', r, c))
    return out


def _add_loops(h, v, fraction, rng):
    if fraction <= 0:
        return 0
    walls = inner_walls(h, v)
    n = int(round(fraction * len(walls)))
    for kind, r, c in rng.sample(walls, n):
        (h if kind == 'h' else v)[r][c] = False
    return n


def _compatible(seg, chosen, perimeter, sep, v):
    """Separation rule (AR3), and no layout row left without any wall (make_maze.py)."""
    for c in chosen:
        if boundary_distance(seg[0], c[0], perimeter) < sep:
            return False
        if blank_row_conflict(seg, c):
            return False
    return True


def _pick_exits(params, columns, rows, start, rng, v):
    perimeter = 2 * (columns + rows)
    allowed = allowed_exit_segments(columns, rows, start, params.allow_exit_in_start_cell)
    sep = max(params.min_exit_separation, 1)
    want = params.exits
    for _ in range(200):
        order = list(allowed)
        rng.shuffle(order)
        chosen = []
        for seg in order:
            if _compatible(seg, chosen, perimeter, sep, v):
                chosen.append(seg)
                if len(chosen) == want:
                    return sorted(chosen)
    # Near the maximum, random picking rarely succeeds: take the greedy packing
    # from a random first exit, which is how max_exits() counts.
    firsts = list(allowed)
    rng.shuffle(firsts)
    for first in firsts:
        chosen = [first]
        for seg in sorted(allowed, key=lambda s: (s[0] - first[0]) % perimeter)[1:]:
            if ((seg[0] - chosen[-1][0]) % perimeter >= sep
                    and (first[0] - seg[0]) % perimeter >= sep
                    and _compatible(seg, chosen, perimeter, sep, v)):
                chosen.append(seg)
        if len(chosen) >= want:
            return sorted(rng.sample(chosen, want))
    raise GenerationError(f'{want} exits do not fit with a separation of {sep} cells')


def generate(params, seed):
    """One maze from parameters and a seed. Raises GenerationError if it is impossible."""
    columns, rows = params.columns, params.rows
    if columns < 1 or rows < 1:
        raise GenerationError('the grid has no cells')
    rng = random.Random(seed)
    h, v = _carve(columns, rows, rng)
    _add_loops(h, v, params.loop_fraction, rng)
    if params.start == 'random':
        start = (rng.randrange(columns), rng.randrange(rows))
    else:
        start = tuple(params.start_cell)
    exits = []
    for _, side, c, r in _pick_exits(params, columns, rows, start, rng, v):
        exits.append((side, c, r))
        if side == 'top':
            h[0][c] = False
        elif side == 'bottom':
            h[rows][c] = False
        elif side == 'left':
            v[r][0] = False
        else:
            v[r][columns] = False
    return Maze(params, seed, h, v, exits, start)


def exit_boundary_index(maze, exit_):
    for idx, side, c, r in boundary_segments(maze.columns, maze.rows):
        if (side, c, r) == tuple(exit_):
            return idx
    raise ValueError(exit_)
