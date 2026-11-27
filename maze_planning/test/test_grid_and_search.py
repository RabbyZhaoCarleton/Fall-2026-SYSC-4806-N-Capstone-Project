"""Cost grid and shortest-path search."""

import math

from maze_planning.core.grid import INSCRIBED, LETHAL, UNKNOWN, CostGrid
from maze_planning.core.search import dijkstra, extract_path, polyline_length, smooth_path


def make_grid(rows, res=1.0):
    """Grid from strings: '#' wall, '?' unknown, digits 0-9 -> cost 0..90, '.' free."""
    rows = list(reversed(rows))   # first string is the top row (largest y)
    data = []
    for line in rows:
        for ch in line:
            data.append({'#': LETHAL, '?': UNKNOWN, '.': 0}.get(ch, int(ch) * 10
                                                                if ch.isdigit() else 0))
    return CostGrid(len(rows[0]), len(rows), res, 0.0, 0.0, data)


def test_world_cell_round_trip():
    g = CostGrid(10, 5, 0.05, -1.0, 2.0, [0] * 50)
    assert g.world_to_cell(-1.0, 2.0) == (0, 0)
    assert g.world_to_cell(-0.976, 2.249) == (0, 4)
    assert g.world_to_cell(-1.01, 2.0) is None
    x, y = g.cell_to_world(3, 2)
    assert math.isclose(x, -1.0 + 3.5 * 0.05) and math.isclose(y, 2.0 + 2.5 * 0.05)


def test_unknown_and_inscribed_are_blocked():
    g = CostGrid(3, 1, 1.0, 0, 0, [UNKNOWN, INSCRIBED, 98])
    assert g.blocked == [True, True, False]
    g2 = CostGrid(3, 1, 1.0, 0, 0, [UNKNOWN, INSCRIBED, 98], allow_unknown=True)
    assert g2.blocked == [False, True, False]


def test_route_goes_around_a_wall():
    g = make_grid(['.....',
                   '.###.',
                   '.....'])
    s, t = g.index(0, 1), g.index(4, 1)   # left and right of the wall
    cost, parent = dijkstra(g, s, mu=0.0)
    cells = extract_path(parent, s, t)
    assert cells[0] == s and cells[-1] == t
    assert all(not g.blocked[c] for c in cells)
    # Around the wall: up 1, right 4, down 1. The diagonal shortcuts would cut
    # past the wall's corners, which is not allowed.
    assert math.isclose(cost[t], 6.0)


def test_no_corner_cutting():
    # The only gap is diagonal between two walls: not passable.
    g = make_grid(['.#',
                   '#.'])
    cost, _ = dijkstra(g, g.index(0, 1), mu=0.0)
    assert math.isinf(cost[g.index(1, 0)])


def test_costly_cells_are_avoided_with_mu():
    # Straight through the middle costs extra with mu > 0, so go around.
    g = make_grid(['.....',
                   '.999.',
                   '.....'])
    s, t = g.index(0, 1), g.index(4, 1)
    cheap, _ = dijkstra(g, s, mu=0.0)
    dear, parent = dijkstra(g, s, mu=5.0)
    assert math.isclose(cheap[t], 4.0)          # straight through
    cells = extract_path(parent, s, t)
    assert g.index(2, 1) not in cells           # avoided the expensive middle


def test_unreachable_target():
    g = make_grid(['..#..'])
    cost, parent = dijkstra(g, 0, mu=1.0)
    assert math.isinf(cost[4]) and extract_path(parent, 0, 4) is None


def test_line_of_sight_and_smoothing():
    g = make_grid(['......',
                   '......',
                   '..##..',
                   '......'])
    assert g.line_of_sight(g.index(0, 3), g.index(5, 3))
    assert not g.line_of_sight(g.index(0, 1), g.index(5, 1))   # through the wall
    s, t = g.index(0, 0), g.index(5, 3)
    _, parent = dijkstra(g, s, mu=0.0)
    cells = extract_path(parent, s, t)
    pts = smooth_path(g, cells, max_kappa=1.0)
    assert len(pts) <= len(cells)
    assert polyline_length(pts) <= len(cells) * math.sqrt(2)
    # Every straight segment of the smoothed route is still clear of the wall.
    idx = [g.world_to_cell(x, y) for x, y in pts]
    for a, b in zip(idx, idx[1:]):
        assert g.line_of_sight(g.index(*a), g.index(*b))


def test_float32_resolution_from_ros_gives_the_same_cells():
    # nav_msgs/OccupancyGrid carries the resolution as float32.
    exact = CostGrid(80, 80, 0.05, -0.5, -3.5, [0] * 6400)
    from_ros = CostGrid(80, 80, 0.05000000074505806, -0.5, -3.5, [0] * 6400)
    for x, y in [(0.25, -0.25), (0.4, -0.35), (1.75, 0.0), (3.0, -0.75), (0.0, -3.0)]:
        assert exact.world_to_cell(x, y) == from_ros.world_to_cell(x, y)
