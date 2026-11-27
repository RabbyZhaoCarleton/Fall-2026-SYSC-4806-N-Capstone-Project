"""Module A tests: the property test (ANR3), make_maze.py parity (AR2), speed (ANR2)."""

import importlib.util
import os
import random
import re
import time

import pytest

from maze_experiments.mazegen import (MazeParams, generate, generate_set, max_exits,
                                      valid_corridor_widths, validate)
from maze_experiments.mazegen.files import strip_comments
from maze_experiments.mazegen.generator import exit_boundary_index
from maze_experiments.mazegen.mazeset import build_files, generate_checked, set_digest
from maze_experiments.mazegen.params import boundary_distance
from maze_experiments.profiles import load

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
MAKE_MAZE = os.path.join(REPO, 'maze_sim', 'scripts', 'make_maze.py')


@pytest.fixture(scope='module')
def burger():
    return load('burger')


def make_maze_module():
    spec = importlib.util.spec_from_file_location('make_maze', MAKE_MAZE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


WALL_RE = re.compile(r'<model name="(wall_\d+)">.*?<pose>([^<]+)</pose>.*?<size>([^<]+)</size>',
                     re.S)
MODEL_RE = re.compile(r'<model name="([^"]+)">')


def wall_boxes(sdf):
    return [(m.group(1), m.group(2), m.group(3)) for m in WALL_RE.finditer(sdf)]


def random_params(rng, profile):
    thick = rng.choice([0.01, 0.02, 0.03, 0.05])
    corridor = rng.choice([0.32, 0.38, 0.48, 0.58, 0.68, 0.98])
    pitch = corridor + thick
    cols, rows = rng.randint(1, 14), rng.randint(1, 14)
    sep = rng.choice([1, 2, 2, 3])
    start = [rng.randrange(cols), rng.randrange(rows)]
    mode = rng.choice(['fixed', 'fixed', 'random'])
    allow = rng.random() < 0.2
    limit = max_exits(cols, rows, sep, tuple(start) if mode == 'fixed' else (0, 0), allow)
    if limit < 1:
        return None
    return MazeParams(width=round(cols * pitch, 6), length=round(rows * pitch, 6),
                      corridor_width=corridor, wall_thickness=thick,
                      wall_height=rng.choice([0.25, 0.3, 0.4]), exits=rng.randint(1, limit),
                      min_exit_separation=sep, allow_exit_in_start_cell=allow, start=mode,
                      start_cell=start, loop_fraction=rng.choice([0.0, 0.0, 0.1, 0.3]))


def check_world(sdf, meta, profile):
    """AR9: only the maze and a ground plane covering the maze plus the margin."""
    names = MODEL_RE.findall(sdf)
    assert names[0] == 'ground_plane'
    assert all(n.startswith('wall_') for n in names[1:])
    g = meta['grid']
    width, length = g['columns'] * g['pitch'], g['rows'] * g['pitch']
    t = g['wall_thickness']
    margin = profile.lidar_range + 0.5
    (cx, cy), (sx, sy) = meta['ground_plane']['center'], meta['ground_plane']['size']
    assert cx - sx / 2 <= -t / 2 - margin + 1e-3
    assert cx + sx / 2 >= width + t / 2 + margin - 1e-3
    assert cy + sy / 2 >= t / 2 + margin - 1e-3
    assert cy - sy / 2 <= -length - t / 2 - margin + 1e-3
    for _, pose, size in wall_boxes(sdf):
        x, y = map(float, pose.split()[:2])
        a, b = map(float, size.split()[:2])
        assert -t <= x - a / 2 and x + a / 2 <= width + t
        assert -length - t <= y - b / 2 and y + b / 2 <= t


def test_property_1000_mazes(burger):
    """ANR3: AR3, AR4, AR6, AR8 and AR9 hold for 1000 mazes over random valid parameters."""
    rng = random.Random(20261126)
    done = 0
    while done < 1000:
        params = random_params(rng, burger)
        if params is None:
            continue
        errors, _ = validate(params, burger)
        assert not errors, (params, errors)                       # AR6, AR7 accept it
        assert params.columns * params.pitch == pytest.approx(params.width, abs=1e-3)
        seed = rng.randrange(2 ** 40)
        maze, _ = generate_checked(params, seed)
        # AR4: every cell and every exit reachable (also checked inside generate_checked)
        assert maze.check_reachable() == []
        assert len(maze.distances()) == maze.columns * maze.rows
        # AR3: exit count, separation along the boundary, not in the start cell
        assert len(maze.exits) == params.exits
        perimeter = 2 * (maze.columns + maze.rows)
        idx = [exit_boundary_index(maze, e) for e in maze.exits]
        for i in range(len(idx)):
            for j in range(i + 1, len(idx)):
                assert boundary_distance(idx[i], idx[j], perimeter) >= params.min_exit_separation
        if not params.allow_exit_in_start_cell:
            assert all((c, r) != maze.start for _, c, r in maze.exits)
        # Exactly the exits are gaps in the outer wall.
        gaps = sum(1 for c in range(maze.columns) if not maze.h[0][c]) \
            + sum(1 for c in range(maze.columns) if not maze.h[maze.rows][c]) \
            + sum(1 for r in range(maze.rows) if not maze.v[r][0]) \
            + sum(1 for r in range(maze.rows) if not maze.v[r][maze.columns])
        assert gaps == params.exits
        # AR8: same parameters and seed give byte-identical files
        files = build_files(maze, 0, seed, [], burger)
        again, _ = generate_checked(params, seed)
        assert build_files(again, 0, seed, [], burger) == files
        # AR9
        import json
        check_world(files['world.sdf'].decode(), json.loads(files['maze.json']), burger)
        done += 1


def test_layout_matches_make_maze(burger):
    """AR2: make_maze.py on a generated layout gives identical walls (its 0.5 m / 0.02 m grid)."""
    make_maze = make_maze_module()
    params = MazeParams(width=5, length=5, corridor_width=0.48, wall_thickness=0.02, exits=2)
    for seed in range(20):
        maze, _ = generate_checked(params, seed)
        files = build_files(maze, 0, seed, [], burger)
        ours = files['world.sdf'].decode()
        theirs = make_maze.build(strip_comments(files['layout.txt'].decode()))
        assert wall_boxes(ours) == wall_boxes(theirs)
        assert len(wall_boxes(ours)) == maze.wall_count()


def test_existing_layouts_parse_like_make_maze():
    """Our wall parser agrees with make_maze.py on the repository's own maze."""
    from maze_experiments.mazegen.files import walls_from_layout
    make_maze = make_maze_module()
    ref = wall_boxes(make_maze.build(make_maze.MAZE))
    ours = walls_from_layout(make_maze.MAZE, make_maze.CELL, make_maze.THICK)
    assert len(ours) == len(ref)


def test_speed_20x20(burger):
    """ANR2: a 20 x 20 maze including its checks in less than 1 s."""
    params = MazeParams(width=10, length=10, corridor_width=0.48, wall_thickness=0.02, exits=4,
                        loop_fraction=0.1)
    t = time.perf_counter()
    maze, _ = generate_checked(params, 7)
    build_files(maze, 0, 7, [], burger)
    assert time.perf_counter() - t < 1.0


def test_set_is_reproducible_and_distinct(tmp_path, burger):
    """AR8 and AR12: a set regenerates identically, and its mazes all differ."""
    params = MazeParams(width=1.5, length=1.5, exits=1)  # 3 x 3: few distinct mazes
    a = generate_set(params, 99, 12, str(tmp_path / 'a'), burger)
    b = generate_set(params, 99, 12, str(tmp_path / 'b'), burger)
    assert set_digest(a) == set_digest(b)
    layouts = [strip_comments(open(os.path.join(f, 'layout.txt')).read()) for f in a]
    import json
    starts = [json.load(open(os.path.join(f, 'maze.json')))['start_cell'] for f in a]
    keys = {(l, tuple(s)) for l, s in zip(layouts, starts)}
    assert len(keys) == 12


def test_validation_rules(burger):
    # AR6: pitch must divide; suggestions are offered
    errors, _ = validate(MazeParams(width=5, length=5, corridor_width=0.45), burger)
    assert errors and 'Nearest valid corridor widths' in errors[0]
    assert 0.48 in valid_corridor_widths(5, 5, 0.02, minimum=burger.min_corridor_width)
    # AR7: hard minimum from the profile, warning below the tested width
    errors, _ = validate(MazeParams(width=3.0, length=3.0, corridor_width=0.28,
                                    wall_thickness=0.02), burger)
    assert any('too narrow' in e for e in errors)
    errors, warnings = validate(MazeParams(width=4.0, length=4.0, corridor_width=0.38,
                                           wall_thickness=0.02), burger)
    assert not errors and warnings
    errors, _ = validate(MazeParams(wall_thickness=0.005, corridor_width=0.495), burger)
    assert errors
    errors, _ = validate(MazeParams(wall_height=0.2), burger)
    assert any('lower than the lidar' in e for e in errors)
    # AR3: exits limited by size and separation
    errors, _ = validate(MazeParams(width=1.0, length=1.0, exits=4), burger)
    assert errors
    assert max_exits(2, 2, 2, (0, 0)) == 3   # top of (1,0), right of (1,1), bottom of (0,1)
    assert max_exits(10, 10, 2, (0, 0)) == 19


def test_metadata(tmp_path, burger):
    """AR13: metadata holds parameters, walls, exits with world coordinates, spawn, paths."""
    import json
    params = MazeParams(width=2.0, length=2.0, exits=2)
    folder = generate_set(params, 5, 1, str(tmp_path), burger)[0]
    meta = json.load(open(os.path.join(folder, 'maze.json')))
    assert meta['spawn'] == {'x': 0.25, 'y': -0.25, 'yaw': 0.0}   # the launch defaults
    assert meta['grid']['columns'] == 4
    for e in meta['exits']:
        assert e['shortest_path_m'] > 0
        x, y = e['gap_centre']
        assert x in (0.0, 2.0) or y in (0.0, -2.0)
    # free area: 4 x 4 cells of 0.5 m minus walls
    assert 3.4 < meta['free_area_m2'] < 3.92
    assert os.path.getsize(os.path.join(folder, 'preview.png')) > 100
