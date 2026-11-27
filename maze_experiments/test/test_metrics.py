"""Module E tests with synthetic data, no ROS graph (ENR2)."""

import math
import os

import numpy as np
import pytest

from maze_experiments.mazegen import MazeParams
from maze_experiments.mazegen.mazeset import build_files, generate_checked
from maze_experiments.metrics import fmt
from maze_experiments.metrics.aggregate import (describe, fisher_exact, final_runs, rate,
                                                rtf_effect, wilson, aggregate_group)
from maze_experiments.metrics.geometry import (GridMap, distance_to_boxes, free_samples,
                                               match_exits, wall_precision_recall)
from maze_experiments.metrics.run_metrics import Extracted, PoseSeries, compute, sim_of
from maze_experiments.metrics.echo import summarize_array


def maze_meta(seed=4):
    import json
    params = MazeParams(width=2.0, length=2.0, exits=2)
    maze, _ = generate_checked(params, seed)
    return json.loads(build_files(maze, 0, seed, [])['maze.json'])


def perfect_map(meta, start, res=0.05, known=True):
    """An occupancy grid that is exactly the true maze, in the map frame (origin at start)."""
    g = meta['grid']
    w, l_ = g['columns'] * g['pitch'], g['rows'] * g['pitch']
    ox, oy = -start[0] - 0.3, -start[1] - l_ - 0.3     # map frame = world shifted by start
    width, height = int((w + 0.6) / res), int((l_ + 0.6) / res)
    xs = ox + (np.arange(width) + 0.5) * res + start[0]
    ys = oy + (np.arange(height) + 0.5) * res + start[1]
    gx, gy = np.meshgrid(xs, ys)
    pts = np.column_stack((gx.ravel(), gy.ravel()))
    d = distance_to_boxes(pts, meta['walls']['boxes'])
    inside = (pts[:, 0] > 0) & (pts[:, 0] < w) & (pts[:, 1] < 0) & (pts[:, 1] > -l_)
    data = np.full(len(pts), -1 if not known else 0, dtype=np.int8)
    data[~inside] = -1
    data[d <= res / 2] = 100
    return GridMap(width, height, res, (ox, oy, 0.0), data)


def synthetic(meta, coverage_steps=((10.0, False), (60.0, True)), exits_found=True,
              false_exit=False, statuses=None):
    ex = Extracted()
    start = (meta['spawn']['x'], meta['spawn']['y'], 0.0)
    ex.clock = [(int(1e9 * t), float(t)) for t in range(0, 201)]
    for k in range(0, 2001):
        t = k * 0.1
        ex.truth.add(t, (start[0] + 0.001 * k * 0, start[1], 0.0))
        ex.wheels.add(t, (0.00005 * k, 0.0, 0.0))   # drifts 10 cm over the run
        ex.ekf.add(t, (0.0, 0.0, 0.0))
        ex.odom_base.add(t, (0.0, 0.0, 0.0))
    ex.map_odom.add(0.0, (0.0, 0.0, 0.0))
    for t, known in coverage_steps:
        g = perfect_map(meta, start, known=known)
        free_map = free_samples(meta, 0.05, 0.05) - np.array(start[:2])
        ex.coverage.append((t, g.known_fraction(free_map)))
        ex.final_map = g
    exits = [tuple(np.array(e['gap_centre']) - np.array(start[:2])) for e in meta['exits']]
    if not exits_found:
        exits = exits[:1]
    if false_exit:
        exits.append((0.75 - start[0], -0.75 - start[1]))   # an inner point
    ex.exits = [(30.0, exits[:1]), (50.0, exits)]
    ex.statuses = statuses or [(20.0, 'exploration_started'), (120.0, 'exploration_complete'),
                               (120.0, 'returning_to_origin'), (180.0, 'returned_to_origin')]
    ex.truth.finish()
    return ex


def test_sim_of_interpolates():
    clock = [(0, 0.0), (int(1e9), 0.8), (int(2e9), 1.6)]
    assert sim_of(clock, int(1.5e9)) == pytest.approx(1.2)
    assert sim_of(clock, -5) == 0.0


def test_perfect_run_is_a_mission_success():
    meta = maze_meta()
    m = compute(synthetic(meta), meta, {'outcome': 'complete', 'attempt': 1}, 0.105)
    assert m['outcome']['mission_success'] is True
    assert m['exits']['found'] == 2 and m['exits']['false'] == 0
    assert m['exploration']['coverage'] == pytest.approx(1.0)
    assert m['exploration']['time_to_coverage_100_s'] == pytest.approx(60.0)
    assert m['time']['exploration_end_s'] == pytest.approx(120.0)
    assert m['time']['mission_s'] == pytest.approx(180.0)
    assert m['mapping']['wall_precision'] > 0.95
    assert m['mapping']['wall_recall'] > 0.95
    assert m['localization']['wheels']['final_pos_m'] == pytest.approx(0.1, abs=1e-3)
    assert m['localization']['ekf']['max_pos_m'] == pytest.approx(0.0, abs=1e-9)
    assert m['return']['distance_to_start_m'] == pytest.approx(0.0, abs=1e-6)
    # The robot stands still at the start-cell centre: 0.25 - wall half thickness - radius
    assert m['safety']['min_clearance_m'] == pytest.approx(0.25 - 0.01 - 0.105, abs=1e-3)


def test_missed_and_false_exits_fail_the_mission():
    meta = maze_meta()
    m = compute(synthetic(meta, exits_found=False), meta, {'outcome': 'complete'}, 0.105)
    assert m['exits']['missed'] == 1 and m['outcome']['mission_success'] is False
    m = compute(synthetic(meta, false_exit=True), meta, {'outcome': 'complete'}, 0.105)
    assert m['exits']['false'] == 1 and m['outcome']['mission_success'] is False
    m = compute(synthetic(meta), meta, {'outcome': 'return_failed'}, 0.105)
    assert m['outcome']['mission_success'] is False


def test_shifted_map_loses_precision():
    meta = maze_meta()
    start = (meta['spawn']['x'], meta['spawn']['y'], 0.0)
    good = perfect_map(meta, start)
    p, r, _ = wall_precision_recall(good, start, meta)
    shifted = GridMap(good.width, good.height, good.resolution,
                      (good.origin[0] + 0.15, good.origin[1], 0.0), good.data)
    p2, r2, _ = wall_precision_recall(shifted, start, meta)
    assert p > 0.95 and r > 0.95
    assert p2 < p - 0.2


def test_exit_matching_is_one_to_one():
    pairs, missed, false = match_exits([(0, 0), (1, 0)], [(0.1, 0), (0.2, 0), (5, 5)], 0.5)
    assert [(i, j) for i, j, _ in pairs] == [(0, 0)]
    assert missed == [1] and false == [1, 2]


def test_statistics():
    lo, hi = wilson(92, 100)
    assert lo == pytest.approx(0.850, abs=0.002) and hi == pytest.approx(0.959, abs=0.002)
    assert wilson(0, 0) == (None, None)
    d = describe([1, 2, 3, 4, None])
    assert d['count'] == 4 and d['median'] == 2.5 and d['min'] == 1
    assert d['std'] == pytest.approx(math.sqrt(5 / 3))
    # Fisher exact test, against known values
    assert fisher_exact(1, 9, 11, 3) == pytest.approx(0.0028, abs=1e-4)
    assert fisher_exact(5, 5, 5, 5) == pytest.approx(1.0)


def row(key, attempt, outcome, success=None, low_rtf=False):
    metrics = None if success is None else {'outcome': {'mission_success': success},
                                            'exits': {'true': 2, 'found': 2 if success else 1,
                                                      'missed': 0 if success else 1,
                                                      'false': 0, 'missed_list': []}}
    return {'run_id': f's{key[0]:02d}-m{key[1]:03d}-r{key[2]:02d}-a{attempt}',
            'record': {'key': list(key), 'attempt': attempt, 'outcome': outcome,
                       'low_rtf': low_rtf},
            'metrics': metrics}


def test_excluded_runs_are_counted_separately():
    rows = [row((0, 0, 0), 1, 'startup_failed'), row((0, 0, 0), 2, 'complete', True),
            row((0, 1, 0), 1, 'timeout', False), row((0, 2, 0), 1, 'infra_failure'),
            row((0, 2, 0), 2, 'interrupted'), row((0, 2, 0), 3, 'stalled', False)]
    counted, excluded = final_runs(rows)
    assert len(counted) == 3 and len(excluded) == 3
    agg = aggregate_group(counted, excluded)
    assert agg['mission_success']['k'] == 1 and agg['mission_success']['n'] == 3
    assert agg['excluded'] == {'startup_failed': 1, 'sim_frozen': 0, 'infra_failure': 1,
                               'interrupted': 1}
    assert agg['outcomes']['timeout']['k'] == 1


def test_rtf_effect_warns_when_rates_differ():
    rows = [row((0, i, 0), 1, 'complete', True) for i in range(20)]
    rows += [row((0, 20 + i, 0), 1, 'complete', False, low_rtf=True) for i in range(10)]
    counted, _ = final_runs(rows)
    eff = rtf_effect(counted, 0.8)
    assert eff['different'] and eff['p_value'] < 0.001
    rows = [row((0, i, 0), 1, 'complete', i % 2 == 0, low_rtf=i < 10) for i in range(20)]
    counted, _ = final_runs(rows)
    assert not rtf_effect(counted, 0.8)['different']


def test_formatting():
    assert fmt.duration(125) == '2:05'
    assert fmt.duration(3725) == '1:02:05'
    assert fmt.value(0.034, 'small_length') == '3.4 cm'
    assert fmt.value(0.92, 'percent') == '92.0%'
    assert fmt.rate_text(rate(92, 100)) == '92 of 100 (92%, 95% CI 85–96%)'


def test_long_arrays_are_summarized():
    text = summarize_array([1.0, 2.0, float('inf')] + [3.0] * 30)
    assert text.startswith('[33 values]') and '1 inf/nan' in text


def test_pose_series_interpolates_heading_across_pi():
    s = PoseSeries()
    s.add(0.0, (0, 0, math.pi - 0.1))
    s.add(1.0, (0, 0, -math.pi + 0.1))
    assert abs(abs(s.at(0.5)[2]) - math.pi) < 1e-9
