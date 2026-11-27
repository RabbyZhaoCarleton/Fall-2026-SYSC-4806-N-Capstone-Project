"""Per-run metrics (ER1, ER2) from the recording, the maze's ground truth and the run record.

One pass over the recording feeds two consumers: maze_monitor's own analyzers
(through a replay Session, exactly as maze_monitor --replay builds it), which
give the safety, navigation and health events, and the extractors here, which
collect ground truth, odometry, maps, exits and statuses. compute() is pure
(numpy only), so it is unit-tested with synthetic data (ENR2).

Times are sim time in seconds since the start of the simulation; wall times
are given separately.
"""

import bisect
import math
import os
import time

import numpy as np

from .. import fileio
from ..experiment import is_robot_result
from . import PIPELINE_VERSION
from .geometry import (GridMap, compose, distance_to_boxes, free_samples, inverse,
                       match_exits, transform_points, wall_precision_recall, wrap)

COVERAGE_LEVELS = (0.5, 0.9, 0.995)   # "100%" means 99.5%: one unseen map cell is not a miss
SERIES_POINTS = 200
SAFETY_KEYS = {'contact': 'contact', 'near_wall': 'near_wall', 'blocked': 'blocked',
               'stuck': 'stuck', 'spinning': 'spinning', 'oscillation': 'oscillation',
               'slip': 'wheel_slip', 'bump': 'bumps'}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def stamp_of(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class PoseSeries:
    """Time-sorted 2D poses, sampled by linear interpolation (drift_report.py's Series)."""

    def __init__(self, t=None, p=None):
        self.t = list(t or [])
        self.p = list(p or [])

    def add(self, t, pose):
        self.t.append(t)
        self.p.append(pose)

    def finish(self):
        order = sorted(range(len(self.t)), key=self.t.__getitem__)
        self.t = [self.t[i] for i in order]
        self.p = [self.p[i] for i in order]
        return self

    def at(self, t):
        if not self.t or t < self.t[0] or t > self.t[-1]:
            return None
        i = bisect.bisect_left(self.t, t)
        if self.t[i] == t or i == 0:
            return self.p[i]
        t0, t1, a, b = self.t[i - 1], self.t[i], self.p[i - 1], self.p[i]
        w = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        return (a[0] + w * (b[0] - a[0]), a[1] + w * (b[1] - a[1]),
                wrap(a[2] + w * wrap(b[2] - a[2])))

    def __len__(self):
        return len(self.t)


class Extracted:
    """Everything compute() needs, in plain Python/numpy."""

    def __init__(self):
        self.clock = []            # [(receive ns, sim s)]
        self.truth = PoseSeries()
        self.wheels = PoseSeries()
        self.ekf = PoseSeries()
        self.map_odom = PoseSeries()
        self.odom_base = PoseSeries()
        self.maps = []             # [(sim, GridMap)] (kept until the start pose is known)
        self.coverage = []         # [(sim, fraction)]
        self.final_map = None
        self.statuses = []         # [(sim, status)]
        self.exits = []            # [(sim, [(x, y) map frame])]
        self.rosout = {}           # node -> {'warnings', 'errors', 'examples'}
        self.events = []           # maze_monitor events, as dicts with sim times
        self.monitor_summary = {}
        self.nav = {}
        self.messages = 0
        self.wall_span_s = None


def sim_of(clock, recv_ns):
    """Sim time at a receive time, from the recorded /clock (interpolated)."""
    if not clock:
        return None
    i = bisect.bisect_left(clock, (recv_ns, -1.0))
    if i <= 0:
        return clock[0][1]
    if i >= len(clock):
        return clock[-1][1]
    (n0, s0), (n1, s1) = clock[i - 1], clock[i]
    return s0 + (s1 - s0) * (recv_ns - n0) / (n1 - n0) if n1 > n0 else s1


# ================================================================ extraction
def extract(run_dir, meta, use_monitor=True):
    """Read the run's recording once. Returns an Extracted."""
    from maze_monitor.recorder import read_bag
    from maze_monitor.run_store import RunStore
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    ex = Extracted()
    info = RunStore(os.path.dirname(run_dir)).load(run_dir)
    session = None
    if use_monitor:
        from maze_monitor.session import Session
        from ..recording import monitor_config
        cfg = monitor_config('full')
        session = Session(cfg, 'replay', info, start_ns=info.get('start_ns'))
    classes = {}
    pending_status, pending_exits = [], []
    start_pose = [None]
    free_world = free_samples(meta, 0.05, 0.05)
    free_map = [None]

    def coverage_of(grid):
        if free_map[0] is None:
            return None
        return grid.known_fraction(free_map[0])

    wanted = {'/clock', '/ground_truth/poses', '/odom', '/odometry/filtered', '/tf', '/map',
              '/explore/status', '/explore/exits', '/rosout'}
    first_ns = last_ns = None
    for topic, type_name, data, t_ns in read_bag(info.bag_path):
        ex.messages += 1
        first_ns = t_ns if first_ns is None else first_ns
        last_ns = t_ns
        if session is not None:
            session.ingest(topic, type_name, data, t_ns)
        if topic not in wanted:
            continue
        cls = classes.get(type_name)
        if cls is None:
            cls = classes[type_name] = get_message(type_name)
        try:
            msg = deserialize_message(data, cls)
        except Exception:
            continue
        if topic == '/clock':
            ex.clock.append((t_ns, msg.clock.sec + msg.clock.nanosec * 1e-9))
        elif topic == '/ground_truth/poses':
            if msg.poses:
                p = msg.poses[0]
                pose = (p.position.x, p.position.y, yaw_of(p.orientation))
                ex.truth.add(stamp_of(msg.header), pose)
                if start_pose[0] is None:
                    start_pose[0] = pose
                    free_map[0] = transform_points(inverse(pose), free_world)
                    for sim, grid in ex.maps:
                        ex.coverage.append((sim, coverage_of(grid)))
                    ex.maps = []
        elif topic in ('/odom', '/odometry/filtered'):
            p = msg.pose.pose
            (ex.wheels if topic == '/odom' else ex.ekf).add(
                stamp_of(msg.header), (p.position.x, p.position.y, yaw_of(p.orientation)))
        elif topic == '/tf':
            for tr in msg.transforms:
                parent = tr.header.frame_id.lstrip('/')
                child = tr.child_frame_id.lstrip('/')
                if (parent, child) == ('map', 'odom'):
                    ex.map_odom.add(stamp_of(tr.header), (
                        tr.transform.translation.x, tr.transform.translation.y,
                        yaw_of(tr.transform.rotation)))
                elif (parent, child) == ('odom', 'base_footprint'):
                    ex.odom_base.add(stamp_of(tr.header), (
                        tr.transform.translation.x, tr.transform.translation.y,
                        yaw_of(tr.transform.rotation)))
        elif topic == '/map':
            grid = GridMap.from_msg(msg, stamp_of(msg.header))
            sim = grid.stamp if grid.stamp else sim_of(ex.clock, t_ns)
            ex.final_map = grid
            if free_map[0] is None:
                ex.maps.append((sim, grid))
            else:
                ex.coverage.append((sim, coverage_of(grid)))
        elif topic == '/explore/status':
            pending_status.append((t_ns, msg.status))
        elif topic == '/explore/exits':
            pending_exits.append((t_ns, [(f.centroid.x, f.centroid.y) for f in msg.frontiers]))
        elif topic == '/rosout':
            if msg.level >= 30:
                node = ex.rosout.setdefault(msg.name, {'warnings': 0, 'errors': 0,
                                                       'examples': []})
                node['errors' if msg.level >= 40 else 'warnings'] += 1
                if len(node['examples']) < 3 and msg.msg[:160] not in node['examples']:
                    node['examples'].append(msg.msg[:160])
    ex.truth.finish()
    ex.wheels.finish()
    ex.ekf.finish()
    ex.map_odom.finish()
    ex.odom_base.finish()
    ex.clock.sort()
    for t_ns, status in pending_status:
        if not ex.statuses or ex.statuses[-1][1] != status:
            ex.statuses.append((sim_of(ex.clock, t_ns), status))
    for t_ns, pts in pending_exits:
        ex.exits.append((sim_of(ex.clock, t_ns), pts))
    if first_ns is not None:
        ex.wall_span_s = (last_ns - first_ns) * 1e-9
    if session is not None:
        session.analyzer.finish(session.timeline.duration)
        ex.monitor_summary = session.summary()
        start_ns = session.start_ns or first_ns or 0
        for e in session.timeline.events:
            ex.events.append({
                'sim': sim_of(ex.clock, start_ns + int(e.t * 1e9)),
                'sim_end': None if e.t_end is None else
                sim_of(ex.clock, start_ns + int(e.t_end * 1e9)),
                'severity': e.severity, 'category': e.category, 'title': e.title,
                'key': e.key, 'count': e.count})
        v = session.timeline.value
        t_end = session.timeline.duration
        ex.nav = {'goals_succeeded': int(v('nav.goals_succeeded', t_end, 0) or 0),
                  'goals_failed': int(v('nav.goals_failed', t_end, 0) or 0),
                  'recoveries': int(v('nav.recoveries', t_end, 0) or 0),
                  'replans': int(v('nav.route_changes', t_end, 0) or 0)}
        # Save the monitor's summary for the run list, as maze_monitor --replay would.
        try:
            info.update(summary=ex.monitor_summary)
        except OSError:
            pass
    return ex


# ================================================================ computation
def _downsample(rows, n=SERIES_POINTS):
    if len(rows) <= n:
        return rows
    step = len(rows) / n
    return [rows[int(i * step)] for i in range(n)] + [rows[-1]]


def _first_at_or_above(series, level):
    for t, v in series:
        if v is not None and v >= level:
            return t
    return None


def localization_errors(ex, origin):
    """drift_report.py's method: each estimate against the truth, in the start frame."""
    origin_inv = inverse(origin)
    estimates = {'wheels': ex.wheels.at, 'ekf': ex.ekf.at}
    if len(ex.map_odom):
        def slam(t):
            mo, ob = ex.map_odom.at(t), ex.odom_base.at(t)
            if ob is None:
                return None
            return compose(mo if mo is not None else (0.0, 0.0, 0.0), ob)
        estimates['slam'] = slam
    stats = {}
    series = {'t': []}
    for name in estimates:
        series[name] = []
        stats[name] = {'max_pos': 0.0, 'max_yaw': 0.0, 'sum': 0.0, 'n': 0, 'last': None}
    step = max(1, len(ex.truth.t) // 2000)
    for k in range(0, len(ex.truth.t), step):
        t, p = ex.truth.t[k], ex.truth.p[k]
        true = compose(origin_inv, p)
        series['t'].append(round(t, 2))
        for name, fn in estimates.items():
            e = fn(t)
            if e is None:
                series[name].append(None)
                continue
            pos = math.hypot(e[0] - true[0], e[1] - true[1])
            yaw = abs(wrap(e[2] - true[2]))
            st = stats[name]
            st['max_pos'] = max(st['max_pos'], pos)
            st['max_yaw'] = max(st['max_yaw'], yaw)
            st['sum'] += pos
            st['n'] += 1
            st['last'] = (pos, yaw)
            series[name].append(round(pos, 4))
    out = {}
    for name, st in stats.items():
        if not st['n']:
            continue
        out[name] = {'final_pos_m': round(st['last'][0], 4),
                     'final_yaw_deg': round(math.degrees(st['last'][1]), 2),
                     'max_pos_m': round(st['max_pos'], 4),
                     'max_yaw_deg': round(math.degrees(st['max_yaw']), 2),
                     'mean_pos_m': round(st['sum'] / st['n'], 4)}
    idx = list(range(len(series['t'])))
    keep = _downsample(idx)
    series = {k: [v[i] for i in keep] for k, v in series.items()}
    return out, series


def compute(ex, meta, record, profile_radius, tolerance=None):
    """Metrics dict (ER1) from extracted data, the maze metadata and the run record."""
    outcome = record.get('outcome')
    pitch = meta['grid']['pitch']
    tolerance = tolerance or pitch
    m = {}
    sim0 = ex.clock[0][1] if ex.clock else (ex.truth.t[0] if len(ex.truth) else 0.0)
    sim_end = ex.clock[-1][1] if ex.clock else (ex.truth.t[-1] if len(ex.truth) else None)

    def rel(t):
        return None if t is None else round(t - sim0, 2)

    # ---- time
    names = [s for _, s in ex.statuses]
    end_expl = next(((t, s) for t, s in ex.statuses
                     if s in ('exploration_complete', 'exploration_incomplete')), (None, None))
    final = next(((t, s) for t, s in reversed(ex.statuses)
                  if s in ('returned_to_origin', 'return_failed')), (None, None))
    m['time'] = {
        'exploration_end_s': rel(end_expl[0]), 'exploration_end_status': end_expl[1],
        'mission_s': rel(final[0]) if final[0] is not None else rel(sim_end),
        'final_status': final[1] or (names[-1] if names else None),
        'sim_s': rel(sim_end), 'wall_s': record.get('wall_s'),
        'rtf_mean': record.get('rtf_mean'), 'rtf_min': record.get('rtf_min'),
        'low_rtf': record.get('low_rtf'),
    }
    # ---- exploration
    origin = ex.truth.p[0] if len(ex.truth) else (meta['spawn']['x'], meta['spawn']['y'], 0.0)
    dist = 0.0
    for a, b in zip(ex.truth.p, ex.truth.p[1:]):
        dist += math.hypot(b[0] - a[0], b[1] - a[1])
    cov = [(rel(t), round(c, 4)) for t, c in ex.coverage if c is not None and t is not None]
    final_cov = cov[-1][1] if cov else None
    m['exploration'] = {
        'distance_m': round(dist, 3),
        'coverage': final_cov,
        'time_to_coverage_50_s': _first_at_or_above(cov, COVERAGE_LEVELS[0]),
        'time_to_coverage_90_s': _first_at_or_above(cov, COVERAGE_LEVELS[1]),
        'time_to_coverage_100_s': _first_at_or_above(cov, COVERAGE_LEVELS[2]),
        'coverage_series': _downsample(cov),
        'free_area_m2': meta.get('free_area_m2'),
    }
    # ---- exits
    true_exits = [tuple(e['gap_centre']) for e in meta['exits']]
    final_detected = ex.exits[-1][1] if ex.exits else []
    detected_world = [tuple(p) for p in transform_points(origin, final_detected)] \
        if final_detected else []
    pairs, missed, false = match_exits(true_exits, detected_world, tolerance)
    matches = []
    for i, j, d in pairs:
        first = None
        for t, pts in ex.exits:
            if pts and any(math.hypot(w[0] - true_exits[i][0], w[1] - true_exits[i][1])
                           <= tolerance for w in transform_points(origin, pts)):
                first = rel(t)
                break
        e = meta['exits'][i]
        matches.append({'exit': i, 'side': e['side'], 'cell': e['cell'],
                        'detected': [round(v, 3) for v in detected_world[j]],
                        'error_m': round(d, 3), 'detected_at_s': first})
    start_cell = tuple(meta['start_cell'])

    def near_start(e):
        c, r = e['cell']
        return abs(c - start_cell[0]) + abs(r - start_cell[1]) <= 1
    m['exits'] = {
        'true': len(true_exits), 'found': len(pairs), 'missed': len(missed),
        'false': len(false), 'tolerance_m': tolerance, 'matches': matches,
        'missed_list': [{'exit': i, 'side': meta['exits'][i]['side'],
                         'cell': meta['exits'][i]['cell'],
                         'next_to_start': near_start(meta['exits'][i]),
                         'shortest_path_m': meta['exits'][i].get('shortest_path_m')}
                        for i in missed],
        'false_list': [[round(v, 3) for v in detected_world[j]] for j in false],
    }
    # ---- mapping
    if ex.final_map is not None:
        prec, rec, occ = wall_precision_recall(ex.final_map, origin, meta)
        m['mapping'] = {'wall_precision': None if prec is None else round(prec, 4),
                        'wall_recall': None if rec is None else round(rec, 4),
                        'occupied_cells': occ, 'resolution_m': ex.final_map.resolution}
    else:
        m['mapping'] = {'wall_precision': None, 'wall_recall': None, 'occupied_cells': 0}
    # ---- localization
    loc, loc_series = localization_errors(ex, origin) if len(ex.truth) else ({}, {})
    jumps = sum(e['count'] for e in ex.events if e['title'].startswith('Localization jump'))
    m['localization'] = {**loc, 'jumps': jumps, 'series': loc_series}
    # ---- return
    if len(ex.truth):
        last = ex.truth.p[-1]
        m['return'] = {'distance_to_start_m': round(math.hypot(last[0] - origin[0],
                                                               last[1] - origin[1]), 3),
                       'returned': 'returned_to_origin' in names}
    else:
        m['return'] = {'distance_to_start_m': None, 'returned': False}
    # ---- safety
    safety = {'min_clearance_m': None}
    if len(ex.truth):
        pts = np.array([(p[0], p[1]) for p in ex.truth.p[::max(1, len(ex.truth) // 4000)]])
        d = distance_to_boxes(pts, meta['walls']['boxes'])
        safety['min_clearance_m'] = round(float(d.min()) - profile_radius, 4)
    for key, name in SAFETY_KEYS.items():
        safety[f'{name}_events'] = sum(e['count'] for e in ex.events if e['key'] == key)
    safety['collision_events'] = sum(e['count'] for e in ex.events
                                     if e['category'] == 'collision')
    m['safety'] = safety
    # ---- navigation
    goals = sum(e['count'] for e in ex.events if e['key'] == 'new_goal')
    clears = sum(e['count'] for e in ex.events if e['key'].startswith('clear_'))
    m['navigation'] = {'goals_sent': goals, **ex.nav, 'costmap_clears': clears}
    # ---- health
    warnings = sum(v['warnings'] for v in ex.rosout.values())
    errors = sum(v['errors'] for v in ex.rosout.values())
    m['health'] = {'warnings': warnings, 'errors': errors,
                   'by_node': dict(sorted(ex.rosout.items(),
                                          key=lambda kv: -(kv[1]['errors'] * 1000 +
                                                           kv[1]['warnings'])))}
    # ---- main events, for failures in the export
    notable = [e for e in ex.events if e['severity'] in ('warning', 'error')]
    notable.sort(key=lambda e: (e['sim'] if e['sim'] is not None else 0))
    m['events'] = [{'t': rel(e['sim']), 'severity': e['severity'], 'category': e['category'],
                    'title': e['title'], 'count': e['count']} for e in notable[:40]]
    m['explore_statuses'] = [[rel(t), s] for t, s in ex.statuses]
    # ---- outcome (ER2)
    success = (outcome == 'complete' and m['exits']['found'] == m['exits']['true']
               and m['exits']['false'] == 0)
    m['outcome'] = {'outcome': outcome, 'reason': record.get('reason'),
                    'counts_as_robot_result': is_robot_result(outcome),
                    'mission_success': bool(success) if is_robot_result(outcome) else None,
                    'attempt': record.get('attempt')}
    return m


def empty_metrics(record, why):
    outcome = record.get('outcome')
    return {'outcome': {'outcome': outcome, 'reason': record.get('reason'),
                        'counts_as_robot_result': is_robot_result(outcome),
                        'mission_success': False if is_robot_result(outcome) else None,
                        'attempt': record.get('attempt')},
            'time': {'wall_s': record.get('wall_s'), 'sim_s': record.get('sim_s'),
                     'rtf_mean': record.get('rtf_mean'), 'rtf_min': record.get('rtf_min'),
                     'low_rtf': record.get('low_rtf')},
            'no_recording': why}


def process_run(run_dir, profile_radius=None, tolerance=None):
    """Compute and write <run>/metrics.json. Returns the metrics."""
    import json
    t0 = time.monotonic()
    with open(os.path.join(run_dir, 'run.json')) as f:
        run_meta = json.load(f)
    record = run_meta.get('experiment') or {}
    spec = {}
    try:
        spec = fileio.read_json(os.path.join(run_dir, 'run_spec.json'))
    except (OSError, ValueError):
        pass
    maze_rel = spec.get('maze') or record.get('maze')
    maze_dir = os.path.normpath(os.path.join(run_dir, maze_rel)) if maze_rel else None
    meta = fileio.read_json(os.path.join(maze_dir, 'maze.json')) if maze_dir else None
    if profile_radius is None:
        exp_dir = os.path.dirname(os.path.dirname(run_dir))
        profile_radius = _profile_radius(exp_dir)
    bag = os.path.join(run_dir, 'bag')
    has_bag = os.path.isdir(bag) and any(f.endswith('.db3') for f in os.listdir(bag))
    if meta is None or not has_bag:
        metrics = empty_metrics(record, 'no recording' if meta else 'maze metadata missing')
    else:
        try:
            ex = extract(run_dir, meta)
            metrics = compute(ex, meta, record, profile_radius, tolerance)
            metrics['messages'] = ex.messages
        except Exception as exc:
            if is_robot_result(record.get('outcome')):
                raise
            metrics = empty_metrics(record, f'recording unreadable: {exc}')
    key = record.get('key') or spec.get('key') or [None, None, None]
    metrics.update({
        'schema_version': fileio.SCHEMA_VERSION,
        'pipeline_version': PIPELINE_VERSION,
        'run_id': os.path.basename(run_dir),
        'key': key, 'set_index': key[0], 'maze_index': key[1], 'repeat': key[2],
        'attempt': record.get('attempt'), 'set_name': record.get('set_name') or
        spec.get('set_name'), 'maze': maze_rel,
        'processing_s': round(time.monotonic() - t0, 2),
    })
    fileio.write_json(os.path.join(run_dir, 'metrics.json'), metrics)
    return metrics


def _profile_radius(exp_dir):
    try:
        from .. import configuration, profiles
        cfg = fileio.read_yaml(os.path.join(exp_dir, configuration.CONFIG_FILE))
        return profiles.load(cfg.get('robot_profile', 'burger')).footprint_radius
    except Exception:
        return 0.105
