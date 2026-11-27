"""Aggregation per maze set and per experiment (ER3, ER6). Pure Python (ENR2)."""

import json
import math
import os

from ..experiment import EXCLUDED_OUTCOMES, ROBOT_OUTCOMES, is_robot_result, read_run_record

# (path in metrics.json, label, unit, kind) — kind tells formatting (format.py)
NUMERIC = [
    ('time.exploration_end_s', 'Time to end of exploration', 's', 'duration'),
    ('time.mission_s', 'Total mission time', 's', 'duration'),
    ('time.wall_s', 'Wall time', 's', 'duration'),
    ('time.rtf_mean', 'Mean RTF', '', 'ratio'),
    ('time.rtf_min', 'Minimum RTF', '', 'ratio'),
    ('exploration.distance_m', 'Distance driven', 'm', 'length'),
    ('exploration.coverage', 'Coverage', '', 'percent'),
    ('exploration.time_to_coverage_50_s', 'Time to 50% coverage', 's', 'duration'),
    ('exploration.time_to_coverage_90_s', 'Time to 90% coverage', 's', 'duration'),
    ('exploration.time_to_coverage_100_s', 'Time to 100% coverage', 's', 'duration'),
    ('exits.found', 'Exits found', '', 'count'),
    ('exits.missed', 'Exits missed', '', 'count'),
    ('exits.false', 'False exits', '', 'count'),
    ('mapping.wall_precision', 'Wall precision', '', 'percent'),
    ('mapping.wall_recall', 'Wall recall', '', 'percent'),
    ('localization.slam.final_pos_m', 'SLAM final position error', 'm', 'small_length'),
    ('localization.slam.max_pos_m', 'SLAM max position error', 'm', 'small_length'),
    ('localization.slam.max_yaw_deg', 'SLAM max heading error', '°', 'angle'),
    ('localization.ekf.final_pos_m', 'EKF final position error', 'm', 'small_length'),
    ('localization.ekf.max_pos_m', 'EKF max position error', 'm', 'small_length'),
    ('localization.wheels.final_pos_m', 'Wheel odometry final position error', 'm',
     'small_length'),
    ('localization.jumps', 'Localization jumps', '', 'count'),
    ('return.distance_to_start_m', 'Final distance to start', 'm', 'small_length'),
    ('safety.min_clearance_m', 'Minimum clearance to a wall', 'm', 'small_length'),
    ('safety.contact_events', 'Contact events', '', 'count'),
    ('safety.near_wall_events', 'Near-wall events', '', 'count'),
    ('safety.blocked_events', 'Blocked events', '', 'count'),
    ('safety.stuck_events', 'Stuck events', '', 'count'),
    ('safety.spinning_events', 'Spinning events', '', 'count'),
    ('safety.oscillation_events', 'Oscillation events', '', 'count'),
    ('navigation.goals_sent', 'Goals sent', '', 'count'),
    ('navigation.goals_succeeded', 'Goals succeeded', '', 'count'),
    ('navigation.goals_failed', 'Goals failed', '', 'count'),
    ('navigation.recoveries', 'Recoveries', '', 'count'),
    ('navigation.replans', 'Replans', '', 'count'),
    ('health.warnings', 'Warnings in /rosout', '', 'count'),
    ('health.errors', 'Errors in /rosout', '', 'count'),
]
NUMERIC_INFO = {p: (label, unit, kind) for p, label, unit, kind in NUMERIC}


def get_path(d, path):
    for part in path.split('.'):
        if not isinstance(d, dict):
            return None
        d = d.get(part)
    return d


# ================================================================ statistics
def wilson(k, n, z=1.959964):
    """95% Wilson score interval for k successes out of n."""
    if n == 0:
        return None, None
    p = k / n
    denom = 1 + z * z / n
    centre = (p + z * z / (2 * n)) / denom
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denom
    return max(0.0, centre - half), min(1.0, centre + half)


def rate(k, n):
    lo, hi = wilson(k, n)
    return {'k': k, 'n': n, 'share': None if n == 0 else k / n, 'ci_low': lo, 'ci_high': hi}


def percentile(sorted_vals, q):
    if not sorted_vals:
        return None
    pos = (len(sorted_vals) - 1) * q
    lo = math.floor(pos)
    hi = math.ceil(pos)
    if lo == hi:
        return sorted_vals[int(pos)]
    return sorted_vals[lo] + (sorted_vals[hi] - sorted_vals[lo]) * (pos - lo)


def describe(values):
    vals = sorted(float(v) for v in values if v is not None and not
                  (isinstance(v, float) and math.isnan(v)))
    n = len(vals)
    if n == 0:
        return {'count': 0}
    mean = sum(vals) / n
    std = math.sqrt(sum((v - mean) ** 2 for v in vals) / (n - 1)) if n > 1 else 0.0
    return {'count': n, 'mean': mean, 'std': std, 'median': percentile(vals, 0.5),
            'p10': percentile(vals, 0.1), 'p90': percentile(vals, 0.9),
            'min': vals[0], 'max': vals[-1]}


def fisher_exact(a, b, c, d):
    """Two-sided Fisher exact test p-value for the table [[a, b], [c, d]]."""
    n = a + b + c + d
    r1, c1 = a + b, a + c

    def logp(x):
        return (math.lgamma(r1 + 1) + math.lgamma(n - r1 + 1) + math.lgamma(c1 + 1)
                + math.lgamma(n - c1 + 1) - math.lgamma(n + 1) - math.lgamma(x + 1)
                - math.lgamma(r1 - x + 1) - math.lgamma(c1 - x + 1)
                - math.lgamma(n - r1 - c1 + x + 1))
    lo, hi = max(0, r1 + c1 - n), min(r1, c1)
    if hi < lo:
        return 1.0
    p_obs = logp(a)
    total = 0.0
    for x in range(lo, hi + 1):
        lp = logp(x)
        if lp <= p_obs + 1e-9:
            total += math.exp(lp)
    return min(1.0, total)


# ================================================================ collecting runs
def collect_runs(exp):
    """Every attempt of the experiment: run record + metrics (if processed)."""
    rows = []
    for rid in exp.run_ids():
        run_dir = exp.run_dir(rid)
        rec = read_run_record(run_dir)
        metrics = None
        try:
            with open(os.path.join(run_dir, 'metrics.json')) as f:
                metrics = json.load(f)
        except (OSError, ValueError):
            pass
        rows.append({'run_id': rid, 'dir': run_dir, 'record': rec, 'metrics': metrics})
    return rows


def final_runs(rows):
    """Per planned run: the attempt that counts (a robot result), plus all excluded attempts."""
    by_key = {}
    for row in rows:
        key = tuple(row['record'].get('key') or ())
        if len(key) != 3:
            from ..ids import parse_run_id
            parsed = parse_run_id(row['run_id'])
            if not parsed:
                continue
            key = parsed[:3]
        by_key.setdefault(key, []).append(row)
    counted, excluded = [], []
    for key, attempts in by_key.items():
        attempts.sort(key=lambda r: r['record'].get('attempt') or 0)
        robot = [r for r in attempts if is_robot_result(r['record'].get('outcome'))]
        if robot:
            counted.append(robot[-1])
        excluded += [r for r in attempts if r['record'].get('outcome') in EXCLUDED_OUTCOMES]
    return counted, excluded


def aggregate_group(counted, excluded):
    n = len(counted)
    successes = sum(1 for r in counted
                    if (r['metrics'] or {}).get('outcome', {}).get('mission_success'))
    processed = [r for r in counted if r['metrics']]
    out = {
        'robot_results': n,
        'processed': len(processed),
        'mission_success': rate(successes, len(processed)),
        'outcomes': {o: rate(sum(1 for r in counted if r['record'].get('outcome') == o), n)
                     for o in ROBOT_OUTCOMES},
        'excluded': {o: sum(1 for r in excluded if r['record'].get('outcome') == o)
                     for o in EXCLUDED_OUTCOMES},
        'excluded_total': len(excluded),
        'metrics': {path: describe(get_path(r['metrics'], path) for r in processed)
                    for path, *_ in NUMERIC},
    }
    exits_true = sum(get_path(r['metrics'], 'exits.true') or 0 for r in processed)
    exits_found = sum(get_path(r['metrics'], 'exits.found') or 0 for r in processed)
    out['exits_found_share'] = rate(exits_found, exits_true)
    out['all_exits_found'] = rate(sum(1 for r in processed
                                      if get_path(r['metrics'], 'exits.missed') == 0),
                                  len(processed))
    out['no_false_exit'] = rate(sum(1 for r in processed
                                    if get_path(r['metrics'], 'exits.false') == 0),
                                len(processed))
    missed = [m for r in processed for m in (get_path(r['metrics'], 'exits.missed_list') or [])]
    out['missed_exits'] = {'total': len(missed),
                           'next_to_start': sum(1 for m in missed if m.get('next_to_start'))}
    return out


def rtf_effect(counted, min_rtf):
    """ER6: do runs with low RTF have a different mission-success rate?"""
    processed = [r for r in counted if r['metrics']]
    low = [r for r in processed if r['record'].get('low_rtf')]
    ok = [r for r in processed if not r['record'].get('low_rtf')]

    def succ(rows):
        return sum(1 for r in rows if r['metrics']['outcome'].get('mission_success'))
    a, c = succ(low), succ(ok)
    b, d = len(low) - a, len(ok) - c
    p = fisher_exact(a, b, c, d) if low and ok else None
    return {'min_rtf': min_rtf, 'low': rate(a, len(low)), 'normal': rate(c, len(ok)),
            'p_value': p, 'different': p is not None and p < 0.05}


def aggregate(exp):
    rows = collect_runs(exp)
    counted, excluded = final_runs(rows)
    sets = exp.sets()
    per_set = []
    for k, s in enumerate(sets):
        c = [r for r in counted if (r['record'].get('key') or [None])[0] == k]
        e = [r for r in excluded if (r['record'].get('key') or [None])[0] == k]
        agg = aggregate_group(c, e)
        agg.update({'index': k, 'name': s['name'], 'maze': s['maze'],
                    'planned': int(s['mazes']) * int(s['repeats'])})
        per_set.append(agg)
    overall = aggregate_group(counted, excluded)
    overall['planned'] = len(exp.plan())
    min_rtf = float(exp.config.data['run_limits'].get('min_rtf', 0.8))
    return {'overall': overall, 'sets': per_set, 'rtf_effect': rtf_effect(counted, min_rtf),
            'rows': rows, 'counted': counted, 'excluded': excluded}


def experiment_summary(exp):
    """The small summary kept in experiment.json for the Experiments list (DNR2)."""
    rows = collect_runs(exp)
    counted, excluded = final_runs(rows)
    processed = [r for r in counted if r['metrics']]
    successes = sum(1 for r in processed if r['metrics']['outcome'].get('mission_success'))
    outcomes = {}
    for r in counted:
        o = r['record'].get('outcome')
        outcomes[o] = outcomes.get(o, 0) + 1
    planned = len(exp.plan())
    return {'planned': planned, 'robot_results': len(counted), 'processed': len(processed),
            'successes': successes,
            'success_rate': None if not processed else successes / len(processed),
            'excluded': len(excluded), 'attempts': len(rows), 'outcomes': outcomes}
