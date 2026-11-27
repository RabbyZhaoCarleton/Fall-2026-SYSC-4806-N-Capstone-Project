"""Plain-language findings and tables per results category (ER7, 2.3.2).

Categories: Overview, Outcomes, Exploration, Exits, Mapping, Localization,
Safety, Performance. Each gives a few sentences and tables, for the whole
experiment and per maze set. The GUI, the Markdown export and the CLI report
all use these, so they always say the same thing.
"""

from ..experiment import EXCLUDED_OUTCOMES, ROBOT_OUTCOMES
from . import fmt
from .aggregate import NUMERIC_INFO

CATEGORIES = ('Overview', 'Outcomes', 'Exploration', 'Exits', 'Mapping', 'Localization',
              'Safety', 'Performance')

CATEGORY_METRICS = {
    'Overview': ['time.mission_s', 'exploration.coverage', 'exits.found', 'exits.false'],
    'Outcomes': [],
    'Exploration': ['time.exploration_end_s', 'time.mission_s', 'exploration.distance_m',
                    'exploration.coverage', 'exploration.time_to_coverage_50_s',
                    'exploration.time_to_coverage_90_s', 'exploration.time_to_coverage_100_s',
                    'return.distance_to_start_m'],
    'Exits': ['exits.found', 'exits.missed', 'exits.false'],
    'Mapping': ['mapping.wall_precision', 'mapping.wall_recall'],
    'Localization': ['localization.slam.final_pos_m', 'localization.slam.max_pos_m',
                     'localization.slam.max_yaw_deg', 'localization.ekf.final_pos_m',
                     'localization.ekf.max_pos_m', 'localization.wheels.final_pos_m',
                     'localization.jumps'],
    'Safety': ['safety.min_clearance_m', 'safety.contact_events', 'safety.near_wall_events',
               'safety.blocked_events', 'safety.stuck_events', 'safety.spinning_events',
               'safety.oscillation_events'],
    'Performance': ['time.wall_s', 'time.rtf_mean', 'time.rtf_min', 'navigation.goals_sent',
                    'navigation.goals_succeeded', 'navigation.goals_failed',
                    'navigation.recoveries', 'navigation.replans', 'health.warnings',
                    'health.errors'],
}

OUTCOME_TEXT = {
    'complete': 'explored the whole maze and returned to the start',
    'incomplete': 'gave up on some frontiers',
    'return_failed': 'explored the maze but could not return to the start',
    'timeout': 'ran out of time',
    'stalled': 'stopped making progress',
    'stack_crash': 'had a robot software process crash',
    'startup_failed': 'did not start in time',
    'sim_frozen': 'had the simulation clock stop',
    'infra_failure': 'were stopped by the machine (memory, suspend, clock jump)',
    'interrupted': 'were interrupted',
}


def metric_table(agg, paths):
    rows = []
    for path in paths:
        st = agg['metrics'].get(path) or {}
        label, unit, kind = NUMERIC_INFO[path]
        if not st.get('count'):
            rows.append([label, '0', '—', '—', '—', '—', '—', '—'])
            continue
        f = lambda v: fmt.value(v, kind, unit)  # noqa: E731
        rows.append([label, str(st['count']), f(st['mean']),
                     fmt.value(st['std'], kind if kind != 'percent' else 'percent', unit),
                     f(st['median']), f'{f(st["p10"])} – {f(st["p90"])}', f(st['min']),
                     f(st['max'])])
    return {'title': 'Metrics', 'columns': ['Metric', 'Runs', 'Mean', 'Std', 'Median',
                                            '10th–90th pct', 'Min', 'Max'], 'rows': rows}


def outcome_table(agg):
    rows = []
    for o in ROBOT_OUTCOMES:
        r = agg['outcomes'][o]
        if r['k']:
            rows.append([o, str(r['k']), fmt.ci_text(r)])
    for o in EXCLUDED_OUTCOMES:
        n = agg['excluded'].get(o, 0)
        if n:
            rows.append([f'{o} (excluded, retried)', str(n), '—'])
    return {'title': 'Outcomes', 'columns': ['Outcome', 'Runs', 'Share (95% CI)'], 'rows': rows}


def _median(agg, path):
    return (agg['metrics'].get(path) or {}).get('median')


def sentences(category, agg, rtf=None, scope='experiment'):
    n = agg['robot_results']
    out = []
    if n == 0 and category != 'Performance':
        if agg['excluded_total']:
            return [f'No run of this {scope} produced a robot result yet; '
                    f'{agg["excluded_total"]} attempt(s) were excluded.']
        return [f'No run of this {scope} has finished yet.']
    if category in ('Overview', 'Outcomes'):
        ms = agg['mission_success']
        if ms['n']:
            out.append(f'Mission success (explored, returned, every exit found, no false exit): '
                       f'{fmt.rate_text(ms)}.')
        top = sorted(((o, agg['outcomes'][o]['k']) for o in ROBOT_OUTCOMES),
                     key=lambda x: -x[1])
        parts = [f'{k} {OUTCOME_TEXT[o]} ({o})' for o, k in top if k]
        if parts:
            out.append(f'Of {n} runs, ' + '; '.join(parts) + '.')
        if agg['excluded_total']:
            ex = ', '.join(f'{v} {o}' for o, v in agg['excluded'].items() if v)
            out.append(f'{agg["excluded_total"]} attempt(s) did not count as robot results and '
                       f'were repeated ({ex}).')
    if category in ('Overview', 'Exits'):
        allf = agg['all_exits_found']
        if allf['n']:
            out.append(f'The robot found every exit in {fmt.rate_text(allf)}.')
        mx = agg['missed_exits']
        if mx['total']:
            out.append(f'{mx["total"]} exit(s) were missed in total; {mx["next_to_start"]} of '
                       f'them were in or next to the start cell.')
        nf = agg['no_false_exit']
        if nf['n'] and nf['k'] < nf['n']:
            out.append(f'{nf["n"] - nf["k"]} run(s) reported at least one false exit '
                       f'(an inner opening taken for an exit).')
    if category in ('Overview', 'Exploration'):
        cov = _median(agg, 'exploration.coverage')
        t = _median(agg, 'time.mission_s')
        d = _median(agg, 'exploration.distance_m')
        if cov is not None:
            out.append(f'Median coverage of the maze\'s free area: {fmt.percent(cov, 1)}; '
                       f'median mission time {fmt.duration(t)} (sim time), median distance '
                       f'{fmt.value(d, "length")}.')
        t90 = _median(agg, 'exploration.time_to_coverage_90_s')
        if category == 'Exploration' and t90 is not None:
            out.append(f'90% of the maze was known after a median {fmt.duration(t90)}.')
        ret = _median(agg, 'return.distance_to_start_m')
        if category == 'Exploration' and ret is not None:
            out.append(f'At the end, the robot stood a median '
                       f'{fmt.value(ret, "small_length")} from its start.')
    if category == 'Mapping':
        p, r = _median(agg, 'mapping.wall_precision'), _median(agg, 'mapping.wall_recall')
        if p is not None:
            out.append(f'Median wall precision {fmt.percent(p, 1)} (share of mapped walls '
                       f'within one map cell of a real wall) and recall {fmt.percent(r, 1)} '
                       f'(share of real walls that were mapped).')
    if category == 'Localization':
        s, e = _median(agg, 'localization.slam.max_pos_m'), \
            _median(agg, 'localization.ekf.max_pos_m')
        if s is not None:
            out.append(f'Median maximum position error against ground truth: SLAM '
                       f'{fmt.value(s, "small_length")}, EKF {fmt.value(e, "small_length")}.')
        j = agg['metrics'].get('localization.jumps') or {}
        if j.get('count'):
            out.append(f'Localization jumps (SLAM corrections over 10 cm or 5°): mean '
                       f'{j["mean"]:.1f} per run, at most {int(j["max"])}.')
    if category == 'Safety':
        c = agg['metrics'].get('safety.min_clearance_m') or {}
        if c.get('count'):
            out.append(f'The closest the robot came to a wall (true pose, robot edge): '
                       f'{fmt.value(c["min"], "small_length")}; median of the per-run minimum '
                       f'{fmt.value(c["median"], "small_length")}.')
        con = agg['metrics'].get('safety.contact_events') or {}
        if con.get('count'):
            touching = con['mean'] * con['count']
            out.append(f'The monitor detected {int(round(touching))} contact event(s) in '
                       f'{con["count"]} runs.')
    if category == 'Performance':
        rm = agg['metrics'].get('time.rtf_mean') or {}
        if rm.get('count'):
            out.append(f'Mean real-time factor {rm["mean"]:.2f} (lowest run minimum '
                       f'{(agg["metrics"].get("time.rtf_min") or {}).get("min", 0):.2f}).')
        if rtf is not None:
            out += rtf_sentences(rtf)
    return out


def rtf_sentences(rtf):
    out = []
    low, normal = rtf['low'], rtf['normal']
    if low['n'] == 0:
        out.append(f'No run fell below the minimum RTF of {rtf["min_rtf"]}.')
        return out
    if normal['n'] == 0:
        out.append(f'Every run ({low["n"]}) fell below the minimum RTF of {rtf["min_rtf"]}, so '
                   f'there is nothing to compare with. The machine was too loaded or too slow '
                   f'for the results to be trusted; run with fewer runs at once or on a faster '
                   f'machine.')
        return out
    out.append(f'{low["n"]} run(s) fell below the minimum RTF of {rtf["min_rtf"]}: mission '
               f'success {fmt.ci_text(low)} there against {fmt.ci_text(normal)} in the other '
               f'runs' + (f' (Fisher exact p = {rtf["p_value"]:.3f}).' if rtf['p_value']
                          is not None else '.'))
    if rtf['different']:
        out.append('WARNING: runs with low RTF have a different success rate. The machine load '
                   'may have affected the results.')
    return out


def build(agg_all):
    """{'experiment': {category: {...}}, 'sets': [{category: {...}}]}"""
    def one(agg, rtf, scope):
        cats = {}
        for c in CATEGORIES:
            tables = []
            if c in ('Overview', 'Outcomes'):
                tables.append(outcome_table(agg))
            if CATEGORY_METRICS[c]:
                tables.append(metric_table(agg, CATEGORY_METRICS[c]))
            cats[c] = {'findings': sentences(c, agg, rtf, scope), 'tables': tables}
        return cats
    return {'experiment': one(agg_all['overall'], agg_all['rtf_effect'], 'experiment'),
            'sets': [one(s, None, 'set') for s in agg_all['sets']]}
