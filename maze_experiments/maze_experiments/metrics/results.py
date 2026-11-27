"""Processed results of an experiment, and its exports (ER5, ER8).

results/ holds only derived files and can be regenerated at any time
(`maze_experiments reprocess <experiment>`):
    results.json   aggregates, findings, chart list and per-run rows (the GUI reads it)
    runs.csv       one row per run attempt
    report.md      the Markdown export meant for an LLM
    charts/*.png   ER10
"""

import csv
import datetime
import io
import json
import os

import yaml

from .. import fileio
from . import PIPELINE_VERSION, fmt
from .aggregate import NUMERIC, aggregate, get_path
from .charts import draw_all
from .findings import CATEGORIES, build as build_findings

CSV_EXTRA = ['run_id', 'set_index', 'set_name', 'maze_index', 'repeat', 'attempt', 'outcome',
             'reason', 'counts_as_robot_result', 'mission_success', 'low_rtf', 'exits.true']
MAX_MD_BYTES = 200 * 1024


def run_row(row):
    rec, m = row['record'], row['metrics'] or {}
    key = rec.get('key') or [None, None, None]
    out = {
        'run_id': row['run_id'], 'set_index': key[0], 'set_name': rec.get('set_name') or
        m.get('set_name'), 'maze_index': key[1], 'repeat': key[2],
        'attempt': rec.get('attempt'), 'outcome': rec.get('outcome'),
        'reason': rec.get('reason'), 'counts_as_robot_result': rec.get('counts_as_robot_result'),
        'mission_success': (m.get('outcome') or {}).get('mission_success'),
        'low_rtf': rec.get('low_rtf'), 'exits.true': get_path(m, 'exits.true'),
        'processed': bool(row['metrics']),
    }
    for path, *_ in NUMERIC:
        out[path] = get_path(m, path)
    return out


def provenance_warnings(exp):
    prov = exp.meta.get('provenance') or {}
    out = []
    cap = prov.get('capstone') or {}
    if cap.get('uncommitted_changes'):
        out.append(f'The robot code had uncommitted changes when this experiment ran '
                   f'({len(cap.get("changed_files") or [])} file(s), diff sha256 '
                   f'{str(cap.get("diff_sha256"))[:12]}). The results may not be reproducible '
                   f'from commit {str(cap.get("commit"))[:10]}.')
    for name, r in ((prov.get('third_party') or {}).get('repos') or {}).items():
        if r.get('checked_out') and r.get('pinned') and \
                not str(r['checked_out']).startswith(str(r['pinned'])):
            out.append(f'{name} was at {str(r["checked_out"])[:10]}, not the pinned '
                       f'{r["pinned"]}.')
    return out


def build_results(exp):
    """Aggregate, write results.json, runs.csv, report.md and the charts."""
    agg = aggregate(exp)
    findings = build_findings(agg)
    out_dir = exp.results_dir
    charts = draw_all(agg, os.path.join(out_dir, 'charts'))
    rows = [run_row(r) for r in sorted(agg['rows'], key=lambda r: r['run_id'])]
    data = {
        'schema_version': fileio.SCHEMA_VERSION,
        'pipeline_version': PIPELINE_VERSION,
        'generated': datetime.datetime.now().isoformat(timespec='seconds'),
        'experiment': exp.id,
        'overall': agg['overall'],
        'sets': agg['sets'],
        'rtf_effect': agg['rtf_effect'],
        'findings': findings,
        'categories': list(CATEGORIES),
        'charts': charts,
        'runs': rows,
        'warnings': provenance_warnings(exp) + (
            ['Runs with low RTF have a different mission-success rate: the machine load may '
             'have affected the results.'] if agg['rtf_effect']['different'] else []),
    }
    fileio.write_json(os.path.join(out_dir, 'results.json'), data)
    fileio.write_text(os.path.join(out_dir, 'runs.csv'), runs_csv(rows))
    fileio.write_text(os.path.join(out_dir, 'report.md'), markdown(exp, data))
    return data


def load_results(exp):
    path = os.path.join(exp.results_dir, 'results.json')
    if not os.path.isfile(path):
        return None
    data = fileio.read_json(path)
    return data


def runs_csv(rows):
    buf = io.StringIO()
    cols = CSV_EXTRA + [p for p, *_ in NUMERIC]
    w = csv.writer(buf)
    w.writerow(cols)
    for r in rows:
        w.writerow(['' if r.get(c) is None else r.get(c) for c in cols])
    return buf.getvalue()


def all_json(exp, data):
    """ER8: all results as JSON, including every run's metrics."""
    runs = {}
    for rid in exp.run_ids():
        try:
            with open(os.path.join(exp.run_dir(rid), 'metrics.json')) as f:
                runs[rid] = json.load(f)
        except (OSError, ValueError):
            runs[rid] = None
    return json.dumps({**data, 'run_metrics': runs,
                       'configuration': exp.config.data,
                       'experiment': {k: v for k, v in exp.meta.items()}},
                      indent=1, default=str)


# ================================================================ Markdown (ER8)
def _table(columns, rows):
    out = ['| ' + ' | '.join(columns) + ' |', '|' + '---|' * len(columns)]
    for r in rows:
        out.append('| ' + ' | '.join(str(c).replace('|', '/') for c in r) + ' |')
    return '\n'.join(out)


def _rate_cell(r):
    return fmt.ci_text(r)


def markdown(exp, data, raw=None, events_per_failure=6):
    text = _markdown(exp, data, raw, events_per_failure)
    while len(text.encode()) > MAX_MD_BYTES and events_per_failure > 0:
        events_per_failure //= 2
        text = _markdown(exp, data, raw, events_per_failure)
    return text


def _markdown(exp, data, raw, events_per_failure):
    o = data['overall']
    L = []
    L.append(f'# Maze experiment results: {exp.name}')
    L.append('')
    L.append('This file describes one experiment of the maze robot test suite: a simulated '
             'TurtleBot3-class robot (lidar + SLAM + Nav2 + explore_lite) explores randomly '
             'generated mazes, identifies every exit, and returns to its start. It is meant to '
             'be pasted into an LLM for analysis. Times are simulation time unless marked '
             'wall time; durations are m:ss. Mission success means: outcome `complete`, every '
             'true exit found and no false exit.')
    L.append('')
    L.append(f'- Experiment: `{exp.id}` (state {exp.state}, created {exp.created})')
    L.append(f'- Configuration: {exp.meta.get("config_name")} (`{exp.config_id}`)')
    L.append(f'- Results pipeline version {data["pipeline_version"]}, generated '
             f'{data["generated"]}')
    for w in data.get('warnings') or []:
        L.append(f'- **Warning:** {w}')
    L.append('')
    L.append('## Summary')
    for s in data['findings']['experiment']['Overview']['findings']:
        L.append(f'- {s}')
    for s in data['findings']['experiment']['Performance']['findings']:
        L.append(f'- {s}')
    L.append('')
    L.append('## Configuration')
    L.append('```yaml')
    cfg = dict(exp.config.data)
    L.append(yaml.safe_dump(cfg, sort_keys=False).rstrip())
    L.append('```')
    L.append('')
    L.append('## Execution settings')
    ex = dict(exp.execution)
    machine = ex.pop('machine', {}) or {}
    L.append('```yaml')
    L.append(yaml.safe_dump({k: v for k, v in ex.items()}, sort_keys=False).rstrip())
    L.append('```')
    L.append('')
    L.append('## Provenance')
    prov = exp.meta.get('provenance') or {}
    cap = prov.get('capstone') or {}
    L.append(f'- Robot code: commit `{cap.get("commit")}` on `{cap.get("branch")}`, '
             f'uncommitted changes: {"yes" if cap.get("uncommitted_changes") else "no"}'
             + (f' (diff sha256 `{str(cap.get("diff_sha256"))[:16]}`)'
                if cap.get('uncommitted_changes') else ''))
    for name, r in ((prov.get('third_party') or {}).get('repos') or {}).items():
        L.append(f'- {name}: pinned `{r.get("pinned")}`, checked out '
                 f'`{str(r.get("checked_out"))[:10]}`, local changes: '
                 f'{"yes" if r.get("local_changes") else "no"}')
    patches = (prov.get('third_party') or {}).get('patches') or []
    if patches:
        L.append('- Patches: ' + ', '.join(os.path.basename(p['file']) for p in patches))
    for label, p in (prov.get('parameter_files') or {}).items():
        L.append(f'- {label}: `{os.path.basename(p.get("path") or "")}` sha256 '
                 f'`{str(p.get("sha256"))[:16]}`')
    v = prov.get('versions') or {}
    L.append(f'- ROS {v.get("ros_distro")} (rclpy {v.get("rclpy")}, Nav2 {v.get("nav2")}, '
             f'slam_toolbox {v.get("slam_toolbox")}), {v.get("gazebo")}')
    m = prov.get('machine') or machine
    r = m.get('renderer') or {}
    L.append(f'- Machine: {m.get("cpu_model")}, {m.get("logical_cpus")} logical CPUs, '
             f'{fmt.size(m.get("wsl_memory_bytes") or m.get("memory_total"))} WSL memory, '
             f'renderer {r.get("name") if isinstance(r, dict) else r}')
    L.append('')
    L.append('## Results per maze set')
    for s in data['sets']:
        L.append(f'### {s["name"]}')
        mz = s['maze']
        L.append(f'Maze: {mz["width"]} m x {mz["length"]} m, corridors {mz["corridor_width"]} m, '
                 f'walls {mz["wall_thickness"]} m, {mz["exits"]} exit(s), start '
                 f'{mz["start"]}, loop fraction {mz["loop_fraction"]}. Planned runs '
                 f'{s["planned"]}, robot results {s["robot_results"]}, excluded attempts '
                 f'{s["excluded_total"]}.')
        L.append('')
        rows = [['Mission success', _rate_cell(s['mission_success'])],
                ['All exits found', _rate_cell(s['all_exits_found'])],
                ['No false exit', _rate_cell(s['no_false_exit'])]]
        for oc, rr in s['outcomes'].items():
            if rr['k']:
                rows.append([f'Outcome {oc}', _rate_cell(rr)])
        for oc, n in s['excluded'].items():
            if n:
                rows.append([f'Excluded {oc}', str(n)])
        L.append(_table(['Rate', 'Share (95% CI)'], rows))
        L.append('')
        mrows = []
        from .aggregate import NUMERIC_INFO
        for path, (label, unit, kind) in NUMERIC_INFO.items():
            st = s['metrics'].get(path) or {}
            if not st.get('count'):
                continue
            f = lambda x: fmt.value(x, kind, unit)  # noqa: E731
            mrows.append([label, st['count'], f(st['mean']), f(st['median']),
                          f'{f(st["p10"])}–{f(st["p90"])}', f(st['min']), f(st['max'])])
        if mrows:
            L.append(_table(['Metric', 'n', 'Mean', 'Median', 'P10–P90', 'Min', 'Max'], mrows))
            L.append('')
    L.append('## Runs')
    L.append('One row per attempt. Excluded attempts (startup_failed, sim_frozen, infra_failure, '
             'interrupted) were repeated and do not count.')
    L.append('')
    rows = []
    for r in data['runs']:
        rows.append([r['run_id'], r['set_index'], r['maze_index'], r['repeat'], r['attempt'],
                     r['outcome'], {True: 'yes', False: 'no'}.get(r['mission_success'], '—'),
                     fmt.duration(r.get('time.mission_s')),
                     fmt.percent(r.get('exploration.coverage'), 1),
                     f'{r.get("exits.found") if r.get("exits.found") is not None else "—"}/'
                     f'{r.get("exits.true") if r.get("exits.true") is not None else "—"}',
                     r.get('exits.false') if r.get('exits.false') is not None else '—',
                     fmt.value(r.get('localization.slam.max_pos_m'), 'small_length'),
                     fmt.value(r.get('safety.min_clearance_m'), 'small_length'),
                     fmt.value(r.get('time.rtf_mean'), 'ratio')])
    L.append(_table(['Run', 'Set', 'Maze', 'Rep', 'Try', 'Outcome', 'Success', 'Mission',
                     'Coverage', 'Exits', 'False', 'SLAM max err', 'Min clear', 'RTF'], rows))
    L.append('')
    L.append('## Failures')
    fails = [r for r in data['runs'] if r['mission_success'] is not True]
    if not fails:
        L.append('None: every run was a mission success.')
    for r in fails:
        try:
            with open(os.path.join(exp.run_dir(r['run_id']), 'metrics.json')) as f:
                m = json.load(f)
        except (OSError, ValueError):
            m = {}
        L.append(f'### {r["run_id"]}: {r["outcome"]}')
        L.append(f'- Reason: {r.get("reason")}')
        ex_ = m.get('exits') or {}
        if ex_:
            L.append(f'- Exits: {ex_.get("found")}/{ex_.get("true")} found, {ex_.get("false")} '
                     f'false' + (''.join(
                         f'; missed {mm["side"]} of cell {tuple(mm["cell"])}'
                         + (' (next to start)' if mm.get('next_to_start') else '')
                         for mm in ex_.get('missed_list') or [])))
        sts = m.get('explore_statuses') or []
        if sts:
            L.append('- Explore status: ' + ', '.join(f'{s} at {fmt.duration(t)}'
                                                      for t, s in sts))
        for e in (m.get('events') or [])[:events_per_failure]:
            L.append(f'- {fmt.duration(e.get("t"))} [{e["severity"]}] {e["title"]}'
                     + (f' (x{e["count"]})' if e.get('count', 1) > 1 else ''))
        L.append('')
    if raw:
        L.append(raw_section(exp, raw))
    return '\n'.join(L) + '\n'


def raw_section(exp, raw):
    """The raw messages of selected topics of one run (2.3.2, ER8)."""
    from .echo import to_text
    from maze_monitor.recorder import read_bag
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    run, topics = raw['run'], set(raw['topics'])
    limit = int(raw.get('max_messages', 50))
    L = [f'## Raw messages of run {run}', '']
    counts = {t: 0 for t in topics}
    bag = os.path.join(exp.run_dir(run), 'bag')
    for topic, type_name, data, t_ns in read_bag(bag):
        if topic not in topics or counts[topic] >= limit:
            continue
        counts[topic] += 1
        msg = deserialize_message(data, get_message(type_name))
        L.append(f'`{topic}` ({type_name}) at {t_ns}:')
        L.append('```yaml')
        L.append(to_text(msg))
        L.append('```')
    return '\n'.join(L)
