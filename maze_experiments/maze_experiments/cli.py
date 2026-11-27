"""The command line (GR1, FNR2). Every command uses operations.Suite, like the GUI."""

import json
import os
import subprocess
import sys

import yaml

from . import configuration, experiment as exp_mod, fileio, manager_control, resources
from .configuration import ConfigError
from .metrics import fmt
from .operations import Suite


def out(text=''):
    print(text)


def err(text):
    print(f'maze_experiments: {text}', file=sys.stderr)


def dispatch(args, data_dir):
    suite = Suite(data_dir)
    if args.command not in ('config', 'machine'):
        for exp in suite.find_orphans():
            err(f'experiment {exp.id} was running without a manager; it is now interrupted '
                f'(resume it with: maze_experiments resume {exp.id})')
    handler = globals()['cmd_' + args.command.replace('-', '_')]
    try:
        return handler(suite, args) or 0
    except ConfigError as exc:
        err('the configuration has problems:')
        for p in exc.problems:
            print(f'  - {p}', file=sys.stderr)
        return 1
    except KeyError as exc:
        err(str(exc.args[0]) if exc.args else str(exc))
        return 1
    except (ValueError, FileNotFoundError, fileio.SchemaError) as exc:
        err(str(exc))
        return 1


# ================================================================ config
EXAMPLE = os.path.join('config', 'examples', 'use_case_5x5.yaml')


def cmd_config(suite, args):
    cc = args.config_command
    if cc == 'example':
        from .paths import own_share
        with open(os.path.join(own_share(), EXAMPLE)) as f:
            out(f.read().rstrip())
        return 0
    if cc in ('create', 'check'):
        path = os.path.abspath(os.path.expanduser(args.file))
        with open(path) as f:
            raw = yaml.safe_load(f) or {}
        if raw.get('schema_version', 1) > fileio.SCHEMA_VERSION:
            fileio.check_schema(raw, path)
        if getattr(args, 'name', None):
            raw['name'] = args.name
        data = configuration.normalize(raw)
        if cc == 'check':
            errors, warnings = configuration.check(data, suite.data_dir, os.path.dirname(path))
            for w in warnings:
                out(f'warning: {w}')
            for e in errors:
                out(f'error: {e}')
            sets = configuration.expand_sets(data)
            out(f'{len(sets)} maze set(s), {configuration.total_runs(data)} runs. '
                + ('OK.' if not errors else f'{len(errors)} problem(s).'))
            return 1 if errors else 0
        cfg, warnings = suite.configs.create(raw, source_dir=os.path.dirname(path))
        for w in warnings:
            out(f'warning: {w}')
        out(f'Saved configuration "{cfg.name}" as {cfg.id} ({cfg.total_runs()} runs in '
            f'{len(cfg.sets())} set(s)).')
        out(f'  {cfg.path}')
        return 0
    if cc == 'list':
        exps = suite.experiments.list()
        rows = []
        for c in suite.configs.list():
            n = sum(1 for e in exps if e.config_id == c.id)
            rows.append([c.id, c.name, str(c.total_runs()), str(n)])
        table(['ID', 'Name', 'Runs', 'Experiments'], rows)
        return 0
    cfg = suite.configs.get(args.config)
    if cc == 'show':
        with open(cfg.file) as f:
            out(f.read().rstrip())
        out(f'# folder: {cfg.path}')
        return 0
    if cc == 'delete':
        suite.configs.delete(cfg, suite.experiments.list())
        out(f'Deleted configuration {cfg.id}.')
        return 0


def table(columns, rows):
    widths = [max([len(c)] + [len(str(r[i])) for r in rows]) for i, c in enumerate(columns)]
    out('  '.join(c.ljust(w) for c, w in zip(columns, widths)))
    for r in rows:
        out('  '.join(str(v).ljust(w) for v, w in zip(r, widths)))


# ================================================================ run
def cmd_machine(suite, args):
    m = resources.measure_machine(suite.data_dir)
    r = m.get('renderer') or {}
    out(f'CPU:               {m.get("cpu_model") or "unknown"}')
    out(f'Logical CPUs:      {m.get("logical_cpus") or "unknown"}')
    out(f'WSL memory:        {fmt.size(m.get("memory_total"))} total, '
        f'{fmt.size(m.get("memory_available"))} available')
    out(f'Disk free:         {fmt.size(m.get("disk_free"))} in {suite.data_dir}')
    out(f'Renderer:          {r.get("name", "unknown")} ({r.get("kind", "unknown")})')
    g = m.get('gpu_memory_mb')
    out(f'GPU memory:        {f"{g["total"]} MB ({g["used"]} MB used)" if g else "unknown"}')
    out(f'Confinement:       {"systemd user scopes" if m.get("systemd_scope") else "taskset"}'
        f'{" + cpuset" if resources.cpuset_delegated() else ""}')
    for w in resources.clock_warnings():
        out(f'warning: {w}')
    return 0


def describe_prep(prep, config):
    m, ex, est = prep['machine'], prep['execution'], prep['estimate']
    r = m.get('renderer') or {}
    out(f'Configuration "{config.name}": {config.total_runs()} runs in {len(config.sets())} '
        f'maze set(s).')
    out(f'Machine: {m.get("logical_cpus")} logical CPUs, {fmt.size(m.get("memory_available"))} '
        f'memory available, {fmt.size(m.get("disk_free"))} disk free, renderer '
        f'{r.get("name", "unknown")}.')
    out(f'Allocation: {ex["cpus"]} CPUs, {fmt.size(ex["memory_bytes"])} memory, '
        f'{fmt.size(ex["disk_bytes"])} disk.')
    if est:
        out(f'Estimate: about {fmt.long_duration(est["duration_s"])} at up to '
            f'{est["concurrency"]} runs at once, {fmt.size(est["disk_bytes"])} of disk.')
    else:
        out('Estimate: after the first run (no calibration for this machine yet; the first '
            'run runs alone and calibrates).')
    for w in prep['warnings']:
        out(f'warning: {w}')
    for e in prep['errors']:
        out(f'error: {e}')


def cmd_run(suite, args):
    config = suite.configs.get(args.config)
    prep = suite.prepare_run(config, args.cpus, args.memory, args.disk, args.max_concurrency,
                             args.recalibrate)
    describe_prep(prep, config)
    if prep['errors'] and not args.force:
        err('not started; fix the errors above (or use --force for estimate problems)')
        return 1
    if not args.yes and sys.stdin.isatty():
        if input('Start? [Y/n] ').strip().lower() not in ('', 'y', 'yes'):
            return 1
    exp, pid = suite.start(config, prep, name=args.name, foreground=args.foreground,
                           clock_jumps=args.clock_jumps)
    out(f'Experiment {exp.id} created.')
    if args.foreground:
        from .manager import Manager
        return Manager(exp, suite.data_dir).run()
    out(f'The manager runs in the background (PID {pid}); closing this terminal does not '
        f'stop it.')
    out(f'  Follow it:  maze_experiments status {exp.id}')
    out(f'  Results:    maze_experiments report {exp.id} --format md')
    return 0


# ================================================================ status
def live_of(exp):
    try:
        return fileio.read_json(exp.p('live.json'))
    except (OSError, ValueError):
        return None


def success_text(summary):
    s = summary or {}
    if not s.get('processed'):
        return '—'
    return f'{s["successes"]}/{s["processed"]} ({fmt.percent(s.get("success_rate"))})'


def cmd_status(suite, args):
    if args.experiment:
        exp = suite.experiments.get(args.experiment)
        return status_detail(exp)
    exps = suite.experiments.list()
    if not args.all:
        exps = [e for e in exps if e.state in exp_mod.ACTIVE_STATES or e.state ==
                'interrupted'] + [e for e in exps if e.state not in exp_mod.ACTIVE_STATES and
                                  e.state != 'interrupted'][:10]
    rows = []
    for e in exps:
        s = e.meta.get('summary') or {}
        alive = manager_control.manager_alive(e)
        rows.append([e.id, e.name[:30], e.state + ('' if alive or e.state not in
                                                   exp_mod.ACTIVE_STATES else ' (no manager)'),
                     f'{s.get("robot_results", 0)}/{e.meta.get("planned_runs", "?")}',
                     success_text(s)])
    if not rows:
        out('No experiments yet. Create a configuration (maze_experiments config example > '
            'my_config.yaml; maze_experiments config create my_config.yaml) and run it.')
        return 0
    table(['Experiment', 'Name', 'State', 'Runs done', 'Mission success'], rows)
    return 0


def status_detail(exp):
    s = exp.meta.get('summary') or {}
    alive = manager_control.manager_alive(exp)
    out(f'{exp.id}  "{exp.name}"')
    out(f'State: {exp.state}{" (manager running, PID " + str(manager_control.manager_pid(exp)) + ")" if alive else ""}')
    out(f'Configuration: {exp.meta.get("config_name")} ({exp.config_id})')
    out(f'Runs: {s.get("robot_results", 0)} of {exp.meta.get("planned_runs")} done, '
        f'{s.get("excluded", 0)} excluded attempt(s); mission success {success_text(s)}')
    if s.get('outcomes'):
        out('Outcomes: ' + ', '.join(f'{k} {v}' for k, v in sorted(s['outcomes'].items())))
    live = live_of(exp) if alive else None
    if live:
        out(f'Concurrency: {len([a for a in live["active"] if a["phase"] != "stopping"])} '
            f'running, target {live["target"]}, limit {live["cap"]}'
            f'{" (held back: low RTF)" if live.get("held_back") else ""}; '
            f'{live["queue"]} queued, {live["processing"]} to process'
            f'{"" if live.get("calibrated") else "; calibrating"}')
        rows = []
        for a in live['active']:
            rows.append([a['run'], a['phase'], fmt.duration(a.get('sim')),
                         '—' if a.get('rtf') is None else f'{a["rtf"]:.2f}',
                         '—' if a.get('area') is None else f'{a["area"]:.1f} m²',
                         a.get('explore') or '—', a.get('outcome') or ''])
        if rows:
            table(['Run', 'Phase', 'Sim', 'RTF', 'Known area', 'Explore', 'Outcome'], rows)
        for m in live.get('messages') or []:
            out(f'[{m["level"]}] {m["text"]}')
    recent = fileio.tail_jsonl(exp.manager_log, 16384)
    shown = [r for r in recent if r.get('event') in ('outcome', 'retried', 'resources', 'error',
                                                     'host_event', 'calibration', 'message')]
    if shown:
        out('Recent events:')
        for r in shown[-10:]:
            detail = {k: v for k, v in r.items() if k not in ('time', 'event')}
            out(f'  {r["time"][11:19]} {r["event"]}: {json.dumps(detail)[:150]}')
    return 0


def cmd_pause(suite, args):
    exp = suite.experiments.get(args.experiment)
    suite.pause(exp)
    out('Pause requested: no new runs start; active runs finish.')


def cmd_resume(suite, args):
    exp = suite.experiments.get(args.experiment)
    out(suite.resume(exp))


def cmd_cancel(suite, args):
    exp = suite.experiments.get(args.experiment)
    out(suite.cancel(exp))


# ================================================================ results
def cmd_report(suite, args):
    from .metrics.results import all_json, build_results, load_results, markdown
    from .metrics.findings import CATEGORIES
    exp = suite.experiments.get(args.experiment)
    data = load_results(exp)
    if data is None:
        data = build_results(exp)
    if args.format == 'md':
        raw = None
        if args.raw_run:
            raw = {'run': args.raw_run, 'topics': (args.raw_topics or '/explore/status').split(','),
                   'max_messages': args.raw_max}
        text = markdown(exp, data, raw=raw)
    elif args.format == 'csv':
        with open(os.path.join(exp.results_dir, 'runs.csv')) as f:
            text = f.read()
    elif args.format == 'json':
        text = all_json(exp, data)
    else:
        lines = [f'{exp.name} ({exp.id}), state {exp.state}']
        for w in data.get('warnings') or []:
            lines.append(f'WARNING: {w}')
        for cat in CATEGORIES:
            lines.append(f'\n{cat}')
            for s in data['findings']['experiment'][cat]['findings']:
                lines.append(f'  - {s}')
        for k, s in enumerate(data['sets']):
            lines.append(f'\nMaze set {s["name"]}')
            for sent in data['findings']['sets'][k]['Overview']['findings']:
                lines.append(f'  - {sent}')
        text = '\n'.join(lines) + '\n'
    if args.output:
        with open(args.output, 'w') as f:
            f.write(text)
        out(f'Wrote {args.output}')
    else:
        sys.stdout.write(text)
    return 0


def cmd_reprocess(suite, args):
    exp = suite.experiments.get(args.experiment)

    def progress(i, n):
        print(f'\r{i}/{n} runs', end='', file=sys.stderr, flush=True)
    data, errors = suite.reprocess(exp, args.results_only, progress)
    print(file=sys.stderr)
    for e in errors:
        err(e)
    out(f'Results rebuilt with pipeline version {data["pipeline_version"]}: '
        f'{exp.results_dir}')
    return 1 if errors else 0


def cmd_import(suite, args):
    kind, obj, extra = suite.import_folder(args.folder)
    if kind == 'experiment':
        out(f'Imported experiment {obj.id}' + (' and recreated its configuration.' if extra
                                               else '.'))
    else:
        out(f'Imported configuration {obj.id}.' if extra else
            f'Configuration {obj.id} was already there.')


def cmd_export(suite, args):
    exp = suite.experiments.get(args.experiment)
    target = os.path.abspath(os.path.expanduser(args.output or '.'))
    if args.zip:
        out(f'Wrote {suite.zip_experiment(exp, target)}')
        return 0
    for f in suite.export(exp, target):
        out(f'Wrote {f}')


def cmd_delete(suite, args):
    exp = suite.experiments.get(args.experiment)
    if not args.yes:
        if not sys.stdin.isatty() or input(f'Delete experiment {exp.id} and all its data? '
                                           f'[y/N] ').strip().lower() not in ('y', 'yes'):
            return 1
    suite.delete_experiment(exp)
    out(f'Deleted {exp.id}.')


def cmd_replay(suite, args):
    exp = suite.experiments.get(args.experiment)
    run_dir = exp.run_dir(args.run)
    if not os.path.isdir(os.path.join(run_dir, 'bag')):
        raise ValueError(f'Run {args.run} has no recording.')
    return subprocess.call(['ros2', 'run', 'maze_monitor', 'maze_monitor', '--no-ros',
                            '--replay', run_dir])
