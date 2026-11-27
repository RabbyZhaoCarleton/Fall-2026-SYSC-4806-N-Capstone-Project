"""Experiments: one execution of a configuration (1.4, DR3, DR4, FR5).

<data dir>/experiments/<id>/
    experiment.json        state, config id, execution settings, provenance (DR6)
    configuration.yaml     copy of the configuration used (with params/)
    manager.jsonl          the manager's log (BR11)
    resources.jsonl        resource time series (CR8)
    control.jsonl          pause / resume / cancel requests to the manager
    mazes/<set>/<maze>/    world.sdf, layout.txt, maze.json, preview.png
    runs/<run id>/         run.json, bag/, launch.log, status.jsonl, metrics.json
    results/               processed results (Module E); can always be regenerated

Everything inside uses relative paths, so a copied folder still works (DR4).
"""

import datetime
import os
import shutil

from . import configuration, fileio, ids

EXPERIMENT_FILE = 'experiment.json'
STATES = ('created', 'generating', 'calibrating', 'running', 'paused', 'processing',
          'completed', 'cancelled', 'interrupted')
ACTIVE_STATES = ('generating', 'calibrating', 'running', 'paused', 'processing')
FINAL_STATES = ('completed', 'cancelled')

# BR9
ROBOT_OUTCOMES = ('complete', 'incomplete', 'return_failed', 'timeout', 'stalled', 'stack_crash')
EXCLUDED_OUTCOMES = ('startup_failed', 'sim_frozen', 'infra_failure', 'interrupted')
OUTCOMES = ROBOT_OUTCOMES + EXCLUDED_OUTCOMES


def is_robot_result(outcome):
    return outcome in ROBOT_OUTCOMES


def set_folder(index):
    return f'set-{index:02d}'


class PlannedRun:
    """One planned run: maze set s, maze m, repeat r. Attempts get their own folders."""

    __slots__ = ('order', 'set_index', 'maze_index', 'repeat')

    def __init__(self, order, set_index, maze_index, repeat):
        self.order = order
        self.set_index = set_index
        self.maze_index = maze_index
        self.repeat = repeat

    @property
    def key(self):
        return (self.set_index, self.maze_index, self.repeat)

    def run_id(self, attempt):
        return ids.run_id(self.set_index, self.maze_index, self.repeat, attempt)

    def __repr__(self):
        return f'PlannedRun{self.key}'


def plan_runs(sets):
    """Run order (BR14, FNR1): round-robin over sets, then mazes, then repeats.

    A stopped or partial experiment covers every set evenly, and slow drifts in
    machine load don't line up with one set. The order is deterministic.
    """
    out = []
    max_r = max((int(s['repeats']) for s in sets), default=0)
    max_m = max((int(s['mazes']) for s in sets), default=0)
    for r in range(max_r):
        for m in range(max_m):
            for k, s in enumerate(sets):
                if m < int(s['mazes']) and r < int(s['repeats']):
                    out.append(PlannedRun(len(out), k, m, r))
    return out


class Experiment:
    def __init__(self, path, meta=None):
        self.path = os.path.abspath(path)
        self.meta = meta if meta is not None else fileio.read_json(self.file)
        self._config = None

    # ---------------------------------------------------------------- basics
    file = property(lambda self: os.path.join(self.path, EXPERIMENT_FILE))
    id = property(lambda self: self.meta.get('id') or os.path.basename(self.path))
    name = property(lambda self: self.meta.get('name') or self.id)
    state = property(lambda self: self.meta.get('state', 'created'))
    config_id = property(lambda self: self.meta.get('config_id'))
    created = property(lambda self: self.meta.get('created', ''))
    execution = property(lambda self: self.meta.get('execution') or {})

    def reload(self):
        self.meta = fileio.read_json(self.file)
        return self

    def save(self):
        fileio.write_json(self.file, self.meta)

    def update(self, **fields):
        self.meta.update(fields)
        self.save()

    def set_state(self, state, reason=''):
        assert state in STATES, state
        now = datetime.datetime.now().isoformat(timespec='seconds')
        hist = self.meta.setdefault('state_history', [])
        hist.append({'state': state, 'time': now, **({'reason': reason} if reason else {})})
        self.meta['state'] = state
        if state in FINAL_STATES:
            self.meta['ended'] = now
        self.save()

    # ---------------------------------------------------------------- files
    def p(self, *parts):
        return os.path.join(self.path, *parts)

    @property
    def config(self):
        if self._config is None:
            data = fileio.read_yaml(self.p(configuration.CONFIG_FILE))
            self._config = configuration.Configuration(self.path, data)
        return self._config

    def sets(self):
        return self.config.sets()

    def plan(self):
        return plan_runs(self.sets())

    def maze_dir(self, set_index, maze_index):
        from .mazegen import maze_name
        return self.p('mazes', set_folder(set_index), maze_name(maze_index))

    def run_dir(self, run_id):
        return self.p('runs', run_id)

    def run_ids(self):
        d = self.p('runs')
        return sorted(os.listdir(d)) if os.path.isdir(d) else []

    manager_log = property(lambda self: self.p('manager.jsonl'))
    resources_log = property(lambda self: self.p('resources.jsonl'))
    control_file = property(lambda self: self.p('control.jsonl'))
    results_dir = property(lambda self: self.p('results'))

    def log(self, event, **fields):
        rec = {'time': datetime.datetime.now().isoformat(timespec='milliseconds'),
               'event': event, **fields}
        fileio.append_jsonl(self.manager_log, rec, kind='manager')
        return rec

    def request(self, command):
        """Ask the manager to pause, resume or cancel (it reads control.jsonl every tick)."""
        fileio.append_jsonl(self.control_file, {
            'time': datetime.datetime.now().isoformat(timespec='seconds'), 'command': command},
            kind='control')

    # ---------------------------------------------------------------- runs
    def attempts(self):
        """{plan key: [run.json 'experiment' records of each attempt, in attempt order]}."""
        out = {}
        for rid in self.run_ids():
            parsed = ids.parse_run_id(rid)
            if parsed is None:
                continue
            s, m, r, a = parsed
            rec = read_run_record(self.run_dir(rid))
            rec.setdefault('attempt', a)
            rec.setdefault('run_id', rid)
            out.setdefault((s, m, r), []).append(rec)
        for v in out.values():
            v.sort(key=lambda x: x['attempt'])
        return out


def read_run_record(run_dir):
    """The suite's part of run.json ('experiment'), with finished flags (DR8)."""
    import json
    rec = {}
    try:
        with open(os.path.join(run_dir, 'run.json')) as f:
            meta = json.load(f)
        rec = dict(meta.get('experiment') or {})
    except (OSError, ValueError):
        pass
    rec['has_metrics'] = os.path.isfile(os.path.join(run_dir, 'metrics.json'))
    bag = os.path.join(run_dir, 'bag')
    rec['recording_closed'] = os.path.isfile(os.path.join(bag, 'metadata.yaml')) or \
        rec.get('recording') == 'deleted'
    rec['finished'] = bool(rec.get('outcome')) and rec['has_metrics'] and rec['recording_closed']
    return rec


class ExperimentStore:
    def __init__(self, data_dir):
        self.data_dir = data_dir
        self.root = os.path.join(data_dir, 'experiments')

    def list(self):
        """All experiments, newest first, from experiment.json only (DNR2)."""
        out = []
        if not os.path.isdir(self.root):
            return out
        for entry in os.listdir(self.root):
            f = os.path.join(self.root, entry, EXPERIMENT_FILE)
            try:
                out.append(Experiment(os.path.dirname(f), fileio.read_json(f)))
            except Exception:
                continue
        out.sort(key=lambda e: e.created, reverse=True)
        return out

    def get(self, ref):
        if ref and os.path.isfile(os.path.join(os.path.expanduser(ref), EXPERIMENT_FILE)):
            return Experiment(os.path.expanduser(ref))
        direct = os.path.join(self.root, ref or '')
        if ref and os.path.isfile(os.path.join(direct, EXPERIMENT_FILE)):
            return Experiment(direct)
        exps = self.list()
        if ref == 'latest' and exps:
            return exps[0]
        matches = [e for e in exps if e.id.startswith(ref or '\0') or e.name == ref]
        if len(matches) == 1:
            return matches[0]
        if len(matches) > 1:
            raise KeyError(f'"{ref}" matches several experiments; use the id')
        raise KeyError(f'No experiment "{ref}"')

    def create(self, config, execution, provenance=None, name=None):
        eid = ids.new_id(name or config.name)
        path = os.path.join(self.root, eid)
        os.makedirs(path)
        # A full copy of the configuration (with its params/), so the folder is
        # self-contained (DR4) and can recreate the configuration on import.
        shutil.copyfile(config.file, os.path.join(path, configuration.CONFIG_FILE))
        params = os.path.join(config.path, configuration.PARAMS_DIR)
        if os.path.isdir(params):
            shutil.copytree(params, os.path.join(path, configuration.PARAMS_DIR))
        plan = plan_runs(config.sets())
        meta = {
            'schema_version': fileio.SCHEMA_VERSION,
            'id': eid,
            'name': name or config.name,
            'config_id': config.id,
            'config_name': config.name,
            'created': datetime.datetime.now().isoformat(timespec='seconds'),
            'state': 'created',
            'state_history': [],
            'execution': execution,
            'provenance': provenance or {},
            'planned_runs': len(plan),
            'summary': {'planned': len(plan), 'finished': 0, 'robot_results': 0,
                        'successes': None, 'excluded': 0},
        }
        exp = Experiment(path, meta)
        exp.set_state('created')
        return exp

    def delete(self, exp):
        if os.path.dirname(os.path.abspath(exp.path)) != os.path.abspath(self.root):
            raise ValueError(f'Refusing to delete a folder outside {self.root}')
        from . import manager_control
        if manager_control.manager_alive(exp):
            raise ValueError('The experiment is running. Cancel it first.')
        shutil.rmtree(exp.path)

    def import_folder(self, folder, config_store):
        """Import an experiment folder (2.3.4, DR4). Recreates its configuration if missing."""
        src = os.path.abspath(os.path.expanduser(folder))
        exp_src = Experiment(src)
        dest = os.path.join(self.root, exp_src.id)
        if os.path.abspath(src) != os.path.abspath(dest):
            if os.path.exists(dest):
                raise ValueError(f'An experiment with id {exp_src.id} already exists')
            os.makedirs(self.root, exist_ok=True)
            shutil.copytree(src, dest, symlinks=True)
        exp = Experiment(dest)
        created_config = False
        try:
            config_store.get(exp.config_id)
        except KeyError:
            cfg_dir = os.path.join(config_store.root, exp.config_id)
            os.makedirs(cfg_dir, exist_ok=True)
            shutil.copyfile(exp.p(configuration.CONFIG_FILE),
                            os.path.join(cfg_dir, configuration.CONFIG_FILE))
            if os.path.isdir(exp.p(configuration.PARAMS_DIR)):
                shutil.copytree(exp.p(configuration.PARAMS_DIR),
                                os.path.join(cfg_dir, configuration.PARAMS_DIR))
            created_config = True
        return exp, created_config
