"""Configurations (FR1 to FR4, FR8): saved, immutable descriptions of an experiment.

A configuration lives in <data dir>/configurations/<id>/configuration.yaml,
with copies of any stack parameter files it uses in params/. It is never
edited: "Create modified version" makes a new configuration (2.3.3).
"""

import copy
import datetime
import hashlib
import itertools
import os
import shutil

from . import fileio, ids, paths, profiles
from .mazegen.params import MazeParams, validate as validate_maze

CONFIG_FILE = 'configuration.yaml'
PARAMS_DIR = 'params'

DEFAULT_LIMITS = {
    'sim_timeout_s': 900.0,       # BR5, sim time
    'startup_timeout_s': 120.0,   # BR3, wall time
    'stall_window_s': 120.0,      # BR7, sim time
    'stall_min_area_m2': 0.05,    # BR7
    'stall_min_move_m': 0.25,     # BR7
    'retries': 2,                 # BR10
    'min_rtf': 0.8,               # CR5
}
RECORDING_PROFILES = ('full', 'metrics_only')
RETENTION = ('all', 'failures', 'none')
DEFAULT_RECORDING = {'profile': 'full', 'retention': 'all'}
DEFAULT_ANALYSIS = {'exit_match_tolerance_m': None}   # None: one cell pitch (ER1)
STACK_PARAM_KEYS = profiles.STACK_PARAM_KEYS


class ConfigError(ValueError):
    def __init__(self, problems):
        self.problems = list(problems)
        super().__init__('\n'.join(self.problems))


# ---------------------------------------------------------------- model
def default_set(name='set 1'):
    return {'name': name, 'maze': MazeParams().to_dict(), 'mazes': 10, 'repeats': 1, 'seed': 1,
            'stack_params': {}}


def normalize(raw):
    """Fill defaults; keep unknown keys out. Input may be hand-written YAML."""
    data = copy.deepcopy(raw or {})
    out = {
        'schema_version': fileio.SCHEMA_VERSION,
        'name': str(data.get('name') or 'unnamed'),
        'description': str(data.get('description') or ''),
        'robot_profile': str(data.get('robot_profile') or 'burger'),
        'run_limits': {**DEFAULT_LIMITS, **(data.get('run_limits') or {})},
        'recording': {**DEFAULT_RECORDING, **(data.get('recording') or {})},
        'analysis': {**DEFAULT_ANALYSIS, **(data.get('analysis') or {})},
        'maze_sets': [],
    }
    for key in ('id', 'created', 'derived_from'):
        if data.get(key):
            out[key] = data[key]
    for i, s in enumerate(data.get('maze_sets') or []):
        maze = {**MazeParams().to_dict(), **(s.get('maze') or {})}
        out['maze_sets'].append({
            'name': str(s.get('name') or f'set {i + 1}'),
            'maze': maze,
            'mazes': s.get('mazes', 10),
            'repeats': s.get('repeats', 1),
            'seed': s.get('seed', i + 1),
            'stack_params': dict(s.get('stack_params') or {}),
        })
    return out


def _fmt_value(v):
    if isinstance(v, float):
        return f'{v:g}'
    return str(v)


def expand_sets(config):
    """Sweeps (FR3): a maze parameter given as a list becomes one set per value.

    Lists in several parameters expand into every combination. start_cell is
    a list by nature; it sweeps only when given as a list of lists.
    """
    out = []
    for s in config['maze_sets']:
        maze = s['maze']
        sweep_keys = [k for k, v in maze.items()
                      if isinstance(v, list) and (k != 'start_cell' or
                                                  (v and isinstance(v[0], list)))]
        if not sweep_keys:
            out.append({**s, 'sweep': {}})
            continue
        for combo in itertools.product(*(maze[k] for k in sweep_keys)):
            values = dict(zip(sweep_keys, combo))
            suffix = ', '.join(f'{k}={_fmt_value(v)}' for k, v in values.items())
            out.append({**s, 'name': f'{s["name"]} ({suffix})', 'maze': {**maze, **values},
                        'sweep': values})
    return out


def total_runs(config):
    return sum(int(s['mazes']) * int(s['repeats']) for s in expand_sets(config))


def maze_params(set_):
    return MazeParams.from_dict(set_['maze'])


# ---------------------------------------------------------------- checks
def check(config, data_dir=None, config_dir=None, workspace=True):
    """Every problem at once (FR4). Returns (errors, warnings)."""
    errors, warnings = [], []
    try:
        profile = profiles.load(config['robot_profile'], data_dir)
    except Exception as exc:
        errors.append(f'Robot profile: {exc}')
        profile = None
    if profile is not None:
        try:
            for key, path in profile.stack_params().items():
                if not os.path.isfile(path):
                    errors.append(f'Robot profile {profile.name}: {key} file {path} not found')
        except FileNotFoundError as exc:
            errors.append(str(exc))
    lim = config['run_limits']
    for key in ('sim_timeout_s', 'startup_timeout_s', 'stall_window_s', 'min_rtf'):
        try:
            if float(lim[key]) <= 0:
                errors.append(f'run_limits.{key} must be positive')
        except (TypeError, ValueError):
            errors.append(f'run_limits.{key} must be a number')
    try:
        if int(lim['retries']) < 0:
            errors.append('run_limits.retries must be 0 or more')
    except (TypeError, ValueError):
        errors.append('run_limits.retries must be a whole number')
    if config['recording']['profile'] not in RECORDING_PROFILES:
        errors.append(f'recording.profile must be one of {", ".join(RECORDING_PROFILES)}')
    if config['recording']['retention'] not in RETENTION:
        errors.append(f'recording.retention must be one of {", ".join(RETENTION)}')
    sets = config['maze_sets']
    if not sets:
        errors.append('The configuration has no maze sets.')
    names = set()
    for s in expand_sets(config):
        label = f'Maze set "{s["name"]}"'
        if s['name'] in names:
            errors.append(f'{label}: two sets have this name')
        names.add(s['name'])
        try:
            params = maze_params(s)
        except (TypeError, ValueError) as exc:
            errors.append(f'{label}: {exc}')
            continue
        e, w = validate_maze(params, profile)
        errors += [f'{label}: {x}' for x in e]
        warnings += [f'{label}: {x}' for x in w]
        for key in ('mazes', 'repeats'):
            try:
                if int(s[key]) < 1:
                    errors.append(f'{label}: {key} must be at least 1')
            except (TypeError, ValueError):
                errors.append(f'{label}: {key} must be a whole number')
        try:
            int(s['seed'])
        except (TypeError, ValueError):
            errors.append(f'{label}: seed must be a whole number')
        for key, rel in (s.get('stack_params') or {}).items():
            if key not in STACK_PARAM_KEYS:
                errors.append(f'{label}: unknown stack parameter file "{key}" '
                              f'(use {", ".join(STACK_PARAM_KEYS)})')
                continue
            path = rel if os.path.isabs(rel) or config_dir is None else \
                os.path.join(config_dir, rel)
            if not os.path.isfile(os.path.expanduser(path)):
                errors.append(f'{label}: {key} file {path} not found')
    if workspace:
        if paths.package_share('maze_sim') is None:
            errors.append('maze_sim is not installed: build the workspace with ./setup.sh and '
                          'open a new terminal.')
        if paths.package_share('maze_experiments') is None:
            errors.append('maze_experiments is not built into the workspace: run ./setup.sh '
                          '(or colcon build --packages-up-to maze_experiments) and open a new '
                          'terminal.')
    if data_dir is not None:
        try:
            paths.check_data_dir(data_dir)
            paths.ensure_writable(data_dir)
        except (OSError, ValueError) as exc:
            errors.append(f'Data directory: {exc}')
    return errors, warnings


# ---------------------------------------------------------------- store
class Configuration:
    def __init__(self, path, data):
        self.path = path            # folder
        self.data = data

    id = property(lambda self: self.data.get('id') or os.path.basename(self.path))
    name = property(lambda self: self.data.get('name', ''))
    created = property(lambda self: self.data.get('created', ''))

    @property
    def file(self):
        return os.path.join(self.path, CONFIG_FILE)

    def sets(self):
        return expand_sets(self.data)

    def total_runs(self):
        return total_runs(self.data)

    def stack_param_files(self, set_):
        """Absolute paths of a set's own parameter files (FR8)."""
        return {k: os.path.join(self.path, v) for k, v in (set_.get('stack_params') or {}).items()}


class ConfigStore:
    def __init__(self, data_dir):
        self.root = os.path.join(data_dir, 'configurations')
        self.data_dir = data_dir

    def list(self):
        out = []
        if not os.path.isdir(self.root):
            return out
        for entry in os.listdir(self.root):
            f = os.path.join(self.root, entry, CONFIG_FILE)
            if os.path.isfile(f):
                try:
                    out.append(Configuration(os.path.dirname(f), fileio.read_yaml(f)))
                except Exception:
                    continue
        out.sort(key=lambda c: c.created, reverse=True)
        return out

    def get(self, ref):
        """By id, unique id prefix, folder path, or exact name."""
        if ref and os.path.isfile(os.path.join(os.path.expanduser(ref), CONFIG_FILE)):
            path = os.path.abspath(os.path.expanduser(ref))
            return Configuration(path, fileio.read_yaml(os.path.join(path, CONFIG_FILE)))
        direct = os.path.join(self.root, ref or '', CONFIG_FILE)
        if ref and os.path.isfile(direct):
            return Configuration(os.path.dirname(direct), fileio.read_yaml(direct))
        configs = self.list()
        by_name = [c for c in configs if c.name == ref]
        if len(by_name) == 1:
            return by_name[0]
        by_prefix = [c for c in configs if c.id.startswith(ref or '\0')]
        if len(by_prefix) == 1:
            return by_prefix[0]
        if len(by_name) > 1 or len(by_prefix) > 1:
            raise KeyError(f'"{ref}" matches several configurations; use the id')
        raise KeyError(f'No configuration "{ref}"')

    def create(self, raw, source_dir=None, derived_from=None, check_workspace=True):
        """Save a new configuration. Parameter files are copied into it (FR8)."""
        data = normalize(raw)
        errors, warnings = check(data, self.data_dir, source_dir, workspace=check_workspace)
        if errors:
            raise ConfigError(errors)
        cid = ids.new_id(data['name'])
        folder = os.path.join(self.root, cid)
        os.makedirs(folder)
        try:
            for i, s in enumerate(data['maze_sets']):
                copied = {}
                for key, rel in (s.get('stack_params') or {}).items():
                    src = rel if os.path.isabs(rel) or source_dir is None else \
                        os.path.join(source_dir, rel)
                    src = os.path.expanduser(src)
                    dest_rel = os.path.join(PARAMS_DIR, f'set{i:02d}_{key}.yaml')
                    os.makedirs(os.path.join(folder, PARAMS_DIR), exist_ok=True)
                    shutil.copyfile(src, os.path.join(folder, dest_rel))
                    copied[key] = dest_rel
                s['stack_params'] = copied
            data['id'] = cid
            data['created'] = datetime.datetime.now().isoformat(timespec='seconds')
            if derived_from:
                data['derived_from'] = derived_from
            fileio.write_yaml(os.path.join(folder, CONFIG_FILE), data,
                              header='# maze_experiments configuration. Never edited: make a '
                                     'modified copy instead.\n')
        except BaseException:
            shutil.rmtree(folder, ignore_errors=True)
            raise
        return Configuration(folder, data), warnings

    def copy_for_edit(self, config):
        """A dict for the editor: every field of `config`, ready to save under a new name."""
        data = copy.deepcopy(config.data)
        for key in ('id', 'created', 'schema_version'):
            data.pop(key, None)
        data['name'] = f'{config.name} (modified)'
        for s in data['maze_sets']:
            s['stack_params'] = {k: os.path.join(config.path, v)
                                 for k, v in (s.get('stack_params') or {}).items()}
        return data

    def delete(self, config, experiments):
        used = [e for e in experiments if e.config_id == config.id]
        if used:
            raise ConfigError([f'The configuration "{config.name}" has {len(used)} '
                               f'experiment(s). Delete them first; a configuration can only '
                               f'be deleted when no experiment uses it.'])
        if os.path.dirname(os.path.abspath(config.path)) != os.path.abspath(self.root):
            raise ConfigError([f'Refusing to delete a folder outside {self.root}'])
        shutil.rmtree(config.path)

    def import_folder(self, folder):
        """Import a configuration folder copied from elsewhere (2.3.4)."""
        src = os.path.abspath(os.path.expanduser(folder))
        data = fileio.read_yaml(os.path.join(src, CONFIG_FILE))
        cid = data.get('id') or os.path.basename(src)
        dest = os.path.join(self.root, cid)
        if os.path.exists(dest):
            return Configuration(dest, fileio.read_yaml(os.path.join(dest, CONFIG_FILE))), False
        os.makedirs(self.root, exist_ok=True)
        shutil.copytree(src, dest)
        return Configuration(dest, data), True


def file_hash(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(1 << 16), b''):
            h.update(block)
    return h.hexdigest()
