"""The settings of one simulation run, the launch command they make, and saved presets.

No Qt and no ROS in here, so it can be tested on its own.
"""

import os
import re
import shlex
from dataclasses import asdict, dataclass, field, fields

import yaml

from .param_files import PARAM_FILES, resolve

LAUNCH_PACKAGE = 'maze_sim'
LAUNCH_FILE = 'maze.launch.py'

# What the robot does on its own. The values match the run labels that
# maze.launch.py gives runs it starts itself.
AUTONOMY = {
    'manual': 'Drive it yourself (no Nav2)',
    'nav': 'Nav2: send goals in RViz',
    'explore': 'Explore the maze autonomously',
}

BOOL_ARGS = ('gui', 'rviz', 'slam', 'ekf', 'use_sim_time', 'shutdown_when_done')
# The monitor manages runs itself, so the launch must not open a second monitor.
FIXED_ARGS = {'monitor': 'false'}
_ARG = re.compile(r'^([A-Za-z_][A-Za-z0-9_]*):=(.*)$', re.S)


def _bool(value, default=False):
    if isinstance(value, bool):
        return value
    if value is None:
        return default
    return str(value).strip().lower() in ('true', '1', 'yes', 'on')


def _num(value):
    return f'{float(value):g}'


def parse_extra_args(text):
    """'a:=1 b:="x y"' -> [('a', '1'), ('b', 'x y')]. Raises ValueError."""
    out = []
    try:
        parts = shlex.split(text or '')
    except ValueError as exc:
        raise ValueError(f'Extra launch arguments: {exc}') from None
    for part in parts:
        m = _ARG.match(part)
        if not m:
            raise ValueError(f'Extra launch arguments: "{part}" is not of the form name:=value')
        if m.group(1) in FIXED_ARGS:
            raise ValueError(f'Extra launch arguments: {m.group(1)} is set by the monitor')
        out.append((m.group(1), m.group(2)))
    return out


@dataclass
class RunSettings:
    autonomy: str = 'explore'
    shutdown_when_done: bool = False
    gui: bool = False
    rviz: bool = True
    slam: bool = True
    ekf: bool = True
    use_sim_time: bool = True
    world: str = ''                 # '' = the launch file's default maze
    x_pose: float = 0.25
    y_pose: float = -0.25
    params: dict = field(default_factory=dict)      # key -> chosen file ('' = default)
    overrides: dict = field(default_factory=dict)   # key -> {path tuple: value}
    extra_args: str = ''
    record: bool = True
    label: str = ''                 # '' = the autonomy name
    notes: str = ''

    # ------------------------------------------------------------ derived
    @property
    def nav(self):
        return self.autonomy == 'nav'

    @property
    def explore(self):
        return self.autonomy == 'explore'

    @property
    def uses_nav2(self):
        return self.autonomy in ('nav', 'explore')

    def run_label(self):
        return self.label.strip() or self.autonomy

    def override_count(self):
        return sum(len(v) for v in self.overrides.values())

    def launch_arguments(self, param_paths=None):
        """[(name, value)] in a stable order. param_paths: {key: file} from prepare_run_files."""
        param_paths = param_paths if param_paths is not None else {
            k: v for k, v in self.params.items() if v}
        args = {
            'explore': 'true' if self.explore else 'false',
            'nav': 'true' if self.nav else 'false',
        }
        if self.explore:
            args['shutdown_when_done'] = 'true' if self.shutdown_when_done else 'false'
        for name in ('gui', 'rviz', 'slam', 'ekf', 'use_sim_time'):
            args[name] = 'true' if getattr(self, name) else 'false'
        if self.world.strip():
            args['world'] = os.path.abspath(os.path.expanduser(self.world.strip()))
        args['x_pose'], args['y_pose'] = _num(self.x_pose), _num(self.y_pose)
        for key, (arg, _, _) in PARAM_FILES.items():
            if param_paths.get(key):
                args[arg] = param_paths[key]
        for name, value in parse_extra_args(self.extra_args):
            args[name] = value  # extra arguments win over the form
        args.update(FIXED_ARGS)
        return list(args.items())

    def command(self, param_paths=None):
        return ['ros2', 'launch', LAUNCH_PACKAGE, LAUNCH_FILE] + [
            f'{k}:={v}' for k, v in self.launch_arguments(param_paths)]

    def shell_command(self, param_paths=None):
        return shlex.join(self.command(param_paths))

    def launch_args_record(self, param_paths=None):
        """Launch arguments as maze.launch.py saves them in run.json, with files resolved."""
        args = dict(self.launch_arguments(param_paths))
        for key, (arg, _, _) in PARAM_FILES.items():
            args.setdefault(arg, (param_paths or {}).get(key) or resolve(key, self.params.get(key, '')))
        args.pop('monitor', None)
        return args

    def problems(self):
        """What keeps these settings from starting, as sentences. Empty when they are fine."""
        out = []
        if self.autonomy not in AUTONOMY:
            out.append(f'Unknown autonomy mode "{self.autonomy}".')
        world = self.world.strip()
        if world and not os.path.isfile(os.path.expanduser(world)):
            out.append(f'World file not found: {world}')
        for key, path in self.params.items():
            if path and not os.path.isfile(os.path.expanduser(path)):
                out.append(f'{PARAM_FILES[key][2]} parameter file not found: {path}')
        try:
            parse_extra_args(self.extra_args)
        except ValueError as exc:
            out.append(str(exc))
        return out

    # ------------------------------------------------------------ storage
    def to_dict(self):
        d = asdict(self)
        d['params'] = {k: v for k, v in self.params.items() if v}
        d['overrides'] = {
            key: [{'path': list(path), 'value': value} for path, value in sorted(items.items())]
            for key, items in self.overrides.items() if items}
        return d

    @classmethod
    def from_dict(cls, d):
        s = cls()
        if not isinstance(d, dict):
            return s
        known = {f.name for f in fields(cls)}
        for key, value in d.items():
            if key not in known or key in ('params', 'overrides'):
                continue
            default = getattr(s, key)
            try:
                if isinstance(default, bool):
                    value = _bool(value, default)
                elif isinstance(default, float):
                    value = float(value)
                elif isinstance(default, str):
                    value = '' if value is None else str(value)
            except (TypeError, ValueError):
                continue
            setattr(s, key, value)
        if s.autonomy not in AUTONOMY:
            s.autonomy = 'explore'
        s.params = {k: str(v) for k, v in (d.get('params') or {}).items()
                    if k in PARAM_FILES and v}
        for key, items in (d.get('overrides') or {}).items():
            if key not in PARAM_FILES:
                continue
            s.overrides[key] = {tuple(str(p) for p in item['path']): item['value']
                                for item in items or [] if item.get('path')}
        return s

    @classmethod
    def from_launch_args(cls, args):
        """Settings that repeat a run started from a terminal (run.json launch_args)."""
        s = cls()
        if _bool(args.get('explore')):
            s.autonomy = 'explore'
        elif _bool(args.get('nav')):
            s.autonomy = 'nav'
        else:
            s.autonomy = 'manual'
        for name in BOOL_ARGS:
            if name in args:
                setattr(s, name, _bool(args[name], getattr(s, name)))
        for name in ('x_pose', 'y_pose'):
            try:
                setattr(s, name, float(args.get(name, getattr(s, name))))
            except (TypeError, ValueError):
                pass
        world = args.get('world') or ''
        default_world = _default_world()
        if world and os.path.abspath(world) != default_world:
            s.world = world
        for key, (arg, _, _) in PARAM_FILES.items():
            path = args.get(arg) or ''
            if path and os.path.abspath(path) != resolve(key):
                s.params[key] = path
        return s

    def copy(self):
        return RunSettings.from_dict(self.to_dict())


def _default_world():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory(LAUNCH_PACKAGE), 'worlds', 'maze.sdf')
    except Exception:
        return ''


# ---------------------------------------------------------------- presets
def _slug(name):
    return re.sub(r'[^A-Za-z0-9_.-]+', '-', name.strip()).strip('-.')[:60] or 'preset'


def default_presets_dir():
    base = os.environ.get('XDG_CONFIG_HOME') or os.path.expanduser('~/.config')
    return os.path.join(base, 'maze_monitor', 'presets')


class PresetStore:
    """Named run settings, one YAML file each, so they can be shared or kept in git."""

    def __init__(self, directory=None):
        self.directory = directory or default_presets_dir()

    def _path(self, name):
        return os.path.join(self.directory, _slug(name) + '.yaml')

    def names(self):
        if not os.path.isdir(self.directory):
            return []
        out = []
        for entry in sorted(os.listdir(self.directory)):
            if entry.endswith('.yaml'):
                try:
                    with open(os.path.join(self.directory, entry)) as f:
                        data = yaml.safe_load(f) or {}
                    out.append(str(data.get('name') or entry[:-5]))
                except (OSError, yaml.YAMLError):
                    continue
        return sorted(out, key=str.lower)

    def load(self, name):
        with open(self._path(name)) as f:
            data = yaml.safe_load(f) or {}
        return RunSettings.from_dict(data.get('settings') or {})

    def save(self, name, settings):
        os.makedirs(self.directory, exist_ok=True)
        path = self._path(name)
        tmp = path + '.tmp'
        with open(tmp, 'w') as f:
            f.write('# maze_monitor run preset\n')
            yaml.safe_dump({'name': name.strip(), 'settings': settings.to_dict()}, f,
                           sort_keys=False)
        os.replace(tmp, path)
        return path

    def delete(self, name):
        path = self._path(name)
        if os.path.isfile(path):
            os.remove(path)
