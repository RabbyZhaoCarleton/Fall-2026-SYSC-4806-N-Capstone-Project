"""ROS 2 parameter files: find the defaults, change single values, and save a copy per run.

A run can override single parameters, for example explore_lite's
progress_timeout, without editing the files in maze_navigation. The monitor
then writes a copy of the file with the changes into the run folder and starts
the launch with that copy. Every run folder keeps a copy of each parameter
file it used, changed or not, so a run can always be traced back to its
settings.
"""

import copy
import os
import shutil

import yaml

# key: (launch argument, file in maze_navigation, title)
PARAM_FILES = {
    'slam': ('slam_params', 'config/slam_maze.yaml', 'SLAM (slam_toolbox)'),
    'nav': ('nav_params', 'config/nav2_maze.yaml', 'Navigation (Nav2)'),
    'explore': ('explore_params', 'config/explore_maze.yaml', 'Exploration (explore_lite)'),
    'ekf': ('ekf_params', 'config/ekf_maze.yaml', 'EKF (robot_localization)'),
}
PARAMS_DIR = 'params'


def default_path(key):
    """Installed default file for key, or the one in the source tree, or ''."""
    rel = PARAM_FILES[key][1]
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('maze_navigation'), rel)
        if os.path.isfile(path):
            return path
    except Exception:
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.normpath(os.path.join(here, '..', '..', 'maze_navigation', rel))
    return path if os.path.isfile(path) else ''


def resolve(key, chosen=''):
    """The file a run uses for key: the chosen one, or the default."""
    return os.path.abspath(os.path.expanduser(chosen)) if chosen else default_path(key)


def load(path):
    with open(path) as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f'{path} is not a ROS 2 parameter file (no node names at the top)')
    return data


def leaves(data, prefix=()):
    """[(path tuple, value)] for every parameter. Lists are single values, as in ROS."""
    out = []
    for key, value in data.items():
        path = prefix + (str(key),)
        if isinstance(value, dict) and value:
            out.extend(leaves(value, path))
        else:
            out.append((path, value))
    return out


def get(data, path, default=None):
    for key in path:
        if not isinstance(data, dict) or key not in data:
            return default
        data = data[key]
    return data


def set_value(data, path, value):
    for key in path[:-1]:
        data = data.setdefault(key, {})
    data[path[-1]] = value


def apply(data, overrides):
    """A copy of data with the overrides ({path tuple: value}) applied."""
    out = copy.deepcopy(data)
    for path, value in overrides.items():
        set_value(out, tuple(path), value)
    return out


def display_name(path):
    """'controller_server', 'FollowPath.max_vel_x' for (node, 'ros__parameters', ...)."""
    rest = path[2:] if len(path) > 2 and path[1] == 'ros__parameters' else path[1:]
    return path[0], '.'.join(rest)


def format_value(value):
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, (list, dict)):
        return yaml.safe_dump(value, default_flow_style=True, width=10 ** 6).strip()
    if value is None:
        return ''
    return str(value)


def parse_value(text, like):
    """Parse text typed by the user as a value of the same ROS type as like.

    ROS 2 refuses a parameter of the wrong type when the node starts (an
    integer parameter given 1.5, for example), so check it here instead.
    """
    text = text.strip()
    try:
        value = yaml.safe_load(text) if text else ''
    except yaml.YAMLError as exc:
        raise ValueError(f'Not a valid value: {exc}') from None
    if isinstance(like, bool):
        if not isinstance(value, bool):
            raise ValueError('Expected true or false.')
    elif isinstance(like, int):
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError('Expected a whole number (this parameter is an integer).')
    elif isinstance(like, float):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError('Expected a number.')
        value = float(value)
    elif isinstance(like, str):
        if not isinstance(value, str):
            value = text
    elif isinstance(like, list):
        if not isinstance(value, list):
            raise ValueError('Expected a list, for example [1.0, 2.0] or ["a", "b"].')
        kinds = {type(v) for v in like}
        if len(kinds) == 1 and value:
            kind = kinds.pop()
            if kind is float:
                if any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in value):
                    raise ValueError('Expected a list of numbers.')
                value = [float(v) for v in value]
            elif any(not isinstance(v, kind) or isinstance(v, bool) != (kind is bool)
                     for v in value):
                raise ValueError(f'Expected a list of {kind.__name__} values.')
    return value


def write_with_overrides(src, overrides, dest):
    """Write src with overrides applied to dest, with a header that lists the changes."""
    data = apply(load(src), overrides)
    lines = [f'# Written by maze_monitor for one run: a copy of\n#   {src}\n# with these changes:']
    for path, value in sorted(overrides.items()):
        node, name = display_name(tuple(path))
        lines.append(f'#   {node}: {name} = {format_value(value)}')
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    with open(dest, 'w') as f:
        f.write('\n'.join(lines) + '\n')
        yaml.safe_dump(data, f, sort_keys=False, default_flow_style=None, width=100)


def prepare_run_files(run_dir, chosen, overrides):
    """Copy the parameter files of a run into run_dir/params.

    chosen: {key: path or ''}; overrides: {key: {path tuple: value}}.
    Returns {key: path} for the files the launch must use instead of its
    defaults: the changed copies, and files the user chose. Unchanged default
    files are copied for the record only, so the launch command stays short.
    """
    use = {}
    out_dir = os.path.join(run_dir, PARAMS_DIR)
    os.makedirs(out_dir, exist_ok=True)
    for key in PARAM_FILES:
        src = resolve(key, chosen.get(key, ''))
        if not src or not os.path.isfile(src):
            if chosen.get(key) or overrides.get(key):
                raise FileNotFoundError(f'Parameter file not found: {src or chosen.get(key)}')
            continue
        dest = os.path.join(out_dir, os.path.basename(src))
        changes = overrides.get(key) or {}
        if changes:
            write_with_overrides(src, changes, dest)
            use[key] = dest
        else:
            if os.path.abspath(src) != os.path.abspath(dest):
                shutil.copyfile(src, dest)
            if chosen.get(key):
                use[key] = src
    return use
