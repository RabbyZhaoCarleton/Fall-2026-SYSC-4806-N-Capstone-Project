"""Where things are: the data directory, the repository, installed packages.

The data directory defaults to ~/maze_experiments. It can be changed with the
--data-dir option, the MAZE_EXPERIMENTS_DIR environment variable, or
data_dir in ~/.config/maze_experiments/settings.yaml (in that order). It must
be on the Linux file system: SQLite on the Windows drive (/mnt/c) is slow and
its file locking is unreliable (2.1).
"""

import os
import subprocess

import yaml

DEFAULT_DATA_DIR = '~/maze_experiments'
SETTINGS_FILE = '~/.config/maze_experiments/settings.yaml'


class DataDirError(ValueError):
    pass


def settings():
    path = os.path.expanduser(SETTINGS_FILE)
    try:
        with open(path) as f:
            return yaml.safe_load(f) or {}
    except (OSError, ValueError):
        return {}


def check_data_dir(path):
    """Raise DataDirError if the folder can't hold experiment data."""
    real = os.path.realpath(os.path.expanduser(path))
    if real == '/mnt' or real.startswith('/mnt/'):
        raise DataDirError(
            f'The data directory {real} is on the Windows drive. Use a folder on the Linux '
            f'file system (for example ~/maze_experiments): SQLite under /mnt is slow and its '
            f'file locking is unreliable.')
    return real


def data_dir(override=None):
    path = (override or os.environ.get('MAZE_EXPERIMENTS_DIR') or settings().get('data_dir')
            or DEFAULT_DATA_DIR)
    return check_data_dir(path)


def ensure_writable(path):
    os.makedirs(path, exist_ok=True)
    probe = os.path.join(path, '.write-test')
    with open(probe, 'w') as f:
        f.write('ok')
    os.unlink(probe)


def package_share(name):
    """Share directory of an installed ROS package, or None."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(name)
    except Exception:
        return None


def own_share():
    share = package_share('maze_experiments')
    if share and os.path.isdir(os.path.join(share, 'config')):
        return share
    # Running from the source tree (tests, or before the first build).
    return os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))


def resolve_package_path(ref):
    """'package://<pkg>/<path>' to an absolute path; other paths are expanded."""
    if isinstance(ref, str) and ref.startswith('package://'):
        pkg, _, rest = ref[len('package://'):].partition('/')
        share = package_share(pkg)
        if share is None:
            src = repo_root()
            if src and os.path.isdir(os.path.join(src, pkg)):
                share = os.path.join(src, pkg)
            else:
                raise FileNotFoundError(f'Package {pkg} is not installed (needed for {ref}). '
                                        f'Build the workspace: ./setup.sh')
        return os.path.join(share, rest)
    return os.path.abspath(os.path.expanduser(ref))


def _git_root(path):
    try:
        out = subprocess.run(['git', '-C', path, 'rev-parse', '--show-toplevel'],
                             capture_output=True, text=True, timeout=10)
    except (OSError, subprocess.SubprocessError):
        return None
    return out.stdout.strip() or None if out.returncode == 0 else None


def repo_root():
    """The capstone repository: from MAZE_CAPSTONE_REPO, the source tree, or the install."""
    env = os.environ.get('MAZE_CAPSTONE_REPO')
    if env and os.path.isdir(env):
        return os.path.abspath(env)
    here = os.path.realpath(os.path.dirname(__file__))
    root = _git_root(here)
    if root and os.path.isdir(os.path.join(root, 'maze_sim')):
        return root
    share = package_share('maze_sim')
    if share:
        launch = os.path.realpath(os.path.join(share, 'launch', 'maze.launch.py'))
        root = _git_root(os.path.dirname(launch))
        if root:
            return root
    guess = os.path.expanduser('~/ros2_ws/src/capstone')
    return guess if os.path.isdir(guess) else None


def workspace_root():
    root = repo_root()
    if root:
        ws = os.path.abspath(os.path.join(root, '..', '..'))
        if os.path.isdir(os.path.join(ws, 'src')):
            return ws
    return None
