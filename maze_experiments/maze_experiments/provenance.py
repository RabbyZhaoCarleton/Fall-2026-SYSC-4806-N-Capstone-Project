"""Provenance of an experiment (DR6): exactly which code and parameters produced it.

The robot stack changes often (2.2, item 9), so a result is meaningless
without this.
"""

import glob
import hashlib
import os
import platform
import subprocess

import yaml

from . import paths


def _run(cmd, cwd=None, timeout=20):
    try:
        out = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.SubprocessError):
        return None
    return out.stdout.strip() if out.returncode == 0 else None


def file_hash(path):
    h = hashlib.sha256()
    try:
        with open(path, 'rb') as f:
            for block in iter(lambda: f.read(1 << 16), b''):
                h.update(block)
    except OSError:
        return None
    return h.hexdigest()


def git_state(path):
    if not path or not os.path.isdir(path):
        return {'path': path, 'commit': None, 'error': 'not found'}
    commit = _run(['git', '-C', path, 'rev-parse', 'HEAD'])
    branch = _run(['git', '-C', path, 'rev-parse', '--abbrev-ref', 'HEAD'])
    status = _run(['git', '-C', path, 'status', '--porcelain', '--untracked-files=no'])
    diff = None
    if status:
        try:
            d = subprocess.run(['git', '-C', path, 'diff', 'HEAD'], capture_output=True,
                               timeout=30).stdout
            diff = hashlib.sha256(d).hexdigest()
        except (OSError, subprocess.SubprocessError):
            diff = 'unknown'
    return {'path': path, 'commit': commit, 'branch': branch,
            'uncommitted_changes': bool(status),
            'changed_files': status.splitlines() if status else [],
            'diff_sha256': diff}


def third_party(repo):
    """Pinned versions from capstone.repos, the checked-out commits, and the patches."""
    out = {'repos': {}, 'patches': []}
    if not repo:
        return out
    try:
        with open(os.path.join(repo, 'capstone.repos')) as f:
            pinned = (yaml.safe_load(f) or {}).get('repositories', {})
    except OSError:
        pinned = {}
    src = os.path.dirname(repo)
    for name, spec in pinned.items():
        state = git_state(os.path.join(src, name))
        out['repos'][name] = {'pinned': str(spec.get('version')), 'url': spec.get('url'),
                              'checked_out': state.get('commit'),
                              'local_changes': state.get('uncommitted_changes'),
                              'diff_sha256': state.get('diff_sha256')}
    for patch in sorted(glob.glob(os.path.join(repo, 'patches', '*', '*.patch'))):
        out['patches'].append({'file': os.path.relpath(patch, repo), 'sha256': file_hash(patch)})
    return out


def ros_versions():
    def pkg(name):
        return _run(['dpkg-query', '-W', '-f=${Version}', name])
    distro = os.environ.get('ROS_DISTRO')
    gz = _run(['gz', 'sim', '--versions'], timeout=30)
    return {'ros_distro': distro,
            'rclpy': pkg(f'ros-{distro}-rclpy') if distro else None,
            'nav2': pkg(f'ros-{distro}-nav2-bringup') if distro else None,
            'slam_toolbox': pkg(f'ros-{distro}-slam-toolbox') if distro else None,
            'gazebo': gz.splitlines()[-1] if gz else None}


def machine(renderer=None):
    from . import resources
    m = resources.measure_machine(renderer=renderer)
    return {'cpu_model': m.get('cpu_model'), 'logical_cpus': m.get('logical_cpus'),
            'wsl_memory_bytes': m.get('memory_total'), 'renderer': m.get('renderer'),
            'gpu_memory_mb': m.get('gpu_memory_mb'), 'kernel': platform.release(),
            'hostname_hash': hashlib.sha256(platform.node().encode()).hexdigest()[:12]}


def collect(profile, param_files, renderer=None):
    """param_files: {label: absolute path} of every stack parameter file used."""
    repo = paths.repo_root()
    return {
        'capstone': git_state(repo),
        'third_party': third_party(repo),
        'parameter_files': {label: {'path': p, 'sha256': file_hash(p)}
                            for label, p in sorted(param_files.items())},
        'robot_profile': {'name': profile.name, 'file': profile.path,
                          'sha256': file_hash(profile.path) if profile.path else None,
                          'data': profile.data},
        'versions': ros_versions(),
        'machine': machine(renderer),
        'maze_experiments_version': __import__('maze_experiments').__version__,
    }
