"""Robot profiles (1.4, FR1): the simulated robot and its default stack parameters."""

import glob
import os

from . import fileio, paths

REQUIRED = ('name', 'footprint_radius', 'navigation_radius', 'costmap_resolution',
            'tested_corridor_width', 'lidar', 'stack_params')
STACK_PARAM_KEYS = ('nav_params', 'slam_params', 'explore_params', 'ekf_params')


class ProfileError(ValueError):
    pass


class RobotProfile:
    def __init__(self, data, path=None):
        missing = [k for k in REQUIRED if k not in data]
        if missing:
            raise ProfileError(f'{path or "robot profile"}: missing {", ".join(missing)}')
        self.data = data
        self.path = path

    name = property(lambda self: self.data['name'])
    footprint_radius = property(lambda self: float(self.data['footprint_radius']))
    navigation_radius = property(lambda self: float(self.data['navigation_radius']))
    costmap_resolution = property(lambda self: float(self.data['costmap_resolution']))
    tested_corridor_width = property(lambda self: float(self.data['tested_corridor_width']))
    lidar_range = property(lambda self: float(self.data['lidar']['range_max']))
    lidar_min_range = property(lambda self: float(self.data['lidar'].get('range_min', 0.0)))
    lidar_height = property(lambda self: float(self.data['lidar']['mount_height']))

    @property
    def min_corridor_width(self):
        """Hard minimum (AR7): 2 x navigation radius + 2 x costmap resolution."""
        return 2 * self.navigation_radius + 2 * self.costmap_resolution

    @property
    def min_wall_height(self):
        return self.lidar_height + 0.05

    @property
    def env(self):
        return {str(k): str(v) for k, v in (self.data.get('env') or {}).items()}

    @property
    def launch_args(self):
        args = {k: str(v) for k, v in (self.data.get('launch_args') or {}).items()
                if v not in (None, '')}
        return args

    def stack_params(self):
        """Absolute paths of the default stack parameter files."""
        return {k: paths.resolve_package_path(v)
                for k, v in (self.data.get('stack_params') or {}).items()}


def profile_dirs(data_dir=None):
    dirs = [os.path.join(paths.own_share(), 'config', 'robots')]
    if data_dir:
        dirs.append(os.path.join(data_dir, 'robots'))
    return dirs


def available(data_dir=None):
    names = {}
    for d in profile_dirs(data_dir):
        for f in sorted(glob.glob(os.path.join(d, '*.yaml'))):
            names[os.path.splitext(os.path.basename(f))[0]] = f
    return names


def load(name_or_path, data_dir=None):
    if os.path.isfile(os.path.expanduser(str(name_or_path))):
        path = os.path.expanduser(name_or_path)
    else:
        path = available(data_dir).get(name_or_path)
        if path is None:
            raise ProfileError(f'Robot profile "{name_or_path}" not found in '
                               f'{", ".join(profile_dirs(data_dir))}')
    data = fileio.read_yaml(path)
    return RobotProfile(data, path)
