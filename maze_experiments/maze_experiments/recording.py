"""Recording rules for runs: maze_monitor's defaults plus the recording profile (DR1, DR2)."""

import os
import re

from . import fileio, paths

REQUIRED_TOPICS = ('/ground_truth/poses', '/odom', '/odometry/filtered', '/tf', '/tf_static',
                   '/scan', '/imu', '/map', '/cmd_vel', '/plan', '/explore/status',
                   '/explore/exits', '/explore/frontier_states', '/rosout', '/clock')


def profile_file(name):
    return os.path.join(paths.own_share(), 'config', 'recording', f'{name}.yaml')


def monitor_config(profile='full'):
    """A maze_monitor Config with the monitor's default rules and the profile's exclusions."""
    from maze_monitor.config import Config
    share = paths.package_share('maze_monitor')
    default = os.path.join(share, 'config', 'monitor.yaml') if share else None
    cfg = Config.load(default if default and os.path.isfile(default) else None)
    prof = fileio.read_yaml(profile_file(profile))
    cfg.exclude = list(cfg.exclude) + [re.compile(p) for p in prof.get('extra_exclude') or []]
    return cfg
