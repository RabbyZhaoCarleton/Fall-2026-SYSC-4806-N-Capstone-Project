"""Settings for the monitor: defaults, overridden by an optional YAML file."""

import copy
import os
import re
from dataclasses import dataclass, field

import yaml

DEFAULTS = {
    'runs_dir': '~/maze_runs',
    'record': {
        'exclude': [
            '/parameter_events', '/rosout_agg', r'.*/bond', r'.*/transition_event',
            r'.*image.*', r'.*/camera/.*', r'.*points.*', r'.*costmap_raw.*',
            r'.*costmap_updates', r'.*_action/cancel_goal.*', '/evaluation', '/marker',
            r'.*/voxel_grid', r'.*costmap/.*_layer.*',
        ],
        'max_rate_hz': {'/clock': 10.0, '/joint_states': 10.0, '/imu': 100.0,
                        '/ground_truth/poses': 20.0},
        'large_message_kb': 256,
        'large_message_max_hz': 0.5,
    },
    'robot': {'radius': 0.105},
    'thresholds': {
        'near_wall_clearance': 0.05,
        'contact_clearance': 0.015,
        'blocked_min_cmd': 0.05,
        'blocked_ratio': 0.3,
        'blocked_time': 1.0,
        'slip_time': 1.5,
        'stuck_window': 10.0,
        'stuck_distance': 0.10,
        'spin_angle_deg': 360.0,
        'spin_window': 15.0,
        'spin_distance': 0.15,
        'oscillation_flips': 6,
        'oscillation_window': 5.0,
        'localization_jump_m': 0.10,
        'localization_jump_deg': 5.0,
        'impact_accel': 6.0,
        'silent_periods': 5.0,
        'slow_rtf': 0.5,
        'exploration_stall': 60.0,
    },
    'hidden_topics': [
        '/clock', '/joint_states', '/robot_description', '/tf_static',
        r'.*/published_footprint', r'.*/footprint', r'.*/transition_event',
    ],
}


def _merge(base, override):
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _merge(base[key], value)
        else:
            base[key] = value
    return base


def _patterns(items):
    return [re.compile(p) for p in items]


@dataclass
class Config:
    runs_dir: str
    exclude: list
    max_rate_hz: dict
    large_message_bytes: int
    large_message_max_hz: float
    robot_radius: float
    thresholds: dict
    hidden_patterns: list
    raw: dict = field(repr=False, default_factory=dict)

    @classmethod
    def load(cls, path=None, runs_dir=None):
        data = copy.deepcopy(DEFAULTS)
        if path:
            with open(os.path.expanduser(path)) as f:
                _merge(data, yaml.safe_load(f) or {})
        runs_dir = runs_dir or os.environ.get('MAZE_RUNS_DIR')
        if runs_dir:
            data['runs_dir'] = runs_dir
        return cls(
            runs_dir=os.path.abspath(os.path.expanduser(data['runs_dir'])),
            exclude=_patterns(data['record']['exclude']),
            max_rate_hz=dict(data['record']['max_rate_hz']),
            large_message_bytes=int(float(data['record']['large_message_kb']) * 1024),
            large_message_max_hz=float(data['record']['large_message_max_hz']),
            robot_radius=float(data['robot']['radius']),
            thresholds=dict(data['thresholds']),
            hidden_patterns=_patterns(data['hidden_topics']),
            raw=data,
        )

    def is_excluded(self, topic):
        return any(p.fullmatch(topic) for p in self.exclude)

    def hidden_by_default(self, topic):
        return any(p.fullmatch(topic) for p in self.hidden_patterns)

    def th(self, name):
        return self.thresholds[name]
