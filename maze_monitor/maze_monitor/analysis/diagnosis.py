"""Plain-language status of the robot at one moment, built from a Snapshot."""

import math

from .analyzer import severity_rank
from .geometry import direction_name


class Units:
    """Angle unit for display: degrees or radians."""

    def __init__(self, degrees=True):
        self.degrees = degrees

    def angle(self, rad, digits=None):
        if rad is None:
            return '—'
        if self.degrees:
            return f'{math.degrees(rad):.{1 if digits is None else digits}f}°'
        return f'{rad:.{3 if digits is None else max(digits, 2)}f} rad'

    def rate(self, rad_s):
        if rad_s is None:
            return '—'
        if self.degrees:
            return f'{math.degrees(rad_s):.0f}°/s'
        return f'{rad_s:.2f} rad/s'


def activity(v, w, units):
    """Describe a motion in words, for example 'driving forward at 0.18 m/s'."""
    if v is None and w is None:
        return 'no odometry'
    v = v or 0.0
    w = w or 0.0
    moving = abs(v) >= 0.02
    turning = abs(w) >= 0.05
    side = 'left' if w > 0 else 'right'
    if not moving and not turning:
        return 'standing still'
    if not moving:
        return f'turning {side} in place at {units.rate(abs(w))}'
    direction = 'forward' if v > 0 else 'backward'
    text = f'driving {direction} at {abs(v):.2f} m/s'
    if turning:
        text += f' while turning {side} at {units.rate(abs(w))}'
    return text


def diagnose(snap, units):
    """Return (level, headline, lines). level is 'ok', 'warning', or 'error'."""
    g = snap.get
    lines = []

    actual = activity(g('odom.v'), g('odom.w'), units)
    cmd_v, cmd_w = g('cmd.v'), g('cmd.w')
    if cmd_v is not None or cmd_w is not None:
        told = activity(cmd_v, cmd_w, units)
        if told == actual or (told == 'standing still' and actual == 'standing still'):
            lines.append(f'The robot is {actual}, as commanded.')
        else:
            lines.append(f'The robot is {actual}; it is told to be {told}.')
    else:
        lines.append(f'The robot is {actual}. No speed commands received yet.')

    state = g('nav.state')
    if state == 'active':
        start = g('nav.goal_start')
        dist = g('nav.distance_remaining')
        parts = []
        if start is not None:
            parts.append(f'{snap.t - start:.0f} s so far')
        rec = g('nav.goal_recoveries')
        if rec is not None:
            parts.append(f'{int(rec)} recover{"y" if rec == 1 else "ies"}')
        where = f'{dist:.2f} m away' if dist is not None else 'distance unknown'
        lines.append(f'Driving to a goal {where} ({", ".join(parts)}).' if parts
                     else f'Driving to a goal {where}.')
    elif state in ('succeeded', 'aborted', 'canceled'):
        lines.append(f'No active goal. The last goal {state}.')
    else:
        lines.append('No navigation goal yet.')

    explore = g('explore.status')
    frontiers = g('explore.frontiers')
    known = g('map.known_area')
    if explore or frontiers is not None:
        text = (explore or 'exploring').replace('exploration_', '').replace('_', ' ')
        extra = []
        if frontiers is not None:
            extra.append(f'{int(frontiers)} frontiers left')
        if known is not None:
            extra.append(f'{known:.1f} m² mapped')
        lines.append(f'Exploration: {text}' + (f' ({", ".join(extra)}).' if extra else '.'))

    nearest, bearing = g('clear.min'), g('clear.bearing')
    if nearest is not None and bearing is not None:
        lines.append(f'Nearest wall: {max(nearest, 0):.2f} m, {direction_name(bearing)}.')

    # Most severe first; among equals, the monitor's own findings before raw log lines,
    # then the most recent.
    problems = sorted((e for e in snap.events if e.severity != 'info'),
                      key=lambda e: (-severity_rank(e.severity), e.category == 'log', -e.t))
    level = 'ok'
    if problems:
        level = problems[0].severity
    headline = {'ok': 'OK', 'warning': 'Warning', 'error': 'Problem'}[level]
    if problems:
        headline += ': ' + problems[0].title
    return level, headline, lines, problems
