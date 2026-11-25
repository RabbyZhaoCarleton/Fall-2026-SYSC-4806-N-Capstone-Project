"""Turns messages into series and events.

The analyzer owns a set of interpreters. Each interpreter handles certain
message types, keeps its own state, writes series, and raises events. Live
mode and replay call the analyzer in the same way, with the same tick grid,
so they produce the same results.
"""

import math

from ..timeline import Event, SEVERITIES
from .geometry import Pose2D

TICK = 0.25  # seconds between periodic detector runs


class Context:
    """What interpreters use to read shared state and write results."""

    def __init__(self, timeline, config):
        self.timeline = timeline
        self.config = config
        self.state = {'pose': None}  # values shared between interpreters
        self._open = {}  # condition key -> [event, last_active_time]

    @property
    def pose(self):
        return self.state.get('pose')

    def th(self, name):
        return self.config.th(name)

    # ------------------------------------------------------------ series
    def emit(self, t, key, value, label=None, unit='', numeric=True, min_interval=0.0, force=False):
        self.timeline.get_series(key, label, unit, numeric, min_interval).add(t, value, force)

    def declare(self, key, label, unit='', numeric=True, min_interval=0.0):
        return self.timeline.get_series(key, label, unit, numeric, min_interval)

    # ------------------------------------------------------------ events
    def _where(self):
        p = self.pose
        return (p.x, p.y) if p is not None else (None, None)

    def event(self, t, severity, category, title, detail='', merge_key=None,
              merge_window=0.0, x=None, y=None):
        """Record an instant event. Repeats with the same merge_key are merged."""
        tl = self.timeline
        if merge_key and merge_window > 0:
            for e in reversed(tl.events[-50:]):
                if e.key == merge_key and not e.is_condition:
                    if t - (e.t_end or e.t) <= merge_window:
                        e.count += 1
                        e.t_end = t
                        if severity_rank(severity) > severity_rank(e.severity):
                            e.severity = severity
                        tl.touch_events()
                        return e
                    break
        if x is None:
            x, y = self._where()
        return tl.add_event(Event(0, t, severity, category, title, detail, t_end=None,
                                  x=x, y=y, key=merge_key or ''))

    def condition(self, t, key, active, severity='warning', category='', title='', detail='',
                  hold=0.5):
        """Track a condition that lasts. Returns the open event, if any.

        While `active` stays true, one event stays open; its title and detail
        follow the latest values. It closes after being inactive for `hold`
        seconds, with its end time set to the last active time.
        """
        entry = self._open.get(key)
        if active:
            if entry is None:
                x, y = self._where()
                ev = self.timeline.add_event(Event(0, t, severity, category, title, detail,
                                                   is_condition=True, x=x, y=y, key=key))
                self._open[key] = [ev, t]
                return ev
            ev = entry[0]
            entry[1] = t
            changed = False
            if severity_rank(severity) > severity_rank(ev.severity):
                ev.severity = severity
                changed = True
            if title and title != ev.title and not ev.extra.get('freeze_title'):
                ev.title = title
                changed = True
            if detail and detail != ev.detail:
                ev.detail = detail
                changed = True
            if changed:
                self.timeline.touch_events()
            return ev
        if entry is not None and t - entry[1] >= hold:
            entry[0].t_end = entry[1]
            del self._open[key]
            self.timeline.touch_events()
        return None

    def is_open(self, key):
        return key in self._open

    def close_all(self, t):
        for ev, last in self._open.values():
            ev.t_end = last
        self._open.clear()
        self.timeline.touch_events()


def severity_rank(severity):
    return SEVERITIES.index(severity) if severity in SEVERITIES else 0


class Interpreter:
    """Base class. Subclasses set `types` and override the handlers."""

    #: Message type names this interpreter reads (for example, 'nav_msgs/msg/Odometry').
    types = ()

    def accepts(self, topic, type_name):
        return type_name in self.types

    def on_message(self, ctx, t, topic, msg):
        pass

    def tick(self, ctx, t):
        pass

    def on_any(self, ctx, t, track):
        """Called for every message of every topic, without deserializing."""


class Analyzer:
    def __init__(self, timeline, config, interpreters=None):
        from . import interpreters as interp
        self.timeline = timeline
        self.config = config
        self.ctx = Context(timeline, config)
        self.interpreters = interpreters or interp.default_interpreters()
        self._routes = {}  # (topic, type) -> [interpreters]
        self._next_tick = TICK

    def _route(self, topic, type_name):
        key = (topic, type_name)
        r = self._routes.get(key)
        if r is None:
            r = self._routes[key] = [i for i in self.interpreters if i.accepts(topic, type_name)]
        return r

    def feed(self, t, track, index):
        """Analyze message `index` of `track`, received at time t."""
        self.advance(t)
        for i in self.interpreters:
            i.on_any(self.ctx, t, track)
        handlers = self._route(track.name, track.type_name)
        if not handlers:
            return
        msg = track.message(index)
        if msg is None:
            return
        for h in handlers:
            try:
                h.on_message(self.ctx, t, track.name, msg)
            except Exception as exc:  # never let one bad message stop the analysis
                self.ctx.event(t, 'warning', 'monitor', f'Monitor could not analyze {track.name}',
                               f'{type(exc).__name__}: {exc}', merge_key=f'err:{track.name}',
                               merge_window=1e9)

    def advance(self, t):
        """Run the periodic detectors on the tick grid up to time t."""
        while self._next_tick <= t:
            tick_t = self._next_tick
            self.timeline.advance(tick_t)
            for i in self.interpreters:
                i.tick(self.ctx, tick_t)
            self._next_tick += TICK

    def finish(self, t):
        self.advance(t)
        self.ctx.close_all(t)


def summarize(timeline):
    """A small summary of a run, stored in run.json for the run list."""
    t = timeline.duration
    counts = {s: 0 for s in SEVERITIES}
    for e in timeline.events:
        counts[e.severity] += 1
    collisions = sum(1 for e in timeline.events if e.category == 'collision')
    v = timeline.value
    known = v('map.known_area', t)
    return {
        'duration_s': round(t, 1),
        'distance_m': round(v('pose.distance', t, 0.0), 2),
        'errors': counts['error'],
        'warnings': counts['warning'],
        'infos': counts['info'],
        'collisions': collisions,
        'goals_succeeded': int(v('nav.goals_succeeded', t, 0)),
        'goals_failed': int(v('nav.goals_failed', t, 0)),
        'recoveries': int(v('nav.recoveries', t, 0)),
        'known_area_m2': None if known is None else round(known, 2),
        'explore_status': v('explore.status', t),
        'messages': sum(len(tr) for tr in timeline.tracks.values()),
        'topics': len(timeline.tracks),
    }


def fmt_xy(x, y):
    if x is None or y is None or math.isnan(x):
        return 'unknown position'
    return f'({x:.2f}, {y:.2f})'


__all__ = ['Analyzer', 'Context', 'Interpreter', 'Pose2D', 'summarize', 'TICK', 'fmt_xy']
