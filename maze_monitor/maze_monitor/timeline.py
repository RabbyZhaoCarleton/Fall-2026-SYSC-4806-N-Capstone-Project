"""Everything known about one run, indexed by time.

The views draw a Timeline at one point in time (the cursor). Live mode appends
to the timeline as messages arrive; replay builds it once from a recording.
All times are seconds since the start of the run.
"""

import bisect
from array import array
from collections import OrderedDict
from dataclasses import dataclass, field
from typing import Optional

SEVERITIES = ('info', 'warning', 'error')
# How long an instant event (one without a duration) counts as "recent".
RECENT_WINDOW = 5.0


class TopicTrack:
    """Receive times and serialized messages of one topic."""

    def __init__(self, name, type_name):
        self.name = name
        self.type_name = type_name
        self.times = array('d')
        self.data = []
        self.total_bytes = 0
        self._msg_class = None
        self._type_error = None
        self._cache = OrderedDict()

    def append(self, t, data):
        if self.times and t < self.times[-1]:
            t = self.times[-1]
        self.times.append(t)
        self.data.append(data)
        self.total_bytes += len(data)

    def __len__(self):
        return len(self.times)

    def index_at(self, t):
        """Index of the last message at or before t, or -1."""
        return bisect.bisect_right(self.times, t) - 1

    def count_at(self, t):
        return self.index_at(t) + 1

    @property
    def msg_class(self):
        if self._msg_class is None and self._type_error is None:
            try:
                from rosidl_runtime_py.utilities import get_message
                self._msg_class = get_message(self.type_name)
            except Exception as exc:  # unknown or unbuilt message type
                self._type_error = str(exc)
        return self._msg_class

    @property
    def type_error(self):
        _ = self.msg_class
        return self._type_error

    def message(self, index):
        """Deserialized message at an index (cached), or None."""
        if index < 0 or index >= len(self.data):
            return None
        if index in self._cache:
            self._cache.move_to_end(index)
            return self._cache[index]
        cls = self.msg_class
        if cls is None:
            return None
        from rclpy.serialization import deserialize_message
        try:
            msg = deserialize_message(self.data[index], cls)
        except Exception:
            return None
        self._cache[index] = msg
        if len(self._cache) > 16:
            self._cache.popitem(last=False)
        return msg

    def latest(self, t):
        """(time, message) of the last message at or before t, or (None, None)."""
        i = self.index_at(t)
        if i < 0:
            return None, None
        return self.times[i], self.message(i)

    def rate_at(self, t, window=2.0):
        """Average message rate in Hz over the window ending at t."""
        if not self.times or t < self.times[0]:
            return 0.0
        span = min(window, max(t - self.times[0], 1e-3))
        n = self.index_at(t) - self.index_at(t - span)
        return n / span if span >= 0.2 else 0.0

    def bytes_rate_at(self, t, window=2.0):
        if not self.times:
            return 0.0
        return self.rate_at(t, window) * self.total_bytes / len(self.times)


class Series:
    """Values over time. Lookups return the last value at or before a time."""

    def __init__(self, key, label, unit='', numeric=True, min_interval=0.0):
        self.key = key
        self.label = label
        self.unit = unit
        self.numeric = numeric
        self.min_interval = min_interval
        self.times = array('d')
        self.values = array('d') if numeric else []

    def add(self, t, value, force=False):
        if value is None:
            return
        if self.times:
            if t < self.times[-1]:
                t = self.times[-1]
            if not force and self.min_interval and t - self.times[-1] < self.min_interval:
                # Keep the newest value in the last slot instead of adding a sample.
                self.values[-1] = value
                return
            if not self.numeric and self.values[-1] == value:
                return
        self.times.append(t)
        self.values.append(value)

    def __len__(self):
        return len(self.times)

    def index_at(self, t):
        return bisect.bisect_right(self.times, t) - 1

    def value_at(self, t, max_age=None):
        i = self.index_at(t)
        if i < 0:
            return None
        if max_age is not None and t - self.times[i] > max_age:
            return None
        return self.values[i]

    def time_of_value_at(self, t):
        i = self.index_at(t)
        return self.times[i] if i >= 0 else None

    def window(self, t0, t1):
        """Times and values in [t0, t1], plus the sample just before t0."""
        i0 = max(bisect.bisect_left(self.times, t0) - 1, 0)
        i1 = bisect.bisect_right(self.times, t1)
        return self.times[i0:i1], self.values[i0:i1]


@dataclass
class Event:
    id: int
    t: float
    severity: str
    category: str
    title: str
    detail: str = ''
    t_end: Optional[float] = None
    is_condition: bool = False
    x: Optional[float] = None
    y: Optional[float] = None
    count: int = 1
    key: str = ''
    extra: dict = field(default_factory=dict)

    @property
    def ongoing(self):
        return self.is_condition and self.t_end is None

    def active_at(self, t):
        if t < self.t:
            return False
        if self.is_condition:
            return self.t_end is None or t <= self.t_end
        return t - (self.t_end if self.t_end is not None else self.t) <= RECENT_WINDOW

    def end_or(self, now):
        return self.t_end if self.t_end is not None else now

    def to_dict(self):
        return {'t': round(self.t, 3), 't_end': None if self.t_end is None else round(self.t_end, 3),
                'severity': self.severity, 'category': self.category, 'title': self.title,
                'detail': self.detail, 'count': self.count, 'x': self.x, 'y': self.y}


class Timeline:
    """Tracks (messages), series (derived values), and events of one run."""

    def __init__(self, start_wall_ns=0):
        self.start_wall_ns = start_wall_ns
        self.tracks = {}
        self.series = {}
        self.events = []
        self.events_version = 0
        self.end_time = 0.0

    # ------------------------------------------------------------ tracks
    def add_message(self, t, topic, type_name, data):
        track = self.tracks.get(topic)
        if track is None:
            track = self.tracks[topic] = TopicTrack(topic, type_name)
        track.append(t, data)
        self.advance(t)
        return track

    def advance(self, t):
        if t > self.end_time:
            self.end_time = t

    @property
    def duration(self):
        return self.end_time

    # ------------------------------------------------------------ series
    def get_series(self, key, label=None, unit='', numeric=True, min_interval=0.0):
        s = self.series.get(key)
        if s is None:
            s = self.series[key] = Series(key, label or key, unit, numeric, min_interval)
        return s

    def value(self, key, t, default=None, max_age=None):
        s = self.series.get(key)
        if s is None:
            return default
        v = s.value_at(t, max_age)
        return default if v is None else v

    # ------------------------------------------------------------ events
    def add_event(self, event):
        event.id = len(self.events)
        self.events.append(event)
        self.events_version += 1
        return event

    def touch_events(self):
        self.events_version += 1

    def events_until(self, t):
        return [e for e in self.events if e.t <= t]

    def active_events(self, t):
        return [e for e in self.events if e.active_at(t)]

    def next_event_time(self, t, min_severity='warning'):
        floor = SEVERITIES.index(min_severity)
        for e in self.events:
            if e.t > t + 1e-6 and SEVERITIES.index(e.severity) >= floor:
                return e.t
        return None

    def prev_event_time(self, t, min_severity='warning'):
        floor = SEVERITIES.index(min_severity)
        for e in reversed(self.events):
            if e.t < t - 1e-6 and SEVERITIES.index(e.severity) >= floor:
                return e.t
        return None

    # ------------------------------------------------------------ snapshot
    def snapshot(self, t):
        """Values of all series and the active events at time t."""
        values = {key: s.value_at(t) for key, s in self.series.items()}
        return Snapshot(t, values, self.active_events(t), self)


class Snapshot:
    def __init__(self, t, values, events, timeline):
        self.t = t
        self.values = values
        self.events = events
        self.timeline = timeline

    def get(self, key, default=None):
        v = self.values.get(key)
        return default if v is None else v

    def age(self, key):
        s = self.timeline.series.get(key)
        if s is None:
            return None
        tv = s.time_of_value_at(self.t)
        return None if tv is None else self.t - tv
