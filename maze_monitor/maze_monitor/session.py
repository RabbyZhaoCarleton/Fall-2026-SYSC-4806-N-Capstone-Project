"""A Session is a Timeline plus the Analyzer that fills it.

Live sessions are fed from the ROS bridge; replay sessions are fed from a
recording. Both feed messages in the same way, so they produce the same
timeline.
"""

from .analysis.analyzer import Analyzer, summarize
from .clock import now_ns
from .recorder import bag_message_count, read_bag
from .timeline import Timeline


class LoadCancelled(Exception):
    pass


class Session:
    def __init__(self, config, mode, run=None, start_ns=None):
        self.config = config
        self.mode = mode  # 'live' or 'replay'
        self.run = run
        self.start_ns = start_ns
        self.timeline = Timeline(start_ns or 0)
        self.analyzer = Analyzer(self.timeline, config)

    @property
    def title(self):
        return self.run.name if self.run is not None else 'Not recording'

    def time_of(self, t_ns):
        if self.start_ns is None:
            self.start_ns = t_ns
            self.timeline.start_wall_ns = t_ns
        return max((t_ns - self.start_ns) * 1e-9, 0.0)

    def ingest(self, topic, type_name, data, t_ns):
        t = self.time_of(t_ns)
        track = self.timeline.add_message(t, topic, type_name, data)
        self.analyzer.feed(t, track, len(track) - 1)

    def now(self):
        """Current run time in live mode."""
        if self.start_ns is None:
            return 0.0
        return max((now_ns() - self.start_ns) * 1e-9, 0.0)

    def advance_live(self, lag=0.2):
        """Run periodic detectors up to shortly before now (messages in flight arrive first)."""
        t = self.now() - lag
        if t > 0:
            self.analyzer.advance(t)

    def summary(self):
        return summarize(self.timeline)


def load_run(config, run, progress=None):
    """Build a replay Session from a saved run. progress(done, total) may return False to cancel."""
    session = Session(config, 'replay', run, start_ns=run.get('start_ns'))
    total = bag_message_count(run.bag_path)
    done = 0
    for topic, type_name, data, t_ns in read_bag(run.bag_path):
        session.ingest(topic, type_name, data, t_ns)
        done += 1
        if progress is not None and done % 1000 == 0:
            if progress(done, total) is False:
                raise LoadCancelled()
    session.analyzer.finish(session.timeline.duration)
    summary = session.summary()
    # Repair metadata of runs that did not end cleanly (crash, kill, power loss).
    if run.get('status') != 'complete' or not run.summary:
        try:
            run.update(summary=summary, status='complete (recovered)',
                       start_ns=session.start_ns)
        except OSError:
            pass
    return session
