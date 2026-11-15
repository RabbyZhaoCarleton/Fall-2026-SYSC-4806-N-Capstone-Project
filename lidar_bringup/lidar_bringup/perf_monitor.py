"""High-resolution performance tracking for real-time loops (SLAM, navigation, motor control).

Pure Python (no ROS dependency) so it can be reused on the PC, the Pi 5, or in unit tests.

Two things are measured per loop / callback:
  * LATENCY  - how long one iteration's work takes (ms)          [start() -> stop()]
  * FREQUENCY - how often iterations START (Hz) and its jitter   [start() -> next start()]

All timing uses time.perf_counter(): monotonic, highest-resolution clock available,
immune to NTP / wall-clock jumps.

Typical use:
    tracker = PerfTracker("slam_loop", target_hz=10, deadline_ms=100)
    while running:
        tracker.start()
        do_work()
        tracker.stop()
    print(tracker.report())

or with a context manager / decorator:
    with tracker.measure():
        do_work()

    @timed(tracker)
    def do_work(): ...
"""
import math
import time
from collections import deque
from contextlib import contextmanager
from functools import wraps
from typing import Callable, Deque, Dict, Optional


def _percentile(sorted_vals, pct: float) -> float:
    """Nearest-rank percentile on an already-sorted list. Returns nan if empty."""
    if not sorted_vals:
        return math.nan
    k = max(0, min(len(sorted_vals) - 1, math.ceil(pct / 100.0 * len(sorted_vals)) - 1))
    return sorted_vals[k]


class PerfTracker:
    """Rolling-window latency + frequency tracker."""

    def __init__(self, name: str, window: int = 200,
                 target_hz: Optional[float] = None,
                 deadline_ms: Optional[float] = None):
        """
        name:        label used in reports
        window:      number of recent samples kept (rolling statistics)
        target_hz:   expected loop rate; used to flag a slow loop in report()
        deadline_ms: max acceptable per-iteration latency; overruns are counted.
                     Defaults to one full period (1000/target_hz) if target_hz is given.
        """
        self.name = name
        self.target_hz = target_hz
        self.deadline_ms = deadline_ms if deadline_ms is not None else (
            1000.0 / target_hz if target_hz else None)

        self._latencies_ms: Deque[float] = deque(maxlen=window)
        self._periods_s: Deque[float] = deque(maxlen=window)
        self._t_start: Optional[float] = None
        self._last_start: Optional[float] = None

        self.total_iterations = 0
        self.deadline_misses = 0

    # ------------------------------------------------------------------ timing
    def start(self) -> None:
        """Mark the beginning of one iteration (also records the loop period)."""
        now = time.perf_counter()
        if self._last_start is not None:
            self._periods_s.append(now - self._last_start)
        self._last_start = now
        self._t_start = now

    def stop(self) -> float:
        """Mark the end of one iteration. Returns this iteration's latency in ms."""
        if self._t_start is None:
            return 0.0
        latency_ms = (time.perf_counter() - self._t_start) * 1e3
        self._t_start = None
        self._latencies_ms.append(latency_ms)
        self.total_iterations += 1
        if self.deadline_ms is not None and latency_ms > self.deadline_ms:
            self.deadline_misses += 1
        return latency_ms

    @contextmanager
    def measure(self):
        """Context-manager form of start()/stop()."""
        self.start()
        try:
            yield
        finally:
            self.stop()

    def reset(self) -> None:
        self._latencies_ms.clear()
        self._periods_s.clear()
        self._t_start = self._last_start = None
        self.total_iterations = self.deadline_misses = 0

    # ------------------------------------------------------------------- stats
    def stats(self) -> Dict[str, float]:
        """Snapshot of rolling statistics (latency in ms, frequency in Hz)."""
        lat = sorted(self._latencies_ms)
        per = list(self._periods_s)
        mean_period = (sum(per) / len(per)) if per else math.nan
        jitter_ms = math.nan
        if len(per) > 1:
            m = mean_period
            jitter_ms = math.sqrt(sum((p - m) ** 2 for p in per) / (len(per) - 1)) * 1e3
        return {
            "iterations": self.total_iterations,
            "lat_mean_ms": (sum(lat) / len(lat)) if lat else math.nan,
            "lat_p50_ms": _percentile(lat, 50),
            "lat_p95_ms": _percentile(lat, 95),
            "lat_p99_ms": _percentile(lat, 99),
            "lat_max_ms": lat[-1] if lat else math.nan,
            "freq_hz": (1.0 / mean_period) if per and mean_period > 0 else math.nan,
            "period_jitter_ms": jitter_ms,
            "deadline_misses": self.deadline_misses,
        }

    def is_healthy(self, hz_tolerance: float = 0.9) -> bool:
        """True if loop rate >= tolerance * target_hz and p95 latency is within deadline."""
        s = self.stats()
        if math.isnan(s["freq_hz"]):
            return False
        if self.target_hz and s["freq_hz"] < hz_tolerance * self.target_hz:
            return False
        if self.deadline_ms and s["lat_p95_ms"] > self.deadline_ms:
            return False
        return True

    def report(self) -> str:
        s = self.stats()
        target = f" (target {self.target_hz:.1f} Hz)" if self.target_hz else ""
        flag = "" if self.is_healthy() else "  <-- BELOW TARGET"
        return (f"[{self.name}] {s['freq_hz']:.2f} Hz{target} | "
                f"lat mean {s['lat_mean_ms']:.3f} / p95 {s['lat_p95_ms']:.3f} / "
                f"max {s['lat_max_ms']:.3f} ms | jitter {s['period_jitter_ms']:.3f} ms | "
                f"deadline misses {s['deadline_misses']}/{s['iterations']}{flag}")


def timed(tracker: PerfTracker) -> Callable:
    """Decorator: time every call of the wrapped function with `tracker`."""
    def deco(fn):
        @wraps(fn)
        def wrapper(*a, **kw):
            with tracker.measure():
                return fn(*a, **kw)
        return wrapper
    return deco


class FixedRateLoop:
    """Run a function at a fixed rate with drift-free scheduling.

    Sleeps until the *absolute* next deadline (not "period minus work"), so errors do
    not accumulate. If an iteration overruns, the schedule re-syncs instead of trying
    to "catch up" with a burst of back-to-back iterations.
    """

    def __init__(self, hz: float, tracker: Optional[PerfTracker] = None):
        self.period = 1.0 / hz
        self.tracker = tracker or PerfTracker("fixed_loop", target_hz=hz)
        self.overruns = 0

    def run(self, fn: Callable[[], None], duration_s: Optional[float] = None,
            max_iters: Optional[int] = None) -> None:
        t_end = time.perf_counter() + duration_s if duration_s else None
        next_t = time.perf_counter()
        i = 0
        while True:
            self.tracker.start()
            fn()
            self.tracker.stop()
            i += 1
            if (max_iters and i >= max_iters) or (t_end and time.perf_counter() >= t_end):
                break
            next_t += self.period
            slack = next_t - time.perf_counter()
            if slack > 0:
                time.sleep(slack)
            else:                       # overrun: resync the schedule
                self.overruns += 1
                next_t = time.perf_counter()
