"""Receive-time clock for recordings.

The wall clock can step backward: WSL resyncs its clock with Windows about
every 30 seconds, moving it by up to a few seconds. Stamping messages with that
clock reorders them in the recording. This clock reads the wall time once and
then advances with the monotonic clock, so it never steps.
"""

import time

_WALL_AT_START = time.time_ns()
_MONO_AT_START = time.monotonic_ns()


def now_ns():
    return _WALL_AT_START + (time.monotonic_ns() - _MONO_AT_START)
