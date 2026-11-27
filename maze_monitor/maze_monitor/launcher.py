"""Start and stop the simulation (ros2 launch) as a child process group.

No Qt and no ROS in here, so it can be tested on its own.

The child runs in its own session. Its process group then holds every process
the launch starts (Gazebo, SLAM, Nav2, RViz), so they can be stopped together,
and a Ctrl+C in the monitor's terminal does not reach them behind the
monitor's back. Stopping does what Ctrl+C in a terminal does: SIGINT to the
whole group, so ros2 launch shuts its nodes down in order. If the group is
still alive after a timeout, it gets SIGTERM, then SIGKILL.
"""

import collections
import os
import queue
import re
import signal
import subprocess
import threading
import time

IDLE, RUNNING, STOPPING, EXITED = 'idle', 'running', 'stopping', 'exited'

_ANSI = re.compile(r'\x1b\[[0-9;?]*[A-Za-z]')


class ManagedProcess:
    """One child process and everything it starts. Not thread-safe: use it from one thread."""

    def __init__(self, name='launch', stop_timeout=20.0, term_timeout=5.0):
        self.name = name
        self.stop_timeout = stop_timeout
        self.term_timeout = term_timeout
        self.lines = queue.SimpleQueue()      # output lines, for the GUI
        self.tail = collections.deque(maxlen=200)
        self.state = IDLE
        self.command = None
        self.returncode = None
        self.user_stopped = False   # stop() was called (not: the child ended by itself)
        self.started_at = None      # time.monotonic()
        self.ended_at = None
        self.log_path = None
        self._proc = None
        self._pgid = None
        self._reader = None
        self._stages = []           # remaining (signal, seconds to wait after it)
        self._deadline = None

    # ------------------------------------------------------------ queries
    @property
    def active(self):
        """True from start() until the whole process group has exited."""
        return self.state in (RUNNING, STOPPING)

    @property
    def pid(self):
        return self._proc.pid if self._proc is not None else None

    def elapsed(self):
        if self.started_at is None:
            return 0.0
        return (self.ended_at or time.monotonic()) - self.started_at

    def drain(self, limit=5000):
        """Output lines received since the last call."""
        out = []
        while len(out) < limit:
            try:
                out.append(self.lines.get_nowait())
            except queue.Empty:
                break
        return out

    # ------------------------------------------------------------ start
    def start(self, command, env=None, cwd=None, log_path=None):
        if self.active:
            raise RuntimeError(f'{self.name} is already running')
        env = dict(os.environ if env is None else env)
        env.setdefault('PYTHONUNBUFFERED', '1')
        env['RCUTILS_COLORIZED_OUTPUT'] = '0'
        log = None
        if log_path:
            os.makedirs(os.path.dirname(os.path.abspath(log_path)), exist_ok=True)
            log = open(log_path, 'a', buffering=1, errors='replace')
            log.write(f'$ {" ".join(command)}\n')
        try:
            self._proc = subprocess.Popen(
                command, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, env=env, cwd=cwd, start_new_session=True,
                text=True, errors='replace', bufsize=1)
        except OSError:
            if log is not None:
                log.close()
            raise
        self._pgid = self._proc.pid
        self.command = list(command)
        self.log_path = log_path
        self.returncode = None
        self.user_stopped = False
        self.started_at, self.ended_at = time.monotonic(), None
        self._stages, self._deadline = [], None
        self.tail.clear()
        self.state = RUNNING
        self._reader = threading.Thread(target=self._read, args=(self._proc.stdout, log),
                                        name=f'{self.name}-output', daemon=True)
        self._reader.start()

    def _read(self, stream, log):
        try:
            for line in stream:
                line = _ANSI.sub('', line.rstrip('\n'))
                self.tail.append(line)
                self.lines.put(line)
                if log is not None:
                    log.write(line + '\n')
        except (OSError, ValueError):
            pass
        finally:
            if log is not None:
                log.close()

    # ------------------------------------------------------------ stop
    def stop(self):
        """Ask the process group to stop (like Ctrl+C). Call poll() until it is EXITED."""
        if not self.active:
            return
        self.user_stopped = True
        if self.state != STOPPING:
            self._begin_stop()

    def kill(self):
        """Stop the process group at once (SIGKILL)."""
        if not self.active:
            return
        self.user_stopped = True
        self.state = STOPPING
        self._stages = []
        self._signal(signal.SIGKILL)
        self._deadline = None

    def _begin_stop(self):
        self.state = STOPPING
        self._stages = [(signal.SIGINT, self.stop_timeout), (signal.SIGTERM, self.term_timeout),
                        (signal.SIGKILL, None)]
        self._next_stage()

    def _next_stage(self):
        sig, wait = self._stages.pop(0)
        self._signal(sig)
        self._deadline = time.monotonic() + wait if wait is not None else None

    def _signal(self, sig):
        try:
            os.killpg(self._pgid, sig)
        except (ProcessLookupError, PermissionError):
            pass

    def _group_alive(self):
        try:
            os.killpg(self._pgid, 0)
            return True
        except ProcessLookupError:
            return False
        except PermissionError:
            return True

    # ------------------------------------------------------------ poll
    def poll(self):
        """Reap the child, advance a stop, and return the state. Call it regularly."""
        if self._proc is None or self.state == EXITED:
            return self.state
        if self.returncode is None:
            rc = self._proc.poll()
            if rc is not None:
                self.returncode = rc
        alive = self._group_alive()
        if self.returncode is not None and not alive:
            if self._reader is not None:
                self._reader.join(timeout=1.0)
            self.ended_at = time.monotonic()
            self.state = EXITED
            return self.state
        if self.returncode is not None and self.state == RUNNING:
            # The launch ended by itself but left processes behind (for example, a
            # Gazebo server after a crash). They would break the next start.
            self._begin_stop()
        if self.state == STOPPING and self._deadline is not None \
                and time.monotonic() >= self._deadline and self._stages:
            self._next_stage()
        return self.state

    def wait(self, timeout):
        """Poll until EXITED or timeout seconds pass. Returns True if it exited."""
        end = time.monotonic() + timeout
        while self.poll() != EXITED:
            if time.monotonic() >= end:
                return False
            time.sleep(0.05)
        return True

    def stop_and_wait(self, timeout=None):
        """Stop and block until the group is gone, force-killing it at the end of timeout."""
        if not self.active:
            return True
        self.stop()
        timeout = self.stop_timeout + self.term_timeout if timeout is None else timeout
        if self.wait(timeout):
            return True
        self.kill()
        return self.wait(3.0)


# ---------------------------------------------------------------- leftovers
LEFTOVER_PATTERNS = (r'(^|\s|/)gz sim(\s|$)', r'(^|/)ros2 launch maze_sim(\s|$)')


def find_leftovers(patterns=LEFTOVER_PATTERNS, exclude_pgids=()):
    """Processes of this user whose command line matches a pattern: [(pid, command)].

    Used before starting a run: a Gazebo server left over from an earlier run
    publishes its own /clock, /odom and /scan, and maze.launch.py refuses to start.
    """
    regexes = [re.compile(p) for p in patterns]
    me = os.getpid()
    uid = os.getuid()
    found = []
    try:
        entries = os.listdir('/proc')
    except OSError:
        return found
    for entry in entries:
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid == me:
            continue
        try:
            if os.stat(f'/proc/{pid}').st_uid != uid:
                continue
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                cmd = f.read().replace(b'\0', b' ').decode(errors='replace').strip()
            if exclude_pgids and os.getpgid(pid) in exclude_pgids:
                continue
        except (OSError, ProcessLookupError):
            continue
        # "ros2" is a Python script: drop the interpreter so the patterns see "ros2 launch".
        shown = re.sub(r'^\S*python3?(\.\d+)?\s+', '', cmd)
        if any(r.search(shown) for r in regexes):
            found.append((pid, shown))
    return sorted(found)


def stop_processes(pids, timeout=5.0):
    """SIGINT, then SIGKILL after timeout. Returns the pids that are still alive."""
    def alive(pid):
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        # A zombie counts as gone.
        try:
            with open(f'/proc/{pid}/stat') as f:
                return f.read().rsplit(')', 1)[1].split()[0] != 'Z'
        except OSError:
            return False

    for sig in (signal.SIGINT, signal.SIGKILL):
        for pid in pids:
            try:
                os.kill(pid, sig)
            except (ProcessLookupError, PermissionError):
                pass
        end = time.monotonic() + (timeout if sig == signal.SIGINT else 2.0)
        while time.monotonic() < end:
            pids = [p for p in pids if alive(p)]
            if not pids:
                return []
            time.sleep(0.1)
    return [p for p in pids if alive(p)]
