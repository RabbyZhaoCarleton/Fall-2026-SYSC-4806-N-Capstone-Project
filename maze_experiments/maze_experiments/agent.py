"""The run agent (BR8, DR1): one per run, in the run's own ROS domain.

It starts the run's launch (maze.launch.py with monitor:=false), records the
run headlessly with maze_monitor's own RosBridge, Recorder and RunStore
format, and once per second appends the run's progress to status.jsonl:
readiness milestones, explore status, sim time, RTF, known map area and the
true position. The manager reads only these files, so it never joins a run's
ROS domain, and the files double as the run's log.

The agent is the leader of the run's process group. The manager stops a run
by sending SIGINT to the whole group, like Ctrl+C in a terminal: the launch
shuts the stack down, and the agent closes the recording once the launch has
exited.
"""

import datetime
import math
import os
import re
import signal
import subprocess
import sys
import time

from . import fileio

STATUS_FILE = 'status.jsonl'
SPEC_FILE = 'run_spec.json'
LAUNCH_LOG = 'launch.log'
DIED_RE = re.compile(r'\[ERROR\] \[([^\]]+)\]: process has died \[pid (\d+), exit code (-?\d+)')
# The launch is stopping everything (explore_lite ended, or a required process died):
# processes that die after this line die because of the shutdown.
SHUTDOWN_RE = re.compile(r'\[launch\]: .*(shutting down launched system|Sending signal|'
                         r'user interrupted)', re.I)
LAUNCH_EXIT_WAIT = 15.0   # s the agent waits for the launch after a stop, before closing


def iso_now():
    return datetime.datetime.now().isoformat(timespec='milliseconds')


class Progress:
    """What the agent knows about the run, from the messages it sees."""

    def __init__(self):
        self.sim = None
        self.clock_first = None
        self.clock_samples = []        # (monotonic, sim) over the last ~10 s
        self.clock_advancing = False
        self.scan_count = 0
        self.truth_count = 0
        self.pos = None
        self.yaw = None
        self.area = None
        self.statuses = []             # [(sim, status)]
        self.exits = None              # [[x, y], ...] latest /explore/exits centroids
        self.rtf = None

    def on_clock(self, mono, sim):
        if self.sim is not None and sim > self.sim:
            self.clock_advancing = True
        if self.clock_first is None:
            self.clock_first = sim
        self.sim = sim
        self.clock_samples.append((mono, sim))
        while self.clock_samples and mono - self.clock_samples[0][0] > 10.0:
            self.clock_samples.pop(0)
        if len(self.clock_samples) >= 2:
            (m0, s0), (m1, s1) = self.clock_samples[0], self.clock_samples[-1]
            if m1 - m0 >= 2.0:
                self.rtf = (s1 - s0) / (m1 - m0)

    @property
    def explore_started(self):
        return any(s == 'exploration_started' for _, s in self.statuses)

    def ready(self):
        return {'clock': self.clock_advancing, 'scan': self.scan_count > 0,
                'ground_truth': self.truth_count > 0, 'explore_started': self.explore_started}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Agent:
    def __init__(self, run_dir):
        self.run_dir = os.path.abspath(run_dir)
        self.spec = fileio.read_json(os.path.join(self.run_dir, SPEC_FILE))
        self.status_path = os.path.join(self.run_dir, STATUS_FILE)
        self.progress = Progress()
        self.stop_requested = None     # monotonic time of SIGINT/SIGTERM
        self.launch = None
        self.bridge = None
        self.recorder = None
        self.run_info = None
        self.died = []
        self.launch_shutdown = False
        self._log_offset = 0
        self._classes = {}
        self.t0 = time.monotonic()

    # ---------------------------------------------------------------- output
    def status(self, **fields):
        rec = {'t': round(time.monotonic() - self.t0, 3), 'time': iso_now(), **fields}
        fileio.append_jsonl(self.status_path, rec, kind='status')

    def tick_status(self, final=False):
        p = self.progress
        rec = {
            'sim': None if p.sim is None else round(p.sim - (p.clock_first or 0.0), 3),
            'sim_abs': p.sim,
            'rtf': None if p.rtf is None else round(p.rtf, 3),
            'ready': p.ready(),
            'explore': p.statuses[-1][1] if p.statuses else None,
            'area': None if p.area is None else round(p.area, 3),
            'pos': None if p.pos is None else [round(p.pos[0], 3), round(p.pos[1], 3)],
            'launch': 'running' if self.launch and self.launch.poll() is None else
                      ('exited' if self.launch else 'not started'),
            'launch_rc': None if not self.launch else self.launch.returncode,
            'msgs': self.recorder.messages if self.recorder else 0,
            'rec_bytes': self.recorder.bytes if self.recorder else 0,
            'stopping': self.stop_requested is not None,
        }
        if final:
            rec['final'] = True
            rec['statuses'] = [[round(s - (p.clock_first or 0.0), 3) if s is not None else None,
                                st] for s, st in p.statuses]
            rec['exits'] = p.exits
            rec['died'] = self.died
        self.status(**rec)

    # ---------------------------------------------------------------- start
    def start_recording(self):
        from maze_monitor.recorder import Recorder
        from maze_monitor.ros_bridge import RosBridge
        from maze_monitor.run_store import RunInfo
        from maze_monitor.clock import now_ns
        from .recording import monitor_config

        cfg = monitor_config(self.spec.get('recording_profile', 'full'))
        self.config = cfg
        start_ns = now_ns()
        start = datetime.datetime.fromtimestamp(start_ns / 1e9)
        # run.json in maze_monitor's run format (RunStore.create), so the run replays in
        # maze_monitor --replay; the suite's own fields go under "experiment".
        self.run_info = RunInfo(self.run_dir, {
            'format': 1,
            'schema_version': fileio.SCHEMA_VERSION,
            'name': self.spec['run_id'],
            'label': 'explore',
            'started': start.isoformat(timespec='seconds'),
            'ended': None,
            'status': 'recording',
            'launch_args': self.spec['launch_args'],
            'notes': '',
            'tags': ['maze_experiments'],
            'starred': False,
            'summary': {},
            'start_ns': start_ns,
            'experiment': {k: self.spec[k] for k in ('run_id', 'experiment_id', 'key',
                                                     'attempt', 'maze', 'domain', 'partition')
                           if k in self.spec},
        })
        self.run_info.save()
        self.recorder = Recorder(self.run_info.bag_path)
        self.bridge = RosBridge(cfg, ros_args=['maze_experiments_agent'])
        self.bridge.start()
        self.bridge.set_recorder(self.recorder)
        self.need_own_scan = cfg.is_excluded('/scan')
        if self.need_own_scan:
            self._subscribe_scan()

    def _subscribe_scan(self):
        """The metrics_only profile drops /scan; readiness still needs to see it publish."""
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
        node = self.bridge._node

        def on_scan(_msg):
            self.progress.scan_count += 1
        self._scan_sub = node.create_subscription(LaserScan, '/scan', on_scan,
                                                  qos_profile_sensor_data)

    def resolve(self, value):
        """Relative paths in the run spec are relative to the run folder (DR4)."""
        if isinstance(value, str) and (value.startswith('../') or value.startswith('./')):
            return os.path.normpath(os.path.join(self.run_dir, value))
        return value

    def start_launch(self):
        args = [f'{k}:={self.resolve(v)}' for k, v in self.spec['launch_args'].items()]
        cmd = ['ros2', 'launch', 'maze_sim', 'maze.launch.py'] + args
        log = open(os.path.join(self.run_dir, LAUNCH_LOG), 'ab')
        log.write(f'$ {" ".join(cmd)}\n'.encode())
        log.flush()
        # Same process group as the agent (no new session): a SIGINT to the group
        # reaches the launch like Ctrl+C in a terminal.
        self.launch = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=log,
                                       stderr=subprocess.STDOUT, cwd=self.run_dir)
        log.close()
        self.status(event='launch_started', pid=self.launch.pid, cmd=cmd)

    # ---------------------------------------------------------------- loop
    def _message_class(self, type_name):
        cls = self._classes.get(type_name)
        if cls is None:
            from rosidl_runtime_py.utilities import get_message
            cls = self._classes[type_name] = get_message(type_name)
        return cls

    def drain(self, budget=0.2):
        from rclpy.serialization import deserialize_message
        import queue
        p = self.progress
        end = time.monotonic() + budget
        while time.monotonic() < end:
            try:
                topic, type_name, data, _t_ns = self.bridge.queue.get_nowait()
            except queue.Empty:
                return
            if topic not in ('/clock', '/scan', '/ground_truth/poses', '/explore/status',
                             '/map', '/explore/exits'):
                continue
            try:
                msg = deserialize_message(data, self._message_class(type_name))
            except Exception:
                continue
            mono = time.monotonic()
            if topic == '/clock':
                p.on_clock(mono, msg.clock.sec + msg.clock.nanosec * 1e-9)
            elif topic == '/scan':
                p.scan_count += 1
            elif topic == '/ground_truth/poses':
                if msg.poses:
                    p.truth_count += 1
                    q = msg.poses[0]
                    p.pos = (q.position.x, q.position.y)
                    p.yaw = yaw_of(q.orientation)
            elif topic == '/explore/status':
                if not p.statuses or p.statuses[-1][1] != msg.status:
                    p.statuses.append((p.sim, msg.status))
                    self.status(event='explore_status', status=msg.status,
                                sim=None if p.sim is None else
                                round(p.sim - (p.clock_first or 0.0), 3))
            elif topic == '/map':
                import numpy as np
                known = int(np.count_nonzero(np.asarray(msg.data, dtype=np.int8) != -1))
                p.area = known * msg.info.resolution ** 2
            elif topic == '/explore/exits':
                p.exits = [[round(f.centroid.x, 3), round(f.centroid.y, 3)]
                           for f in msg.frontiers]

    def scan_launch_log(self):
        path = os.path.join(self.run_dir, LAUNCH_LOG)
        try:
            with open(path, 'rb') as f:
                f.seek(self._log_offset)
                data = f.read()
        except OSError:
            return
        end = data.rfind(b'\n')
        if end < 0:
            return
        self._log_offset += end + 1
        for line in data[:end].decode(errors='replace').splitlines():
            if not self.launch_shutdown and SHUTDOWN_RE.search(line):
                self.launch_shutdown = True
                self.status(event='launch_shutdown', line=line.strip()[:200])
            m = DIED_RE.search(line)
            if m:
                p = self.progress
                rec = {'process': m.group(1), 'pid': int(m.group(2)),
                       'exit_code': int(m.group(3)),
                       'sim': None if p.sim is None else round(p.sim - (p.clock_first or 0), 3),
                       'after_stop': self.stop_requested is not None or self.launch_shutdown}
                self.died.append(rec)
                self.status(event='process_died', **rec)

    def on_signal(self, signum, _frame):
        if self.stop_requested is None:
            self.stop_requested = time.monotonic()
            self.status(event='stop_requested', signal=signal.Signals(signum).name)

    def run(self):
        signal.signal(signal.SIGINT, self.on_signal)
        signal.signal(signal.SIGTERM, self.on_signal)
        self.status(event='agent_started', pid=os.getpid(), pgid=os.getpgrp(),
                    domain=os.environ.get('ROS_DOMAIN_ID'),
                    partition=os.environ.get('GZ_PARTITION'))
        try:
            self.start_recording()
        except Exception as exc:
            self.status(event='recording_failed', error=f'{type(exc).__name__}: {exc}')
            self.status(final=True, error='recording_failed')
            return 3
        if self.stop_requested is None:
            self.start_launch()
        next_tick = time.monotonic()
        while True:
            self.drain()
            now = time.monotonic()
            if now >= next_tick:
                self.scan_launch_log()
                self.tick_status()
                next_tick += 1.0
                if now > next_tick:
                    next_tick = now + 1.0
            if self.launch is None or self.launch.poll() is not None:
                break
            if self.stop_requested is not None and \
                    now - self.stop_requested > LAUNCH_EXIT_WAIT:
                self.status(event='launch_still_running', note='closing the recording anyway')
                break
            time.sleep(0.05)
        self.finish()
        return 0

    def finish(self):
        # Messages already received belong to the run: record them, then close.
        self.drain(budget=1.0)
        self.scan_launch_log()
        if self.bridge is not None:
            old = self.bridge.set_recorder(None)
            if old is not None:
                old.close()
        self.run_info.update(status='complete',
                             ended=datetime.datetime.now().isoformat(timespec='seconds'))
        self.tick_status(final=True)
        if self.bridge is not None:
            try:
                self.bridge.stop()
            except Exception:
                pass


def main(run_dir):
    agent = Agent(run_dir)
    try:
        rc = agent.run()
    except Exception as exc:
        import traceback
        try:
            agent.status(event='agent_error', error=f'{type(exc).__name__}: {exc}',
                         traceback=traceback.format_exc())
        except Exception:
            pass
        rc = 2
    sys.stdout.flush()
    # rclpy threads can keep the interpreter alive; the recording is closed by now.
    os._exit(rc)
