"""Module B (with Module C's controls): the experiment manager.

One background process per running experiment. Every second it:
  1. reads pause / resume / cancel requests (control.jsonl);
  2. checks for a host suspend or a wall-clock jump (BR13);
  3. samples the machine and every run (CR8) and protects the machine (CR7);
  4. reads each run's status.jsonl and decides whether the run must stop and
     with which outcome (BR3, BR5, BR6, BR7, BR9);
  5. stops runs: SIGINT to the run's process group, SIGTERM after 20 s, then
     SIGKILL, and confirms that no process is left (BR2, BR4);
  6. processes finished runs (ER4) and starts new ones while the allocation and
     the RTF limit allow (CR4, CR5).
All durations are measured with the monotonic clock (BNR4). The manager never
joins a run's ROS domain: it reads only files (BR8).
"""

import datetime
import os
import signal
import subprocess
import sys
import time
import traceback

from . import (agent as agent_mod, experiment as exp_mod, fileio, ids, leases, profiles,
               resources)
from .experiment import EXCLUDED_OUTCOMES, is_robot_result
from .manager_control import ManagerLock

TICK = 1.0
STOP_SIGINT_WAIT = 20.0      # BR2: wait for the recording to close
STOP_SIGTERM_WAIT = 5.0
STOP_SIGKILL_WAIT = 5.0
SUSPEND_GAP = 30.0           # BR13
CLOCK_JUMP_STEP = 2.0        # s difference between wall and monotonic time over one tick
JOURNAL_EVERY = 10.0
LIVE_EVERY = 2.0
CALIBRATION_MIN_SIM = 60.0   # s of sim time a run needs to calibrate from
UNCOUNTED_REASONS = ('manager_stopped', 'cancelled', 'manager_crashed')  # BR12: no retry used


def iso(t=None):
    return (datetime.datetime.fromtimestamp(t) if t else datetime.datetime.now()) \
        .isoformat(timespec='seconds')


class Slot:
    """One active run attempt."""

    def __init__(self, planned, attempt):
        self.planned = planned
        self.attempt = attempt
        self.run_id = planned.run_id(attempt)
        self.dir = None
        self.domain = None
        self.partition = None
        self.proc = None
        self.pgid = None
        self.leader_start = None
        self.marker = None
        self.started = time.monotonic()
        self.started_wall = time.time()
        self.ready_at = None
        self.offset = 0
        self.last = {}
        self.statuses = []
        self.died = []
        self.final = None
        self.rtfs = []
        self.low_rtf_seconds = 0.0
        self.stall_ref = None          # (sim, area, pos)
        self.outcome = None
        self.reason = ''
        self.stop_at = None
        self.stage = 0                 # 0 running, 1 SIGINT, 2 SIGTERM, 3 SIGKILL, 4 leftover
        self.stage_at = None
        self.sampler = None
        self.cpu_samples = []
        self.cpus = None
        self.memory_max = None
        self.slot_index = 0
        self.calibration = False
        self.concurrency_at_start = 1
        self.flags = []
        self.leftover_reported = False

    @property
    def sim(self):
        return self.last.get('sim')

    @property
    def rtf(self):
        return self.last.get('rtf')


class Manager:
    def __init__(self, exp, data_dir, resume=False):
        self.exp = exp
        self.data_dir = data_dir
        self.resume = resume
        self.lock = ManagerLock(exp)
        self.active = {}            # run id -> Slot (running or stopping)
        self.leftover = {}          # run id -> Slot whose processes survived SIGKILL
        self.queue = []             # [(PlannedRun, attempt)]
        self.processing = []        # run dirs waiting for metrics
        self.proc_job = None        # (Popen, run dir, started)
        self.paused = False
        self.cancelling = False
        self.stopping_manager = False
        self.control_offset = 0
        self.machine_sampler = resources.MachineSampler()
        self.messages = []
        self._last_tick = None
        self._last_journal = time.time()
        self._last_live = 0.0
        self._last_summary = 0.0
        self.warned_low_rtf_single = False
        self.disk_paused = False

    # ================================================================ helpers
    def log(self, event, **fields):
        return self.exp.log(event, **fields)

    def say(self, level, text):
        self.messages.append({'time': iso(), 'level': level, 'text': text})
        self.messages = self.messages[-20:]
        self.log('message', level=level, text=text)

    @property
    def limits(self):
        return self.exp.config.data['run_limits']

    # ================================================================ setup
    def setup(self):
        cfg = self.exp.config.data
        self.profile = profiles.load(cfg['robot_profile'], self.data_dir)
        ex = self.exp.execution
        self.machine = ex.get('machine') or resources.measure_machine(self.data_dir)
        self.min_rtf = float(self.limits.get('min_rtf', 0.8))
        self.retries = int(self.limits.get('retries', 2))
        cpus_all = self.machine.get('cpu_list') or resources.allowed_cpus()
        n = int(ex.get('cpus') or max(1, len(cpus_all) // 2))
        # The highest-numbered CPUs; CPU 0 tends to carry the desktop and interrupts.
        self.cpu_list = sorted(cpus_all)[-n:] if cpus_all else []
        self.memory_alloc = int(ex.get('memory_bytes') or 0)
        self.disk_alloc = int(ex.get('disk_bytes') or 0)
        self.calibration = None
        if not ex.get('recalibrate'):
            self.calibration = resources.load_calibration(self.data_dir, self.machine,
                                                          self.profile.name)
        self.controller = resources.ConcurrencyController(self.min_rtf, cap=1)
        self.update_cap()

    def update_cap(self):
        cap = resources.concurrency_cap(self.exp.execution, self.calibration,
                                        leases.free_count() + len(self.active))
        if self.calibration is None:
            cap = 1
        failed = [lvl for lvl, n in getattr(self, 'controller', None).failures.items()
                  if n >= self.controller.max_failures] if hasattr(self, 'controller') else []
        if failed:
            cap = min(cap, min(failed) - 1)
        if cap != self.controller.cap:
            self.log('resources', change='cap', cap=cap,
                     calibration=bool(self.calibration))
        self.controller.cap = cap
        self.controller.target = min(self.controller.target, cap)

    # ================================================================ generation
    def generate_mazes(self):
        from .mazegen import generate_set, MazeParams
        self.exp.set_state('generating')
        for k, s in enumerate(self.exp.sets()):
            out = self.exp.p('mazes', exp_mod.set_folder(k))
            have = all(os.path.isfile(os.path.join(self.exp.maze_dir(k, i), 'maze.json'))
                       for i in range(int(s['mazes'])))
            if have:
                continue
            params = MazeParams.from_dict(s['maze'])
            self.log('generating', set=k, name=s['name'], mazes=int(s['mazes']),
                     seed=int(s['seed']))
            generate_set(params, int(s['seed']), int(s['mazes']), out, self.profile,
                         set_name=s['name'],
                         log=lambda text, k=k: self.log('maze_discarded', set=k, detail=text))
        self.log('generated', sets=len(self.exp.sets()))

    # ================================================================ queue / recovery
    def recover(self):
        """BR12: clean up after a manager, WSL or PC that stopped during the experiment."""
        for rid in self.exp.run_ids():
            run_dir = self.exp.run_dir(rid)
            rec = exp_mod.read_run_record(run_dir)
            if rec.get('outcome'):
                if not rec['has_metrics'] and rec.get('recording') != 'deleted':
                    self.processing.append(run_dir)
                continue
            # An attempt without an outcome was active when the manager stopped.
            killed = self.kill_leftovers(run_dir)
            self.finalize_record(run_dir, rid, 'interrupted', 'manager_crashed',
                                 extra={'leftover_processes_killed': killed})
            self.log('outcome', run=rid, outcome='interrupted', reason='manager_crashed',
                     leftover_processes_killed=killed)
            self.processing.append(run_dir)

    def kill_leftovers(self, run_dir):
        """Stop a dead manager's run processes, never an unrelated process (BR12)."""
        try:
            spec = fileio.read_json(os.path.join(run_dir, agent_mod.SPEC_FILE))
        except (OSError, ValueError):
            return 0
        if spec.get('boot_id') and spec['boot_id'] != resources.boot_id():
            return 0   # rebooted since: nothing of it can be alive
        marker = f'{leases.ENV_MARKER}={spec["experiment_id"]}/{spec["run_id"]}'.encode()
        start = spec.get('leader_start')
        pids = self.marked_processes(marker, min_start=start)
        for sig, wait in ((signal.SIGINT, STOP_SIGINT_WAIT), (signal.SIGTERM, STOP_SIGTERM_WAIT),
                          (signal.SIGKILL, STOP_SIGKILL_WAIT)):
            pids = self.marked_processes(marker, min_start=start)
            if not pids:
                break
            for pid in pids:
                try:
                    os.kill(pid, sig)
                except OSError:
                    pass
            deadline = time.monotonic() + wait
            while time.monotonic() < deadline and self.marked_processes(marker, min_start=start):
                time.sleep(0.5)
        left = self.marked_processes(marker, min_start=start)
        if left:
            self.log('error', run=spec['run_id'], text='leftover processes after SIGKILL',
                     pids=left)
        if spec.get('domain') and not left:
            leases.release(spec['domain'], f'{spec["experiment_id"]}/{spec["run_id"]}')
        return len(pids)

    @staticmethod
    def marked_processes(marker, min_start=None):
        out = []
        for entry in os.listdir('/proc'):
            if not entry.isdigit() or int(entry) == os.getpid():
                continue
            st = resources.proc_stat(entry)
            if st is None or st[4] == 'Z':
                continue
            if min_start is not None and st[1] < min_start:
                continue
            if resources.process_env_has(entry, marker):
                out.append(int(entry))
        return out

    def build_queue(self):
        attempts = self.exp.attempts()
        self.queue = []
        for planned in self.exp.plan():
            recs = attempts.get(planned.key, [])
            if any(is_robot_result(r.get('outcome')) for r in recs):
                continue
            used = sum(1 for r in recs if r.get('outcome') in EXCLUDED_OUTCOMES
                       and r.get('reason') not in UNCOUNTED_REASONS)
            if used > self.retries:
                continue
            nxt = max((r['attempt'] for r in recs), default=0) + 1
            self.queue.append((planned, nxt))

    # ================================================================ control
    def read_control(self):
        recs, self.control_offset = fileio.read_jsonl_from(self.exp.control_file,
                                                           self.control_offset)
        for rec in recs:
            cmd = rec.get('command')
            self.log('control', command=cmd)
            if cmd == 'pause' and not self.cancelling:
                self.paused = True
                self.exp.set_state('paused')
            elif cmd == 'resume' and not self.cancelling:
                self.paused = False
                self.disk_paused = False
                self.exp.set_state('calibrating' if self.calibration is None else 'running')
            elif cmd == 'cancel':
                self.cancelling = True
                for slot in list(self.active.values()):
                    if slot.outcome is None:
                        self.stop(slot, 'interrupted', 'cancelled')

    # ================================================================ runs
    def start_run(self, planned, attempt):
        slot = Slot(planned, attempt)
        exp = self.exp
        key = f'{exp.id}/{slot.run_id}'
        lease = leases.acquire(key, owner_pid=os.getpid())
        if lease is None:
            self.say('warning', 'No free ROS domain ID (1-101); waiting for one.')
            return None
        slot.domain, slot.partition = lease
        slot.dir = exp.run_dir(slot.run_id)
        os.makedirs(slot.dir, exist_ok=True)
        sets = exp.sets()
        set_ = sets[planned.set_index]
        maze_dir = exp.maze_dir(planned.set_index, planned.maze_index)
        meta = fileio.read_json(os.path.join(maze_dir, 'maze.json'))
        launch_args = {
            'explore': 'true', 'shutdown_when_done': 'true', 'gui': 'false', 'rviz': 'false',
            'monitor': 'false', 'gz_server_check': 'partition',
            'world': os.path.join(maze_dir, 'world.sdf'),
            'x_pose': f'{meta["spawn"]["x"]:.4f}', 'y_pose': f'{meta["spawn"]["y"]:.4f}',
        }
        if self.profile.data.get('bridge_config'):
            # The profile names its bridge config (FR1); for the Burger it is the launch
            # file's own default, so the launch behaves exactly as by hand.
            from .paths import resolve_package_path
            launch_args['robot_bridge_config'] = resolve_package_path(
                self.profile.data['bridge_config'])
        launch_args.update(self.profile.launch_args)
        if self.profile.data.get('spawn', {}).get('z') is not None:
            launch_args['robot_spawn_z'] = str(self.profile.data['spawn']['z'])
        params = self.profile.stack_params()
        params.update(exp.config.stack_param_files(set_))
        launch_args.update(params)
        used = {s.slot_index for s in self.active.values()}
        slot.slot_index = next(i for i in range(len(self.active) + 1) if i not in used)
        cap = max(self.controller.cap, 1)
        slot.cpus = resources.assign_cpus(self.cpu_list, slot.slot_index, cap)
        if self.calibration and self.memory_alloc:
            peak = self.calibration.get('peak_memory_bytes') or 0
            slot.memory_max = int(max(self.memory_alloc / cap, peak * 1.5))
        else:
            slot.memory_max = self.memory_alloc or None
        # Paths inside the experiment folder are stored relative to the run folder, so a
        # copied experiment stays self-contained (DR4); the agent resolves them.
        for k, v in list(launch_args.items()):
            if isinstance(v, str) and os.path.isabs(v) and \
                    os.path.commonpath([v, exp.path]) == exp.path:
                launch_args[k] = os.path.relpath(v, slot.dir)
        prefix, confinement = resources.launch_prefix(slot.cpus, slot.memory_max)
        slot.marker = f'{leases.ENV_MARKER}={key}'.encode()
        spec = {
            'run_id': slot.run_id, 'experiment_id': exp.id, 'key': list(planned.key),
            'attempt': attempt, 'set_name': set_['name'],
            'maze': os.path.relpath(maze_dir, slot.dir), 'launch_args': launch_args,
            'recording_profile': exp.config.data['recording']['profile'],
            'domain': slot.domain, 'partition': slot.partition, 'confinement': confinement,
            'boot_id': resources.boot_id(),
        }
        fileio.write_json(os.path.join(slot.dir, agent_mod.SPEC_FILE), spec)
        env = dict(os.environ)
        env.pop('ROS_LOCALHOST_ONLY', None)
        env.update(self.profile.env)
        env.update({
            'ROS_DOMAIN_ID': str(slot.domain),
            'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
            'GZ_PARTITION': slot.partition,
            'GZ_IP': '127.0.0.1',
            leases.ENV_MARKER: key,
            'PYTHONUNBUFFERED': '1',
        })
        cmd = prefix + [sys.executable, '-m', 'maze_experiments.main', '--data-dir',
                        self.data_dir, '_agent', slot.dir]
        out = open(os.path.join(slot.dir, 'agent.log'), 'ab')
        try:
            slot.proc = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=out,
                                         stderr=subprocess.STDOUT, env=env, cwd=slot.dir,
                                         start_new_session=True)
        except OSError as exc:
            out.close()
            leases.release(slot.domain, key)
            self.say('error', f'Could not start run {slot.run_id}: {exc}')
            return None
        out.close()
        slot.pgid = slot.proc.pid
        st = resources.proc_stat(slot.proc.pid)
        slot.leader_start = st[1] if st else None
        spec['pgid'], spec['leader_start'] = slot.pgid, slot.leader_start
        fileio.write_json(os.path.join(slot.dir, agent_mod.SPEC_FILE), spec)
        leases.attach(slot.domain, slot.pgid, slot.leader_start)
        slot.sampler = resources.RunSampler(slot.pgid, slot.proc.pid)
        slot.calibration = self.calibration is None
        slot.concurrency_at_start = len(self.active) + 1
        self.active[slot.run_id] = slot
        self.log('started', run=slot.run_id, key=list(planned.key), attempt=attempt,
                 domain=slot.domain, partition=slot.partition, pgid=slot.pgid,
                 cpus=slot.cpus, memory_max=slot.memory_max, confinement=confinement,
                 calibration=slot.calibration, concurrency=slot.concurrency_at_start)
        return slot

    def read_status(self, slot):
        recs, slot.offset = fileio.read_jsonl_from(os.path.join(slot.dir, agent_mod.STATUS_FILE),
                                                   slot.offset)
        for rec in recs:
            ev = rec.get('event')
            if ev == 'explore_status':
                slot.statuses.append(rec.get('status'))
                self.log('explore_status', run=slot.run_id, status=rec.get('status'),
                         sim=rec.get('sim'))
            elif ev == 'process_died':
                slot.died.append(rec)
            elif ev in ('recording_failed', 'agent_error'):
                slot.flags.append(ev)
                self.log('error', run=slot.run_id, text=ev, detail=rec.get('error'))
            elif ev is None:
                slot.last = rec
                if rec.get('final'):
                    slot.final = rec
                if slot.ready_at is not None and rec.get('rtf') is not None \
                        and not rec.get('stopping'):
                    slot.rtfs.append(rec['rtf'])

    def evaluate(self, slot, now):
        """Decide whether a running run must stop, and why (BR3, BR5, BR6, BR7, BR9)."""
        lim = self.limits
        last = slot.last
        wall = now - slot.started
        agent_gone = slot.proc.poll() is not None
        if slot.ready_at is None and last.get('ready') and all(last['ready'].values()):
            slot.ready_at = now
            slot.stall_ref = (last.get('sim') or 0.0, last.get('area') or 0.0, last.get('pos'))
            self.log('ready', run=slot.run_id, after_s=round(wall, 1), sim=last.get('sim'))
        if slot.final is not None or agent_gone or last.get('launch') == 'exited':
            if slot.final is None and not agent_gone:
                return None   # launch ended; the agent is closing the recording
            return self.outcome_from_statuses(slot, agent_gone and slot.final is None)
        if slot.ready_at is None:
            if wall > float(lim['startup_timeout_s']):
                missing = [k for k, v in (last.get('ready') or {}).items() if not v] or ['all']
                return 'startup_failed', 'not ready: ' + ', '.join(missing)
            return None
        sim = last.get('sim') or 0.0
        if sim >= float(lim['sim_timeout_s']):
            return 'timeout', f'{sim:.0f} s of sim time'
        wall_limit = float(lim['startup_timeout_s']) + float(lim['sim_timeout_s']) / self.min_rtf
        if wall > wall_limit:
            return 'sim_frozen', f'{wall:.0f} s wall time with {sim:.0f} s of sim time'
        died = [d for d in slot.died if not d.get('after_stop')]
        if died and not any(st in ('returned_to_origin', 'return_failed')
                            for st in slot.statuses):
            d = died[0]
            return 'stack_crash', f'{d["process"]} exited with code {d["exit_code"]}'
        # BR7: no progress in map area or true position for the window
        ref_sim, ref_area, ref_pos = slot.stall_ref
        area, pos = last.get('area') or 0.0, last.get('pos')
        moved = (pos is not None and ref_pos is not None and
                 ((pos[0] - ref_pos[0]) ** 2 + (pos[1] - ref_pos[1]) ** 2) ** 0.5
                 >= float(lim['stall_min_move_m']))
        if area - ref_area >= float(lim['stall_min_area_m2']) or moved \
                or (ref_pos is None and pos is not None):
            slot.stall_ref = (sim, area, pos)
        elif sim - ref_sim >= float(lim['stall_window_s']):
            return 'stalled', (f'no map growth of {lim["stall_min_area_m2"]} m² and no '
                               f'movement of {lim["stall_min_move_m"]} m in '
                               f'{lim["stall_window_s"]:.0f} s of sim time')
        if slot.memory_max and not (self.machine.get('systemd_scope')) and \
                slot.sampler.memory > slot.memory_max * 1.2:
            return 'infra_failure', 'memory_limit'
        return None

    def outcome_from_statuses(self, slot, agent_crashed):
        statuses = slot.statuses or (slot.final or {}).get('statuses') or []
        names = [s[1] if isinstance(s, list) else s for s in statuses]
        if agent_crashed:
            return 'infra_failure', 'agent_exited'
        if 'exploration_incomplete' in names:
            return 'incomplete', 'exploration_incomplete' + (
                ', returned to the start' if 'returned_to_origin' in names else
                ', return failed' if 'return_failed' in names else '')
        if 'exploration_complete' in names:
            if 'returned_to_origin' in names:
                return 'complete', 'exploration_complete, returned_to_origin'
            if 'return_failed' in names:
                return 'return_failed', 'exploration_complete, return_failed'
            return 'stack_crash', 'explore_lite ended without a final return status'
        if slot.ready_at is not None:
            died = [d for d in slot.died if not d.get('after_stop')]
            why = f'{died[0]["process"]} exited with code {died[0]["exit_code"]}' if died else \
                'the launch ended without a final explore status'
            return 'stack_crash', why
        return 'startup_failed', 'the launch ended before the run was ready'

    def stop(self, slot, outcome, reason):
        if slot.outcome is not None:
            return
        slot.outcome, slot.reason = outcome, reason
        slot.stop_at = time.monotonic()
        self.log('stopping', run=slot.run_id, outcome=outcome, reason=reason,
                 sim=slot.sim)
        self.signal(slot, signal.SIGINT)
        slot.stage, slot.stage_at = 1, slot.stop_at

    def signal(self, slot, sig):
        try:
            os.killpg(slot.pgid, sig)
        except ProcessLookupError:
            pass
        except OSError as exc:
            self.log('error', run=slot.run_id, text=f'killpg failed: {exc}')
        # Processes that left the group (they keep the run's environment marker)
        for pid in self.marked_processes(slot.marker):
            try:
                if os.getpgid(pid) != slot.pgid:
                    os.kill(pid, sig)
            except OSError:
                pass

    def remaining(self, slot):
        if slot.proc.poll() is None:
            return [slot.proc.pid]
        return self.marked_processes(slot.marker, min_start=slot.leader_start)

    def advance_stop(self, slot, now):
        left = self.remaining(slot)
        if not left:
            return True
        if slot.stage == 1 and now - slot.stage_at >= STOP_SIGINT_WAIT:
            self.log('escalate', run=slot.run_id, signal='SIGTERM', processes=len(left))
            self.signal(slot, signal.SIGTERM)
            slot.stage, slot.stage_at = 2, now
        elif slot.stage == 2 and now - slot.stage_at >= STOP_SIGTERM_WAIT:
            self.log('escalate', run=slot.run_id, signal='SIGKILL', processes=len(left))
            self.signal(slot, signal.SIGKILL)
            slot.stage, slot.stage_at = 3, now
        elif slot.stage == 3 and now - slot.stage_at >= STOP_SIGKILL_WAIT \
                and not slot.leftover_reported:
            slot.leftover_reported = True
            self.log('error', run=slot.run_id, text='processes left after SIGKILL; its domain '
                     'ID and partition stay reserved until they are gone', pids=left)
            self.say('error', f'Run {slot.run_id}: {len(left)} process(es) survived SIGKILL.')
        return False

    # ================================================================ finishing a run
    def bag_state(self, run_dir):
        bag = os.path.join(run_dir, 'bag')
        if os.path.isfile(os.path.join(bag, 'metadata.yaml')):
            return 'closed'
        if not any(f.endswith('.db3') for f in os.listdir(bag)) if os.path.isdir(bag) else True:
            return 'missing'
        try:
            r = subprocess.run(['ros2', 'bag', 'reindex', bag, '-s', 'sqlite3'],
                               capture_output=True, text=True, timeout=120)
            if r.returncode == 0 and os.path.isfile(os.path.join(bag, 'metadata.yaml')):
                return 'reindexed'
        except (OSError, subprocess.SubprocessError):
            pass
        return 'unreadable'

    def finalize_record(self, run_dir, run_id, outcome, reason, extra=None):
        from maze_monitor.run_store import RunStore
        info = RunStore(os.path.dirname(run_dir)).load(run_dir)
        if not info.meta:
            parsed = ids.parse_run_id(run_id)
            info.meta = {'format': 1, 'name': run_id, 'label': 'explore', 'status': 'no recording',
                         'summary': {}}
            if parsed:
                info.meta['experiment'] = {'key': list(parsed[:3]), 'attempt': parsed[3]}
        block = dict(info.meta.get('experiment') or {})
        parsed = ids.parse_run_id(run_id)
        block.update({'run_id': run_id, 'outcome': outcome, 'reason': reason,
                      'counts_as_robot_result': is_robot_result(outcome),
                      'finalized': iso()})
        if parsed:
            block.setdefault('key', list(parsed[:3]))
            block.setdefault('attempt', parsed[3])
        block.update(extra or {})
        info.meta['schema_version'] = fileio.SCHEMA_VERSION
        info.meta['experiment'] = block
        if info.meta.get('status') == 'recording':
            info.meta['status'] = 'interrupted'
        info.save()

    def finish(self, slot):
        rec_state = self.bag_state(slot.dir)
        outcome, reason = slot.outcome, slot.reason
        if rec_state in ('missing', 'unreadable') and is_robot_result(outcome):
            outcome, reason = 'infra_failure', f'recording {rec_state}'
        sim = (slot.final or slot.last).get('sim')
        wall = (slot.stop_at or time.monotonic()) - slot.started
        cpu = [c for c in slot.cpu_samples if c is not None]
        extra = {
            'set_name': self.exp.sets()[slot.planned.set_index]['name'],
            'started': iso(slot.started_wall), 'ended': iso(),
            'wall_s': round(wall, 1), 'sim_s': sim,
            'ready_after_s': None if slot.ready_at is None else
            round(slot.ready_at - slot.started, 1),
            'rtf_mean': round(sum(slot.rtfs) / len(slot.rtfs), 3) if slot.rtfs else None,
            'rtf_min': round(min(slot.rtfs), 3) if slot.rtfs else None,
            'low_rtf': bool(slot.rtfs) and min(slot.rtfs) < self.min_rtf,
            'min_rtf_limit': self.min_rtf,
            'explore_statuses': slot.statuses,
            'died': slot.died, 'flags': slot.flags,
            'recording': rec_state,
            'resources': {'cpu_avg_cores': round(sum(cpu) / len(cpu), 3) if cpu else None,
                          'cpu_peak_cores': round(max(cpu), 3) if cpu else None,
                          'memory_peak_bytes': slot.sampler.peak_memory if slot.sampler else None,
                          'cpus': slot.cpus, 'memory_max': slot.memory_max},
            'concurrency_at_start': slot.concurrency_at_start,
            'calibration_run': slot.calibration,
            'domain': slot.domain, 'partition': slot.partition, 'pgid': slot.pgid,
        }
        self.finalize_record(slot.dir, slot.run_id, outcome, reason, extra)
        leases.release(slot.domain, f'{self.exp.id}/{slot.run_id}')
        shm = resources.fastdds_shm_clean()
        self.log('stopped', run=slot.run_id, outcome=outcome, reason=reason, sim=sim,
                 wall=round(wall, 1), recording=rec_state, shm_clean=shm)
        self.log('outcome', run=slot.run_id, outcome=outcome, reason=reason)
        if slot.calibration:
            self.calibrate(slot, sim)
        if not is_robot_result(outcome) and reason not in UNCOUNTED_REASONS \
                and not self.cancelling and not self.stopping_manager:
            used = sum(1 for r in self.exp.attempts().get(slot.planned.key, [])
                       if r.get('outcome') in EXCLUDED_OUTCOMES
                       and r.get('reason') not in UNCOUNTED_REASONS)
            if used <= self.retries:
                self.queue.insert(0, (slot.planned, slot.attempt + 1))
                self.log('retried', run=slot.run_id, next_attempt=slot.attempt + 1,
                         used=used, allowed=self.retries)
            else:
                self.log('retries_exhausted', run=slot.run_id, used=used)
        elif outcome == 'interrupted' and not self.cancelling and not self.stopping_manager:
            self.queue.insert(0, (slot.planned, slot.attempt + 1))
        self.processing.append(slot.dir)

    def calibrate(self, slot, sim):
        """CR3: measure the calibration run's cost."""
        cpu = [c for c in slot.cpu_samples if c is not None]
        if not cpu or not sim or sim < CALIBRATION_MIN_SIM or slot.ready_at is None:
            self.log('calibration', result='not enough data; the next run calibrates',
                     sim=sim)
            return
        disk = resources.dir_size(os.path.join(slot.dir, 'bag'))
        wall = (slot.stop_at or time.monotonic()) - slot.started
        cal = {
            'avg_cpu_cores': round(sum(cpu) / len(cpu), 3),
            'peak_cpu_cores': round(max(cpu), 3),
            'peak_memory_bytes': slot.sampler.peak_memory,
            'disk_bytes_per_sim_minute': int(disk / (sim / 60.0)),
            'rtf_mean': round(sum(slot.rtfs) / len(slot.rtfs), 3) if slot.rtfs else None,
            'rtf_min': round(min(slot.rtfs), 3) if slot.rtfs else None,
            'sim_duration_s': round(sim, 1), 'wall_duration_s': round(wall, 1),
            'recording_profile': self.exp.config.data['recording']['profile'],
            'maze_area_m2': resources._set_area(self.exp.sets()[slot.planned.set_index]),
            'experiment': self.exp.id, 'run': slot.run_id,
        }
        self.calibration = resources.save_calibration(self.data_dir, self.machine,
                                                      self.profile.name, cal)
        self.log('calibration', **cal)
        self.update_cap()
        if not self.paused and self.exp.state == 'calibrating':
            self.exp.set_state('running')

    # ================================================================ processing (ER4)
    def run_processing(self):
        if self.proc_job is not None:
            proc, run_dir, started = self.proc_job
            if proc.poll() is None:
                return
            ok = proc.returncode == 0 and os.path.isfile(os.path.join(run_dir, 'metrics.json'))
            self.log('processed', run=os.path.basename(run_dir), ok=ok,
                     seconds=round(time.monotonic() - started, 1))
            if ok:
                self.apply_retention(run_dir)
            self.proc_job = None
            self.update_summary(force=True)
        if self.processing:
            run_dir = self.processing.pop(0)
            log = open(os.path.join(run_dir, 'processing.log'), 'ab')
            cmd = ['nice', '-n', '15', sys.executable, '-m', 'maze_experiments.main',
                   '--data-dir', self.data_dir, '_process-run', run_dir]
            proc = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=log,
                                    stderr=subprocess.STDOUT, start_new_session=True)
            log.close()
            self.proc_job = (proc, run_dir, time.monotonic())

    def apply_retention(self, run_dir):
        """DR9: keep all recordings, only those of runs that were not mission successes, or none."""
        policy = self.exp.config.data['recording'].get('retention', 'all')
        if policy == 'all':
            return
        try:
            metrics = fileio.read_json(os.path.join(run_dir, 'metrics.json'))
        except (OSError, ValueError):
            return
        if policy == 'failures' and not metrics.get('outcome', {}).get('mission_success'):
            return
        import shutil
        bag = os.path.join(run_dir, 'bag')
        if os.path.isdir(bag):
            shutil.rmtree(bag, ignore_errors=True)
            from maze_monitor.run_store import RunStore
            info = RunStore(os.path.dirname(run_dir)).load(run_dir)
            info.meta.setdefault('experiment', {})['recording'] = 'deleted'
            info.save()
            self.log('recording_deleted', run=os.path.basename(run_dir), policy=policy)

    # ================================================================ summary / live
    def update_summary(self, force=False):
        now = time.monotonic()
        if not force and now - self._last_summary < 10.0:
            return
        self._last_summary = now
        from .metrics.aggregate import experiment_summary
        try:
            summary = experiment_summary(self.exp)
        except Exception as exc:
            self.log('error', text=f'summary failed: {exc}')
            return
        self.exp.reload()
        self.exp.meta['summary'] = summary
        self.exp.save()

    def write_live(self, machine_sample):
        now = time.monotonic()
        if now - self._last_live < LIVE_EVERY:
            return
        self._last_live = now
        active = []
        for s in self.active.values():
            active.append({
                'run': s.run_id, 'key': list(s.planned.key), 'attempt': s.attempt,
                'set': s.planned.set_index, 'maze': s.planned.maze_index,
                'phase': 'stopping' if s.outcome else ('running' if s.ready_at else 'starting'),
                'sim': s.sim, 'rtf': s.rtf, 'area': s.last.get('area'),
                'explore': s.last.get('explore'), 'outcome': s.outcome, 'reason': s.reason,
                'wall': round(now - s.started, 1), 'cpus': s.cpus,
                'cpu': s.sampler.cpu_cores if s.sampler else None,
                'memory': s.sampler.memory if s.sampler else None,
            })
        fileio.write_json(self.exp.p('live.json'), {
            'time': iso(), 'pid': os.getpid(), 'state': self.exp.state, 'paused': self.paused,
            'target': self.controller.target, 'cap': self.controller.cap,
            'held_back': self.controller.held_back, 'calibrated': self.calibration is not None,
            'queue': len(self.queue), 'processing': len(self.processing) + bool(self.proc_job),
            'active': active, 'leftover': list(self.leftover), 'machine': machine_sample,
            'messages': self.messages[-10:],
        })

    # ================================================================ machine checks
    def check_clock(self, now):
        mono, boot, wall = now, time.clock_gettime(time.CLOCK_BOOTTIME), time.time()
        problems = []
        if self._last_tick is not None:
            m0, b0, w0 = self._last_tick
            if mono - m0 > SUSPEND_GAP or boot - b0 > SUSPEND_GAP:
                problems.append(('host_suspend', f'{max(mono - m0, boot - b0):.0f} s gap '
                                                 f'between manager ticks'))
            elif abs((wall - w0) - (mono - m0)) > CLOCK_JUMP_STEP:
                problems.append(('clock_jump', f'wall clock stepped '
                                               f'{(wall - w0) - (mono - m0):+.1f} s'))
        self._last_tick = (mono, boot, wall)
        if wall - self._last_journal >= JOURNAL_EVERY:
            since = self._last_journal
            jumps = resources.journal_clock_changes(f'@{since:.0f}')
            self._last_journal = wall
            if jumps:
                problems.append(('clock_jump', f'journal: "Clock change detected" x{len(jumps)}'))
        action = self.exp.execution.get('clock_jump_action', 'retry')
        for kind, detail in problems:
            self.log('host_event', kind=kind, detail=detail)
            for slot in list(self.active.values()):
                if slot.outcome is not None:
                    continue
                if kind == 'clock_jump' and action == 'flag':
                    slot.flags.append(f'clock_jump: {detail}')
                    continue
                self.stop(slot, 'infra_failure', f'{kind}: {detail}')

    def protect_machine(self, sample):
        """CR7. Returns True if new runs may start."""
        total, avail = sample.get('mem_total'), sample.get('mem_available')
        may_start = True
        if total and avail is not None:
            if avail < 0.05 * total:
                running = [s for s in self.active.values() if s.outcome is None]
                if running:
                    newest = max(running, key=lambda s: s.started)
                    self.say('warning', f'Available memory below 5%: stopping the newest run '
                                        f'({newest.run_id}).')
                    self.stop(newest, 'infra_failure', 'low_memory')
                may_start = False
            elif avail < 0.10 * total:
                may_start = False
        if self.calibration and not self.disk_paused:
            per_run = (self.calibration.get('disk_bytes_per_sim_minute') or 0) * \
                (self.calibration.get('sim_duration_s') or 0) / 60.0
            projected = per_run * (len(self.queue) + len(self.active))
            used = resources.dir_size(self.exp.path) if self.disk_alloc else 0
            free = sample.get('disk_free')
            if (self.disk_alloc and used + projected > self.disk_alloc) or \
                    (free is not None and projected > free):
                self.disk_paused = True
                self.paused = True
                self.exp.set_state('paused', reason='disk')
                self.say('error', f'Projected disk use ({resources.fmt_size(projected)} more) '
                                  f'exceeds the allocation or the free space. The experiment is '
                                  f'paused; free space or raise the allocation, then resume.')
        return may_start

    # ================================================================ main loop
    def tick(self):
        now = time.monotonic()
        self.read_control()
        self.check_clock(now)
        sample = self.machine_sampler.sample(self.data_dir)
        runs = {}
        for slot in list(self.active.values()):
            self.read_status(slot)
            cpu, mem = slot.sampler.sample() if slot.sampler else (None, None)
            if slot.outcome is None and slot.ready_at is not None:
                slot.cpu_samples.append(cpu)
            runs[slot.run_id] = {'cpu': None if cpu is None else round(cpu, 3), 'mem': mem,
                                 'rtf': slot.rtf, 'sim': slot.sim}
            if slot.outcome is None:
                decision = self.evaluate(slot, now)
                if decision:
                    self.stop(slot, *decision)
            if slot.outcome is not None and self.advance_stop(slot, now):
                del self.active[slot.run_id]
                self.finish(slot)
            elif slot.stage == 3 and slot.leftover_reported:
                del self.active[slot.run_id]
                self.leftover[slot.run_id] = slot
        for rid, slot in list(self.leftover.items()):
            if not self.remaining(slot):
                del self.leftover[rid]
                self.log('leftover_gone', run=rid)
                self.finish(slot)
        fileio.append_jsonl(self.exp.resources_log, {
            't': round(now, 1), 'time': iso(), 'machine': sample, 'runs': runs,
            'concurrency': len([s for s in self.active.values() if s.outcome is None]),
            'target': self.controller.target}, kind='resources')
        changes = self.controller.update(
            [(s.run_id, s.ready_at, s.rtf) for s in self.active.values() if s.outcome is None],
            now)
        for c in changes:
            self.log('resources', **c)
        running = [s for s in self.active.values() if s.outcome is None]
        if (len(running) == 1 and running[0].ready_at and running[0].rtf is not None
                and running[0].rtf < self.min_rtf and not self.warned_low_rtf_single
                and self.controller.low_since.get(running[0].run_id) is not None
                and now - self.controller.low_since[running[0].run_id]
                >= resources.LOW_RTF_SECONDS):
            self.warned_low_rtf_single = True
            self.say('warning', f'Even a single run cannot keep the minimum RTF of '
                                f'{self.min_rtf} on this machine. Runs continue one at a time '
                                f'and are flagged low_rtf (CNR3).')
        may_start = self.protect_machine(sample)
        self.run_processing()
        if not (self.paused or self.cancelling or self.stopping_manager) and may_start:
            self.start_more(now)
        self.update_summary()
        self.write_live(sample)

    def start_more(self, now):
        running = [s for s in self.active.values() if s.outcome is None]
        if self.calibration is None:
            if self.active or not self.queue:
                return   # CR3: the calibration run runs alone
            if self.exp.state != 'calibrating':
                self.exp.set_state('calibrating')
            planned, attempt = self.queue.pop(0)
            if self.start_run(planned, attempt) is None:
                self.queue.insert(0, (planned, attempt))
            return
        while self.queue and self.controller.may_start(len(running)) \
                and len(self.active) < self.controller.cap + 1:
            # Starting several runs in the same second makes them all slow at startup
            if running and now - max(s.started for s in running) < 15.0:
                return
            planned, attempt = self.queue.pop(0)
            slot = self.start_run(planned, attempt)
            if slot is None:
                self.queue.insert(0, (planned, attempt))
                return
            running.append(slot)

    def done(self):
        return (not self.queue and not self.active and not self.leftover
                and not self.processing and self.proc_job is None)

    def on_signal(self, signum, _frame):
        self.stopping_manager = True

    def shutdown_runs(self, outcome_reason):
        for slot in list(self.active.values()):
            if slot.outcome is None:
                self.stop(slot, 'interrupted', outcome_reason)
        deadline = time.monotonic() + STOP_SIGINT_WAIT + STOP_SIGTERM_WAIT + STOP_SIGKILL_WAIT + 5
        while self.active and time.monotonic() < deadline:
            now = time.monotonic()
            for slot in list(self.active.values()):
                self.read_status(slot)
                if self.advance_stop(slot, now):
                    del self.active[slot.run_id]
                    self.finish(slot)
            time.sleep(0.5)

    def run(self):
        if not self.lock.acquire():
            print('Another manager is already running this experiment.', file=sys.stderr)
            return 1
        signal.signal(signal.SIGTERM, self.on_signal)
        signal.signal(signal.SIGINT, self.on_signal)
        signal.signal(signal.SIGHUP, signal.SIG_IGN)
        try:
            self.exp.reload()
            self.setup()
            self.log('manager_started', pid=os.getpid(), resume=self.resume,
                     state=self.exp.state, calibration=bool(self.calibration),
                     cpus=self.cpu_list, memory=self.memory_alloc)
            try:
                self.control_offset = os.path.getsize(self.exp.control_file)
            except OSError:
                self.control_offset = 0
            if self.exp.state in ('created', 'generating') or not os.path.isdir(self.exp.p('mazes')):
                self.generate_mazes()
            self.recover()
            self.build_queue()
            self.log('queued', runs=len(self.queue), to_process=len(self.processing),
                     order=[self.queue[i][0].run_id(self.queue[i][1])
                            for i in range(min(len(self.queue), 500))])
            self.exp.set_state('calibrating' if self.calibration is None and self.queue
                               else 'running')
            next_tick = time.monotonic()
            while not self.stopping_manager:
                self.tick()
                if self.cancelling and not self.active and not self.leftover:
                    break
                if self.done():
                    break
                next_tick += TICK
                delay = next_tick - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_tick = time.monotonic()
            if self.stopping_manager:
                self.log('manager_stopping', reason='signal')
                self.shutdown_runs('manager_stopped')
                self.exp.set_state('interrupted', reason='manager stopped')
                return 0
            # Finish processing what has finished, then the results.
            while self.processing or self.proc_job is not None:
                self.run_processing()
                time.sleep(0.5)
            self.exp.set_state('processing')
            self.log('aggregating')
            from .metrics import results
            try:
                results.build_results(self.exp)
                self.log('results', ok=True)
            except Exception as exc:
                self.log('error', text=f'results failed: {exc}',
                         traceback=traceback.format_exc())
            self.update_summary(force=True)
            self.exp.set_state('cancelled' if self.cancelling else 'completed')
            self.log('manager_finished', state=self.exp.state)
            return 0
        except Exception as exc:
            self.log('error', text=f'manager error: {type(exc).__name__}: {exc}',
                     traceback=traceback.format_exc())
            try:
                self.shutdown_runs('manager_crashed')
                self.exp.set_state('interrupted', reason=f'manager error: {exc}')
            except Exception:
                pass
            return 2
        finally:
            try:
                self.write_live(self.machine_sampler.sample(self.data_dir))
            except Exception:
                pass
            self.lock.release()


def main(exp_path, data_dir, resume=False):
    exp = exp_mod.Experiment(exp_path)
    return Manager(exp, data_dir, resume).run()
