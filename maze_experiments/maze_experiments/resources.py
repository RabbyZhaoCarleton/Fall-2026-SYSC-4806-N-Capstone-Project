"""Module C: what the machine has, what a run costs, and how many runs fit.

Everything is measured inside WSL at runtime (CR1, S1); nothing is tuned to a
particular PC. Anything that can't be measured is reported as unknown (None),
never guessed.
"""

import datetime
import glob
import hashlib
import json
import os
import shutil
import subprocess
import time

from . import fileio, paths

CLK_TCK = os.sysconf('SC_CLK_TCK')
PAGE = os.sysconf('SC_PAGE_SIZE')
CALIBRATION_MAX_AGE_DAYS = 7
MEMORY_HEADROOM = 1.25        # CR4: peak memory x 1.25
LOW_RTF_SECONDS = 30.0        # CR5
RAISE_SETTLE_SECONDS = 60.0   # a new run must hold the minimum RTF this long before the next


# ================================================================ machine (CR1)
def meminfo():
    out = {}
    try:
        with open('/proc/meminfo') as f:
            for line in f:
                key, _, rest = line.partition(':')
                out[key] = int(rest.split()[0]) * 1024
    except OSError:
        pass
    return out


def cpu_model():
    try:
        with open('/proc/cpuinfo') as f:
            for line in f:
                if line.startswith('model name'):
                    return line.split(':', 1)[1].strip()
    except OSError:
        pass
    return None


def allowed_cpus():
    try:
        return sorted(os.sched_getaffinity(0))
    except (AttributeError, OSError):
        n = os.cpu_count()
        return list(range(n)) if n else []


_renderer_cache = {}


def detect_renderer(timeout=30):
    """The renderer Gazebo will use, from tools/check_gpu.py (CR1). None if unknown."""
    key = (os.environ.get('GALLIUM_DRIVER'), os.environ.get('MESA_D3D12_DEFAULT_ADAPTER_NAME'),
           os.environ.get('LIBGL_ALWAYS_SOFTWARE'))
    if key in _renderer_cache:
        return _renderer_cache[key]
    result = None
    repo = paths.repo_root()
    script = os.path.join(repo, 'tools', 'check_gpu.py') if repo else None
    if script and os.path.isfile(script):
        try:
            out = subprocess.run(['python3', script], capture_output=True, text=True,
                                 timeout=timeout)
            for line in out.stdout.splitlines():
                if line.startswith('Renderer:'):
                    name = line.split(':', 1)[1].strip()
                    kind = 'software' if 'llvmpipe' in name.lower() or \
                        'softpipe' in name.lower() else 'gpu'
                    result = {'name': name, 'kind': kind}
        except (OSError, subprocess.SubprocessError):
            result = None
    _renderer_cache[key] = result
    return result


def gpu_memory_mb():
    """(total, used) MB from nvidia-smi if a vendor tool exists; None otherwise."""
    exe = shutil.which('nvidia-smi') or ('/usr/lib/wsl/lib/nvidia-smi'
                                         if os.path.exists('/usr/lib/wsl/lib/nvidia-smi') else None)
    if not exe:
        return None
    try:
        out = subprocess.run([exe, '--query-gpu=memory.total,memory.used',
                              '--format=csv,noheader,nounits'], capture_output=True, text=True,
                             timeout=10)
        total, used = out.stdout.strip().splitlines()[0].split(',')
        return {'total': int(total), 'used': int(used)}
    except Exception:
        return None


def measure_machine(data_dir=None, renderer=None, with_renderer=True):
    mem = meminfo()
    cpus = allowed_cpus()
    disk_free = None
    if data_dir:
        try:
            probe = data_dir
            while not os.path.exists(probe):
                probe = os.path.dirname(probe)
            disk_free = shutil.disk_usage(probe).free
        except OSError:
            disk_free = None
    if renderer is None and with_renderer:
        renderer = detect_renderer()
    return {
        'cpu_model': cpu_model(),
        'logical_cpus': len(cpus) or None,
        'cpu_list': cpus,
        'memory_total': mem.get('MemTotal'),
        'memory_available': mem.get('MemAvailable'),
        'disk_free': disk_free,
        'renderer': renderer,
        'gpu_memory_mb': gpu_memory_mb(),
        'systemd_scope': systemd_scope_available(),
    }


def machine_key(machine):
    r = machine.get('renderer') or {}
    text = json.dumps([machine.get('cpu_model'), machine.get('logical_cpus'),
                       round((machine.get('memory_total') or 0) / 2 ** 30), r.get('name')])
    return hashlib.sha256(text.encode()).hexdigest()[:12]


def default_allocation(machine):
    """CR2: half of the logical CPUs and half of the available memory."""
    cpus = machine.get('logical_cpus') or 1
    mem = machine.get('memory_available') or 0
    return {'cpus': max(1, cpus // 2), 'memory_bytes': mem // 2,
            'disk_bytes': (machine.get('disk_free') or 0) // 2}


def parse_size(text):
    """'8G', '512M', '1.5GiB', or a number of bytes."""
    if isinstance(text, (int, float)):
        return int(text)
    s = str(text).strip().upper().replace('IB', '').replace('B', '')
    mult = {'K': 2 ** 10, 'M': 2 ** 20, 'G': 2 ** 30, 'T': 2 ** 40}
    if s and s[-1] in mult:
        return int(float(s[:-1]) * mult[s[-1]])
    return int(float(s))


def fmt_size(n):
    if n is None:
        return 'unknown'
    for unit in ('B', 'KB', 'MB', 'GB', 'TB'):
        if abs(n) < 1024 or unit == 'TB':
            return f'{n:.0f} {unit}' if unit == 'B' else f'{n:.1f} {unit}'
        n /= 1024.0


# ================================================================ confinement (CR6)
_scope_cache = {}


def systemd_scope_available():
    if 'ok' not in _scope_cache:
        ok = False
        if shutil.which('systemd-run'):
            try:
                ok = subprocess.run(['systemd-run', '--user', '--scope', '--quiet', 'true'],
                                    capture_output=True, timeout=10).returncode == 0
            except (OSError, subprocess.SubprocessError):
                ok = False
        _scope_cache['ok'] = ok
    return _scope_cache['ok']


def cpuset_delegated():
    """Whether user systemd can apply AllowedCPUs (the cpuset controller is delegated)."""
    uid = os.getuid()
    f = f'/sys/fs/cgroup/user.slice/user-{uid}.slice/user@{uid}.service/cgroup.subtree_control'
    try:
        with open(f) as fh:
            return 'cpuset' in fh.read().split()
    except OSError:
        return False


def cpu_list_text(cpus):
    return ','.join(str(c) for c in cpus)


def launch_prefix(cpus=None, memory_max=None, nice=10):
    """Command prefix that confines one run: its own cores, a memory limit, lower priority.

    Uses a user systemd scope when available (memory limit; cores when the
    cpuset controller is delegated), and taskset for the cores otherwise.
    Returns (prefix list, description dict).
    """
    prefix, desc = [], {'method': [], 'cpus': cpus, 'memory_max': memory_max, 'nice': nice}
    if systemd_scope_available() and (memory_max or cpus):
        prefix += ['systemd-run', '--user', '--scope', '--quiet', '--collect']
        if memory_max:
            prefix += ['-p', f'MemoryMax={int(memory_max)}']
        if cpus and cpuset_delegated():
            prefix += ['-p', f'AllowedCPUs={cpu_list_text(cpus)}']
            desc['method'].append('systemd-cpuset')
        desc['method'].append('systemd-scope')
    if cpus and 'systemd-cpuset' not in desc['method'] and shutil.which('taskset'):
        prefix += ['taskset', '-c', cpu_list_text(cpus)]
        desc['method'].append('taskset')
    if nice:
        prefix += ['nice', '-n', str(int(nice))]
        desc['method'].append('nice')
    return prefix, desc


# ================================================================ processes
def boot_id():
    try:
        with open('/proc/sys/kernel/random/boot_id') as f:
            return f.read().strip()
    except OSError:
        return None


def proc_stat(pid):
    """(pgid, start ticks, utime+stime ticks, rss bytes, state) of a process, or None."""
    try:
        with open(f'/proc/{pid}/stat', 'rb') as f:
            data = f.read().decode(errors='replace')
    except OSError:
        return None
    rparen = data.rfind(')')
    fields = data[rparen + 2:].split()
    # fields[0] is state (field 3); pgrp is field 5, utime 14, stime 15, starttime 22, rss 24
    try:
        return (int(fields[2]), int(fields[19]), int(fields[11]) + int(fields[12]),
                int(fields[21]) * PAGE, fields[0])
    except (IndexError, ValueError):
        return None


def process_env_has(pid, marker):
    """True if the process environment contains marker (b'KEY=value')."""
    try:
        with open(f'/proc/{pid}/environ', 'rb') as f:
            return marker in f.read().split(b'\0')
    except OSError:
        return False


def group_members(pgid, marker=None, min_start=None):
    """PIDs whose process group is pgid (optionally with an env marker and start time)."""
    out = []
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        st = proc_stat(entry)
        if st is None or st[0] != pgid or st[4] == 'Z':
            continue
        if min_start is not None and st[1] < min_start:
            continue
        if marker is not None and not process_env_has(entry, marker):
            continue
        out.append(int(entry))
    return out


def cgroup_of(pid):
    try:
        with open(f'/proc/{pid}/cgroup') as f:
            for line in f:
                if line.startswith('0::'):
                    path = '/sys/fs/cgroup' + line.strip()[3:]
                    return path if path.endswith('.scope') else None
    except OSError:
        pass
    return None


class RunSampler:
    """CPU and memory of one run's process tree, from its cgroup or its process group."""

    def __init__(self, pgid, leader_pid):
        self.pgid = pgid
        self.cgroup = cgroup_of(leader_pid)
        self._members = []
        self._members_at = 0.0
        self._last = None   # (monotonic, cpu seconds)
        self.cpu_cores = 0.0
        self.memory = 0
        self.peak_memory = 0

    def _cpu_and_mem(self, now):
        if self.cgroup and os.path.isdir(self.cgroup):
            try:
                with open(os.path.join(self.cgroup, 'cpu.stat')) as f:
                    usage = int(f.readline().split()[1]) / 1e6
                with open(os.path.join(self.cgroup, 'memory.current')) as f:
                    mem = int(f.read())
                return usage, mem
            except (OSError, ValueError, IndexError):
                pass
        if now - self._members_at > 2.0:
            self._members = group_members(self.pgid)
            self._members_at = now
        cpu, mem = 0, 0
        for pid in self._members:
            st = proc_stat(pid)
            if st is not None and st[0] == self.pgid:
                cpu += st[2]
                mem += st[3]
        return cpu / CLK_TCK, mem

    def sample(self):
        now = time.monotonic()
        usage, mem = self._cpu_and_mem(now)
        if self._last is not None and now > self._last[0]:
            self.cpu_cores = max(0.0, (usage - self._last[1]) / (now - self._last[0]))
        self._last = (now, usage)
        self.memory = mem
        self.peak_memory = max(self.peak_memory, mem)
        return self.cpu_cores, mem


class MachineSampler:
    def __init__(self):
        self._last = None

    def sample(self, data_dir=None):
        try:
            with open('/proc/stat') as f:
                parts = [int(x) for x in f.readline().split()[1:]]
            idle = parts[3] + (parts[4] if len(parts) > 4 else 0)
            total = sum(parts[:8])
        except (OSError, ValueError):
            idle = total = None
        cpu = None
        if total is not None and self._last is not None and total > self._last[1]:
            cpu = 1.0 - (idle - self._last[0]) / (total - self._last[1])
        if total is not None:
            self._last = (idle, total)
        mem = meminfo()
        disk = None
        if data_dir:
            try:
                disk = shutil.disk_usage(data_dir).free
            except OSError:
                pass
        return {'cpu': None if cpu is None else round(cpu, 3),
                'mem_total': mem.get('MemTotal'), 'mem_available': mem.get('MemAvailable'),
                'disk_free': disk}


# ================================================================ calibration (CR3)
def calibration_path(data_dir, machine, profile_name):
    return os.path.join(data_dir, 'calibration', f'{machine_key(machine)}-{profile_name}.json')


def load_calibration(data_dir, machine, profile_name, max_age_days=CALIBRATION_MAX_AGE_DAYS):
    path = calibration_path(data_dir, machine, profile_name)
    try:
        cal = fileio.read_json(path)
    except (OSError, ValueError):
        return None
    try:
        when = datetime.datetime.fromisoformat(cal['measured'])
    except (KeyError, ValueError):
        return None
    if (datetime.datetime.now() - when).days > max_age_days:
        return None
    return cal


def save_calibration(data_dir, machine, profile_name, cal):
    cal = dict(cal)
    cal['measured'] = datetime.datetime.now().isoformat(timespec='seconds')
    cal['machine_key'] = machine_key(machine)
    cal['profile'] = profile_name
    fileio.write_json(calibration_path(data_dir, machine, profile_name), cal)
    return cal


def _set_area(set_):
    m = set_['maze']
    first = lambda v: v[0] if isinstance(v, list) else v  # noqa: E731
    return float(first(m['width'])) * float(first(m['length']))


def estimate(config, calibration, execution, machine):
    """CR9: duration and disk use of an experiment, or None if there is no calibration.

    Every run is assumed to cost what the calibration run cost. Run length was
    found to depend little on maze size (2 x 2 m and 5 x 5 m runs took similar
    time), so it is not scaled; the run screen shows the real progress.
    """
    if not calibration:
        return None
    runs = config.total_runs()
    wall_s = calibration.get('wall_duration_s') or 0
    sim_s = calibration.get('sim_duration_s') or 0
    disk_rate = calibration.get('disk_bytes_per_sim_minute') or 0
    if config.data['recording'].get('profile') == 'metrics_only':
        disk_rate *= calibration.get('metrics_only_disk_ratio', 0.3)
    total_wall, total_disk = 0.0, 0.0
    for s in config.sets():
        n = int(s['mazes']) * int(s['repeats'])
        scale = 1.0
        total_wall += n * wall_s * scale
        total_disk += n * disk_rate * sim_s * scale / 60.0
    conc = concurrency_cap(execution, calibration, free_domains=101)
    retention = config.data['recording'].get('retention')
    stored = total_disk if retention == 'all' else total_disk * 0.5 if \
        retention == 'failures' else total_disk / max(runs, 1) * conc
    return {'runs': runs, 'concurrency': conc, 'duration_s': total_wall / max(conc, 1),
            'disk_bytes': stored, 'disk_peak_per_run': total_disk / max(runs, 1)}


# ================================================================ concurrency (CR4, CR5)
def concurrency_cap(execution, calibration, free_domains):
    """CR4: min(CPUs / avg CPU, memory / (peak x 1.25), free domain IDs, user maximum)."""
    limits = [free_domains, int(execution.get('max_concurrency') or 64)]
    if calibration:
        cpus = execution.get('cpus') or 1
        # One allocated CPU is kept for processing finished runs (ER4) when there are several.
        usable = cpus - 1 if cpus >= 3 else cpus
        avg = max(calibration.get('avg_cpu_cores') or 0.0, 0.1)
        limits.append(int(usable // avg))
        mem = execution.get('memory_bytes') or 0
        peak = calibration.get('peak_memory_bytes') or 0
        if peak > 0 and mem > 0:
            limits.append(int(mem // (peak * MEMORY_HEADROOM)))
    return max(1, min(limits))


class ConcurrencyController:
    """Raises concurrency one run at a time while every run keeps the minimum RTF (CR4, CR5)."""

    def __init__(self, min_rtf, cap=1):
        self.min_rtf = min_rtf
        self.cap = cap
        self.target = 1
        self.low_since = {}          # run id -> monotonic time its RTF first fell below
        self.ceiling = None          # level that failed, and until when it is avoided
        self.ceiling_until = 0.0
        self.last_change = time.monotonic()
        self.held_back = False       # CR5: no new run until RTF recovers
        self.failures = {}           # level -> times it was lowered from
        self.max_failures = 2        # after this, the level is not tried again

    def update(self, active, now=None):
        """active: [(run id, ready monotonic time or None, current rtf or None)].

        Returns a list of change records for the log.
        """
        now = time.monotonic() if now is None else now
        changes = []
        low_now = []
        for rid, ready_at, rtf in active:
            if ready_at is None or rtf is None:
                self.low_since.pop(rid, None)
                continue
            if rtf < self.min_rtf:
                self.low_since.setdefault(rid, now)
                if now - self.low_since[rid] >= LOW_RTF_SECONDS:
                    low_now.append(rid)
            else:
                self.low_since.pop(rid, None)
        for rid in list(self.low_since):
            if rid not in {a[0] for a in active}:
                del self.low_since[rid]
        self.held_back = bool(self.low_since)
        if low_now and now - self.last_change >= LOW_RTF_SECONDS and self.target > 1:
            self.ceiling = self.target
            n = self.failures[self.target] = self.failures.get(self.target, 0) + 1
            self.ceiling_until = now + 600.0 * 2 ** (n - 1)
            if n >= self.max_failures:
                self.cap = min(self.cap, self.target - 1)
            self.target -= 1
            self.last_change = now
            changes.append({'change': 'lower', 'target': self.target,
                            'reason': f'RTF below {self.min_rtf} for {LOW_RTF_SECONDS:.0f} s',
                            'runs': low_now,
                            'level_failures': self.failures[self.ceiling],
                            'cap': self.cap})
            for rid in low_now:
                self.low_since[rid] = now
            return changes
        ready = [a for a in active if a[1] is not None]
        if (self.target < self.cap and not self.low_since and len(active) >= self.target
                and ready and len(ready) == len(active)
                and all(r is not None and r >= self.min_rtf for _, _, r in ready)
                and now - max(a[1] for a in ready) >= RAISE_SETTLE_SECONDS
                and now - self.last_change >= RAISE_SETTLE_SECONDS
                and not (self.ceiling is not None and self.target + 1 >= self.ceiling
                         and now < self.ceiling_until)):
            self.target += 1
            self.last_change = now
            changes.append({'change': 'raise', 'target': self.target,
                            'reason': f'all {len(ready)} active run(s) at RTF >= {self.min_rtf}'})
        return changes

    def may_start(self, n_active):
        return n_active < self.target and not self.held_back


def assign_cpus(cpu_list, slot, n_slots):
    """Cores for one concurrent run: an equal, disjoint share of the allocated CPUs (CR6)."""
    if not cpu_list:
        return None
    n_slots = max(1, n_slots)
    per = max(1, len(cpu_list) // n_slots)
    start = (slot * per) % len(cpu_list)
    return cpu_list[start:start + per] or cpu_list[:per]


# ================================================================ clock (FR6, BR13)
def clock_warnings(minutes=10):
    """FR6: an unstable WSL clock. Returns a list of warnings (empty if fine)."""
    out = []
    is_wsl = 'microsoft' in os.uname().release.lower()
    try:
        active = subprocess.run(['systemctl', 'is-active', 'systemd-timesyncd'],
                                capture_output=True, text=True, timeout=10).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        active = ''
    if is_wsl and active == 'active':
        out.append('systemd-timesyncd is active under WSL. It fights with WSL\'s own time sync '
                   'and makes the clock jump, which stalls SLAM. See "WSL clock jumps" in '
                   'README.md for the fix.')
    jumps = journal_clock_changes(f'-{int(minutes)}min')
    if jumps is None:
        out.append('Could not read the system journal to check for clock jumps (unknown).')
    elif jumps:
        out.append(f'The journal shows {len(jumps)} "Clock change detected" entries in the last '
                   f'{minutes} minutes: the WSL clock is jumping. Runs during a jump are marked '
                   f'infra_failure and repeated. See "WSL clock jumps" in README.md for the fix.')
    return out


def journal_clock_changes(since):
    """Times (unix s) of 'Clock change detected' journal entries since `since`; None if unknown."""
    if not shutil.which('journalctl'):
        return None
    try:
        out = subprocess.run(['journalctl', '--since', since, '-q', '--no-pager', '-o',
                              'short-unix', '-g', 'Clock change detected'],
                             capture_output=True, text=True, timeout=15)
    except (OSError, subprocess.SubprocessError):
        return None
    if out.returncode not in (0, 1):
        return None
    times = []
    for line in out.stdout.splitlines():
        try:
            times.append(float(line.split()[0]))
        except (IndexError, ValueError):
            continue
    return times


def fastdds_shm_clean():
    """BR16: remove Fast DDS shared-memory files no live process uses."""
    exe = shutil.which('fastdds')
    if not exe:
        return 'fastdds not found'
    try:
        out = subprocess.run([exe, 'shm', 'clean'], capture_output=True, text=True, timeout=60)
        return out.stdout.strip().splitlines()[-1] if out.stdout.strip() else 'ok'
    except (OSError, subprocess.SubprocessError) as exc:
        return f'failed: {exc}'


def dir_size(path):
    total = 0
    for f in glob.glob(os.path.join(path, '**', '*'), recursive=True):
        try:
            if os.path.isfile(f):
                total += os.path.getsize(f)
        except OSError:
            pass
    return total
