"""Starting and finding an experiment's manager process (Module B).

The manager is a background process with its own session, so closing the GUI
or the terminal doesn't stop it (BNR5). It holds an exclusive lock on
<experiment>/manager.lock for as long as it lives; the kernel releases the
lock when the process dies in any way, so "is a manager alive?" never trusts
a stale PID file.
"""

import fcntl
import json
import os
import subprocess
import sys
import time

LOCK_FILE = 'manager.lock'
OUT_FILE = 'manager.out'


class ManagerLock:
    def __init__(self, exp):
        self.path = exp.p(LOCK_FILE)
        self.fd = None

    def acquire(self):
        fd = os.open(self.path, os.O_RDWR | os.O_CREAT, 0o644)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            os.close(fd)
            return False
        os.ftruncate(fd, 0)
        os.write(fd, json.dumps({'pid': os.getpid(), 'started': time.time()}).encode())
        self.fd = fd
        return True

    def release(self):
        if self.fd is not None:
            try:
                fcntl.flock(self.fd, fcntl.LOCK_UN)
            finally:
                os.close(self.fd)
                self.fd = None


def manager_alive(exp):
    path = exp.p(LOCK_FILE)
    if not os.path.exists(path):
        return False
    try:
        fd = os.open(path, os.O_RDONLY)
    except OSError:
        return False
    try:
        fcntl.flock(fd, fcntl.LOCK_SH | fcntl.LOCK_NB)
    except OSError:
        return True
    else:
        fcntl.flock(fd, fcntl.LOCK_UN)
        return False
    finally:
        os.close(fd)


def manager_pid(exp):
    try:
        with open(exp.p(LOCK_FILE)) as f:
            return json.load(f).get('pid')
    except (OSError, ValueError):
        return None


def start_manager(exp, data_dir, resume=False, wait=10.0):
    """Start a detached manager for `exp`. Returns its PID."""
    cmd = [sys.executable, '-m', 'maze_experiments.main', '--data-dir', data_dir,
           '_manager', exp.path]
    if resume:
        cmd.append('--resume')
    out = open(exp.p(OUT_FILE), 'ab')
    proc = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=out, stderr=out,
                            start_new_session=True, close_fds=True)
    out.close()
    deadline = time.monotonic() + wait
    while time.monotonic() < deadline:
        if manager_alive(exp):
            return proc.pid
        if proc.poll() is not None:
            raise RuntimeError(f'The manager exited at once (code {proc.returncode}); see '
                               f'{exp.p(OUT_FILE)}')
        time.sleep(0.1)
    return proc.pid
