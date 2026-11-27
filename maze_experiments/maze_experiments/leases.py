"""Machine-wide leases on ROS domain IDs 1 to 101 (BR1, BR4).

A lease is a file ~/.cache/maze_experiments/leases/domain-<n>.json naming the
process group that uses the domain. It is created atomically (O_EXCL), so two
managers never get the same ID. A lease whose processes are all gone is stale
and can be taken over; a lease whose processes are still alive is never
reused, even if its manager died (BR4).
"""

import json
import os
import secrets

from . import resources

DOMAIN_MIN, DOMAIN_MAX = 1, 101
ENV_MARKER = 'MAZE_EXPERIMENTS_RUN'


def lease_dir():
    d = os.path.join(os.path.expanduser('~/.cache/maze_experiments'), 'leases')
    os.makedirs(d, exist_ok=True)
    return d


def _path(domain):
    return os.path.join(lease_dir(), f'domain-{domain}.json')


def read(domain):
    try:
        with open(_path(domain)) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def holder_alive(lease):
    """True if any process of the lease's run is still alive (same boot, env marker)."""
    if not lease:
        return False
    if lease.get('boot_id') and lease.get('boot_id') != resources.boot_id():
        return False
    pgid = lease.get('pgid')
    owner = lease.get('owner_pid')
    if pgid is None:
        # Reserved but not started yet: alive while the manager that reserved it lives.
        return bool(owner) and os.path.exists(f'/proc/{owner}')
    marker = f'{ENV_MARKER}={lease.get("run")}'.encode()
    return bool(resources.group_members(pgid, marker=marker))


def acquire(run_key, owner_pid=None):
    """Reserve a free domain ID. Returns (domain, partition) or None if all are taken."""
    for domain in range(DOMAIN_MIN, DOMAIN_MAX + 1):
        path = _path(domain)
        if os.path.exists(path):
            if holder_alive(read(domain)):
                continue
            try:
                os.unlink(path)
            except OSError:
                continue
        partition = f'maze_exp_{domain}_{secrets.token_hex(3)}'
        rec = {'domain': domain, 'partition': partition, 'run': run_key,
               'owner_pid': owner_pid or os.getpid(), 'pgid': None,
               'boot_id': resources.boot_id()}
        try:
            fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
        except FileExistsError:
            continue
        with os.fdopen(fd, 'w') as f:
            json.dump(rec, f)
        return domain, partition
    return None


def attach(domain, pgid, leader_start):
    rec = read(domain) or {}
    rec.update(pgid=pgid, leader_start=leader_start)
    tmp = _path(domain) + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(rec, f)
    os.replace(tmp, _path(domain))


def release(domain, run_key=None):
    rec = read(domain)
    if rec is not None and run_key is not None and rec.get('run') != run_key:
        return
    try:
        os.unlink(_path(domain))
    except OSError:
        pass


def free_count():
    n = 0
    for domain in range(DOMAIN_MIN, DOMAIN_MAX + 1):
        if not os.path.exists(_path(domain)) or not holder_alive(read(domain)):
            n += 1
    return n
