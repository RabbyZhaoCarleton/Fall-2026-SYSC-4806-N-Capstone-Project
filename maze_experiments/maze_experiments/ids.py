"""IDs for configurations and experiments: a timestamp plus a short random suffix.

Folders made on different PCs never collide, and IDs sort by creation time (DR3).
"""

import datetime
import re
import secrets


def slug(text, length=32):
    s = re.sub(r'[^A-Za-z0-9]+', '-', text or '').strip('-').lower()
    return s[:length].strip('-')


def new_id(name='', now=None):
    now = now or datetime.datetime.now()
    base = now.strftime('%Y%m%d-%H%M%S') + '-' + secrets.token_hex(2)
    s = slug(name)
    return f'{base}-{s}' if s else base


def run_id(set_index, maze_index, repeat, attempt):
    """Run IDs are deterministic inside an experiment, so a resume finds them again."""
    return f's{set_index:02d}-m{maze_index:03d}-r{repeat:02d}-a{attempt}'


def parse_run_id(rid):
    m = re.fullmatch(r's(\d+)-m(\d+)-r(\d+)-a(\d+)', rid)
    if not m:
        return None
    return tuple(int(g) for g in m.groups())
