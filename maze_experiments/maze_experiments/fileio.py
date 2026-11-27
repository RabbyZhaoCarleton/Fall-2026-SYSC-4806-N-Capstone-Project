"""Writing and reading the suite's files.

Every write is atomic (temporary file, then rename), like maze_monitor's
run_store.py, so a crash or a kill never leaves a half-written file (DR7).
Files that grow while an experiment runs (*.jsonl) are only ever appended to,
one complete line per write. Every JSON and YAML file carries a
schema_version (DR5); a .jsonl file carries it in its first line.
"""

import json
import os
import tempfile

import yaml

SCHEMA_VERSION = 1


class SchemaError(ValueError):
    """A file was written by a newer (or unknown) version of the suite."""


def check_schema(data, path, supported=SCHEMA_VERSION):
    version = data.get('schema_version') if isinstance(data, dict) else None
    if version is None:
        raise SchemaError(f'{path}: no schema_version; this file was not written by '
                          f'maze_experiments (or is damaged)')
    if not isinstance(version, int) or version > supported:
        raise SchemaError(f'{path}: schema_version {version} needs a newer maze_experiments '
                          f'(this one reads up to {supported}). Update the repository and '
                          f'rebuild.')
    return version


def _atomic_write(path, text):
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=directory, prefix='.' + os.path.basename(path), suffix='.tmp')
    try:
        with os.fdopen(fd, 'w') as f:
            f.write(text)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise


def write_bytes(path, data):
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=directory, prefix='.' + os.path.basename(path), suffix='.tmp')
    try:
        with os.fdopen(fd, 'wb') as f:
            f.write(data)
        os.replace(tmp, path)
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise


def write_text(path, text):
    _atomic_write(path, text)


def _with_schema(data):
    if isinstance(data, dict) and 'schema_version' not in data:
        data = {'schema_version': SCHEMA_VERSION, **data}
    return data


def write_json(path, data):
    _atomic_write(path, json.dumps(_with_schema(data), indent=2, sort_keys=False,
                                   default=_json_default) + '\n')


def write_yaml(path, data, header=''):
    text = yaml.safe_dump(_with_schema(data), sort_keys=False, default_flow_style=False,
                          allow_unicode=True)
    _atomic_write(path, header + text)


def _json_default(obj):
    if hasattr(obj, 'tolist'):
        return obj.tolist()
    if isinstance(obj, (set, tuple)):
        return list(obj)
    return str(obj)


def read_json(path, check=True):
    with open(path) as f:
        data = json.load(f)
    if check:
        check_schema(data, path)
    return data


def read_yaml(path, check=True):
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    if check:
        check_schema(data, path)
    return data


def append_jsonl(path, record, kind=None):
    """Append one record (one line). A new file starts with a header line."""
    new = not os.path.exists(path)
    line = json.dumps(record, default=_json_default, separators=(',', ':')) + '\n'
    if new:
        header = {'schema_version': SCHEMA_VERSION, 'type': 'header', 'kind': kind or
                  os.path.splitext(os.path.basename(path))[0]}
        line = json.dumps(header, separators=(',', ':')) + '\n' + line
    # One write() of one buffer in append mode: concurrent appenders never interleave
    # inside a line, and a crash loses at most the line being written.
    fd = os.open(path, os.O_WRONLY | os.O_APPEND | os.O_CREAT, 0o644)
    try:
        os.write(fd, line.encode())
    finally:
        os.close(fd)


def read_jsonl(path, since=0):
    """Records of a .jsonl file (header line skipped). A torn last line is ignored.

    since: byte offset to start at; read_jsonl_from returns the new offset too.
    """
    return read_jsonl_from(path, since)[0]


def read_jsonl_from(path, offset=0):
    records = []
    try:
        with open(path, 'rb') as f:
            f.seek(offset)
            data = f.read()
    except FileNotFoundError:
        return records, offset
    end = data.rfind(b'\n')
    if end < 0:
        return records, offset
    for raw in data[:end].split(b'\n'):
        if not raw.strip():
            continue
        try:
            rec = json.loads(raw)
        except ValueError:
            continue
        if isinstance(rec, dict) and rec.get('type') == 'header':
            check_schema(rec, path)
            continue
        records.append(rec)
    return records, offset + end + 1


def tail_jsonl(path, max_bytes=65536):
    """The last records of a .jsonl file, reading at most max_bytes from its end."""
    try:
        size = os.path.getsize(path)
    except OSError:
        return []
    start = max(0, size - max_bytes)
    with open(path, 'rb') as f:
        f.seek(start)
        data = f.read()
    if start > 0:
        nl = data.find(b'\n')
        data = data[nl + 1:] if nl >= 0 else b''
    out = []
    for raw in data.split(b'\n'):
        if not raw.strip():
            continue
        try:
            rec = json.loads(raw)
        except ValueError:
            continue
        if isinstance(rec, dict) and rec.get('type') != 'header':
            out.append(rec)
    return out
