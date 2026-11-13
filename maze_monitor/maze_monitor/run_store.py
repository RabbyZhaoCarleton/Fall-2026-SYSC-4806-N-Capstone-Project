"""Run folders on disk: create, list, update metadata, export, and delete.

Each run is one folder in the runs directory:

    <runs_dir>/<run id>/run.json   metadata, notes, tags, star, and summary
    <runs_dir>/<run id>/bag/       rosbag2 recording (SQLite 3)
"""

import datetime
import json
import os
import re
import shutil

RUN_FILE = 'run.json'
BAG_DIR = 'bag'


def _safe(text):
    return re.sub(r'[^A-Za-z0-9_.-]+', '-', text).strip('-')[:40]


def folder_size(path):
    total = 0
    for root, _, files in os.walk(path):
        for f in files:
            try:
                total += os.path.getsize(os.path.join(root, f))
            except OSError:
                pass
    return total


class RunInfo:
    """Metadata of one run, as stored in run.json."""

    def __init__(self, path, meta):
        self.path = path
        self.meta = meta

    @property
    def id(self):
        return os.path.basename(self.path)

    @property
    def name(self):
        return self.meta.get('name') or self.id

    @property
    def bag_path(self):
        return os.path.join(self.path, BAG_DIR)

    @property
    def summary(self):
        return self.meta.get('summary') or {}

    def get(self, key, default=None):
        return self.meta.get(key, default)

    def save(self):
        tmp = os.path.join(self.path, RUN_FILE + '.tmp')
        with open(tmp, 'w') as f:
            json.dump(self.meta, f, indent=2, sort_keys=True)
        os.replace(tmp, os.path.join(self.path, RUN_FILE))

    def update(self, **fields):
        self.meta.update(fields)
        self.save()


class RunStore:
    def __init__(self, runs_dir):
        self.runs_dir = runs_dir

    def create(self, label='', launch_args=None, start=None):
        os.makedirs(self.runs_dir, exist_ok=True)
        start = start or datetime.datetime.now()
        base = start.strftime('%Y-%m-%d_%H-%M-%S') + (f'_{_safe(label)}' if label else '')
        path = os.path.join(self.runs_dir, base)
        n = 1
        while os.path.exists(path):
            n += 1
            path = os.path.join(self.runs_dir, f'{base}_{n}')
        os.makedirs(path)
        info = RunInfo(path, {
            'format': 1,
            'name': os.path.basename(path),
            'label': label,
            'started': start.isoformat(timespec='seconds'),
            'ended': None,
            'status': 'recording',
            'launch_args': launch_args or {},
            'notes': '',
            'tags': [],
            'starred': False,
            'summary': {},
        })
        info.save()
        return info

    def load(self, path):
        path = os.path.abspath(os.path.expanduser(path))
        meta_file = os.path.join(path, RUN_FILE)
        meta = {}
        if os.path.isfile(meta_file):
            try:
                with open(meta_file) as f:
                    meta = json.load(f)
            except (OSError, ValueError):
                meta = {'status': 'damaged metadata'}
        return RunInfo(path, meta)

    def list(self):
        if not os.path.isdir(self.runs_dir):
            return []
        runs = []
        for entry in os.listdir(self.runs_dir):
            path = os.path.join(self.runs_dir, entry)
            if os.path.isdir(os.path.join(path, BAG_DIR)) or os.path.isfile(os.path.join(path, RUN_FILE)):
                runs.append(self.load(path))
        runs.sort(key=lambda r: r.get('started') or r.id, reverse=True)
        return runs

    def resolve(self, ref):
        """Find a run by path, folder name, or 'latest'."""
        if ref == 'latest':
            runs = self.list()
            return runs[0] if runs else None
        for candidate in (ref, os.path.join(self.runs_dir, ref)):
            candidate = os.path.abspath(os.path.expanduser(candidate))
            if os.path.isdir(candidate):
                # Accept the bag folder itself, too.
                if os.path.basename(candidate) == BAG_DIR:
                    candidate = os.path.dirname(candidate)
                return self.load(candidate)
        return None

    def rename(self, info, new_name):
        """Change the display name. The folder name stays, so paths stay valid."""
        info.update(name=new_name.strip() or info.id)

    def delete(self, info):
        if os.path.dirname(os.path.abspath(info.path)) != os.path.abspath(self.runs_dir):
            raise ValueError(f'Refusing to delete a folder outside the runs directory: {info.path}')
        shutil.rmtree(info.path)

    def export(self, info, target_zip):
        """Write the run folder to a .zip file. Returns the file path."""
        base = target_zip[:-4] if target_zip.endswith('.zip') else target_zip
        return shutil.make_archive(base, 'zip', root_dir=os.path.dirname(info.path),
                                   base_dir=os.path.basename(info.path))

    def total_size(self):
        return folder_size(self.runs_dir) if os.path.isdir(self.runs_dir) else 0
