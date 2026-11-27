"""The configuration editor (2.3.3, GR5).

Fields are checked as the user types. A maze parameter field takes one value
or a comma-separated list, which sweeps it (FR3). The editor lists the corridor
widths that fit the dimensions (AR6), shows a sample maze for the selected set
(AR14), and shows the number of runs and the estimates (CR9). Configurations
are never edited in place: saving always creates a new one.
"""

import copy

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QPixmap
from python_qt_binding.QtWidgets import (QComboBox, QDialog, QDialogButtonBox, QFileDialog,
                                         QFormLayout, QGridLayout, QGroupBox, QHBoxLayout,
                                         QLabel, QLineEdit, QListWidget, QMessageBox,
                                         QPlainTextEdit, QPushButton, QScrollArea, QSpinBox,
                                         QSplitter, QVBoxLayout, QWidget)

from .. import configuration, profiles, resources
from ..mazegen import MazeParams, generate, valid_corridor_widths
from ..mazegen.files import preview_png, walls_from_layout, layout_text
from ..metrics import fmt
from .common import background, button, heading, style

MAZE_FIELDS = [  # key, label, kind
    ('width', 'Width (m)', float), ('length', 'Length (m)', float),
    ('corridor_width', 'Corridor width (m)', float), ('wall_thickness', 'Wall thickness (m)', float),
    ('wall_height', 'Wall height (m)', float), ('exits', 'Exits', int),
    ('min_exit_separation', 'Min. exit separation (cells)', int),
    ('loop_fraction', 'Loop fraction (0–1)', float),
]
LIMIT_FIELDS = [('sim_timeout_s', 'Run timeout (s, sim time)'),
                ('startup_timeout_s', 'Startup timeout (s, wall time)'),
                ('stall_window_s', 'Stall window (s, sim time)'),
                ('stall_min_area_m2', 'Stall: min. new map area (m²)'),
                ('stall_min_move_m', 'Stall: min. movement (m)'),
                ('retries', 'Retries of non-robot failures'),
                ('min_rtf', 'Minimum real-time factor')]


def parse_value(text, kind):
    """'0.48' -> 0.48; '1, 2, 3' -> [1, 2, 3] (a sweep). Raises ValueError."""
    parts = [p.strip() for p in text.replace(';', ',').split(',') if p.strip()]
    if not parts:
        raise ValueError('empty')
    vals = [kind(float(p)) if kind is int else kind(p) for p in parts]
    if kind is int and any(float(p) != int(float(p)) for p in parts):
        raise ValueError('whole numbers only')
    return vals[0] if len(vals) == 1 else vals


def show_value(v):
    if isinstance(v, list):
        return ', '.join(f'{x:g}' if isinstance(x, float) else str(x) for x in v)
    return f'{v:g}' if isinstance(v, float) else str(v)


class ConfigEditor(QDialog):
    def __init__(self, win, config=None, view_only=False):
        super().__init__(win)
        self.win = win
        self.source = config
        self.view_only = view_only
        if config is None:
            self.data = configuration.normalize({'name': 'New configuration',
                                                 'maze_sets': [configuration.default_set()]})
        elif view_only:
            self.data = copy.deepcopy(config.data)
        else:
            self.data = configuration.normalize(win.suite.configs.copy_for_edit(config))
        self.calibration = None
        self.machine = None
        self.current_set = -1
        self.setWindowTitle('Configuration' if view_only else
                            'New configuration' if config is None else 'Create modified version')
        self.resize(1180, 720)
        outer = QVBoxLayout(self)
        if view_only:
            note = QLabel('Configurations are never edited, so that experiments always match '
                          'the configuration they used. Use "Create modified version" to change '
                          'one.')
            note.setWordWrap(True)
            note.setStyleSheet(f'color: {style.GRAY.name()}')
            outer.addWidget(note)
        split = QSplitter(Qt.Horizontal)
        outer.addWidget(split, 1)
        split.addWidget(self._general())
        split.addWidget(self._sets())
        split.addWidget(self._preview())
        split.setSizes([330, 470, 380])
        self.problems = QLabel('')
        self.problems.setWordWrap(True)
        self.problems.setTextInteractionFlags(Qt.TextSelectableByMouse)
        scroll = QScrollArea()
        scroll.setWidget(self.problems)
        scroll.setWidgetResizable(True)
        scroll.setMaximumHeight(110)
        outer.addWidget(scroll)
        bb = QDialogButtonBox(QDialogButtonBox.Close if view_only else QDialogButtonBox.Cancel)
        if view_only:
            mod = bb.addButton('Create modified version', QDialogButtonBox.ActionRole)
            mod.clicked.connect(self.modify)
        else:
            self.save_btn = bb.addButton('Save', QDialogButtonBox.AcceptRole)
            bb.accepted.connect(self.save)
        bb.rejected.connect(self.reject)
        outer.addWidget(bb)
        self.load_general()
        self.refresh_set_list()
        self.set_list.setCurrentRow(0)
        if view_only:
            for w in self.findChildren((QLineEdit, QSpinBox, QComboBox, QPlainTextEdit,
                                        QPushButton)):
                if w.parent() is not bb and not isinstance(w, QListWidget):
                    w.setEnabled(isinstance(w, QListWidget))
            self.set_list.setEnabled(True)
        background(self._measure, self._measured, on_error=lambda _t: None, parent=self)
        self.validate()

    # ---------------------------------------------------------------- layout
    def _general(self):
        box = QGroupBox('General')
        form = QFormLayout(box)
        self.name = QLineEdit()
        self.desc = QPlainTextEdit()
        self.desc.setMaximumHeight(70)
        self.profile = QComboBox()
        self.profile.addItems(sorted(profiles.available(self.win.suite.data_dir)))
        form.addRow('Name', self.name)
        form.addRow('Description', self.desc)
        form.addRow('Robot profile', self.profile)
        self.limits = {}
        for key, label in LIMIT_FIELDS:
            e = QLineEdit()
            e.textChanged.connect(lambda *_: self.validate())
            self.limits[key] = e
            form.addRow(label, e)
        self.rec_profile = QComboBox()
        self.rec_profile.addItems(configuration.RECORDING_PROFILES)
        self.rec_profile.setToolTip('metrics_only drops /scan and /imu (about a third of the '
                                    'disk), losing the lidar-based safety events')
        self.retention = QComboBox()
        self.retention.addItems(configuration.RETENTION)
        self.retention.setToolTip('Which recordings to keep after processing: all, only runs '
                                  'that were not mission successes, or none')
        self.tolerance = QLineEdit()
        self.tolerance.setPlaceholderText('one cell pitch')
        form.addRow('Recording profile', self.rec_profile)
        form.addRow('Keep recordings', self.retention)
        form.addRow('Exit match tolerance (m)', self.tolerance)
        for w in (self.name, self.tolerance):
            w.textChanged.connect(lambda *_: self.validate())
        self.profile.currentIndexChanged.connect(lambda *_: self.validate())
        return box

    def _sets(self):
        box = QGroupBox('Maze sets')
        lay = QVBoxLayout(box)
        self.set_list = QListWidget()
        self.set_list.setMaximumHeight(110)
        self.set_list.currentRowChanged.connect(self.select_set)
        lay.addWidget(self.set_list)
        h = QHBoxLayout()
        h.addWidget(button('Add', self.add_set))
        h.addWidget(button('Duplicate', self.dup_set))
        h.addWidget(button('Remove', self.remove_set))
        h.addStretch(1)
        lay.addLayout(h)
        form = QFormLayout()
        self.set_name = QLineEdit()
        form.addRow('Set name', self.set_name)
        self.maze_edits = {}
        for key, label, _kind in MAZE_FIELDS:
            e = QLineEdit()
            e.setToolTip('One value, or a comma-separated list to sweep it (one set per value)')
            self.maze_edits[key] = e
            form.addRow(label, e)
        self.valid_widths = QComboBox()
        self.valid_widths.setToolTip('Corridor widths whose cell pitch divides both dimensions')
        self.valid_widths.activated.connect(self.pick_width)
        form.insertRow(3, 'Valid corridor widths', self.valid_widths)
        self.exit_max = QLabel('')
        self.exit_max.setStyleSheet(f'color: {style.GRAY.name()}')
        form.addRow('', self.exit_max)
        self.start = QComboBox()
        self.start.addItems(['fixed', 'random'])
        self.start_col, self.start_row = QSpinBox(), QSpinBox()
        for s in (self.start_col, self.start_row):
            s.setRange(0, 99)
        sc = QHBoxLayout()
        sc.addWidget(self.start)
        sc.addWidget(QLabel('column'))
        sc.addWidget(self.start_col)
        sc.addWidget(QLabel('row'))
        sc.addWidget(self.start_row)
        form.addRow('Start cell', sc)
        self.mazes, self.repeats, self.seed = QSpinBox(), QSpinBox(), QSpinBox()
        self.mazes.setRange(1, 100000)
        self.repeats.setRange(1, 1000)
        self.seed.setRange(0, 2 ** 31 - 1)
        form.addRow('Mazes (M)', self.mazes)
        form.addRow('Repeats per maze (R)', self.repeats)
        form.addRow('Seed', self.seed)
        self.param_edits = {}
        for key in configuration.STACK_PARAM_KEYS:
            e = QLineEdit()
            e.setPlaceholderText("robot profile's default")
            pick = QPushButton('…')
            pick.setMaximumWidth(30)
            pick.clicked.connect(lambda _c=False, e=e: self.pick_file(e))
            h = QHBoxLayout()
            h.addWidget(e, 1)
            h.addWidget(pick)
            self.param_edits[key] = e
            form.addRow(key.replace('_params', ' params'), h)
        lay.addLayout(form)
        for w in [self.set_name, *self.maze_edits.values(), *self.param_edits.values()]:
            w.textChanged.connect(self.set_changed)
        for w in (self.start_col, self.start_row, self.mazes, self.repeats, self.seed):
            w.valueChanged.connect(self.set_changed)
        self.start.currentIndexChanged.connect(self.set_changed)
        return box

    def _preview(self):
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.addWidget(heading('Sample maze', 1))
        self.preview = QLabel('')
        self.preview.setAlignment(Qt.AlignCenter)
        self.preview.setMinimumSize(300, 300)
        self.preview.setStyleSheet('background: white; border: 1px solid #d8d8d8')
        lay.addWidget(self.preview, 1)
        self.preview_note = QLabel('')
        self.preview_note.setWordWrap(True)
        self.preview_note.setStyleSheet(f'color: {style.GRAY.name()}')
        lay.addWidget(self.preview_note)
        lay.addWidget(heading('Runs and estimates', 1))
        self.totals = QLabel('')
        self.totals.setWordWrap(True)
        lay.addWidget(self.totals)
        return w

    # ---------------------------------------------------------------- data <-> widgets
    def load_general(self):
        self._loading = True
        try:
            self._load_general()
        finally:
            self._loading = False

    def _load_general(self):
        d = self.data
        self.name.setText(d['name'])
        self.desc.setPlainText(d.get('description', ''))
        i = self.profile.findText(d['robot_profile'])
        self.profile.setCurrentIndex(max(i, 0))
        for key, e in self.limits.items():
            e.setText(show_value(d['run_limits'][key]))
        self.rec_profile.setCurrentText(d['recording']['profile'])
        self.retention.setCurrentText(d['recording']['retention'])
        tol = d['analysis'].get('exit_match_tolerance_m')
        self.tolerance.setText('' if tol is None else show_value(tol))

    def refresh_set_list(self):
        row = self.set_list.currentRow()
        self.set_list.blockSignals(True)
        self.set_list.clear()
        for s in self.data['maze_sets']:
            self.set_list.addItem(s['name'])
        self.set_list.blockSignals(False)
        if 0 <= row < self.set_list.count():
            self.set_list.setCurrentRow(row)

    def select_set(self, index):
        self.current_set = index
        if index < 0 or index >= len(self.data['maze_sets']):
            return
        s = self.data['maze_sets'][index]
        widgets = [self.set_name, *self.maze_edits.values(), *self.param_edits.values(),
                   self.start_col, self.start_row, self.mazes, self.repeats, self.seed,
                   self.start]
        for w in widgets:
            w.blockSignals(True)
        self.set_name.setText(s['name'])
        for key, e in self.maze_edits.items():
            e.setText(show_value(s['maze'][key]))
        self.start.setCurrentText(s['maze'].get('start', 'fixed'))
        cell = s['maze'].get('start_cell') or [0, 0]
        if cell and isinstance(cell[0], list):
            cell = cell[0]
        self.start_col.setValue(int(cell[0]))
        self.start_row.setValue(int(cell[1]))
        self.mazes.setValue(int(s['mazes']))
        self.repeats.setValue(int(s['repeats']))
        self.seed.setValue(int(s['seed']))
        for key, e in self.param_edits.items():
            e.setText((s.get('stack_params') or {}).get(key, ''))
        for w in widgets:
            w.blockSignals(False)
        self.validate()

    def set_changed(self, *_):
        i = self.current_set
        if i < 0:
            return
        s = self.data['maze_sets'][i]
        s['name'] = self.set_name.text()
        bad = []
        for key, label, kind in MAZE_FIELDS:
            e = self.maze_edits[key]
            try:
                s['maze'][key] = parse_value(e.text(), kind)
                e.setStyleSheet('')
            except ValueError:
                e.setStyleSheet(f'border: 1px solid {style.RED.name()}')
                bad.append(label)
        s['maze']['start'] = self.start.currentText()
        s['maze']['start_cell'] = [self.start_col.value(), self.start_row.value()]
        s['mazes'], s['repeats'], s['seed'] = (self.mazes.value(), self.repeats.value(),
                                               self.seed.value())
        s['stack_params'] = {k: e.text().strip() for k, e in self.param_edits.items()
                             if e.text().strip()}
        item = self.set_list.item(i)
        if item is not None:
            item.setText(s['name'])
        self.validate(bad)

    def collect(self):
        d = self.data
        d['name'] = self.name.text().strip()
        d['description'] = self.desc.toPlainText().strip()
        d['robot_profile'] = self.profile.currentText()
        for key, e in self.limits.items():
            try:
                d['run_limits'][key] = parse_value(e.text(), int if key == 'retries' else float)
                e.setStyleSheet('')
            except ValueError:
                e.setStyleSheet(f'border: 1px solid {style.RED.name()}')
        d['recording'] = {'profile': self.rec_profile.currentText(),
                          'retention': self.retention.currentText()}
        try:
            t = self.tolerance.text().strip()
            d['analysis']['exit_match_tolerance_m'] = float(t) if t else None
        except ValueError:
            pass
        return d

    # ---------------------------------------------------------------- checks
    def validate(self, bad_fields=()):
        if getattr(self, '_loading', False):
            return
        d = self.collect()
        errors, warnings = configuration.check(d, workspace=False)
        errors = [f'{f}: not a number (or a comma-separated list)' for f in bad_fields] + errors
        html = [f'<span style="color:{style.RED.name()}">✖ {e}</span>' for e in errors]
        html += [f'<span style="color:{style.AMBER.name()}">⚠ {w}</span>' for w in warnings]
        self.problems.setText('<br>'.join(html) if html else
                              f'<span style="color:{style.GREEN.name()}">✔ The configuration is '
                              f'valid.</span>')
        if not self.view_only:
            self.save_btn.setEnabled(not errors and bool(d['name']))
        self.update_set_hints()
        self.update_totals()

    def _first(self, v):
        return v[0] if isinstance(v, list) else v

    def update_set_hints(self):
        i = self.current_set
        if i < 0 or i >= len(self.data['maze_sets']):
            return
        m = self.data['maze_sets'][i]['maze']
        try:
            prof = profiles.load(self.data['robot_profile'], self.win.suite.data_dir)
            minimum = prof.min_corridor_width
        except Exception:
            prof, minimum = None, 0.0
        try:
            w, l_, t = (float(self._first(m['width'])), float(self._first(m['length'])),
                        float(self._first(m['wall_thickness'])))
            widths = valid_corridor_widths(w, l_, t, minimum=minimum, maximum=3.0)
        except (TypeError, ValueError):
            widths = []
        self.valid_widths.clear()
        self.valid_widths.addItems([f'{c:.4g} m (pitch {c + t:.4g} m, {round(w / (c + t))} x '
                                    f'{round(l_ / (c + t))} cells)' for c in widths]
                                   if widths else ['none fit these dimensions'])
        self._widths = widths
        try:
            cur = float(self._first(m['corridor_width']))
            match = [i for i, c in enumerate(widths) if abs(c - cur) < 1e-3]
            if match:
                self.valid_widths.setCurrentIndex(match[0])
            elif widths:
                self.valid_widths.insertItem(0, f'{cur:g} m does not fit: pick one below')
                self.valid_widths.setCurrentIndex(0)
                self._widths = [None] + widths
        except (TypeError, ValueError):
            pass
        self.render_preview(prof)

    def pick_width(self, index):
        if getattr(self, '_widths', None) and index < len(self._widths) \
                and self._widths[index] is not None:
            self.maze_edits['corridor_width'].setText(f'{self._widths[index]:.4g}')

    def render_preview(self, prof):
        i = self.current_set
        s = self.data['maze_sets'][i]
        try:
            params = MazeParams.from_dict({k: self._first(v) if k != 'start_cell' else v
                                           for k, v in s['maze'].items()})
            from ..mazegen.params import validate
            errors, _ = validate(params, prof)
            if errors:
                raise ValueError(errors[0])
            from ..mazegen.generator import derive_seed
            from ..mazegen.mazeset import generate_checked
            maze, _ = generate_checked(params, derive_seed(int(s['seed']), 0))
            text = layout_text(maze)
            png = preview_png(maze, walls_from_layout(text, params.pitch, params.wall_thickness))
            pix = QPixmap()
            pix.loadFromData(png, 'PNG')
            side = max(min(self.preview.width(), self.preview.height(), 420) - 6, 200)
            self.preview.setPixmap(pix.scaled(side, side, Qt.KeepAspectRatio,
                                              Qt.SmoothTransformation))
            from ..mazegen.params import max_exits
            limit = max_exits(params.columns, params.rows, params.min_exit_separation,
                              tuple(params.start_cell) if params.start == 'fixed' else (0, 0),
                              params.allow_exit_in_start_cell)
            self.exit_max.setText(f'{params.columns} x {params.rows} cells; at most {limit} exits')
            self.preview_note.setText(f'Maze 0 of this set (seed {maze.seed}). Black walls, green '
                                      f'exits, blue start.')
        except Exception as exc:
            self.preview.setPixmap(QPixmap())
            self.preview.setText('No preview')
            self.preview_note.setText(str(exc)[:300])
            self.exit_max.setText('')

    def update_totals(self):
        d = self.data
        try:
            sets = configuration.expand_sets(d)
            runs = configuration.total_runs(d)
        except Exception:
            self.totals.setText('')
            return
        text = f'{len(sets)} maze set(s) after sweeps, {runs} runs.'
        if self.calibration and self.machine:
            fake = configuration.Configuration('', d)
            alloc = resources.default_allocation(self.machine)
            est = resources.estimate(fake, self.calibration, {**alloc, 'max_concurrency': None},
                                     self.machine)
            if est:
                text += (f' With the default allocation (half of this machine): about '
                         f'{fmt.long_duration(est["duration_s"])} at up to '
                         f'{est["concurrency"]} at once, {fmt.size(est["disk_bytes"])} of disk.')
        elif self.machine:
            text += ' Estimates after the first run (no calibration on this machine yet).'
        self.totals.setText(text)

    def _measure(self):
        m = resources.measure_machine(self.win.suite.data_dir)
        cal = resources.load_calibration(self.win.suite.data_dir, m, self.data['robot_profile'])
        return m, cal

    def _measured(self, res):
        self.machine, self.calibration = res
        self.update_totals()

    # ---------------------------------------------------------------- actions
    def add_set(self):
        self.data['maze_sets'].append(configuration.default_set(
            f'set {len(self.data["maze_sets"]) + 1}'))
        self.refresh_set_list()
        self.set_list.setCurrentRow(len(self.data['maze_sets']) - 1)

    def dup_set(self):
        i = self.current_set
        if i < 0:
            return
        s = copy.deepcopy(self.data['maze_sets'][i])
        s['name'] += ' (copy)'
        s['seed'] = int(s['seed']) + 1
        self.data['maze_sets'].append(s)
        self.refresh_set_list()
        self.set_list.setCurrentRow(len(self.data['maze_sets']) - 1)

    def remove_set(self):
        i = self.current_set
        if i < 0 or len(self.data['maze_sets']) <= 1:
            return
        del self.data['maze_sets'][i]
        self.refresh_set_list()
        self.set_list.setCurrentRow(min(i, len(self.data['maze_sets']) - 1))
        self.validate()

    def pick_file(self, edit):
        path, _ = QFileDialog.getOpenFileName(self, 'Parameter file', '', 'YAML (*.yaml *.yml)')
        if path:
            edit.setText(path)

    def modify(self):
        self.reject()
        self.win.edit_config(self.source)

    def save(self):
        d = self.collect()
        try:
            cfg, warnings = self.win.suite.configs.create(
                d, derived_from=self.source.id if self.source is not None else None)
        except configuration.ConfigError as exc:
            QMessageBox.warning(self, 'Cannot save', '\n'.join(exc.problems))
            return
        except Exception as exc:
            QMessageBox.warning(self, 'Cannot save', str(exc))
            return
        self.win.statusBar().showMessage(f'Saved configuration "{cfg.name}" ({cfg.id}).', 8000)
        self.accept()
