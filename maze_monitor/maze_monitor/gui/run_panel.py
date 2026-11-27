"""Run panel: choose the settings of a simulation run, and start, stop, or restart it."""

import json

from python_qt_binding.QtCore import QSettings, Qt, Signal
from python_qt_binding.QtWidgets import (QApplication, QButtonGroup, QCheckBox, QComboBox,
                                         QDoubleSpinBox, QFileDialog, QFormLayout, QFrame,
                                         QGridLayout, QGroupBox, QHBoxLayout, QInputDialog,
                                         QLabel, QLineEdit, QMessageBox, QPlainTextEdit,
                                         QPushButton, QRadioButton, QScrollArea, QSizePolicy,
                                         QToolButton, QVBoxLayout, QWidget)

from . import style
from .params_dialog import ParamsDialog
from .. import param_files
from ..run_settings import AUTONOMY, PresetStore, RunSettings

NO_PRESET = '(last used)'


class PathEdit(QWidget):
    """A line edit with a browse button. Empty means the default."""

    changed = Signal()

    def __init__(self, placeholder, file_filter, parent=None):
        super().__init__(parent)
        self.file_filter = file_filter
        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        self.edit = QLineEdit()
        self.edit.setPlaceholderText(placeholder)
        self.edit.setClearButtonEnabled(True)
        self.edit.textChanged.connect(self.changed)
        row.addWidget(self.edit, 1)
        b = QToolButton()
        b.setText('…')
        b.setToolTip('Choose a file')
        b.clicked.connect(self._browse)
        row.addWidget(b)

    def _browse(self):
        start = self.edit.text() or self.edit.placeholderText()
        path, _ = QFileDialog.getOpenFileName(self, 'Choose a file', start, self.file_filter)
        if path:
            self.edit.setText(path)

    def text(self):
        return self.edit.text().strip()

    def setText(self, text):
        self.edit.setText(text or '')


class RunPanel(QWidget):
    start_requested = Signal()
    stop_requested = Signal()
    restart_requested = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.qsettings = QSettings('maze_monitor', 'maze_monitor')
        self.presets = PresetStore()
        self._overrides = {}
        self._loading = False
        self._running_settings = None   # settings of the active run, to spot later edits
        self.managed_elsewhere = None   # text when another program owns the run

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)

        # ---- state and buttons (always visible)
        head = QVBoxLayout()
        head.setContentsMargins(8, 6, 8, 4)
        self.state_label = QLabel()
        self.state_label.setFont(style.bold())
        head.addWidget(self.state_label)
        self.detail_label = QLabel()
        self.detail_label.setWordWrap(True)
        self.detail_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        head.addWidget(self.detail_label)
        self.ready_label = QLabel()
        self.ready_label.setWordWrap(True)
        head.addWidget(self.ready_label)
        buttons = QHBoxLayout()
        self.start_button = QPushButton('Start')
        self.start_button.setToolTip('Start the simulation with the settings below (F5)')
        self.start_button.clicked.connect(self.start_requested)
        self.stop_button = QPushButton('Stop')
        self.stop_button.setToolTip('Stop the simulation and save the run (Shift+F5)')
        self.stop_button.clicked.connect(self.stop_requested)
        self.restart_button = QPushButton('Restart')
        self.restart_button.setToolTip('Stop the run and start a new one with the settings below '
                                       '(Ctrl+F5)')
        self.restart_button.clicked.connect(self.restart_requested)
        for b in (self.start_button, self.stop_button, self.restart_button):
            buttons.addWidget(b)
        head.addLayout(buttons)
        self.pending_label = QLabel('Settings changed since the start: restart to apply them.')
        self.pending_label.setWordWrap(True)
        self.pending_label.setStyleSheet(f'color: {style.BLUE.name()};')
        self.pending_label.hide()
        head.addWidget(self.pending_label)
        outer.addLayout(head)
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        outer.addWidget(line)

        # ---- settings (scrolls)
        body = QWidget()
        form_layout = QVBoxLayout(body)
        form_layout.setContentsMargins(8, 4, 8, 8)

        presets = QHBoxLayout()
        presets.addWidget(QLabel('Preset'))
        self.preset_combo = QComboBox()
        self.preset_combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.preset_combo.setToolTip('Saved settings. Choosing one fills in the form below.')
        self.preset_combo.activated[int].connect(self._preset_chosen)
        presets.addWidget(self.preset_combo, 1)
        save = QToolButton()
        save.setText('Save…')
        save.setToolTip('Save the settings below as a preset')
        save.clicked.connect(self._save_preset)
        presets.addWidget(save)
        delete = QToolButton()
        delete.setText('Delete')
        delete.setToolTip('Delete the chosen preset')
        delete.clicked.connect(self._delete_preset)
        presets.addWidget(delete)
        form_layout.addLayout(presets)

        # robot behaviour
        box = QGroupBox('What the robot does')
        v = QVBoxLayout(box)
        self.autonomy = QButtonGroup(self)
        self.autonomy_buttons = {}
        for i, (key, text) in enumerate(AUTONOMY.items()):
            b = QRadioButton(text)
            self.autonomy.addButton(b, i)
            self.autonomy_buttons[key] = b
            v.addWidget(b)
        self.autonomy_buttons['manual'].setToolTip(
            'No Nav2: drive with the keyboard (Robot control panel) or teleop')
        self.autonomy_buttons['nav'].setToolTip(
            'Nav2 without explore_lite. You can still start exploring during the run.')
        self.autonomy_buttons['explore'].setToolTip(
            'Nav2 and explore_lite map the maze on their own (explore:=true)')
        self.autonomy.buttonClicked.connect(self._changed)
        self.shutdown_when_done = QCheckBox('Stop the run when exploration is done')
        self.shutdown_when_done.setToolTip(
            'shutdown_when_done:=true. explore_lite exits once the robot is back at its start; '
            'the whole run then stops and is saved.')
        v.addWidget(self.shutdown_when_done)
        form_layout.addWidget(box)

        # windows and world
        box = QGroupBox('Simulation')
        grid = QFormLayout(box)
        windows = QHBoxLayout()
        self.gui = QCheckBox('Gazebo window')
        self.gui.setToolTip('gui:=true. The simulation runs either way; the window costs '
                            'GPU time and can crash under WSL.')
        self.rviz = QCheckBox('RViz')
        self.rviz.setToolTip('rviz:=true')
        windows.addWidget(self.gui)
        windows.addWidget(self.rviz)
        windows.addStretch(1)
        grid.addRow('Windows', windows)
        self.world = PathEdit('Default maze (maze_sim/worlds/maze.sdf)', 'SDF worlds (*.sdf);;All files (*)')
        self.world.setToolTip('world:=… Make a new maze with maze_sim/scripts/make_maze.py.')
        grid.addRow('World', self.world)
        pose = QHBoxLayout()
        self.x_pose = self._spin(-50, 50, 0.05, ' m')
        self.y_pose = self._spin(-50, 50, 0.05, ' m')
        pose.addWidget(QLabel('x'))
        pose.addWidget(self.x_pose)
        pose.addWidget(QLabel('y'))
        pose.addWidget(self.y_pose)
        pose.addStretch(1)
        grid.addRow('Robot start', pose)
        form_layout.addWidget(box)

        # recording
        box = QGroupBox('Recording')
        grid = QFormLayout(box)
        self.record = QCheckBox('Save the run to the runs folder')
        self.record.setToolTip('Off: watch live without saving. The launch output is still kept '
                               'in ~/.cache/maze_monitor.')
        grid.addRow(self.record)
        self.label = QLineEdit()
        self.label.setPlaceholderText('Default: the mode (explore, nav, manual)')
        self.label.setToolTip('Added to the run folder name, for example "wider-inflation"')
        grid.addRow('Label', self.label)
        self.notes = QLineEdit()
        self.notes.setPlaceholderText('What you changed and what you expect')
        self.notes.setToolTip('Saved as the run notes. You can edit them later in Runs….')
        grid.addRow('Notes', self.notes)
        form_layout.addWidget(box)

        # parameters
        box = QGroupBox('Parameters')
        v = QVBoxLayout(box)
        row = QHBoxLayout()
        self.params_button = QPushButton('Change parameters…')
        self.params_button.setToolTip('Change single SLAM, Nav2, explore_lite, or EKF parameters '
                                      'for the next run, without editing the files')
        self.params_button.clicked.connect(self._edit_params)
        row.addWidget(self.params_button)
        self.params_summary = QLabel()
        self.params_summary.setWordWrap(True)
        row.addWidget(self.params_summary, 1)
        v.addLayout(row)
        form_layout.addWidget(box)

        # advanced (collapsed)
        self.advanced_toggle = QToolButton()
        self.advanced_toggle.setText('Advanced')
        self.advanced_toggle.setCheckable(True)
        self.advanced_toggle.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        self.advanced_toggle.setArrowType(Qt.RightArrow)
        self.advanced_toggle.setStyleSheet('QToolButton { border: none; font-weight: bold; }')
        form_layout.addWidget(self.advanced_toggle)
        self.advanced = QWidget()
        grid = QFormLayout(self.advanced)
        grid.setContentsMargins(0, 0, 0, 0)
        comps = QGridLayout()
        self.slam = QCheckBox('SLAM')
        self.slam.setToolTip('slam:=true (slam_toolbox)')
        self.ekf = QCheckBox('EKF')
        self.ekf.setToolTip('ekf:=true: odometry from wheels and IMU gyro (robot_localization)')
        self.use_sim_time = QCheckBox('Simulation time')
        self.use_sim_time.setToolTip('use_sim_time:=true: nodes use the Gazebo clock')
        comps.addWidget(self.slam, 0, 0)
        comps.addWidget(self.ekf, 0, 1)
        comps.addWidget(self.use_sim_time, 1, 0, 1, 2)
        grid.addRow('Components', comps)
        self.param_paths = {}
        for key, (arg, rel, title) in param_files.PARAM_FILES.items():
            edit = PathEdit(f'Default ({rel.split("/")[-1]})', 'YAML files (*.yaml *.yml);;All files (*)')
            edit.setToolTip(f'{arg}:=… The whole parameter file for {title}.')
            self.param_paths[key] = edit
            grid.addRow(title.split(' (')[0] + ' file', edit)
        self.extra_args = QLineEdit()
        self.extra_args.setPlaceholderText('name:=value …')
        self.extra_args.setToolTip('More launch arguments, as on the command line. '
                                   '"ros2 launch maze_sim maze.launch.py --show-args" lists them.')
        grid.addRow('More arguments', self.extra_args)
        self.command_view = QPlainTextEdit()
        self.command_view.setReadOnly(True)
        self.command_view.setFont(style.mono_font(-1))
        self.command_view.setMaximumHeight(90)
        self.command_view.setToolTip('The command the monitor runs. Changed parameters are passed '
                                     'as files in the run folder.')
        copy = QPushButton('Copy command')
        copy.clicked.connect(lambda: QApplication.clipboard().setText(self.command_view.toPlainText()))
        grid.addRow('Command', self.command_view)
        grid.addRow('', copy)
        self.advanced.hide()
        self.advanced_toggle.toggled.connect(self._toggle_advanced)
        form_layout.addWidget(self.advanced)
        form_layout.addStretch(1)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setWidget(body)
        outer.addWidget(scroll, 1)

        for w in (self.shutdown_when_done, self.gui, self.rviz, self.record, self.slam, self.ekf,
                  self.use_sim_time):
            w.toggled.connect(self._changed)
        for w in (self.x_pose, self.y_pose):
            w.valueChanged.connect(self._changed)
        for w in (self.label, self.notes, self.extra_args):
            w.textChanged.connect(self._changed)
        self.world.changed.connect(self._changed)
        for edit in self.param_paths.values():
            edit.changed.connect(self._changed)

        self._reload_presets()
        self.set_settings(self._last_used())
        if self.qsettings.value('run/advanced', 'false') == 'true':
            self.advanced_toggle.setChecked(True)
        self.set_state('Idle', 'Choose the settings and click Start.', style.BLACK, active=False)

    @staticmethod
    def _spin(lo, hi, step, suffix):
        s = QDoubleSpinBox()
        s.setRange(lo, hi)
        s.setSingleStep(step)
        s.setDecimals(2)
        s.setSuffix(suffix)
        return s

    def _toggle_advanced(self, on):
        self.advanced.setVisible(on)
        self.advanced_toggle.setArrowType(Qt.DownArrow if on else Qt.RightArrow)
        self.qsettings.setValue('run/advanced', 'true' if on else 'false')

    # ------------------------------------------------------------ settings
    def settings(self):
        s = RunSettings()
        for key, b in self.autonomy_buttons.items():
            if b.isChecked():
                s.autonomy = key
        s.shutdown_when_done = self.shutdown_when_done.isChecked()
        s.gui, s.rviz = self.gui.isChecked(), self.rviz.isChecked()
        s.slam, s.ekf = self.slam.isChecked(), self.ekf.isChecked()
        s.use_sim_time = self.use_sim_time.isChecked()
        s.world = self.world.text()
        s.x_pose, s.y_pose = self.x_pose.value(), self.y_pose.value()
        s.params = {k: e.text() for k, e in self.param_paths.items() if e.text()}
        s.overrides = {k: dict(v) for k, v in self._overrides.items() if v}
        s.extra_args = self.extra_args.text().strip()
        s.record = self.record.isChecked()
        s.label = self.label.text().strip()
        s.notes = self.notes.text().strip()
        return s

    def set_settings(self, s):
        self._loading = True
        try:
            self.autonomy_buttons.get(s.autonomy, self.autonomy_buttons['explore']).setChecked(True)
            self.shutdown_when_done.setChecked(s.shutdown_when_done)
            self.gui.setChecked(s.gui)
            self.rviz.setChecked(s.rviz)
            self.slam.setChecked(s.slam)
            self.ekf.setChecked(s.ekf)
            self.use_sim_time.setChecked(s.use_sim_time)
            self.world.setText(s.world)
            self.x_pose.setValue(s.x_pose)
            self.y_pose.setValue(s.y_pose)
            for key, edit in self.param_paths.items():
                edit.setText(s.params.get(key, ''))
            self._overrides = {k: dict(v) for k, v in s.overrides.items()}
            self.extra_args.setText(s.extra_args)
            self.record.setChecked(s.record)
            self.label.setText(s.label)
            self.notes.setText(s.notes)
        finally:
            self._loading = False
        self._changed()

    def _last_used(self):
        try:
            return RunSettings.from_dict(json.loads(self.qsettings.value('run/last_settings', '') or '{}'))
        except (TypeError, ValueError):
            return RunSettings()

    def remember(self):
        self.qsettings.setValue('run/last_settings', json.dumps(self.settings().to_dict()))

    def _changed(self, *_):
        if self._loading:
            return
        s = self.settings()
        self.shutdown_when_done.setEnabled(s.explore)
        n = s.override_count()
        if n:
            per = [f'{param_files.PARAM_FILES[k][2].split(" (")[0]}: {len(v)}'
                   for k, v in s.overrides.items() if v]
            self.params_summary.setText(f'<b>{n} changed</b> ({", ".join(per)})')
        else:
            self.params_summary.setText('The files in maze_navigation, unchanged.')
        problems = s.problems()
        if problems:
            self.command_view.setPlainText('\n'.join(problems))
        else:
            text = s.shell_command()
            if n:
                text += '\n# plus the changed parameter files, written to the run folder'
            self.command_view.setPlainText(text)
        self._update_pending()

    def _update_pending(self):
        running = self._running_settings
        if running is None:
            self.pending_label.hide()
            return
        a, b = running.to_dict(), self.settings().to_dict()
        for d in (a, b):
            d.pop('notes', None)
            d.pop('label', None)
        self.pending_label.setVisible(a != b)

    # ------------------------------------------------------------ parameters
    def _edit_params(self):
        s = self.settings()
        dlg = ParamsDialog(s.params, s.overrides, self)
        if dlg.exec_():
            self._overrides = dlg.overrides()
            self._changed()

    # ------------------------------------------------------------ presets
    def _reload_presets(self, select=None):
        self.preset_combo.clear()
        self.preset_combo.addItem(NO_PRESET)
        for name in self.presets.names():
            self.preset_combo.addItem(name)
        if select:
            i = self.preset_combo.findText(select)
            if i >= 0:
                self.preset_combo.setCurrentIndex(i)

    def _preset_chosen(self, index):
        if index <= 0:
            self.set_settings(self._last_used())
            return
        name = self.preset_combo.itemText(index)
        try:
            self.set_settings(self.presets.load(name))
        except (OSError, ValueError, KeyError) as exc:
            QMessageBox.warning(self, 'Cannot load preset', f'{name}: {exc}')

    def _save_preset(self):
        current = self.preset_combo.currentText()
        name, ok = QInputDialog.getText(self, 'Save preset', 'Preset name:',
                                        text='' if current == NO_PRESET else current)
        if not ok or not name.strip():
            return
        name = name.strip()
        if name in self.presets.names() and name != current:
            if QMessageBox.question(self, 'Save preset', f'Replace the preset "{name}"?') \
                    != QMessageBox.Yes:
                return
        try:
            path = self.presets.save(name, self.settings())
        except OSError as exc:
            QMessageBox.warning(self, 'Cannot save preset', str(exc))
            return
        self._reload_presets(select=name)
        self.detail_label.setText(f'Saved preset to {path}')

    def _delete_preset(self):
        if self.preset_combo.currentIndex() <= 0:
            return
        name = self.preset_combo.currentText()
        if QMessageBox.question(self, 'Delete preset', f'Delete the preset "{name}"?') \
                != QMessageBox.Yes:
            return
        self.presets.delete(name)
        self._reload_presets()

    def load_preset(self, name):
        """Fill the form from a preset (used by --preset). Returns False if there is none."""
        if name not in self.presets.names():
            return False
        self._reload_presets(select=name)
        self.set_settings(self.presets.load(name))
        return True

    # ------------------------------------------------------------ state
    def set_running_settings(self, settings):
        self._running_settings = settings
        self._update_pending()

    def set_state(self, title, detail, color, active, can_start=True, ready=''):
        self.state_label.setText(title)
        self.state_label.setStyleSheet(f'color: {color.name()};')
        self.detail_label.setText(detail)
        self.ready_label.setText(ready)
        self.ready_label.setVisible(bool(ready))
        elsewhere = self.managed_elsewhere is not None
        self.start_button.setEnabled(not active and can_start and not elsewhere)
        self.stop_button.setEnabled(active and not elsewhere)
        self.restart_button.setEnabled(can_start and not elsewhere)
        self.start_button.setDefault(not active)
