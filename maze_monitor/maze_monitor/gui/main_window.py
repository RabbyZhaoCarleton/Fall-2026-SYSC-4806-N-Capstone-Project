"""Main window: run control, modes (live, replay), recording, playback, and layout."""

import datetime
import os
import shlex
import tempfile
import time

from python_qt_binding.QtCore import QElapsedTimer, QSettings, Qt, QTimer, QUrl
from python_qt_binding.QtGui import QDesktopServices, QKeySequence
from python_qt_binding.QtWidgets import (QAction, QApplication, QButtonGroup, QComboBox,
                                         QDockWidget, QFileDialog, QLabel, QMainWindow,
                                         QMessageBox, QProgressDialog, QSizePolicy,
                                         QStackedWidget, QToolBar, QToolButton, QWidget)

from . import style
from .analysis_view import AnalysisView
from .events_panel import EventsPanel
from .launch_log import LaunchLog
from .raw_view import RawView
from .robot_control import KeyboardDriver, RobotControlPanel
from .run_manager import RunManager
from .run_panel import RunPanel
from .topics_panel import TopicsPanel
from .transport import Transport
from ..analysis.diagnosis import Units
from .. import launcher, param_files
from ..clock import now_ns
from ..recorder import Recorder
from ..run_settings import RunSettings
from ..run_store import RunStore
from ..session import LoadCancelled, Session, load_run

TICK_MS = 50
RENDER_EVERY = 0.1       # seconds between redraws of the main view
PANELS_EVERY = 0.5       # seconds between refreshes of the side panels
SUMMARY_EVERY = 10.0     # seconds between run.json summary updates while recording
DRAIN_BUDGET = 0.04      # seconds per tick spent on incoming messages


# A launch that ends by itself this soon after starting failed to start.
FAILED_START_S = 15.0
EXPLORE_STATES = {
    'exploration_started': 'exploring',
    'exploration_in_progress': 'exploring',
    'exploration_paused': 'paused',
    'exploration_complete': 'done: the whole maze is explored',
    'exploration_incomplete': 'done: some frontiers could not be reached',
    'returning_to_origin': 'returning to the start',
    'returned_to_origin': 'back at the start',
    'return_failed': 'could not return to the start',
}


class Options:
    """How the monitor was started.

    record: record the live data at once (the run is not started from the monitor).
    attached: started by maze.launch.py for that one run, so the Run panel can't start or stop it.
    """

    def __init__(self, live=True, record=False, label='', launch_args=None, replay=None,
                 attached=False, start=False, preset=None):
        self.live, self.record, self.label = live, record, label
        self.launch_args, self.replay = launch_args or {}, replay
        self.attached, self.start, self.preset = attached, start, preset


class MainWindow(QMainWindow):
    def __init__(self, config, options, bridge=None):
        super().__init__()
        self.config = config
        self.options = options
        self.store = RunStore(config.runs_dir)
        self.settings = QSettings('maze_monitor', 'maze_monitor')
        self.bridge = bridge
        self.live = None          # live Session (fed from ROS), or None
        self.view = None          # Session on screen (live or replay)
        self.recorder = None
        self.cursor = 0.0
        self.playing = False
        self.following = True
        self.units = Units(self.settings.value('view/degrees', 'true') == 'true')
        self._last_render = 0.0
        self._last_panels = 0.0
        self._last_summary = time.monotonic()
        self._clock = QElapsedTimer()
        self._clock.start()
        self._last_tick_ms = self._clock.elapsed()
        self._shutdown_done = False
        lc = config.launch
        self.launch = launcher.ManagedProcess('launch', float(lc.get('stop_timeout', 20.0)),
                                              float(lc.get('term_timeout', 5.0)))
        self.explore_proc = launcher.ManagedProcess('explore', 10.0, 3.0)
        self.managed = None           # the run started from the Run panel (dict), or None
        self._restart_pending = False  # start a new run once the current one has stopped
        self._last_end = None         # (ended_by, status, run, exit code) of the last run
        self._quit_after_stop = False
        self.quit_without_asking = False   # set on SIGINT/SIGTERM
        self._paused_for_driving = False

        self.setWindowTitle('Maze monitor')
        self._build_ui()
        if not self.restoreGeometry(self.settings.value('window/geometry', b'') or b''):
            self.resize(1600, 950)
        if not self.restoreState(self.settings.value('window/state', b'') or b''):
            QTimer.singleShot(0, lambda: (
                self.resizeDocks([self.run_dock, self.events_dock], [360, 400], Qt.Horizontal),
                self.resizeDocks([self.control_dock, self.log_dock], [330, 160], Qt.Vertical)))

        if self.bridge is not None:
            self.live = Session(config, 'live', None, start_ns=now_ns())
            self.view = self.live
            if options.record:
                self.start_recording(options.label, options.launch_args)
        if options.attached:
            self.run_panel.managed_elsewhere = (
                'This monitor was opened by maze.launch.py and records that run; stop it with '
                'Ctrl+C in its terminal. To start, stop, and restart runs from here, start the '
                'monitor on its own: ros2 run maze_monitor maze_monitor')
        if options.preset and not self.run_panel.load_preset(options.preset):
            QTimer.singleShot(0, lambda: QMessageBox.warning(
                self, 'Preset not found', f'There is no preset named "{options.preset}".'))
        if options.replay:
            QTimer.singleShot(0, lambda: self.open_run(options.replay))
        elif options.start and not options.attached:
            QTimer.singleShot(300, self.start_run)
        self._session_changed()
        self._update_run_state()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(TICK_MS)

    # ================================================================ UI
    def _build_ui(self):
        self.setDockNestingEnabled(True)
        # ---- toolbar
        tb = QToolBar('Main')
        tb.setObjectName('main_toolbar')
        tb.setMovable(False)
        self.addToolBar(Qt.TopToolBarArea, tb)
        self.mode_label = QLabel()
        self.mode_label.setFont(style.bold())
        self.mode_label.setMinimumWidth(260)
        tb.addWidget(self.mode_label)
        tb.addSeparator()
        self.start_action = tb.addAction('Start', self.start_run)
        self.start_action.setShortcut(QKeySequence('F5'))
        self.start_action.setToolTip('Start a simulation run with the settings in the Run panel (F5)')
        self.stop_action = tb.addAction('Stop', self.stop_run)
        self.stop_action.setShortcut(QKeySequence('Shift+F5'))
        self.stop_action.setToolTip('Stop the simulation and save the run (Shift+F5)')
        self.restart_action = tb.addAction('Restart', self.restart_run)
        self.restart_action.setShortcut(QKeySequence('Ctrl+F5'))
        self.restart_action.setToolTip('Stop the run and start a new one with the settings in the '
                                       'Run panel (Ctrl+F5)')
        self.drive_action = tb.addAction('Drive')
        self.drive_action.setCheckable(True)
        self.drive_action.setShortcut(QKeySequence('F2'))
        self.drive_action.setToolTip('Drive the robot with W A S D or the arrow keys (F2)')
        self.drive_action.toggled.connect(self.set_driving)
        self.pause_action = QAction('Pause or resume exploring', self)
        self.pause_action.setShortcut(QKeySequence('F3'))
        self.pause_action.triggered.connect(self.toggle_exploration_pause)
        self.addAction(self.pause_action)
        tb.addSeparator()
        self.view_group = QButtonGroup(self)
        for i, name in enumerate(('Analysis', 'Raw')):
            b = QToolButton()
            b.setText(name)
            b.setCheckable(True)
            b.setToolTip(f'Show the {name.lower()} view (Ctrl+{i + 1})')
            self.view_group.addButton(b, i)
            tb.addWidget(b)
        self.view_group.buttonClicked[int].connect(self.set_view_mode)
        tb.addSeparator()
        tb.addWidget(QLabel('Angles '))
        self.angle_combo = QComboBox()
        self.angle_combo.addItems(('Degrees', 'Radians'))
        self.angle_combo.setCurrentIndex(0 if self.units.degrees else 1)
        self.angle_combo.currentIndexChanged.connect(self._units_changed)
        self.angle_combo.setToolTip('Unit for headings and turn rates (Ctrl+U)')
        tb.addWidget(self.angle_combo)
        tb.addSeparator()
        self.rec_action = tb.addAction('Stop recording', self.toggle_recording)
        self.new_run_action = tb.addAction('New run', self.new_run)
        self.new_run_action.setToolTip('Save the current run and start a new one, for example after '
                                       'resetting the simulation')
        self.live_action = tb.addAction('Back to live', self.back_to_live)
        tb.addSeparator()
        tb.addAction('Runs…', self.show_runs).setToolTip('Saved runs: replay, compare, run again (Ctrl+R)')
        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        tb.addWidget(spacer)
        self.rec_label = QLabel()
        tb.addWidget(self.rec_label)

        # ---- central views
        self.stack = QStackedWidget()
        self.analysis_view = AnalysisView(self.config)
        self.raw_view = RawView()
        self.stack.addWidget(self.analysis_view)
        self.stack.addWidget(self.raw_view)
        self.analysis_view.seek_requested.connect(self.seek)
        self.setCentralWidget(self.stack)

        # ---- docks
        self.topics = TopicsPanel(self.config)
        self.topics.visibility_changed.connect(self._render_now)
        self.topics_dock = QDockWidget('Topics')
        self.topics_dock.setObjectName('topics_dock')
        self.topics_dock.setWidget(self.topics)
        self.addDockWidget(Qt.LeftDockWidgetArea, self.topics_dock)
        self.events = EventsPanel()
        self.events.seek_requested.connect(self.seek)
        self.events_dock = QDockWidget('Events')
        self.events_dock.setObjectName('events_dock')
        self.events_dock.setWidget(self.events)
        self.addDockWidget(Qt.RightDockWidgetArea, self.events_dock)

        self.run_panel = RunPanel()
        self.run_panel.start_requested.connect(self.start_run)
        self.run_panel.stop_requested.connect(self.stop_run)
        self.run_panel.restart_requested.connect(self.restart_run)
        self.run_dock = QDockWidget('Run')
        self.run_dock.setObjectName('run_dock')
        self.run_dock.setWidget(self.run_panel)
        self.addDockWidget(Qt.LeftDockWidgetArea, self.run_dock)
        self.tabifyDockWidget(self.run_dock, self.topics_dock)
        self.run_dock.raise_()

        self.control = RobotControlPanel(self.config.teleop)
        self.control.set_units(self.units.degrees)
        self.control.pause_requested.connect(self.set_exploration_paused)
        self.control.start_exploring_requested.connect(self.start_exploring)
        self.control.stop_exploring_requested.connect(self.explore_proc.stop)
        self.control.drive_toggled.connect(self.set_driving)
        self.control_dock = QDockWidget('Robot control')
        self.control_dock.setObjectName('control_dock')
        self.control_dock.setWidget(self.control)
        self.addDockWidget(Qt.RightDockWidgetArea, self.control_dock)
        self.splitDockWidget(self.control_dock, self.events_dock, Qt.Vertical)
        self.driver = KeyboardDriver(self._send_velocity, float(self.config.teleop.get('rate_hz', 20.0)),
                                     self)
        self.driver.command_changed.connect(self.control.show_command)
        self.control.linear.valueChanged.connect(self._speeds_changed)
        self.control.angular.valueChanged.connect(self._speeds_changed)
        self._speeds_changed()

        self.log = LaunchLog()
        self.log_dock = QDockWidget('Launch output')
        self.log_dock.setObjectName('log_dock')
        self.log_dock.setWidget(self.log)
        self.addDockWidget(Qt.BottomDockWidgetArea, self.log_dock)

        # ---- transport
        self.transport = Transport()
        self.transport.seek_requested.connect(self.seek)
        self.transport.play_toggled.connect(self.toggle_play)
        self.transport.step_requested.connect(self.step)
        self.transport.event_jump.connect(self.jump_event)
        self.transport.live_requested.connect(self.follow_live)
        tdock = QDockWidget('Timeline')
        tdock.setObjectName('timeline_dock')
        tdock.setWidget(self.transport)
        tdock.setFeatures(QDockWidget.NoDockWidgetFeatures)
        tdock.setTitleBarWidget(QWidget())
        self.addDockWidget(Qt.BottomDockWidgetArea, tdock)

        # ---- menus
        mb = self.menuBar()
        m = mb.addMenu('&Run')
        m.addAction(self.start_action)
        m.addAction(self.stop_action)
        m.addAction(self.restart_action)
        m.addSeparator()
        m.addAction(self.drive_action)
        m.addAction(self.pause_action)
        m.addSeparator()
        m.addAction('Stop leftover simulations…', self.stop_leftovers)
        m = mb.addMenu('&File')
        m.addAction('Open run folder…', self.open_run_dialog, QKeySequence('Ctrl+O'))
        m.addAction('Runs…', self.show_runs, QKeySequence('Ctrl+R'))
        m.addSeparator()
        m.addAction('Quit', self.close, QKeySequence('Ctrl+Q'))
        m = mb.addMenu('&View')
        m.addAction('Analysis view', lambda: self.set_view_mode(0), QKeySequence('Ctrl+1'))
        m.addAction('Raw view', lambda: self.set_view_mode(1), QKeySequence('Ctrl+2'))
        m.addAction('Toggle degrees and radians', self._toggle_units, QKeySequence('Ctrl+U'))
        m.addSeparator()
        m.addAction(self.run_dock.toggleViewAction())
        m.addAction(self.control_dock.toggleViewAction())
        m.addAction(self.log_dock.toggleViewAction())
        m.addAction(self.topics_dock.toggleViewAction())
        m.addAction(self.events_dock.toggleViewAction())
        m = mb.addMenu('&Playback')
        for text, fn, key in (
                ('Play or pause', self.toggle_play, 'Space'),
                ('Step back', lambda: self.step(-self.transport.step.value()), 'Left'),
                ('Step forward', lambda: self.step(self.transport.step.value()), 'Right'),
                ('Back 0.1 s', lambda: self.step(-0.1), 'Shift+Left'),
                ('Forward 0.1 s', lambda: self.step(0.1), 'Shift+Right'),
                ('Previous problem', lambda: self.jump_event(-1), 'Ctrl+Left'),
                ('Next problem', lambda: self.jump_event(1), 'Ctrl+Right'),
                ('Slower', lambda: self.transport.change_speed(-1), '['),
                ('Faster', lambda: self.transport.change_speed(1), ']'),
                ('Go to start', lambda: self.seek(0.0), 'Home'),
                ('Go to end', lambda: self.seek(1e12), 'End'),
                ('Follow live', self.follow_live, 'L')):
            act = m.addAction(text, fn)
            act.setShortcut(QKeySequence(key))
        m = mb.addMenu('&Help')
        m.addAction('User manual', self._open_manual, QKeySequence('F1'))
        m.addAction('About the colors', self._about_colors)

        self.set_view_mode(int(self.settings.value('view/mode', 0)))
        self.statusBar()

    def _open_manual(self):
        path = None
        try:
            from ament_index_python.packages import get_package_share_directory
            path = os.path.join(get_package_share_directory('maze_monitor'), 'docs', 'user-manual.md')
        except Exception:
            pass
        if not path or not os.path.isfile(path):
            path = os.path.join(os.path.dirname(__file__), '..', '..', 'docs', 'user-manual.md')
        QDesktopServices.openUrl(QUrl.fromLocalFile(os.path.abspath(path)))
        self.statusBar().showMessage(f'User manual: {os.path.abspath(path)}', 10000)

    def _about_colors(self):
        QMessageBox.information(self, 'Colors', style.__doc__.split('\n', 2)[2])

    # ================================================================ modes
    def set_view_mode(self, index):
        index = 0 if index not in (0, 1) else index
        self.stack.setCurrentIndex(index)
        self.view_group.button(index).setChecked(True)
        self.settings.setValue('view/mode', index)
        self._render_now()

    def _units_changed(self, index):
        self.units = Units(index == 0)
        self.control.set_units(self.units.degrees)
        self.settings.setValue('view/degrees', 'true' if self.units.degrees else 'false')
        self._render_now()

    def _toggle_units(self):
        self.angle_combo.setCurrentIndex(1 - self.angle_combo.currentIndex())

    def _session_changed(self):
        self.events.set_timeline(self.view.timeline if self.view is not None else None)
        self._update_mode_label()
        self._render_now()

    def _update_mode_label(self):
        v = self.view
        recording = self.recorder is not None
        if v is None:
            self.mode_label.setText('No data: open a run')
        elif v.mode == 'live':
            self.mode_label.setText('LIVE · ' + (f'recording {v.run.name}' if recording and v.run else 'not recording'))
        else:
            self.mode_label.setText(f'REPLAY · {v.title}')
        if self.driver.enabled:
            self.mode_label.setText('DRIVING · ' + self.mode_label.text())
        self.mode_label.setStyleSheet(f'color: {(style.RED if v is not None and v.mode == "live" and recording else style.BLACK).name()};')
        live_ok = self.bridge is not None
        # A run started from the monitor owns its recording; the buttons are for
        # recording a system started some other way (the real robot, a terminal).
        manual_rec = live_ok and self.managed is None and not self.launch.active
        self.rec_action.setVisible(manual_rec)
        self.rec_action.setText('Stop recording' if recording else 'Start recording')
        self.new_run_action.setVisible(manual_rec and recording)
        self.live_action.setVisible(live_ok and v is not self.live)
        self.setWindowTitle(f'Maze monitor · {self.mode_label.text()}')

    # ================================================================ recording
    def start_recording(self, label=None, launch_args=None, run=None, show_live=False):
        """Record the live data to a new run (or to run, a run folder made for it)."""
        if self.bridge is None or self.recorder is not None:
            return False
        start_ns = now_ns()
        if run is None:
            run = self.store.create(self.options.label if label is None else label,
                                    self.options.launch_args if launch_args is None else launch_args,
                                    start=datetime.datetime.fromtimestamp(start_ns / 1e9))
        run.update(start_ns=start_ns, status='recording')
        try:
            self.recorder = Recorder(run.bag_path)
        except Exception as exc:
            QMessageBox.critical(self, 'Cannot record', f'Could not create the recording:\n{exc}')
            run.update(status='failed to start')
            return False
        was_live = self.view is self.live
        self.live = Session(self.config, 'live', run, start_ns=start_ns)
        self.bridge.set_recorder(self.recorder)
        if was_live or self.view is None or show_live:
            self.view = self.live
            self.cursor, self.following, self.playing = 0.0, True, False
        self._last_summary = time.monotonic()
        self._session_changed()
        self.statusBar().showMessage(f'Recording to {run.path}', 8000)
        return True

    def stop_recording(self, switch_view=True, status='complete', **fields):
        if self.recorder is None:
            return
        old = self.bridge.set_recorder(None)
        self._drain(budget=1.0)
        if old is not None:
            old.close()
        self.recorder = None
        session = self.live
        end_t = session.now()
        session.analyzer.finish(end_t)
        run = session.run
        if run is not None:
            run.update(summary=session.summary(), status=status,
                       ended=datetime.datetime.now().isoformat(timespec='seconds'), **fields)
        # The finished run stays on screen as a replay; live data continues in a new,
        # unrecorded session that "Back to live" shows.
        session.mode = 'replay'
        self.live = Session(self.config, 'live', None, start_ns=now_ns())
        if switch_view and self.view is session:
            self.following = False
            self.playing = False
            self.cursor = min(self.cursor, session.timeline.duration)
        self._session_changed()
        if run is not None:
            self.statusBar().showMessage(f'Saved run {run.name}', 8000)

    def toggle_recording(self):
        if self.recorder is not None:
            self.stop_recording()
        else:
            self.start_recording()

    def new_run(self):
        self.stop_recording(switch_view=False)
        self.view = None
        self.start_recording()

    # ================================================================ run control
    def start_run(self):
        """Start the simulation with the Run panel's settings, recording it as a new run."""
        if self.options.attached or self.launch.active:
            return
        if self.bridge is None:
            QMessageBox.warning(self, 'Not connected to ROS 2',
                                'The monitor is not connected to ROS 2 (--no-ros, or ROS 2 failed '
                                'to start), so it would not see the run. Start it without --no-ros.')
            return
        settings = self.run_panel.settings()
        problems = settings.problems()
        if problems:
            QMessageBox.warning(self, 'Cannot start the run', '\n'.join(problems))
            return
        if not self._clear_leftovers():
            return
        self.run_panel.remember()
        # A recording of a system started some other way ends here; it stays saved.
        self.stop_recording(switch_view=False)
        self.set_driving(False)

        start = datetime.datetime.now()
        run = None
        if settings.record:
            run = self.store.create(settings.run_label(), {}, start=start)
            work_dir = run.path
        else:
            work_dir = os.path.join(os.path.expanduser('~/.cache/maze_monitor'), 'unrecorded')
            os.makedirs(work_dir, exist_ok=True)
            work_dir = tempfile.mkdtemp(prefix=start.strftime('%Y-%m-%d_%H-%M-%S_'), dir=work_dir)
        try:
            files = param_files.prepare_run_files(work_dir, settings.params, settings.overrides)
        except (OSError, ValueError) as exc:
            QMessageBox.warning(self, 'Cannot start the run', f'Parameter files: {exc}')
            if run is not None:
                run.update(status='failed to start', ended=start.isoformat(timespec='seconds'))
            return
        command = settings.command(files)
        if run is not None:
            run.update(launch_args=settings.launch_args_record(files), notes=settings.notes,
                       run_settings=settings.to_dict(), command=shlex.join(command),
                       started_from='monitor')
            if not self.start_recording(run=run, show_live=True):
                return
        else:
            self.view = self.live = Session(self.config, 'live', None, start_ns=now_ns())
            self.cursor, self.following, self.playing = 0.0, True, False
            self._session_changed()
        log_path = os.path.join(work_dir, 'launch.log')
        try:
            self.launch.start(command, log_path=log_path)
        except OSError as exc:
            QMessageBox.critical(self, 'Cannot start the run',
                                 f'Could not run ros2 ({exc}). Start the monitor from a terminal '
                                 'where ROS 2 is set up (source ~/ros2_ws/install/setup.bash).')
            self.stop_recording(switch_view=False, status='failed to start')
            return
        self.managed = {'settings': settings, 'run': run, 'files': files, 'dir': work_dir,
                        'explore_started_here': False}
        self.run_panel.set_running_settings(settings)
        self.log.start(log_path, f'$ {shlex.join(command)}')
        self.log_dock.show()
        self.statusBar().showMessage(
            f'Started the simulation{" and recording to " + run.path if run else ""}', 8000)
        self._update_run_state()

    def stop_run(self):
        if not self.launch.active:
            return
        self.set_driving(False)
        self.explore_proc.stop()
        self.launch.stop()
        self.statusBar().showMessage('Stopping the simulation (Ctrl+C)…', 5000)
        self._update_run_state()

    def restart_run(self):
        if self.options.attached:
            return
        if self.launch.active:
            self._restart_pending = True
            self.stop_run()
        else:
            self.start_run()

    def _clear_leftovers(self):
        """Offer to stop simulations left over from earlier runs. False: don't start."""
        own = {self.launch.pid} if self.launch.active else set()
        found = launcher.find_leftovers(exclude_pgids=own)
        if not found:
            return True
        names = '\n'.join(f'  {pid}: {cmd[:110]}' for pid, cmd in found[:10])
        answer = QMessageBox.question(
            self, 'A simulation is still running',
            'These processes are left over from an earlier run (or run in another terminal). '
            'A second Gazebo would publish its own /clock, /odom and /scan, and maze.launch.py '
            f'refuses to start next to one.\n\n{names}\n\nStop them and start the run?',
            QMessageBox.Yes | QMessageBox.Cancel, QMessageBox.Yes)
        if answer != QMessageBox.Yes:
            return False
        return self._stop_pids([pid for pid, _ in found])

    def _stop_pids(self, pids):
        QApplication.setOverrideCursor(Qt.WaitCursor)
        try:
            alive = launcher.stop_processes(pids)
        finally:
            QApplication.restoreOverrideCursor()
        if alive:
            QMessageBox.warning(self, 'Processes still running',
                                f'Could not stop: {", ".join(map(str, alive))}')
            return False
        return True

    def stop_leftovers(self):
        own = {self.launch.pid} if self.launch.active else set()
        found = launcher.find_leftovers(exclude_pgids=own)
        if not found:
            QMessageBox.information(self, 'No leftovers', 'No Gazebo or maze_sim launch is running '
                                    'outside the run started here.')
            return
        names = '\n'.join(f'  {pid}: {cmd[:110]}' for pid, cmd in found[:10])
        if QMessageBox.question(self, 'Stop leftover simulations', f'Stop these processes?\n\n{names}') \
                == QMessageBox.Yes:
            self._stop_pids([pid for pid, _ in found])

    def _poll_launch(self):
        for proc in (self.launch, self.explore_proc):
            if proc.active:
                proc.poll()
                lines = proc.drain()
                if lines:
                    self.log.append(lines)
        if self.managed is not None and not self.launch.active:
            self._finish_run()

    def _finish_run(self):
        managed, self.managed = self.managed, None
        self.set_driving(False)
        if self.explore_proc.active:
            self.explore_proc.stop_and_wait(5.0)
        code = self.launch.returncode
        elapsed = self.launch.elapsed()
        if self.launch.user_stopped:
            ended_by, status = 'stopped from the monitor', 'complete'
        elif code not in (0, None) and elapsed < FAILED_START_S:
            ended_by, status = f'the launch failed to start (exit code {code})', 'launch failed'
        else:
            ended_by, status = f'the launch ended by itself (exit code {code})', 'complete'
        self.log.append([f'--- {ended_by} after {style.fmt_time(elapsed)} ---'], level_override='')
        run = managed['run']
        if run is not None:
            restarting = bool(self._restart_pending)
            self.stop_recording(switch_view=not restarting, status=status, ended_by=ended_by,
                                launch_exit_code=code)
        else:
            self.live = Session(self.config, 'live', None, start_ns=now_ns())
            if self.view is not None and self.view.mode == 'live':
                self.view = self.live
            self._session_changed()
        self.run_panel.set_running_settings(None)
        self._last_end = (ended_by, status, run, code)
        if self._shutdown_done:
            return
        if self._quit_after_stop:
            QTimer.singleShot(0, self.close)
            return
        if self._restart_pending:
            self._restart_pending = False
            QTimer.singleShot(200, self.start_run)
        elif status == 'launch failed':
            tail = '\n'.join(list(self.launch.tail)[-12:])
            QMessageBox.warning(self, 'The run did not start', f'{ended_by}.\n\nLast output:\n{tail}')
        self._update_run_state()

    def _update_run_state(self):
        p = self.run_panel
        managed = self.managed
        if self.options.attached:
            p.set_state('Run started by a launch file', p.managed_elsewhere, style.BLACK, active=False,
                        can_start=False)
        elif self.launch.active:
            name = managed['run'].name if managed and managed['run'] else 'not recorded'
            t = style.fmt_time(self.launch.elapsed())
            if self.launch.state == launcher.STOPPING:
                what = 'restarting' if self._restart_pending else 'stopping'
                p.set_state(f'Stopping… {t}', f'Waiting for the simulation to shut down, then '
                            f'{what}. {name}', style.AMBER, active=True)
            else:
                s = managed['settings'] if managed else None
                mode = s.autonomy if s else ''
                p.set_state(f'Running {t} · {mode}', f'Run: {name}', style.GREEN, active=True,
                            ready=self._readiness(s))
        elif self.bridge is None:
            p.set_state('Not connected to ROS 2', 'Replay only (--no-ros).', style.GRAY, active=False,
                        can_start=False)
        else:
            last = self._last_end
            if last is None:
                p.set_state('Idle', 'Choose the settings and click Start.', style.BLACK, active=False)
            else:
                ended_by, status, run, code = last
                color = style.RED if status == 'launch failed' else style.BLACK
                where = f' Saved as {run.name}.' if run is not None else ''
                p.set_state('Ended' if status != 'launch failed' else 'Failed to start',
                            f'{ended_by[0].upper()}{ended_by[1:]}.{where}', color, active=False)
        active = self.launch.active
        can = not self.options.attached and self.bridge is not None
        self.start_action.setEnabled(can and not active)
        self.stop_action.setEnabled(can and active)
        self.restart_action.setEnabled(can)
        self._update_exploration()

    def _readiness(self, settings):
        """'Gazebo ✓ · SLAM ✓ · Nav2 … · explore_lite …' for what this run starts."""
        if self.bridge is None or settings is None:
            return ''
        b, c = self.bridge, self.config.control
        clock = b.topics.get('/clock')
        parts = [('Gazebo', clock is not None and clock.publishers > 0)]
        if settings.slam:
            parts.append(('SLAM', b.has_node(c['slam_node'])))
        if settings.uses_nav2:
            parts.append(('Nav2', b.has_node(c['nav2_node'])))
        if settings.explore:
            parts.append(('explore_lite', b.has_node(c['explore_node'])))
        ok, wait = style.GREEN.name(), style.GRAY.name()
        return ' · '.join(f'<span style="color:{ok if up else wait}">{name} {"✓" if up else "…"}</span>'
                          for name, up in parts)

    # ================================================================ robot control
    def _explore_status(self):
        if self.live is None:
            return None
        return self.live.timeline.value('explore.status', self.live.now() + 1.0)

    def _update_exploration(self):
        if self.bridge is None:
            self.control.set_exploration('Not connected to ROS 2.', False, False, False, False)
            self.control.setEnabled(False)
            return
        c = self.config.control
        running = self.bridge.has_node(c['explore_node'])
        nav2 = self.bridge.has_node(c['nav2_node'])
        status = self._explore_status()
        paused = status == 'exploration_paused'
        here = self.explore_proc.active
        if running:
            text = 'explore_lite: ' + EXPLORE_STATES.get(status, status.replace('_', ' ')
                                                          if status else 'starting')
            if self._paused_for_driving and paused:
                text += ' (paused for driving)'
        elif here:
            text = 'Starting explore_lite…'
        elif nav2:
            text = 'explore_lite is not running. Nav2 is up, so you can start exploring.'
        else:
            text = 'explore_lite is not running, and neither is Nav2 (needed to explore).'
        self.control.set_exploration(text, running, paused, nav2 and not here, here)

    def set_exploration_paused(self, pause):
        if self.bridge is None:
            return
        self.bridge.set_exploring(not pause)
        if not pause:
            self._paused_for_driving = False
            if self.driver.enabled:
                self.set_driving(False)
        self.statusBar().showMessage('Asked explore_lite to ' + ('pause.' if pause else 'resume.'),
                                     4000)

    def toggle_exploration_pause(self):
        if self.bridge is not None and self.bridge.has_node(self.config.control['explore_node']):
            self.set_exploration_paused(self._explore_status() != 'exploration_paused')

    def start_exploring(self):
        """Start explore_lite in a running simulation that has Nav2 but no explore_lite."""
        if self.explore_proc.active or self.bridge is None:
            return
        managed = self.managed
        files = managed['files'] if managed else {}
        params = files.get('explore') or param_files.resolve(
            'explore', managed['settings'].params.get('explore', '') if managed else '')
        sim_time = managed['settings'].use_sim_time if managed else True
        command = ['ros2', 'run', 'explore_lite', 'explore', '--ros-args',
                   '-r', f'__node:={self.config.control["explore_node"]}',
                   '--params-file', params, '-p', f'use_sim_time:={"true" if sim_time else "false"}']
        log_path = os.path.join(managed['dir'], 'launch.log') if managed else None
        try:
            self.explore_proc.start(command, log_path=log_path)
        except OSError as exc:
            QMessageBox.warning(self, 'Cannot start explore_lite', str(exc))
            return
        if managed is not None:
            managed['explore_started_here'] = True
            run = managed['run']
            if run is not None:
                run.update(explore_started_at_s=round(self.live.now(), 1))
        self.log.append([f'$ {shlex.join(command)}'], level_override='')
        self._update_exploration()

    def _speeds_changed(self, *_):
        self.driver.linear = self.control.linear_speed()
        self.driver.angular = self.control.angular_speed()

    def _send_velocity(self, v, w):
        if self.bridge is not None:
            self.bridge.send_velocity(v, w)

    def set_driving(self, on):
        on = bool(on) and self.bridge is not None
        if on and not self.driver.enabled and self.control.pause_while_driving.isChecked():
            c = self.config.control
            if self.bridge.has_node(c['explore_node']) and self._explore_status() != 'exploration_paused':
                self.bridge.set_exploring(False)
                self._paused_for_driving = True
            self.bridge.cancel_navigation()
        self.driver.set_enabled(on)
        for w in (self.drive_action, self.control.drive_button):
            if w.isChecked() != on:
                w.blockSignals(True)
                w.setChecked(on)
                w.blockSignals(False)
        if on:
            from .robot_control import _is_text_input
            focus = QApplication.focusWidget()
            if _is_text_input(focus):
                focus.clearFocus()  # the keys would type into it instead of driving
            self.statusBar().showMessage('Driving: W A S D or arrow keys, Space stops, F2 ends.', 6000)
        self.control.show_command(0.0, 0.0)
        self._update_mode_label()

    def back_to_live(self):
        if self.live is None:
            return
        self.view = self.live
        self.following, self.playing = True, False
        self._session_changed()

    def current_run_path(self):
        if self.recorder is not None and self.live is not None and self.live.run is not None:
            return self.live.run.path
        return None

    # ================================================================ replay
    def show_runs(self):
        dlg = RunManager(self.store, self.current_run_path, self)
        dlg.open_requested.connect(self.open_run)
        dlg.settings_requested.connect(self.use_run_settings)
        dlg.exec_()

    def use_run_settings(self, path):
        """Fill the Run panel with the settings of a saved run."""
        run = self.store.resolve(path)
        if run is None:
            return
        if run.get('run_settings'):
            settings = RunSettings.from_dict(run.get('run_settings'))
        else:  # started from a terminal: rebuild the settings from its launch arguments
            settings = RunSettings.from_launch_args(run.get('launch_args') or {})
            settings.label = run.get('label', '') or ''
        settings.notes = ''
        self.run_panel.set_settings(settings)
        self.run_dock.show()
        self.run_dock.raise_()
        self.statusBar().showMessage(f'Settings of {run.name} are in the Run panel; '
                                     'click Start to run them again.', 8000)

    def open_run_dialog(self):
        path = QFileDialog.getExistingDirectory(self, 'Open run folder', self.config.runs_dir)
        if path:
            self.open_run(path)

    def open_run(self, ref):
        run = self.store.resolve(ref)
        if run is None:
            QMessageBox.warning(self, 'Run not found', f'No run found at "{ref}".')
            return
        if self.current_run_path() and os.path.abspath(run.path) == os.path.abspath(self.current_run_path()):
            self.back_to_live()
            return
        dlg = QProgressDialog(f'Loading {run.name}…', 'Cancel', 0, 100, self)
        dlg.setWindowModality(Qt.WindowModal)
        dlg.setMinimumDuration(300)

        def progress(done, total):
            if total:
                dlg.setMaximum(total)
                dlg.setValue(min(done, total))
            dlg.setLabelText(f'Loading {run.name}… {done:,} messages')
            QApplication.processEvents()
            # Keep recording while loading: drain live messages, too.
            self._drain(budget=0.01)
            return not dlg.wasCanceled()

        try:
            session = load_run(self.config, run, progress)
        except LoadCancelled:
            dlg.close()
            return
        except Exception as exc:
            dlg.close()
            QMessageBox.warning(self, 'Cannot open run', f'{run.path}\n\n{exc}')
            return
        dlg.close()
        self.view = session
        self.cursor, self.playing, self.following = 0.0, False, False
        self._session_changed()
        self.statusBar().showMessage(
            f'Opened {run.name}: {len(session.timeline.events)} events, '
            f'{style.fmt_time(session.timeline.duration)} long', 8000)

    # ================================================================ playback
    def duration(self):
        v = self.view
        if v is None:
            return 0.0
        if v.mode == 'live':
            return max(v.timeline.duration, v.now() - 0.2, 0.0)
        return v.timeline.duration

    def seek(self, t):
        d = self.duration()
        self.cursor = min(max(t, 0.0), d)
        if self.view is not None and self.view.mode == 'live':
            self.following = self.cursor >= d - 0.25
        self._render_now()

    def step(self, dt):
        self.seek(self.cursor + dt)

    def toggle_play(self):
        if self.view is None:
            return
        if self.following and self.view.mode == 'live':
            self.following = False  # pause the live view
            self.playing = False
        elif not self.playing and self.cursor >= self.duration() - 0.05 and self.view.mode == 'replay':
            self.cursor = 0.0
            self.playing = True
        else:
            self.playing = not self.playing
        self._render_now()

    def follow_live(self):
        if self.view is not None and self.view.mode == 'live':
            self.following = True
            self.playing = False
            self._render_now()
        elif self.live is not None:
            self.back_to_live()

    def jump_event(self, direction):
        if self.view is None:
            return
        tl = self.view.timeline
        t = tl.next_event_time(self.cursor) if direction > 0 else tl.prev_event_time(self.cursor)
        if t is None:
            self.statusBar().showMessage('No more warnings or errors in that direction.', 3000)
            return
        self.seek(t)

    # ================================================================ loop
    def _drain(self, budget=DRAIN_BUDGET):
        if self.bridge is None or self.live is None:
            return
        q = self.bridge.queue
        deadline = time.monotonic() + budget
        live = self.live
        while time.monotonic() < deadline:
            try:
                topic, type_name, data, t_ns = q.get_nowait()
            except Exception:
                break
            live.ingest(topic, type_name, data, t_ns)

    def _tick(self):
        now_ms = self._clock.elapsed()
        dt = (now_ms - self._last_tick_ms) / 1000.0
        self._last_tick_ms = now_ms
        self._poll_launch()
        if self.bridge is not None and self.live is not None:
            self._drain()
            self.live.advance_live()
        v = self.view
        if v is not None:
            d = self.duration()
            if v.mode == 'live' and self.following:
                self.cursor = d
            elif self.playing:
                self.cursor += dt * self.transport.speed_value
                if self.cursor >= d:
                    self.cursor = d
                    if v.mode == 'live':
                        self.following = True
                    self.playing = False
        now = time.monotonic()
        if now - self._last_render >= RENDER_EVERY:
            self._render()
        if now - self._last_panels >= PANELS_EVERY:
            self._render_panels()
        if self.recorder is not None and now - self._last_summary >= SUMMARY_EVERY:
            self._last_summary = now
            try:
                self.live.run.update(summary=self.live.summary())
            except OSError:
                pass

    def _render_now(self):
        self._render()
        self._render_panels()

    def _render(self):
        self._last_render = time.monotonic()
        v = self.view
        vis = self.topics.is_visible
        if v is not None:
            try:
                if self.stack.currentIndex() == 0:
                    self.analysis_view.update_view(v, self.cursor, vis, self.units)
                else:
                    self.raw_view.update_view(v, self.cursor, vis)
            except Exception as exc:  # keep the monitor alive; show what failed
                self.statusBar().showMessage(f'Display error: {type(exc).__name__}: {exc}', 5000)
                import traceback
                traceback.print_exc()
        events = v.timeline.events if v is not None else []
        self.transport.set_state(self.duration(), self.cursor, events, self.playing,
                                 self.following and v is not None and v.mode == 'live',
                                 v is not None and v.mode == 'live')

    def _render_panels(self):
        self._last_panels = time.monotonic()
        v = self.view
        graph = self.bridge.topics if (self.bridge is not None and v is self.live) else {}
        problems = {}
        if v is not None:
            for e in v.timeline.active_events(self.cursor):
                if e.key.startswith('silent:'):
                    problems[e.key[7:]] = 'silent'
        self.topics.refresh(v, self.cursor, dict(graph), problems)
        self.events.refresh(self.cursor)
        if self.recorder is not None:
            self.rec_label.setText(f'{self.recorder.messages:,} messages · '
                                   f'{style.fmt_bytes(self.recorder.bytes)} recorded')
        else:
            self.rec_label.setText('')
        self._update_mode_label()
        self._update_run_state()

    # ================================================================ shutdown
    def closeEvent(self, ev):
        if self.launch.active:
            if self._quit_after_stop:
                # Closed again while stopping: don't wait any longer.
                if self.quit_without_asking or QMessageBox.question(
                        self, 'Still stopping', 'The simulation is still shutting down. '
                        'Force it to stop now?') == QMessageBox.Yes:
                    self.launch.kill()
                    self.explore_proc.kill()
                ev.ignore()
                return
            if not self.quit_without_asking:
                answer = QMessageBox.question(
                    self, 'Quit', 'A simulation run is active. Stop it, save the run, and quit?',
                    QMessageBox.Yes | QMessageBox.Cancel, QMessageBox.Yes)
                if answer != QMessageBox.Yes:
                    ev.ignore()
                    return
            # Stop without blocking the window; _finish_run closes it once the
            # simulation is gone.
            self._quit_after_stop = True
            self.stop_run()
            ev.ignore()
            return
        self.run_panel.remember()
        self.shutdown()
        super().closeEvent(ev)

    def shutdown(self):
        if self._shutdown_done:
            return
        self._shutdown_done = True
        self.settings.setValue('window/geometry', self.saveGeometry())
        self.settings.setValue('window/state', self.saveState())
        try:
            self.timer.stop()
        except AttributeError:
            pass
        try:
            self.driver.set_enabled(False)
            # Normally stopped already (closeEvent); this covers other ways out.
            for proc in (self.explore_proc, self.launch):
                if proc.active:
                    proc.stop_and_wait()
            if self.managed is not None:
                self._restart_pending = False
                self._finish_run()
            self.stop_recording(switch_view=False)
        finally:
            if self.bridge is not None:
                self.bridge.stop()
