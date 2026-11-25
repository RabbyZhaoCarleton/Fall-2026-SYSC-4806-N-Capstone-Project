"""Main window: modes (live, replay), recording, playback, and layout."""

import datetime
import os
import time

from python_qt_binding.QtCore import QElapsedTimer, QSettings, Qt, QTimer, QUrl
from python_qt_binding.QtGui import QDesktopServices, QKeySequence
from python_qt_binding.QtWidgets import (QApplication, QButtonGroup, QComboBox, QDockWidget,
                                         QFileDialog, QLabel, QMainWindow, QMessageBox,
                                         QProgressDialog, QSizePolicy, QStackedWidget, QToolBar,
                                         QToolButton, QWidget)

from . import style
from .analysis_view import AnalysisView
from .events_panel import EventsPanel
from .raw_view import RawView
from .run_manager import RunManager
from .topics_panel import TopicsPanel
from .transport import Transport
from ..analysis.diagnosis import Units
from ..clock import now_ns
from ..recorder import Recorder
from ..run_store import RunStore
from ..session import LoadCancelled, Session, load_run

TICK_MS = 50
RENDER_EVERY = 0.1       # seconds between redraws of the main view
PANELS_EVERY = 0.5       # seconds between refreshes of the side panels
SUMMARY_EVERY = 10.0     # seconds between run.json summary updates while recording
DRAIN_BUDGET = 0.04      # seconds per tick spent on incoming messages


class Options:
    def __init__(self, live=True, record=True, label='', launch_args=None, replay=None):
        self.live, self.record, self.label = live, record, label
        self.launch_args, self.replay = launch_args or {}, replay


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

        self.setWindowTitle('Maze monitor')
        self._build_ui()
        if not self.restoreGeometry(self.settings.value('window/geometry', b'') or b''):
            self.resize(1600, 950)
        if not self.restoreState(self.settings.value('window/state', b'') or b''):
            QTimer.singleShot(0, lambda: self.resizeDocks(
                [self.topics_dock, self.events_dock], [330, 400], Qt.Horizontal))

        if self.bridge is not None:
            self.live = Session(config, 'live', None, start_ns=now_ns())
            self.view = self.live
            if options.record:
                self.start_recording()
        if options.replay:
            QTimer.singleShot(0, lambda: self.open_run(options.replay))
        self._session_changed()

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
        tb.addAction('Runs…', self.show_runs).setToolTip('Manage saved runs (Ctrl+R)')
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
        self.mode_label.setStyleSheet(f'color: {(style.RED if v is not None and v.mode == "live" and recording else style.BLACK).name()};')
        live_ok = self.bridge is not None
        self.rec_action.setVisible(live_ok)
        self.rec_action.setText('Stop recording' if recording else 'Start recording')
        self.new_run_action.setVisible(live_ok and recording)
        self.live_action.setVisible(live_ok and v is not self.live)
        self.setWindowTitle(f'Maze monitor · {self.mode_label.text()}')

    # ================================================================ recording
    def start_recording(self):
        if self.bridge is None or self.recorder is not None:
            return
        start_ns = now_ns()
        run = self.store.create(self.options.label, self.options.launch_args,
                                start=datetime.datetime.fromtimestamp(start_ns / 1e9))
        run.update(start_ns=start_ns)
        try:
            self.recorder = Recorder(run.bag_path)
        except Exception as exc:
            QMessageBox.critical(self, 'Cannot record', f'Could not create the recording:\n{exc}')
            run.update(status='failed to start')
            return
        was_live = self.view is self.live
        self.live = Session(self.config, 'live', run, start_ns=start_ns)
        self.bridge.set_recorder(self.recorder)
        if was_live or self.view is None:
            self.view = self.live
            self.cursor, self.following, self.playing = 0.0, True, False
        self._last_summary = time.monotonic()
        self._session_changed()
        self.statusBar().showMessage(f'Recording to {run.path}', 8000)

    def stop_recording(self, switch_view=True):
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
            run.update(summary=session.summary(), status='complete',
                       ended=datetime.datetime.now().isoformat(timespec='seconds'))
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
        dlg.exec_()

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

    # ================================================================ shutdown
    def closeEvent(self, ev):
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
            self.stop_recording(switch_view=False)
        finally:
            if self.bridge is not None:
                self.bridge.stop()
