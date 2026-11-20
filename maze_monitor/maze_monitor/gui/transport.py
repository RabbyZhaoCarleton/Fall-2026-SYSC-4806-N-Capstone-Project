"""Timeline bar and playback controls."""

from python_qt_binding.QtCore import QPointF, QRectF, QSettings, Qt, Signal
from python_qt_binding.QtGui import QFontMetrics, QPainter
from python_qt_binding.QtWidgets import (QComboBox, QDoubleSpinBox, QHBoxLayout, QLabel,
                                         QPushButton, QSizePolicy, QStyle, QToolButton,
                                         QVBoxLayout, QWidget)

from . import style
from .widgets import pen

SPEEDS = (0.1, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0)


class TimelineBar(QWidget):
    """The whole run as a bar: event marks, condition spans, and the cursor. Click or drag to seek."""

    seek_requested = Signal(float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(34)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.setMouseTracking(True)
        self.duration = 1.0
        self.cursor = 0.0
        self.events = []
        self.dragging = False

    def set_state(self, duration, cursor, events):
        self.duration = max(duration, 1e-3)
        self.cursor = cursor
        self.events = events
        self.update()

    def _x(self, t):
        return 6 + (self.width() - 12) * min(max(t / self.duration, 0.0), 1.0)

    def _t(self, x):
        return min(max((x - 6) / max(self.width() - 12, 1), 0.0), 1.0) * self.duration

    def mousePressEvent(self, ev):
        if ev.button() == Qt.LeftButton:
            self.dragging = True
            self.seek_requested.emit(self._t(ev.pos().x()))

    def mouseMoveEvent(self, ev):
        if self.dragging:
            self.seek_requested.emit(self._t(ev.pos().x()))
        t = self._t(ev.pos().x())
        near = [e for e in self.events if abs(self._x(e.t) - ev.pos().x()) < 4]
        tip = style.fmt_time(t)
        if near:
            tip += '\n' + '\n'.join(f'{style.fmt_time(e.t)}  {e.title}' for e in near[:6])
        self.setToolTip(tip)

    def mouseReleaseEvent(self, ev):
        self.dragging = False

    def paintEvent(self, _):
        p = QPainter(self)
        h = self.height()
        track = QRectF(6, h / 2 - 3, self.width() - 12, 6)
        p.fillRect(track, style.LIGHT)
        p.fillRect(QRectF(6, h / 2 - 3, self._x(self.cursor) - 6, 6), style.GRAY)
        # Conditions as spans above the track, instant events as ticks below.
        for e in self.events:
            if e.severity == 'info':
                continue
            col = style.SEVERITY[e.severity]
            if e.is_condition:
                x0, x1 = self._x(e.t), self._x(e.end_or(self.duration))
                row = 2 if e.severity == 'error' else 7
                p.fillRect(QRectF(x0, row, max(x1 - x0, 2), 4), col)
            else:
                x = self._x(e.t)
                p.setPen(pen(col, 2))
                p.drawLine(QPointF(x, h / 2 + 5), QPointF(x, h - 2))
        for e in self.events:
            if e.severity == 'info' and e.category == 'navigation':
                x = self._x(e.t)
                p.setPen(pen(style.GRAY, 1))
                p.drawLine(QPointF(x, h / 2 + 5), QPointF(x, h - 6))
        x = self._x(self.cursor)
        p.setPen(pen(style.BLACK, 2))
        p.drawLine(QPointF(x, 0), QPointF(x, h))


class Transport(QWidget):
    """Playback controls. The main window owns the cursor; this widget only requests changes."""

    seek_requested = Signal(float)
    play_toggled = Signal()
    step_requested = Signal(float)   # signed seconds
    event_jump = Signal(int)         # -1 previous, +1 next
    speed_changed = Signal(float)
    live_requested = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.settings = QSettings('maze_monitor', 'maze_monitor')
        outer = QVBoxLayout(self)
        outer.setContentsMargins(6, 2, 6, 4)
        outer.setSpacing(2)
        self.bar = TimelineBar()
        self.bar.seek_requested.connect(self.seek_requested)
        outer.addWidget(self.bar)

        row = QHBoxLayout()
        st = self.style()

        def button(icon, tip, slot, text=''):
            b = QToolButton()
            if icon is not None:
                b.setIcon(st.standardIcon(icon))
            if text:
                b.setText(text)
                b.setToolButtonStyle(Qt.ToolButtonTextBesideIcon if icon is not None else Qt.ToolButtonTextOnly)
            b.setToolTip(tip)
            b.clicked.connect(slot)
            row.addWidget(b)
            return b

        button(QStyle.SP_MediaSkipBackward, 'Go to the start (Home)', lambda: self.seek_requested.emit(0.0))
        self.back = button(QStyle.SP_MediaSeekBackward, 'Back by the step size (Left)',
                           lambda: self.step_requested.emit(-self.step.value()))
        self.play = button(QStyle.SP_MediaPlay, 'Play or pause (Space)', self.play_toggled.emit)
        self.fwd = button(QStyle.SP_MediaSeekForward, 'Forward by the step size (Right)',
                          lambda: self.step_requested.emit(self.step.value()))
        self.end = button(QStyle.SP_MediaSkipForward, 'Go to the end (End)',
                          lambda: self.seek_requested.emit(1e12))
        row.addWidget(QLabel('Step'))
        self.step = QDoubleSpinBox()
        self.step.setRange(0.1, 600.0)
        self.step.setDecimals(1)
        self.step.setSuffix(' s')
        self.step.setValue(float(self.settings.value('transport/step', 5.0)))
        self.step.valueChanged.connect(lambda v: self.settings.setValue('transport/step', v))
        self.step.setToolTip('How far the step buttons and the Left and Right keys move')
        row.addWidget(self.step)
        row.addWidget(QLabel('Speed'))
        self.speed = QComboBox()
        for s in SPEEDS:
            self.speed.addItem(f'{s:g}×', s)
        self.speed.setCurrentIndex(SPEEDS.index(1.0))
        self.speed.currentIndexChanged.connect(lambda i: self.speed_changed.emit(SPEEDS[i]))
        self.speed.setToolTip('Playback speed ([ slower, ] faster)')
        row.addWidget(self.speed)
        row.addSpacing(12)
        button(None, 'Jump to the previous warning or error (Ctrl+Left)',
               lambda: self.event_jump.emit(-1), '◀ Previous problem')
        button(None, 'Jump to the next warning or error (Ctrl+Right)',
               lambda: self.event_jump.emit(1), 'Next problem ▶')
        row.addStretch(1)
        self.time_label = QLabel()
        self.time_label.setFont(style.mono_font(1))
        row.addWidget(self.time_label)
        self.live = QPushButton('Live')
        self.live.setCheckable(True)
        self.live.setToolTip('Follow the newest data (L)')
        self.live.clicked.connect(self.live_requested.emit)
        row.addWidget(self.live)
        outer.addLayout(row)

    def set_state(self, duration, cursor, events, playing, following, live_mode):
        self.bar.set_state(duration, cursor, events)
        self.play.setIcon(self.style().standardIcon(QStyle.SP_MediaPause if playing else QStyle.SP_MediaPlay))
        self.time_label.setText(f'{style.fmt_time(cursor)} / {style.fmt_time(duration)}')
        self.live.setVisible(live_mode)
        self.live.setChecked(following)

    def change_speed(self, delta):
        i = min(max(self.speed.currentIndex() + delta, 0), len(SPEEDS) - 1)
        self.speed.setCurrentIndex(i)

    @property
    def speed_value(self):
        return SPEEDS[self.speed.currentIndex()]

    def fm(self):
        return QFontMetrics(self.font())
