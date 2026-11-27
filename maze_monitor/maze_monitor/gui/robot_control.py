"""Robot control panel: pause or resume exploration, and drive the robot with the keyboard."""

import math

from python_qt_binding.QtCore import QEvent, QObject, Qt, QTimer, Signal
from python_qt_binding.QtWidgets import (QAbstractSpinBox, QApplication, QCheckBox, QComboBox,
                                         QDoubleSpinBox, QFormLayout, QGroupBox, QHBoxLayout,
                                         QLabel, QLineEdit, QPlainTextEdit, QPushButton,
                                         QTextEdit, QVBoxLayout, QWidget)

from . import style
from .. import teleop

KEYS = {
    Qt.Key_W: teleop.FORWARD, Qt.Key_Up: teleop.FORWARD,
    Qt.Key_S: teleop.BACK, Qt.Key_Down: teleop.BACK,
    Qt.Key_A: teleop.LEFT, Qt.Key_Left: teleop.LEFT,
    Qt.Key_D: teleop.RIGHT, Qt.Key_Right: teleop.RIGHT,
}
STOP_KEYS = (Qt.Key_Space, Qt.Key_X)
ZERO_BURST = 3   # stop commands sent after the last key is released


def _is_text_input(widget):
    if isinstance(widget, (QLineEdit, QTextEdit, QAbstractSpinBox)):
        return True
    if isinstance(widget, QPlainTextEdit):
        return not widget.isReadOnly()
    return isinstance(widget, QComboBox) and widget.isEditable()


class KeyboardDriver(QObject):
    """While enabled, held keys drive the robot, wherever the keyboard focus is in the monitor.

    It filters key events for the whole application, so the arrow keys and
    Space drive instead of moving the timeline. Text boxes keep their keys.
    The robot stops when the last key is released, when the monitor loses the
    keyboard focus, and when driving is switched off.
    """

    command_changed = Signal(float, float)   # linear m/s, angular rad/s
    enabled_changed = Signal(bool)

    def __init__(self, send, rate_hz=20.0, parent=None):
        super().__init__(parent)
        self.send = send      # send(linear, angular)
        self.enabled = False
        self.linear = 0.15
        self.angular = 1.2
        self.held = set()
        self.slow = False
        self._zeros = 0
        self._last = (0.0, 0.0)
        self.timer = QTimer(self)
        self.timer.setInterval(int(1000 / max(rate_hz, 1.0)))
        self.timer.timeout.connect(self._publish)
        QApplication.instance().applicationStateChanged.connect(self._app_state)

    def set_enabled(self, on):
        on = bool(on)
        if on == self.enabled:
            return
        self.enabled = on
        app = QApplication.instance()
        self.held.clear()
        if on:
            app.installEventFilter(self)
            self.timer.start()
        else:
            app.removeEventFilter(self)
            self._stop()
            self._publish()
            self.timer.stop()
        self.enabled_changed.emit(on)

    def _stop(self):
        self.held.clear()
        self._zeros = ZERO_BURST

    def _app_state(self, state):
        if self.enabled and state != Qt.ApplicationActive:
            self._stop()
            self._publish()

    def eventFilter(self, obj, ev):
        t = ev.type()
        if t not in (QEvent.ShortcutOverride, QEvent.KeyPress, QEvent.KeyRelease):
            return False
        key = ev.key()
        if key not in KEYS and key not in STOP_KEYS and key != Qt.Key_Shift:
            return False
        if int(ev.modifiers() & (Qt.ControlModifier | Qt.AltModifier | Qt.MetaModifier)):
            return False
        if _is_text_input(QApplication.focusWidget()):
            return False
        if t == QEvent.ShortcutOverride:
            ev.accept()   # keep the timeline shortcuts (arrows, Space) from firing
            return True
        if ev.isAutoRepeat():
            return True
        self.slow = bool(int(ev.modifiers() & Qt.ShiftModifier))
        if key == Qt.Key_Shift:
            self.slow = t == QEvent.KeyPress
        elif key in STOP_KEYS:
            if t == QEvent.KeyPress:
                self._stop()
        elif t == QEvent.KeyPress:
            self.held.add(KEYS[key])
        else:
            self.held.discard(KEYS[key])
            if not self.held:
                self._zeros = ZERO_BURST
        self._publish()
        return True

    def _publish(self):
        v, w = teleop.velocity(self.held, self.linear, self.angular, self.slow)
        if self.held:
            self.send(v, w)
        elif self._zeros > 0:
            self._zeros -= 1
            v = w = 0.0
            self.send(0.0, 0.0)
        if (v, w) != self._last:
            self._last = (v, w)
            self.command_changed.emit(v, w)


class RobotControlPanel(QWidget):
    pause_requested = Signal(bool)          # True: pause, False: resume
    start_exploring_requested = Signal()
    stop_exploring_requested = Signal()
    drive_toggled = Signal(bool)

    def __init__(self, teleop_config, parent=None):
        super().__init__(parent)
        self.cfg = teleop_config
        self.degrees = True
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 4, 6, 6)

        box = QGroupBox('Exploration')
        v = QVBoxLayout(box)
        self.explore_label = QLabel('explore_lite is not running.')
        self.explore_label.setWordWrap(True)
        v.addWidget(self.explore_label)
        row = QHBoxLayout()
        self.pause_button = QPushButton('Pause exploring')
        self.pause_button.setToolTip('Pause explore_lite; the robot stops at once (F3)')
        self.pause_button.clicked.connect(self._pause_clicked)
        row.addWidget(self.pause_button)
        self.start_explore_button = QPushButton('Start exploring')
        self.start_explore_button.setToolTip(
            'Start explore_lite in the running simulation. It needs Nav2, and it returns the '
            'robot to where it is now when it is done (return_to_init).')
        self.start_explore_button.clicked.connect(self.start_exploring_requested)
        row.addWidget(self.start_explore_button)
        self.stop_explore_button = QPushButton('End')
        self.stop_explore_button.setToolTip('Stop the explore_lite started here; Nav2 keeps running')
        self.stop_explore_button.clicked.connect(self.stop_exploring_requested)
        row.addWidget(self.stop_explore_button)
        v.addLayout(row)
        layout.addWidget(box)
        self._paused = False

        box = QGroupBox('Drive with the keyboard')
        v = QVBoxLayout(box)
        self.drive_button = QPushButton('Drive (F2)')
        self.drive_button.setCheckable(True)
        self.drive_button.setToolTip('Drive the robot with W A S D or the arrow keys (F2)')
        self.drive_button.toggled.connect(self.drive_toggled)
        v.addWidget(self.drive_button)
        form = QFormLayout()
        self.linear = QDoubleSpinBox()
        self.linear.setRange(0.01, float(teleop_config.get('max_linear_speed', 0.22)))
        self.linear.setSingleStep(0.01)
        self.linear.setDecimals(2)
        self.linear.setSuffix(' m/s')
        self.linear.setValue(float(teleop_config.get('linear_speed', 0.15)))
        form.addRow('Drive speed', self.linear)
        self.angular = QDoubleSpinBox()
        self.angular.setSingleStep(5)
        form.addRow('Turn speed', self.angular)
        self._max_angular = float(teleop_config.get('max_angular_speed', 2.84))
        self._set_angular_unit(float(teleop_config.get('angular_speed', 1.2)))
        v.addLayout(form)
        self.pause_while_driving = QCheckBox('Pause exploring and cancel Nav2 goals when driving')
        self.pause_while_driving.setChecked(True)
        self.pause_while_driving.setToolTip('Otherwise Nav2 and your keys both send speed '
                                            'commands and the robot jerks between them.')
        v.addWidget(self.pause_while_driving)
        keys = QLabel('W or ↑ forward · S or ↓ back · A or ← left · D or → right\n'
                      'Hold Shift to go slower · Space or X stops')
        keys.setStyleSheet(f'color: {style.GRAY.name()};')
        keys.setWordWrap(True)
        v.addWidget(keys)
        self.command_label = QLabel()
        self.command_label.setStyleSheet(f'color: {style.BLUE.name()};')
        v.addWidget(self.command_label)
        layout.addWidget(box)
        layout.addStretch(1)
        self.show_command(0.0, 0.0)

    # ------------------------------------------------------------ speeds
    def _set_angular_unit(self, rad_value):
        k = math.degrees(1) if self.degrees else 1.0
        self.angular.blockSignals(True)
        self.angular.setDecimals(0 if self.degrees else 2)
        self.angular.setSingleStep(5 if self.degrees else 0.1)
        self.angular.setSuffix(' °/s' if self.degrees else ' rad/s')
        self.angular.setRange(0.01 * k, self._max_angular * k)
        self.angular.setValue(rad_value * k)
        self.angular.blockSignals(False)

    def set_units(self, degrees):
        rad = self.angular_speed()
        self.degrees = degrees
        self._set_angular_unit(rad)

    def linear_speed(self):
        return self.linear.value()

    def angular_speed(self):
        return math.radians(self.angular.value()) if self.degrees else self.angular.value()

    def show_command(self, v, w):
        if not self.drive_button.isChecked():
            self.command_label.setText('')
            return
        turn = f'{math.degrees(w):.0f} °/s' if self.degrees else f'{w:.2f} rad/s'
        self.command_label.setText(f'Sending: {v:+.2f} m/s, {turn}')

    # ------------------------------------------------------------ exploration
    def _pause_clicked(self):
        self.pause_requested.emit(not self._paused)

    def set_exploration(self, text, running, paused, can_start, started_here):
        self.explore_label.setText(text)
        self._paused = paused
        self.pause_button.setEnabled(running)
        self.pause_button.setText('Resume exploring' if paused else 'Pause exploring')
        self.pause_button.setToolTip('Resume explore_lite (F3)' if paused else
                                     'Pause explore_lite; the robot stops at once (F3)')
        self.start_explore_button.setVisible(not running)
        self.start_explore_button.setEnabled(can_start)
        self.stop_explore_button.setVisible(started_here)
