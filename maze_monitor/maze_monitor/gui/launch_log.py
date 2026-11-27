"""Launch output panel: what the simulation prints, as in the terminal it used to run in."""

import collections
import html
import os

from python_qt_binding.QtCore import QUrl
from python_qt_binding.QtGui import QDesktopServices
from python_qt_binding.QtWidgets import (QCheckBox, QHBoxLayout, QLabel, QPlainTextEdit,
                                         QPushButton, QVBoxLayout, QWidget)

from . import style

MAX_LINES = 20000


def line_level(line):
    """'error', 'warning' or '' for one line of ros2 launch output."""
    if '[ERROR]' in line or '[FATAL]' in line or 'Traceback (most recent' in line \
            or 'process has died' in line or line.startswith(('RuntimeError', 'Error')):
        return 'error'
    if '[WARN]' in line or '[WARNING]' in line:
        return 'warning'
    return ''


class LaunchLog(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.lines = collections.deque(maxlen=MAX_LINES)  # (level, text)
        self.log_path = None
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 2, 4, 4)
        row = QHBoxLayout()
        self.problems_only = QCheckBox('Errors and warnings only')
        self.problems_only.toggled.connect(self._rebuild)
        row.addWidget(self.problems_only)
        self.counts = QLabel()
        self.counts.setStyleSheet(f'color: {style.GRAY.name()};')
        row.addWidget(self.counts, 1)
        self.open_button = QPushButton('Open log file')
        self.open_button.setToolTip('The whole output of this run, saved as launch.log in the run folder')
        self.open_button.clicked.connect(self._open_file)
        self.open_button.setEnabled(False)
        row.addWidget(self.open_button)
        clear = QPushButton('Clear')
        clear.setToolTip('Clear this panel (the log file keeps everything)')
        clear.clicked.connect(self.clear)
        row.addWidget(clear)
        layout.addLayout(row)
        self.text = QPlainTextEdit()
        self.text.setReadOnly(True)
        self.text.setFont(style.mono_font(-1))
        self.text.setMaximumBlockCount(MAX_LINES)
        self.text.setLineWrapMode(QPlainTextEdit.NoWrap)
        self.text.setPlaceholderText('The output of the simulation appears here when you start a run.')
        layout.addWidget(self.text, 1)
        self._errors = self._warnings = 0

    def start(self, log_path, header):
        self.clear()
        self.log_path = log_path
        self.open_button.setEnabled(bool(log_path))
        self.append([header], level_override='')

    def clear(self):
        self.lines.clear()
        self.text.clear()
        self._errors = self._warnings = 0
        self._update_counts()

    def append(self, new_lines, level_override=None):
        if not new_lines:
            return
        only = self.problems_only.isChecked()
        chunk = []
        for text in new_lines:
            level = line_level(text) if level_override is None else level_override
            self.lines.append((level, text))
            if level == 'error':
                self._errors += 1
            elif level == 'warning':
                self._warnings += 1
            if not only or level:
                chunk.append(self._html(level, text))
        if chunk:
            bar = self.text.verticalScrollBar()
            at_end = bar.value() >= bar.maximum() - 4
            self.text.appendHtml('<br>'.join(chunk))
            if at_end:
                bar.setValue(bar.maximum())
        self._update_counts()

    @staticmethod
    def _html(level, text):
        color = {'error': style.RED, 'warning': style.AMBER}.get(level, style.BLACK).name()
        return f'<span style="color:{color}; white-space:pre">{html.escape(text)}</span>'

    def _rebuild(self):
        only = self.problems_only.isChecked()
        self.text.clear()
        chunk = [self._html(level, text) for level, text in self.lines if not only or level]
        if chunk:
            self.text.appendHtml('<br>'.join(chunk))

    def _update_counts(self):
        parts = []
        if self._errors:
            parts.append(f'{self._errors} error lines')
        if self._warnings:
            parts.append(f'{self._warnings} warning lines')
        self.counts.setText(' · '.join(parts))

    def _open_file(self):
        if self.log_path and os.path.isfile(self.log_path):
            QDesktopServices.openUrl(QUrl.fromLocalFile(self.log_path))
