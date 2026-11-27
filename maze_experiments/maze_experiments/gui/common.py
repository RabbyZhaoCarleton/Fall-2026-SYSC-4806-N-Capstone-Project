"""Shared GUI helpers: background work (GNR1), the monitor's style (GNR3), small widgets."""

import traceback

from python_qt_binding.QtCore import QObject, QThread, Qt, Signal
from python_qt_binding.QtWidgets import (QApplication, QHBoxLayout, QLabel, QMessageBox,
                                         QProgressDialog, QPushButton, QWidget)

from maze_monitor.gui import style  # noqa: F401  (re-exported: one palette for both tools)

_running = set()


class _Worker(QObject):
    done = Signal(object)
    failed = Signal(str)
    progress = Signal(int, int)

    def __init__(self, fn, with_progress):
        super().__init__()
        self.fn = fn
        self.with_progress = with_progress

    def run(self):
        try:
            if self.with_progress:
                result = self.fn(lambda i, n: self.progress.emit(int(i), int(n)))
            else:
                result = self.fn()
        except Exception as exc:  # shown to the user, never lost
            self.failed.emit(f'{type(exc).__name__}: {exc}\n\n{traceback.format_exc()}')
            return
        self.done.emit(result)


def background(fn, on_done=None, on_error=None, parent=None, progress_text=None,
               with_progress=False):
    """Run fn off the GUI thread (GNR1). Shows a progress dialog when progress_text is given."""
    thread = QThread()
    worker = _Worker(fn, with_progress)
    worker.moveToThread(thread)
    dialog = None
    if progress_text:
        dialog = QProgressDialog(progress_text, None, 0, 0, parent)
        dialog.setWindowModality(Qt.WindowModal)
        dialog.setMinimumDuration(150)
        dialog.setCancelButton(None)
        worker.progress.connect(lambda i, n: (dialog.setMaximum(n), dialog.setValue(i)))

    def finish():
        if dialog is not None:
            dialog.close()
        thread.quit()

    def ok(result):
        finish()
        if on_done:
            on_done(result)

    def bad(text):
        finish()
        if on_error:
            on_error(text)
        else:
            QMessageBox.warning(parent, 'Something went wrong', text.split('\n\n')[0],
                                QMessageBox.Ok)
    worker.done.connect(ok)
    worker.failed.connect(bad)
    thread.started.connect(worker.run)
    entry = (thread, worker)
    _running.add(entry)
    thread.finished.connect(lambda: _running.discard(entry))
    thread.start()
    return thread


def copy_text(text):
    QApplication.clipboard().setText(text)


def button(text, slot, tip=''):
    b = QPushButton(text)
    b.clicked.connect(slot)
    if tip:
        b.setToolTip(tip)
    return b


def row(*widgets, stretch_first=False):
    w = QWidget()
    lay = QHBoxLayout(w)
    lay.setContentsMargins(0, 0, 0, 0)
    for i, x in enumerate(widgets):
        if x is None:
            lay.addStretch(1)
        else:
            lay.addWidget(x, 1 if stretch_first and i == 0 else 0)
    return w


def heading(text, size=3):
    lab = QLabel(text)
    f = lab.font()
    f.setPointSizeF(f.pointSizeF() + size)
    f.setBold(True)
    lab.setFont(f)
    return lab


def color_hex(c):
    return c.name()


STATE_COLORS = {'completed': style.GREEN, 'running': style.BLUE, 'calibrating': style.BLUE,
                'generating': style.BLUE, 'processing': style.BLUE, 'paused': style.AMBER,
                'interrupted': style.AMBER, 'cancelled': style.GRAY, 'created': style.GRAY}
OUTCOME_COLORS = {'complete': style.GREEN, 'incomplete': style.AMBER,
                  'return_failed': style.AMBER, 'timeout': style.AMBER, 'stalled': style.AMBER,
                  'stack_crash': style.RED, 'startup_failed': style.GRAY,
                  'sim_frozen': style.GRAY, 'infra_failure': style.GRAY,
                  'interrupted': style.GRAY}
