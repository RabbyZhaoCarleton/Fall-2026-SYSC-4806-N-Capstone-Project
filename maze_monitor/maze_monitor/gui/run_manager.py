"""Saved runs: list, search, open, rename, annotate, star, export, delete, and run again."""

import datetime
import os

from python_qt_binding.QtCore import QSettings, Qt, QUrl, Signal
from python_qt_binding.QtGui import QDesktopServices
from python_qt_binding.QtWidgets import (QAbstractItemView, QDialog, QDialogButtonBox,
                                         QFileDialog, QFormLayout, QHBoxLayout, QHeaderView,
                                         QInputDialog, QLabel, QLineEdit, QMessageBox,
                                         QPlainTextEdit, QPushButton, QTableWidget,
                                         QTableWidgetItem, QVBoxLayout)

from . import style
from ..run_store import folder_size

COLUMNS = ('★', 'Name', 'Started', 'Duration', 'Distance', 'Errors', 'Warnings',
           'Goals ok / failed', 'Explored', 'Label', 'Tags', 'Size', 'Status')


class NumItem(QTableWidgetItem):
    """Table item that sorts by a number instead of its text."""

    def __init__(self, text, key):
        super().__init__(text)
        self.key = key

    def __lt__(self, other):
        try:
            return self.key < other.key
        except (AttributeError, TypeError):
            return super().__lt__(other)


class NotesDialog(QDialog):
    def __init__(self, run, parent=None):
        super().__init__(parent)
        self.setWindowTitle(f'Notes and tags: {run.name}')
        form = QFormLayout(self)
        self.tags = QLineEdit(', '.join(run.get('tags', [])))
        self.tags.setPlaceholderText('For example: regression, wall-hit, new-params')
        form.addRow('Tags (comma-separated)', self.tags)
        self.notes = QPlainTextEdit(run.get('notes', ''))
        self.notes.setPlaceholderText('What you changed, what you expected, what happened.')
        self.notes.setMinimumSize(480, 200)
        form.addRow('Notes', self.notes)
        buttons = QDialogButtonBox(QDialogButtonBox.Save | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        form.addRow(buttons)

    def values(self):
        tags = [t.strip() for t in self.tags.text().split(',') if t.strip()]
        return self.notes.toPlainText(), tags


class RunManager(QDialog):
    open_requested = Signal(str)
    settings_requested = Signal(str)   # run folder whose settings go into the Run panel

    def __init__(self, store, current_path_fn, parent=None):
        super().__init__(parent)
        self.setWindowTitle('Saved runs')
        self.store = store
        self.current_path_fn = current_path_fn  # path of the run being recorded, or None
        self.settings = QSettings('maze_monitor', 'maze_monitor')
        self.resize(1200, 560)
        layout = QVBoxLayout(self)

        top = QHBoxLayout()
        self.search = QLineEdit()
        self.search.setPlaceholderText('Search name, label, tags, and notes')
        self.search.setClearButtonEnabled(True)
        self.search.textChanged.connect(self._filter)
        top.addWidget(self.search, 1)
        refresh = QPushButton('Refresh')
        refresh.clicked.connect(self.reload)
        top.addWidget(refresh)
        layout.addLayout(top)

        self.table = QTableWidget(0, len(COLUMNS))
        self.table.setHorizontalHeaderLabels(COLUMNS)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.ExtendedSelection)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.verticalHeader().setVisible(False)
        self.table.setAlternatingRowColors(True)
        self.table.setSortingEnabled(True)
        hh = self.table.horizontalHeader()
        for c in range(len(COLUMNS)):
            hh.setSectionResizeMode(c, QHeaderView.ResizeToContents)
        hh.setSectionResizeMode(1, QHeaderView.Stretch)
        self.table.doubleClicked.connect(lambda _: self._open())
        self.table.itemSelectionChanged.connect(self._selection_changed)
        layout.addWidget(self.table, 1)

        self.notes_view = QLabel()
        self.notes_view.setWordWrap(True)
        self.notes_view.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.notes_view.setStyleSheet(f'color: {style.GRAY.name()};')
        layout.addWidget(self.notes_view)

        row = QHBoxLayout()
        self.buttons = {}
        for key, text, fn, tip in (
                ('open', 'Open replay', self._open, 'Replay the selected run (double-click also works)'),
                ('again', 'Use its settings', self._again,
                 'Put the settings of the selected run into the Run panel, to run it again'),
                ('rename', 'Rename…', self._rename, 'Change the display name'),
                ('notes', 'Notes and tags…', self._notes, 'Write what you changed and what happened'),
                ('star', 'Star', self._star, 'Mark runs you want to keep; starred runs sort first'),
                ('export', 'Export .zip…', self._export, 'Save the selected run as a .zip file to share'),
                ('folder', 'Show folder', self._folder, 'Open the run folder in the file manager'),
                ('delete', 'Delete…', self._delete, 'Delete the selected runs from disk')):
            b = QPushButton(text)
            b.setToolTip(tip)
            b.clicked.connect(fn)
            row.addWidget(b)
            self.buttons[key] = b
        row.addStretch(1)
        self.footer = QLabel()
        self.footer.setStyleSheet(f'color: {style.GRAY.name()};')
        row.addWidget(self.footer)
        close = QPushButton('Close')
        close.clicked.connect(self.accept)
        row.addWidget(close)
        layout.addLayout(row)
        self.runs = []
        self.reload()

    # ------------------------------------------------------------ data
    def reload(self):
        self.runs = self.store.list()
        self.runs.sort(key=lambda r: (not r.get('starred', False),), reverse=False)
        self.table.setSortingEnabled(False)
        self.table.setRowCount(len(self.runs))
        current = self.current_path_fn()
        for r, run in enumerate(self.runs):
            s = run.summary
            size = folder_size(run.path)
            started = run.get('started') or ''
            try:
                started_text = datetime.datetime.fromisoformat(started).strftime('%Y-%m-%d %H:%M:%S')
            except ValueError:
                started_text = started
            status = run.get('status', 'unknown')
            if current and os.path.abspath(run.path) == os.path.abspath(current):
                status = 'recording now'
            ok, failed = s.get('goals_succeeded'), s.get('goals_failed')
            known = s.get('known_area_m2')
            cells = [
                NumItem('★' if run.get('starred') else '', 0 if run.get('starred') else 1),
                NumItem(run.name, run.name.lower()),
                NumItem(started_text, started),
                NumItem(style.fmt_time(s.get('duration_s')) if s.get('duration_s') is not None else '—',
                        s.get('duration_s') or 0),
                NumItem(f'{s["distance_m"]:.2f} m' if s.get('distance_m') is not None else '—',
                        s.get('distance_m') or 0),
                NumItem(str(s.get('errors', '—')), s.get('errors') or 0),
                NumItem(str(s.get('warnings', '—')), s.get('warnings') or 0),
                NumItem(f'{ok} / {failed}' if ok is not None else '—', (ok or 0) - (failed or 0)),
                NumItem(f'{known:.1f} m²' if known is not None else '—', known or 0),
                NumItem(run.get('label', ''), run.get('label', '')),
                NumItem(', '.join(run.get('tags', [])), ', '.join(run.get('tags', []))),
                NumItem(style.fmt_bytes(size), size),
                NumItem(status, status),
            ]
            for c, item in enumerate(cells):
                item.setData(Qt.UserRole, r)
                if c == 5 and s.get('errors'):
                    item.setForeground(style.RED)
                if c == 6 and s.get('warnings'):
                    item.setForeground(style.AMBER)
                if c == 12 and status not in ('complete', 'recording now'):
                    item.setForeground(style.AMBER)
                self.table.setItem(r, c, item)
        self.table.setSortingEnabled(True)
        self._filter(self.search.text())
        total = self.store.total_size()
        self.footer.setText(f'{len(self.runs)} runs, {style.fmt_bytes(total)} in {self.store.runs_dir}')
        self._selection_changed()

    def _filter(self, text):
        text = text.strip().lower()
        for row in range(self.table.rowCount()):
            run = self.runs[self.table.item(row, 0).data(Qt.UserRole)]
            hay = ' '.join([run.name, run.get('label', '') or '', ' '.join(run.get('tags', [])),
                            run.get('notes', '') or '']).lower()
            self.table.setRowHidden(row, bool(text) and text not in hay)

    def selected(self):
        rows = sorted({i.row() for i in self.table.selectedItems()})
        return [self.runs[self.table.item(r, 0).data(Qt.UserRole)] for r in rows]

    def _is_current(self, run):
        cur = self.current_path_fn()
        return bool(cur) and os.path.abspath(cur) == os.path.abspath(run.path)

    def _selection_changed(self):
        sel = self.selected()
        one = len(sel) == 1
        for key in ('open', 'again', 'rename', 'notes', 'export', 'folder'):
            self.buttons[key].setEnabled(one)
        self.buttons['star'].setEnabled(bool(sel))
        self.buttons['delete'].setEnabled(bool(sel) and not any(self._is_current(r) for r in sel))
        if one:
            self.buttons['open'].setEnabled(not self._is_current(sel[0]))
            run = sel[0]
            self.buttons['star'].setText('Unstar' if run.get('starred') else 'Star')
            notes = run.get('notes') or 'No notes.'
            self.notes_view.setText(f'{run.path}\n{notes}')
        else:
            self.notes_view.setText(f'{len(sel)} runs selected.' if sel else '')

    # ------------------------------------------------------------ actions
    def _open(self):
        sel = self.selected()
        if len(sel) == 1 and not self._is_current(sel[0]):
            self.open_requested.emit(sel[0].path)
            self.accept()

    def _again(self):
        sel = self.selected()
        if len(sel) == 1:
            self.settings_requested.emit(sel[0].path)
            self.accept()

    def _rename(self):
        run = self.selected()[0]
        name, ok = QInputDialog.getText(self, 'Rename run', 'Name:', text=run.name)
        if ok:
            self.store.rename(run, name)
            self.reload()

    def _notes(self):
        run = self.selected()[0]
        dlg = NotesDialog(run, self)
        if dlg.exec_():
            notes, tags = dlg.values()
            run.update(notes=notes, tags=tags)
            self.reload()

    def _star(self):
        sel = self.selected()
        target = not all(r.get('starred') for r in sel)
        for run in sel:
            run.update(starred=target)
        self.reload()

    def _export(self):
        run = self.selected()[0]
        start = self.settings.value('runs/export_dir', os.path.expanduser('~'))
        path, _ = QFileDialog.getSaveFileName(self, 'Export run', os.path.join(start, run.id + '.zip'),
                                              'Zip files (*.zip)')
        if not path:
            return
        self.settings.setValue('runs/export_dir', os.path.dirname(path))
        try:
            out = self.store.export(run, path)
        except OSError as exc:
            QMessageBox.warning(self, 'Export failed', str(exc))
            return
        QMessageBox.information(self, 'Run exported', f'Saved to {out}')

    def _folder(self):
        QDesktopServices.openUrl(QUrl.fromLocalFile(self.selected()[0].path))

    def _delete(self):
        sel = [r for r in self.selected() if not self._is_current(r)]
        if not sel:
            return
        starred = [r for r in sel if r.get('starred')]
        names = '\n'.join(f'  {r.name}' for r in sel[:12]) + ('\n  …' if len(sel) > 12 else '')
        text = f'Delete {len(sel)} run{"s" if len(sel) > 1 else ""} permanently?\n\n{names}'
        if starred:
            text += f'\n\n{len(starred)} of them {"is" if len(starred) == 1 else "are"} starred.'
        if QMessageBox.question(self, 'Delete runs', text, QMessageBox.Yes | QMessageBox.No,
                                QMessageBox.No) != QMessageBox.Yes:
            return
        errors = []
        for run in sel:
            try:
                self.store.delete(run)
            except (OSError, ValueError) as exc:
                errors.append(f'{run.name}: {exc}')
        if errors:
            QMessageBox.warning(self, 'Some runs were not deleted', '\n'.join(errors))
        self.reload()
