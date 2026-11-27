"""The graphical application (Module G): home screen with Configurations and Experiments.

    ros2 run maze_experiments maze_experiments      (or just: maze_experiments)

The GUI never runs an experiment itself: it starts a background manager and
watches its files, so closing the window doesn't stop anything (BNR5).
"""

import os
import signal
import subprocess
import sys

from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtGui import QColor, QKeySequence
from python_qt_binding.QtWidgets import (QAbstractItemView, QAction, QApplication, QComboBox,
                                         QFileDialog, QHBoxLayout, QHeaderView, QLabel,
                                         QLineEdit, QMainWindow, QMenu, QMessageBox, QSplitter,
                                         QStackedWidget, QTreeWidget, QTreeWidgetItem,
                                         QVBoxLayout, QWidget)

from .. import experiment as exp_mod, manager_control
from ..metrics import fmt
from ..operations import Suite
from . import desktop
from .common import (STATE_COLORS, background, button, copy_text, heading, row, style)


class ListPanel(QWidget):
    """A titled list with search and sorting by date or name (GR2)."""

    def __init__(self, title, columns, parent=None):
        super().__init__(parent)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 4)
        self.title = heading(title, 2)
        self.search = QLineEdit()
        self.search.setPlaceholderText('Search')
        self.search.setClearButtonEnabled(True)
        self.sort = QComboBox()
        self.sort.addItems(['Newest first', 'Oldest first', 'Name A–Z', 'Name Z–A'])
        self.buttons = QHBoxLayout()
        top = QHBoxLayout()
        top.addWidget(self.title)
        top.addStretch(1)
        top.addLayout(self.buttons)
        lay.addLayout(top)
        lay.addWidget(row(self.search, self.sort, stretch_first=True))
        self.tree = QTreeWidget()
        self.tree.setHeaderLabels(columns)
        self.tree.setRootIsDecorated(False)
        self.tree.setAlternatingRowColors(True)
        self.tree.setSelectionMode(QAbstractItemView.SingleSelection)
        self.tree.setContextMenuPolicy(Qt.CustomContextMenu)
        self.tree.header().setSectionResizeMode(QHeaderView.Interactive)
        self.tree.header().setStretchLastSection(True)
        lay.addWidget(self.tree, 1)
        self.items = []    # [(object, [texts], date, name, colors)]
        self.search.textChanged.connect(self.refill)
        self.sort.currentIndexChanged.connect(self.refill)

    def set_items(self, items):
        self.items = items
        self.refill()

    def refill(self):
        selected = self.current()
        q = self.search.text().strip().lower()
        items = [i for i in self.items if not q or q in ' '.join(i[1]).lower()]
        mode = self.sort.currentIndex()
        if mode in (0, 1):
            items.sort(key=lambda i: i[2], reverse=mode == 0)
        else:
            items.sort(key=lambda i: i[3].lower(), reverse=mode == 3)
        self.tree.clear()
        for obj, texts, _, _, colors in items:
            it = QTreeWidgetItem(texts)
            it.setData(0, Qt.UserRole, obj)
            for col, color in (colors or {}).items():
                it.setForeground(col, color)
            self.tree.addTopLevelItem(it)
            if selected is not None and getattr(selected, 'path', None) == obj.path:
                self.tree.setCurrentItem(it)
        for c in range(self.tree.columnCount() - 1):
            self.tree.resizeColumnToContents(c)

    def current(self):
        it = self.tree.currentItem()
        return it.data(0, Qt.UserRole) if it is not None else None

    def at(self, pos):
        it = self.tree.itemAt(pos)
        return it.data(0, Qt.UserRole) if it is not None else None


class HomeScreen(QWidget):
    def __init__(self, win):
        super().__init__()
        self.win = win
        lay = QVBoxLayout(self)
        split = QSplitter(Qt.Horizontal)
        self.configs = ListPanel('Configurations', ['Name', 'Runs', 'Sets', 'Experiments',
                                                    'Created'])
        self.exps = ListPanel('Experiments', ['Name', 'State', 'Date', 'Configuration',
                                              'Runs done', 'Mission success'])
        self.configs.buttons.addWidget(button('Create new configuration', win.new_config))
        self.exps.buttons.addWidget(button('Import…', win.import_folder,
                                           'Import an experiment or configuration folder'))
        split.addWidget(self.configs)
        split.addWidget(self.exps)
        split.setSizes([480, 720])
        lay.addWidget(split, 1)
        self.configs.tree.customContextMenuRequested.connect(self.config_menu)
        self.exps.tree.customContextMenuRequested.connect(self.exp_menu)
        self.configs.tree.itemDoubleClicked.connect(
            lambda it, _c: self.win.edit_config(it.data(0, Qt.UserRole), view_only=True))
        self.exps.tree.itemDoubleClicked.connect(
            lambda it, _c: self.win.open_experiment(it.data(0, Qt.UserRole)))

    def refresh(self):
        def load():
            suite = self.win.suite
            return suite.configs.list(), suite.experiments.list()
        background(load, self.fill, parent=self)

    def fill(self, data):
        configs, exps = data
        counts = {}
        for e in exps:
            counts[e.config_id] = counts.get(e.config_id, 0) + 1
        items = []
        for c in configs:
            n = counts.get(c.id, 0)
            items.append((c, [c.name, str(c.total_runs()), str(len(c.sets())),
                              'not run yet' if n == 0 else f'{n} experiment(s)',
                              c.created.replace('T', ' ')], c.created, c.name,
                          {3: style.GRAY} if n == 0 else {}))
        self.configs.set_items(items)
        items = []
        for e in exps:
            s = e.meta.get('summary') or {}
            alive = manager_control.manager_alive(e) if e.state in exp_mod.ACTIVE_STATES \
                else False
            state = e.state if alive or e.state not in exp_mod.ACTIVE_STATES else \
                f'{e.state} (no manager)'
            success = '—' if not s.get('processed') else \
                f'{fmt.percent(s.get("success_rate"))} ({s["successes"]}/{s["processed"]})'
            items.append((e, [e.name, state, e.created.replace('T', ' '),
                              e.meta.get('config_name', ''),
                              f'{s.get("robot_results", 0)}/{e.meta.get("planned_runs", "?")}',
                              success], e.created, e.name,
                          {1: STATE_COLORS.get(e.state, style.BLACK)}))
        self.exps.set_items(items)

    # ---------------------------------------------------------------- menus (GR3)
    def config_menu(self, pos):
        cfg = self.configs.at(pos)
        if cfg is None:
            return
        m = QMenu(self)
        m.addAction('Run this configuration', lambda: self.win.run_config(cfg))
        m.addAction('Create modified version', lambda: self.win.edit_config(cfg))
        m.addAction('View', lambda: self.win.edit_config(cfg, view_only=True))
        m.addSeparator()
        m.addAction('Delete', lambda: self.win.delete_config(cfg))
        m.addSeparator()
        self.win.add_path_actions(m, cfg.path)
        m.exec_(self.configs.tree.viewport().mapToGlobal(pos))

    def exp_menu(self, pos):
        exp = self.exps.at(pos)
        if exp is None:
            return
        m = QMenu(self)
        m.addAction('Open results', lambda: self.win.open_experiment(exp))
        if exp.state in exp_mod.ACTIVE_STATES and manager_control.manager_alive(exp):
            m.addAction('Show run screen', lambda: self.win.show_run(exp))
        if exp.state == 'interrupted' or (exp.state == 'paused' and
                                          not manager_control.manager_alive(exp)):
            m.addAction('Resume', lambda: self.win.resume(exp))
        m.addAction('Delete', lambda: self.win.delete_experiment(exp))
        m.addAction('Export…', lambda: self.win.export(exp))
        m.addSeparator()
        self.win.add_path_actions(m, exp.path)
        m.exec_(self.exps.tree.viewport().mapToGlobal(pos))


class MainWindow(QMainWindow):
    def __init__(self, data_dir):
        super().__init__()
        self.suite = Suite(data_dir)
        self.setWindowTitle('Maze experiments')
        self.resize(1280, 760)
        self.setMinimumSize(1000, 600)
        self.stack = QStackedWidget()
        self.setCentralWidget(self.stack)
        self.home = HomeScreen(self)
        self.stack.addWidget(self.home)
        self.screens = {}
        tb = self.addToolBar('Main')
        tb.setMovable(False)
        self.back_action = tb.addAction('◀ Home', self.go_home)
        self.back_action.setShortcut(QKeySequence('Alt+Left'))
        tb.addAction('Refresh', self.refresh).setShortcut(QKeySequence.Refresh)
        tb.addSeparator()
        tb.addAction('New configuration', self.new_config)
        tb.addAction('Import…', self.import_folder)
        tb.addAction('Machine…', self.show_machine)
        tb.addSeparator()
        tb.addAction('Manual', self.open_manual)
        self.data_label = QLabel(f'  Data: {data_dir}')
        self.data_label.setStyleSheet(f'color: {style.GRAY.name()}')
        tb.addWidget(self.data_label)
        self.statusBar()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(5000)
        QTimer.singleShot(0, self.startup)

    # ---------------------------------------------------------------- navigation
    def startup(self):
        def orphans():
            return self.suite.find_orphans()

        def done(found):
            for e in found:
                self.statusBar().showMessage(
                    f'Experiment "{e.name}" was running without a manager; it is now '
                    f'interrupted. Right-click it and choose Resume.', 15000)
            self.refresh()
        background(orphans, done, parent=self)

    def refresh(self):
        self.home.refresh()

    def tick(self):
        if self.stack.currentWidget() is self.home:
            self.home.refresh()

    def go_home(self):
        self.stack.setCurrentWidget(self.home)
        self.refresh()

    def show_screen(self, key, factory):
        w = self.screens.get(key)
        if w is None:
            w = factory()
            self.screens[key] = w
            self.stack.addWidget(w)
        self.stack.setCurrentWidget(w)
        if hasattr(w, 'activate'):
            w.activate()
        return w

    def open_experiment(self, exp):
        """Results screen; the run screen while the experiment is running."""
        if exp.state in exp_mod.ACTIVE_STATES and manager_control.manager_alive(exp):
            return self.show_run(exp)
        from .results_screen import ResultsScreen
        self.show_screen(('results', exp.path), lambda: ResultsScreen(self, exp))

    def show_run(self, exp):
        from .run_screen import RunScreen
        self.show_screen(('run', exp.path), lambda: RunScreen(self, exp))

    def experiment_finished(self, exp):
        """FR7: open the results when an experiment completes while the GUI is open."""
        self.screens.pop(('results', exp.path), None)
        self.open_experiment(exp_mod.Experiment(exp.path))

    # ---------------------------------------------------------------- configurations
    def new_config(self):
        from .config_editor import ConfigEditor
        ConfigEditor(self, None).exec_()
        self.refresh()

    def edit_config(self, cfg, view_only=False):
        from .config_editor import ConfigEditor
        ConfigEditor(self, cfg, view_only=view_only).exec_()
        self.refresh()

    def delete_config(self, cfg):
        used = [e for e in self.suite.experiments.list() if e.config_id == cfg.id]
        if used:
            QMessageBox.information(
                self, 'Cannot delete',
                f'"{cfg.name}" has {len(used)} experiment(s). A configuration can only be '
                f'deleted when no experiment uses it, because experiments refer to the '
                f'configuration they ran. Delete its experiments first.')
            return
        if QMessageBox.question(self, 'Delete configuration',
                                f'Delete the configuration "{cfg.name}"?') != QMessageBox.Yes:
            return
        try:
            self.suite.configs.delete(cfg, [])
        except Exception as exc:
            QMessageBox.warning(self, 'Cannot delete', str(exc))
        self.refresh()

    def run_config(self, cfg):
        from .run_dialog import RunDialog
        dlg = RunDialog(self, cfg)
        if dlg.exec_() and dlg.experiment is not None:
            self.refresh()
            self.show_run(dlg.experiment)

    # ---------------------------------------------------------------- experiments
    def resume(self, exp):
        try:
            self.suite.resume(exp)
        except Exception as exc:
            QMessageBox.warning(self, 'Cannot resume', str(exc))
            return
        self.show_run(exp_mod.Experiment(exp.path))

    def delete_experiment(self, exp):
        if QMessageBox.question(
                self, 'Delete experiment',
                f'Delete the experiment "{exp.name}" ({exp.id}) with all its runs and '
                f'recordings? This cannot be undone.') != QMessageBox.Yes:
            return
        try:
            self.suite.delete_experiment(exp)
        except Exception as exc:
            QMessageBox.warning(self, 'Cannot delete', str(exc))
        for key in [k for k in self.screens if k[1] == exp.path]:
            w = self.screens.pop(key)
            self.stack.removeWidget(w)
        self.refresh()

    def export(self, exp, raw=None):
        folder = QFileDialog.getExistingDirectory(self, 'Export results to folder',
                                                  os.path.expanduser('~'))
        if not folder:
            return

        def work():
            return self.suite.export(exp, folder, raw=raw)

        def done(files):
            QMessageBox.information(self, 'Exported', 'Wrote:\n' + '\n'.join(files) +
                                    '\n\nThe .md file is meant to be pasted into an LLM.')
        background(work, done, parent=self, progress_text='Exporting…')

    def import_folder(self):
        folder = QFileDialog.getExistingDirectory(
            self, 'Import an experiment or configuration folder', os.path.expanduser('~'))
        if not folder:
            return

        def done(res):
            kind, obj, extra = res
            text = f'Imported {kind} "{obj.name}".'
            if kind == 'experiment' and extra:
                text += ' Its configuration was missing and was recreated from the copy in ' \
                        'the experiment.'
            self.statusBar().showMessage(text, 10000)
            self.refresh()
        background(lambda: self.suite.import_folder(folder), done, parent=self)

    # ---------------------------------------------------------------- paths (GR4)
    def add_path_actions(self, menu, path):
        menu.addAction('Open folder', lambda: self.open_folder(path))
        menu.addAction('Open terminal here', lambda: self.open_terminal(path))
        menu.addAction('Copy path', lambda: self.copy_path(path))

    def open_folder(self, path):
        what = desktop.open_folder(path)
        if what is None:
            self.copy_path(path, 'No file manager found; the path was copied instead.')
        else:
            self.statusBar().showMessage(what, 5000)

    def open_terminal(self, path):
        what = desktop.open_terminal(path)
        if what is None:
            self.copy_path(path, 'No terminal found (Windows Terminal or a Linux terminal); '
                                 'the path was copied instead.')
        else:
            self.statusBar().showMessage(what, 5000)

    def copy_path(self, path, text=None):
        copy_text(path)
        self.statusBar().showMessage(text or f'Copied {path}', 8000)

    # ---------------------------------------------------------------- misc
    def show_machine(self):
        from .run_dialog import machine_text
        from .. import resources

        def done(m):
            QMessageBox.information(self, 'This machine', machine_text(m))
        background(lambda: resources.measure_machine(self.suite.data_dir), done, parent=self,
                   progress_text='Measuring this machine…')

    def open_manual(self):
        from ..paths import own_share
        path = os.path.join(own_share(), 'docs', 'user-manual.md')
        from .results_screen import MarkdownDialog
        MarkdownDialog(self, 'User manual', path).exec_()


def main(data_dir):
    from python_qt_binding.QtCore import Qt as _Qt
    QApplication.setAttribute(_Qt.AA_EnableHighDpiScaling, True)
    app = QApplication(sys.argv[:1])
    app.setApplicationName('Maze experiments')
    app.setStyle('Fusion')
    app.setPalette(style.light_palette())
    win = MainWindow(data_dir)
    win.show()
    signal.signal(signal.SIGINT, lambda *_: QTimer.singleShot(0, win.close))
    keep = QTimer()
    keep.timeout.connect(lambda: None)   # lets Python handle Ctrl+C
    keep.start(200)
    return app.exec_()
