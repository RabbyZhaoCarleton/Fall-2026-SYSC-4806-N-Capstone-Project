"""Edit single ROS parameters of a run without changing the files in maze_navigation."""

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import (QAbstractItemView, QCheckBox, QDialog, QDialogButtonBox,
                                         QHBoxLayout, QHeaderView, QLabel, QLineEdit, QMessageBox,
                                         QPushButton, QTabWidget, QTreeWidget, QTreeWidgetItem,
                                         QVBoxLayout, QWidget)

from . import style
from .. import param_files

PATH_ROLE = Qt.UserRole
VALUE_COL, FILE_COL = 1, 2


class FileTab(QWidget):
    """The parameters of one file: one branch per node, one row per parameter."""

    def __init__(self, key, path, overrides, parent=None):
        super().__init__(parent)
        self.key = key
        self.path = path
        self.overrides = dict(overrides)   # {path tuple: value}
        self.file_values = {}
        self._updating = False
        layout = QVBoxLayout(self)
        info = QLabel(path or 'File not found.')
        info.setTextInteractionFlags(Qt.TextSelectableByMouse)
        info.setStyleSheet(f'color: {style.GRAY.name()};')
        info.setWordWrap(True)
        layout.addWidget(info)
        row = QHBoxLayout()
        self.filter = QLineEdit()
        self.filter.setPlaceholderText('Filter parameters, for example "timeout" or "inflation"')
        self.filter.setClearButtonEnabled(True)
        self.filter.textChanged.connect(self._apply_filter)
        row.addWidget(self.filter, 1)
        self.changed_only = QCheckBox('Changed only')
        self.changed_only.toggled.connect(self._apply_filter)
        row.addWidget(self.changed_only)
        layout.addLayout(row)
        self.tree = QTreeWidget()
        self.tree.setColumnCount(3)
        self.tree.setHeaderLabels(('Parameter', 'Value for the run', 'Value in the file'))
        self.tree.setAlternatingRowColors(True)
        self.tree.setSelectionMode(QAbstractItemView.ExtendedSelection)
        self.tree.setEditTriggers(QAbstractItemView.DoubleClicked | QAbstractItemView.EditKeyPressed
                                  | QAbstractItemView.SelectedClicked)
        self.tree.header().setSectionResizeMode(0, QHeaderView.Stretch)
        self.tree.header().setSectionResizeMode(VALUE_COL, QHeaderView.Interactive)
        self.tree.header().setSectionResizeMode(FILE_COL, QHeaderView.Interactive)
        self.tree.header().resizeSection(VALUE_COL, 220)
        self.tree.header().resizeSection(FILE_COL, 220)
        self.tree.itemChanged.connect(self._item_changed)
        layout.addWidget(self.tree, 1)
        self._load()

    def _load(self):
        if not self.path:
            return
        try:
            data = param_files.load(self.path)
        except (OSError, ValueError) as exc:
            self.tree.setDisabled(True)
            QTreeWidgetItem(self.tree, [f'Cannot read the file: {exc}'])
            return
        self._updating = True
        nodes = {}
        for path, value in param_files.leaves(data):
            self.file_values[path] = value
            node, name = param_files.display_name(path)
            parent = nodes.get(node)
            if parent is None:
                parent = nodes[node] = QTreeWidgetItem(self.tree, [node])
                parent.setFont(0, style.bold())
                parent.setFlags(Qt.ItemIsEnabled)
            item = QTreeWidgetItem(parent, [name, '', param_files.format_value(value)])
            item.setData(0, PATH_ROLE, path)
            item.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable | Qt.ItemIsEditable)
            item.setToolTip(FILE_COL, f'Type: {type(value).__name__}')
            self._show(item)
        # Overrides of parameters that are no longer in the file still apply; show them.
        for path, value in self.overrides.items():
            if path not in self.file_values:
                node, name = param_files.display_name(path)
                parent = nodes.get(node) or QTreeWidgetItem(self.tree, [node])
                nodes[node] = parent
                item = QTreeWidgetItem(parent, [name, '', '(not in the file)'])
                item.setData(0, PATH_ROLE, path)
                item.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable | Qt.ItemIsEditable)
                self._show(item)
        self._updating = False
        for i in range(self.tree.topLevelItemCount()):
            self.tree.topLevelItem(i).setExpanded(True)
        self._apply_filter()

    def _show(self, item):
        path = item.data(0, PATH_ROLE)
        changed = path in self.overrides
        value = self.overrides[path] if changed else self.file_values.get(path)
        self._updating = True
        item.setText(VALUE_COL, param_files.format_value(value))
        font = style.bold() if changed else self.tree.font()
        item.setFont(0, font)
        item.setFont(VALUE_COL, font)
        item.setToolTip(VALUE_COL, 'Changed for this run. Select it and click Reset to undo.'
                        if changed else 'Double-click to change it for this run.')
        self._updating = False

    def _item_changed(self, item, column):
        if self._updating or column != VALUE_COL:
            return
        path = item.data(0, PATH_ROLE)
        if path is None:
            return
        like = self.file_values.get(path, self.overrides.get(path))
        try:
            value = param_files.parse_value(item.text(VALUE_COL), like)
        except ValueError as exc:
            QMessageBox.warning(self, 'Value not changed', f'{item.text(0)}: {exc}')
            self._show(item)
            return
        if path in self.file_values and value == self.file_values[path] \
                and type(value) is type(self.file_values[path]):
            self.overrides.pop(path, None)
        else:
            self.overrides[path] = value
        self._show(item)
        self.window().update_counts()

    def items(self):
        for i in range(self.tree.topLevelItemCount()):
            parent = self.tree.topLevelItem(i)
            for j in range(parent.childCount()):
                yield parent.child(j)

    def reset(self, selected_only):
        for item in list(self.items()):
            if not selected_only or item.isSelected():
                self.overrides.pop(item.data(0, PATH_ROLE), None)
                self._show(item)
        self._apply_filter()

    def _apply_filter(self):
        text = self.filter.text().strip().lower()
        changed_only = self.changed_only.isChecked()
        for i in range(self.tree.topLevelItemCount()):
            parent = self.tree.topLevelItem(i)
            node_match = text and text in parent.text(0).lower()
            shown = 0
            for j in range(parent.childCount()):
                item = parent.child(j)
                hay = f'{item.text(0)} {item.text(VALUE_COL)}'.lower()
                ok = (not text or node_match or text in hay) and \
                    (not changed_only or item.data(0, PATH_ROLE) in self.overrides)
                item.setHidden(not ok)
                shown += ok
            parent.setHidden(shown == 0)


class ParamsDialog(QDialog):
    """Returns the overrides with overrides() after exec_()."""

    def __init__(self, chosen_files, overrides, parent=None):
        super().__init__(parent)
        self.setWindowTitle('Parameters for the next run')
        self.resize(900, 640)
        layout = QVBoxLayout(self)
        intro = QLabel(
            'Double-click a value to change it for the runs you start from the monitor. The '
            'files in maze_navigation stay as they are: each run gets a copy with your changes '
            'in its run folder (params/), and changed values are shown in bold.')
        intro.setWordWrap(True)
        layout.addWidget(intro)
        self.tabs = QTabWidget()
        self.file_tabs = {}
        for key, (_, _, title) in param_files.PARAM_FILES.items():
            tab = FileTab(key, param_files.resolve(key, chosen_files.get(key, '')),
                          overrides.get(key) or {})
            self.file_tabs[key] = tab
            self.tabs.addTab(tab, title)
        layout.addWidget(self.tabs, 1)
        row = QHBoxLayout()
        reset_sel = QPushButton('Reset selected')
        reset_sel.clicked.connect(lambda: self.tabs.currentWidget().reset(True))
        reset_sel.clicked.connect(self.update_counts)
        row.addWidget(reset_sel)
        reset_file = QPushButton('Reset this file')
        reset_file.clicked.connect(lambda: self.tabs.currentWidget().reset(False))
        reset_file.clicked.connect(self.update_counts)
        row.addWidget(reset_file)
        row.addStretch(1)
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        row.addWidget(buttons)
        layout.addLayout(row)
        self.update_counts()

    def update_counts(self):
        for i, (key, tab) in enumerate(self.file_tabs.items()):
            title = param_files.PARAM_FILES[key][2]
            n = len(tab.overrides)
            self.tabs.setTabText(i, f'{title} ({n})' if n else title)

    def overrides(self):
        return {key: dict(tab.overrides) for key, tab in self.file_tabs.items() if tab.overrides}
