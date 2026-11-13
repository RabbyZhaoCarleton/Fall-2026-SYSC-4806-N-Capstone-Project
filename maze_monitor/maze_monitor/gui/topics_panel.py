"""Topics panel: every topic with its rate and a check box to show or hide it."""

import json

from python_qt_binding.QtCore import QSettings, Qt, Signal
from python_qt_binding.QtWidgets import (QHBoxLayout, QHeaderView, QLineEdit, QPushButton,
                                         QTreeWidget, QTreeWidgetItem, QVBoxLayout, QWidget)

from . import style


class TopicsPanel(QWidget):
    visibility_changed = Signal()

    def __init__(self, config, parent=None):
        super().__init__(parent)
        self.config = config
        self.settings = QSettings('maze_monitor', 'maze_monitor')
        try:
            self.overrides = json.loads(self.settings.value('topics/visible', '{}') or '{}')
        except ValueError:
            self.overrides = {}
        self.setMinimumWidth(240)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        self.filter = QLineEdit()
        self.filter.setPlaceholderText('Filter topics')
        self.filter.setClearButtonEnabled(True)
        self.filter.textChanged.connect(self._apply_filter)
        layout.addWidget(self.filter)
        self.tree = QTreeWidget()
        self.tree.setHeaderLabels(('Topic', 'Hz', 'Status'))
        self.tree.setRootIsDecorated(False)
        self.tree.setUniformRowHeights(True)
        self.tree.header().setSectionResizeMode(0, QHeaderView.Stretch)
        for c in (1, 2):
            self.tree.header().setSectionResizeMode(c, QHeaderView.ResizeToContents)
        self.tree.itemChanged.connect(self._item_changed)
        layout.addWidget(self.tree)
        row = QHBoxLayout()
        for text, fn in (('Show all', lambda: self._set_all(True)),
                         ('Hide all', lambda: self._set_all(False)),
                         ('Defaults', self._defaults)):
            b = QPushButton(text)
            b.clicked.connect(fn)
            row.addWidget(b)
        layout.addLayout(row)
        self.items = {}
        self._updating = False

    # ------------------------------------------------------------ visibility
    def is_visible(self, topic):
        if topic in self.overrides:
            return self.overrides[topic]
        return not self.config.hidden_by_default(topic)

    def _save(self):
        self.settings.setValue('topics/visible', json.dumps(self.overrides))
        self.visibility_changed.emit()

    def _item_changed(self, item, column):
        if self._updating or column != 0:
            return
        name = item.data(0, Qt.UserRole)
        self.overrides[name] = item.checkState(0) == Qt.Checked
        self._save()

    def _set_all(self, on):
        for name, item in self.items.items():
            if not item.isHidden():
                self.overrides[name] = on
        self._sync_checks()
        self._save()

    def _defaults(self):
        self.overrides = {}
        self._sync_checks()
        self._save()

    def _sync_checks(self):
        self._updating = True
        for name, item in self.items.items():
            item.setCheckState(0, Qt.Checked if self.is_visible(name) else Qt.Unchecked)
        self._updating = False

    def _apply_filter(self, text):
        text = text.strip().lower()
        for name, item in self.items.items():
            item.setHidden(bool(text) and text not in name.lower())

    # ------------------------------------------------------------ refresh
    def refresh(self, session, t, graph, problems):
        """graph: {name: TopicInfo} from the ROS bridge (live) or {} (replay)."""
        tl = session.timeline if session is not None else None
        names = set(graph)
        if tl is not None:
            names |= set(tl.tracks)
        self._updating = True
        for name in sorted(names):
            item = self.items.get(name)
            if item is None:
                item = QTreeWidgetItem([name])
                item.setData(0, Qt.UserRole, name)
                item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
                item.setCheckState(0, Qt.Checked if self.is_visible(name) else Qt.Unchecked)
                self.items[name] = item
                self.tree.addTopLevelItem(item)
                self.tree.sortItems(0, Qt.AscendingOrder)
                if self.filter.text():
                    self._apply_filter(self.filter.text())
            track = tl.tracks.get(name) if tl is not None else None
            info = graph.get(name)
            rate = track.rate_at(t, 3.0) if track is not None else 0.0
            count = track.count_at(t) if track is not None else 0
            if info is not None and info.excluded:
                status, color = 'excluded', style.GRAY
            elif info is not None and info.error:
                status, color = 'error', style.AMBER
            elif name in problems:
                status, color = problems[name], style.AMBER
            elif info is not None and info.publishers == 0 and not count:
                status, color = 'no pub', style.GRAY
            elif count == 0:
                status, color = 'waiting', style.GRAY
            elif rate == 0.0:
                status, color = 'idle', style.GRAY
            else:
                status, color = 'OK', style.BLACK
            item.setText(1, f'{rate:.1f}' if rate else '')
            item.setText(2, status)
            tip = name + '\n' + (info.type if info is not None else track.type_name if track is not None else '')
            explain = {'excluded': 'Not recorded or analyzed: excluded in the settings.',
                       'no pub': 'No node publishes this topic.',
                       'waiting': 'No message received yet.',
                       'idle': 'No message in the last 3 seconds.',
                       'silent': 'Stopped publishing unexpectedly.'}
            if info is not None and info.error:
                tip += '\n' + info.error
            elif status in explain:
                tip += '\n' + explain[status]
            tip += f'\n{count} messages up to the cursor'
            item.setToolTip(0, tip)
            item.setToolTip(2, tip)
            for c in range(3):
                item.setForeground(c, color if c == 2 or color == style.GRAY else style.BLACK)
        self._updating = False
