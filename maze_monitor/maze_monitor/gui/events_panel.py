"""Events panel: findings of the analyzer, filterable; selecting one seeks to it."""

import html

from python_qt_binding.QtCore import Qt, Signal
from python_qt_binding.QtWidgets import (QCheckBox, QComboBox, QHBoxLayout, QHeaderView, QLabel,
                                         QSplitter, QTextBrowser, QTreeWidget, QTreeWidgetItem,
                                         QVBoxLayout, QWidget)

from . import style

CATEGORY_NAMES = {
    'collision': 'Collision', 'proximity': 'Proximity', 'motion': 'Motion', 'stuck': 'Stuck',
    'navigation': 'Navigation', 'planning': 'Planning', 'recovery': 'Recovery',
    'localization': 'Localization', 'exploration': 'Exploration', 'log': 'Log',
    'topic': 'Topic', 'system': 'System', 'monitor': 'Monitor',
}


class EventsPanel(QWidget):
    seek_requested = Signal(float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumWidth(280)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        row = QHBoxLayout()
        self.boxes = {}
        for sev, label in (('error', 'Errors'), ('warning', 'Warnings'), ('info', 'Info')):
            b = QCheckBox(label)
            b.setChecked(True)
            b.setStyleSheet(f'color: {style.SEVERITY[sev].name()};')
            b.toggled.connect(self._rebuild)
            self.boxes[sev] = b
            row.addWidget(b)
        self.category = QComboBox()
        self.category.addItem('All categories', '')
        for key, name in CATEGORY_NAMES.items():
            self.category.addItem(name, key)
        self.category.currentIndexChanged.connect(self._rebuild)
        row.addWidget(self.category)
        row.addStretch(1)
        layout.addLayout(row)

        split = QSplitter(Qt.Vertical)
        self.tree = QTreeWidget()
        self.tree.setHeaderLabels(('Time', 'Event'))
        self.tree.setRootIsDecorated(False)
        self.tree.setUniformRowHeights(True)
        self.tree.header().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.tree.header().setSectionResizeMode(1, QHeaderView.Stretch)
        self.tree.itemClicked.connect(self._clicked)
        self.tree.currentItemChanged.connect(lambda cur, _: self._show_detail(cur))
        split.addWidget(self.tree)
        self.detail = QTextBrowser()
        self.detail.setOpenLinks(False)
        split.addWidget(self.detail)
        split.setStretchFactor(0, 3)
        split.setStretchFactor(1, 1)
        layout.addWidget(split)
        self.hint = QLabel('Click an event to jump to it. Gray: after the cursor. Bold: active now.')
        self.hint.setStyleSheet(f'color: {style.GRAY.name()};')
        self.hint.setWordWrap(True)
        layout.addWidget(self.hint)

        self.timeline = None
        self.version = -1
        self.items = []
        self.by_id = {}
        self.cursor = 0.0

    def set_timeline(self, timeline):
        self.timeline = timeline
        self._rebuild()

    def _passes(self, e):
        cat = self.category.currentData()
        return self.boxes[e.severity].isChecked() and (not cat or e.category == cat)

    def _rebuild(self):
        self.tree.clear()
        self.items = []
        self.by_id = {}
        self.version = -1
        self.refresh(self.cursor)

    def refresh(self, cursor):
        self.cursor = cursor
        tl = self.timeline
        if tl is None:
            self.tree.clear()
            self.items = []
            self.by_id = {}
            return
        if not hasattr(self, 'by_id') or (self.version == -1 and self.items):
            self.tree.clear()
            self.items = []
            self.by_id = {}
        if tl.events_version != self.version:
            self.version = tl.events_version
            bar = self.tree.verticalScrollBar()
            at_bottom = bar.value() >= bar.maximum() - 2
            added = False
            for e in tl.events:
                it = self.by_id.get(e.id)
                title = e.title + (f'  (×{e.count})' if e.count > 1 else '')
                if it is not None:
                    if it.text(1) != title:
                        it.setText(1, title)
                        it.setToolTip(1, e.detail)
                    continue
                if not self._passes(e):
                    continue
                it = QTreeWidgetItem([style.fmt_time(e.t), title])
                it.setData(0, Qt.UserRole, e.id)
                it.setToolTip(1, f'{CATEGORY_NAMES.get(e.category, e.category)}: {e.detail}')
                self.items.append((it, e))
                self.by_id[e.id] = it
                self.tree.addTopLevelItem(it)
                added = True
            if added and at_bottom:
                self.tree.scrollToBottom()
            cur = self.tree.currentItem()
            if cur is not None:
                self._show_detail(cur)
        last_past = None
        for it, e in self.items:
            if e.t <= cursor + 1e-6:
                last_past = it
        if last_past is not None and last_past is not getattr(self, '_auto_item', None):
            self._auto_item = last_past
            self.tree.scrollToItem(last_past, QTreeWidget.PositionAtCenter)
        for it, e in self.items:
            future = e.t > cursor + 1e-6
            color = style.LIGHT if future else style.SEVERITY[e.severity]
            if e.severity == 'info' and not future:
                color = style.BLACK
            for c in range(2):
                want = color if c == 1 or future else style.GRAY
                if it.foreground(c).color() != want:
                    it.setForeground(c, want)
            f = it.font(1)
            active = e.active_at(cursor) and e.severity != 'info'
            if f.bold() != active:
                f.setBold(active)
                it.setFont(1, f)

    def _clicked(self, item, _col):
        e = self._event(item)
        if e is not None:
            self.seek_requested.emit(e.t)

    def _event(self, item):
        if item is None or self.timeline is None:
            return None
        eid = item.data(0, Qt.UserRole)
        return self.timeline.events[eid] if 0 <= eid < len(self.timeline.events) else None

    def _show_detail(self, item):
        e = self._event(item)
        if e is None:
            self.detail.clear()
            return
        col = style.SEVERITY[e.severity].name()
        when = style.fmt_time(e.t)
        if e.is_condition:
            when += ' to ' + (style.fmt_time(e.t_end) if e.t_end is not None else 'now (ongoing)')
        elif e.count > 1:
            when += f', repeated {e.count} times until {style.fmt_time(e.t_end)}'
        where = f'<br>Position: ({e.x:.2f}, {e.y:.2f})' if e.x is not None else ''
        self.detail.setHtml(
            f'<b style="color:{col}">{html.escape(e.title)}</b><br>'
            f'<span style="color:{style.GRAY.name()}">{e.severity.capitalize()} · '
            f'{CATEGORY_NAMES.get(e.category, e.category)} · {when}{where}</span>'
            f'<p>{html.escape(e.detail)}</p>')
