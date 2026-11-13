"""Raw view: topic statistics, message fields, and derived values, unprocessed."""

import array
import math

import numpy as np
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import (QHeaderView, QLabel, QSplitter, QTableWidget,
                                         QTableWidgetItem, QTreeWidget, QTreeWidgetItem,
                                         QVBoxLayout, QWidget)

from . import style

ARRAY_PREVIEW = 8


def fmt_number(v):
    if isinstance(v, bool):
        return str(v)
    if isinstance(v, int):
        return str(v)
    if isinstance(v, float):
        if math.isnan(v) or math.isinf(v):
            return str(v)
        return f'{v:.6g}'
    return repr(v)


def summarize_array(values):
    n = len(values)
    if n == 0:
        return '[] (empty)'
    try:
        arr = np.asarray(values, dtype=np.float64)
        finite = arr[np.isfinite(arr)]
        stats = (f'min {fmt_number(float(finite.min()))}  max {fmt_number(float(finite.max()))}  '
                 f'mean {fmt_number(float(finite.mean()))}' if finite.size else 'no finite values')
        nonfinite = n - finite.size
        if nonfinite:
            stats += f'  ({nonfinite} inf/nan)'
        head = ', '.join(fmt_number(float(x)) for x in arr[:ARRAY_PREVIEW])
        return f'[{n} values]  {stats}  |  {head}{", …" if n > ARRAY_PREVIEW else ""}'
    except (TypeError, ValueError):
        return f'[{n} values]'


def flatten(msg, prefix=''):
    """Yield (field path, value text) for every field of a message."""
    if hasattr(msg, 'get_fields_and_field_types'):
        for name in msg.get_fields_and_field_types():
            yield from flatten(getattr(msg, name), f'{prefix}.{name}' if prefix else name)
        return
    if isinstance(msg, (list, tuple)) and msg and hasattr(msg[0], 'get_fields_and_field_types'):
        yield prefix, f'[{len(msg)} items]'
        for i, item in enumerate(msg[:ARRAY_PREVIEW]):
            yield from flatten(item, f'{prefix}[{i}]')
        if len(msg) > ARRAY_PREVIEW:
            yield f'{prefix}[…]', f'{len(msg) - ARRAY_PREVIEW} more items not shown'
        return
    if isinstance(msg, (bytes, bytearray)):
        yield prefix, summarize_array(list(msg)) if len(msg) > ARRAY_PREVIEW else repr(msg)
        return
    if isinstance(msg, (list, tuple, array.array, np.ndarray)):
        if len(msg) <= ARRAY_PREVIEW and not isinstance(msg, np.ndarray):
            yield prefix, '[' + ', '.join(fmt_number(v) if not isinstance(v, str) else repr(v)
                                          for v in msg) + ']'
        else:
            yield prefix, summarize_array(msg)
        return
    if isinstance(msg, str):
        yield prefix, repr(msg) if len(msg) < 300 else repr(msg[:300]) + '…'
        return
    yield prefix, fmt_number(msg)


class RawView(QWidget):
    COLS = ('Topic / field', 'Type / value', 'Rate', 'Count', 'Age', 'Data rate')

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        split = QSplitter(Qt.Vertical)
        layout.addWidget(split)

        top = QWidget()
        tl = QVBoxLayout(top)
        tl.setContentsMargins(0, 0, 0, 0)
        hint = QLabel('Last message of each visible topic at the cursor. '
                      'Expand a topic to see its fields. Long arrays show a summary and the first values.')
        hint.setStyleSheet(f'color: {style.GRAY.name()};')
        hint.setWordWrap(True)
        tl.addWidget(hint)
        self.tree = QTreeWidget()
        self.tree.setColumnCount(len(self.COLS))
        self.tree.setHeaderLabels(self.COLS)
        self.tree.setUniformRowHeights(True)
        self.tree.setAlternatingRowColors(True)
        self.tree.header().setSectionResizeMode(0, QHeaderView.Interactive)
        self.tree.header().setSectionResizeMode(1, QHeaderView.Stretch)
        for c in range(2, len(self.COLS)):
            self.tree.header().setSectionResizeMode(c, QHeaderView.ResizeToContents)
        self.tree.setColumnWidth(0, 320)
        self.tree.itemExpanded.connect(lambda _: self._refresh_children())
        tl.addWidget(self.tree)
        split.addWidget(top)

        bottom = QWidget()
        bl = QVBoxLayout(bottom)
        bl.setContentsMargins(0, 0, 0, 0)
        label = QLabel('Derived values at the cursor (computed by the analyzer, unrounded)')
        label.setFont(style.bold())
        bl.addWidget(label)
        self.table = QTableWidget(0, 5)
        self.table.setHorizontalHeaderLabels(('Key', 'Meaning', 'Value', 'Unit', 'Age (s)'))
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setAlternatingRowColors(True)
        self.table.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        for c in (0, 2, 3, 4):
            self.table.horizontalHeader().setSectionResizeMode(c, QHeaderView.ResizeToContents)
        bl.addWidget(self.table)
        split.addWidget(bottom)
        split.setStretchFactor(0, 3)
        split.setStretchFactor(1, 2)

        self.items = {}
        self.session = None
        self.t = 0.0
        self.mono = style.mono_font()

    def update_view(self, session, t, visible):
        self.session, self.t = session, t
        tl = session.timeline
        names = sorted(n for n in tl.tracks if visible(n))
        # Remove rows of hidden topics; add rows for new ones.
        for name in list(self.items):
            if name not in names:
                item = self.items.pop(name)
                self.tree.takeTopLevelItem(self.tree.indexOfTopLevelItem(item))
        for i, name in enumerate(names):
            item = self.items.get(name)
            if item is None:
                item = QTreeWidgetItem([name])
                item.setChildIndicatorPolicy(QTreeWidgetItem.ShowIndicator)
                self.items[name] = item
                self.tree.insertTopLevelItem(i, item)
            tr = tl.tracks[name]
            count = tr.count_at(t)
            last_t = tr.times[count - 1] if count else None
            age = t - last_t if last_t is not None else None
            item.setText(1, tr.type_name + ('' if count else '   (no message yet at this time)'))
            item.setText(2, f'{tr.rate_at(t):.1f} Hz')
            item.setText(3, str(count))
            item.setText(4, f'{age:.2f} s' if age is not None else '—')
            item.setText(5, style.fmt_bytes(tr.bytes_rate_at(t)) + '/s')
            stale = age is None or age > 5.0
            for c in range(6):
                item.setForeground(c, style.GRAY if stale else style.BLACK)
            if tr.type_error:
                item.setText(1, 'Type not available: ' + tr.type_error.splitlines()[0])
        self._refresh_children()
        self._refresh_table(tl, t)

    def _refresh_children(self):
        if self.session is None:
            return
        tl = self.session.timeline
        for name, item in self.items.items():
            if not item.isExpanded():
                continue
            tr = tl.tracks.get(name)
            if tr is None:
                continue
            _, msg = tr.latest(self.t)
            rows = list(flatten(msg)) if msg is not None else []
            while item.childCount() > len(rows):
                item.removeChild(item.child(item.childCount() - 1))
            for k, (path, value) in enumerate(rows):
                if k < item.childCount():
                    child = item.child(k)
                    if child.text(0) != path:
                        child.setText(0, path)
                    if child.text(1) != value:
                        child.setText(1, value)
                else:
                    child = QTreeWidgetItem([path, value])
                    child.setFont(1, self.mono)
                    item.addChild(child)

    def _refresh_table(self, tl, t):
        keys = sorted(tl.series)
        self.table.setRowCount(len(keys))
        for r, key in enumerate(keys):
            s = tl.series[key]
            v = s.value_at(t)
            tv = s.time_of_value_at(t)
            vals = (key, s.label, '—' if v is None else (fmt_number(v) if s.numeric else str(v)),
                    s.unit, f'{t - tv:.2f}' if tv is not None else '—')
            for c, text in enumerate(vals):
                it = self.table.item(r, c)
                if it is None:
                    it = QTableWidgetItem()
                    if c == 2:
                        it.setFont(self.mono)
                    self.table.setItem(r, c, it)
                if it.text() != text:
                    it.setText(text)
