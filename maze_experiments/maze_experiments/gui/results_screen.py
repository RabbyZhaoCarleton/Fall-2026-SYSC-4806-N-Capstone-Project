"""The results screen (2.3.2, GR7): processed results and raw data.

Processed results: a sidebar with the categories (Overview, Outcomes,
Exploration, Exits, Mapping, Localization, Safety, Performance), each with
tables, charts and plain-language findings, for the whole experiment or one
maze set. They are read from results/results.json, so a 100-run experiment
opens in well under 5 s (GNR2).

Raw data: run -> topic -> message, shown like `ros2 topic echo` (ER9), with
selectable text, a copy button per entry, and Open in maze monitor per run.
"""

import html
import json
import os
import subprocess

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import (QComboBox, QDialog, QHeaderView, QLabel,
                                         QListWidget, QMessageBox, QPlainTextEdit, QPushButton,
                                         QSplitter, QTableWidget, QTableWidgetItem, QTabWidget,
                                         QTextBrowser, QTreeWidget, QTreeWidgetItem, QVBoxLayout,
                                         QWidget)

from .. import experiment as exp_mod, manager_control
from ..metrics import fmt
from ..metrics.findings import CATEGORIES
from .common import background, button, copy_text, heading, row, style

CHARTS_FOR = {'Overview': ['outcomes.png', 'distributions.png'], 'Outcomes': ['outcomes.png'],
              'Exploration': ['coverage.png', 'distributions.png'],
              'Localization': ['drift.png'], 'Safety': ['distributions.png']}
EXTRA = ('Runs', 'Provenance')
PAGE = 200


class MarkdownDialog(QDialog):
    def __init__(self, parent, title, path):
        super().__init__(parent)
        self.setWindowTitle(title)
        self.resize(900, 700)
        lay = QVBoxLayout(self)
        view = QTextBrowser()
        view.setOpenExternalLinks(True)
        try:
            with open(path) as f:
                view.setMarkdown(f.read())
        except OSError as exc:
            view.setPlainText(str(exc))
        lay.addWidget(view)


def load(exp, rebuild=False):
    from ..metrics.results import build_results, load_results
    exp = exp_mod.Experiment(exp.path)
    data = None if rebuild else load_results(exp)
    stale = False
    if data is not None:
        mtime = os.path.getmtime(os.path.join(exp.results_dir, 'results.json'))
        for rid in exp.run_ids():
            m = os.path.join(exp.run_dir(rid), 'metrics.json')
            if os.path.exists(m) and os.path.getmtime(m) > mtime:
                stale = True
                break
    if data is None or (stale and not manager_control.manager_alive(exp)):
        data = build_results(exp)
        stale = False
    return exp, data, stale


class ResultsScreen(QWidget):
    def __init__(self, win, exp):
        super().__init__()
        self.win = win
        self.exp = exp
        self.data = None
        lay = QVBoxLayout(self)
        self.title = heading(f'Results: {exp.name}', 3)
        self.subtitle = QLabel('Loading…')
        self.subtitle.setWordWrap(True)
        lay.addWidget(row(self.title, None, button('Export…', self.export),
                          button('Update results', self.rebuild,
                                 'Rebuild the results from the run metrics'),
                          button('Reprocess all runs', self.reprocess,
                                 'Recompute every run\'s metrics from its recording (ER5)'),
                          button('Open folder', lambda: win.open_folder(self.exp.path))))
        lay.addWidget(self.subtitle)
        self.warning = QLabel('')
        self.warning.setWordWrap(True)
        self.warning.setStyleSheet(f'color: {style.AMBER.name()}; font-weight: bold')
        self.warning.hide()
        lay.addWidget(self.warning)
        tabs = QTabWidget()
        lay.addWidget(tabs, 1)
        tabs.addTab(self._processed(), 'Processed results')
        self.raw = RawData(self)
        tabs.addTab(self.raw, 'Raw data')
        background(lambda: load(exp), self.loaded, parent=self,
                   progress_text='Loading results…')

    def _processed(self):
        w = QWidget()
        lay = QVBoxLayout(w)
        self.scope = QComboBox()
        self.scope.setSizeAdjustPolicy(QComboBox.AdjustToContents)
        self.scope.setMinimumContentsLength(30)
        self.scope.currentIndexChanged.connect(self.render)
        lay.addWidget(row(QLabel('Show:'), self.scope, None))
        split = QSplitter(Qt.Horizontal)
        self.sidebar = QListWidget()
        self.sidebar.addItems(list(CATEGORIES) + list(EXTRA))
        self.sidebar.setMaximumWidth(180)
        self.sidebar.currentRowChanged.connect(self.render)
        self.view = QTextBrowser()
        self.view.setOpenLinks(False)
        split.addWidget(self.sidebar)
        split.addWidget(self.view)
        split.setSizes([160, 1000])
        lay.addWidget(split, 1)
        return w

    def loaded(self, res):
        self.exp, self.data, stale = res
        exp, d = self.exp, self.data
        o = d['overall']
        self.subtitle.setText(
            f'{exp.state} · {o["robot_results"]} of {o["planned"]} runs gave a robot result · '
            f'{o["excluded_total"]} excluded attempt(s) · configuration '
            f'{exp.meta.get("config_name")} · results generated {d["generated"].replace("T", " ")}'
            f' (pipeline {d["pipeline_version"]})' + (' · newer run metrics exist: click Update '
                                                       'results' if stale else ''))
        if d.get('warnings'):
            self.warning.setText('⚠ ' + '<br>⚠ '.join(html.escape(w) for w in d['warnings']))
            self.warning.show()
        else:
            self.warning.hide()
        self.scope.blockSignals(True)
        self.scope.clear()
        self.scope.addItem('Whole experiment')
        for s in d['sets']:
            self.scope.addItem(f'Maze set: {s["name"]}')
        self.scope.blockSignals(False)
        if self.sidebar.currentRow() < 0:
            self.sidebar.setCurrentRow(0)
        self.render()
        self.raw.set_runs(d['runs'])

    # ---------------------------------------------------------------- rendering
    def render(self, *_):
        if not self.data:
            return
        cat = self.sidebar.currentItem().text() if self.sidebar.currentItem() else 'Overview'
        k = self.scope.currentIndex() - 1
        if cat == 'Runs':
            self.view.setHtml(self.runs_html(k))
            return
        if cat == 'Provenance':
            self.view.setHtml(self.provenance_html())
            return
        part = self.data['findings']['experiment' if k < 0 else 'sets']
        part = part if k < 0 else part[k]
        c = part[cat]
        h = [f'<h2>{cat}</h2>']
        if k >= 0:
            s = self.data['sets'][k]
            mz = s['maze']
            h.append(f'<p style="color:{style.GRAY.name()}">{html.escape(s["name"])}: '
                     f'{mz["width"]} x {mz["length"]} m, corridors {mz["corridor_width"]} m, '
                     f'{mz["exits"]} exit(s), {s["planned"]} planned runs.</p>')
            prev = os.path.join(self.exp.maze_dir(k, 0), 'preview.png')
            if cat == 'Overview' and os.path.exists(prev):
                h.append(f'<p><img src="{prev}" width="220"></p>')
        h.append('<ul>' + ''.join(f'<li>{html.escape(t)}</li>' for t in c['findings']) + '</ul>')
        for t in c['tables']:
            h.append(table_html(t))
        if k < 0:
            for chart in CHARTS_FOR.get(cat, []):
                path = os.path.join(self.exp.results_dir, 'charts', chart)
                if os.path.exists(path):
                    h.append(f'<p><img src="{path}" width="860"></p>')
        self.view.setHtml(''.join(h))

    def runs_html(self, k):
        rows = [r for r in self.data['runs'] if k < 0 or r['set_index'] == k]
        t = {'title': 'Runs', 'columns': ['Run', 'Set', 'Maze', 'Try', 'Outcome', 'Success',
                                          'Mission', 'Coverage', 'Exits', 'False', 'SLAM max err',
                                          'Min clearance', 'RTF', 'Reason'], 'rows': []}
        for r in rows:
            t['rows'].append([r['run_id'], r['set_index'], r['maze_index'], r['attempt'],
                              r['outcome'], {True: 'yes', False: 'no'}.get(r['mission_success'],
                                                                           '—'),
                              fmt.duration(r.get('time.mission_s')),
                              fmt.percent(r.get('exploration.coverage'), 1),
                              f'{r.get("exits.found", "—")}/{r.get("exits.true", "—")}',
                              r.get('exits.false', '—'),
                              fmt.value(r.get('localization.slam.max_pos_m'), 'small_length'),
                              fmt.value(r.get('safety.min_clearance_m'), 'small_length'),
                              fmt.value(r.get('time.rtf_mean'), 'ratio'), r.get('reason') or ''])
        return '<h2>Runs</h2><p>Every attempt. Excluded attempts were repeated and do not ' \
               'count. The Raw data tab shows each run\'s messages.</p>' + table_html(t)

    def provenance_html(self):
        prov = self.exp.meta.get('provenance') or {}
        ex = {k: v for k, v in self.exp.execution.items() if k != 'machine'}
        h = ['<h2>Provenance</h2>']
        for w in self.data.get('warnings') or []:
            h.append(f'<p style="color:{style.AMBER.name()}"><b>⚠ {html.escape(w)}</b></p>')
        h.append('<h3>Execution settings</h3><pre>' + html.escape(json.dumps(ex, indent=2)) +
                 '</pre>')
        h.append('<h3>Code, parameters and machine</h3><pre>' +
                 html.escape(json.dumps({k: v for k, v in prov.items() if k != 'robot_profile'},
                                        indent=2, default=str)) + '</pre>')
        return ''.join(h)

    # ---------------------------------------------------------------- actions
    def export(self):
        raw = self.raw.export_selection()
        self.win.export(self.exp, raw=raw)

    def rebuild(self):
        background(lambda: load(self.exp, rebuild=True), self.loaded, parent=self,
                   progress_text='Updating the results…')

    def reprocess(self):
        if QMessageBox.question(self, 'Reprocess', 'Recompute every run\'s metrics from its '
                                'recording and rebuild the results? This can take a while.') \
                != QMessageBox.Yes:
            return

        def work(progress):
            data, errors = self.win.suite.reprocess(self.exp, progress=progress)
            return load(self.exp)
        background(work, self.loaded, parent=self, progress_text='Reprocessing runs…',
                   with_progress=True)


def table_html(t):
    head = ''.join(f'<th align="left" style="padding:2px 8px">{html.escape(str(c))}</th>'
                   for c in t['columns'])
    body = []
    for i, r in enumerate(t['rows']):
        bg = '#f7f7f7' if i % 2 else '#ffffff'
        body.append(f'<tr style="background:{bg}">' + ''.join(
            f'<td style="padding:2px 8px">{html.escape(str(c))}</td>' for c in r) + '</tr>')
    return (f'<h3>{html.escape(t["title"])}</h3><table cellspacing="0" '
            f'style="border-collapse:collapse"><tr>{head}</tr>{"".join(body)}</table>')


# ======================================================================== raw data
def bag_topics(run_dir):
    from maze_monitor.recorder import _open_reader
    reader = _open_reader(os.path.join(run_dir, 'bag'))
    meta = reader.get_metadata()
    counts = {t.topic_metadata.name: t.message_count for t in meta.topics_with_message_count}
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    return sorted((name, types.get(name, ''), counts.get(name, 0)) for name in types)


def topic_messages(run_dir, topic, start_ns, limit=5000):
    """[(seconds since the run started, serialized bytes)] of one topic, filtered in storage."""
    import rosbag2_py
    from maze_monitor.recorder import _open_reader
    reader = _open_reader(os.path.join(run_dir, 'bag'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    out = []
    while reader.has_next() and len(out) < limit:
        _, data, t_ns = reader.read_next()
        out.append(((t_ns - (start_ns or t_ns)) * 1e-9, data))
    return out


class RawData(QWidget):
    def __init__(self, screen):
        super().__init__()
        self.screen = screen
        self.messages = []
        self.type_name = ''
        self.run_dir = None
        lay = QVBoxLayout(self)
        split = QSplitter(Qt.Horizontal)
        left = QWidget()
        ll = QVBoxLayout(left)
        ll.setContentsMargins(0, 0, 0, 0)
        self.tree = QTreeWidget()
        self.tree.setHeaderLabels(['Run / topic', 'Messages'])
        self.tree.itemExpanded.connect(self.expand)
        self.tree.currentItemChanged.connect(self.select)
        ll.addWidget(self.tree, 1)
        self.monitor_btn = button('Open in maze monitor', self.open_monitor,
                                  'Replay the selected run in maze_monitor')
        ll.addWidget(self.monitor_btn)
        split.addWidget(left)
        mid = QWidget()
        ml = QVBoxLayout(mid)
        ml.setContentsMargins(0, 0, 0, 0)
        self.page = QComboBox()
        self.page.currentIndexChanged.connect(self.fill_page)
        ml.addWidget(row(QLabel('Messages'), None, self.page))
        self.list = QTableWidget(0, 3)
        self.list.setHorizontalHeaderLabels(['Time', 'Message', ''])
        self.list.verticalHeader().setVisible(False)
        self.list.setEditTriggers(QTableWidget.NoEditTriggers)
        self.list.setSelectionBehavior(QTableWidget.SelectRows)
        self.list.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        self.list.currentCellChanged.connect(lambda r, *_: self.show_message(r))
        ml.addWidget(self.list, 1)
        split.addWidget(mid)
        self.text = QPlainTextEdit()
        self.text.setReadOnly(True)
        self.text.setFont(style.mono_font())
        self.text.setTextInteractionFlags(Qt.TextSelectableByMouse | Qt.TextSelectableByKeyboard)
        split.addWidget(self.text)
        split.setSizes([300, 360, 560])
        lay.addWidget(split, 1)

    def set_runs(self, runs):
        self.tree.clear()
        for r in runs:
            it = QTreeWidgetItem([f'{r["run_id"]}  {r["outcome"] or ""}', ''])
            it.setData(0, Qt.UserRole, ('run', r['run_id']))
            it.setChildIndicatorPolicy(QTreeWidgetItem.ShowIndicator)
            self.tree.addTopLevelItem(it)
        self.tree.resizeColumnToContents(0)

    def run_dir_of(self, item):
        while item is not None and item.parent() is not None:
            item = item.parent()
        if item is None:
            return None
        return self.screen.exp.run_dir(item.data(0, Qt.UserRole)[1])

    def expand(self, item):
        kind, rid = item.data(0, Qt.UserRole)
        if kind != 'run' or item.childCount():
            return
        run_dir = self.screen.exp.run_dir(rid)

        def done(topics):
            for name, type_name, n in topics:
                ch = QTreeWidgetItem([name, str(n)])
                ch.setToolTip(0, type_name)
                ch.setData(0, Qt.UserRole, ('topic', name, type_name))
                item.addChild(ch)
            if not topics:
                item.addChild(QTreeWidgetItem(['(no recording)', '']))

        def failed(text):
            item.addChild(QTreeWidgetItem([f'(recording not readable: {text.splitlines()[0]})',
                                           '']))
        background(lambda: bag_topics(run_dir), done, failed, parent=self)

    def select(self, item, _prev=None):
        if item is None:
            return
        data = item.data(0, Qt.UserRole)
        self.run_dir = self.run_dir_of(item)
        if not data or data[0] != 'topic':
            return
        _, topic, type_name = data
        self.type_name = type_name
        run_dir = self.run_dir
        try:
            with open(os.path.join(run_dir, 'run.json')) as f:
                start_ns = json.load(f).get('start_ns')
        except (OSError, ValueError):
            start_ns = None

        def done(msgs):
            self.messages = msgs
            self.page.blockSignals(True)
            self.page.clear()
            for p in range(0, max(len(msgs), 1), PAGE):
                self.page.addItem(f'{p + 1}–{min(p + PAGE, len(msgs))} of {len(msgs)}'
                                  + (' (first 5000)' if len(msgs) >= 5000 else ''))
            self.page.blockSignals(False)
            self.fill_page()
        background(lambda: topic_messages(run_dir, topic, start_ns), done, parent=self,
                   progress_text=f'Reading {topic}…')

    def _deserialize(self, data):
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        return deserialize_message(data, get_message(self.type_name))

    def text_of(self, index):
        from ..metrics.echo import to_text
        t, data = self.messages[index]
        try:
            body = to_text(self._deserialize(data))
        except Exception as exc:
            body = f'(cannot decode {self.type_name}: {exc})'
        return f'# t = {t:.3f} s (receive time since the run started)\n{body}'

    def fill_page(self, *_):
        p = max(self.page.currentIndex(), 0) * PAGE
        chunk = range(p, min(p + PAGE, len(self.messages)))
        self.list.setRowCount(len(chunk))
        for r, i in enumerate(chunk):
            t, data = self.messages[i]
            self.list.setItem(r, 0, QTableWidgetItem(fmt.duration(t) + f'.{int(t * 10) % 10}'))
            preview = ''
            try:
                from ..metrics.echo import lines
                preview = ' · '.join(x.strip() for x in lines(self._deserialize(data))[:3])
            except Exception:
                pass
            self.list.setItem(r, 1, QTableWidgetItem(preview[:200]))
            b = QPushButton('Copy')
            b.setToolTip('Copy this message as text')
            b.clicked.connect(lambda _c=False, i=i: (copy_text(self.text_of(i)),
                                                     self.screen.win.statusBar().showMessage(
                                                         'Copied', 3000)))
            self.list.setCellWidget(r, 2, b)
        self.list.resizeColumnToContents(0)
        self.list.resizeColumnToContents(2)
        if chunk:
            self.list.setCurrentCell(0, 1)

    def show_message(self, r):
        p = max(self.page.currentIndex(), 0) * PAGE + r
        if 0 <= p < len(self.messages):
            self.text.setPlainText(self.text_of(p))

    def open_monitor(self):
        item = self.tree.currentItem()
        run_dir = self.run_dir_of(item)
        if run_dir is None or not os.path.isdir(os.path.join(run_dir, 'bag')):
            QMessageBox.information(self, 'No recording', 'Select a run with a recording.')
            return
        subprocess.Popen(['ros2', 'run', 'maze_monitor', 'maze_monitor', '--no-ros',
                          '--replay', run_dir], start_new_session=True,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.screen.win.statusBar().showMessage('Opening the run in maze_monitor…', 5000)

    def export_selection(self):
        """Raw messages of the selected topic of the selected run, for the export (ER8)."""
        item = self.tree.currentItem()
        if item is None:
            return None
        data = item.data(0, Qt.UserRole)
        if not data or data[0] != 'topic':
            return None
        if QMessageBox.question(self, 'Export', f'Add the raw messages of {data[1]} of this run '
                                f'(first 20) to the Markdown export?') != QMessageBox.Yes:
            return None
        return {'run': os.path.basename(self.run_dir_of(item)), 'topics': [data[1]],
                'max_messages': 20}
