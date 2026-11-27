"""The run screen (GR6): every run, the concurrency, the resource graphs; Pause, Resume, Cancel.

It only reads the experiment's files (live.json, manager.jsonl, resources.jsonl
and the run folders), so it shows the same live state after the GUI was
closed and opened again (BNR5).
"""

import datetime

from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtWidgets import (QHeaderView, QLabel, QMessageBox, QSplitter,
                                         QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget)

from maze_monitor.gui.widgets import TimePlot

from .. import experiment as exp_mod, fileio, manager_control
from ..metrics import fmt
from .common import OUTCOME_COLORS, STATE_COLORS, background, button, heading, row, style

WINDOW_S = 900.0   # resource graphs show the last 15 minutes


def snapshot(exp):
    """Everything the screen shows, read off the GUI thread."""
    exp = exp_mod.Experiment(exp.path)
    live = None
    alive = manager_control.manager_alive(exp)
    try:
        live = fileio.read_json(exp.p('live.json'))
    except (OSError, ValueError):
        pass
    attempts = exp.attempts()
    plan = exp.plan()
    resources_tail = fileio.tail_jsonl(exp.resources_log, 400_000)
    events = [r for r in fileio.tail_jsonl(exp.manager_log, 30_000)
              if r.get('event') in ('outcome', 'retried', 'resources', 'error', 'host_event',
                                    'calibration', 'message', 'control', 'started')]
    return {'exp': exp, 'live': live if alive else None, 'alive': alive,
            'attempts': attempts, 'plan': plan, 'resources': resources_tail,
            'events': events[-8:]}


class RunScreen(QWidget):
    def __init__(self, win, exp):
        super().__init__()
        self.win = win
        self.exp = exp
        self.busy = False
        self.finished_seen = exp.state in exp_mod.FINAL_STATES
        lay = QVBoxLayout(self)
        self.title = heading(f'Experiment: {exp.name}', 3)
        lay.addWidget(self.title)
        self.state = QLabel('')
        self.state.setWordWrap(True)
        self.pause_btn = button('Pause', self.pause, 'Start no new runs; active runs finish')
        self.resume_btn = button('Resume', self.resume)
        self.cancel_btn = button('Cancel', self.cancel, 'Stop active runs and end the experiment')
        self.results_btn = button('Results so far', self.results)
        lay.addWidget(row(self.state, None, self.pause_btn, self.resume_btn, self.cancel_btn,
                          self.results_btn, stretch_first=True))
        split = QSplitter(Qt.Vertical)
        self.table = QTableWidget(0, 8)
        self.table.setHorizontalHeaderLabels(['Run', 'Set', 'Maze', 'Repeat', 'State', 'Sim time',
                                              'RTF', 'Outcome'])
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setStretchLastSection(True)
        split.addWidget(self.table)
        bottom = QWidget()
        bl = QVBoxLayout(bottom)
        bl.setContentsMargins(0, 0, 0, 0)
        self.plot = TimePlot()
        self.plot.setMinimumHeight(260)
        bl.addWidget(self.plot, 1)
        self.events = QLabel('')
        self.events.setWordWrap(True)
        self.events.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.events.setFont(style.mono_font(-1))
        self.events.setMaximumHeight(150)
        bl.addWidget(self.events)
        split.addWidget(bottom)
        split.setSizes([330, 400])
        lay.addWidget(split, 1)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.poll)
        self.timer.start(1000)
        self.poll()

    def activate(self):
        self.poll()

    def poll(self):
        if self.busy or not self.isVisible() and self.win.stack.currentWidget() is not self:
            return
        self.busy = True

        def done(snap):
            self.busy = False
            self.show_snapshot(snap)
        background(lambda: snapshot(self.exp), done,
                   on_error=lambda _t: setattr(self, 'busy', False), parent=self)

    # ---------------------------------------------------------------- drawing
    def show_snapshot(self, snap):
        exp, live = snap['exp'], snap['live']
        self.exp = exp
        state = exp.state
        color = STATE_COLORS.get(state, style.BLACK).name()
        s = exp.meta.get('summary') or {}
        text = (f'<b style="color:{color}">{state}</b> — {s.get("robot_results", 0)} of '
                f'{exp.meta.get("planned_runs")} runs done, {s.get("excluded", 0)} excluded '
                f'attempt(s)')
        if s.get('processed'):
            text += f', mission success {s["successes"]}/{s["processed"]}'
        if live:
            running = [a for a in live['active'] if a['phase'] != 'stopping']
            text += (f'<br>Concurrency: {len(running)} running (target {live["target"]}, limit '
                     f'{live["cap"]}){" — held back: a run is below the minimum RTF" if live.get("held_back") else ""}'
                     f'{"" if live.get("calibrated") else " — calibrating with the first run"}; '
                     f'{live["queue"]} queued, {live["processing"]} to process.')
            for m in live.get('messages') or []:
                c = style.RED if m['level'] == 'error' else style.AMBER
                text += f'<br><span style="color:{c.name()}">{m["text"]}</span>'
        elif state in exp_mod.ACTIVE_STATES:
            text += '<br>No manager is running this experiment.'
        self.state.setText(text)
        alive = snap['alive']
        self.pause_btn.setEnabled(alive and state != 'paused')
        self.resume_btn.setEnabled(state in ('paused', 'interrupted'))
        self.cancel_btn.setEnabled(state not in exp_mod.FINAL_STATES)
        self.fill_table(snap)
        self.fill_plot(snap['resources'])
        lines = []
        for r in snap['events']:
            detail = {k: v for k, v in r.items() if k not in ('time', 'event')}
            short = ', '.join(f'{k}={v}' for k, v in detail.items()
                              if not isinstance(v, (dict, list)))[:140]
            lines.append(f'{r["time"][11:19]}  {r["event"]}: {short}')
        self.events.setText('\n'.join(lines))
        if state in exp_mod.FINAL_STATES and not self.finished_seen:
            self.finished_seen = True
            if state == 'completed':
                self.win.experiment_finished(exp)   # FR7

    def fill_table(self, snap):
        live = {a['run']: a for a in (snap['live'] or {}).get('active', [])}
        sets = snap['exp'].sets()
        rows = []
        for planned in snap['plan']:
            recs = snap['attempts'].get(planned.key, [])
            last = recs[-1] if recs else None
            rid = last.get('run_id') if last else planned.run_id(1)
            a = live.get(rid)
            if a:
                st, sim, rtf, outc = a['phase'], a.get('sim'), a.get('rtf'), a.get('outcome')
            elif last and last.get('outcome'):
                outc = last['outcome']
                st = 'done' if last.get('finished') else 'processing'
                if outc in exp_mod.EXCLUDED_OUTCOMES:
                    st = 'excluded'
                sim, rtf = last.get('sim_s'), last.get('rtf_mean')
            else:
                st, sim, rtf, outc = 'queued', None, None, None
            attempts = f' (try {last["attempt"]})' if last and last.get('attempt', 1) > 1 else ''
            rows.append((rid + attempts, sets[planned.set_index]['name'], planned.maze_index,
                         planned.repeat, st, sim, rtf, outc))
        order = {'starting': 0, 'running': 0, 'stopping': 0, 'processing': 1, 'queued': 2,
                 'done': 3, 'excluded': 3}
        rows.sort(key=lambda r: order.get(r[4], 4))
        self.table.setRowCount(len(rows))
        for i, (rid, set_name, maze, rep, st, sim, rtf, outc) in enumerate(rows):
            cells = [rid, set_name, str(maze), str(rep), st, fmt.duration(sim),
                     '—' if rtf is None else f'{rtf:.2f}', outc or '']
            for j, text in enumerate(cells):
                it = QTableWidgetItem(text)
                if j == 7 and outc:
                    it.setForeground(OUTCOME_COLORS.get(outc, style.BLACK))
                if j == 6 and rtf is not None:
                    limit = float(snap['exp'].config.data['run_limits'].get('min_rtf', 0.8))
                    if rtf < limit:
                        it.setForeground(style.AMBER)
                if j == 4 and st in ('running', 'starting'):
                    it.setForeground(style.BLUE)
                self.table.setItem(i, j, it)

    def fill_plot(self, recs):
        if not recs:
            self.plot.set_data([], 0, 1, 0, [])
            return
        t_end = recs[-1]['t']
        recs = [r for r in recs if r['t'] >= t_end - WINDOW_S]
        t_start = recs[0]['t']
        times = [r['t'] - t_start for r in recs]
        cpu = [(r['machine'].get('cpu') or 0) * 100 for r in recs]
        mem = [None if not r['machine'].get('mem_total') else
               100 * (1 - r['machine']['mem_available'] / r['machine']['mem_total'])
               for r in recs]
        conc = [r.get('concurrency') for r in recs]
        target = [r.get('target') for r in recs]
        run_ids = sorted({rid for r in recs for rid in r.get('runs', {})})
        rtf_traces = []
        for rid in run_ids[-8:]:
            rtf_traces.append(([t for t, r in zip(times, recs) if rid in r['runs']],
                               [r['runs'][rid].get('rtf') for r in recs if rid in r['runs']],
                               style.BLACK))
        limit = float(self.exp.config.data['run_limits'].get('min_rtf', 0.8))
        rtf_traces.append(([times[0], times[-1]], [limit, limit], style.AMBER))
        lanes = [('Machine CPU', '%', [(times, cpu, style.BLACK)], 1.0),
                 ('Machine memory used', '%', [(times, mem, style.BLACK)], 1.0),
                 ('Runs at once (blue: target)', '',
                  [(times, target, style.BLUE), (times, conc, style.BLACK)], 1.0),
                 ('RTF per run (amber: minimum)', '', rtf_traces, 1.0)]
        self.plot.set_data(lanes, 0.0, max(times[-1], 1.0), times[-1], [])

    # ---------------------------------------------------------------- actions
    def pause(self):
        try:
            self.win.suite.pause(self.exp)
        except Exception as exc:
            QMessageBox.warning(self, 'Cannot pause', str(exc))
        self.poll()

    def resume(self):
        try:
            self.win.suite.resume(self.exp)
        except Exception as exc:
            QMessageBox.warning(self, 'Cannot resume', str(exc))
        self.poll()

    def cancel(self):
        if QMessageBox.question(self, 'Cancel experiment',
                                'Stop the active runs and end this experiment? Finished runs '
                                'are kept.') != QMessageBox.Yes:
            return
        self.win.suite.cancel(self.exp)
        self.poll()

    def results(self):
        from .results_screen import ResultsScreen
        self.win.screens.pop(('results', self.exp.path), None)
        self.win.show_screen(('results', self.exp.path), lambda: ResultsScreen(self.win,
                                                                                self.exp))
