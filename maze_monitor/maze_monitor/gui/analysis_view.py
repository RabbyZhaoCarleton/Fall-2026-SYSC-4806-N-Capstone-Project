"""Analysis view: cards that interpret the data at the cursor."""

import html
import math

from python_qt_binding.QtCore import QSettings, Qt, QTimer, Signal
from python_qt_binding.QtWidgets import (QAction, QBoxLayout, QCheckBox, QComboBox, QHBoxLayout, QLabel,
                                         QMenu, QPushButton, QScrollArea, QSizePolicy,
                                         QSplitter, QToolButton, QVBoxLayout, QWidget)

from . import style
from .widgets import (Card, HeadingDial, KeyValueGrid, MapView, SpeedBars, StatusBanner,
                      SurroundingsView, TimePlot, clearance_color)
from ..analysis.diagnosis import activity, diagnose
from ..analysis.geometry import Pose2D, direction_name, wrap
from ..analysis.interpreters import HealthInterpreter

NAV_STATUS = '/navigate_to_pose/_action/status'
NAV_FEEDBACK = '/navigate_to_pose/_action/feedback'

# Values that are plotted together in one lane: (commanded, actual).
PAIRS = {'cmd.v': 'odom.v', 'cmd.w': 'odom.w'}
DEFAULT_PLOTS = ['clear.min', 'cmd.v', 'odom.v', 'nav.distance_remaining']
WINDOWS = [('Last 30 s', 30.0), ('Last 2 min', 120.0), ('Last 10 min', 600.0), ('Whole run', None)]


def _decimate(xs, ys, n=2500):
    if len(xs) <= n:
        return list(xs), list(ys)
    step = len(xs) / n
    idx = [int(i * step) for i in range(n)] + [len(xs) - 1]
    return [xs[i] for i in idx], [ys[i] for i in idx]


class AnalysisView(QWidget):
    seek_requested = Signal(float)

    def __init__(self, config, parent=None):
        super().__init__(parent)
        self.config = config
        self.settings = QSettings('maze_monitor', 'maze_monitor')
        keys = self.settings.value('plots/keys', DEFAULT_PLOTS) or DEFAULT_PLOTS
        self.plot_keys = [keys] if isinstance(keys, str) else list(keys)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)

        # Status card across the top.
        self.status_card = Card('Status')
        self.banner = self.status_card.add(StatusBanner())
        self.status_text = self.status_card.add(QLabel())
        self.status_text.setWordWrap(True)
        self.status_text.setTextFormat(Qt.RichText)
        self.status_text.setTextInteractionFlags(Qt.TextSelectableByMouse)
        outer.addWidget(self.status_card)

        split = QSplitter(Qt.Horizontal)
        outer.addWidget(split, 1)

        # Left column: map and plots.
        left = QSplitter(Qt.Vertical)
        self.map_card = Card('Map', ('/map', '/odom', '/tf', '/plan', '/explore/frontiers', '/scan'))
        bar = QHBoxLayout()
        self.follow_box = QCheckBox('Follow')
        self.follow_box.setToolTip('Keep the robot in the center of the map')
        self.follow_box.toggled.connect(self._follow_changed)
        self.scan_box = QCheckBox('Lidar')
        self.scan_box.setToolTip('Draw the lidar points on the map')
        self.scan_box.setChecked(True)
        self.all_events_box = QCheckBox('Info events')
        self.all_events_box.setToolTip('Also mark info events on the map, not only warnings and errors')
        fit = QPushButton('Fit')
        fit.setToolTip('Fit the whole map in view (or double-click the map)')
        self.trail_combo = QComboBox()
        for label, _ in (('Trail: all', None), ('Trail: 60 s', 60.0), ('Trail: 10 s', 10.0)):
            self.trail_combo.addItem(label)
        for w in (self.follow_box, self.scan_box, self.all_events_box, self.trail_combo):
            bar.addWidget(w)
        bar.addStretch(1)
        bar.addWidget(fit)
        self.map_card.body.addLayout(bar)
        self.map = self.map_card.add(MapView(), 1)
        fit.clicked.connect(self.map.fit)
        self.map.seek_requested.connect(self.seek_requested)
        legend = QLabel('Black: robot · Blue: plan, goal · Purple: frontiers · Dots: events')
        legend.setStyleSheet(f'color: {style.GRAY.name()};')
        legend.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Fixed)
        legend.setToolTip('Black: the robot (circle and heading line) and its trail.\n'
                          'Blue: the planned path and the goal (X).\n'
                          'Purple: exploration frontiers.\n'
                          'Amber and red dots: warnings and errors where they happened; '
                          'a ring marks events active at the cursor. Click a dot to jump to it.\n'
                          'Gray points: the lidar scan at the cursor.\n'
                          'Scroll to zoom, drag to pan, double-click to fit.')
        self.map_card.add(legend)
        left.addWidget(self.map_card)

        self.plot_card = Card('Plots')
        pbar = QHBoxLayout()
        self.plot_menu_button = QToolButton()
        self.plot_menu_button.setText('Values…')
        self.plot_menu_button.setPopupMode(QToolButton.InstantPopup)
        self.plot_menu = QMenu(self)
        self.plot_menu_button.setMenu(self.plot_menu)
        self.plot_menu.aboutToShow.connect(self._fill_plot_menu)
        self.window_combo = QComboBox()
        for label, _ in WINDOWS:
            self.window_combo.addItem(label)
        self.window_combo.setCurrentIndex(int(self.settings.value('plots/window', 1)))
        self.window_combo.currentIndexChanged.connect(
            lambda i: self.settings.setValue('plots/window', i))
        pbar.addWidget(self.plot_menu_button)
        pbar.addWidget(self.window_combo)
        hint = QLabel('Blue: commanded · Black: measured · Shading: warnings and errors · '
                      'Click to jump')
        hint.setStyleSheet(f'color: {style.GRAY.name()};')
        hint.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        hint.setToolTip(hint.text())
        pbar.addWidget(hint, 1)
        self.plot_card.body.addLayout(pbar)
        self.plot = self.plot_card.add(TimePlot(), 1)
        self.plot.seek_requested.connect(self.seek_requested)
        left.addWidget(self.plot_card)
        left.setStretchFactor(0, 3)
        left.setStretchFactor(1, 2)
        split.addWidget(left)

        # Right column: the smaller cards, scrollable.
        right = QWidget()
        rl = QVBoxLayout(right)
        rl.setContentsMargins(0, 0, 0, 0)
        row = QBoxLayout(QBoxLayout.LeftToRight)
        self.dial_row = row
        self.right_column = right
        self.heading_card = Card('Heading', ('/odom', '/tf'))
        self.dial = self.heading_card.add(HeadingDial(), 1)
        self.heading_text = self.heading_card.add(QLabel())
        self.heading_text.setWordWrap(True)
        row.addWidget(self.heading_card, 1)
        self.sur_card = Card('Surroundings', ('/scan',))
        self.sur = self.sur_card.add(SurroundingsView(), 1)
        self.sur_text = self.sur_card.add(QLabel())
        self.sur_text.setWordWrap(True)
        row.addWidget(self.sur_card, 1)
        rl.addLayout(row)

        self.motion_card = Card('Motion', ('/cmd_vel', '/odom'))
        self.bars = self.motion_card.add(SpeedBars())
        self.motion_text = self.motion_card.add(QLabel())
        self.motion_text.setWordWrap(True)
        self.motion_kv = self.motion_card.add(KeyValueGrid(1))
        rl.addWidget(self.motion_card)

        self.nav_card = Card('Navigation', ('/plan', NAV_STATUS, NAV_FEEDBACK, '/behavior_tree_log'))
        self.nav_kv = self.nav_card.add(KeyValueGrid(1))
        rl.addWidget(self.nav_card)

        self.exp_card = Card('Exploration', ('/map', '/explore/frontiers', '/explore/status'))
        self.exp_kv = self.exp_card.add(KeyValueGrid(1))
        rl.addWidget(self.exp_card)

        self.health_card = Card('Health', ('/clock', '/rosout'))
        self.health_kv = self.health_card.add(KeyValueGrid(1))
        rl.addWidget(self.health_card)

        self.sum_card = Card('Run summary')
        self.sum_kv = self.sum_card.add(KeyValueGrid(1))
        rl.addWidget(self.sum_card)
        rl.addStretch(1)

        scroll = QScrollArea()
        scroll.setMinimumWidth(380)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setWidgetResizable(True)
        scroll.setWidget(right)
        scroll.setFrameShape(QScrollArea.NoFrame)
        split.addWidget(scroll)
        split.setStretchFactor(0, 1)
        split.setStretchFactor(1, 0)
        split.setChildrenCollapsible(False)
        self.split = split
        self._sized = False
        split.splitterMoved.connect(self._splitter_moved)
        self.cards = [self.map_card, self.plot_card, self.heading_card, self.sur_card,
                      self.motion_card, self.nav_card, self.exp_card, self.health_card]
        self._available = []

    def _splitter_moved(self, *_):
        self._sized = True
        self._arrange_dials()

    def resizeEvent(self, ev):
        super().resizeEvent(ev)
        # Splitter stretch factors only share extra space, so keep a 58/42 split
        # until the user drags the divider.
        QTimer.singleShot(0, self._apply_split)

    def _apply_split(self):
        if not self._sized:
            w = self.split.width()
            self.split.setSizes([int(w * 0.58), w - int(w * 0.58)])
        self._arrange_dials()

    def _arrange_dials(self):
        # Heading and Surroundings side by side when there is room, else stacked.
        wide = self.right_column.width() >= 470
        want = QBoxLayout.LeftToRight if wide else QBoxLayout.TopToBottom
        if self.dial_row.direction() != want:
            self.dial_row.setDirection(want)

    # ------------------------------------------------------------ settings
    def _follow_changed(self, on):
        self.map.follow = on

    def _fill_plot_menu(self):
        self.plot_menu.clear()
        for key, label in self._available:
            act = QAction(label, self.plot_menu)
            act.setCheckable(True)
            act.setChecked(key in self.plot_keys)
            act.toggled.connect(lambda on, k=key: self._toggle_plot(k, on))
            self.plot_menu.addAction(act)
        if not self._available:
            self.plot_menu.addAction('No values yet').setEnabled(False)

    def _toggle_plot(self, key, on):
        if on and key not in self.plot_keys:
            self.plot_keys.append(key)
        elif not on and key in self.plot_keys:
            self.plot_keys.remove(key)
        self.settings.setValue('plots/keys', self.plot_keys)

    # ------------------------------------------------------------ update
    def update_view(self, session, t, visible, units):
        tl = session.timeline
        snap = tl.snapshot(t)
        cfg = self.config

        for card in self.cards:
            card.setVisible(not card.topics or any(visible(x) for x in card.topics))

        # ---- status
        level, headline, lines, problems = diagnose(snap, units)
        self.banner.set_level(level, headline)
        parts = [' '.join(html.escape(s) for s in lines)]
        for k, e in enumerate(problems[:4]):
            col = style.SEVERITY[e.severity].name()
            since = f'since {style.fmt_time(e.t)}' if e.is_condition else f'at {style.fmt_time(e.t)}'
            line = (f'<span style="color:{col}"><b>{html.escape(e.title[:110])}</b></span> '
                    f'<span style="color:{style.GRAY.name()}">({since})</span>')
            if k == 0 and e.detail:
                line += ' — ' + html.escape(e.detail[:300])
            parts.append(line)
        if len(problems) > 4:
            parts.append(f'<span style="color:{style.GRAY.name()}">… and {len(problems) - 4} more '
                         'active; see the Events panel.</span>')
        self.status_text.setText('<br>'.join(parts))

        g = snap.get
        pose = None
        if g('pose.x') is not None:
            pose = Pose2D(g('pose.x'), g('pose.y'), g('pose.yaw', 0.0))
        nav_active = g('nav.state') == 'active'
        goal = (g('nav.goal_x'), g('nav.goal_y')) if nav_active and g('nav.goal_x') is not None else None

        # ---- map
        if self.map_card.isVisible():
            self._update_map(tl, t, pose, goal, nav_active, visible)

        # ---- heading
        if self.heading_card.isVisible():
            gb = math.atan2(goal[1] - pose.y, goal[0] - pose.x) if goal and pose else None
            self.dial.set_state(pose.yaw if pose else None, g('cmd.w') if visible('/cmd_vel') else None,
                                g('odom.w'), gb, units, g('pose.frame', 'odom'))
            text = []
            if pose is not None:
                text.append(f'Facing {units.angle(pose.yaw)} in the {g("pose.frame", "odom")} frame.')
            if g('odom.w') is not None and abs(g('odom.w')) >= 0.05:
                side = 'left (counterclockwise)' if g('odom.w') > 0 else 'right (clockwise)'
                text.append(f'Turning {side} at {units.rate(abs(g("odom.w")))}.')
            if gb is not None and pose is not None:
                off = wrap(gb - pose.yaw)
                text.append(f'Goal is {units.angle(abs(off), 0)} to the {"left" if off > 0 else "right"}.')
            self.heading_text.setText(' '.join(text))

        # ---- surroundings
        if self.sur_card.isVisible():
            track = tl.tracks.get('/scan')
            scan = track.latest(t)[1] if track is not None else None
            clear = {k: g(f'clear.{k}') for k in ('front', 'left', 'back', 'right')}
            near, contact = cfg.th('near_wall_clearance'), cfg.th('contact_clearance')
            self.sur.set_state(scan, cfg.robot_radius, near, contact, clear,
                               g('clear.min'), g('clear.bearing'), units)
            n = g('clear.min')
            if n is not None:
                b = g('clear.bearing')
                col = clearance_color(n, near, contact)
                verdict = 'touching' if n <= contact else 'too close' if n < near else 'safe distance'
                self.sur_text.setText(f'<span style="color:{col.name()}">Nearest wall {max(n, 0):.3f} m, '
                                      f'{direction_name(b)} ({units.angle(b, 0)}): {verdict}.</span>'
                                      f' Valid readings: {g("scan.valid_pct", 0):.0f}%.')
            else:
                self.sur_text.setText('No lidar data.')

        # ---- motion
        if self.motion_card.isVisible():
            cmd_v, cmd_w = g('cmd.v'), g('cmd.w')
            act_v, act_w = g('odom.v'), g('odom.w')
            self.bars.set_values(cmd_v, act_v, cmd_w, act_w, units)
            told = activity(cmd_v, cmd_w, units) if cmd_v is not None else None
            did = activity(act_v, act_w, units)
            if told is None:
                txt = f'Actual: {did}. No commands received.'
            elif told == did:
                txt = f'Doing what it is told: {did}.'
            else:
                txt = f'Told: {told}. Actual: {did}.'
            self.motion_text.setText(txt)
            cmd_age = snap.age('cmd.v')
            self.motion_kv.set_rows([
                ('Last command', f'{cmd_age:.1f} s ago' if cmd_age is not None else '—'),
                ('Distance driven', f'{g("pose.distance", 0):.2f} m'),
                ('Max speed (run)', f'{self._max(tl, "odom.v", t):.2f} m/s'),
                ('IMU accel', f'{g("imu.accel"):.2f} m/s²' if g('imu.accel') is not None else '—'),
            ])

        # ---- navigation
        if self.nav_card.isVisible():
            state = g('nav.state', 'no goal yet')
            col = {'succeeded': style.GREEN, 'aborted': style.RED, 'canceled': style.GRAY}.get(state)
            start = g('nav.goal_start')
            dr = g('nav.distance_remaining') if nav_active else None
            self.nav_kv.set_rows([
                ('Goal state', state, col),
                ('Time on goal', f'{t - start:.1f} s' if (nav_active and start is not None) else '—'),
                ('Distance left', f'{dr:.2f} m' if dr is not None else '—'),
                ('Path length', f'{g("nav.path_length"):.2f} m' if g('nav.path_length') is not None else '—'),
                ('Goal position', f'({goal[0]:.2f}, {goal[1]:.2f})' if goal else '—'),
                ('Goals reached', f'{int(g("nav.goals_succeeded", 0))}'),
                ('Goals failed', f'{int(g("nav.goals_failed", 0))}',
                 style.RED if g('nav.goals_failed', 0) else None),
                ('Recoveries', f'{int(g("nav.goal_recoveries", 0))} this goal, {int(g("nav.recoveries", 0))} total',
                 style.AMBER if g('nav.recoveries', 0) else None),
                ('Route changes', f'{int(g("nav.route_changes", 0))}'),
                ('Planner failures', f'{int(g("nav.plan_failures", 0))}',
                 style.AMBER if g('nav.plan_failures', 0) else None),
            ])

        # ---- exploration
        if self.exp_card.isVisible():
            known = g('map.known_area')
            past = tl.value('map.known_area', t - 30.0)
            growth = (known - past) * 2.0 if known is not None and past is not None else None
            self.exp_kv.set_rows([
                ('Status', (g('explore.status') or '—').replace('_', ' ')),
                ('Known area', f'{known:.2f} m²' if known is not None else '—'),
                ('Growth (30 s)', f'{growth:.2f} m²/min' if growth is not None else '—',
                 style.AMBER if growth is not None and growth <= 0.0 and g('explore.frontiers', 0) else None),
                ('Frontiers', f'{int(g("explore.frontiers"))}' if g('explore.frontiers') is not None else '—'),
                ('Postponed', f'{int(g("explore.postponed", 0))}'),
                ('Abandoned', f'{int(g("explore.blacklisted", 0))}',
                 style.AMBER if g('explore.blacklisted', 0) else None),
                ('Exits found', f'{int(g("explore.exits", 0))}'),
                ('Map size', f'{g("map.width", 0):.1f} × {g("map.height", 0):.1f} m'
                 if g('map.width') is not None else '—'),
            ])

        # ---- health
        if self.health_card.isVisible():
            rows = []
            for name in HealthInterpreter.STEADY + ('/map', '/plan'):
                tr = tl.tracks.get(name)
                if tr is None or not visible(name):
                    continue
                last_t, _ = tr.latest(t)
                rate = tr.rate_at(t, 3.0)
                silent = any(e.key == f'silent:{name}' for e in snap.events)
                rows.append((name, f'{rate:.1f} Hz' + (f'  silent {t - last_t:.1f} s' if silent and last_t is not None else ''),
                             style.AMBER if silent else None))
            rtf = g('sim.rtf')
            rows.append(('Real-time factor', f'{rtf:.2f}×' if rtf is not None else '—',
                         style.AMBER if rtf is not None and rtf < cfg.th('slow_rtf') else None))
            rows.append(('Log warnings', f'{int(g("log.warnings", 0))}',
                         style.AMBER if g('log.warnings', 0) else None))
            rows.append(('Log errors', f'{int(g("log.errors", 0))}',
                         style.RED if g('log.errors', 0) else None))
            self.health_kv.set_rows(rows)

        # ---- plots
        if self.plot_card.isVisible():
            self._update_plots(tl, t, units)

        # ---- summary
        dur = t
        evs = [e for e in tl.events if e.t <= t]
        nerr = sum(1 for e in evs if e.severity == 'error')
        nwarn = sum(1 for e in evs if e.severity == 'warning')
        dist = g('pose.distance', 0.0)
        self.sum_kv.set_rows([
            ('Time', style.fmt_time(dur)),
            ('Distance', f'{dist:.2f} m'),
            ('Average speed', f'{dist / dur:.3f} m/s' if dur > 0 else '—'),
            ('Errors', str(nerr), style.RED if nerr else None),
            ('Warnings', str(nwarn), style.AMBER if nwarn else None),
            ('Collision events', str(sum(1 for e in evs if e.category == 'collision')),
             style.RED if any(e.category == 'collision' for e in evs) else None),
        ])

    # ------------------------------------------------------------ helpers
    @staticmethod
    def _max(tl, key, t):
        s = tl.series.get(key)
        if s is None or not len(s):
            return 0.0
        i = s.index_at(t)
        return max((abs(v) for v in s.values[:i + 1]), default=0.0)

    def _update_map(self, tl, t, pose, goal, nav_active, visible):
        tr = tl.tracks.get('/map')
        if tr is not None and visible('/map'):
            i = tr.index_at(t)
            self.map.set_grid(tr.message(i) if i >= 0 else None, ('/map', i))
        else:
            self.map.set_grid(None, None)
        trail = None
        sx, sy = tl.series.get('pose.x'), tl.series.get('pose.y')
        if sx is not None and sy is not None:
            win = (None, 60.0, 10.0)[self.trail_combo.currentIndex()]
            t0 = 0.0 if win is None else t - win
            xt, xv = sx.window(t0, t)
            _, yv = sy.window(t0, t)
            n = min(len(xv), len(yv))
            trail = _decimate(list(xv[:n]), list(yv[:n]))
        plan = None
        ptr = tl.tracks.get('/plan')
        if ptr is not None and visible('/plan') and nav_active:
            _, msg = ptr.latest(t)
            if msg is not None and msg.poses:
                plan = ([p.pose.position.x for p in msg.poses], [p.pose.position.y for p in msg.poses])
        frontiers = []
        ftr = tl.tracks.get('/explore/frontiers')
        if ftr is not None and visible('/explore/frontiers'):
            _, msg = ftr.latest(t)
            if msg is not None:
                frontiers = [(m.pose.position.x, m.pose.position.y) for m in msg.markers
                             if m.action == 0 and m.type == 2]
        floor = 0 if self.all_events_box.isChecked() else 1
        sev_rank = {'info': 0, 'warning': 1, 'error': 2}
        events = [(e, e.active_at(t)) for e in tl.events
                  if e.t <= t and e.x is not None and sev_rank[e.severity] >= floor]
        scan_pts = None
        str_ = tl.tracks.get('/scan')
        if self.scan_box.isChecked() and str_ is not None and visible('/scan') and pose is not None:
            _, scan = str_.latest(t)
            if scan is not None:
                xs, ys = [], []
                c, s = math.cos(pose.yaw), math.sin(pose.yaw)
                for k in range(0, len(scan.ranges), 2):
                    r = scan.ranges[k]
                    if not (scan.range_min <= r <= scan.range_max):
                        continue
                    a = scan.angle_min + k * scan.angle_increment
                    lx, ly = r * math.cos(a), r * math.sin(a)
                    xs.append(pose.x + c * lx - s * ly)
                    ys.append(pose.y + s * lx + c * ly)
                scan_pts = (xs, ys)
        if not visible('/odom'):
            pose, trail = None, None
        self.map.set_overlays(pose, trail, plan, goal if visible('/plan') else None,
                              frontiers, events, scan_pts, self.config.robot_radius)

    def _update_plots(self, tl, t, units):
        numeric = sorted(((k, s) for k, s in tl.series.items() if s.numeric and k != 'nav.goal_start'),
                         key=lambda kv: kv[1].label)
        self._available = [(k, f'{s.label} ({s.unit})' if s.unit else s.label) for k, s in numeric]
        win = WINDOWS[self.window_combo.currentIndex()][1]
        t1 = t if win is None else max(t, win)
        t0 = 0.0 if win is None else t1 - win
        if win is None:
            t1 = max(tl.duration, 1.0)
        lanes = []
        used = set()
        for key in self.plot_keys:
            if key in used or key not in tl.series:
                continue
            keys = [key]
            if key in PAIRS and PAIRS[key] in self.plot_keys:
                keys.append(PAIRS[key])
            inverse = {v: k for k, v in PAIRS.items()}
            if key in inverse and inverse[key] in self.plot_keys:
                continue  # drawn in the lane of its commanded partner
            used.update(keys)
            s0 = tl.series[keys[0]]
            unit, scale = s0.unit, 1.0
            if unit.startswith('rad') and units.degrees:
                unit, scale = unit.replace('rad', '°'), 180.0 / math.pi
            traces = []
            for k in keys:
                s = tl.series[k]
                ts, vs = s.window(t0, min(t1, t))
                ts, vs = _decimate(list(ts), list(vs), 1500)
                color = style.BLUE if k.startswith('cmd.') or k.startswith('nav.goal') else style.BLACK
                traces.append((ts, vs, color))
            if len(keys) == 1:
                label = s0.label
            else:
                base = s0.label.replace('Commanded ', '')
                label = base[:1].upper() + base[1:] + ' (commanded and actual)'
            lanes.append((label, unit, traces, scale))
        bands = [(e.t, e.end_or(t), style.SEVERITY[e.severity]) for e in tl.events
                 if e.is_condition and e.severity != 'info' and e.t <= t1 and e.end_or(t) >= t0 and e.t <= t]
        self.plot.set_data(lanes, t0, t1, t, bands)
