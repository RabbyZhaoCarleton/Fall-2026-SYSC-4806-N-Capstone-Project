"""Drawing widgets: heading dial, surroundings, map, speed bars, plots, and cards."""

import math

import numpy as np
from python_qt_binding.QtCore import QPointF, QRectF, Qt, Signal
from python_qt_binding.QtGui import (QBrush, QFontMetrics, QImage, QPainter, QPainterPath,
                                     QPen, QPolygonF)
from python_qt_binding.QtWidgets import (QFrame, QGridLayout, QHBoxLayout, QLabel, QSizePolicy,
                                         QVBoxLayout, QWidget)

from . import style
from ..analysis.diagnosis import Units


def pen(color, width=1.0, style_=Qt.SolidLine, cosmetic=True):
    p = QPen(color, width, style_)
    p.setCosmetic(cosmetic)
    return p


# ======================================================================== card
class Card(QFrame):
    """A titled box. `topics` lists the topics it reads, shown under the title."""

    def __init__(self, title, topics=(), parent=None):
        super().__init__(parent)
        self.topics = tuple(topics)
        self.setFrameShape(QFrame.StyledPanel)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(8, 6, 8, 8)
        outer.setSpacing(4)
        head = QHBoxLayout()
        self.title_label = QLabel(title)
        self.title_label.setFont(style.bold())
        head.addWidget(self.title_label)
        self.source = QLabel(', '.join(self.topics))
        self.source.setStyleSheet(f'color: {style.GRAY.name()};')
        self.source.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        # Never let a long topic list widen the card; the full list is in the tooltip.
        self.source.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        self.source.setMinimumWidth(0)
        self.source.setToolTip('Topics this card reads:\n' + '\n'.join(self.topics))
        head.addWidget(self.source, 1)
        outer.addLayout(head)
        self.body = QVBoxLayout()
        self.body.setSpacing(4)
        outer.addLayout(self.body)

    def add(self, widget, stretch=0):
        self.body.addWidget(widget, stretch)
        return widget


class KeyValueGrid(QWidget):
    """Rows of 'name   value'. Values use a fixed-width font so digits line up."""

    def __init__(self, columns=1, parent=None):
        super().__init__(parent)
        self.columns = columns
        self.grid = QGridLayout(self)
        self.grid.setContentsMargins(0, 0, 0, 0)
        self.grid.setHorizontalSpacing(10)
        self.grid.setVerticalSpacing(2)
        self.labels = []

    def set_rows(self, rows):
        while len(self.labels) < len(rows):
            i = len(self.labels)
            k, v = QLabel(), QLabel()
            k.setStyleSheet(f'color: {style.GRAY.name()};')
            v.setFont(style.mono_font())
            v.setTextInteractionFlags(Qt.TextSelectableByMouse)
            r, c = divmod(i, self.columns)
            self.grid.addWidget(k, r, 2 * c)
            self.grid.addWidget(v, r, 2 * c + 1)
            self.labels.append((k, v))
        for i, (k, v) in enumerate(self.labels):
            if i < len(rows):
                name, value, color = (list(rows[i]) + [None])[:3]
                k.setText(name)
                v.setText(value)
                v.setStyleSheet(f'color: {color.name()};' if color is not None else '')
                k.show()
                v.show()
            else:
                k.hide()
                v.hide()


def clearance_color(c, near, contact):
    if c is None:
        return style.GRAY
    if c <= contact:
        return style.RED
    if c < near:
        return style.AMBER
    return style.BLACK


# ======================================================================== heading
class HeadingDial(QWidget):
    """A circle with a line for the heading. 0 points right (+x in the map), angles grow CCW."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.yaw = None
        self.cmd_w = None
        self.act_w = None
        self.goal_bearing = None
        self.units = Units(True)
        self.frame = 'map'
        self.setMinimumSize(150, 190)
        self.setToolTip('Black line: heading.\nBlue arc: commanded turn over the next second.\n'
                        'Black arc: actual turn over the next second.\n'
                        'Blue dashed line: direction to the goal.\n'
                        '0 points along +x of the map; angles grow counterclockwise.')

    def set_state(self, yaw, cmd_w, act_w, goal_bearing, units, frame):
        self.yaw, self.cmd_w, self.act_w = yaw, cmd_w, act_w
        self.goal_bearing, self.units, self.frame = goal_bearing, units, frame
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        fm = QFontMetrics(self.font())
        text_h = fm.height() * 2 + 6
        w, h = self.width(), self.height() - text_h
        r = max(min(w, h) / 2 - fm.height() - 6, 20)
        c = QPointF(w / 2, h / 2 + 2)

        def at(angle, radius):
            return QPointF(c.x() + radius * math.cos(angle), c.y() - radius * math.sin(angle))

        p.setPen(pen(style.LIGHT, 1.5))
        p.drawEllipse(c, r, r)
        for k in range(12):
            a = k * math.pi / 6
            p.setPen(pen(style.GRAY, 1))
            p.drawLine(at(a, r), at(a, r - (8 if k % 3 == 0 else 4)))
        p.setPen(style.GRAY)
        for a, deg_label, rad_label in ((0, '0°', '0'), (math.pi / 2, '90°', 'π/2'),
                                        (math.pi, '180°', 'π'), (-math.pi / 2, '-90°', '-π/2')):
            label = deg_label if self.units.degrees else rad_label
            pt = at(a, r + fm.height() * 0.5 + 6)
            tw = fm.horizontalAdvance(label)
            p.drawText(QPointF(pt.x() - tw / 2, pt.y() + fm.ascent() / 2 - 1), label)

        if self.goal_bearing is not None:
            p.setPen(pen(style.BLUE, 1.5, Qt.DashLine))
            p.drawLine(c, at(self.goal_bearing, r - 2))

        if self.yaw is not None:
            for w_, color, rr in ((self.cmd_w, style.BLUE, r * 0.93), (self.act_w, style.BLACK, r * 0.82)):
                if w_ is None or abs(w_) < 0.02:
                    continue
                span = max(min(w_, math.pi), -math.pi)
                rect = QRectF(c.x() - rr, c.y() - rr, 2 * rr, 2 * rr)
                p.setPen(pen(color, 3 if color == style.BLUE else 2))
                p.drawArc(rect, int(math.degrees(self.yaw) * 16), int(math.degrees(span) * 16))
            p.setPen(pen(style.BLACK, 3))
            p.drawLine(c, at(self.yaw, r * 0.9))
            p.setBrush(style.BLACK)
            p.drawEllipse(c, 3, 3)

        p.setPen(style.BLACK)
        big = style.mono_font(4)
        p.setFont(big)
        txt = self.units.angle(self.yaw) if self.yaw is not None else 'no data'
        bfm = QFontMetrics(big)
        p.drawText(QPointF((self.width() - bfm.horizontalAdvance(txt)) / 2, h + bfm.ascent() + 4), txt)
        p.setFont(self.font())
        p.setPen(style.GRAY)
        sub = f'{self.frame} frame'
        p.drawText(QPointF((self.width() - fm.horizontalAdvance(sub)) / 2,
                           h + bfm.height() + fm.ascent() + 4), sub)


# ======================================================================== surroundings
class SurroundingsView(QWidget):
    """Top-down lidar view around the robot, robot facing up."""

    RANGES = (0.5, 1.0, 2.0, 4.0)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.scan = None
        self.radius = 0.105
        self.near = 0.05
        self.contact = 0.015
        self.clear = {}
        self.nearest = None
        self.bearing = None
        self.view_range = 1.0
        self.units = Units(True)
        self.setMinimumSize(170, 170)
        self.setToolTip('Lidar points around the robot, robot facing up.\n'
                        'Amber: closer than the warning clearance. Red: contact.\n'
                        'Red ticks at the robot edge: readings below the lidar minimum.\n'
                        'Scroll to change the range.')

    def set_state(self, scan, radius, near, contact, clear, nearest, bearing, units):
        self.scan, self.radius, self.near, self.contact = scan, radius, near, contact
        self.clear, self.nearest, self.bearing, self.units = clear, nearest, bearing, units
        self.update()

    def wheelEvent(self, ev):
        i = self.RANGES.index(self.view_range)
        i = max(0, min(len(self.RANGES) - 1, i + (-1 if ev.angleDelta().y() > 0 else 1)))
        self.view_range = self.RANGES[i]
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        fm = QFontMetrics(self.font())
        margin = fm.height() + 4
        w, h = self.width(), self.height()
        c = QPointF(w / 2, h / 2)
        scale = (min(w, h) / 2 - margin) / self.view_range

        def to_screen(x, y):  # robot frame: x forward, y left
            return QPointF(c.x() - y * scale, c.y() - x * scale)

        step = 0.25 if self.view_range <= 1.0 else 0.5 if self.view_range <= 2.0 else 1.0
        p.setPen(pen(style.LIGHT, 1))
        ring = step
        while ring <= self.view_range + 1e-6:
            p.drawEllipse(c, ring * scale, ring * scale)
            ring += step
        p.setPen(style.GRAY)
        p.drawText(QPointF(c.x() + 3, c.y() - self.view_range * scale + fm.ascent()), f'{self.view_range:g} m')

        p.setPen(pen(style.AMBER, 1, Qt.DashLine))
        rr = (self.radius + self.near) * scale
        p.drawEllipse(c, rr, rr)
        p.setPen(pen(style.BLACK, 1.5))
        p.setBrush(style.FAINT)
        p.drawEllipse(c, self.radius * scale, self.radius * scale)
        p.drawLine(c, to_screen(self.radius, 0))
        p.setBrush(Qt.NoBrush)

        scan = self.scan
        if scan is not None:
            r = np.asarray(scan.ranges, dtype=np.float64)
            ang = scan.angle_min + np.arange(len(r)) * scan.angle_increment
            valid = np.isfinite(r) & (r >= scan.range_min) & (r <= scan.range_max)
            blind = np.isneginf(r) | (np.isfinite(r) & (r < scan.range_min))
            groups = [(style.BLACK, 2.5, []), (style.AMBER, 3.5, []), (style.RED, 3.5, [])]
            for ri, ai in zip(r[valid], ang[valid]):
                if ri > self.view_range * 1.05:
                    continue
                c_ = ri - self.radius
                band = 2 if c_ <= self.contact else 1 if c_ < self.near else 0
                groups[band][2].append(to_screen(ri * math.cos(ai), ri * math.sin(ai)))
            for col, width, pts in groups:
                if pts:
                    p.setPen(pen(col, width))
                    p.drawPoints(QPolygonF(pts))
            p.setPen(pen(style.RED, 2))
            for ai in ang[blind]:
                a = to_screen(self.radius * math.cos(ai), self.radius * math.sin(ai))
                b = to_screen((self.radius + 0.03) * math.cos(ai), (self.radius + 0.03) * math.sin(ai))
                p.drawLine(a, b)
        else:
            p.setPen(style.GRAY)
            p.drawText(self.rect(), Qt.AlignCenter, 'No lidar data')

        if self.nearest is not None and self.bearing is not None:
            d = self.nearest + self.radius
            col = clearance_color(self.nearest, self.near, self.contact)
            a = to_screen(self.radius * math.cos(self.bearing), self.radius * math.sin(self.bearing))
            b = to_screen(d * math.cos(self.bearing), d * math.sin(self.bearing))
            p.setPen(pen(col, 1.5))
            p.drawLine(a, b)

        # Sector clearances at the four edges.
        p.setFont(style.mono_font())
        mfm = QFontMetrics(p.font())
        for name, pos in (('front', 'top'), ('back', 'bottom'), ('left', 'left'), ('right', 'right')):
            v = self.clear.get(name)
            text = f'{name} {max(v, 0):.2f} m' if v is not None else f'{name} —'
            p.setPen(clearance_color(v, self.near, self.contact))
            tw = mfm.horizontalAdvance(text)
            if pos == 'top':
                pt = QPointF(c.x() - tw / 2, mfm.ascent() + 1)
            elif pos == 'bottom':
                pt = QPointF(c.x() - tw / 2, h - 3)
            elif pos == 'left':
                pt = QPointF(2, c.y() + mfm.ascent() + 12)
            else:
                pt = QPointF(w - tw - 2, c.y() + mfm.ascent() + 12)
            p.drawText(pt, text)


# ======================================================================== map
class MapView(QWidget):
    """The occupancy map with the robot, its trail, plan, goal, frontiers, and event marks."""

    seek_requested = Signal(float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(280, 200)
        self.setMouseTracking(True)
        self.grid = None
        self.grid_key = None
        self.image = None
        self.image_rect = None
        self.pose = None
        self.trail = None
        self.plan = None
        self.goal = None
        self.frontiers = []
        self.events = []
        self.scan_pts = None
        self.radius = 0.105
        self.center = None
        self.scale = 120.0  # pixels per metre
        self.follow = False
        self.fitted = False
        self.user_moved = False
        self.drag = None
        self.hover = None
        self.setToolTip('')

    # ------------------------------------------------------------ data
    def set_grid(self, msg, key):
        if key == self.grid_key:
            return
        self.grid_key = key
        self.grid = msg
        if msg is None:
            self.image = None
            return
        from ..analysis.interpreters import grid_array
        info = msg.info
        data = grid_array(msg).reshape(info.height, info.width)
        gray = np.full(data.shape, 205, dtype=np.uint8)  # unknown
        known = data >= 0
        gray[known] = (255 - np.clip(data[known].astype(np.int32), 0, 100) * 255 // 100).astype(np.uint8)
        self._img_buf = np.ascontiguousarray(gray)
        self.image = QImage(self._img_buf.data, info.width, info.height, info.width,
                            QImage.Format_Grayscale8)
        o = info.origin.position
        self.image_rect = QRectF(o.x, o.y, info.width * info.resolution, info.height * info.resolution)
        if not self.fitted:
            self.fit()

    def set_overlays(self, pose, trail, plan, goal, frontiers, events, scan_pts, radius):
        self.pose, self.trail, self.plan, self.goal = pose, trail, plan, goal
        self.frontiers, self.events, self.scan_pts, self.radius = frontiers, events, scan_pts, radius
        if self.follow and pose is not None:
            self.center = QPointF(pose.x, pose.y)
        if self.center is None and pose is not None:
            self.center = QPointF(pose.x, pose.y)
        self.update()

    def fit(self):
        rect = self.image_rect
        if rect is None and self.trail is not None and len(self.trail[0]) > 1:
            xs, ys = self.trail
            rect = QRectF(min(xs), min(ys), max(max(xs) - min(xs), 0.5), max(max(ys) - min(ys), 0.5))
        if rect is None:
            return
        self.center = rect.center()
        self.scale = max(min((self.width() - 20) / max(rect.width(), 0.1),
                             (self.height() - 20) / max(rect.height(), 0.1)), 5.0)
        self.fitted = True
        self.update()

    # ------------------------------------------------------------ transforms
    def to_screen(self, x, y):
        c = self.center or QPointF(0, 0)
        return QPointF(self.width() / 2 + (x - c.x()) * self.scale,
                       self.height() / 2 - (y - c.y()) * self.scale)

    def to_world(self, pt):
        c = self.center or QPointF(0, 0)
        return ((pt.x() - self.width() / 2) / self.scale + c.x(),
                -(pt.y() - self.height() / 2) / self.scale + c.y())

    # ------------------------------------------------------------ mouse
    def resizeEvent(self, ev):
        super().resizeEvent(ev)
        if not self.user_moved:
            self.fit()

    def wheelEvent(self, ev):
        self.user_moved = True
        before = self.to_world(ev.pos())
        self.scale = min(max(self.scale * (1.2 if ev.angleDelta().y() > 0 else 1 / 1.2), 5), 2000)
        after = self.to_world(ev.pos())
        c = self.center or QPointF(0, 0)
        self.center = QPointF(c.x() + before[0] - after[0], c.y() + before[1] - after[1])
        self.update()

    def mousePressEvent(self, ev):
        if ev.button() == Qt.LeftButton:
            hit = self._event_at(ev.pos())
            if hit is not None:
                self.seek_requested.emit(hit.t)
                return
            self.drag = (ev.pos(), QPointF(self.center or QPointF(0, 0)))

    def mouseMoveEvent(self, ev):
        self.hover = self.to_world(ev.pos())
        if self.drag is not None:
            start, c0 = self.drag
            d = ev.pos() - start
            self.center = QPointF(c0.x() - d.x() / self.scale, c0.y() + d.y() / self.scale)
            self.user_moved = True
        hit = self._event_at(ev.pos())
        self.setToolTip(f'{style.fmt_time(hit.t)}  {hit.title}\nClick to jump here.' if hit else '')
        self.update()

    def mouseReleaseEvent(self, ev):
        self.drag = None

    def mouseDoubleClickEvent(self, ev):
        self.user_moved = False
        self.fit()

    def leaveEvent(self, ev):
        self.hover = None
        self.update()

    def _event_at(self, pos):
        best, best_d = None, 8.0
        for e, _active in self.events:
            if e.x is None:
                continue
            s = self.to_screen(e.x, e.y)
            d = math.hypot(s.x() - pos.x(), s.y() - pos.y())
            if d < best_d:
                best, best_d = e, d
        return best

    # ------------------------------------------------------------ paint
    def paintEvent(self, _):
        p = QPainter(self)
        p.fillRect(self.rect(), style.FAINT)
        if self.center is None:
            p.setPen(style.GRAY)
            p.drawText(self.rect(), Qt.AlignCenter, 'No map or pose data yet')
            return
        p.save()
        c = self.center
        p.translate(self.width() / 2, self.height() / 2)
        p.scale(self.scale, -self.scale)
        p.translate(-c.x(), -c.y())
        if self.image is not None:
            p.setRenderHint(QPainter.SmoothPixmapTransform, False)
            p.drawImage(self.image_rect, self.image)
        p.restore()
        p.setRenderHint(QPainter.Antialiasing)

        def poly(xs, ys):
            return QPolygonF([self.to_screen(x, y) for x, y in zip(xs, ys)])

        if self.scan_pts is not None and len(self.scan_pts[0]):
            p.setPen(pen(style.GRAY, 2))
            p.drawPoints(poly(*self.scan_pts))
        if self.trail is not None and len(self.trail[0]) > 1:
            p.setPen(pen(style.BLACK, 1.5))
            p.drawPolyline(poly(*self.trail))
        if self.plan is not None and len(self.plan[0]) > 1:
            p.setPen(pen(style.BLUE, 2))
            p.drawPolyline(poly(*self.plan))
        if self.goal is not None:
            g = self.to_screen(*self.goal)
            p.setPen(pen(style.BLUE, 2.5))
            p.drawLine(QPointF(g.x() - 6, g.y() - 6), QPointF(g.x() + 6, g.y() + 6))
            p.drawLine(QPointF(g.x() - 6, g.y() + 6), QPointF(g.x() + 6, g.y() - 6))
        if self.frontiers:
            # (x, y, kind, xs, ys): cells in the state's colour, then a mark at
            # the centre; maze exits get a labelled green ring
            for _, _, kind, xs, ys in self.frontiers:
                if xs:
                    p.setPen(pen(style.alpha(style.FRONTIER[kind], 140), 3))
                    p.drawPoints(poly(xs, ys))
            fm = QFontMetrics(self.font())
            for x, y, kind, _, _ in self.frontiers:
                s = self.to_screen(x, y)
                color = style.FRONTIER[kind]
                if kind in ('exit', 'candidate'):
                    p.setBrush(Qt.NoBrush)
                    p.setPen(pen(color, 2.5, Qt.SolidLine if kind == 'exit' else Qt.DashLine))
                    p.drawEllipse(s, 9, 9)
                    p.setPen(pen(color))
                    p.drawText(QPointF(s.x() + 11, s.y() + fm.ascent() / 2 - 1),
                               'exit' if kind == 'exit' else 'exit?')
                else:
                    p.setPen(Qt.NoPen)
                    p.setBrush(color)
                    p.drawEllipse(s, 4, 4)
        for e, active in self.events:
            if e.x is None:
                continue
            s = self.to_screen(e.x, e.y)
            col = style.SEVERITY[e.severity]
            p.setBrush(style.alpha(col, 200))
            p.setPen(pen(style.WHITE, 1))
            p.drawEllipse(s, 4, 4)
            if active:
                p.setBrush(Qt.NoBrush)
                p.setPen(pen(col, 2))
                p.drawEllipse(s, 9, 9)
        if self.pose is not None:
            s = self.to_screen(self.pose.x, self.pose.y)
            rr = max(self.radius * self.scale, 5)
            p.setPen(pen(style.BLACK, 2))
            p.setBrush(style.alpha(style.WHITE, 180))
            p.drawEllipse(s, rr, rr)
            tip = self.to_screen(self.pose.x + math.cos(self.pose.yaw) * max(self.radius, 5 / self.scale) * 1.6,
                                 self.pose.y + math.sin(self.pose.yaw) * max(self.radius, 5 / self.scale) * 1.6)
            p.drawLine(s, tip)

        # Scale bar and cursor coordinates.
        fm = QFontMetrics(self.font())
        target = 80 / self.scale
        nice = min((0.05, 0.1, 0.25, 0.5, 1, 2, 5, 10), key=lambda v: abs(v - target))
        x0, y0 = 10, self.height() - 10
        p.setPen(pen(style.BLACK, 2))
        p.drawLine(QPointF(x0, y0), QPointF(x0 + nice * self.scale, y0))
        p.drawText(QPointF(x0, y0 - 4), f'{nice:g} m')
        if self.hover is not None:
            text = f'x {self.hover[0]:.2f}  y {self.hover[1]:.2f}'
            p.setPen(style.GRAY)
            p.drawText(QPointF(self.width() - fm.horizontalAdvance(text) - 8, self.height() - 8), text)


# ======================================================================== speed bars
class SpeedBars(QWidget):
    """Commanded (blue outline) versus actual (black fill) speeds on a centered scale."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.vals = (None, None, None, None)
        self.max_v, self.max_w = 0.3, 1.5
        self.units = Units(True)
        self.setMinimumHeight(86)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.setToolTip('Blue outline: commanded. Black fill: measured by odometry.\n'
                        'Right of center: forward or left turn.')

    def set_values(self, cmd_v, act_v, cmd_w, act_w, units):
        self.vals = (cmd_v, act_v, cmd_w, act_w)
        self.units = units
        for v in (cmd_v, act_v):
            if v is not None:
                self.max_v = max(self.max_v, abs(v))
        for v in (cmd_w, act_w):
            if v is not None:
                self.max_w = max(self.max_w, abs(v))
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        fm = QFontMetrics(self.font())
        label_w = fm.horizontalAdvance('Linear  ') + 4
        value_w = fm.horizontalAdvance('cmd -000°/s  act -000°/s') + 8
        cmd_v, act_v, cmd_w, act_w = self.vals
        rows = (('Linear', cmd_v, act_v, self.max_v, lambda v: f'{v:+.2f}'),
                ('Turn', cmd_w, act_w, self.max_w,
                 (lambda v: f'{math.degrees(v):+.0f}°/s') if self.units.degrees else (lambda v: f'{v:+.2f}')))
        row_h = (self.height() - 4) / 2
        x0, x1 = label_w, self.width() - value_w
        cx = (x0 + x1) / 2
        half = (x1 - x0) / 2
        for i, (name, cmd, act, vmax, fmt) in enumerate(rows):
            y = 2 + i * row_h
            p.setPen(style.BLACK)
            p.drawText(QRectF(0, y, label_w, row_h), Qt.AlignVCenter | Qt.AlignLeft, name)
            p.fillRect(QRectF(x0, y + row_h / 2 - 1, x1 - x0, 2), style.LIGHT)
            p.setPen(pen(style.GRAY, 1))
            p.drawLine(QPointF(cx, y + 4), QPointF(cx, y + row_h - 4))
            bh = row_h * 0.5
            if cmd is not None:
                wv = half * max(min(cmd / vmax, 1), -1)
                p.setPen(pen(style.BLUE, 2))
                p.setBrush(Qt.NoBrush)
                p.drawRect(QRectF(min(cx, cx + wv), y + (row_h - bh) / 2, abs(wv), bh))
            if act is not None:
                wv = half * max(min(act / vmax, 1), -1)
                p.fillRect(QRectF(min(cx, cx + wv), y + row_h / 2 - bh / 4, abs(wv), bh / 2), style.BLACK)
            unit = '' if (i == 1 and self.units.degrees) else (' m/s' if i == 0 else ' rad/s')
            p.setFont(style.mono_font())
            p.setPen(style.BLUE)
            ctext = 'cmd ' + (fmt(cmd) if cmd is not None else '—')
            p.drawText(QRectF(x1 + 6, y, value_w, row_h / 2 + 2), Qt.AlignBottom | Qt.AlignLeft, ctext + unit)
            p.setPen(style.BLACK)
            atext = 'act ' + (fmt(act) if act is not None else '—')
            p.drawText(QRectF(x1 + 6, y + row_h / 2, value_w, row_h / 2), Qt.AlignTop | Qt.AlignLeft, atext + unit)
            p.setFont(self.font())


# ======================================================================== time plot
class TimePlot(QWidget):
    """Stacked lanes, one per series, over a time window, with a cursor line."""

    seek_requested = Signal(float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.lanes = []
        self.t0, self.t1, self.cursor = 0.0, 1.0, 0.0
        self.bands = []
        self.setMinimumHeight(160)
        self.setMouseTracking(True)
        self.hover_t = None

    def set_data(self, lanes, t0, t1, cursor, bands):
        self.lanes, self.t0, self.t1, self.cursor, self.bands = lanes, t0, max(t1, t0 + 1e-3), cursor, bands
        self.update()

    def _x(self, t, left, width):
        return left + (t - self.t0) / (self.t1 - self.t0) * width

    def mousePressEvent(self, ev):
        left, width = self._plot_area()
        if width > 0 and ev.button() == Qt.LeftButton:
            t = self.t0 + (ev.pos().x() - left) / width * (self.t1 - self.t0)
            self.seek_requested.emit(min(max(t, self.t0), self.t1))

    def mouseMoveEvent(self, ev):
        left, width = self._plot_area()
        self.hover_t = self.t0 + (ev.pos().x() - left) / max(width, 1) * (self.t1 - self.t0)
        self.update()

    def leaveEvent(self, ev):
        self.hover_t = None
        self.update()

    def _plot_area(self):
        fm = QFontMetrics(style.mono_font())
        left = fm.horizontalAdvance('-0000.00') + 8
        return left, self.width() - left - 8

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        fm = QFontMetrics(self.font())
        p.setFont(style.mono_font())
        mfm = QFontMetrics(p.font())
        left, width = self._plot_area()
        axis_h = mfm.height() + 4
        if not self.lanes:
            p.setFont(self.font())
            p.setPen(style.GRAY)
            p.drawText(self.rect(), Qt.AlignCenter, 'Select values to plot')
            return
        lane_h = (self.height() - axis_h) / len(self.lanes)
        for ts, te, color in self.bands:
            x0 = max(self._x(ts, left, width), left)
            x1 = min(self._x(te, left, width), left + width)
            if x1 > x0:
                p.fillRect(QRectF(x0, 0, max(x1 - x0, 1), self.height() - axis_h), style.alpha(color, 40))
        for i, lane in enumerate(self.lanes):
            label, unit, traces, scale = lane
            top = i * lane_h
            plot_top, plot_h = top + fm.height() + 2, lane_h - fm.height() - 8
            p.setPen(pen(style.LIGHT, 1))
            p.drawLine(QPointF(left, top + lane_h - 2), QPointF(left + width, top + lane_h - 2))
            p.setFont(self.font())
            p.setPen(style.BLACK)
            p.drawText(QPointF(left + 4, top + fm.ascent() + 1), f'{label} ({unit})' if unit else label)
            p.setFont(style.mono_font())
            vals = [v * scale for _, values, _ in traces for v in values
                    if v is not None and math.isfinite(v)]
            if not vals or plot_h < 8:
                continue
            lo, hi = min(vals), max(vals)
            if 0 < lo < 0.3 * hi:
                lo = 0.0
            if hi - lo < 1e-6:
                hi, lo = hi + 0.5 * max(abs(hi), 0.1), lo - 0.5 * max(abs(lo), 0.1)
            pad = 0.08 * (hi - lo)
            lo, hi = lo - pad, hi + pad

            def y(v, lo=lo, hi=hi, plot_top=plot_top, plot_h=plot_h):
                return plot_top + (hi - v) / (hi - lo) * plot_h

            p.setPen(style.GRAY)
            p.drawText(QRectF(0, plot_top - mfm.height() / 2, left - 4, mfm.height()),
                       Qt.AlignRight | Qt.AlignVCenter, f'{hi:.2f}')
            p.drawText(QRectF(0, plot_top + plot_h - mfm.height() / 2, left - 4, mfm.height()),
                       Qt.AlignRight | Qt.AlignVCenter, f'{lo:.2f}')
            if lo < 0 < hi:
                p.setPen(pen(style.LIGHT, 1, Qt.DashLine))
                p.drawLine(QPointF(left, y(0)), QPointF(left + width, y(0)))
            for times, values, color in traces:
                path = QPainterPath()
                started = False
                for t, v in zip(times, values):
                    if v is None or not math.isfinite(v):
                        started = False
                        continue
                    pt = QPointF(min(max(self._x(t, left, width), left), left + width), y(v * scale))
                    if started:
                        path.lineTo(pt)
                    else:
                        path.moveTo(pt)
                        started = True
                p.setPen(pen(color, 1.5))
                p.setBrush(Qt.NoBrush)
                p.drawPath(path)
        p.setPen(style.GRAY)
        for frac in (0.0, 0.5, 1.0):
            t = self.t0 + frac * (self.t1 - self.t0)
            text = style.fmt_time(t)
            tw = mfm.horizontalAdvance(text)
            x = left + frac * width - (tw if frac == 1 else tw / 2 if frac == 0.5 else 0)
            p.drawText(QPointF(x, self.height() - 3), text)
        xc = self._x(self.cursor, left, width)
        p.setPen(pen(style.BLACK, 1.5))
        p.drawLine(QPointF(xc, 0), QPointF(xc, self.height() - axis_h))
        if self.hover_t is not None and self.t0 <= self.hover_t <= self.t1:
            xh = self._x(self.hover_t, left, width)
            p.setPen(pen(style.GRAY, 1, Qt.DotLine))
            p.drawLine(QPointF(xh, 0), QPointF(xh, self.height() - axis_h))
            p.drawText(QPointF(min(xh + 4, self.width() - 60), self.height() - axis_h - 4),
                       style.fmt_time(self.hover_t))


class StatusBanner(QLabel):
    """The headline of the Status card: a colored bar with the overall level."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFont(style.bold())
        self.setMargin(6)
        self.setWordWrap(True)
        self.set_level('ok', 'No data yet')

    def set_level(self, level, text):
        col = style.LEVEL.get(level, style.GRAY)
        self.setStyleSheet(f'background: {col.name()}; color: white; border-radius: 2px;')
        self.setText(text)


class Placeholder(QLabel):
    def __init__(self, text, parent=None):
        super().__init__(text, parent)
        self.setAlignment(Qt.AlignCenter)
        self.setStyleSheet(f'color: {style.GRAY.name()};')


__all__ = ['Card', 'KeyValueGrid', 'HeadingDial', 'SurroundingsView', 'MapView', 'SpeedBars',
           'TimePlot', 'StatusBanner', 'Placeholder', 'clearance_color', 'pen', 'QBrush']
