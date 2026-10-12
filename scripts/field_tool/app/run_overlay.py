import math

import numpy as np
from PySide6.QtCore import QElapsedTimer, QObject, QPointF, QRectF, Qt, QTimer, Signal
from PySide6.QtGui import QColor, QFont, QFontMetricsF, QPen, QPolygonF
from PySide6.QtWidgets import QGraphicsItem

from app import field_config as cfg
from app.field_view import field_to_scene

SPEEDS = [0.1, 0.25, 0.5, 1.0, 2.0, 4.0]
GHOST_MAX_GAP_S = 0.25  #hide a trace's robot outline if it has no point this close to the playhead (outside its data)


class TraceItem(QGraphicsItem):
    """One whole trace (thousands of points) drawn as a single item - way faster than an item per point."""

    def __init__(self, trace, draw=True):
        super().__init__()
        self.trace = trace
        self.draw = draw  #False = only there to be clicked/played (the planner draws the plan's path itself)
        self._points = QPolygonF([field_to_scene(x, y) for x, y in zip(trace.x, trace.y)])
        self._rect = self._points.boundingRect().adjusted(-3, -3, 3, 3)
        color = QColor(trace.color)
        line_color = QColor(color)
        line_color.setAlpha(110)
        self._line_pen = QPen(line_color, 1.5)
        self._line_pen.setCosmetic(True)
        self._dot_pen = QPen(color, 4, Qt.SolidLine, Qt.RoundCap)  #round cap makes drawPoints draw dots
        self._dot_pen.setCosmetic(True)  #dots stay 4px no matter the zoom
        self.setZValue(5 if trace.key == "robot" else 4)

    def boundingRect(self):
        return self._rect

    def paint(self, painter, option, widget=None):
        if not self.draw:
            return
        painter.setPen(self._line_pen)
        painter.drawPolyline(self._points)
        painter.setPen(self._dot_pen)
        painter.drawPoints(self._points)


class RobotGhost(QGraphicsItem):
    """Real-size robot outline at a pose (scales with zoom).
    Local coords: origin = tracking center, +x = robot forward, -y = robot left (Qt's y points down)."""

    def __init__(self, color):
        super().__init__()
        self._color = QColor(color)
        self._body = QRectF()
        self.setZValue(7)
        self.setAcceptedMouseButtons(Qt.NoButton)  #clicks go through to the points underneath

    def set_profile(self, profile):
        self.prepareGeometryChange()
        self._body = QRectF(profile.center_forward_in - profile.length_in / 2,
                            -profile.center_left_in - profile.width_in / 2,
                            profile.length_in, profile.width_in)

    def set_pose(self, x, y, heading_deg):
        self.setPos(field_to_scene(x, y))
        self.setRotation(-heading_deg)  #field angles are CCW, Qt rotation is CW on screen

    def boundingRect(self):
        return self._body.united(QRectF(-1.5, -1.5, 3, 3)).adjusted(-1, -1, 1, 1)

    def paint(self, painter, option, widget=None):
        fill = QColor(self._color)
        fill.setAlpha(45)
        pen = QPen(self._color, 1.5)
        pen.setCosmetic(True)
        painter.setPen(pen)
        painter.setBrush(fill)
        painter.drawRect(self._body)

        front = QPen(QColor("white"), 3.5)  #front edge, so heading reads at a glance
        front.setCosmetic(True)
        painter.setPen(front)
        painter.drawLine(self._body.topRight(), self._body.bottomRight())

        painter.setPen(pen)  #tracking center
        painter.drawLine(QPointF(-1.5, 0), QPointF(1.5, 0))
        painter.drawLine(QPointF(0, -1.5), QPointF(0, 1.5))


class Callout(QGraphicsItem):
    """Ring + heading arrow on the selected point, with a data box connected to it by a leader line.
    Ignores the view's zoom, so everything in here is in screen pixels."""

    PAD = 6
    GAP = 22  #screen px from the point to the box corner

    def __init__(self):
        super().__init__()
        self.setFlag(QGraphicsItem.ItemIgnoresTransformations)
        self.setZValue(10)
        self._font = QFont("Consolas", 9)
        self._title = ""
        self._lines = []
        self._color = QColor("white")
        self._heading = 0.0
        self._flip_x = False
        self._flip_y = False
        self._box = QRectF()
        self._layout()

    def set_content(self, title, lines, color, heading_deg):
        self.prepareGeometryChange()
        self._title, self._lines, self._color, self._heading = title, lines, QColor(color), heading_deg
        self._layout()
        self.update()

    def set_flip(self, flip_x, flip_y):
        if (flip_x, flip_y) != (self._flip_x, self._flip_y):
            self.prepareGeometryChange()
            self._flip_x, self._flip_y = flip_x, flip_y
            self._layout()
            self.update()

    def box_size(self):
        return self._box.width(), self._box.height()

    def _layout(self):
        fm = QFontMetricsF(self._font)
        rows = [self._title] + self._lines
        w = max(fm.horizontalAdvance(s) for s in rows) + 2 * self.PAD
        h = fm.height() * len(rows) + 2 * self.PAD
        x = -self.GAP - w if self._flip_x else self.GAP
        y = self.GAP if self._flip_y else -self.GAP - h
        self._box = QRectF(x, y, w, h)

    def boundingRect(self):
        return self._box.united(QRectF(-32, -32, 64, 64)).adjusted(-2, -2, 2, 2)

    def paint(self, painter, option, widget=None):
        pen = QPen(self._color, 2)
        painter.setPen(pen)
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(QPointF(0, 0), 7, 7)

        #heading arrow (screen y is down, so the field's CCW angle flips sign)
        rad = math.radians(self._heading)
        dx, dy = math.cos(rad), -math.sin(rad)
        tip = QPointF(dx * 28, dy * 28)
        painter.drawLine(QPointF(dx * 7, dy * 7), tip)
        for side in (1, -1):
            painter.drawLine(tip, QPointF(tip.x() - dx * 7 + dy * 4 * side, tip.y() - dy * 7 - dx * 4 * side))

        #leader line from the ring to the box's nearest corner
        corner = QPointF(self._box.right() if self._flip_x else self._box.left(),
                         self._box.top() if self._flip_y else self._box.bottom())
        length = math.hypot(corner.x(), corner.y())
        painter.setPen(QPen(self._color, 1))
        painter.drawLine(QPointF(corner.x() / length * 7, corner.y() / length * 7), corner)

        painter.setPen(QPen(self._color, 1.5))
        painter.setBrush(QColor(18, 18, 18, 230))
        painter.drawRoundedRect(self._box, 4, 4)

        painter.setFont(self._font)
        fm = QFontMetricsF(self._font)
        x = self._box.left() + self.PAD
        y = self._box.top() + self.PAD + fm.ascent()
        painter.setPen(self._color)
        painter.drawText(QPointF(x, y), self._title)
        painter.setPen(QColor("#e8e8e8"))
        for line in self._lines:
            y += fm.height()
            painter.drawText(QPointF(x, y), line)


def _fmt(value, spec, unit):
    return "—" if value is None or not np.isfinite(value) else f"{value:{spec}} {unit}"


def nearest_index(tr, t):
    i = int(np.searchsorted(tr.t, t))
    if i <= 0:
        return 0
    if i >= len(tr):
        return len(tr) - 1
    return i if tr.t[i] - t < t - tr.t[i - 1] else i - 1


class RunOverlay(QObject):
    """Draws a loaded run on the field, handles picking points and playback.

    Everything hangs off one playhead time: clicking/stepping a point moves it to that point's time,
    playing moves it in real time. Each trace's robot outline sits at that trace's point nearest the
    playhead, so odom and camera are always shown at the same instant."""

    changed = Signal()  #playhead / selection / playback state changed

    def __init__(self, view, profile):
        super().__init__()
        self.view = view
        self.run = None
        self.items = {}   #trace key -> TraceItem
        self.ghosts = {}  #trace key -> RobotGhost
        self.profile = profile
        self.extra = {}           #key -> Trace that isn't part of the run (the auton plan), kept across run loads
        self.visible = {"robot": True, "camera": True}
        self.show_ghosts = True
        self.hidden_ghosts = set()  #trace keys whose outline is hidden for now (the plan's while dragging a step)
        self.motion_mode = False  #False = position data, True = motion data
        self.show_frame = False   #False = time, True = video frame number
        self.selected = None      #(trace key, point index) the data box is attached to
        self.time = None          #playhead, seconds on the run's clock

        self.playing = False
        self.speed = 1.0
        self._timer = QTimer(self)
        self._timer.setInterval(16)  #~60 fps redraw
        self._timer.timeout.connect(self._tick)
        self._clock = QElapsedTimer()
        self._play_origin = 0.0  #playhead time when the clock was last (re)started

        self.callout = Callout()
        self.callout.hide()
        view.scene().addItem(self.callout)
        view.view_changed.connect(self._place_callout)  #field clicks come in through click() from the main window

    # ---- run / display settings

    def set_run(self, run):
        self.pause()
        for item in list(self.items.values()) + list(self.ghosts.values()):
            self.view.scene().removeItem(item)
        self.items, self.ghosts = {}, {}
        self.run = run
        if run is not None:
            for tr in run.traces():
                self._add_trace(tr, draw=True)
        for tr in self.extra.values():
            self._add_trace(tr, draw=False)
        self.selected, self.time = None, None
        self._update()

    def _add_trace(self, tr, draw):
        item = TraceItem(tr, draw)
        item.setVisible(self.visible.get(tr.key, True))
        self.view.scene().addItem(item)
        self.items[tr.key] = item
        ghost = RobotGhost(tr.color)
        ghost.set_profile(self.profile)
        ghost.hide()
        self.view.scene().addItem(ghost)
        self.ghosts[tr.key] = ghost

    def set_extra_trace(self, key, tr):
        """Add / replace / (tr=None) remove a non-run trace without losing the playhead or playback."""
        if key in self.items:
            self.view.scene().removeItem(self.items.pop(key))
            self.view.scene().removeItem(self.ghosts.pop(key))
        self.extra.pop(key, None)
        if tr is not None:
            self.extra[key] = tr
            self._add_trace(tr, draw=False)
        if self.selected is not None and self.selected[0] == key:
            if tr is None:
                self.selected = None
            else:  #same instant on the new version of the trace
                self.selected = (key, nearest_index(tr, self.time) if self.time is not None else 0)
        rng = self.time_range()
        if rng is None:
            self.pause()
            self.time = None
        self._update()

    def _trace(self, key):
        return self.items[key].trace if key in self.items else None

    def _visible_traces(self):
        return [item.trace for item in self.items.values() if item.isVisible()]

    def time_range(self):
        traces = self._visible_traces()
        if not traces:
            return None
        return min(tr.t[0] for tr in traces), max(tr.t[-1] for tr in traces)

    def set_visible(self, key, visible):
        self.visible[key] = visible
        if key in self.items:
            self.items[key].setVisible(visible)
        if not visible and self.selected and self.selected[0] == key:
            self.selected = None
        self._update()

    def set_ghosts_visible(self, on):
        self.show_ghosts = on
        self._update()

    def set_profile(self, profile):
        self.profile = profile
        for ghost in self.ghosts.values():
            ghost.set_profile(profile)
        self._update()

    def set_motion_mode(self, on):
        self.motion_mode = on
        self._update()

    def set_show_frame(self, on):
        self.show_frame = on
        self._update()

    # ---- selection

    def select(self, key, index):
        tr = self._trace(key)
        index = min(max(int(index), 0), len(tr) - 1)
        self.selected = (key, index)
        self._set_time(float(tr.t[index]))

    def click(self, x, y, in_per_px):
        best = None
        for tr in self._visible_traces():
            d = np.hypot(tr.x - x, tr.y - y)
            i = int(np.argmin(d))
            if best is None or d[i] < best[0]:
                best = (d[i], tr.key, i)
        if best and best[0] <= cfg.CLICK_TOLERANCE_PX * in_per_px:
            self.pause()
            self.select(best[1], best[2])
        else:
            self.selected = None  #clicked empty field: drop the data box, keep the playhead
            self._update()

    def step(self, n):
        self.pause()
        if self.selected is not None:
            key, i = self.selected
            self.select(key, i + n)
            return
        traces = self._visible_traces()
        if traces:  #nothing selected yet: start on odom (else whatever's shown) at the playhead, or its first point
            tr = next((t for t in traces if t.key == "robot"), traces[0])
            self.select(tr.key, nearest_index(tr, self.time) if self.time is not None else 0)

    def jump_to(self, key):
        """Select `key`'s point at the current instant - quick camera vs odom comparison."""
        tr = self._trace(key)
        if tr is None or not self.items[key].isVisible():
            return
        self.pause()
        self.select(key, nearest_index(tr, self.time) if self.time is not None else 0)

    def clear(self):
        self.pause()
        self.selected, self.time = None, None
        self._update()

    # ---- playback

    def toggle_play(self):
        if self.playing:
            self.pause()
        else:
            self.play()

    def play(self):
        rng = self.time_range()
        if rng is None:
            return
        if self.time is None or self.time >= rng[1]:
            self.time = rng[0]  #start over from the beginning
        self.playing = True
        self._restart_clock()
        self._timer.start()
        self._update()

    def pause(self):
        if self.playing:
            self.playing = False
            self._timer.stop()
            self.changed.emit()

    def seek(self, t):
        rng = self.time_range()
        if rng is None:
            return
        self._set_time(min(max(t, rng[0]), rng[1]))
        if self.playing:
            self._restart_clock()

    def go_start(self):
        rng = self.time_range()
        if rng:
            self.seek(rng[0])

    def go_end(self):
        rng = self.time_range()
        if rng:
            self.pause()
            self.seek(rng[1])

    def change_speed(self, direction):
        i = SPEEDS.index(self.speed) + direction
        self.speed = SPEEDS[min(max(i, 0), len(SPEEDS) - 1)]
        if self.playing:
            self._restart_clock()  #so the playhead doesn't jump when the rate changes
        self.changed.emit()

    def _restart_clock(self):
        self._play_origin = self.time
        self._clock.start()

    def _tick(self):
        rng = self.time_range()
        if rng is None:
            self.pause()
            return
        t = self._play_origin + self._clock.elapsed() / 1000.0 * self.speed
        if t >= rng[1]:
            t = rng[1]
            self.playing = False
            self._timer.stop()
        self._set_time(t)

    def _set_time(self, t):
        self.time = t
        if self.selected is not None:  #the data box follows its trace through time
            key, _ = self.selected
            self.selected = (key, nearest_index(self._trace(key), t))
        self._update()

    # ---- drawing

    def _update(self):
        for key, ghost in self.ghosts.items():
            tr = self._trace(key)
            show = (self.show_ghosts and self.time is not None and self.items[key].isVisible()
                    and key not in self.hidden_ghosts)
            if show:
                i = nearest_index(tr, self.time)
                show = bool(abs(tr.t[i] - self.time) <= GHOST_MAX_GAP_S)
                if show:
                    ghost.set_pose(tr.x[i], tr.y[i], tr.heading[i])
            ghost.setVisible(show)
        self._refresh_callout()
        self.changed.emit()

    def _time_line(self, tr, i):
        if not self.show_frame:
            return f"t     = {tr.t[i]:.3f} s"
        f = tr.frame[i]
        if not np.isfinite(f):
            return f"frame = — ({tr.no_frame_reason})"
        return f"frame = {int(f)}" if tr.frame_exact else f"frame ≈ {int(round(f))}"

    def _refresh_callout(self):
        if self.selected is None:
            self.callout.hide()
            return
        key, i = self.selected
        tr = self._trace(key)
        title = f"{tr.name}  #{i + 1}/{len(tr)}"
        lines = [self._time_line(tr, i)]
        if self.motion_mode:
            lines += [f"v     = {_fmt(tr.v[i], '+.1f', 'in/s')}",
                      f"a     = {_fmt(tr.a[i], '+.0f', 'in/s²')}",
                      f"ω     = {_fmt(tr.omega[i], '+.1f', '°/s')}"]
        else:
            lines += [f"x     = {tr.x[i]:+.2f} in",
                      f"y     = {tr.y[i]:+.2f} in",
                      f"θ     = {tr.heading[i]:.1f}°"]
        self.callout.setPos(field_to_scene(tr.x[i], tr.y[i]))
        self.callout.set_content(title, lines, tr.color, tr.heading[i])
        self.callout.show()
        self._place_callout()

    def _place_callout(self):
        """Flip the box to whichever side of the point has room on screen."""
        if not self.callout.isVisible():
            return
        p = self.view.mapFromScene(self.callout.pos())
        w, h = self.callout.box_size()
        flip_x = p.x() + Callout.GAP + w + 8 > self.view.viewport().width()
        flip_y = p.y() - Callout.GAP - h - 8 < 0
        self.callout.set_flip(flip_x, flip_y)
