"""Draws the open plan on the field: predicted path, step numbers, robot outlines, and draggable handles."""
import math

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QFont, QFontMetricsF, QPen, QPolygonF
from PySide6.QtWidgets import QGraphicsItem

from app import field_config as cfg
from app.field_view import INTERACTIVE, field_to_scene, scene_to_field
from app.run_overlay import RobotGhost

ANGLE_HANDLE_IN = 14.0   #how far out from the robot the heading / angle handles sit


class PlanPathItem(QGraphicsItem):
    """Every step's predicted path in one item. Selected step = white, async = dashed."""

    def __init__(self):
        super().__init__()
        self._paths = []  #(QPolygonF, selected, async)
        self._ends = QPolygonF()
        self._rect = QRectF()
        self.setZValue(6)
        self.setAcceptedMouseButtons(Qt.NoButton)

    def set_data(self, sim, selected):
        self.prepareGeometryChange()
        self._paths, ends = [], []
        for s in sim.steps:
            if not s.step.kind.drivetrain:
                continue
            poly = QPolygonF([field_to_scene(x, y) for _, x, y, _ in s.samples])
            is_sel = selected is not None and s.path[:len(selected)] == selected
            self._paths.append((poly, is_sel, s.is_async))
            ends.append(field_to_scene(s.end[0], s.end[1]))
        self._ends = QPolygonF(ends)
        rect = self._ends.boundingRect()
        for poly, _, _ in self._paths:
            rect = rect.united(poly.boundingRect())
        self._rect = rect.adjusted(-3, -3, 3, 3)
        self.update()

    def boundingRect(self):
        return self._rect

    def paint(self, painter, option, widget=None):
        color = QColor(cfg.PLAN_COLOR)
        for poly, is_sel, is_async in self._paths:
            pen = QPen(QColor("white") if is_sel else color, 3 if is_sel else 2)
            pen.setCosmetic(True)
            if is_async:
                pen.setStyle(Qt.DashLine)
            painter.setPen(pen)
            painter.drawPolyline(poly)
        dot = QPen(color, 7, Qt.SolidLine, Qt.RoundCap)
        dot.setCosmetic(True)
        painter.setPen(dot)
        painter.drawPoints(self._ends)


class Badge(QGraphicsItem):
    """Step number next to where the step ends. Constant screen size."""

    def __init__(self):
        super().__init__()
        self.setFlag(QGraphicsItem.ItemIgnoresTransformations)
        self.setZValue(8)
        self.setAcceptedMouseButtons(Qt.NoButton)
        self._font = QFont("Segoe UI", 8)
        self._font.setBold(True)
        self._text = ""
        self._selected = False
        self._rect = QRectF()

    def set_text(self, text, selected):
        self.prepareGeometryChange()
        self._text, self._selected = text, selected
        w = QFontMetricsF(self._font).horizontalAdvance(text) + 8
        self._rect = QRectF(6, -18, max(w, 16), 14)
        self.update()

    def boundingRect(self):
        return self._rect.adjusted(-1, -1, 1, 1)

    def paint(self, painter, option, widget=None):
        painter.setPen(QPen(QColor("white") if self._selected else QColor(cfg.PLAN_COLOR), 1))
        painter.setBrush(QColor(18, 18, 18, 220))
        painter.drawRoundedRect(self._rect, 4, 4)
        painter.setFont(self._font)
        painter.drawText(self._rect, Qt.AlignCenter, self._text)


class Handle(QGraphicsItem):
    """Draggable dot (constant screen size). key says what dragging it changes, see Planner.drag()."""

    R = 6

    def __init__(self, overlay, key, marker):
        super().__init__()
        self.overlay, self.key, self.marker = overlay, key, marker  #marker: circle | diamond | square
        self.setFlag(QGraphicsItem.ItemIgnoresTransformations)
        self.setData(0, INTERACTIVE)
        self.setZValue(9)
        self.setAcceptedMouseButtons(Qt.LeftButton)
        self.setCursor(Qt.SizeAllCursor)
        self.color = QColor(cfg.PLAN_COLOR)
        self.selected = False

    def boundingRect(self):
        return QRectF(-self.R - 2, -self.R - 2, 2 * self.R + 4, 2 * self.R + 4)

    def paint(self, painter, option, widget=None):
        painter.setPen(QPen(QColor("white") if self.selected else QColor(20, 20, 20), 1.5))
        painter.setBrush(self.color)
        r = self.R if self.selected else self.R - 1
        if self.marker == "diamond":
            painter.drawPolygon(QPolygonF([QPointF(0, -r - 1), QPointF(r + 1, 0), QPointF(0, r + 1), QPointF(-r - 1, 0)]))
        elif self.marker == "square":
            painter.drawRect(QRectF(-r, -r, 2 * r, 2 * r))
        else:
            painter.drawEllipse(QPointF(0, 0), r, r)

    def mousePressEvent(self, event):
        event.accept()
        self.overlay.planner.begin_drag(self.key)

    def mouseMoveEvent(self, event):
        x, y = scene_to_field(event.scenePos())
        self.overlay.planner.drag(self.key, x, y, bool(event.modifiers() & Qt.ShiftModifier))

    def mouseReleaseEvent(self, event):
        self.overlay.planner.end_drag()


class PlanOverlay:
    def __init__(self, view, planner, profile):
        self.view, self.planner = view, planner
        scene = view.scene()
        self.path_item = PlanPathItem()
        scene.addItem(self.path_item)
        self.start_ghost = RobotGhost(cfg.PLAN_START_COLOR)
        self.end_ghost = RobotGhost(cfg.PLAN_COLOR)
        for g in (self.start_ghost, self.end_ghost):
            g.setZValue(7)
            scene.addItem(g)
        self.set_profile(profile)
        self.badges = []
        self.handles = {}  #key -> Handle; reused across redraws so a handle being dragged keeps its mouse grab
        self.visible = False
        self.playhead_active = False  #while scrubbing / playing, the moving plan outline replaces the end-of-step one
        self._apply_visible()
        planner.changed.connect(self.refresh)

    def set_playhead_active(self, on):
        if on != self.playhead_active:
            self.playhead_active = on
            self.refresh()

    def set_profile(self, profile):
        self.start_ghost.set_profile(profile)
        self.end_ghost.set_profile(profile)

    def set_visible(self, on):
        self.visible = on
        self._apply_visible()
        self.refresh()

    def _apply_visible(self):
        for item in [self.path_item, self.start_ghost, self.end_ghost] + self.badges + list(self.handles.values()):
            item.setVisible(self.visible)

    def _handle_specs(self):
        """(key, field x, y, marker, color) for every handle that should exist right now."""
        plan, sim = self.planner.plan, self.planner.sim
        sel = self.planner.selected
        specs = []
        h = math.radians(plan.start_heading)
        specs.append((("start",), plan.start_x, plan.start_y, "square", cfg.PLAN_START_COLOR))
        specs.append((("start_heading",), plan.start_x + ANGLE_HANDLE_IN * math.cos(h),
                      plan.start_y + ANGLE_HANDLE_IN * math.sin(h), "diamond", cfg.PLAN_START_COLOR))
        for s in sim.steps:
            kind = s.step.kind
            if kind.handle == "point":  #every waypoint is always draggable
                specs.append((("point", s.path), s.step.params["x"], s.step.params["y"], "circle", cfg.PLAN_COLOR))
            elif s.path == sel and kind.handle == "distance":
                specs.append((("distance", s.path), s.end[0], s.end[1], "circle", cfg.PLAN_COLOR))
            elif s.path == sel and kind.handle == "angle":
                a = math.radians(s.step.params["target"])
                specs.append((("angle", s.path), s.start[0] + ANGLE_HANDLE_IN * math.cos(a),
                              s.start[1] + ANGLE_HANDLE_IN * math.sin(a), "diamond", cfg.PLAN_COLOR))
        return specs

    def refresh(self):
        planner = self.planner
        if not self.visible or planner.plan is None:
            return
        plan, sim, sel = planner.plan, planner.sim, planner.selected
        scene = self.view.scene()
        self.path_item.set_data(sim, sel)
        self.start_ghost.set_pose(plan.start_x, plan.start_y, plan.start_heading)

        #outline where the selected step (or sequence) leaves the robot. While dragging it replaces the playback outline
        mine = [s for s in sim.steps if sel is not None and s.path[:len(sel)] == sel]
        self.end_ghost.setVisible(bool(mine) and (not self.playhead_active or planner.dragging))
        if mine:
            self.end_ghost.set_pose(*mine[-1].end)

        #number badges at the end of each step; steps that end in the same spot share one ("3,4")
        spots = {}
        for s in sim.steps:
            k = (round(s.end[0], 1), round(s.end[1], 1))
            spots.setdefault(k, []).append(s)
        while len(self.badges) < len(spots):
            b = Badge()
            scene.addItem(b)
            self.badges.append(b)
        for b in self.badges[len(spots):]:
            b.hide()
        for b, (k, steps) in zip(self.badges, spots.items()):
            nums = [str(s.number) for s in steps]
            text = ",".join(nums) if len(nums) <= 3 else f"{nums[0]}-{nums[-1]}"
            b.set_text(text, any(sel is not None and s.path[:len(sel)] == sel for s in steps))
            b.setPos(field_to_scene(*k))
            b.show()

        specs = self._handle_specs()
        keys = {spec[0] for spec in specs}
        for key in [k for k in self.handles if k not in keys]:
            scene.removeItem(self.handles.pop(key))
        for key, x, y, marker, color in specs:
            handle = self.handles.get(key)
            if handle is None:
                handle = Handle(self, key, marker)
                scene.addItem(handle)
                self.handles[key] = handle
            handle.color = QColor(color)
            handle.selected = len(key) > 1 and key[1] == sel
            handle.setPos(field_to_scene(x, y))
            handle.setVisible(True)
            handle.update()

    def step_at(self, x, y, tolerance):
        """Step whose end point is under a click (last one wins, so the newest of a stack is picked)."""
        best = None
        for s in self.planner.sim.steps:
            d = math.hypot(s.end[0] - x, s.end[1] - y)
            if d <= tolerance and (best is None or d <= best[0] + 1e-9):
                best = (d, s.path)
        return best[1] if best else None
