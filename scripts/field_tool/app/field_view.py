from PySide6.QtCore import Qt, QPointF, QRectF, Signal
from PySide6.QtGui import QColor, QPainter, QPen, QPixmap
from PySide6.QtWidgets import QGraphicsItemGroup, QGraphicsScene, QGraphicsView

from app import field_config as cfg


#Everything drawn on the field goes through these two, so the rest of the tool only thinks in field inches.
#Qt's +Y points down the screen, field +Y points up, so y just gets flipped.
def field_to_scene(x, y):
    return QPointF(x, -y)


def scene_to_field(point):
    return point.x(), -point.y()


#items tagged with setData(0, INTERACTIVE) (e.g. planner handles) get the left click themselves
#instead of it becoming a field_clicked
INTERACTIVE = "interactive"


class FieldView(QGraphicsView):
    cursor_moved = Signal(float, float)  #field x, y in inches
    cursor_left = Signal()
    field_clicked = Signal(float, float, float)  #field x, y, and inches per screen pixel (for click tolerance)
    view_changed = Signal()  #zoomed, panned or resized

    MAX_ZOOM = 25.0  #relative to "whole field fits in the window"
    PAN_BUTTONS = (Qt.MiddleButton, Qt.RightButton)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setScene(QGraphicsScene(self))
        self.setRenderHints(QPainter.Antialiasing | QPainter.SmoothPixmapTransform)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorViewCenter)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setBackgroundBrush(QColor("#1b1b1b"))
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.StrongFocus)

        self.field_rect = self._add_field_image()
        #a little slack past the image so you can pan an edge to the middle of the screen when zoomed in
        self.scene().setSceneRect(self.field_rect.adjusted(-20, -20, 20, 20))
        self.grid = self._make_grid()
        self.axes = self._make_axes()

        self._zoom = 1.0
        self._pan_last = None
        self.placing = False  #planner is waiting for a click: every left click is a field click, even on handles

    def set_placing(self, on):
        self.placing = on
        if on:
            self.viewport().setCursor(Qt.CrossCursor)
        else:
            self.viewport().unsetCursor()

    def _add_field_image(self):
        pixmap = QPixmap(str(cfg.FIELD_IMAGE))
        if pixmap.isNull():
            raise FileNotFoundError(f"couldn't load field image: {cfg.FIELD_IMAGE}")
        #scale the image so its tile seams land exactly on multiples of TILE_IN
        in_per_px = cfg.TILE_IN / cfg.IMAGE_PX_PER_TILE
        item = self.scene().addPixmap(pixmap)
        item.setTransformationMode(Qt.SmoothTransformation)
        item.setScale(in_per_px)
        item.setPos(-cfg.IMAGE_CENTER_PX * in_per_px, -cfg.IMAGE_CENTER_PX * in_per_px)
        item.setZValue(0)
        return item.sceneBoundingRect()

    def _make_grid(self):
        group = QGraphicsItemGroup()
        pen = QPen(QColor(0, 220, 255, 90), 1)
        pen.setCosmetic(True)  #width in screen pixels, so lines don't get fat when zoomed in
        #tile seams at 0, +-24, +-48 (the +-72 seams are under the walls)
        seams = int(cfg.WALL_IN // cfg.TILE_IN)
        for i in range(-seams, seams + 1):
            v = i * cfg.TILE_IN
            group.addToGroup(self.scene().addLine(v, -cfg.WALL_IN, v, cfg.WALL_IN, pen))
            group.addToGroup(self.scene().addLine(-cfg.WALL_IN, v, cfg.WALL_IN, v, pen))

        #real inner wall face (+-70), sits ~1.6in inside the image's drawn walls
        wall_pen = QPen(QColor(0, 220, 255, 160), 1.5)
        wall_pen.setCosmetic(True)
        group.addToGroup(self.scene().addRect(QRectF(-cfg.WALL_IN, -cfg.WALL_IN, 2 * cfg.WALL_IN, 2 * cfg.WALL_IN), wall_pen))
        group.setZValue(1)
        self.scene().addItem(group)
        return group

    def _make_axes(self, length=12.0):
        group = QGraphicsItemGroup()
        for (dx, dy), color, label in (((1, 0), QColor("#ff5555"), "+X"), ((0, 1), QColor("#55ff7f"), "+Y")):
            pen = QPen(color, 2.5)
            pen.setCosmetic(True)
            tip = field_to_scene(dx * length, dy * length)
            group.addToGroup(self.scene().addLine(0, 0, tip.x(), tip.y(), pen))
            #arrowhead: two short lines angled back from the tip
            head = 2.0
            for side in (1, -1):
                back = field_to_scene(dx * (length - head) - dy * head * side * 0.6,
                                      dy * (length - head) + dx * head * side * 0.6)
                group.addToGroup(self.scene().addLine(tip.x(), tip.y(), back.x(), back.y(), pen))
            text = self.scene().addSimpleText(label)
            text.setBrush(color)
            text.setFlag(text.GraphicsItemFlag.ItemIgnoresTransformations)  #constant text size at any zoom
            text.setPos(field_to_scene(dx * (length + 1.5) - 1.0, dy * (length + 4.0)))
            group.addToGroup(text)
        group.setZValue(2)
        self.scene().addItem(group)
        return group

    def fit_field(self):
        self.resetTransform()
        self.fitInView(self.field_rect, Qt.KeepAspectRatio)
        self._zoom = 1.0
        self.view_changed.emit()

    def set_grid_visible(self, visible):
        self.grid.setVisible(visible)

    def set_axes_visible(self, visible):
        self.axes.setVisible(visible)

    def showEvent(self, event):
        super().showEvent(event)
        self.fit_field()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self._zoom == 1.0:  #only re-fit if the user hasn't zoomed in yet
            self.fit_field()
        else:
            self.view_changed.emit()

    def wheelEvent(self, event):
        factor = 1.0015 ** event.angleDelta().y()
        new_zoom = min(max(self._zoom * factor, 1.0), self.MAX_ZOOM)
        if new_zoom == 1.0:
            self.fit_field()  #snap back exactly, so zooming all the way out always re-centers
            return
        self.scale(new_zoom / self._zoom, new_zoom / self._zoom)
        self._zoom = new_zoom
        self.view_changed.emit()

    #Pan: drag with middle/right mouse, or Ctrl + left drag (for trackpads).
    #Plain left click selects points (field_clicked). Keys are all window shortcuts - see keybinds.py.
    def mousePressEvent(self, event):
        ctrl_drag = event.button() == Qt.LeftButton and event.modifiers() & Qt.ControlModifier
        if event.button() in self.PAN_BUTTONS or ctrl_drag:
            self._pan_last = event.position().toPoint()
            self.viewport().setCursor(Qt.ClosedHandCursor)
            return
        if event.button() == Qt.LeftButton:
            item = None if self.placing else self.itemAt(event.position().toPoint())
            if item is not None and item.isVisible() and item.data(0) == INTERACTIVE:
                super().mousePressEvent(event)  #let the item handle (drag) it
                return
            x, y = scene_to_field(self.mapToScene(event.position().toPoint()))
            self.field_clicked.emit(x, y, 1.0 / self.transform().m11())
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self._pan_last is not None:
            delta = event.position().toPoint() - self._pan_last
            self._pan_last = event.position().toPoint()
            self.horizontalScrollBar().setValue(self.horizontalScrollBar().value() - delta.x())
            self.verticalScrollBar().setValue(self.verticalScrollBar().value() - delta.y())
            self.view_changed.emit()
        x, y = scene_to_field(self.mapToScene(event.position().toPoint()))
        self.cursor_moved.emit(x, y)
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if self._pan_last is not None:
            self._pan_last = None
            self.set_placing(self.placing)  #back to the crosshair if placing, else the normal cursor
            return
        super().mouseReleaseEvent(event)

    def contextMenuEvent(self, event):
        event.accept()  #right button is used for panning, don't pop a menu

    def leaveEvent(self, event):
        self.cursor_left.emit()
        super().leaveEvent(event)
