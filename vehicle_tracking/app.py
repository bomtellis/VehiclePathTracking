from __future__ import annotations

from collections import deque
from math import atan2, ceil, cos, degrees, hypot, radians, sin
from pathlib import Path
import sys

from PySide6.QtCore import QPointF, QRectF, QSize, Qt, QTimer, Signal
from PySide6.QtGui import QAction, QBrush, QColor, QKeyEvent, QPen, QPolygonF, QTransform
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QDialog,
    QDialogButtonBox,
    QFileDialog,
    QFormLayout,
    QFrame,
    QGraphicsEllipseItem,
    QGraphicsItem,
    QGraphicsItemGroup,
    QGraphicsLineItem,
    QGraphicsPathItem,
    QGraphicsPolygonItem,
    QGraphicsScene,
    QGraphicsView,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QProgressDialog,
    QScrollArea,
    QSlider,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QToolBar,
    QVBoxLayout,
    QWidget,
)
from PySide6.QtGui import QPainterPath

from .dxf_io import (
    DxfBlockGeometry,
    DxfDrawing,
    export_tracking_dxf,
    get_block_geometry,
    load_dxf,
    payload_outline_points,
    vehicle_corners,
)
from .models import Pose, RoutePlan, RouteStore, SteeringMode, VehicleProfile, VehicleStore, WheelSpec, step_pose
from .qtbootstrap import QtBootstrap, line_icon


ROOT = Path(__file__).resolve().parent.parent


class TrackingView(QGraphicsView):
    positionPlaced = Signal(str, QPointF, float)

    def __init__(self, scene: QGraphicsScene) -> None:
        super().__init__(scene)
        self.setRenderHints(self.renderHints())
        self.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self._placement_mode: str | None = None
        self._placement_anchor: QPointF | None = None
        self._default_heading = 0.0
        self._heading_line: QGraphicsLineItem | None = None

    def set_placement_mode(self, mode: str | None, default_heading: float = 0.0) -> None:
        if self._heading_line is not None and self._heading_line.scene() is self.scene():
            self.scene().removeItem(self._heading_line)
        self._heading_line = None
        self._placement_anchor = None
        self._placement_mode = mode
        self._default_heading = default_heading
        if mode is None:
            self.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)
            self.unsetCursor()
        else:
            self.setDragMode(QGraphicsView.DragMode.NoDrag)
            self.setCursor(Qt.CursorShape.CrossCursor)

    def mousePressEvent(self, event) -> None:
        if self._placement_mode is not None:
            if event.button() == Qt.MouseButton.LeftButton:
                self._placement_anchor = self.mapToScene(event.position().toPoint())
                pen = QPen(QColor("#d97706"), 0)
                pen.setStyle(Qt.PenStyle.DashLine)
                self._heading_line = self.scene().addLine(
                    self._placement_anchor.x(),
                    self._placement_anchor.y(),
                    self._placement_anchor.x(),
                    self._placement_anchor.y(),
                    pen,
                )
                event.accept()
                return
            if event.button() == Qt.MouseButton.RightButton:
                self.set_placement_mode(None)
                event.accept()
                return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:
        if self._placement_anchor is not None and self._heading_line is not None:
            current = self.mapToScene(event.position().toPoint())
            self._heading_line.setLine(
                self._placement_anchor.x(), self._placement_anchor.y(), current.x(), current.y()
            )
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event) -> None:
        if (
            self._placement_mode is not None
            and self._placement_anchor is not None
            and event.button() == Qt.MouseButton.LeftButton
        ):
            mode = self._placement_mode
            anchor = self._placement_anchor
            current = self.mapToScene(event.position().toPoint())
            pixel_anchor = self.mapFromScene(anchor)
            pixel_distance = hypot(
                event.position().x() - pixel_anchor.x(), event.position().y() - pixel_anchor.y()
            )
            heading = self._default_heading
            if pixel_distance >= 5.0:
                heading = degrees(atan2(-(current.y() - anchor.y()), current.x() - anchor.x()))
            self.set_placement_mode(None)
            self.positionPlaced.emit(mode, anchor, heading)
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def wheelEvent(self, event) -> None:
        factor = 1.15 if event.angleDelta().y() > 0 else 1 / 1.15
        self.scale(factor, factor)


class PoseHandleItem(QGraphicsItemGroup):
    def __init__(
        self,
        kind: str,
        x: float,
        scene_y: float,
        heading_deg: float,
        size: float,
        color: QColor,
        end_marker: bool,
        moved_callback,
        released_callback,
    ) -> None:
        super().__init__()
        self.kind = kind
        self._ready = False
        self._moved_callback = moved_callback
        self._released_callback = released_callback
        pen = QPen(color, 0)
        brush_color = QColor(color)
        brush_color.setAlpha(45)
        ellipse = QGraphicsEllipseItem(-size, -size, size * 2.0, size * 2.0)
        ellipse.setPen(pen)
        ellipse.setBrush(QBrush(brush_color))
        self.addToGroup(ellipse)
        if end_marker:
            first = QGraphicsLineItem(-size, -size, size, size)
            second = QGraphicsLineItem(-size, size, size, -size)
            first.setPen(pen)
            second.setPen(pen)
            self.addToGroup(first)
            self.addToGroup(second)
        heading = radians(heading_deg)
        arrow = QGraphicsLineItem(0.0, 0.0, cos(heading) * size * 1.8, -sin(heading) * size * 1.8)
        arrow.setPen(pen)
        self.addToGroup(arrow)
        self.setPos(x, scene_y)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsMovable, True)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable, True)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemSendsGeometryChanges, True)
        self.setCursor(Qt.CursorShape.OpenHandCursor)
        self.setZValue(20.0)
        self.setToolTip(f"Drag the {kind} position")
        self._ready = True

    def itemChange(self, change, value):
        if self._ready and change == QGraphicsItem.GraphicsItemChange.ItemPositionHasChanged:
            self._moved_callback(self.kind, value)
        return super().itemChange(change, value)

    def mousePressEvent(self, event) -> None:
        self.setCursor(Qt.CursorShape.ClosedHandCursor)
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event) -> None:
        super().mouseReleaseEvent(event)
        self.setCursor(Qt.CursorShape.OpenHandCursor)
        self._released_callback(self.kind, self.pos())


class RoutePointHandleItem(QGraphicsEllipseItem):
    def __init__(self, index: int, x: float, scene_y: float, size: float, moved_callback, released_callback) -> None:
        super().__init__(-size, -size, size * 2.0, size * 2.0)
        self.index = index
        self._ready = False
        self._moved_callback = moved_callback
        self._released_callback = released_callback
        self.setPen(QPen(QColor("#b45309"), 0))
        self.setBrush(QBrush(QColor("#f59e0b")))
        self.setPos(x, scene_y)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsMovable, True)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable, True)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemSendsGeometryChanges, True)
        self.setCursor(Qt.CursorShape.OpenHandCursor)
        self.setZValue(25.0)
        self.setToolTip(f"Route control point {index + 1}: drag to reshape the route")
        self._ready = True

    def itemChange(self, change, value):
        if self._ready and change == QGraphicsItem.GraphicsItemChange.ItemPositionHasChanged:
            self._moved_callback(self.index, value)
        return super().itemChange(change, value)

    def mousePressEvent(self, event) -> None:
        self.setCursor(Qt.CursorShape.ClosedHandCursor)
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event) -> None:
        super().mouseReleaseEvent(event)
        self.setCursor(Qt.CursorShape.OpenHandCursor)
        self._released_callback(self.index, self.pos())


class WheelPlacementView(QGraphicsView):
    pointPlaced = Signal(QPointF)
    directionDrawn = Signal(float)

    def __init__(self, scene: QGraphicsScene) -> None:
        super().__init__(scene)
        self.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self._placing = False
        self._drawing_direction = False
        self._direction_anchor: QPointF | None = None
        self._direction_line: QGraphicsLineItem | None = None

    def begin_placement(self) -> None:
        self._placing = True
        self._drawing_direction = False
        self.setDragMode(QGraphicsView.DragMode.NoDrag)
        self.setCursor(Qt.CursorShape.CrossCursor)

    def begin_direction(self) -> None:
        self._placing = False
        self._drawing_direction = True
        self.setDragMode(QGraphicsView.DragMode.NoDrag)
        self.setCursor(Qt.CursorShape.CrossCursor)

    def mousePressEvent(self, event) -> None:
        if self._placing:
            if event.button() == Qt.MouseButton.LeftButton:
                point = self.mapToScene(event.position().toPoint())
                self._placing = False
                self.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)
                self.unsetCursor()
                self.pointPlaced.emit(point)
                event.accept()
                return
            if event.button() == Qt.MouseButton.RightButton:
                self._placing = False
                self.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)
                self.unsetCursor()
                event.accept()
                return
        if self._drawing_direction:
            if event.button() == Qt.MouseButton.LeftButton:
                self._direction_anchor = self.mapToScene(event.position().toPoint())
                pen = QPen(QColor("#16a34a"), 0)
                pen.setStyle(Qt.PenStyle.DashLine)
                self._direction_line = self.scene().addLine(
                    self._direction_anchor.x(),
                    self._direction_anchor.y(),
                    self._direction_anchor.x(),
                    self._direction_anchor.y(),
                    pen,
                )
                event.accept()
                return
            if event.button() == Qt.MouseButton.RightButton:
                self._cancel_direction()
                event.accept()
                return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:
        if self._direction_anchor is not None and self._direction_line is not None:
            point = self.mapToScene(event.position().toPoint())
            self._direction_line.setLine(
                self._direction_anchor.x(), self._direction_anchor.y(), point.x(), point.y()
            )
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event) -> None:
        if (
            self._direction_anchor is not None
            and self._drawing_direction
            and event.button() == Qt.MouseButton.LeftButton
        ):
            end = self.mapToScene(event.position().toPoint())
            dx = end.x() - self._direction_anchor.x()
            dy = -(end.y() - self._direction_anchor.y())
            angle = degrees(atan2(dy, dx)) if hypot(dx, dy) > 1e-9 else 0.0
            self._cancel_direction()
            self.directionDrawn.emit(angle)
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def _cancel_direction(self) -> None:
        if self._direction_line is not None and self._direction_line.scene() is self.scene():
            self.scene().removeItem(self._direction_line)
        self._direction_line = None
        self._direction_anchor = None
        self._drawing_direction = False
        self.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)
        self.unsetCursor()

    def wheelEvent(self, event) -> None:
        factor = 1.15 if event.angleDelta().y() > 0 else 1 / 1.15
        self.scale(factor, factor)


class WheelPlacementDialog(QDialog):
    def __init__(
        self,
        geometry: DxfBlockGeometry,
        wheels: list[WheelSpec],
        forward_angle_deg: float = 0.0,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle(f"Place Wheels — {geometry.name}")
        self.resize(960, 650)
        self.geometry = geometry
        self.result_wheels: list[WheelSpec] = []
        self.forward_angle_deg = forward_angle_deg
        self.result_forward_angle_deg = forward_angle_deg
        self._move_row: int | None = None
        self.scene = QGraphicsScene(self)
        self.view = WheelPlacementView(self.scene)
        self.view.pointPlaced.connect(self._place_wheel)
        self.view.directionDrawn.connect(self._set_forward_angle)

        root = QVBoxLayout(self)
        instructions = QLabel(
            "Draw the forward travel direction first. Then place or move wheel centres on the block. "
            "Wheel X/Y values are measured in that vehicle-oriented coordinate system."
        )
        instructions.setWordWrap(True)
        root.addWidget(instructions)

        splitter = QSplitter()
        splitter.addWidget(self.view)
        editor = QWidget()
        editor_layout = QVBoxLayout(editor)
        orientation_title = QLabel("Travel Direction")
        orientation_title.setObjectName("SectionTitle")
        editor_layout.addWidget(orientation_title)
        direction_controls = QHBoxLayout()
        draw_direction = QPushButton(line_icon("direction", "#ffffff"), "Draw Travel Direction")
        draw_direction.clicked.connect(self.view.begin_direction)
        self.forward_angle_spin = QDoubleSpinBox()
        self.forward_angle_spin.setRange(-360.0, 360.0)
        self.forward_angle_spin.setDecimals(1)
        self.forward_angle_spin.setSuffix("°")
        self.forward_angle_spin.setValue(forward_angle_deg)
        self.forward_angle_spin.valueChanged.connect(self._set_forward_angle)
        direction_controls.addWidget(draw_direction)
        direction_controls.addWidget(self.forward_angle_spin)
        editor_layout.addLayout(direction_controls)
        legend = QLabel("Green arrow = forward travel. Wheel long axis = rolling direction. Red = steerable; blue = fixed.")
        legend.setWordWrap(True)
        editor_layout.addWidget(legend)
        self.table = QTableWidget(0, 6)
        self.table.setHorizontalHeaderLabels(["Name", "X", "Y", "Radius", "Steer", "Drive"])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        editor_layout.addWidget(self.table)

        controls = QHBoxLayout()
        place_new = QPushButton(line_icon("add", "#ffffff"), "Place New Wheel")
        place_new.clicked.connect(self._begin_new)
        move_selected = QPushButton("Move Selected")
        move_selected.clicked.connect(self._begin_move)
        remove_selected = QPushButton("Remove")
        remove_selected.setProperty("variant", "secondary")
        remove_selected.clicked.connect(self._remove_selected)
        controls.addWidget(place_new)
        controls.addWidget(move_selected)
        controls.addWidget(remove_selected)
        editor_layout.addLayout(controls)
        clear_all = QPushButton("Clear All Wheels")
        clear_all.setProperty("variant", "secondary")
        clear_all.clicked.connect(self._clear_all)
        editor_layout.addWidget(clear_all)
        splitter.addWidget(editor)
        splitter.setSizes([620, 320])
        root.addWidget(splitter)

        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Save | QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self._save)
        buttons.rejected.connect(self.reject)
        root.addWidget(buttons)

        for wheel in wheels:
            self._append_wheel(wheel)
        self._redraw()

    def _append_wheel(self, wheel: WheelSpec) -> None:
        row = self.table.rowCount()
        self.table.insertRow(row)
        values = [
            wheel.name,
            f"{wheel.x:.3f}",
            f"{wheel.y:.3f}",
            f"{wheel.radius:.3f}",
            "yes" if wheel.steerable else "no",
            "yes" if wheel.drive else "no",
        ]
        for column, value in enumerate(values):
            self.table.setItem(row, column, QTableWidgetItem(value))

    def _wheels_from_table(self) -> list[WheelSpec]:
        wheels: list[WheelSpec] = []
        for row in range(self.table.rowCount()):
            cells = [self.table.item(row, column).text() if self.table.item(row, column) else "" for column in range(6)]
            wheels.append(
                WheelSpec(
                    cells[0] or f"Wheel {row + 1}",
                    float(cells[1] or 0.0),
                    float(cells[2] or 0.0),
                    float(cells[3] or self._default_radius()),
                    steerable=cells[4].strip().lower() in {"yes", "true", "1", "y"},
                    drive=cells[5].strip().lower() in {"yes", "true", "1", "y"},
                )
            )
        return wheels

    def _default_radius(self) -> float:
        min_x, min_y, max_x, max_y = self.geometry.bounds
        return max(max(max_x - min_x, max_y - min_y) / 20.0, 0.01)

    @staticmethod
    def _vehicle_to_block(x: float, y: float, angle_deg: float) -> tuple[float, float]:
        angle = radians(angle_deg)
        return x * cos(angle) - y * sin(angle), x * sin(angle) + y * cos(angle)

    @staticmethod
    def _block_to_vehicle(x: float, y: float, angle_deg: float) -> tuple[float, float]:
        angle = radians(angle_deg)
        return x * cos(angle) + y * sin(angle), -x * sin(angle) + y * cos(angle)

    def _set_forward_angle(self, angle_deg: float) -> None:
        try:
            wheels = self._wheels_from_table()
        except ValueError:
            wheels = []
        block_positions = [
            self._vehicle_to_block(wheel.x, wheel.y, self.forward_angle_deg) for wheel in wheels
        ]
        self.forward_angle_deg = float(angle_deg)
        self.forward_angle_spin.blockSignals(True)
        self.forward_angle_spin.setValue(self.forward_angle_deg)
        self.forward_angle_spin.blockSignals(False)
        for row, (block_x, block_y) in enumerate(block_positions):
            vehicle_x, vehicle_y = self._block_to_vehicle(
                block_x, block_y, self.forward_angle_deg
            )
            self.table.setItem(row, 1, QTableWidgetItem(f"{vehicle_x:.3f}"))
            self.table.setItem(row, 2, QTableWidgetItem(f"{vehicle_y:.3f}"))
        self._redraw()

    def _begin_new(self) -> None:
        self._move_row = None
        self.view.begin_placement()

    def _begin_move(self) -> None:
        row = self.table.currentRow()
        if row < 0:
            QMessageBox.information(self, "Select a wheel", "Select a wheel row before choosing Move Selected.")
            return
        self._move_row = row
        self.view.begin_placement()

    def _place_wheel(self, scene_point: QPointF) -> None:
        block_x = float(scene_point.x())
        block_y = float(-scene_point.y())
        x, y = self._block_to_vehicle(block_x, block_y, self.forward_angle_deg)
        if self._move_row is None:
            index = self.table.rowCount() + 1
            self._append_wheel(WheelSpec(f"Wheel {index}", x, y, radius=self._default_radius()))
            self.table.selectRow(self.table.rowCount() - 1)
        else:
            self.table.setItem(self._move_row, 1, QTableWidgetItem(f"{x:.3f}"))
            self.table.setItem(self._move_row, 2, QTableWidgetItem(f"{y:.3f}"))
            self.table.selectRow(self._move_row)
        self._move_row = None
        self._redraw()

    def _remove_selected(self) -> None:
        row = self.table.currentRow()
        if row >= 0:
            self.table.removeRow(row)
            self._redraw()

    def _clear_all(self) -> None:
        self.table.setRowCount(0)
        self._redraw()

    def _redraw(self) -> None:
        self.scene.clear()
        pen = QPen(QColor("#64748b"), 0)
        _add_primitives_to_scene(self.scene, self.geometry.primitives, pen)
        span = max(
            self.geometry.bounds[2] - self.geometry.bounds[0],
            self.geometry.bounds[3] - self.geometry.bounds[1],
            1.0,
        )
        min_x, min_y, max_x, max_y = self.geometry.bounds
        extremity_pen = QPen(QColor("#a21caf"), 0)
        extremity_pen.setStyle(Qt.PenStyle.DashLine)
        bounds_item = self.scene.addRect(
            min_x,
            -max_y,
            max_x - min_x,
            max_y - min_y,
            extremity_pen,
        )
        bounds_item.setToolTip("Selected block extremity outline")
        extremity_size = span / 90.0
        for x, y in ((min_x, min_y), (max_x, min_y), (max_x, max_y), (min_x, max_y)):
            marker = self.scene.addEllipse(
                x - extremity_size,
                -y - extremity_size,
                extremity_size * 2.0,
                extremity_size * 2.0,
                QPen(QColor("#a21caf"), 0),
                QBrush(QColor("#a21caf")),
            )
            marker.setToolTip("Block extremity")
        origin_size = span / 25.0
        origin_pen = QPen(QColor("#16a34a"), 0)
        origin_x = self.scene.addLine(-origin_size, 0.0, origin_size, 0.0, origin_pen)
        origin_y = self.scene.addLine(0.0, -origin_size, 0.0, origin_size, origin_pen)
        origin_x.setToolTip("Block insertion point (0, 0)")
        origin_y.setToolTip("Block insertion point (0, 0)")
        center_x = (min_x + max_x) / 2.0
        center_y = (min_y + max_y) / 2.0
        direction_angle = radians(self.forward_angle_deg)
        direction_length = span * 0.32
        direction_x = cos(direction_angle)
        direction_y = sin(direction_angle)
        tip_x = center_x + direction_x * direction_length
        tip_y = center_y + direction_y * direction_length
        direction_pen = QPen(QColor("#16a34a"), 0)
        shaft = self.scene.addLine(center_x, -center_y, tip_x, -tip_y, direction_pen)
        shaft.setToolTip(f"Forward travel: {self.forward_angle_deg:.1f}°")
        head_length = direction_length * 0.22
        head_width = direction_length * 0.12
        base_x = tip_x - direction_x * head_length
        base_y = tip_y - direction_y * head_length
        normal_x = -direction_y
        normal_y = direction_x
        arrow = QGraphicsPolygonItem(
            QPolygonF(
                [
                    QPointF(tip_x, -tip_y),
                    QPointF(base_x + normal_x * head_width, -(base_y + normal_y * head_width)),
                    QPointF(base_x - normal_x * head_width, -(base_y - normal_y * head_width)),
                ]
            )
        )
        arrow.setPen(direction_pen)
        arrow.setBrush(QBrush(QColor("#16a34a")))
        arrow.setToolTip(f"Forward travel: {self.forward_angle_deg:.1f}°")
        self.scene.addItem(arrow)
        marker_minimum = span / 100.0
        try:
            wheels = self._wheels_from_table()
        except ValueError:
            wheels = []
        for wheel in wheels:
            radius = max(wheel.radius, marker_minimum)
            color = QColor("#dc2626") if wheel.steerable else QColor("#2563eb")
            block_x, block_y = self._vehicle_to_block(
                wheel.x, wheel.y, self.forward_angle_deg
            )
            half_width = max(radius * 0.35, marker_minimum * 0.45)
            forward_x = cos(direction_angle)
            forward_y = sin(direction_angle)
            side_x = -forward_y
            side_y = forward_x
            points = []
            for length_sign, width_sign in ((1, 1), (1, -1), (-1, -1), (-1, 1)):
                point_x = block_x + forward_x * radius * length_sign + side_x * half_width * width_sign
                point_y = block_y + forward_y * radius * length_sign + side_y * half_width * width_sign
                points.append(QPointF(point_x, -point_y))
            wheel_item = QGraphicsPolygonItem(QPolygonF(points))
            wheel_item.setPen(QPen(color, 0))
            fill = QColor(color)
            fill.setAlpha(220 if wheel.drive else 70)
            wheel_item.setBrush(QBrush(fill))
            state = "steerable" if wheel.steerable else "fixed"
            drive = "drive" if wheel.drive else "free-rolling"
            wheel_item.setToolTip(f"{wheel.name}: {state}, {drive}")
            self.scene.addItem(wheel_item)
        rect = self.scene.itemsBoundingRect()
        margin = max(rect.width(), rect.height()) * 0.1
        self.scene.setSceneRect(rect.adjusted(-margin, -margin, margin, margin))
        self.view.fitInView(rect.adjusted(-margin, -margin, margin, margin), Qt.AspectRatioMode.KeepAspectRatio)

    def _save(self) -> None:
        try:
            self.result_wheels = self._wheels_from_table()
        except ValueError:
            QMessageBox.warning(self, "Invalid wheel", "Wheel X, Y, and radius values must be numbers.")
            return
        self.result_forward_angle_deg = self.forward_angle_deg
        self.accept()


def _add_primitives_to_scene(scene: QGraphicsScene, primitives, pen: QPen) -> list:
    path = _primitives_to_path(primitives)
    if path.isEmpty():
        return []
    return [scene.addPath(path, pen)]


def _primitives_to_path(primitives) -> QPainterPath:
    path = QPainterPath()
    for primitive in primitives:
        if primitive.kind == "polyline" and len(primitive.points) >= 2:
            path.moveTo(primitive.points[0][0], -primitive.points[0][1])
            for x, y in primitive.points[1:]:
                path.lineTo(x, -y)
        elif primitive.kind == "point" and primitive.points:
            x, y = primitive.points[0]
            path.addEllipse(QPointF(x, -y), 0.05, 0.05)
    return path


class VehicleTrackerWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Vehicle Tracking")
        self.resize(1360, 840)
        self.store = VehicleStore(ROOT / "vehicles.json")
        self.vehicles = self.store.load()
        self.current_profile = self.vehicles[0]
        self.current_dxf: DxfDrawing | None = None
        self.route_store = RouteStore(ROOT / "vehicle_routes.json")
        saved_start, self.saved_routes = self.route_store.load(None)
        self.start_pose = saved_start or Pose(0.0, 0.0, 0.0, 0.0)
        self.end_pose: Pose | None = None
        self.route_waypoints: list[tuple[float, float]] = []
        self.active_route_index: int | None = None
        self._updating_route_combo = False
        self.poses = [self.start_pose]
        self.speed = 0.0
        self.travel_direction = 1
        self.steering = 0.0
        self.lateral = 0.0
        self.scene = QGraphicsScene(self)
        self.view = TrackingView(self.scene)
        self.view.positionPlaced.connect(self.place_position)
        self.timer = QTimer(self)
        self.timer.setInterval(80)
        self.timer.timeout.connect(self.advance_vehicle)
        self.route_animation_timer = QTimer(self)
        self.route_animation_timer.setInterval(50)
        self.route_animation_timer.timeout.connect(self.advance_route_animation)
        self.route_animation_poses: list[Pose] = []
        self.route_animation_index = 0
        self.route_animation_item: QGraphicsItemGroup | None = None
        self.vehicle_items: list = []
        self.path_item: QGraphicsPathItem | None = None
        self.sweep_items: list[QGraphicsPathItem] = []
        self.indicative_path_item: QGraphicsPathItem | None = None
        self.planned_sweep_items: list[QGraphicsPathItem] = []
        self.planned_block_trace_items: list = []
        self.route_failure_items: list[QGraphicsPathItem] = []
        self.saved_route_items: list[QGraphicsPathItem] = []
        self.payload_trace_items: list = []
        self.position_items: list = []
        self.route_point_items: list[RoutePointHandleItem] = []
        self._block_path_cache: dict[tuple[int, str], QPainterPath] = {}
        self._build_actions()
        self._build_layout()
        self._load_profile_to_form(self.current_profile)
        self._refresh_route_combo()
        self.redraw_scene()

    def _build_actions(self) -> None:
        toolbar = QToolBar("Main")
        toolbar.setIconSize(QSize(22, 22))
        self.addToolBar(toolbar)

        open_action = QAction(line_icon("open", "#ffffff"), "Import DXF", self)
        open_action.triggered.connect(self.import_dxf)
        toolbar.addAction(open_action)

        export_action = QAction(line_icon("export", "#ffffff"), "Export Tracking DXF", self)
        export_action.triggered.connect(self.export_dxf)
        toolbar.addAction(export_action)

        toolbar.addSeparator()
        self.run_action = QAction(line_icon("play", "#ffffff"), "Run", self)
        self.run_action.triggered.connect(self.toggle_run)
        toolbar.addAction(self.run_action)

        reset_action = QAction(line_icon("reset", "#ffffff"), "Reset Path", self)
        reset_action.triggered.connect(self.reset_path)
        toolbar.addAction(reset_action)

        toolbar.addSeparator()
        place_start_action = QAction(line_icon("start", "#ffffff"), "Place Vehicle / Start Position", self)
        place_start_action.triggered.connect(self.begin_place_start)
        toolbar.addAction(place_start_action)

        place_end_action = QAction(line_icon("end", "#ffffff"), "Place End Position", self)
        place_end_action.triggered.connect(self.begin_place_end)
        toolbar.addAction(place_end_action)

        fit_action = QAction(line_icon("fit", "#ffffff"), "Fit DXF", self)
        fit_action.triggered.connect(self.fit_drawing)
        toolbar.addAction(fit_action)

    def _build_layout(self) -> None:
        splitter = QSplitter()
        splitter.addWidget(self.view)
        side_scroll = QScrollArea()
        side_scroll.setWidgetResizable(True)
        side_scroll.setFrameShape(QFrame.Shape.NoFrame)
        side_scroll.setWidget(self._side_panel())
        side_scroll.setMinimumWidth(400)
        splitter.addWidget(side_scroll)
        splitter.setSizes([960, 400])
        self.setCentralWidget(splitter)
        self.statusBar().showMessage("Import a DXF or start steering on the empty canvas.")

    def _side_panel(self) -> QWidget:
        panel = QFrame()
        panel.setObjectName("SidePanel")
        panel.setMinimumWidth(380)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(18, 18, 18, 18)
        layout.setSpacing(12)

        title = QLabel("Vehicle Setup")
        title.setObjectName("PanelTitle")
        layout.addWidget(title)

        self.vehicle_combo = QComboBox()
        self.vehicle_combo.addItems([vehicle.name for vehicle in self.vehicles])
        self.vehicle_combo.currentIndexChanged.connect(self.change_vehicle)
        layout.addWidget(self.vehicle_combo)

        form = QFormLayout()
        form.setLabelAlignment(Qt.AlignmentFlag.AlignLeft)
        self.name_edit = QLineEdit()
        self.block_combo = QComboBox()
        self.block_combo.setEditable(True)
        self.block_combo.currentTextChanged.connect(self.selected_block_changed)
        self.block_forward_spin = QDoubleSpinBox()
        self.block_forward_spin.setRange(-360.0, 360.0)
        self.block_forward_spin.setDecimals(1)
        self.block_forward_spin.setSuffix("°")
        self.block_forward_spin.valueChanged.connect(self.selected_block_changed)
        self.length_spin = self._spin(0.001, 1_000_000.0, 2.8)
        self.width_spin = self._spin(0.001, 1_000_000.0, 1.2)
        self.wheelbase_spin = self._spin(0.001, 1_000_000.0, 1.8)
        self.steering_mode_combo = QComboBox()
        for mode in SteeringMode:
            self.steering_mode_combo.addItem(mode.label, mode.value)
        self.max_steer_spin = self._spin(1.0, 89.0, 70.0)
        self.min_radius_spin = self._spin(0.001, 1_000_000.0, 1.4)
        self.pose_spacing_spin = self._spin(0.001, 1_000_000.0, 0.75)
        form.addRow("Name", self.name_edit)
        form.addRow("DXF block", self.block_combo)
        form.addRow("Block forward", self.block_forward_spin)
        form.addRow("Length", self.length_spin)
        form.addRow("Width", self.width_spin)
        form.addRow("Wheelbase", self.wheelbase_spin)
        form.addRow("Steering", self.steering_mode_combo)
        form.addRow("Max steer deg", self.max_steer_spin)
        form.addRow("Min turn radius", self.min_radius_spin)
        form.addRow("Pose spacing", self.pose_spacing_spin)
        layout.addLayout(form)

        place_wheels = QPushButton(line_icon("wheel", "#ffffff"), "Place Wheels on Selected Block")
        place_wheels.clicked.connect(self.place_wheels_on_block)
        layout.addWidget(place_wheels)

        wheel_title = QLabel("Wheel Layer")
        wheel_title.setObjectName("SectionTitle")
        layout.addWidget(wheel_title)
        self.wheel_table = QTableWidget(0, 6)
        self.wheel_table.setHorizontalHeaderLabels(["Name", "X", "Y", "Radius", "Steer", "Drive"])
        self.wheel_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.wheel_table.setMinimumHeight(160)
        layout.addWidget(self.wheel_table)

        wheel_buttons = QHBoxLayout()
        add_wheel = QPushButton(line_icon("add", "#ffffff"), "Add")
        add_wheel.clicked.connect(self.add_wheel_row)
        remove_wheel = QPushButton("Remove")
        remove_wheel.setProperty("variant", "secondary")
        remove_wheel.clicked.connect(self.remove_wheel_row)
        wheel_buttons.addWidget(add_wheel)
        wheel_buttons.addWidget(remove_wheel)
        layout.addLayout(wheel_buttons)

        payload_title = QLabel("Payload Tracking")
        payload_title.setObjectName("SectionTitle")
        layout.addWidget(payload_title)
        self.payload_enabled_checkbox = QCheckBox("Track payload on vehicle")
        self.payload_enabled_checkbox.toggled.connect(self.payload_changed)
        layout.addWidget(self.payload_enabled_checkbox)
        payload_form = QFormLayout()
        self.payload_x_spin = self._spin(-1_000_000.0, 1_000_000.0, 0.0)
        self.payload_y_spin = self._spin(-1_000_000.0, 1_000_000.0, 0.0)
        self.payload_length_spin = self._spin(0.001, 1_000_000.0, 1.2)
        self.payload_width_spin = self._spin(0.001, 1_000_000.0, 1.0)
        self.payload_rotation_spin = self._spin(-360.0, 360.0, 0.0)
        self.payload_rotation_spin.setSuffix("°")
        for control in (
            self.payload_x_spin,
            self.payload_y_spin,
            self.payload_length_spin,
            self.payload_width_spin,
            self.payload_rotation_spin,
        ):
            control.valueChanged.connect(self.payload_changed)
        payload_form.addRow("Centre X", self.payload_x_spin)
        payload_form.addRow("Centre Y", self.payload_y_spin)
        payload_form.addRow("Length", self.payload_length_spin)
        payload_form.addRow("Width", self.payload_width_spin)
        payload_form.addRow("Rotation", self.payload_rotation_spin)
        layout.addLayout(payload_form)

        save_profile = QPushButton(line_icon("save", "#ffffff"), "Save Vehicle")
        save_profile.clicked.connect(self.save_vehicle)
        layout.addWidget(save_profile)

        position_title = QLabel("Start / End Positions")
        position_title.setObjectName("SectionTitle")
        layout.addWidget(position_title)
        position_buttons = QHBoxLayout()
        place_start = QPushButton(line_icon("start", "#ffffff"), "Place Vehicle")
        place_start.clicked.connect(self.begin_place_start)
        place_end = QPushButton(line_icon("end", "#ffffff"), "Place End")
        place_end.clicked.connect(self.begin_place_end)
        position_buttons.addWidget(place_start)
        position_buttons.addWidget(place_end)
        layout.addLayout(position_buttons)
        heading_form = QFormLayout()
        self.start_heading_spin = QDoubleSpinBox()
        self.start_heading_spin.setRange(-360.0, 360.0)
        self.start_heading_spin.setDecimals(1)
        self.start_heading_spin.setSuffix("°")
        self.start_heading_spin.valueChanged.connect(self.update_pose_headings)
        self.end_heading_spin = QDoubleSpinBox()
        self.end_heading_spin.setRange(-360.0, 360.0)
        self.end_heading_spin.setDecimals(1)
        self.end_heading_spin.setSuffix("°")
        self.end_heading_spin.valueChanged.connect(self.update_pose_headings)
        heading_form.addRow("Start heading", self.start_heading_spin)
        heading_form.addRow("End heading", self.end_heading_spin)
        layout.addLayout(heading_form)
        spacing_form = QFormLayout()
        self.endpoint_spacing_mode_combo = QComboBox()
        self.endpoint_spacing_mode_combo.addItem("Freehand", "freehand")
        self.endpoint_spacing_mode_combo.addItem("Vehicle clearance", "vehicle")
        self.endpoint_spacing_mode_combo.addItem("Payload clearance", "payload")
        self.endpoint_spacing_spin = self._spin(0.0, 1_000_000.0, 0.5)
        self.endpoint_spacing_spin.setDecimals(3)
        self.endpoint_spacing_spin.setSingleStep(0.01)
        spacing_form.addRow("End placement", self.endpoint_spacing_mode_combo)
        spacing_form.addRow("Clearance", self.endpoint_spacing_spin)
        layout.addLayout(spacing_form)

        path_title = QLabel("Saved Paths")
        path_title.setObjectName("SectionTitle")
        layout.addWidget(path_title)
        self.route_combo = QComboBox()
        self.route_combo.currentIndexChanged.connect(self.change_saved_route)
        layout.addWidget(self.route_combo)
        self.route_name_edit = QLineEdit()
        self.route_name_edit.setPlaceholderText("Path name")
        layout.addWidget(self.route_name_edit)
        path_buttons = QHBoxLayout()
        save_path = QPushButton(line_icon("save", "#ffffff"), "Save Path")
        save_path.clicked.connect(self.save_current_route)
        new_path = QPushButton(line_icon("add", "#ffffff"), "New Path")
        new_path.clicked.connect(self.new_route)
        remove_path = QPushButton("Remove Path")
        remove_path.setProperty("variant", "secondary")
        remove_path.clicked.connect(self.remove_saved_route)
        path_buttons.addWidget(save_path)
        path_buttons.addWidget(new_path)
        path_buttons.addWidget(remove_path)
        layout.addLayout(path_buttons)
        self.show_route_checkbox = QCheckBox("Show planned route and swept envelope")
        self.show_route_checkbox.setChecked(True)
        self.show_route_checkbox.toggled.connect(self.toggle_route_visibility)
        layout.addWidget(self.show_route_checkbox)
        self.route_feasibility_label = QLabel("Route check: place a finish position")
        self.route_feasibility_label.setWordWrap(True)
        self.route_feasibility_label.setStyleSheet("color: #667085; font-weight: 700;")
        layout.addWidget(self.route_feasibility_label)
        route_edit_buttons = QHBoxLayout()
        insert_route_point = QPushButton(line_icon("add", "#ffffff"), "Insert Route Point")
        insert_route_point.clicked.connect(self.begin_insert_route_point)
        remove_route_point = QPushButton("Remove Selected")
        remove_route_point.setProperty("variant", "secondary")
        remove_route_point.clicked.connect(self.remove_selected_route_point)
        route_edit_buttons.addWidget(insert_route_point)
        route_edit_buttons.addWidget(remove_route_point)
        layout.addLayout(route_edit_buttons)
        route_actions = QHBoxLayout()
        clear_route_points = QPushButton("Clear Route Points")
        clear_route_points.setProperty("variant", "secondary")
        clear_route_points.clicked.connect(self.clear_route_points)
        self.animate_route_button = QPushButton(line_icon("play", "#ffffff"), "Animate Route")
        self.animate_route_button.clicked.connect(self.toggle_route_animation)
        route_actions.addWidget(clear_route_points)
        route_actions.addWidget(self.animate_route_button)
        layout.addLayout(route_actions)
        self.position_label = QLabel()
        self.position_label.setObjectName("PositionSummary")
        self.position_label.setWordWrap(True)
        layout.addWidget(self.position_label)
        self._update_position_label()

        controls_title = QLabel("Steering Controls")
        controls_title.setObjectName("SectionTitle")
        layout.addWidget(controls_title)
        self.steer_slider = QSlider(Qt.Orientation.Horizontal)
        self.steer_slider.setRange(-100, 100)
        self.steer_slider.valueChanged.connect(self.update_steer_from_slider)
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setRange(-100, 100)
        self.speed_slider.valueChanged.connect(self.update_speed_from_slider)
        self.steer_value_label = QLabel("Steer: 0.0°")
        self.speed_value_label = QLabel("Speed: stopped")
        self.direction_value_label = QLabel("Direction of travel: Forward (stopped)")
        self.direction_value_label.setObjectName("DirectionIndicator")
        self.direction_value_label.setStyleSheet("color: #16a34a; font-weight: 700;")
        layout.addWidget(self.direction_value_label)
        layout.addWidget(self.steer_value_label)
        layout.addWidget(self.steer_slider)
        layout.addWidget(self.speed_value_label)
        layout.addWidget(self.speed_slider)

        steer_buttons = QHBoxLayout()
        left = QPushButton(line_icon("left", "#ffffff"), "Left")
        left.clicked.connect(lambda: self.bump_steer(-10))
        stop = QPushButton(line_icon("stop", "#ffffff"), "Stop")
        stop.setProperty("variant", "warning")
        stop.clicked.connect(self.stop_vehicle)
        right = QPushButton(line_icon("right", "#ffffff"), "Right")
        right.clicked.connect(lambda: self.bump_steer(10))
        steer_buttons.addWidget(left)
        steer_buttons.addWidget(stop)
        steer_buttons.addWidget(right)
        layout.addLayout(steer_buttons)

        layout.addStretch(1)
        return panel

    def _spin(self, minimum: float, maximum: float, value: float) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setDecimals(3)
        spin.setValue(value)
        spin.setSingleStep(0.1)
        return spin

    def import_dxf(self) -> None:
        filename, _ = QFileDialog.getOpenFileName(self, "Import DXF", str(ROOT), "DXF files (*.dxf)")
        if not filename:
            return
        try:
            self.current_dxf = load_dxf(Path(filename))
        except Exception as exc:
            QMessageBox.critical(self, "Import failed", str(exc))
            return
        self._block_path_cache.clear()
        self.block_combo.clear()
        self.block_combo.addItem("")
        self.block_combo.addItems(self.current_dxf.block_names)
        if self.current_profile.dxf_block_name:
            self.block_combo.setCurrentText(self.current_profile.dxf_block_name)
        self.start_pose = Pose(0.0, 0.0, 0.0, 0.0)
        saved_start, self.saved_routes = self.route_store.load(self.current_dxf.path)
        if saved_start is not None:
            self.start_pose = saved_start
        self.poses = [self.start_pose]
        self.end_pose = None
        self.route_waypoints.clear()
        self.active_route_index = None
        self.route_name_edit.setText(self._next_route_name())
        self._refresh_route_combo()
        self.stop_route_animation()
        self.start_heading_spin.setValue(self.start_pose.heading_deg)
        self.end_heading_spin.setValue(0.0)
        self._update_position_label()
        self.redraw_scene()
        detail = f"Imported {len(self.current_dxf.primitives):,} drawing paths from {filename}"
        if self.current_dxf.unsupported_types:
            detail += f" (unsupported: {', '.join(self.current_dxf.unsupported_types)})"
        self.statusBar().showMessage(detail)

    def export_dxf(self) -> None:
        default = ROOT / "vehicle_tracking_export.dxf"
        if self.current_dxf:
            default = self.current_dxf.path.with_name(f"{self.current_dxf.path.stem}_vehicle_tracking.dxf")
        filename, _ = QFileDialog.getSaveFileName(self, "Export Tracking DXF", str(default), "DXF files (*.dxf)")
        if not filename:
            return
        profile = self.form_profile()
        progress = QProgressDialog("Preparing clean DXF", None, 0, 100, self)
        progress.setWindowTitle("Export Tracking DXF")
        progress.setWindowModality(Qt.WindowModality.WindowModal)
        progress.setMinimumDuration(0)
        progress.setAutoClose(False)
        progress.setValue(0)
        progress.show()

        def update_progress(value: int, message: str) -> None:
            progress.setLabelText(message)
            progress.setValue(value)
            self.statusBar().showMessage(f"DXF export: {message} ({value}%)")
            QApplication.processEvents()

        try:
            planned_poses = self.planned_route_poses(profile)
            planned_routes = self.all_planned_route_poses(profile)
            block_outline = self.block_outline_points(profile)
            export_tracking_dxf(
                source_path=self.current_dxf.path if self.current_dxf else None,
                output_path=Path(filename),
                profile=profile,
                poses=self.poses,
                planned_poses=planned_poses,
                block_outline=block_outline,
                progress_callback=update_progress,
                planned_routes=planned_routes,
            )
        except Exception as exc:
            QMessageBox.critical(self, "Export failed", str(exc))
            return
        finally:
            progress.close()
        route_note = f" with {len(planned_routes)} planned route(s)" if planned_routes else ""
        self.statusBar().showMessage(
            f"Exported application geometry only to {filename}{route_note}; source DXF coordinates preserved"
        )

    def toggle_run(self) -> None:
        if self.timer.isActive():
            self.timer.stop()
            self.run_action.setText("Run")
            self.run_action.setIcon(line_icon("play", "#ffffff"))
        else:
            self.stop_route_animation()
            self.timer.start()
            self.run_action.setText("Pause")
            self.run_action.setIcon(line_icon("stop", "#ffffff"))

    def stop_vehicle(self) -> None:
        self.speed = 0.0
        self.lateral = 0.0
        self.speed_slider.setValue(0)

    def reset_path(self) -> None:
        self.poses = [self.start_pose]
        self.speed = 0.0
        self.steering = 0.0
        self.speed_slider.setValue(0)
        self.steer_slider.setValue(0)
        self.redraw_scene()

    def begin_place_start(self) -> None:
        self.view.set_placement_mode("start", self.start_heading_spin.value())
        self.statusBar().showMessage(
            "Press at the vehicle position, drag in its facing direction, then release. Right-click to cancel."
        )

    def begin_place_end(self) -> None:
        self.view.set_placement_mode("end", self.end_heading_spin.value())
        mode = self.endpoint_spacing_mode_combo.currentText()
        spacing = self.endpoint_spacing_spin.value()
        self.statusBar().showMessage(
            f"Place the endpoint ({mode}, clearance {spacing:.3f}); drag in its facing direction and release."
        )

    def begin_insert_route_point(self) -> None:
        if self.end_pose is None:
            QMessageBox.information(self, "Place an end position", "Place the finish position before inserting route points.")
            return
        self.view.set_placement_mode("route")
        self.statusBar().showMessage("Click the orange planned route to insert a draggable control point.")

    def place_position(self, kind: str, scene_position: QPointF, heading_deg: float = 0.0) -> None:
        x = float(scene_position.x())
        y = float(-scene_position.y())
        snap_note = ""
        self.stop_route_animation()
        if kind == "route":
            route = self.planned_route_poses(self.form_profile())
            if not route:
                return
            nearest_index, nearest_pose = min(
                enumerate(route),
                key=lambda item: (item[1].x - x) ** 2 + (item[1].y - y) ** 2,
            )
            segment_index = min(len(self.route_waypoints), nearest_index // 40)
            self.route_waypoints.insert(segment_index, (nearest_pose.x, nearest_pose.y))
            self.redraw_dynamic_layers(self.form_profile())
            self.redraw_route_handles()
            self._update_navigation_bounds()
            self.statusBar().showMessage(
                f"Inserted route point {segment_index + 1}; drag it to tighten the path."
            )
            return
        if kind == "start":
            if self.timer.isActive():
                self.timer.stop()
                self.run_action.setText("Run")
                self.run_action.setIcon(line_icon("play", "#ffffff"))
            self.start_pose = Pose(x, y, heading_deg, 0.0)
            self.poses = [self.start_pose]
            self.start_heading_spin.blockSignals(True)
            self.start_heading_spin.setValue(heading_deg)
            self.start_heading_spin.blockSignals(False)
            self.speed = 0.0
            self.steering = 0.0
            self.speed_slider.setValue(0)
            self.steer_slider.setValue(0)
            if self.saved_routes:
                self._persist_routes()
            message = f"Vehicle set to X {x:.3f}, Y {y:.3f}, heading {heading_deg:.1f}°"
        else:
            x, y, snap_note = self._spaced_endpoint(x, y, heading_deg, self.form_profile())
            self.end_pose = Pose(x, y, heading_deg, 0.0)
            self.end_heading_spin.blockSignals(True)
            self.end_heading_spin.setValue(heading_deg)
            self.end_heading_spin.blockSignals(False)
            message = f"End set to X {x:.3f}, Y {y:.3f}, heading {heading_deg:.1f}°"
        message += snap_note
        self._update_position_label()
        self.redraw_dynamic_layers(self.form_profile())
        self.redraw_position_markers()
        self.redraw_route_handles()
        self._update_navigation_bounds()
        self.statusBar().showMessage(message)

    def update_pose_headings(self, _value: float = 0.0) -> None:
        self.stop_route_animation()
        if self.sender() is self.start_heading_spin:
            self.start_pose = Pose(
                self.start_pose.x,
                self.start_pose.y,
                self.start_heading_spin.value(),
                0.0,
            )
            self.poses = [self.start_pose]
            if self.saved_routes:
                self._persist_routes()
        if self.end_pose is not None:
            self.end_pose.heading_deg = self.end_heading_spin.value()
        self._update_position_label()
        if hasattr(self, "wheel_table"):
            self.redraw_dynamic_layers(self.form_profile())
            self.redraw_position_markers()

    def selected_block_changed(self, _block_name: str) -> None:
        self.stop_route_animation()
        if hasattr(self, "wheel_table") and self.poses:
            self.redraw_dynamic_layers(self.form_profile())

    def payload_changed(self, _value=None) -> None:
        self.stop_route_animation()
        if hasattr(self, "wheel_table") and self.poses:
            self.redraw_dynamic_layers(self.form_profile())

    def toggle_route_visibility(self, _checked: bool) -> None:
        if not _checked:
            self.stop_route_animation()
        if self.poses:
            self.redraw_dynamic_layers(self.form_profile())
            self.redraw_route_handles()

    def toggle_route_animation(self) -> None:
        if self.route_animation_timer.isActive():
            self.stop_route_animation()
            return
        route = self.planned_route_poses(self.form_profile())
        if len(route) < 2:
            QMessageBox.information(self, "Create a route", "Place start and finish positions before animating the route.")
            return
        if self.timer.isActive():
            self.timer.stop()
            self.run_action.setText("Run")
            self.run_action.setIcon(line_icon("play", "#ffffff"))
        self.show_route_checkbox.setChecked(True)
        self.route_animation_poses = route
        self.route_animation_index = 0
        self.animate_route_button.setText("Stop Animation")
        self.animate_route_button.setIcon(line_icon("stop", "#ffffff"))
        self.route_animation_timer.start()
        self.advance_route_animation()

    def advance_route_animation(self) -> None:
        if not self.route_animation_poses:
            self.stop_route_animation()
            return
        if self.route_animation_item is not None and self.route_animation_item.scene() is self.scene:
            self.scene.removeItem(self.route_animation_item)
        pose = self.route_animation_poses[self.route_animation_index]
        self.route_animation_item = self.draw_vehicle(
            self.form_profile(), pose, ghost=False, direction_override=1
        )
        self.route_animation_item.setOpacity(0.82)
        self.route_animation_item.setZValue(30.0)
        self.route_animation_index += 1
        if self.route_animation_index >= len(self.route_animation_poses):
            self.route_animation_timer.stop()
            self.route_animation_index = 0
            self.animate_route_button.setText("Replay Route")
            self.animate_route_button.setIcon(line_icon("play", "#ffffff"))

    def stop_route_animation(self) -> None:
        self.route_animation_timer.stop()
        self.route_animation_poses = []
        self.route_animation_index = 0
        if self.route_animation_item is not None and self.route_animation_item.scene() is self.scene:
            self.scene.removeItem(self.route_animation_item)
        self.route_animation_item = None
        if hasattr(self, "animate_route_button"):
            self.animate_route_button.setText("Animate Route")
            self.animate_route_button.setIcon(line_icon("play", "#ffffff"))

    def place_wheels_on_block(self) -> None:
        if self.current_dxf is None:
            QMessageBox.information(self, "Import a DXF", "Import a DXF before placing wheels on a block.")
            return
        block_name = self.block_combo.currentText().strip()
        geometry = get_block_geometry(self.current_dxf, block_name)
        if geometry is None:
            QMessageBox.warning(self, "Select a block", "Select a drawable DXF block before placing wheels.")
            return
        dialog = WheelPlacementDialog(
            geometry,
            self.form_profile().wheels,
            self.block_forward_spin.value(),
            self,
        )
        if dialog.exec() != QDialog.DialogCode.Accepted:
            return
        self._set_wheel_table(dialog.result_wheels)
        self.block_forward_spin.setValue(dialog.result_forward_angle_deg)
        oriented_points = [
            WheelPlacementDialog._block_to_vehicle(
                x, y, dialog.result_forward_angle_deg
            )
            for primitive in geometry.primitives
            for x, y in primitive.points
        ]
        if oriented_points:
            oriented_x = [point[0] for point in oriented_points]
            oriented_y = [point[1] for point in oriented_points]
            self.length_spin.setValue(max(oriented_x) - min(oriented_x))
            self.width_spin.setValue(max(oriented_y) - min(oriented_y))
        if len(dialog.result_wheels) >= 2:
            wheel_x = [wheel.x for wheel in dialog.result_wheels]
            wheelbase = max(wheel_x) - min(wheel_x)
            if wheelbase > 0:
                self.wheelbase_spin.setValue(wheelbase)
        self.redraw_scene()
        self.statusBar().showMessage(
            f"Placed {len(dialog.result_wheels)} wheels on block '{block_name}'. Save Vehicle to keep them."
        )

    def _update_position_label(self) -> None:
        start = (
            f"Start: {self.start_pose.x:.3f}, {self.start_pose.y:.3f} "
            f"@ {self.start_pose.heading_deg:.1f}°"
        )
        end = "End: not placed"
        if self.end_pose is not None:
            end = f"End: {self.end_pose.x:.3f}, {self.end_pose.y:.3f} @ {self.end_pose.heading_deg:.1f}°"
        saved = f"Saved paths: {len(self.saved_routes)}"
        self.position_label.setText(f"{start}    {end}    {saved}")

    def advance_vehicle(self) -> None:
        if abs(self.speed) < 0.001 and abs(self.lateral) < 0.001:
            return
        profile = self.form_profile()
        next_pose = step_pose(self.poses[-1], profile, self.steering, self.speed * 0.08, self.lateral * 0.08)
        self.poses.append(next_pose)
        self.redraw_dynamic_layers(profile)

    def update_steer_from_slider(self, value: int) -> None:
        profile = self.form_profile()
        self.steering = profile.max_steering_angle_deg * value / 100.0
        self.steer_value_label.setText(f"Steer: {self.steering:.1f}°")
        self.redraw_dynamic_layers(profile)

    def update_speed_from_slider(self, value: int) -> None:
        profile = self.form_profile()
        self.speed = profile.length * value / 100.0
        if value > 0:
            self.travel_direction = 1
        elif value < 0:
            self.travel_direction = -1
        direction = "Forward" if self.travel_direction > 0 else "Reverse"
        direction_color = "#16a34a" if self.travel_direction > 0 else "#dc2626"
        stopped = " (stopped)" if value == 0 else ""
        self.direction_value_label.setText(f"Direction of travel: {direction}{stopped}")
        self.direction_value_label.setStyleSheet(f"color: {direction_color}; font-weight: 700;")
        self.speed_value_label.setText(
            "Speed: stopped" if value == 0 else f"Speed: {self.speed:.3f} drawing units/s"
        )
        self.redraw_dynamic_layers(profile)

    def bump_steer(self, amount: int) -> None:
        self.steer_slider.setValue(max(-100, min(100, self.steer_slider.value() + amount)))

    def change_vehicle(self, index: int) -> None:
        if 0 <= index < len(self.vehicles):
            self.current_profile = self.vehicles[index]
            self._load_profile_to_form(self.current_profile)
            self.redraw_scene()

    def save_vehicle(self) -> None:
        profile = self.form_profile()
        self.store.upsert(profile)
        self.vehicles = self.store.vehicles
        self.vehicle_combo.blockSignals(True)
        self.vehicle_combo.clear()
        self.vehicle_combo.addItems([vehicle.name for vehicle in self.vehicles])
        self.vehicle_combo.setCurrentText(profile.name)
        self.vehicle_combo.blockSignals(False)
        self.current_profile = profile
        self.statusBar().showMessage(f"Saved vehicle profile '{profile.name}' to vehicles.json")
        self.redraw_scene()

    def add_wheel_row(self) -> None:
        row = self.wheel_table.rowCount()
        self.wheel_table.insertRow(row)
        defaults = ["Wheel", "0.0", "0.0", "0.18", "yes", "yes"]
        for column, value in enumerate(defaults):
            self.wheel_table.setItem(row, column, QTableWidgetItem(value))

    def remove_wheel_row(self) -> None:
        row = self.wheel_table.currentRow()
        if row >= 0:
            self.wheel_table.removeRow(row)

    def _set_wheel_table(self, wheels: list[WheelSpec]) -> None:
        self.wheel_table.setRowCount(0)
        for wheel in wheels:
            row = self.wheel_table.rowCount()
            self.wheel_table.insertRow(row)
            values = [
                wheel.name,
                f"{wheel.x:.3f}",
                f"{wheel.y:.3f}",
                f"{wheel.radius:.3f}",
                "yes" if wheel.steerable else "no",
                "yes" if wheel.drive else "no",
            ]
            for column, value in enumerate(values):
                self.wheel_table.setItem(row, column, QTableWidgetItem(value))

    def form_profile(self) -> VehicleProfile:
        wheels: list[WheelSpec] = []
        for row in range(self.wheel_table.rowCount()):
            cells = [self.wheel_table.item(row, col).text() if self.wheel_table.item(row, col) else "" for col in range(6)]
            try:
                wheels.append(
                    WheelSpec(
                        cells[0] or f"Wheel {row + 1}",
                        float(cells[1] or 0),
                        float(cells[2] or 0),
                        float(cells[3] or 0.18),
                        steerable=cells[4].strip().lower() in {"yes", "true", "1", "y"},
                        drive=cells[5].strip().lower() in {"yes", "true", "1", "y"},
                    )
                )
            except ValueError:
                continue
        return VehicleProfile(
            name=self.name_edit.text().strip() or "Vehicle",
            length=self.length_spin.value(),
            width=self.width_spin.value(),
            wheelbase=self.wheelbase_spin.value(),
            steering_mode=SteeringMode(self.steering_mode_combo.currentData()),
            max_steering_angle_deg=self.max_steer_spin.value(),
            min_turning_radius=self.min_radius_spin.value(),
            pose_spacing=self.pose_spacing_spin.value(),
            dxf_block_name=self.block_combo.currentText().strip(),
            block_forward_angle_deg=self.block_forward_spin.value(),
            payload_enabled=self.payload_enabled_checkbox.isChecked(),
            payload_x=self.payload_x_spin.value(),
            payload_y=self.payload_y_spin.value(),
            payload_length=self.payload_length_spin.value(),
            payload_width=self.payload_width_spin.value(),
            payload_rotation_deg=self.payload_rotation_spin.value(),
            wheels=wheels,
        )

    def _load_profile_to_form(self, profile: VehicleProfile) -> None:
        self.name_edit.setText(profile.name)
        self.length_spin.setValue(profile.length)
        self.width_spin.setValue(profile.width)
        self.wheelbase_spin.setValue(profile.wheelbase)
        self.max_steer_spin.setValue(profile.max_steering_angle_deg)
        self.min_radius_spin.setValue(profile.min_turning_radius)
        self.pose_spacing_spin.setValue(profile.pose_spacing)
        self.block_forward_spin.setValue(profile.block_forward_angle_deg)
        self.payload_enabled_checkbox.setChecked(profile.payload_enabled)
        self.payload_x_spin.setValue(profile.payload_x)
        self.payload_y_spin.setValue(profile.payload_y)
        self.payload_length_spin.setValue(profile.payload_length)
        self.payload_width_spin.setValue(profile.payload_width)
        self.payload_rotation_spin.setValue(profile.payload_rotation_deg)
        mode_index = self.steering_mode_combo.findData(profile.steering_mode.value)
        self.steering_mode_combo.setCurrentIndex(max(0, mode_index))
        if self.block_combo.findText(profile.dxf_block_name) < 0:
            self.block_combo.addItem(profile.dxf_block_name)
        self.block_combo.setCurrentText(profile.dxf_block_name)
        self._set_wheel_table(profile.wheels)

    def redraw_scene(self) -> None:
        self.stop_route_animation()
        self.scene.clear()
        self.vehicle_items.clear()
        self.path_item = None
        self.sweep_items.clear()
        self.indicative_path_item = None
        self.planned_sweep_items.clear()
        self.planned_block_trace_items.clear()
        self.route_failure_items.clear()
        self.saved_route_items.clear()
        self.payload_trace_items.clear()
        self.position_items.clear()
        self.route_point_items.clear()
        self.route_animation_item = None
        if self.current_dxf:
            self.draw_dxf(self.current_dxf)
        self.redraw_dynamic_layers(self.form_profile())
        self.redraw_position_markers()
        self.redraw_route_handles()
        self._update_navigation_bounds()
        self.fit_drawing()

    def draw_dxf(self, drawing: DxfDrawing) -> None:
        pen = QPen(QColor("#9aa7b8"), 0)
        _add_primitives_to_scene(self.scene, drawing.primitives, pen)
        if drawing.bounds is not None:
            min_x, min_y, max_x, max_y = drawing.bounds
            origin_size = max(max_x - min_x, max_y - min_y, 1.0) / 200.0
            origin_pen = QPen(QColor("#16a34a"), 0)
            horizontal = self.scene.addLine(-origin_size, 0.0, origin_size, 0.0, origin_pen)
            vertical = self.scene.addLine(0.0, -origin_size, 0.0, origin_size, origin_pen)
            horizontal.setToolTip("Imported DXF origin (0, 0)")
            vertical.setToolTip("Imported DXF origin (0, 0)")

    def _drawing_scene_rect(self) -> QRectF | None:
        if not self.current_dxf or self.current_dxf.bounds is None:
            return None
        min_x, min_y, max_x, max_y = self.current_dxf.bounds
        rect = QRectF(min_x, -max_y, max_x - min_x, max_y - min_y).normalized()
        if rect.width() <= 0:
            rect.setWidth(1.0)
        if rect.height() <= 0:
            rect.setHeight(1.0)
        return rect

    def _update_navigation_bounds(self) -> None:
        content = self.scene.itemsBoundingRect()
        drawing_rect = self._drawing_scene_rect()
        if drawing_rect is not None:
            content = content.united(drawing_rect) if content.isValid() else drawing_rect
        if not content.isValid():
            content = QRectF(-50.0, -50.0, 100.0, 100.0)
        margin = max(content.width(), content.height(), 10.0)
        self.scene.setSceneRect(content.adjusted(-margin, -margin, margin, margin))

    def fit_drawing(self) -> None:
        rect = self._drawing_scene_rect()
        if rect is None:
            rect = self.scene.itemsBoundingRect()
        if not rect.isValid() or rect.isEmpty():
            return
        padding = max(rect.width(), rect.height()) * 0.025
        self.view.fitInView(rect.adjusted(-padding, -padding, padding, padding), Qt.AspectRatioMode.KeepAspectRatio)

    def redraw_position_markers(self) -> None:
        for item in self.position_items:
            if item.scene() is self.scene:
                self.scene.removeItem(item)
        self.position_items.clear()

        marker_size = max(self.form_profile().width * 0.3, 0.2)
        drawing_rect = self._drawing_scene_rect()
        if drawing_rect is not None:
            marker_size = max(marker_size, max(drawing_rect.width(), drawing_rect.height()) / 300.0)

        self.position_items.extend(
            self._add_position_marker(
                self.start_pose.x,
                self.start_pose.y,
                self.start_pose.heading_deg,
                marker_size,
                QColor("#16a34a"),
                False,
            )
        )
        if self.end_pose is not None:
            self.position_items.extend(
                self._add_position_marker(
                    self.end_pose.x,
                    self.end_pose.y,
                    self.end_pose.heading_deg,
                    marker_size,
                    QColor("#dc2626"),
                    True,
                )
            )
        saved_pen = QPen(QColor("#2563eb"), 0)
        for index, route in enumerate(self.saved_routes):
            if index == self.active_route_index:
                continue
            pose = route.end_pose
            marker = self.scene.addEllipse(
                pose.x - marker_size * 0.45,
                -pose.y - marker_size * 0.45,
                marker_size * 0.9,
                marker_size * 0.9,
                saved_pen,
            )
            heading = radians(pose.heading_deg)
            direction = self.scene.addLine(
                pose.x,
                -pose.y,
                pose.x + marker_size * 1.5 * cos(heading),
                -pose.y - marker_size * 1.5 * sin(heading),
                saved_pen,
            )
            marker.setToolTip(f"Saved endpoint: {route.name}")
            direction.setToolTip(f"Saved endpoint orientation: {route.name}")
            self.position_items.extend([marker, direction])

    def _add_position_marker(
        self, x: float, y: float, heading_deg: float, size: float, color: QColor, end_marker: bool
    ) -> list:
        kind = "end" if end_marker else "start"
        handle = PoseHandleItem(
            kind,
            x,
            -y,
            heading_deg,
            size,
            color,
            end_marker,
            self._pose_handle_moved,
            self._pose_handle_released,
        )
        self.scene.addItem(handle)
        return [handle]

    def _pose_handle_moved(self, kind: str, scene_position: QPointF) -> None:
        self.stop_route_animation()
        x = float(scene_position.x())
        y = float(-scene_position.y())
        if kind == "start":
            if self.timer.isActive():
                self.timer.stop()
                self.run_action.setText("Run")
                self.run_action.setIcon(line_icon("play", "#ffffff"))
            self.start_pose = Pose(x, y, self.start_pose.heading_deg, 0.0)
            self.poses = [self.start_pose]
            self.speed = 0.0
            self.speed_slider.setValue(0)
        elif self.end_pose is not None:
            self.end_pose.x = x
            self.end_pose.y = y
        self._update_position_label()
        self.redraw_dynamic_layers(self.form_profile())

    def _pose_handle_released(self, kind: str, scene_position: QPointF) -> None:
        self._pose_handle_moved(kind, scene_position)
        if kind == "start" and self.saved_routes:
            self._persist_routes()
        self._update_navigation_bounds()
        pose = self.start_pose if kind == "start" else self.end_pose
        if pose is not None:
            self.statusBar().showMessage(
                f"{kind.title()} moved to X {pose.x:.3f}, Y {pose.y:.3f}, heading {pose.heading_deg:.1f}°"
            )

    def redraw_route_handles(self) -> None:
        for item in self.route_point_items:
            if item.scene() is self.scene:
                self.scene.removeItem(item)
        self.route_point_items.clear()
        if not self.show_route_checkbox.isChecked() or self.end_pose is None:
            return
        profile = self.form_profile()
        size = max(profile.width * 0.10, profile.length * 0.035, 0.08)
        drawing_rect = self._drawing_scene_rect()
        if drawing_rect is not None:
            size = max(size, max(drawing_rect.width(), drawing_rect.height()) / 700.0)
        for index, (x, y) in enumerate(self.route_waypoints):
            item = RoutePointHandleItem(
                index,
                x,
                -y,
                size,
                self._route_point_moved,
                self._route_point_released,
            )
            self.scene.addItem(item)
            self.route_point_items.append(item)

    def _route_point_moved(self, index: int, scene_position: QPointF) -> None:
        if not 0 <= index < len(self.route_waypoints):
            return
        self.stop_route_animation()
        self.route_waypoints[index] = (float(scene_position.x()), float(-scene_position.y()))
        self.redraw_dynamic_layers(self.form_profile())

    def _route_point_released(self, index: int, scene_position: QPointF) -> None:
        self._route_point_moved(index, scene_position)
        self._update_navigation_bounds()
        x, y = self.route_waypoints[index]
        self.statusBar().showMessage(f"Route point {index + 1} moved to X {x:.3f}, Y {y:.3f}")

    def remove_selected_route_point(self) -> None:
        selected = next((item for item in self.route_point_items if item.isSelected()), None)
        if selected is None:
            QMessageBox.information(self, "Select a route point", "Select an orange route point to remove.")
            return
        self.stop_route_animation()
        del self.route_waypoints[selected.index]
        self.redraw_dynamic_layers(self.form_profile())
        self.redraw_route_handles()
        self.statusBar().showMessage("Removed the selected route point.")

    def clear_route_points(self) -> None:
        if not self.route_waypoints:
            return
        self.stop_route_animation()
        self.route_waypoints.clear()
        self.redraw_dynamic_layers(self.form_profile())
        self.redraw_route_handles()
        self.statusBar().showMessage("Cleared all route control points.")

    def _route_store_path(self) -> Path | None:
        return self.current_dxf.path if self.current_dxf else None

    def _persist_routes(self) -> None:
        self.route_store.save(self._route_store_path(), self.start_pose, self.saved_routes)

    def _next_route_name(self) -> str:
        used = {route.name for route in self.saved_routes}
        number = 1
        while f"Path {number}" in used:
            number += 1
        return f"Path {number}"

    def _refresh_route_combo(self, selected_index: int | None = None) -> None:
        if not hasattr(self, "route_combo"):
            return
        self._updating_route_combo = True
        self.route_combo.clear()
        self.route_combo.addItem("New unsaved path", None)
        for index, route in enumerate(self.saved_routes):
            self.route_combo.addItem(route.name, index)
        active = self.active_route_index if selected_index is None else selected_index
        self.route_combo.setCurrentIndex(0 if active is None else active + 1)
        self._updating_route_combo = False
        if self.active_route_index is None and not self.route_name_edit.text():
            self.route_name_edit.setText(self._next_route_name())

    def save_current_route(self) -> None:
        if self.end_pose is None:
            QMessageBox.information(self, "Place an endpoint", "Place an endpoint before saving the path.")
            return
        name = self.route_name_edit.text().strip() or self._next_route_name()
        route = RoutePlan(
            name,
            Pose(self.end_pose.x, self.end_pose.y, self.end_pose.heading_deg, 0.0),
            list(self.route_waypoints),
        )
        if self.active_route_index is None:
            self.saved_routes.append(route)
            self.active_route_index = len(self.saved_routes) - 1
        else:
            self.saved_routes[self.active_route_index] = route
        self._persist_routes()
        self._refresh_route_combo(self.active_route_index)
        self._redraw_route_layers()
        self.statusBar().showMessage(f"Saved '{name}' with {len(route.waypoints)} route point(s).")

    def new_route(self) -> None:
        self.stop_route_animation()
        self.active_route_index = None
        self.end_pose = None
        self.route_waypoints.clear()
        self.end_heading_spin.blockSignals(True)
        self.end_heading_spin.setValue(0.0)
        self.end_heading_spin.blockSignals(False)
        self.route_name_edit.setText(self._next_route_name())
        self._refresh_route_combo()
        self._update_position_label()
        self._redraw_route_layers()
        self.statusBar().showMessage("New path ready; place another endpoint from the shared start position.")

    def change_saved_route(self, combo_index: int) -> None:
        if self._updating_route_combo:
            return
        route_index = self.route_combo.itemData(combo_index)
        if route_index is None:
            self.new_route()
            return
        route_index = int(route_index)
        if not 0 <= route_index < len(self.saved_routes):
            return
        self.stop_route_animation()
        self.active_route_index = route_index
        route = self.saved_routes[route_index]
        self.end_pose = Pose(route.end_pose.x, route.end_pose.y, route.end_pose.heading_deg, 0.0)
        self.route_waypoints = list(route.waypoints)
        self.route_name_edit.setText(route.name)
        self.end_heading_spin.blockSignals(True)
        self.end_heading_spin.setValue(route.end_pose.heading_deg)
        self.end_heading_spin.blockSignals(False)
        self._update_position_label()
        self._redraw_route_layers()
        self.statusBar().showMessage(f"Editing saved path '{route.name}'.")

    def remove_saved_route(self) -> None:
        if self.active_route_index is None:
            QMessageBox.information(self, "Select a saved path", "Select the saved path to remove.")
            return
        name = self.saved_routes[self.active_route_index].name
        del self.saved_routes[self.active_route_index]
        self._persist_routes()
        self.active_route_index = None
        self.end_pose = None
        self.route_waypoints.clear()
        self.route_name_edit.setText(self._next_route_name())
        self._refresh_route_combo()
        self._update_position_label()
        self._redraw_route_layers()
        self.statusBar().showMessage(f"Removed saved path '{name}'.")

    def _redraw_route_layers(self) -> None:
        profile = self.form_profile()
        self.redraw_dynamic_layers(profile)
        self.redraw_position_markers()
        self.redraw_route_handles()
        self._update_navigation_bounds()

    def _spaced_endpoint(
        self,
        x: float,
        y: float,
        heading_deg: float,
        profile: VehicleProfile,
    ) -> tuple[float, float, str]:
        mode = self.endpoint_spacing_mode_combo.currentData()
        candidates = [
            route.end_pose
            for index, route in enumerate(self.saved_routes)
            if index != self.active_route_index
        ]
        if mode == "freehand" or not candidates:
            return x, y, ""

        gap = self.endpoint_spacing_spin.value()
        target = Pose(x, y, heading_deg, 0.0)
        if mode == "payload":
            target_center = target.transformed_point(profile.payload_x, profile.payload_y)
            centers = [pose.transformed_point(profile.payload_x, profile.payload_y) for pose in candidates]
            length, width = profile.payload_length, profile.payload_width
            rotation_offset = profile.payload_rotation_deg
        else:
            target_center = (x, y)
            centers = [(pose.x, pose.y) for pose in candidates]
            length, width = profile.length, profile.width
            rotation_offset = 0.0

        nearest_index = min(
            range(len(candidates)),
            key=lambda index: hypot(target_center[0] - centers[index][0], target_center[1] - centers[index][1]),
        )
        reference = candidates[nearest_index]
        reference_center = centers[nearest_index]
        dx = target_center[0] - reference_center[0]
        dy = target_center[1] - reference_center[1]
        distance = hypot(dx, dy)
        if distance < 1e-9:
            direction = radians(reference.heading_deg + 90.0)
            ux, uy = cos(direction), sin(direction)
        else:
            ux, uy = dx / distance, dy / distance
            direction = atan2(uy, ux)

        def extent(pose_heading: float) -> float:
            relative = radians(pose_heading + rotation_offset) - direction
            return 0.5 * (abs(cos(relative)) * length + abs(sin(relative)) * width)

        center_distance = extent(reference.heading_deg) + extent(heading_deg) + gap
        snapped_center = (
            reference_center[0] + ux * center_distance,
            reference_center[1] + uy * center_distance,
        )
        if mode == "payload":
            offset_world = Pose(0.0, 0.0, heading_deg).transformed_point(profile.payload_x, profile.payload_y)
            x = snapped_center[0] - offset_world[0]
            y = snapped_center[1] - offset_world[1]
        else:
            x, y = snapped_center
        return x, y, f"; snapped to {gap:.3f} {mode} clearance"

    def redraw_dynamic_layers(self, profile: VehicleProfile) -> None:
        for item in self.vehicle_items:
            self.scene.removeItem(item)
        self.vehicle_items.clear()
        if self.path_item:
            self.scene.removeItem(self.path_item)
            self.path_item = None
        for item in self.sweep_items:
            self.scene.removeItem(item)
        self.sweep_items.clear()
        if self.indicative_path_item is not None:
            self.scene.removeItem(self.indicative_path_item)
            self.indicative_path_item = None
        for item in self.planned_sweep_items:
            self.scene.removeItem(item)
        self.planned_sweep_items.clear()
        for item in self.planned_block_trace_items:
            self.scene.removeItem(item)
        self.planned_block_trace_items.clear()
        for item in self.route_failure_items:
            self.scene.removeItem(item)
        self.route_failure_items.clear()
        for item in self.saved_route_items:
            self.scene.removeItem(item)
        self.saved_route_items.clear()
        for item in self.payload_trace_items:
            self.scene.removeItem(item)
        self.payload_trace_items.clear()

        if len(self.poses) >= 2:
            display_poses = self._display_poses()
            path = QPainterPath(QPointF(display_poses[0].x, -display_poses[0].y))
            for pose in display_poses[1:]:
                path.lineTo(pose.x, -pose.y)
            self.path_item = self.scene.addPath(path, QPen(QColor("#16a34a"), 0.05))

            for side in (-1, 1):
                sweep = QPainterPath()
                first = True
                for pose in display_poses:
                    heading = radians(pose.heading_deg)
                    x = pose.x + side * profile.width / 2.0 * sin(heading)
                    y = pose.y - side * profile.width / 2.0 * cos(heading)
                    if first:
                        sweep.moveTo(x, -y)
                        first = False
                    else:
                        sweep.lineTo(x, -y)
                self.sweep_items.append(self.scene.addPath(sweep, QPen(QColor("#dc2626"), 0.04)))
            if profile.payload_enabled:
                self.draw_payload_traces(display_poses, profile, planned=False)

        self.redraw_indicative_path(profile)
        for pose in self._sample_poses(profile):
            self.vehicle_items.append(self.draw_vehicle(profile, pose, ghost=True))
        if self.end_pose is not None:
            self.vehicle_items.append(self.draw_vehicle(profile, self.end_pose, ghost=True, detailed=True))
        for index, route in enumerate(self.saved_routes):
            if index == self.active_route_index:
                continue
            self.vehicle_items.append(self.draw_vehicle(profile, route.end_pose, ghost=True, detailed=True))
        self.vehicle_items.append(self.draw_vehicle(profile, self.poses[-1], ghost=False))

    def redraw_indicative_path(self, profile: VehicleProfile) -> None:
        if not self.show_route_checkbox.isChecked():
            self.route_feasibility_label.setText("Route check: hidden")
            self.route_feasibility_label.setStyleSheet("color: #667085; font-weight: 700;")
            return
        inactive_routes = [
            (route.name, self._planned_route_poses_for(route.end_pose, route.waypoints))
            for index, route in enumerate(self.saved_routes)
            if index != self.active_route_index
        ]
        saved_pen = QPen(QColor("#2563eb"), 0)
        saved_pen.setStyle(Qt.PenStyle.DashLine)
        for name, saved_poses in inactive_routes:
            if len(saved_poses) < 2:
                continue
            saved_path = QPainterPath(QPointF(saved_poses[0].x, -saved_poses[0].y))
            for pose in saved_poses[1:]:
                saved_path.lineTo(pose.x, -pose.y)
            item = self.scene.addPath(saved_path, saved_pen)
            item.setZValue(1.8)
            item.setToolTip(f"Saved path: {name}")
            self.saved_route_items.append(item)
        if self.end_pose is None and inactive_routes:
            self.route_feasibility_label.setText(
                f"{len(inactive_routes)} saved path(s); select one to edit or create a new path"
            )
            self.route_feasibility_label.setStyleSheet("color: #2563eb; font-weight: 700;")
            return
        current = self.poses[-1]
        route_poses: list[Pose] = []
        if self.end_pose is not None:
            route_poses = self.planned_route_poses(profile)
        else:
            total_distance = max(profile.length * 4.0, profile.min_turning_radius * 2.0)
            step_distance = total_distance / 60.0
            projected = Pose(current.x, current.y, current.heading_deg, self.steering)
            route_poses.append(projected)
            for _ in range(60):
                projected = step_pose(projected, profile, self.steering, step_distance)
                route_poses.append(projected)

        if len(route_poses) < 2:
            self.route_feasibility_label.setText("Route check: place a finish position")
            self.route_feasibility_label.setStyleSheet("color: #667085; font-weight: 700;")
            return
        path = QPainterPath(QPointF(route_poses[0].x, -route_poses[0].y))
        for pose in route_poses[1:]:
            path.lineTo(pose.x, -pose.y)
        pen = QPen(QColor("#d97706"), 0)
        pen.setStyle(Qt.PenStyle.DashLine)
        self.indicative_path_item = self.scene.addPath(path, pen)
        self.indicative_path_item.setZValue(2.0)
        tooltip = "Planned route from start to finish" if self.end_pose is not None else "Projected steering path"
        self.indicative_path_item.setToolTip(tooltip)

        if self.end_pose is not None:
            self.draw_route_failures(route_poses, profile)
            sweep_color = QColor("#f59e0b")
            sweep_color.setAlpha(150)
            sweep_pen = QPen(sweep_color, 0)
            sweep_pen.setStyle(Qt.PenStyle.DotLine)
            for side in (-1.0, 1.0):
                sweep = QPainterPath()
                for index, pose in enumerate(route_poses):
                    heading = radians(pose.heading_deg)
                    x = pose.x + side * profile.width / 2.0 * sin(heading)
                    y = pose.y - side * profile.width / 2.0 * cos(heading)
                    if index == 0:
                        sweep.moveTo(x, -y)
                    else:
                        sweep.lineTo(x, -y)
                item = self.scene.addPath(sweep, sweep_pen)
                item.setZValue(1.5)
                item.setToolTip("Planned vehicle swept envelope")
                self.planned_sweep_items.append(item)
            self.draw_planned_block_outline(route_poses, profile)
            if profile.payload_enabled:
                self.draw_payload_traces(route_poses, profile, planned=True)
        else:
            self.route_feasibility_label.setText("Route check: projected steering path")
            self.route_feasibility_label.setStyleSheet("color: #667085; font-weight: 700;")

    def draw_route_failures(self, route_poses: list[Pose], profile: VehicleProfile) -> None:
        limit = profile.max_turn_curvature
        invalid: list[bool] = []
        required_curvatures: list[float] = []
        for first, second in zip(route_poses, route_poses[1:]):
            distance = hypot(second.x - first.x, second.y - first.y)
            heading_change = ((second.heading_deg - first.heading_deg + 180.0) % 360.0) - 180.0
            curvature = abs(radians(heading_change)) / max(distance, 1e-9)
            required_curvatures.append(curvature)
            invalid.append(curvature > limit * 1.02)

        worst_curvature = max(required_curvatures, default=0.0)
        available_radius = 1.0 / max(limit, 1e-12)
        if not any(invalid):
            required_radius = 1.0 / max(worst_curvature, 1e-12)
            radius_text = "straight" if worst_curvature < 1e-9 else f"minimum required radius {required_radius:.3f}"
            self.route_feasibility_label.setText(f"Route feasible — {radius_text}")
            self.route_feasibility_label.setStyleSheet("color: #16a34a; font-weight: 700;")
            return

        failure_path = QPainterPath()
        drawing_failure = False
        for index, failed in enumerate(invalid):
            if failed:
                if not drawing_failure:
                    failure_path.moveTo(route_poses[index].x, -route_poses[index].y)
                failure_path.lineTo(route_poses[index + 1].x, -route_poses[index + 1].y)
                drawing_failure = True
            else:
                drawing_failure = False
        pen = QPen(QColor("#dc2626"), 2.5)
        pen.setCosmetic(True)
        item = self.scene.addPath(failure_path, pen)
        item.setZValue(6.0)
        item.setToolTip("Route section exceeds the configured steering curvature")
        self.route_failure_items.append(item)
        invalid_count = sum(invalid)
        self.route_feasibility_label.setText(
            f"Route impossible in {invalid_count} section(s) — available minimum radius {available_radius:.3f}"
        )
        self.route_feasibility_label.setStyleSheet("color: #dc2626; font-weight: 700;")

    def block_outline_points(self, profile: VehicleProfile) -> list[tuple[float, float]]:
        if self.current_dxf is not None and profile.dxf_block_name:
            geometry = get_block_geometry(self.current_dxf, profile.dxf_block_name)
            if geometry is not None:
                angle = radians(profile.block_forward_angle_deg)
                oriented = [
                    (
                        x * cos(angle) + y * sin(angle),
                        -x * sin(angle) + y * cos(angle),
                    )
                    for primitive in geometry.primitives
                    for x, y in primitive.points
                ]
                if oriented:
                    xs = [point[0] for point in oriented]
                    ys = [point[1] for point in oriented]
                    min_x, max_x = min(xs), max(xs)
                    min_y, max_y = min(ys), max(ys)
                    return [(min_x, min_y), (max_x, min_y), (max_x, max_y), (min_x, max_y)]
        half_length = profile.length / 2.0
        half_width = profile.width / 2.0
        return [
            (-half_length, -half_width),
            (half_length, -half_width),
            (half_length, half_width),
            (-half_length, half_width),
        ]

    def draw_planned_block_outline(self, route_poses: list[Pose], profile: VehicleProfile) -> None:
        outline = self.block_outline_points(profile)
        if len(outline) < 3 or len(route_poses) < 2:
            return
        color = QColor("#a21caf")
        trace_color = QColor(color)
        trace_color.setAlpha(175)
        trace_pen = QPen(trace_color, 0)
        trace_pen.setStyle(Qt.PenStyle.DotLine)
        for local_x, local_y in outline:
            first = route_poses[0].transformed_point(local_x, local_y)
            path = QPainterPath(QPointF(first[0], -first[1]))
            for pose in route_poses[1:]:
                x, y = pose.transformed_point(local_x, local_y)
                path.lineTo(x, -y)
            item = self.scene.addPath(path, trace_pen)
            item.setZValue(1.75)
            item.setToolTip("Selected block extremity trace")
            self.planned_block_trace_items.append(item)

        outline_pen = QPen(color, 0)
        for pose, label in ((route_poses[0], "start"), (route_poses[-1], "finish")):
            world = [pose.transformed_point(x, y) for x, y in outline]
            path = QPainterPath(QPointF(world[0][0], -world[0][1]))
            for x, y in world[1:] + [world[0]]:
                path.lineTo(x, -y)
            item = self.scene.addPath(path, outline_pen)
            item.setZValue(3.0)
            item.setToolTip(f"Selected block {label} outline")
            self.planned_block_trace_items.append(item)
            marker_size = max(profile.length, profile.width) / 80.0
            for x, y in world:
                marker = self.scene.addEllipse(
                    x - marker_size,
                    -y - marker_size,
                    marker_size * 2.0,
                    marker_size * 2.0,
                    outline_pen,
                    QBrush(color),
                )
                marker.setZValue(3.5)
                marker.setToolTip(f"Block {label} extremity")
                self.planned_block_trace_items.append(marker)

    def draw_payload_traces(
        self, poses: list[Pose], profile: VehicleProfile, planned: bool
    ) -> None:
        if not profile.payload_enabled or len(poses) < 2:
            return
        outline = payload_outline_points(profile)
        color = QColor("#0891b2") if planned else QColor("#0f766e")
        center_pen = QPen(color, 2.0)
        center_pen.setCosmetic(True)
        center_pen.setStyle(Qt.PenStyle.DashLine if planned else Qt.PenStyle.SolidLine)
        centers = [pose.transformed_point(profile.payload_x, profile.payload_y) for pose in poses]
        center_path = QPainterPath(QPointF(centers[0][0], -centers[0][1]))
        for x, y in centers[1:]:
            center_path.lineTo(x, -y)
        center_item = self.scene.addPath(center_path, center_pen)
        center_item.setZValue(4.0)
        center_item.setToolTip("Planned payload centre trace" if planned else "Driven payload centre trace")
        self.payload_trace_items.append(center_item)

        edge_pen = QPen(color, 0)
        edge_pen.setStyle(Qt.PenStyle.DotLine)
        for local_x, local_y in outline:
            first = poses[0].transformed_point(local_x, local_y)
            path = QPainterPath(QPointF(first[0], -first[1]))
            for pose in poses[1:]:
                x, y = pose.transformed_point(local_x, local_y)
                path.lineTo(x, -y)
            item = self.scene.addPath(path, edge_pen)
            item.setZValue(3.75)
            item.setToolTip("Payload extremity trace")
            self.payload_trace_items.append(item)

        outline_pen = QPen(color, 0)
        for pose in (poses[0], poses[-1]):
            world = [pose.transformed_point(x, y) for x, y in outline]
            path = QPainterPath(QPointF(world[0][0], -world[0][1]))
            for x, y in world[1:] + [world[0]]:
                path.lineTo(x, -y)
            item = self.scene.addPath(path, outline_pen)
            item.setZValue(4.25)
            item.setToolTip("Payload footprint")
            self.payload_trace_items.append(item)

    def planned_route_poses(self, _profile: VehicleProfile) -> list[Pose]:
        if self.end_pose is None:
            return []
        return self._planned_route_poses_for(self.end_pose, self.route_waypoints)

    def all_planned_route_poses(self, _profile: VehicleProfile) -> list[list[Pose]]:
        routes: list[list[Pose]] = []
        if self.end_pose is not None:
            active = self._planned_route_poses_for(self.end_pose, self.route_waypoints)
            if active:
                routes.append(active)
        for index, route in enumerate(self.saved_routes):
            if index == self.active_route_index:
                continue
            poses = self._planned_route_poses_for(route.end_pose, route.waypoints)
            if poses:
                routes.append(poses)
        return routes

    def _planned_route_poses_for(
        self,
        end: Pose,
        waypoints: list[tuple[float, float]],
    ) -> list[Pose]:
        start = self.start_pose
        nodes = [(start.x, start.y), *waypoints, (end.x, end.y)]
        if all(hypot(b[0] - a[0], b[1] - a[1]) < 1e-9 for a, b in zip(nodes, nodes[1:])):
            return []
        start_heading = radians(start.heading_deg)
        end_heading = radians(end.heading_deg)
        first_distance = hypot(nodes[1][0] - nodes[0][0], nodes[1][1] - nodes[0][1])
        last_distance = hypot(nodes[-1][0] - nodes[-2][0], nodes[-1][1] - nodes[-2][1])
        tangents: list[tuple[float, float]] = [
            (cos(start_heading) * first_distance * 1.2, sin(start_heading) * first_distance * 1.2)
        ]
        for index in range(1, len(nodes) - 1):
            tangents.append(
                (
                    (nodes[index + 1][0] - nodes[index - 1][0]) * 0.5,
                    (nodes[index + 1][1] - nodes[index - 1][1]) * 0.5,
                )
            )
        tangents.append(
            (cos(end_heading) * last_distance * 1.2, sin(end_heading) * last_distance * 1.2)
        )
        route: list[Pose] = []
        for segment in range(len(nodes) - 1):
            p0 = nodes[segment]
            p1 = nodes[segment + 1]
            m0 = tangents[segment]
            m1 = tangents[segment + 1]
            for sample in range(41):
                if segment > 0 and sample == 0:
                    continue
                t = sample / 40.0
                t2 = t * t
                t3 = t2 * t
                h00 = 2.0 * t3 - 3.0 * t2 + 1.0
                h10 = t3 - 2.0 * t2 + t
                h01 = -2.0 * t3 + 3.0 * t2
                h11 = t3 - t2
                x = h00 * p0[0] + h10 * m0[0] + h01 * p1[0] + h11 * m1[0]
                y = h00 * p0[1] + h10 * m0[1] + h01 * p1[1] + h11 * m1[1]
                dh00 = 6.0 * t2 - 6.0 * t
                dh10 = 3.0 * t2 - 4.0 * t + 1.0
                dh01 = -6.0 * t2 + 6.0 * t
                dh11 = 3.0 * t2 - 2.0 * t
                dx = dh00 * p0[0] + dh10 * m0[0] + dh01 * p1[0] + dh11 * m1[0]
                dy = dh00 * p0[1] + dh10 * m0[1] + dh01 * p1[1] + dh11 * m1[1]
                fallback = route[-1].heading_deg if route else start.heading_deg
                heading = degrees(atan2(dy, dx)) if hypot(dx, dy) > 1e-9 else fallback
                route.append(Pose(x, y, heading, 0.0))
        route[0] = Pose(start.x, start.y, start.heading_deg, 0.0)
        route[-1] = Pose(end.x, end.y, end.heading_deg, 0.0)
        return route

    def _display_poses(self, maximum: int = 4000) -> list[Pose]:
        if len(self.poses) <= maximum:
            return self.poses
        stride = ceil((len(self.poses) - 1) / (maximum - 1))
        poses = self.poses[::stride]
        if poses[-1] is not self.poses[-1]:
            poses.append(self.poses[-1])
        return poses

    def _sample_poses(self, profile: VehicleProfile) -> list[Pose]:
        if len(self.poses) <= 2:
            return []
        samples: deque[Pose] = deque(maxlen=80)
        last_x = self.poses[0].x
        last_y = self.poses[0].y
        distance = 0.0
        for pose in self.poses[1:-1]:
            step = ((pose.x - last_x) ** 2 + (pose.y - last_y) ** 2) ** 0.5
            distance += step
            last_x, last_y = pose.x, pose.y
            if distance >= profile.pose_spacing:
                samples.append(pose)
                distance = 0.0
        return list(samples)

    def draw_vehicle(
        self,
        profile: VehicleProfile,
        pose: Pose,
        ghost: bool,
        detailed: bool = False,
        direction_override: int | None = None,
    ) -> QGraphicsItemGroup:
        group = QGraphicsItemGroup()
        block_geometry = None
        if (not ghost or detailed) and self.current_dxf is not None and profile.dxf_block_name:
            block_geometry = get_block_geometry(self.current_dxf, profile.dxf_block_name)
        corners = [QPointF(x, -y) for x, y in vehicle_corners(profile, pose)]
        fill = QColor(37, 99, 235, 20 if block_geometry is not None else (60 if ghost else 180))
        outline = QColor("#2563eb") if not ghost else QColor(37, 99, 235, 110)
        body = QGraphicsPolygonItem(QPolygonF(corners))
        body.setBrush(QBrush(fill))
        body.setPen(QPen(outline, 0.04))
        group.addToGroup(body)

        if block_geometry is not None:
            cache_key = (id(self.current_dxf), profile.dxf_block_name)
            block_path = self._block_path_cache.get(cache_key)
            if block_path is None:
                block_path = _primitives_to_path(block_geometry.primitives)
                self._block_path_cache[cache_key] = block_path
            block_item = QGraphicsPathItem(block_path)
            block_item.setPen(QPen(QColor(37, 99, 235, 110 if ghost else 255), 0))
            block_item.setPos(pose.x, -pose.y)
            block_item.setRotation(-(pose.heading_deg - profile.block_forward_angle_deg))
            group.addToGroup(block_item)

        if profile.payload_enabled and (not ghost or detailed):
            payload_world = [
                pose.transformed_point(local_x, local_y)
                for local_x, local_y in payload_outline_points(profile)
            ]
            payload_item = QGraphicsPolygonItem(
                QPolygonF([QPointF(x, -y) for x, y in payload_world])
            )
            payload_color = QColor("#06b6d4")
            payload_fill = QColor(payload_color)
            payload_fill.setAlpha(45 if ghost else 80)
            payload_pen = QPen(QColor("#0e7490"), 2.0)
            payload_pen.setCosmetic(True)
            payload_pen.setStyle(Qt.PenStyle.DashLine)
            payload_item.setPen(payload_pen)
            payload_item.setBrush(QBrush(payload_fill))
            payload_item.setToolTip("Tracked payload oriented bounding box")
            group.addToGroup(payload_item)
            payload_center = pose.transformed_point(profile.payload_x, profile.payload_y)
            marker_size = max(profile.payload_length, profile.payload_width) / 10.0
            center_one = QGraphicsLineItem(
                payload_center[0] - marker_size,
                -payload_center[1],
                payload_center[0] + marker_size,
                -payload_center[1],
            )
            center_two = QGraphicsLineItem(
                payload_center[0],
                -payload_center[1] - marker_size,
                payload_center[0],
                -payload_center[1] + marker_size,
            )
            center_one.setPen(QPen(QColor("#0e7490"), 0))
            center_two.setPen(QPen(QColor("#0e7490"), 0))
            group.addToGroup(center_one)
            group.addToGroup(center_two)
            corner_size = max(
                min(profile.payload_length, profile.payload_width) / 14.0,
                max(profile.length, profile.width) / 120.0,
            )
            for corner_x, corner_y in payload_world:
                corner = QGraphicsEllipseItem(
                    corner_x - corner_size,
                    -corner_y - corner_size,
                    corner_size * 2.0,
                    corner_size * 2.0,
                )
                corner.setPen(QPen(QColor("#0e7490"), 0))
                corner.setBrush(QBrush(QColor("#06b6d4")))
                corner.setToolTip("Payload bounding-box corner")
                group.addToGroup(corner)

        if not ghost or detailed:
            nose = pose.transformed_point(profile.length / 2.0, 0)
            nose_line = QGraphicsLineItem(pose.x, -pose.y, nose[0], -nose[1])
            nose_line.setPen(QPen(QColor("#172033"), 0.035))
            group.addToGroup(nose_line)

        if not ghost:
            active_direction = self.travel_direction if direction_override is None else direction_override
            direction = 1.0 if active_direction > 0 else -1.0
            color = QColor("#16a34a") if direction > 0 else QColor("#dc2626")
            tail = pose.transformed_point(-direction * profile.length * 0.12, 0.0)
            head_base = pose.transformed_point(direction * profile.length * 0.30, 0.0)
            tip = pose.transformed_point(direction * profile.length * 0.55, 0.0)
            wing_one = pose.transformed_point(
                direction * profile.length * 0.30, profile.width * 0.14
            )
            wing_two = pose.transformed_point(
                direction * profile.length * 0.30, -profile.width * 0.14
            )
            shaft = QGraphicsLineItem(tail[0], -tail[1], head_base[0], -head_base[1])
            shaft.setPen(QPen(color, 0))
            group.addToGroup(shaft)
            arrow_head = QGraphicsPolygonItem(
                QPolygonF(
                    [
                        QPointF(tip[0], -tip[1]),
                        QPointF(wing_one[0], -wing_one[1]),
                        QPointF(wing_two[0], -wing_two[1]),
                    ]
                )
            )
            arrow_head.setPen(QPen(color, 0))
            arrow_head.setBrush(QBrush(color))
            arrow_head.setToolTip("Forward" if direction > 0 else "Reverse")
            group.addToGroup(arrow_head)

        for wheel in profile.wheels if (not ghost or detailed) else ():
            wx, wy = pose.transformed_point(wheel.x, wheel.y)
            wheel_item = QGraphicsEllipseItem(wx - wheel.radius, -wy - wheel.radius, wheel.radius * 2, wheel.radius * 2)
            wheel_item.setBrush(QBrush(QColor("#172033") if wheel.drive else QColor("#94a3b8")))
            wheel_item.setPen(QPen(QColor("#0f172a"), 0.02))
            wheel_angle = pose.heading_deg + (pose.steering_deg if wheel.steerable else 0.0)
            transform = QTransform()
            transform.translate(wx, -wy)
            transform.rotate(-wheel_angle)
            transform.translate(-wx, wy)
            wheel_item.setTransform(transform)
            group.addToGroup(wheel_item)
        self.scene.addItem(group)
        return group

    def keyPressEvent(self, event: QKeyEvent) -> None:
        if event.key() == Qt.Key.Key_W:
            self.speed_slider.setValue(min(100, self.speed_slider.value() + 10))
        elif event.key() == Qt.Key.Key_S:
            self.speed_slider.setValue(max(-100, self.speed_slider.value() - 10))
        elif event.key() == Qt.Key.Key_A:
            self.bump_steer(-10)
        elif event.key() == Qt.Key.Key_D:
            self.bump_steer(10)
        elif event.key() == Qt.Key.Key_Space:
            self.stop_vehicle()
        elif event.key() == Qt.Key.Key_R:
            self.reset_path()
        else:
            super().keyPressEvent(event)


def main() -> None:
    app = QApplication(sys.argv)
    QtBootstrap.apply(app)
    window = VehicleTrackerWindow()
    window.show()
    sys.exit(app.exec())
