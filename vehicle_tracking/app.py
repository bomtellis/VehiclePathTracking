from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from math import atan2, ceil, cos, degrees, hypot, radians, sin, sqrt, tan
from pathlib import Path
import sys

from shiboken6 import isValid
from PySide6.QtCore import QPointF, QRectF, QSettings, QSize, Qt, QTimer, Signal
from PySide6.QtGui import QAction, QBrush, QColor, QFont, QFontMetricsF, QImage, QKeyEvent, QPainter, QPen, QPolygonF, QTransform
from PySide6.QtWidgets import (
    QAbstractItemView,
    QApplication,
    QCheckBox,
    QColorDialog,
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
    QGraphicsTextItem,
    QGraphicsView,
    QHBoxLayout,
    QHeaderView,
    QInputDialog,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMenu,
    QMessageBox,
    QPushButton,
    QProgressDialog,
    QScrollArea,
    QSlider,
    QSplitter,
    QTabWidget,
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
from .models import (
    Pose,
    ProjectStore,
    RouteOperation,
    RoutePlan,
    RouteStore,
    StartPosition,
    SteeringMode,
    VehicleProfile,
    VehicleStore,
    VehicleTrackingProject,
    WheelSpec,
    step_pose,
)
from .qtbootstrap import QtBootstrap, line_icon
from .reports import RouteReportEntry, generate_route_report_pdf
from .video_export import export_qimages_to_mp4


ROOT = Path(__file__).resolve().parent.parent


@dataclass
class PayloadPickupAnalysis:
    possible: bool
    message: str
    dropoff_route_name: str = ""
    position_error: float = 0.0
    alignment_error_deg: float = 0.0
    straight_approach_distance: float = 0.0
    required_straight_distance: float = 0.0


class TrackingView(QGraphicsView):
    positionPlaced = Signal(str, QPointF, float)
    routeSketched = Signal(object)

    def __init__(self, scene: QGraphicsScene) -> None:
        super().__init__(scene)
        self.setRenderHints(self.renderHints())
        self.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self._placement_mode: str | None = None
        self._placement_anchor: QPointF | None = None
        self._default_heading = 0.0
        self._heading_line: QGraphicsLineItem | None = None
        self._sketch_points: list[QPointF] = []
        self._sketch_item: QGraphicsPathItem | None = None

    def set_placement_mode(self, mode: str | None, default_heading: float = 0.0) -> None:
        if self._heading_line is not None and self._heading_line.scene() is self.scene():
            self.scene().removeItem(self._heading_line)
        self._heading_line = None
        self._placement_anchor = None
        if self._sketch_item is not None and self._sketch_item.scene() is self.scene():
            self.scene().removeItem(self._sketch_item)
        self._sketch_item = None
        self._sketch_points.clear()
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
                if self._placement_mode == "route_sketch":
                    self._sketch_points = [self._placement_anchor]
                    path = QPainterPath(self._placement_anchor)
                    pen = QPen(QColor("#0ea5e9"), 0)
                    pen.setStyle(Qt.PenStyle.DashLine)
                    self._sketch_item = self.scene().addPath(path, pen)
                    self._sketch_item.setZValue(30.0)
                    event.accept()
                    return
                placement_color = (
                    "#dc2626"
                    if self._placement_mode.endswith("reverse_action")
                    else "#0284c7"
                    if self._placement_mode.endswith("straight_route")
                    else "#a21caf"
                    if self._placement_mode == "dropoff"
                    else "#d97706"
                )
                pen = QPen(QColor(placement_color), 0)
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
        if self._placement_mode == "route_sketch" and self._sketch_item is not None:
            current = self.mapToScene(event.position().toPoint())
            if not self._sketch_points or hypot(
                current.x() - self._sketch_points[-1].x(),
                current.y() - self._sketch_points[-1].y(),
            ) >= max(0.01, 3.0 / max(self.transform().m11(), 1e-9)):
                self._sketch_points.append(current)
                path = QPainterPath(self._sketch_points[0])
                for point in self._sketch_points[1:]:
                    path.lineTo(point)
                self._sketch_item.setPath(path)
            event.accept()
            return
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
            self._placement_mode == "route_sketch"
            and self._placement_anchor is not None
            and event.button() == Qt.MouseButton.LeftButton
        ):
            points = list(self._sketch_points)
            self.set_placement_mode(None)
            if len(points) >= 2:
                self.routeSketched.emit(points)
            event.accept()
            return
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
    def __init__(
        self,
        index: int,
        x: float,
        scene_y: float,
        size: float,
        moved_callback,
        released_callback,
        selected_callback,
        point_turn: bool,
        point_turn_callback,
        reversing_action: bool,
        reversing_action_callback,
        straight_section: bool,
    ) -> None:
        super().__init__(-size, -size, size * 2.0, size * 2.0)
        self.index = index
        self._ready = False
        self._moved_callback = moved_callback
        self._released_callback = released_callback
        self._selected_callback = selected_callback
        self._point_turn_callback = point_turn_callback
        self.point_turn = point_turn
        self._reversing_action_callback = reversing_action_callback
        self.reversing_action = reversing_action
        color = (
            QColor("#dc2626")
            if reversing_action
            else QColor("#7c3aed")
            if point_turn
            else QColor("#0284c7")
            if straight_section
            else QColor("#f59e0b")
        )
        outline = (
            QColor("#991b1b")
            if reversing_action
            else QColor("#5b21b6")
            if point_turn
            else QColor("#075985")
            if straight_section
            else QColor("#b45309")
        )
        self.setPen(QPen(outline, 0))
        self.setBrush(QBrush(color))
        self.setPos(x, scene_y)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsMovable, True)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable, True)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemSendsGeometryChanges, True)
        self.setCursor(Qt.CursorShape.OpenHandCursor)
        self.setZValue(25.0)
        maneuver = (
            "; reversing action enabled"
            if reversing_action
            else "; driven-wheel point turn enabled" if point_turn else ""
        )
        self.setToolTip(
            f"Route control point {index + 1} ({'straight section' if straight_section else 'turn'}): "
            f"drag to reshape the route; right-click for maneuvers{maneuver}"
        )
        self._ready = True

    def itemChange(self, change, value):
        if self._ready and change == QGraphicsItem.GraphicsItemChange.ItemPositionHasChanged:
            self._moved_callback(self.index, value)
        return super().itemChange(change, value)

    def mousePressEvent(self, event) -> None:
        self.setCursor(Qt.CursorShape.ClosedHandCursor)
        super().mousePressEvent(event)
        if event.button() == Qt.MouseButton.LeftButton:
            self._selected_callback(self.index)

    def mouseReleaseEvent(self, event) -> None:
        super().mouseReleaseEvent(event)
        self.setCursor(Qt.CursorShape.OpenHandCursor)
        self._released_callback(self.index, self.pos())

    def contextMenuEvent(self, event) -> None:
        menu = QMenu()
        action = menu.addAction("Driven-wheel point turn")
        action.setCheckable(True)
        action.setChecked(self.point_turn)
        reverse_action = menu.addAction("Reversing action")
        reverse_action.setCheckable(True)
        reverse_action.setChecked(self.reversing_action)
        chosen = menu.exec(event.screenPos())
        if chosen is action:
            self._point_turn_callback(self.index, action.isChecked())
        elif chosen is reverse_action:
            self._reversing_action_callback(self.index, reverse_action.isChecked())
        event.accept()


class CurveTangentHandleItem(QGraphicsEllipseItem):
    def __init__(
        self,
        waypoint_index: int,
        sign: int,
        x: float,
        scene_y: float,
        size: float,
        moved_callback,
        released_callback,
    ) -> None:
        super().__init__(-size, -size, size * 2.0, size * 2.0)
        self.waypoint_index = waypoint_index
        self.sign = sign
        self._moved_callback = moved_callback
        self._released_callback = released_callback
        self._ready = False
        self.setPen(QPen(QColor("#0369a1"), 0))
        self.setBrush(QBrush(QColor("#38bdf8")))
        self.setPos(x, scene_y)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsMovable, True)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemSendsGeometryChanges, True)
        self.setCursor(Qt.CursorShape.SizeAllCursor)
        self.setZValue(24.0)
        side = "outgoing" if sign > 0 else "incoming"
        self.setToolTip(
            f"Curve handle for route point {waypoint_index + 1} ({side}); drag to change bend direction and strength"
        )
        self._ready = True

    def itemChange(self, change, value):
        result = super().itemChange(change, value)
        if self._ready and change == QGraphicsItem.GraphicsItemChange.ItemPositionHasChanged:
            self._moved_callback(self.waypoint_index, self.sign, self.pos())
        return result

    def mouseReleaseEvent(self, event) -> None:
        super().mouseReleaseEvent(event)
        self._released_callback(self.waypoint_index, self.sign, self.pos())


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
    items = []
    if not path.isEmpty():
        items.append(scene.addPath(path, pen))
    for primitive in primitives:
        if primitive.kind != "text" or not primitive.points or not primitive.text:
            continue
        item = QGraphicsTextItem()
        item.setPlainText(primitive.text)
        item.setDefaultTextColor(pen.color())
        item.document().setDocumentMargin(0.0)
        font = QFont("Arial")
        font.setPixelSize(1000)
        item.setFont(font)
        metrics = QFontMetricsF(font)
        cap_height = max(metrics.capHeight(), 1.0)
        scale_y = primitive.text_height / cap_height
        scale_x = scale_y * primitive.width_factor
        bounds = item.boundingRect()
        anchor_x = {
            "left": bounds.left(),
            "center": bounds.center().x(),
            "right": bounds.right(),
        }.get(primitive.horizontal_alignment, bounds.left())
        anchor_y = {
            "top": bounds.top(),
            "middle": bounds.center().y(),
            "bottom": bounds.bottom(),
            "baseline": metrics.ascent(),
        }.get(primitive.vertical_alignment, metrics.ascent())
        x, y = primitive.points[0]
        transform = QTransform()
        transform.translate(x, -y)
        transform.rotate(-primitive.rotation_deg)
        transform.scale(scale_x, scale_y)
        transform.translate(-anchor_x, -anchor_y)
        item.setTransform(transform)
        item.setToolTip(
            f"DXF text (Arial, height {primitive.text_height:g}): {primitive.text}"
        )
        scene.addItem(item)
        items.append(item)
    return items


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


class FloorDxfManagerDialog(QDialog):
    def __init__(
        self,
        levels: list[str],
        level_drawings: dict[str, Path],
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle("Manage Floor DXFs")
        self.resize(780, 430)
        layout = QVBoxLayout(self)
        heading = QLabel("Floor Levels and Drawing Assignments")
        heading.setObjectName("PanelTitle")
        layout.addWidget(heading)
        guidance = QLabel(
            "Every floor should have its own DXF. Add floor levels, then browse to assign or replace "
            "the drawing used when that floor is selected."
        )
        guidance.setWordWrap(True)
        layout.addWidget(guidance)
        self.table = QTableWidget(0, 3)
        self.table.setHorizontalHeaderLabels(["Floor level", "Assigned DXF", "Status"])
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeMode.ResizeToContents)
        self.table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QTableWidget.SelectionMode.SingleSelection)
        layout.addWidget(self.table, 1)
        for level in levels:
            self._append_level(level, level_drawings.get(level))

        actions = QHBoxLayout()
        add_floor = QPushButton(line_icon("add", "#ffffff"), "Add Floor")
        add_floor.clicked.connect(self.add_floor)
        remove_floor = QPushButton("Remove Floor")
        remove_floor.setProperty("variant", "danger")
        remove_floor.clicked.connect(self.remove_floor)
        assign = QPushButton(line_icon("open", "#ffffff"), "Browse / Replace DXF")
        assign.clicked.connect(self.assign_dxf)
        clear = QPushButton("Clear Assignment")
        clear.setProperty("variant", "secondary")
        clear.clicked.connect(self.clear_assignment)
        actions.addWidget(add_floor)
        actions.addWidget(remove_floor)
        actions.addSpacing(12)
        actions.addWidget(assign)
        actions.addWidget(clear)
        actions.addStretch(1)
        layout.addLayout(actions)
        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Save | QDialogButtonBox.StandardButton.Cancel
        )
        buttons.accepted.connect(self._validate_and_accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def _append_level(self, name: str, path: Path | None = None) -> None:
        row = self.table.rowCount()
        self.table.insertRow(row)
        name_item = QTableWidgetItem(name)
        path_item = QTableWidgetItem(str(path) if path is not None else "")
        status_item = QTableWidgetItem()
        status_item.setFlags(status_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
        self.table.setItem(row, 0, name_item)
        self.table.setItem(row, 1, path_item)
        self.table.setItem(row, 2, status_item)
        self._refresh_row(row)
        self.table.selectRow(row)

    def _selected_row(self) -> int:
        rows = self.table.selectionModel().selectedRows()
        return rows[0].row() if rows else -1

    def _refresh_row(self, row: int) -> None:
        value = self.table.item(row, 1).text().strip()
        status = self.table.item(row, 2)
        if not value:
            status.setText("DXF required")
            status.setForeground(QColor(QtBootstrap.semantic_color("warning")))
        elif Path(value).exists():
            status.setText("Ready")
            status.setForeground(QColor(QtBootstrap.semantic_color("success")))
        else:
            status.setText("File missing")
            status.setForeground(QColor(QtBootstrap.semantic_color("danger")))

    def add_floor(self) -> None:
        used = {self.table.item(row, 0).text().strip() for row in range(self.table.rowCount())}
        number = 1
        while f"Level {number}" in used:
            number += 1
        self._append_level(f"Level {number}")

    def remove_floor(self) -> None:
        row = self._selected_row()
        if row < 0:
            QMessageBox.information(self, "Select a floor", "Select the floor row to remove.")
            return
        if self.table.rowCount() <= 1:
            QMessageBox.information(self, "Keep one floor", "At least one floor level must remain.")
            return
        self.table.removeRow(row)
        self.table.selectRow(min(row, self.table.rowCount() - 1))

    def assign_dxf(self) -> None:
        row = self._selected_row()
        if row < 0:
            QMessageBox.information(self, "Select a floor", "Select a floor before assigning its DXF.")
            return
        current = self.table.item(row, 1).text().strip()
        initial = str(Path(current).parent) if current else str(ROOT)
        filename, _ = QFileDialog.getOpenFileName(self, "Assign Floor DXF", initial, "DXF files (*.dxf)")
        if not filename:
            return
        self.table.item(row, 1).setText(filename)
        self._refresh_row(row)

    def clear_assignment(self) -> None:
        row = self._selected_row()
        if row >= 0:
            self.table.item(row, 1).setText("")
            self._refresh_row(row)

    def configuration(self) -> tuple[list[str], dict[str, Path]]:
        levels: list[str] = []
        drawings: dict[str, Path] = {}
        for row in range(self.table.rowCount()):
            level = self.table.item(row, 0).text().strip()
            path = self.table.item(row, 1).text().strip()
            if level:
                levels.append(level)
                if path:
                    drawings[level] = Path(path)
        return levels, drawings

    def _validate_and_accept(self) -> None:
        levels, drawings = self.configuration()
        if not levels:
            QMessageBox.warning(self, "Floor required", "Add at least one floor level.")
            return
        if len(set(levels)) != len(levels):
            QMessageBox.warning(self, "Duplicate floors", "Every floor level must have a unique name.")
            return
        unassigned = [level for level in levels if level not in drawings]
        missing = [level for level, path in drawings.items() if not path.exists()]
        if unassigned:
            QMessageBox.warning(
                self,
                "DXF required",
                "Assign a DXF to every floor before saving:\n" + "\n".join(unassigned),
            )
            return
        if missing:
            QMessageBox.warning(
                self,
                "DXF file missing",
                "Choose an existing DXF for:\n" + "\n".join(missing),
            )
            return
        self.accept()


class AppearanceSettingsDialog(QDialog):
    def __init__(self, theme_mode: str, background: str, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Appearance Settings")
        self.setMinimumWidth(430)
        self.background = background
        layout = QVBoxLayout(self)
        heading = QLabel("Appearance")
        heading.setObjectName("PanelTitle")
        layout.addWidget(heading)
        form = QFormLayout()
        self.theme_combo = QComboBox()
        self.theme_combo.addItem("Follow operating system", "system")
        self.theme_combo.addItem("Light", "light")
        self.theme_combo.addItem("Dark", "dark")
        self.theme_combo.setCurrentIndex(max(0, self.theme_combo.findData(theme_mode)))
        self.theme_combo.currentIndexChanged.connect(self._update_preview)
        form.addRow("Application theme", self.theme_combo)
        background_row = QHBoxLayout()
        self.background_preview = QLabel()
        self.background_preview.setFixedSize(54, 28)
        choose = QPushButton("Choose colour")
        choose.clicked.connect(self.choose_background)
        reset = QPushButton("Theme default")
        reset.setProperty("variant", "secondary")
        reset.clicked.connect(self.reset_background)
        background_row.addWidget(self.background_preview)
        background_row.addWidget(choose)
        background_row.addWidget(reset)
        form.addRow("DXF background", background_row)
        layout.addLayout(form)
        note = QLabel(
            "System mode follows Windows light/dark changes while the app is running. "
            "The DXF background override applies in every theme."
        )
        note.setWordWrap(True)
        layout.addWidget(note)
        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Save | QDialogButtonBox.StandardButton.Cancel
        )
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)
        self._update_preview()

    def choose_background(self) -> None:
        initial = QColor(self.background) if self.background else QColor("#f8fafc")
        color = QColorDialog.getColor(initial, self, "DXF Background Colour")
        if color.isValid():
            self.background = color.name()
            self._update_preview()

    def reset_background(self) -> None:
        self.background = ""
        self._update_preview()

    def _update_preview(self, _value=None) -> None:
        color = self.background or (
            "#172033" if self.theme_combo.currentData() == "dark" else "#f8fafc"
        )
        self.background_preview.setStyleSheet(
            f"background: {color}; border: 1px solid {QtBootstrap.semantic_color('muted')}; border-radius: 4px;"
        )
        self.background_preview.setToolTip(self.background or "Theme default")


class VehicleTrackerWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Vehicle Tracking")
        self.resize(1360, 840)
        self.settings = QSettings("OpenAI", "Vehicle Tracking")
        self.store = VehicleStore(ROOT / "vehicles.json")
        self.vehicles = self.store.load()
        self.current_profile = self.vehicles[0]
        self.current_dxf: DxfDrawing | None = None
        self.route_store = RouteStore(ROOT / "vehicle_routes.json")
        self.levels, self.start_positions, self.saved_routes = self.route_store.load_configuration(None)
        self.project_dxf_path: Path | None = None
        self.project_file_path: Path | None = None
        self.level_drawing_paths = self.route_store.load_level_drawings(None)
        self.level_drawing_cache: dict[str, DxfDrawing] = {}
        self._shared_block_cache: dict[
            str, tuple[DxfDrawing | None, DxfBlockGeometry | None]
        ] = {}
        self.current_start_name = self.start_positions[0].name
        self.current_level_name = self.start_positions[0].level_name
        self.start_pose = Pose(
            self.start_positions[0].pose.x,
            self.start_positions[0].pose.y,
            self.start_positions[0].pose.heading_deg,
        )
        self.end_pose: Pose | None = None
        self.dropoff_pose: Pose | None = None
        self.route_dropoff_waypoint_index: int | None = None
        self.route_waypoints: list[tuple[float, float]] = []
        self.route_point_turns: set[int] = set()
        self.route_reversing_actions: set[int] = set()
        self.route_tangent_handles: dict[int, tuple[float, float]] = {}
        self.route_point_path_modes: dict[int, str] = {}
        self.route_start_operation = "travel"
        self.route_end_operation = "stop"
        self._updating_operation_table = False
        self._selected_route_point_index: int | None = None
        self._current_route_section = "pre"
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
        self.view.routeSketched.connect(self.create_route_from_sketch)
        self.timer = QTimer(self)
        self.timer.setInterval(80)
        self.timer.timeout.connect(self.advance_vehicle)
        self.route_animation_timer = QTimer(self)
        self.route_animation_timer.setInterval(50)
        self.route_animation_timer.timeout.connect(self.advance_route_animation)
        self.route_animation_poses: list[Pose] = []
        self.route_animation_index = 0
        self.route_animation_paused = False
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
        self.route_tangent_items: list[CurveTangentHandleItem] = []
        self.route_tangent_lines: list[QGraphicsLineItem] = []
        self._block_path_cache: dict[tuple[int, str], QPainterPath] = {}
        self._build_actions()
        self._build_layout()
        self._load_profile_to_form(self.current_profile)
        self._refresh_route_combo()
        self.redraw_scene()

    def _build_actions(self) -> None:
        self.run_action = QAction(line_icon("play", "#ffffff"), "Run", self)
        self.run_action.triggered.connect(self.toggle_run)

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
        central = QWidget()
        central.setObjectName("CentralWidget")
        central_layout = QVBoxLayout(central)
        central_layout.setContentsMargins(0, 0, 0, 0)
        central_layout.setSpacing(0)
        central_layout.addWidget(self._ribbon_bar())
        central_layout.addWidget(splitter, 1)
        self.setCentralWidget(central)
        self.statusBar().showMessage("Assign a DXF to each level, then configure starts and paths.")

    def _ribbon_group(self, title: str, buttons: list[QPushButton]) -> QFrame:
        group = QFrame()
        group.setObjectName("RibbonGroup")
        layout = QVBoxLayout(group)
        layout.setContentsMargins(8, 5, 8, 5)
        layout.setSpacing(3)
        row = QHBoxLayout()
        row.setSpacing(6)
        for button in buttons:
            row.addWidget(button)
        layout.addLayout(row)
        label = QLabel(title)
        label.setObjectName("RibbonGroupTitle")
        label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(label)
        return group

    def _ribbon_button(self, icon: str, text: str, callback, variant: str = "") -> QPushButton:
        button = QPushButton(line_icon(icon, QtBootstrap.icon_color(variant)), text)
        button.setProperty("themeIconName", icon)
        if variant:
            button.setProperty("variant", variant)
        button.clicked.connect(callback)
        return button

    def _ribbon_bar(self) -> QTabWidget:
        ribbon = QTabWidget()
        ribbon.setObjectName("RibbonBar")
        ribbon.setDocumentMode(True)
        ribbon.setMaximumHeight(140)

        def scrollable_tab(content: QWidget) -> QScrollArea:
            scroll = QScrollArea()
            scroll.setFrameShape(QFrame.Shape.NoFrame)
            scroll.setWidgetResizable(True)
            scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
            scroll.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
            scroll.setWidget(content)
            return scroll

        home = QWidget()
        home_layout = QHBoxLayout(home)
        home_layout.setContentsMargins(8, 4, 8, 4)
        home_layout.setSpacing(6)
        home_layout.addWidget(self._ribbon_group("Floor drawing", [
            self._ribbon_button("open", "Open Project", self.import_dxf),
            self._ribbon_button("save", "Save Project", self.save_project),
            self._ribbon_button("open", "Manage Floor DXFs", self.manage_floor_dxfs),
            self._ribbon_button("fit", "Fit Floor", self.fit_drawing, "secondary"),
        ]))
        home_layout.addWidget(self._ribbon_group("Positions", [
            self._ribbon_button("start", "Place Start", self.begin_place_start),
            self._ribbon_button("end", "Place Drop-off", self.begin_place_dropoff, "warning"),
            self._ribbon_button("end", "Place End", self.begin_place_end),
            self._ribbon_button("save", "Save Start", self.save_start_position, "secondary"),
        ]))
        self.run_button = self._ribbon_button("play", "Run", self.toggle_run)
        home_layout.addWidget(self._ribbon_group("Drive", [
            self.run_button,
            self._ribbon_button("reset", "Reset Driven Path", self.reset_path, "secondary"),
        ]))
        home_layout.addWidget(self._ribbon_group("Application", [
            self._ribbon_button("wheel", "Settings", self.show_settings, "secondary"),
        ]))
        home_layout.addStretch(1)
        ribbon.addTab(scrollable_tab(home), "Home")

        route_tab = QWidget()
        route_layout = QHBoxLayout(route_tab)
        route_layout.setContentsMargins(8, 4, 8, 4)
        route_layout.setSpacing(6)
        route_layout.addWidget(self._ribbon_group("Saved path", [
            self._ribbon_button("save", "Save Path", self.save_current_route),
            self._ribbon_button("add", "New Path", self.new_route),
            self._ribbon_button("add", "Copy Path", self.copy_current_route, "secondary"),
            self._ribbon_button("stop", "Remove Path", self.remove_saved_route, "secondary"),
        ]))
        route_layout.addWidget(self._ribbon_group("Before drop-off", [
            self._ribbon_button("add", "Turn Point", lambda: self.begin_insert_route_point("pre")),
            self._ribbon_button("add", "Straight Point", lambda: self.begin_insert_straight_point("pre"), "secondary"),
            self._ribbon_button("left", "Reverse Action", lambda: self.begin_place_reverse_action("pre"), "warning"),
        ]))
        route_layout.addWidget(self._ribbon_group("After drop-off", [
            self._ribbon_button("add", "Turn Point", lambda: self.begin_insert_route_point("post")),
            self._ribbon_button("add", "Straight Point", lambda: self.begin_insert_straight_point("post"), "secondary"),
            self._ribbon_button("left", "Reverse Action", lambda: self.begin_place_reverse_action("post"), "warning"),
        ]))
        route_layout.addWidget(self._ribbon_group("Edit path", [
            self._ribbon_button("add", "Draw Route", self.begin_draw_route, "secondary"),
            self._alignment_suggestion_button(),
            self._ribbon_button("stop", "Remove Point", self.remove_selected_route_point, "secondary"),
            self._ribbon_button("reset", "Clear Points", self.clear_route_points, "secondary"),
        ]))
        self.animate_route_button = self._ribbon_button("play", "Animate", self.toggle_route_animation)
        self.pause_route_button = self._ribbon_button("stop", "Pause", self.pause_route_animation, "secondary")
        self.pause_route_button.setEnabled(False)
        route_layout.addWidget(self._ribbon_group("Playback", [
            self.animate_route_button, self.pause_route_button,
        ]))
        route_layout.addWidget(self._ribbon_group("Export", [
            self._ribbon_button("export", "Tracking DXF", self.export_dxf),
            self._ribbon_button("export", "Route Report", self.export_route_report),
            self._ribbon_button("export", "Path MP4", self.export_route_mp4),
        ]))
        route_layout.addStretch(1)
        ribbon.addTab(scrollable_tab(route_tab), "Route & Export")
        return ribbon

    def _alignment_suggestion_button(self) -> QPushButton:
        self.suggest_alignment_button = self._ribbon_button(
            "fit", "Suggest Alignment", self.create_alignment_point_suggestions
        )
        self.suggest_alignment_button.setEnabled(
            self.end_pose is not None and self.dropoff_pose is not None
        )
        self.suggest_alignment_button.setToolTip(
            "Create editable straight-line delivery and reverse-egress points from the drop-off heading"
        )
        return self.suggest_alignment_button

    def show_settings(self) -> None:
        theme = str(self.settings.value("appearance/theme", "system"))
        background = str(self.settings.value("appearance/dxf_background", ""))
        dialog = AppearanceSettingsDialog(theme, background, self)
        if dialog.exec() != QDialog.DialogCode.Accepted:
            return
        theme = str(dialog.theme_combo.currentData())
        background = dialog.background
        self.settings.setValue("appearance/theme", theme)
        self.settings.setValue("appearance/dxf_background", background)
        QtBootstrap.apply(
            QApplication.instance(), theme=theme, dxf_background=background or None
        )
        self._update_level_dxf_label()
        self.redraw_scene()
        self.statusBar().showMessage(
            f"Appearance updated: {dialog.theme_combo.currentText()}, "
            f"DXF background {background or 'theme default'}."
        )

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
        self.steering_mode_combo.currentIndexChanged.connect(self.update_turning_radius_calculation)
        self.max_steer_spin = self._spin(1.0, 89.0, 70.0)
        self.max_steer_spin.valueChanged.connect(self.update_turning_radius_calculation)
        self.wheelbase_spin.valueChanged.connect(self.update_turning_radius_calculation)
        self.min_radius_spin = self._spin(0.0, 1_000_000.0, 1.4)
        self.min_radius_spin.valueChanged.connect(self.update_turning_radius_calculation)
        self.pose_spacing_spin = self._spin(0.001, 1_000_000.0, 0.75)
        form.addRow("Name", self.name_edit)
        form.addRow("DXF block", self.block_combo)
        form.addRow("Block forward", self.block_forward_spin)
        form.addRow("Length", self.length_spin)
        form.addRow("Width", self.width_spin)
        form.addRow("Wheelbase", self.wheelbase_spin)
        form.addRow("Steering", self.steering_mode_combo)
        form.addRow("Max steer deg", self.max_steer_spin)
        form.addRow("Configured min radius", self.min_radius_spin)
        form.addRow("Pose spacing", self.pose_spacing_spin)
        layout.addLayout(form)
        radius_row = QHBoxLayout()
        self.calculated_radius_label = QLabel()
        self.calculated_radius_label.setWordWrap(True)
        self.calculated_radius_label.setObjectName("CalculatedRadius")
        apply_radius = QPushButton("Apply calculated radius")
        apply_radius.setProperty("variant", "secondary")
        apply_radius.clicked.connect(self.apply_calculated_turning_radius)
        radius_row.addWidget(self.calculated_radius_label, 1)
        radius_row.addWidget(apply_radius)
        layout.addLayout(radius_row)
        self.update_turning_radius_calculation()

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
        self.load_distance_spin = self._spin(0.0, 1_000_000.0, 0.0)
        self.aisle_clearance_spin = self._spin(0.0, 1_000_000.0, 200.0)
        for control in (
            self.payload_x_spin,
            self.payload_y_spin,
            self.payload_length_spin,
            self.payload_width_spin,
            self.payload_rotation_spin,
            self.load_distance_spin,
            self.aisle_clearance_spin,
        ):
            control.valueChanged.connect(self.payload_changed)
            control.valueChanged.connect(self.update_turning_radius_calculation)
        payload_form.addRow("Centre X", self.payload_x_spin)
        payload_form.addRow("Centre Y", self.payload_y_spin)
        payload_form.addRow("Length", self.payload_length_spin)
        payload_form.addRow("Width", self.payload_width_spin)
        payload_form.addRow("Rotation", self.payload_rotation_spin)
        payload_form.addRow("Load distance x", self.load_distance_spin)
        payload_form.addRow("Aisle clearance a", self.aisle_clearance_spin)
        layout.addLayout(payload_form)

        save_profile = QPushButton(line_icon("save", "#ffffff"), "Save Vehicle")
        save_profile.clicked.connect(self.save_vehicle)
        layout.addWidget(save_profile)

        position_title = QLabel("Start / End Positions")
        position_title.setObjectName("SectionTitle")
        layout.addWidget(position_title)
        level_row = QHBoxLayout()
        self.level_combo = QComboBox()
        self.level_combo.setEditable(True)
        self.level_combo.addItems(self.levels)
        self.level_combo.setCurrentText(self.current_level_name)
        self.level_combo.currentTextChanged.connect(self.change_level)
        manage_levels = QPushButton(line_icon("open", "#ffffff"), "Manage Floors / DXFs")
        manage_levels.clicked.connect(self.manage_floor_dxfs)
        level_row.addWidget(self.level_combo, 1)
        level_row.addWidget(manage_levels)
        layout.addLayout(level_row)
        self.level_dxf_label = QLabel()
        self.level_dxf_label.setObjectName("FloorDrawingStatus")
        self.level_dxf_label.setWordWrap(True)
        layout.addWidget(self.level_dxf_label)
        self._update_level_dxf_label()
        start_row = QHBoxLayout()
        self.start_position_combo = QComboBox()
        self.start_position_combo.addItems([start.name for start in self.start_positions])
        self.start_position_combo.setCurrentText(self.current_start_name)
        self.start_position_combo.currentTextChanged.connect(self.change_start_position)
        save_start = QPushButton(line_icon("save", "#ffffff"), "Save Start")
        save_start.clicked.connect(self.save_start_position)
        add_start = QPushButton(line_icon("add", "#ffffff"), "New Start")
        add_start.clicked.connect(self.add_start_position)
        remove_start = QPushButton("Remove")
        remove_start.setProperty("variant", "secondary")
        remove_start.clicked.connect(self.remove_start_position)
        start_row.addWidget(self.start_position_combo, 1)
        start_row.addWidget(save_start)
        start_row.addWidget(add_start)
        start_row.addWidget(remove_start)
        layout.addLayout(start_row)
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
        self.dropoff_heading_spin = QDoubleSpinBox()
        self.dropoff_heading_spin.setRange(-360.0, 360.0)
        self.dropoff_heading_spin.setDecimals(1)
        self.dropoff_heading_spin.setSuffix("°")
        self.dropoff_heading_spin.valueChanged.connect(self.update_pose_headings)
        heading_form.addRow("Start heading", self.start_heading_spin)
        heading_form.addRow("Drop-off heading", self.dropoff_heading_spin)
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
        self.show_route_checkbox = QCheckBox("Show planned route and swept envelope")
        self.show_route_checkbox.setChecked(True)
        self.show_route_checkbox.toggled.connect(self.toggle_route_visibility)
        layout.addWidget(self.show_route_checkbox)
        self.show_other_paths_checkbox = QCheckBox("Show other saved paths")
        self.show_other_paths_checkbox.setChecked(
            self.settings.value("visibility/show_other_paths", True, type=bool)
        )
        self.show_other_paths_checkbox.toggled.connect(self.toggle_other_paths_visibility)
        layout.addWidget(self.show_other_paths_checkbox)
        self.current_section_only_checkbox = QCheckBox("Show current section only")
        self.current_section_only_checkbox.setChecked(False)
        self.current_section_only_checkbox.toggled.connect(self.toggle_current_section_visibility)
        layout.addWidget(self.current_section_only_checkbox)
        self.route_feasibility_label = QLabel("Route check: place a finish position")
        self.route_feasibility_label.setWordWrap(True)
        QtBootstrap.style_semantic(self.route_feasibility_label, "muted")
        layout.addWidget(self.route_feasibility_label)
        operations_title = QLabel("Ordered Route Operations")
        operations_title.setObjectName("SectionTitle")
        layout.addWidget(operations_title)
        alignment_strategy_row = QHBoxLayout()
        alignment_strategy_row.addWidget(QLabel("Suggestion egress"))
        self.alignment_strategy_combo = QComboBox()
        self.alignment_strategy_combo.addItem("Best feasible option", "auto")
        self.alignment_strategy_combo.addItem("Reverse to final position", "reverse_to_final")
        self.alignment_strategy_combo.addItem("Reverse out, then travel forward", "resume_forward")
        self.alignment_strategy_combo.setToolTip(
            "Controls whether suggested points keep reversing to the final position or add a reverse action at the egress point to resume forward travel"
        )
        alignment_strategy_row.addWidget(self.alignment_strategy_combo, 1)
        layout.addLayout(alignment_strategy_row)
        self.route_operations_table = QTableWidget(0, 3)
        self.route_operations_table.setHorizontalHeaderLabels(["Order", "Point", "Operation"])
        self.route_operations_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        self.route_operations_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        self.route_operations_table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeMode.Stretch)
        self.route_operations_table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.route_operations_table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.route_operations_table.itemSelectionChanged.connect(self._operation_selection_changed)
        self.route_operations_table.setMinimumHeight(145)
        layout.addWidget(self.route_operations_table)
        operation_order_row = QHBoxLayout()
        self.move_route_point_up_button = QPushButton("Move Point Up")
        self.move_route_point_up_button.setProperty("variant", "secondary")
        self.move_route_point_up_button.clicked.connect(lambda: self.move_selected_route_point(-1))
        self.move_route_point_down_button = QPushButton("Move Point Down")
        self.move_route_point_down_button.setProperty("variant", "secondary")
        self.move_route_point_down_button.clicked.connect(lambda: self.move_selected_route_point(1))
        operation_order_row.addWidget(self.move_route_point_up_button)
        operation_order_row.addWidget(self.move_route_point_down_button)
        layout.addLayout(operation_order_row)
        self._refresh_route_operations_table()
        playback_title = QLabel("Animation Timeline")
        playback_title.setObjectName("SectionTitle")
        layout.addWidget(playback_title)
        self.route_animation_slider = QSlider(Qt.Orientation.Horizontal)
        self.route_animation_slider.setRange(0, 0)
        self.route_animation_slider.setEnabled(False)
        self.route_animation_slider.valueChanged.connect(self.scrub_route_animation)
        layout.addWidget(self.route_animation_slider)
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
        QtBootstrap.style_semantic(self.direction_value_label, "success")
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

    def _turning_radius_profile(self) -> VehicleProfile:
        mode = self.steering_mode_combo.currentData()
        return VehicleProfile(
            length=self.length_spin.value(),
            width=self.width_spin.value(),
            steering_mode=SteeringMode(mode or SteeringMode.ACKERMANN_REAR.value),
            wheelbase=self.wheelbase_spin.value(),
            max_steering_angle_deg=self.max_steer_spin.value(),
            min_turning_radius=self.min_radius_spin.value(),
            payload_enabled=(
                self.payload_enabled_checkbox.isChecked()
                if hasattr(self, "payload_enabled_checkbox") else False
            ),
            payload_x=self.payload_x_spin.value() if hasattr(self, "payload_x_spin") else 0.0,
            payload_y=self.payload_y_spin.value() if hasattr(self, "payload_y_spin") else 0.0,
            payload_length=(self.payload_length_spin.value() if hasattr(self, "payload_length_spin") else 1.2),
            payload_width=(self.payload_width_spin.value() if hasattr(self, "payload_width_spin") else 1.0),
            payload_rotation_deg=(self.payload_rotation_spin.value() if hasattr(self, "payload_rotation_spin") else 0.0),
            load_distance=(self.load_distance_spin.value() if hasattr(self, "load_distance_spin") else 0.0),
            aisle_clearance=(self.aisle_clearance_spin.value() if hasattr(self, "aisle_clearance_spin") else 200.0),
            wheels=(self.form_profile().wheels if hasattr(self, "wheel_table") else []),
        )

    def update_turning_radius_calculation(self, _value=None) -> None:
        if not hasattr(self, "calculated_radius_label"):
            return
        profile = self._turning_radius_profile()
        calculated = profile.calculated_min_turning_radius
        effective = profile.effective_min_turning_radius
        self.calculated_radius_label.setText(
            f"Calculated centre radius for {profile.steering_mode.label}: {calculated:.3f}\n"
            f"{profile.turning_radius_calculation}\n"
            f"Planner centre radius: {effective:.3f}\n"
            f"Rear-steer outer radius Wa: {profile.calculated_outer_turning_radius:.3f}\n"
            f"Pallet-truck aisle width Ast: {profile.pallet_truck_aisle_width:.3f}"
        )
        self.calculated_radius_label.setToolTip(
            "Planner radius controls centre-path curvature. Wa includes the furthest vehicle/load corner. "
            "Ast uses Toyota's pallet/reach-truck formula and the configured load distance and clearance."
        )

    def apply_calculated_turning_radius(self) -> None:
        profile = self._turning_radius_profile()
        self.min_radius_spin.setValue(profile.calculated_min_turning_radius)
        self.update_turning_radius_calculation()
        self.redraw_dynamic_layers(self.form_profile())
        self.statusBar().showMessage(
            f"Applied {profile.calculated_min_turning_radius:.3f} as the minimum radius "
            f"for {profile.steering_mode.label}."
        )

    def add_level(self) -> None:
        name = self.level_combo.currentText().strip()
        if not name:
            name, accepted = QInputDialog.getText(self, "Add level", "Level name")
            if not accepted:
                return
            name = name.strip()
        if not name:
            return
        if name not in self.levels:
            self.levels.append(name)
            self.level_combo.addItem(name)
        self.current_level_name = name
        self.level_combo.setCurrentText(name)
        self._persist_routes()
        self._update_position_label()
        self.statusBar().showMessage(f"Level '{name}' is available for starts and paths.")

    def remove_level(self) -> None:
        name = self.level_combo.currentText().strip()
        if len(self.levels) <= 1:
            QMessageBox.information(self, "Keep one level", "At least one level must remain configured.")
            return
        if any(start.level_name == name for start in self.start_positions) or any(
            route.level_name == name for route in self.saved_routes
        ):
            QMessageBox.warning(
                self,
                "Level in use",
                "Move or remove the start positions and saved paths assigned to this level first.",
            )
            return
        self.levels.remove(name)
        self.level_drawing_paths.pop(name, None)
        self.level_drawing_cache.pop(name, None)
        self._invalidate_shared_block_cache()
        self.level_combo.removeItem(self.level_combo.findText(name))
        self.current_level_name = self.levels[0]
        self.level_combo.setCurrentText(self.current_level_name)
        self._persist_routes()

    def change_level(self, name: str) -> None:
        name = name.strip()
        if name:
            self.current_level_name = name
            self._load_dxf_for_level(name)
            start = next((item for item in self.start_positions if item.level_name == name), None)
            if start is not None and start.name != self.current_start_name:
                self.start_position_combo.blockSignals(True)
                self.start_position_combo.setCurrentText(start.name)
                self.start_position_combo.blockSignals(False)
                self.current_start_name = start.name
                self.start_pose = Pose(start.pose.x, start.pose.y, start.pose.heading_deg)
                self.poses = [self.start_pose]
            self._update_position_label()
            if hasattr(self, "wheel_table"):
                self.redraw_scene()

    def _update_level_dxf_label(self) -> None:
        if not hasattr(self, "level_dxf_label"):
            return
        path = self.level_drawing_paths.get(self.current_level_name)
        if path is None:
            self.level_dxf_label.setText("Floor DXF: not assigned - use Home > Assign DXF to Floor")
            QtBootstrap.style_semantic(self.level_dxf_label, "warning")
        else:
            self.level_dxf_label.setText(f"Floor DXF: {path.name}\n{path}")
            QtBootstrap.style_semantic(self.level_dxf_label, "primary")

    def _apply_current_dxf(self, drawing: DxfDrawing | None) -> None:
        self.current_dxf = drawing
        self._block_path_cache.clear()
        if drawing is not None:
            for block_name in drawing.block_names:
                cached = self._shared_block_cache.get(block_name)
                if cached is None or cached == (None, None):
                    self._shared_block_cache.pop(block_name, None)
        if hasattr(self, "block_combo"):
            selected = self.current_profile.dxf_block_name
            self.block_combo.blockSignals(True)
            self.block_combo.clear()
            self.block_combo.addItem("")
            if drawing is not None:
                self.block_combo.addItems(drawing.block_names)
            self.block_combo.setCurrentText(selected)
            self.block_combo.blockSignals(False)
        self._update_level_dxf_label()

    def _drawing_with_block(self, block_name: str) -> DxfDrawing | None:
        """Find a configured floor drawing that defines the vehicle block."""
        if not block_name:
            return None
        candidates: list[DxfDrawing] = []
        if self.current_dxf is not None:
            candidates.append(self.current_dxf)
        candidates.extend(
            drawing
            for drawing in self.level_drawing_cache.values()
            if drawing not in candidates
        )
        for drawing in candidates:
            if block_name in drawing.block_names:
                return drawing
        for level_name, path in self.level_drawing_paths.items():
            if any(drawing.path == path for drawing in candidates) or not path.exists():
                continue
            try:
                drawing = load_dxf(path)
            except Exception:
                continue
            self.level_drawing_cache[level_name] = drawing
            candidates.append(drawing)
            if block_name in drawing.block_names:
                return drawing
        return None

    def _shared_block_geometry(
        self, block_name: str
    ) -> tuple[DxfDrawing | None, DxfBlockGeometry | None]:
        if block_name in self._shared_block_cache:
            return self._shared_block_cache[block_name]
        drawing = self._drawing_with_block(block_name)
        result = (
            drawing,
            get_block_geometry(drawing, block_name) if drawing is not None else None,
        )
        self._shared_block_cache[block_name] = result
        return result

    def _invalidate_shared_block_cache(self) -> None:
        self._shared_block_cache.clear()
        self._block_path_cache.clear()

    def _load_dxf_for_level(self, level_name: str) -> bool:
        path = self.level_drawing_paths.get(level_name)
        if path is None:
            self._apply_current_dxf(None)
            return False
        cached = self.level_drawing_cache.get(level_name)
        if cached is not None and cached.path == path:
            self._apply_current_dxf(cached)
            return True
        if not path.exists():
            self._apply_current_dxf(None)
            self.statusBar().showMessage(f"The DXF assigned to {level_name} cannot be found: {path}")
            return False
        try:
            drawing = self._load_dxf_with_progress(path, f"Loading {level_name}")
        except Exception as exc:
            self._apply_current_dxf(None)
            QMessageBox.critical(self, "Floor DXF failed", f"Could not load the DXF for {level_name}:\n{exc}")
            return False
        self.level_drawing_cache[level_name] = drawing
        self._apply_current_dxf(drawing)
        return True

    def _load_dxf_with_progress(self, path: Path, title: str) -> DxfDrawing:
        progress = QProgressDialog("Preparing DXF", None, 0, 100, self)
        progress.setWindowTitle(title)
        progress.setWindowModality(Qt.WindowModality.WindowModal)
        progress.setMinimumDuration(0)
        progress.setAutoClose(False)
        progress.setValue(0)
        interactive = QApplication.platformName().casefold() != "offscreen"
        if interactive:
            progress.show()

        def update(value: int, message: str) -> None:
            progress.setLabelText(message)
            progress.setValue(value)
            self.statusBar().showMessage(f"{title}: {message} ({value}%)")
            if interactive:
                QApplication.processEvents()

        try:
            return load_dxf(path, update)
        finally:
            progress.close()

    def assign_dxf_to_level(self) -> None:
        level = self.level_combo.currentText().strip()
        if not level:
            QMessageBox.information(self, "Select a floor", "Select or add a floor before assigning its DXF.")
            return
        existing = self.level_drawing_paths.get(level)
        initial = existing.parent if existing is not None else ROOT
        filename, _ = QFileDialog.getOpenFileName(
            self, f"Assign DXF to {level}", str(initial), "DXF files (*.dxf)"
        )
        if not filename:
            return
        path = Path(filename)
        if self.project_dxf_path is None:
            self.project_dxf_path = path
        self.level_drawing_paths[level] = path
        self.level_drawing_cache.pop(level, None)
        self._invalidate_shared_block_cache()
        self._persist_routes()
        self._load_dxf_for_level(level)
        self.redraw_scene()

    def manage_floor_dxfs(self) -> None:
        dialog = FloorDxfManagerDialog(self.levels, self.level_drawing_paths, self)
        if dialog.exec() != QDialog.DialogCode.Accepted:
            return
        levels, drawings = dialog.configuration()
        removed = set(self.levels) - set(levels)
        used = sorted(
            removed
            & (
                {start.level_name for start in self.start_positions}
                | {route.level_name for route in self.saved_routes}
            )
        )
        if used:
            QMessageBox.warning(
                self,
                "Floor levels in use",
                "These floors still contain start positions or saved paths and cannot be removed:\n"
                + "\n".join(used),
            )
            return
        previous_paths = dict(self.level_drawing_paths)
        self.levels = levels
        self.level_drawing_paths = drawings
        self._invalidate_shared_block_cache()
        for level in list(self.level_drawing_cache):
            if level not in drawings or previous_paths.get(level) != drawings[level]:
                self.level_drawing_cache.pop(level, None)
        if self.project_dxf_path is None and drawings:
            self.project_dxf_path = next(iter(drawings.values()))
        if self.current_level_name not in levels:
            self.current_level_name = levels[0]
        self.level_combo.blockSignals(True)
        self.level_combo.clear()
        self.level_combo.addItems(levels)
        self.level_combo.setCurrentText(self.current_level_name)
        self.level_combo.blockSignals(False)
        self._load_dxf_for_level(self.current_level_name)
        self._persist_routes()
        self._update_position_label()
        self.redraw_scene()
        self.statusBar().showMessage(
            f"Updated {len(levels)} floor DXF assignment(s); active floor: {self.current_level_name}."
        )

    def add_start_position(self) -> None:
        default = f"Start {len(self.start_positions) + 1}"
        name, accepted = QInputDialog.getText(self, "New start position", "Start position name", text=default)
        if not accepted or not name.strip():
            return
        name = name.strip()
        if any(start.name == name for start in self.start_positions):
            QMessageBox.warning(self, "Name already used", "Choose a unique start position name.")
            return
        level = self.level_combo.currentText().strip() or self.levels[0]
        if level not in self.levels:
            self.levels.append(level)
            self.level_combo.addItem(level)
        start = StartPosition(
            name,
            level,
            Pose(self.start_pose.x, self.start_pose.y, self.start_pose.heading_deg),
        )
        self.start_positions.append(start)
        self.start_position_combo.addItem(name)
        self.start_position_combo.setCurrentText(name)
        self.current_start_name = name
        self.current_level_name = level
        self._persist_routes()
        self.statusBar().showMessage(f"Added '{name}' on {level}; place the vehicle and Save Start to configure it.")

    def save_start_position(self) -> None:
        level = self.level_combo.currentText().strip() or self.levels[0]
        if level not in self.levels:
            self.levels.append(level)
            self.level_combo.addItem(level)
        start = next(
            (item for item in self.start_positions if item.name == self.current_start_name),
            None,
        )
        if start is None:
            self.add_start_position()
            return
        start.level_name = level
        start.pose = Pose(self.start_pose.x, self.start_pose.y, self.start_pose.heading_deg)
        self.current_level_name = level
        self._persist_routes()
        self._update_position_label()
        self.statusBar().showMessage(f"Saved '{start.name}' on {level}.")

    def remove_start_position(self) -> None:
        if len(self.start_positions) <= 1:
            QMessageBox.information(self, "Keep one start", "At least one start position must remain configured.")
            return
        name = self.current_start_name
        if any(route.start_position_name == name for route in self.saved_routes):
            QMessageBox.warning(self, "Start in use", "Remove or re-save paths that use this start position first.")
            return
        self.start_positions = [start for start in self.start_positions if start.name != name]
        index = self.start_position_combo.findText(name)
        if index >= 0:
            self.start_position_combo.removeItem(index)
        self.change_start_position(self.start_positions[0].name)
        self._persist_routes()

    def change_start_position(self, name: str) -> None:
        start = next((item for item in self.start_positions if item.name == name), None)
        if start is None:
            return
        self.stop_route_animation()
        self.current_start_name = start.name
        self.current_level_name = start.level_name
        self._load_dxf_for_level(start.level_name)
        self.start_pose = Pose(start.pose.x, start.pose.y, start.pose.heading_deg)
        self.poses = [self.start_pose]
        if hasattr(self, "level_combo"):
            self.level_combo.blockSignals(True)
            self.level_combo.setCurrentText(start.level_name)
            self.level_combo.blockSignals(False)
            self.start_heading_spin.blockSignals(True)
            self.start_heading_spin.setValue(start.pose.heading_deg)
            self.start_heading_spin.blockSignals(False)
            self._update_position_label()
            self._redraw_route_layers()

    def _project_snapshot(self) -> VehicleTrackingProject:
        return VehicleTrackingProject(
            list(self.levels),
            dict(self.level_drawing_paths),
            list(self.start_positions),
            list(self.saved_routes),
            list(self.vehicles),
            self.current_level_name,
            self.current_start_name,
        )

    def _write_project(self) -> None:
        if self.project_file_path is None:
            return
        ProjectStore.save(self.project_file_path, self._project_snapshot())

    def save_project(self) -> None:
        unassigned = [level for level in self.levels if level not in self.level_drawing_paths]
        missing = [
            level for level, path in self.level_drawing_paths.items() if not path.exists()
        ]
        if unassigned or missing:
            detail = []
            if unassigned:
                detail.append("No DXF assigned: " + ", ".join(unassigned))
            if missing:
                detail.append("DXF file missing: " + ", ".join(missing))
            QMessageBox.warning(
                self,
                "Complete floor drawings",
                "Use Manage Floor DXFs before saving the project.\n\n" + "\n".join(detail),
            )
            return
        path = self.project_file_path
        if path is None:
            stem = self.project_dxf_path.stem if self.project_dxf_path else "vehicle_tracking_project"
            filename, _ = QFileDialog.getSaveFileName(
                self,
                "Save Vehicle Tracking Project",
                str(ROOT / f"{stem}.vtproject"),
                "Vehicle Tracking project (*.vtproject)",
            )
            if not filename:
                return
            path = Path(filename)
            if path.suffix.casefold() != ".vtproject":
                path = path.with_suffix(".vtproject")
            self.project_file_path = path
        try:
            ProjectStore.save(path, self._project_snapshot())
        except Exception as exc:
            QMessageBox.critical(self, "Project save failed", str(exc))
            return
        self.setWindowTitle(f"Vehicle Tracking - {path.stem}")
        self.statusBar().showMessage(
            f"Saved project with {len(self.levels)} floor(s), {len(self.saved_routes)} route(s), "
            f"and {len(self.vehicles)} vehicle profile(s): {path}"
        )

    def _open_project_file(self, path: Path) -> None:
        try:
            project = ProjectStore.load(path)
        except Exception as exc:
            QMessageBox.critical(self, "Project open failed", str(exc))
            return
        self.project_file_path = path.resolve()
        self.project_dxf_path = None
        self.levels = project.levels
        self.level_drawing_paths = project.level_drawings
        self.level_drawing_cache.clear()
        self._invalidate_shared_block_cache()
        self.start_positions = project.start_positions
        self.saved_routes = project.routes
        self.vehicles = project.vehicles
        self.current_profile = self.vehicles[0]
        self.current_level_name = project.active_level
        self.current_start_name = project.active_start
        start = next(
            (item for item in self.start_positions if item.name == self.current_start_name),
            self.start_positions[0],
        )
        self.start_pose = Pose(start.pose.x, start.pose.y, start.pose.heading_deg)
        self.poses = [self.start_pose]
        self.end_pose = None
        self.dropoff_pose = None
        self.route_dropoff_waypoint_index = None
        self.route_waypoints.clear()
        self._selected_route_point_index = None
        self.route_point_turns.clear()
        self.route_reversing_actions.clear()
        self.route_tangent_handles.clear()
        self.route_point_path_modes.clear()
        self.route_start_operation = "travel"
        self.route_end_operation = "stop"
        self.active_route_index = None
        self._load_dxf_for_level(self.current_level_name)
        self.level_combo.blockSignals(True)
        self.level_combo.clear()
        self.level_combo.addItems(self.levels)
        self.level_combo.setCurrentText(self.current_level_name)
        self.level_combo.blockSignals(False)
        self.start_position_combo.blockSignals(True)
        self.start_position_combo.clear()
        self.start_position_combo.addItems([item.name for item in self.start_positions])
        self.start_position_combo.setCurrentText(self.current_start_name)
        self.start_position_combo.blockSignals(False)
        self.vehicle_combo.blockSignals(True)
        self.vehicle_combo.clear()
        self.vehicle_combo.addItems([vehicle.name for vehicle in self.vehicles])
        self.vehicle_combo.setCurrentIndex(0)
        self.vehicle_combo.blockSignals(False)
        self._load_profile_to_form(self.current_profile)
        self.route_name_edit.setText(self._next_route_name())
        self._refresh_route_combo()
        self.start_heading_spin.setValue(self.start_pose.heading_deg)
        self.end_heading_spin.setValue(0.0)
        self.dropoff_heading_spin.setValue(0.0)
        self._update_position_label()
        self._refresh_route_operations_table()
        self.redraw_scene()
        self.setWindowTitle(f"Vehicle Tracking - {path.stem}")
        missing = [level for level, drawing in self.level_drawing_paths.items() if not drawing.exists()]
        note = f"; missing DXFs: {', '.join(missing)}" if missing else ""
        self.statusBar().showMessage(
            f"Opened project with {len(self.levels)} floor(s), {len(self.saved_routes)} route(s), "
            f"and {len(self.vehicles)} vehicle profile(s){note}."
        )

    def import_dxf(self) -> None:
        filename, _ = QFileDialog.getOpenFileName(
            self,
            "Open Vehicle Tracking Project",
            str(ROOT),
            "Vehicle Tracking project (*.vtproject);;DXF files (*.dxf)",
        )
        if not filename:
            return
        path = Path(filename)
        if path.suffix.casefold() == ".vtproject":
            self._open_project_file(path)
            return
        try:
            opened_drawing = self._load_dxf_with_progress(path, "Opening DXF project")
        except Exception as exc:
            QMessageBox.critical(self, "Import failed", str(exc))
            return
        self.project_dxf_path = path
        self.project_file_path = None
        self.levels, self.start_positions, self.saved_routes = self.route_store.load_configuration(
            self.project_dxf_path
        )
        self.level_drawing_paths = self.route_store.load_level_drawings(self.project_dxf_path)
        first_start = self.start_positions[0]
        self.current_start_name = first_start.name
        self.current_level_name = first_start.level_name
        if self.current_level_name not in self.level_drawing_paths:
            self.level_drawing_paths[self.current_level_name] = path
        self.level_drawing_cache.clear()
        self._invalidate_shared_block_cache()
        if self.level_drawing_paths[self.current_level_name].resolve() == path.resolve():
            self.level_drawing_cache[self.current_level_name] = opened_drawing
        self._load_dxf_for_level(self.current_level_name)
        self.start_pose = Pose(first_start.pose.x, first_start.pose.y, first_start.pose.heading_deg)
        self.level_combo.blockSignals(True)
        self.level_combo.clear()
        self.level_combo.addItems(self.levels)
        self.level_combo.setCurrentText(self.current_level_name)
        self.level_combo.blockSignals(False)
        self.start_position_combo.blockSignals(True)
        self.start_position_combo.clear()
        self.start_position_combo.addItems([start.name for start in self.start_positions])
        self.start_position_combo.setCurrentText(self.current_start_name)
        self.start_position_combo.blockSignals(False)
        self.poses = [self.start_pose]
        self.end_pose = None
        self.dropoff_pose = None
        self.route_dropoff_waypoint_index = None
        self.route_waypoints.clear()
        self._selected_route_point_index = None
        self.route_point_turns.clear()
        self.route_reversing_actions.clear()
        self.route_tangent_handles.clear()
        self.route_point_path_modes.clear()
        self.route_start_operation = "travel"
        self.route_end_operation = "stop"
        self.active_route_index = None
        self.route_name_edit.setText(self._next_route_name())
        self._refresh_route_combo()
        self.stop_route_animation()
        self.start_heading_spin.setValue(self.start_pose.heading_deg)
        self.end_heading_spin.setValue(0.0)
        self.dropoff_heading_spin.setValue(0.0)
        self._update_position_label()
        self.redraw_scene()
        self._persist_routes()
        active_name = self.current_dxf.path.name if self.current_dxf else path.name
        detail = f"Opened project and assigned {active_name} to {self.current_level_name}"
        if self.current_dxf and self.current_dxf.unsupported_types:
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
            planned_exports = self.all_planned_route_exports()
            planned_route_names = [name for name, _poses in planned_exports]
            planned_routes = [poses for _name, poses in planned_exports]
            block_outline = self.block_outline_points(profile)
            block_drawing = self._drawing_with_block(profile.dxf_block_name)
            export_tracking_dxf(
                source_path=self.current_dxf.path if self.current_dxf else None,
                output_path=Path(filename),
                profile=profile,
                poses=self.poses,
                planned_poses=planned_poses,
                block_outline=block_outline,
                progress_callback=update_progress,
                planned_routes=planned_routes,
                planned_route_names=planned_route_names,
                block_source_path=block_drawing.path if block_drawing is not None else None,
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

    def _route_report_entries(self) -> list[RouteReportEntry]:
        profile = self.form_profile()
        entries: list[RouteReportEntry] = []
        report_drawings: dict[str, DxfDrawing | None] = {}

        def drawing_for_level(level_name: str) -> DxfDrawing | None:
            if level_name in report_drawings:
                return report_drawings[level_name]
            if (
                self.current_dxf is not None
                and level_name == self.current_level_name
            ):
                drawing = self.current_dxf
            else:
                drawing = self.level_drawing_cache.get(level_name)
                path = self.level_drawing_paths.get(level_name)
                if drawing is None and path is not None and path.exists():
                    try:
                        drawing = load_dxf(path)
                        self.level_drawing_cache[level_name] = drawing
                    except Exception:
                        drawing = None
            report_drawings[level_name] = drawing
            return drawing

        route_specs: list[tuple[RoutePlan, Pose]] = []
        if self.end_pose is not None:
            current = RoutePlan(
                self.route_name_edit.text().strip() or "Unsaved Path",
                Pose(self.end_pose.x, self.end_pose.y, self.end_pose.heading_deg),
                list(self.route_waypoints),
                sorted(self.route_point_turns),
                sorted(self.route_reversing_actions),
                self.current_level_name,
                self.current_start_name,
                Pose(self.start_pose.x, self.start_pose.y, self.start_pose.heading_deg),
                dict(self.route_tangent_handles),
                payload_action=self.route_end_operation,
                operations=self._current_ordered_operations(),
                dropoff_pose=self.dropoff_pose,
                point_path_modes=dict(self.route_point_path_modes),
                dropoff_waypoint_index=self.route_dropoff_waypoint_index,
            )
            route_specs.append((current, current.start_pose))
        for index, route in enumerate(self.saved_routes):
            if index == self.active_route_index and self.end_pose is not None:
                continue
            route_specs.append((route, self._start_pose_for_route(route)))
        for route, start in route_specs:
            level_drawing = drawing_for_level(route.level_name)
            _block_drawing, block_geometry = self._shared_block_geometry(
                profile.dxf_block_name
            )
            poses = self._planned_route_poses_for(
                route.end_pose,
                route.waypoints,
                set(route.point_turn_indices),
                set(route.reversing_action_indices),
                start,
                route.tangent_handles,
                route.dropoff_pose,
                route.point_path_modes,
                route.dropoff_waypoint_index,
                self._route_starts_reversing(route),
            )
            invalid, _curvatures, unsupported = self._route_section_analysis(poses, profile)
            notes = []
            operation_labels = {
                "travel": "travel",
                "straight": "straight section",
                "turn": "curved turn",
                "point_turn": "point turn",
                "reverse": "reverse",
                "pickup": "pick up payload",
                "dropoff": "drop off payload",
                "stop": "final stop",
            }
            notes.append(
                "operations: "
                + " -> ".join(
                    operation_labels.get(item.operation, item.operation)
                    for item in route.ordered_operations()
                )
            )
            if route.point_turn_indices:
                notes.append(f"{len(route.point_turn_indices)} point turn(s)")
            if route.reversing_action_indices:
                notes.append(f"{len(route.reversing_action_indices)} reverse action(s)")
            if route.dropoff_pose is not None:
                notes.append(
                    f"drop-off at X {route.dropoff_pose.x:.3f}, Y {route.dropoff_pose.y:.3f}; reverse exit to final position"
                )
            if unsupported:
                notes.append("vehicle does not support a configured point turn")
            pickup = self._payload_pickup_analysis(route, profile, poses)
            dropoff = self._payload_dropoff_analysis(route, profile, poses)
            if pickup is not None:
                notes.append(f"pickup check: {pickup.message}")
            if dropoff is not None:
                notes.append(f"drop-off check: {dropoff.message}")
            entries.append(
                RouteReportEntry(
                    route.name,
                    route.level_name,
                    route.start_position_name,
                    start,
                    route.end_pose,
                    poses,
                    bool(poses)
                    and not any(invalid)
                    and (pickup is None or pickup.possible)
                    and (dropoff is None or dropoff.possible),
                    sum(invalid),
                    ", ".join(notes),
                    level_drawing.primitives if level_drawing is not None else None,
                    block_geometry.primitives if block_geometry is not None else None,
                    [
                        (-profile.length / 2.0, -profile.width / 2.0),
                        (profile.length / 2.0, -profile.width / 2.0),
                        (profile.length / 2.0, profile.width / 2.0),
                        (-profile.length / 2.0, profile.width / 2.0),
                    ],
                    profile.block_forward_angle_deg,
                )
            )
        return entries

    def export_route_report(self) -> None:
        default = ROOT / "output" / "pdf" / "vehicle_route_report.pdf"
        filename, _ = QFileDialog.getSaveFileName(
            self, "Export Route Report", str(default), "PDF files (*.pdf)"
        )
        if not filename:
            return
        entries = self._route_report_entries()
        if not entries:
            QMessageBox.information(self, "No routes", "Create or save at least one path before generating a report.")
            return
        try:
            generate_route_report_pdf(
                Path(filename),
                entries,
                self.current_dxf.path.name if self.current_dxf else "Untitled drawing",
            )
        except Exception as exc:
            QMessageBox.critical(self, "Report failed", str(exc))
            return
        self.statusBar().showMessage(
            f"Generated route report with {sum(entry.feasible for entry in entries)} possible and "
            f"{sum(not entry.feasible for entry in entries)} impossible route(s): {filename}"
        )

    def export_route_mp4(self) -> None:
        route = self.planned_route_poses(self.form_profile())
        if len(route) < 2:
            QMessageBox.information(self, "Select a path", "Select or create a path before exporting an MP4.")
            return
        safe_name = "".join(character if character.isalnum() or character in "-_" else "_" for character in (self.route_name_edit.text().strip() or "path"))
        default = ROOT / "output" / "video" / f"{safe_name}.mp4"
        filename, _ = QFileDialog.getSaveFileName(
            self, "Export Path Animation", str(default), "MP4 video (*.mp4)"
        )
        if not filename:
            return
        width, height, fps = 1280, 720, 20
        xs = [pose.x for pose in route]
        ys = [pose.y for pose in route]
        profile = self.form_profile()
        margin = max(profile.length, profile.width, (max(xs) - min(xs)) * 0.05, (max(ys) - min(ys)) * 0.05, 1.0)
        source = QRectF(
            min(xs) - margin,
            -max(ys) - margin,
            max(max(xs) - min(xs) + margin * 2.0, 1.0),
            max(max(ys) - min(ys) + margin * 2.0, 1.0),
        )
        progress = QProgressDialog("Rendering path animation", None, 0, len(route), self)
        progress.setWindowTitle("Export MP4")
        progress.setWindowModality(Qt.WindowModality.WindowModal)
        progress.setMinimumDuration(0)
        self.stop_route_animation()
        self.route_animation_poses = route
        operation_labels = {
            "travel": "Travel",
            "straight": "Straight",
            "turn": "Turn",
            "point_turn": "Point turn",
            "reverse": "Reverse",
            "pickup": "Pick up payload",
            "dropoff": "Drop off payload",
            "stop": "Stop",
        }
        route_name = self.route_name_edit.text().strip() or "Unsaved Path"
        route_description = " -> ".join(
            operation_labels.get(item.operation, item.operation)
            for item in self._current_ordered_operations()
        )
        hidden_for_export = self._hide_non_selected_route_items_for_export()

        def frames():
            for index in range(len(route)):
                self._show_route_animation_frame(index)
                image = QImage(width, height, QImage.Format.Format_RGB888)
                image.fill(QColor("#ffffff"))
                painter = QPainter(image)
                painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
                self.scene.render(
                    painter,
                    QRectF(0.0, 0.0, float(width), float(height)),
                    source,
                    Qt.AspectRatioMode.KeepAspectRatio,
                )
                self._draw_route_video_overlay(
                    painter,
                    route,
                    source,
                    width,
                    height,
                    route_name,
                    self.current_level_name,
                    route_description,
                )
                painter.end()
                progress.setValue(index + 1)
                QApplication.processEvents()
                yield image

        try:
            frame_count = export_qimages_to_mp4(Path(filename), frames(), width, height, fps)
        except Exception as exc:
            QMessageBox.critical(self, "MP4 export failed", str(exc))
            return
        finally:
            progress.close()
            self.stop_route_animation()
            self._restore_items_after_route_export(hidden_for_export)
        self.statusBar().showMessage(f"Exported {frame_count} frames for the selected path to {filename}.")

    def _hide_non_selected_route_items_for_export(self) -> list[tuple[QGraphicsItem, bool]]:
        items: list[QGraphicsItem] = list(self.saved_route_items)
        if self.path_item is not None:
            items.append(self.path_item)
        items.extend(self.sweep_items)
        items.extend(self.vehicle_items)
        items.extend(
            item
            for item in self.payload_trace_items
            if item.data(0) == "driven-payload-trace"
        )
        items.extend(
            item
            for item in self.position_items
            if item.toolTip().startswith(
                (
                    "Saved endpoint",
                    "Adjacent path drop-off",
                    "Saved payload drop-off",
                    "Saved vehicle at payload drop-off",
                )
            )
        )
        items.extend(
            item
            for item in self.vehicle_items
            if item.data(0) == "saved-route-vehicle"
        )
        states: list[tuple[QGraphicsItem, bool]] = []
        seen: set[int] = set()
        for item in items:
            identity = id(item)
            if identity in seen or item.scene() is not self.scene:
                continue
            seen.add(identity)
            states.append((item, item.isVisible()))
            item.setVisible(False)
        return states

    @staticmethod
    def _restore_items_after_route_export(
        states: list[tuple[QGraphicsItem, bool]],
    ) -> None:
        for item, visible in states:
            if item.scene() is not None:
                item.setVisible(visible)

    @staticmethod
    def _route_video_overlay_rect(
        route: list[Pose],
        source: QRectF,
        width: int,
        height: int,
        panel_width: float,
        panel_height: float,
    ) -> QRectF:
        margin = 18.0
        candidates = [
            QRectF(width - panel_width - margin, margin, panel_width, panel_height),
            QRectF(margin, margin, panel_width, panel_height),
            QRectF(width - panel_width - margin, height - panel_height - margin, panel_width, panel_height),
            QRectF(margin, height - panel_height - margin, panel_width, panel_height),
        ]
        scale = min(width / max(source.width(), 1e-9), height / max(source.height(), 1e-9))
        rendered_width = source.width() * scale
        rendered_height = source.height() * scale
        offset_x = (width - rendered_width) * 0.5
        offset_y = (height - rendered_height) * 0.5
        route_points = [
            QPointF(
                offset_x + (pose.x - source.left()) * scale,
                offset_y + (-pose.y - source.top()) * scale,
            )
            for pose in route
        ]

        def obstruction_score(rect: QRectF) -> tuple[int, float]:
            protected = rect.adjusted(-16.0, -16.0, 16.0, 16.0)
            hits = sum(protected.contains(point) for point in route_points)
            nearest = min(
                (
                    hypot(
                        point.x() - protected.center().x(),
                        point.y() - protected.center().y(),
                    )
                    for point in route_points
                ),
                default=float("inf"),
            )
            return hits, -nearest

        return min(candidates, key=obstruction_score)

    def _draw_route_video_overlay(
        self,
        painter: QPainter,
        route: list[Pose],
        source: QRectF,
        width: int,
        height: int,
        route_name: str,
        level_name: str,
        description: str,
    ) -> None:
        panel_width = 460.0
        interface_font = QApplication.font().family()
        title_font = QFont(interface_font, 17)
        title_font.setBold(True)
        detail_font = QFont(interface_font, 12)
        detail_text = f"Level: {level_name}\nRoute: {description}"
        detail_bounds = QFontMetricsF(detail_font).boundingRect(
            QRectF(0.0, 0.0, panel_width - 28.0, 220.0),
            int(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop | Qt.TextFlag.TextWordWrap),
            detail_text,
        )
        panel_height = min(200.0, max(112.0, detail_bounds.height() + 58.0))
        panel = self._route_video_overlay_rect(
            route, source, width, height, panel_width, panel_height
        )
        painter.save()
        painter.setPen(QPen(QColor("#d7dee8"), 1.0))
        background = QColor("#172033")
        background.setAlpha(225)
        painter.setBrush(QBrush(background))
        painter.drawRoundedRect(panel, 9.0, 9.0)
        painter.setPen(QColor("#ffffff"))
        painter.setFont(title_font)
        painter.drawText(
            panel.adjusted(14.0, 10.0, -14.0, -10.0),
            int(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop),
            route_name,
        )
        painter.setFont(detail_font)
        painter.setPen(QColor("#e5edf8"))
        painter.drawText(
            panel.adjusted(14.0, 42.0, -14.0, -10.0),
            int(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop | Qt.TextFlag.TextWordWrap),
            detail_text,
        )
        painter.restore()

    def toggle_run(self) -> None:
        if self.timer.isActive():
            self.timer.stop()
            self._set_run_ui(False)
        else:
            self.stop_route_animation()
            self.timer.start()
            self._set_run_ui(True)

    def _set_run_ui(self, running: bool) -> None:
        text = "Pause" if running else "Run"
        icon = line_icon("stop" if running else "play", "#ffffff")
        self.run_action.setText(text)
        self.run_action.setIcon(icon)
        if hasattr(self, "run_button"):
            self.run_button.setText(text)
            self.run_button.setIcon(icon)

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

    def begin_place_dropoff(self) -> None:
        if self.end_pose is None:
            QMessageBox.information(
                self,
                "Place final position",
                "Place the final position first, then place the drop-off point that the vehicle will reverse away from.",
            )
            return
        self.view.set_placement_mode("dropoff", self.dropoff_heading_spin.value())
        self.statusBar().showMessage(
            "Place the payload drop-off point and drag in the required vehicle alignment; the vehicle will reverse from it to the final position."
        )

    def _begin_route_point_placement(self, point_kind: str, section: str) -> None:
        if self.end_pose is None:
            QMessageBox.information(self, "Place an end position", "Place the finish position before inserting route points.")
            return
        if section not in {"pre", "post"}:
            section = "pre"
        if self.dropoff_pose is None:
            QMessageBox.information(
                self,
                "Place a drop-off position",
                "Place the drop-off position before adding points to the before/after sections.",
            )
            return
        self._current_route_section = section
        if self.current_section_only_checkbox.isChecked():
            self.redraw_dynamic_layers(self.form_profile())
            self.redraw_route_handles()
        self.view.set_placement_mode(f"{section}_{point_kind}")
        side = "before" if section == "pre" else "after"
        description = {
            "route": "an orange curved-turn point",
            "straight_route": "a blue straight-section point",
            "reverse_action": "a red reversing action",
        }[point_kind]
        self.statusBar().showMessage(
            f"Click the {side} drop-off part of the planned route to insert {description}."
        )

    def begin_insert_route_point(self, section: str = "pre") -> None:
        self._begin_route_point_placement("route", section)

    def begin_draw_route(self) -> None:
        if self.end_pose is None:
            QMessageBox.information(
                self, "Place an end position", "Place the finish position before drawing a route."
            )
            return
        self.view.set_placement_mode("route_sketch")
        self.statusBar().showMessage(
            "Drag from the start toward the finish to sketch the route; release to calculate straight and turn points."
        )

    @staticmethod
    def _simplify_route_sketch(
        points: list[tuple[float, float]], tolerance: float
    ) -> list[tuple[float, float]]:
        if len(points) <= 2:
            return points
        start, end = points[0], points[-1]
        dx, dy = end[0] - start[0], end[1] - start[1]
        length_squared = dx * dx + dy * dy
        best_distance = -1.0
        best_index = 0
        for index, point in enumerate(points[1:-1], 1):
            if length_squared <= 1e-12:
                distance = hypot(point[0] - start[0], point[1] - start[1])
            else:
                amount = max(0.0, min(1.0, ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_squared))
                distance = hypot(
                    point[0] - (start[0] + amount * dx),
                    point[1] - (start[1] + amount * dy),
                )
            if distance > best_distance:
                best_distance, best_index = distance, index
        if best_distance <= tolerance:
            return [start, end]
        left = VehicleTrackerWindow._simplify_route_sketch(points[: best_index + 1], tolerance)
        right = VehicleTrackerWindow._simplify_route_sketch(points[best_index:], tolerance)
        return [*left[:-1], *right]

    def create_route_from_sketch(self, scene_points: list[QPointF]) -> None:
        if self.end_pose is None or len(scene_points) < 2:
            return
        profile = self.form_profile()
        sampled = [(float(point.x()), float(-point.y())) for point in scene_points]
        sampled[0] = (self.start_pose.x, self.start_pose.y)
        sampled[-1] = (self.end_pose.x, self.end_pose.y)
        stroke_span = hypot(
            max(point[0] for point in sampled) - min(point[0] for point in sampled),
            max(point[1] for point in sampled) - min(point[1] for point in sampled),
        )
        tolerance = max(
            0.02,
            min(
                max(profile.width * 0.12, profile.length * 0.025),
                max(stroke_span * 0.015, 0.02),
            ),
        )
        simplified = self._simplify_route_sketch(sampled, tolerance)
        if len(simplified) > 32:
            stride = ceil((len(simplified) - 2) / 30.0)
            simplified = [simplified[0], *simplified[1:-1:stride], simplified[-1]]
        self.stop_route_animation()
        connected_nodes, section_modes = self._connect_straight_sketch(
            simplified, profile.effective_min_turning_radius
        )
        self.route_waypoints = list(connected_nodes[1:-1])
        self.route_point_turns.clear()
        self.route_reversing_actions.clear()
        self.route_tangent_handles.clear()
        self.route_point_path_modes = section_modes
        self._selected_route_point_index = 0 if self.route_waypoints else None
        if self.dropoff_pose is not None:
            self.route_dropoff_waypoint_index = self._nearest_route_segment(
                connected_nodes, self.dropoff_pose.x, self.dropoff_pose.y
            )
        self.redraw_dynamic_layers(profile)
        self.redraw_route_handles()
        self._refresh_route_operations_table()
        self._update_navigation_bounds()
        straight_count = sum(mode == "straight" for mode in self.route_point_path_modes.values())
        turn_count = sum(mode == "minimum_radius" for mode in self.route_point_path_modes.values())
        self.statusBar().showMessage(
            f"Connected {straight_count} drawn straight section(s) with {turn_count} minimum-radius turn(s)."
        )

    @staticmethod
    def _connect_straight_sketch(
        vertices: list[tuple[float, float]], radius: float
    ) -> tuple[list[tuple[float, float]], dict[int, str]]:
        if len(vertices) < 2:
            return vertices, {}
        nodes = [vertices[0]]
        modes: list[str] = []
        for index in range(1, len(vertices) - 1):
            previous, corner, following = vertices[index - 1], vertices[index], vertices[index + 1]
            incoming = (corner[0] - previous[0], corner[1] - previous[1])
            outgoing = (following[0] - corner[0], following[1] - corner[1])
            incoming_length, outgoing_length = hypot(*incoming), hypot(*outgoing)
            if incoming_length < 1e-9 or outgoing_length < 1e-9:
                continue
            in_unit = (incoming[0] / incoming_length, incoming[1] / incoming_length)
            out_unit = (outgoing[0] / outgoing_length, outgoing[1] / outgoing_length)
            turn_angle = abs(atan2(
                in_unit[0] * out_unit[1] - in_unit[1] * out_unit[0],
                in_unit[0] * out_unit[0] + in_unit[1] * out_unit[1],
            ))
            tangent_distance = radius * abs(tan(turn_angle / 2.0))
            if turn_angle < radians(2.0) or tangent_distance >= min(incoming_length, outgoing_length) * 0.48:
                nodes.append(corner)
                modes.append("straight")
                continue
            entry = (
                corner[0] - in_unit[0] * tangent_distance,
                corner[1] - in_unit[1] * tangent_distance,
            )
            exit_point = (
                corner[0] + out_unit[0] * tangent_distance,
                corner[1] + out_unit[1] * tangent_distance,
            )
            nodes.extend((entry, exit_point))
            modes.extend(("straight", "minimum_radius"))
        nodes.append(vertices[-1])
        modes.append("straight")
        return nodes, {index: mode for index, mode in enumerate(modes)}

    def begin_insert_straight_point(self, section: str = "pre") -> None:
        self._begin_route_point_placement("straight_route", section)

    def begin_place_reverse_action(self, section: str = "pre") -> None:
        self._begin_route_point_placement("reverse_action", section)

    def create_alignment_point_suggestions(self) -> None:
        if self.end_pose is None or self.dropoff_pose is None:
            QMessageBox.information(
                self,
                "Place all positions",
                "Place the Start, Drop-off, and Final positions before creating alignment suggestions.",
            )
            return
        self.stop_route_animation()
        profile = self.form_profile()
        straight_distance = max(
            profile.length,
            profile.wheelbase,
            profile.effective_min_turning_radius,
            profile.payload_length if profile.payload_enabled else 0.0,
            0.5,
        )
        heading = radians(self.dropoff_pose.heading_deg)
        backward = (-cos(heading), -sin(heading))
        factors = (1.0, 1.25, 1.5, 2.0, 2.5, 3.0, 4.0, 6.0, 8.0, 12.0)
        strategy = str(self.alignment_strategy_combo.currentData())
        reverse_options = (
            (False, True)
            if strategy == "auto"
            else (True,)
            if strategy == "resume_forward"
            else (False,)
        )
        candidates = sorted(
            (
                (approach, egress, resume_forward)
                for approach in factors
                for egress in factors
                for resume_forward in reverse_options
                if egress > approach
            ),
            key=lambda item: (item[0] + item[1], max(item[:2]), item[2]),
        )
        winning = None
        failure_counts = {"curvature": 0, "delivery": 0, "egress": 0}
        for approach_factor, egress_factor, resume_forward in candidates:
            approach_distance = straight_distance * approach_factor
            egress_distance = straight_distance * egress_factor
            result, reason = self._alignment_candidate_state(
                approach_distance,
                egress_distance,
                heading,
                backward,
                profile,
                resume_forward,
            )
            if result is not None:
                winning = (
                    result,
                    approach_distance,
                    egress_distance,
                    resume_forward,
                )
                break
            failure_counts[reason] += 1
        if winning is None:
            detail = ", ".join(
                f"{count} {reason} failure(s)"
                for reason, count in failure_counts.items()
                if count
            )
            QMessageBox.warning(
                self,
                "No feasible alignment suggestion",
                "No delivery/egress point pair passed all route criteria within the search range. "
                f"Checked {len(candidates)} combinations: {detail}. Adjust the three positions, "
                "drop-off heading, or vehicle turning constraints and try again.",
            )
            self.statusBar().showMessage("No alignment suggestion met all route criteria.")
            return
        state, approach_distance, egress_distance, resume_forward = winning
        (
            self.route_waypoints,
            self.route_point_turns,
            self.route_reversing_actions,
            self.route_tangent_handles,
            self.route_point_path_modes,
            self.route_dropoff_waypoint_index,
            approach_index,
            egress_index,
        ) = state
        self._selected_route_point_index = egress_index
        self.redraw_dynamic_layers(profile)
        self.redraw_route_handles()
        self._refresh_route_operations_table()
        self._update_navigation_bounds()
        self.statusBar().showMessage(
            f"Added feasible delivery and reverse-egress suggestions at {approach_distance:.3f} "
            f"and {egress_distance:.3f} from the drop-off point; "
            f"{'forward travel resumes at the egress point' if resume_forward else 'reverse travel continues to the final position'}."
        )

    def _alignment_candidate_state(
        self,
        approach_distance: float,
        egress_distance: float,
        heading: float,
        backward: tuple[float, float],
        profile: VehicleProfile,
        resume_forward: bool = False,
    ) -> tuple[tuple, str]:
        assert self.end_pose is not None and self.dropoff_pose is not None
        approach = (
            self.dropoff_pose.x + backward[0] * approach_distance,
            self.dropoff_pose.y + backward[1] * approach_distance,
        )
        egress = (
            self.dropoff_pose.x + backward[0] * egress_distance,
            self.dropoff_pose.y + backward[1] * egress_distance,
        )
        waypoints = list(self.route_waypoints)
        point_turns = set(self.route_point_turns)
        reverses = set(self.route_reversing_actions)
        tangents = dict(self.route_tangent_handles)
        modes = dict(self.route_point_path_modes)
        dropoff_index = self._effective_dropoff_waypoint_index()

        def shift(index: int) -> None:
            nonlocal point_turns, reverses, tangents, modes
            point_turns = {item + 1 if item >= index else item for item in point_turns}
            reverses = {item + 1 if item >= index else item for item in reverses}
            tangents = {
                (item + 1 if item >= index else item): vector
                for item, vector in tangents.items()
            }
            modes = {
                (item + 1 if item >= index else item): mode
                for item, mode in modes.items()
            }

        tolerance = max(min(approach_distance, egress_distance) * 1e-6, 1e-6)
        if dropoff_index > 0 and hypot(
            waypoints[dropoff_index - 1][0] - approach[0],
            waypoints[dropoff_index - 1][1] - approach[1],
        ) <= tolerance:
            approach_index = dropoff_index - 1
            waypoints[approach_index] = approach
        else:
            shift(dropoff_index)
            waypoints.insert(dropoff_index, approach)
            approach_index = dropoff_index
            dropoff_index += 1
        if dropoff_index < len(waypoints) and hypot(
            waypoints[dropoff_index][0] - egress[0],
            waypoints[dropoff_index][1] - egress[1],
        ) <= tolerance:
            egress_index = dropoff_index
            waypoints[egress_index] = egress
        else:
            shift(dropoff_index)
            waypoints.insert(dropoff_index, egress)
            egress_index = dropoff_index
        modes[approach_index] = "straight"
        modes[egress_index] = "straight"
        if resume_forward:
            reverses.add(egress_index)
        else:
            reverses.discard(egress_index)
        tangents[approach_index] = (
            cos(heading) * approach_distance * 0.5,
            sin(heading) * approach_distance * 0.5,
        )
        tangents[egress_index] = (
            backward[0] * egress_distance * 0.5,
            backward[1] * egress_distance * 0.5,
        )
        poses = self._planned_route_poses_for(
            self.end_pose,
            waypoints,
            point_turns,
            reverses,
            None,
            tangents,
            self.dropoff_pose,
            modes,
            dropoff_index,
            self.route_start_operation == "reverse",
        )
        invalid, _curvatures, unsupported = self._route_section_analysis(poses, profile)
        if not poses or any(invalid) or unsupported:
            return None, "curvature"
        plan = RoutePlan(
            "Alignment candidate",
            self.end_pose,
            waypoints,
            sorted(point_turns),
            sorted(reverses),
            start_pose=self.start_pose,
            tangent_handles=tangents,
            dropoff_pose=self.dropoff_pose,
            point_path_modes=modes,
            dropoff_waypoint_index=dropoff_index,
        )
        delivery = self._payload_dropoff_analysis(plan, profile, poses)
        if delivery is None or not delivery.possible:
            return None, "delivery"
        marker = next(
            (index for index, pose in enumerate(poses) if pose.maneuver == "dropoff"),
            None,
        )
        reverse_distance = self._straight_reverse_egress_distance(
            poses, marker, self.dropoff_pose.heading_deg
        )
        required = max(profile.length, profile.payload_length, 0.5)
        if reverse_distance + 1e-9 < required:
            return None, "egress"
        return (
            waypoints,
            point_turns,
            reverses,
            tangents,
            modes,
            dropoff_index,
            approach_index,
            egress_index,
        ), ""

    def _straight_reverse_egress_distance(
        self, poses: list[Pose], marker: int | None, heading_deg: float
    ) -> float:
        if marker is None:
            return 0.0
        distance = 0.0
        for first, second in zip(poses[marker:], poses[marker + 1 :]):
            dx, dy = second.x - first.x, second.y - first.y
            step = hypot(dx, dy)
            if step < 1e-9:
                continue
            motion_heading = degrees(atan2(dy, dx))
            if (
                second.maneuver != "reverse"
                or self._axis_heading_error(motion_heading, heading_deg) > 2.0
            ):
                break
            distance += step
        return distance

    def _shift_route_point_metadata_for_insert(self, index: int) -> None:
        self.route_point_turns = {
            item + 1 if item >= index else item for item in self.route_point_turns
        }
        self.route_reversing_actions = {
            item + 1 if item >= index else item
            for item in self.route_reversing_actions
        }
        self.route_tangent_handles = {
            (item + 1 if item >= index else item): vector
            for item, vector in self.route_tangent_handles.items()
        }
        self.route_point_path_modes = {
            (item + 1 if item >= index else item): mode
            for item, mode in self.route_point_path_modes.items()
        }

    @staticmethod
    def _nearest_route_segment(
        nodes: list[tuple[float, float]], x: float, y: float
    ) -> int:
        best_index = 0
        best_distance = float("inf")
        for index, (start, end) in enumerate(zip(nodes, nodes[1:])):
            dx, dy = end[0] - start[0], end[1] - start[1]
            length_squared = dx * dx + dy * dy
            if length_squared <= 1e-12:
                projected_x, projected_y = start
            else:
                amount = max(
                    0.0,
                    min(
                        1.0,
                        ((x - start[0]) * dx + (y - start[1]) * dy) / length_squared,
                    ),
                )
                projected_x = start[0] + amount * dx
                projected_y = start[1] + amount * dy
            distance = (x - projected_x) ** 2 + (y - projected_y) ** 2
            if distance < best_distance:
                best_distance = distance
                best_index = index
        return best_index

    @staticmethod
    def _project_to_route_nodes(
        nodes: list[tuple[float, float]], x: float, y: float
    ) -> tuple[int, tuple[float, float]]:
        segment = VehicleTrackerWindow._nearest_route_segment(nodes, x, y)
        start, end = nodes[segment], nodes[segment + 1]
        dx, dy = end[0] - start[0], end[1] - start[1]
        length_squared = dx * dx + dy * dy
        amount = 0.0 if length_squared <= 1e-12 else max(
            0.0,
            min(1.0, ((x - start[0]) * dx + (y - start[1]) * dy) / length_squared),
        )
        return segment, (start[0] + amount * dx, start[1] + amount * dy)

    def place_position(self, kind: str, scene_position: QPointF, heading_deg: float = 0.0) -> None:
        x = float(scene_position.x())
        y = float(-scene_position.y())
        snap_note = ""
        self.stop_route_animation()
        point_kind = next(
            (value for value in ("straight_route", "reverse_action", "route") if kind.endswith(value)),
            None,
        )
        if point_kind is not None:
            dropoff_index = self._effective_dropoff_waypoint_index()
            section = "post" if kind.startswith("post_") else "pre"
            if self.dropoff_pose is not None and section == "pre":
                section_nodes = [
                    (self.start_pose.x, self.start_pose.y),
                    *self.route_waypoints[:dropoff_index],
                    (self.dropoff_pose.x, self.dropoff_pose.y),
                ]
                local_segment, inserted_position = self._project_to_route_nodes(
                    section_nodes, x, y
                )
                segment_index = local_segment
                self.route_dropoff_waypoint_index = dropoff_index + 1
            elif self.dropoff_pose is not None and section == "post":
                section_nodes = [
                    (self.dropoff_pose.x, self.dropoff_pose.y),
                    *self.route_waypoints[dropoff_index:],
                    (self.end_pose.x, self.end_pose.y),
                ]
                local_segment, inserted_position = self._project_to_route_nodes(
                    section_nodes, x, y
                )
                segment_index = dropoff_index + local_segment
            else:
                section_nodes = [
                    (self.start_pose.x, self.start_pose.y),
                    *self.route_waypoints,
                    (self.end_pose.x, self.end_pose.y),
                ]
                segment_index, inserted_position = self._project_to_route_nodes(
                    section_nodes, x, y
                )
            self.route_point_turns = {
                index + 1 if index >= segment_index else index
                for index in self.route_point_turns
            }
            self.route_reversing_actions = {
                index + 1 if index >= segment_index else index
                for index in self.route_reversing_actions
            }
            self.route_tangent_handles = {
                (index + 1 if index >= segment_index else index): vector
                for index, vector in self.route_tangent_handles.items()
            }
            self.route_point_path_modes = {
                (index + 1 if index >= segment_index else index): mode
                for index, mode in self.route_point_path_modes.items()
            }
            self.route_waypoints.insert(segment_index, inserted_position)
            if point_kind == "reverse_action":
                self.route_reversing_actions.add(segment_index)
            elif point_kind == "straight_route":
                self.route_point_path_modes[segment_index] = "straight"
            self._selected_route_point_index = segment_index
            self.redraw_dynamic_layers(self.form_profile())
            self.redraw_route_handles()
            self._refresh_route_operations_table()
            self._update_navigation_bounds()
            if point_kind == "reverse_action":
                message = (
                    f"Placed reversing action {segment_index + 1}; drag the red point to adjust it."
                )
            elif point_kind == "straight_route":
                message = (
                    f"Inserted straight-section point {segment_index + 1}; drag the blue point to adjust it."
                )
            else:
                message = f"Inserted route point {segment_index + 1}; drag it to tighten the path."
            self.statusBar().showMessage(message)
            return
        if kind == "dropoff":
            self.dropoff_pose = Pose(x, y, heading_deg, 0.0)
            self.route_dropoff_waypoint_index = len(self.route_waypoints)
            self.dropoff_heading_spin.blockSignals(True)
            self.dropoff_heading_spin.setValue(heading_deg)
            self.dropoff_heading_spin.blockSignals(False)
            self.route_end_operation = "stop"
            self._update_position_label()
            self._refresh_route_operations_table()
            self.redraw_dynamic_layers(self.form_profile())
            self.redraw_position_markers()
            self.redraw_route_handles()
            self._update_navigation_bounds()
            self.statusBar().showMessage(
                f"Drop-off set to X {x:.3f}, Y {y:.3f}, heading {heading_deg:.1f} deg; reverse exit added to final position."
            )
            return
        if kind == "start":
            if self.timer.isActive():
                self.timer.stop()
                self._set_run_ui(False)
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
        if self.sender() is self.dropoff_heading_spin and self.dropoff_pose is not None:
            self.dropoff_pose.heading_deg = self.dropoff_heading_spin.value()
        elif self.sender() is self.end_heading_spin and self.end_pose is not None:
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

    def toggle_other_paths_visibility(self, checked: bool) -> None:
        self.settings.setValue("visibility/show_other_paths", checked)
        if self.poses:
            self.redraw_dynamic_layers(self.form_profile())
            self.redraw_position_markers()
            self._update_navigation_bounds()

    def toggle_current_section_visibility(self, _checked: bool) -> None:
        if self.poses:
            self.redraw_dynamic_layers(self.form_profile())
            self.redraw_route_handles()
            self._update_navigation_bounds()

    def toggle_route_animation(self) -> None:
        if self.route_animation_timer.isActive():
            self.stop_route_animation()
            return
        if self.route_animation_poses and self.route_animation_paused:
            if self.route_animation_index >= len(self.route_animation_poses) - 1:
                self.route_animation_index = 0
            self.route_animation_paused = False
            self.route_animation_timer.start()
            self.animate_route_button.setText("Stop Animation")
            self.pause_route_button.setText("Pause")
            return
        route = self.planned_route_poses(self.form_profile())
        if len(route) < 2:
            QMessageBox.information(self, "Create a route", "Place start and finish positions before animating the route.")
            return
        if self.timer.isActive():
            self.timer.stop()
            self._set_run_ui(False)
        self.show_route_checkbox.setChecked(True)
        self.route_animation_poses = route
        self.route_animation_index = 0
        self.route_animation_paused = False
        self.route_animation_slider.blockSignals(True)
        self.route_animation_slider.setRange(0, len(route) - 1)
        self.route_animation_slider.setValue(0)
        self.route_animation_slider.blockSignals(False)
        self.route_animation_slider.setEnabled(True)
        self.pause_route_button.setEnabled(True)
        self.pause_route_button.setText("Pause")
        self.animate_route_button.setText("Stop Animation")
        self.animate_route_button.setIcon(line_icon("stop", "#ffffff"))
        self.route_animation_timer.start()
        self.advance_route_animation()

    def advance_route_animation(self) -> None:
        if not self.route_animation_poses:
            self.stop_route_animation()
            return
        self._show_route_animation_frame(self.route_animation_index)
        self.route_animation_slider.blockSignals(True)
        self.route_animation_slider.setValue(self.route_animation_index)
        self.route_animation_slider.blockSignals(False)
        self.route_animation_index += 1
        if self.route_animation_index >= len(self.route_animation_poses):
            self.route_animation_timer.stop()
            self.route_animation_index = len(self.route_animation_poses) - 1
            self.route_animation_paused = True
            self.animate_route_button.setText("Replay Route")
            self.animate_route_button.setIcon(line_icon("play", "#ffffff"))
            self.pause_route_button.setText("Resume")

    def _show_route_animation_frame(self, index: int) -> None:
        if not self.route_animation_poses:
            return
        index = max(0, min(index, len(self.route_animation_poses) - 1))
        if self.route_animation_item is not None and self.route_animation_item.scene() is self.scene:
            self.scene.removeItem(self.route_animation_item)
        pose = self.route_animation_poses[index]
        self.route_animation_item = self.draw_vehicle(
            self.form_profile(),
            pose,
            ghost=False,
            direction_override=(
                0 if pose.maneuver == "point_turn" else -1 if pose.maneuver == "reverse" else 1
            ),
        )
        self.route_animation_item.setOpacity(0.82)
        self.route_animation_item.setZValue(30.0)

    def pause_route_animation(self) -> None:
        if not self.route_animation_poses:
            return
        if self.route_animation_timer.isActive():
            self.route_animation_timer.stop()
            self.route_animation_paused = True
            self.pause_route_button.setText("Resume")
            self.animate_route_button.setText("Resume Route")
        else:
            self.route_animation_paused = False
            self.route_animation_timer.start()
            self.pause_route_button.setText("Pause")
            self.animate_route_button.setText("Stop Animation")

    def scrub_route_animation(self, index: int) -> None:
        if not self.route_animation_poses:
            return
        self.route_animation_timer.stop()
        self.route_animation_paused = True
        self.route_animation_index = max(0, min(index, len(self.route_animation_poses) - 1))
        self._show_route_animation_frame(self.route_animation_index)
        self.pause_route_button.setText("Resume")
        self.animate_route_button.setText("Resume Route")

    def stop_route_animation(self) -> None:
        self.route_animation_timer.stop()
        self.route_animation_poses = []
        self.route_animation_index = 0
        self.route_animation_paused = False
        if self.route_animation_item is not None and self.route_animation_item.scene() is self.scene:
            self.scene.removeItem(self.route_animation_item)
        self.route_animation_item = None
        if hasattr(self, "animate_route_button"):
            self.animate_route_button.setText("Animate Route")
            self.animate_route_button.setIcon(line_icon("play", "#ffffff"))
            self.pause_route_button.setText("Pause")
            self.pause_route_button.setEnabled(False)
            self.route_animation_slider.blockSignals(True)
            self.route_animation_slider.setRange(0, 0)
            self.route_animation_slider.setValue(0)
            self.route_animation_slider.blockSignals(False)
            self.route_animation_slider.setEnabled(False)

    def place_wheels_on_block(self) -> None:
        block_name = self.block_combo.currentText().strip()
        block_drawing, geometry = self._shared_block_geometry(block_name)
        if block_drawing is None:
            QMessageBox.information(self, "Import a DXF", "Import a DXF before placing wheels on a block.")
            return
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
            f"{self.current_level_name} / {self.current_start_name}: "
            f"{self.start_pose.x:.3f}, {self.start_pose.y:.3f} "
            f"@ {self.start_pose.heading_deg:.1f}°"
        )
        end = "End: not placed"
        if self.end_pose is not None:
            end = f"End: {self.end_pose.x:.3f}, {self.end_pose.y:.3f} @ {self.end_pose.heading_deg:.1f}°"
        dropoff = "Drop-off: not placed"
        if self.dropoff_pose is not None:
            dropoff = (
                f"Drop-off: {self.dropoff_pose.x:.3f}, {self.dropoff_pose.y:.3f} "
                f"@ {self.dropoff_pose.heading_deg:.1f} deg"
            )
        saved = f"Saved paths: {len(self.saved_routes)}"
        self.position_label.setText(f"{start}    {dropoff}    {end}    {saved}")
        if hasattr(self, "suggest_alignment_button"):
            self.suggest_alignment_button.setEnabled(
                self.end_pose is not None and self.dropoff_pose is not None
            )

    def _current_ordered_operations(self) -> list[RouteOperation]:
        operations = [RouteOperation("start", self.route_start_operation)]
        dropoff_index = self._effective_dropoff_waypoint_index()
        for index in range(len(self.route_waypoints)):
            if self.dropoff_pose is not None and index == dropoff_index:
                operations.append(RouteOperation("dropoff", "dropoff"))
                operations.append(RouteOperation("dropoff", "reverse"))
            operation = (
                "point_turn"
                if index in self.route_point_turns
                else "reverse"
                if index in self.route_reversing_actions
                else self.route_point_path_modes.get(index, "turn")
            )
            operations.append(RouteOperation("waypoint", operation, index))
        if self.dropoff_pose is not None and dropoff_index == len(self.route_waypoints):
            operations.append(RouteOperation("dropoff", "dropoff"))
            operations.append(RouteOperation("dropoff", "reverse"))
        operations.append(RouteOperation("end", self.route_end_operation))
        return operations

    def _effective_dropoff_waypoint_index(self) -> int:
        if self.dropoff_pose is None or self.route_dropoff_waypoint_index is None:
            return len(self.route_waypoints)
        return min(max(self.route_dropoff_waypoint_index, 0), len(self.route_waypoints))

    def _refresh_route_operations_table(self) -> None:
        if not hasattr(self, "route_operations_table"):
            return
        self._updating_operation_table = True
        operations = self._current_ordered_operations()
        self.route_operations_table.setRowCount(len(operations))
        labels = {
            "travel": "Travel",
            "straight": "Straight section",
            "turn": "Curved turn",
            "minimum_radius": "Minimum-radius turn",
            "point_turn": "Driven-wheel point turn",
            "reverse": "Reverse direction",
            "pickup": "Pick up payload",
            "dropoff": "Drop off payload",
            "stop": "Stop",
        }
        for row, operation in enumerate(operations):
            order_item = QTableWidgetItem(str(row + 1))
            order_item.setFlags(order_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
            point_name = (
                "Path start"
                if operation.location == "start"
                else "Drop-off point"
                if operation.location == "dropoff"
                else "Path end"
                if operation.location == "end"
                else f"Route point {operation.waypoint_index + 1}"
            )
            point_item = QTableWidgetItem(point_name)
            point_item.setFlags(point_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
            point_item.setData(
                Qt.ItemDataRole.UserRole,
                (operation.location, operation.waypoint_index),
            )
            combo = QComboBox()
            choices = (
                [("Drop off payload", "dropoff")]
                if operation.location == "dropoff" and operation.operation == "dropoff"
                else [("Reverse out", "reverse")]
                if operation.location == "dropoff"
                else
                [("Travel forward", "travel"), ("Start reversing", "reverse")]
                if operation.location == "start"
                else [("Final stop", "stop"), ("Pick up payload", "pickup")]
                if operation.location == "end"
                else [
                    ("Straight section", "straight"),
                    ("Minimum-radius turn", "minimum_radius"),
                    ("Curved turn", "turn"),
                    ("Driven-wheel point turn", "point_turn"),
                    ("Reverse direction", "reverse"),
                ]
            )
            for text_value, data_value in choices:
                combo.addItem(text_value, data_value)
            combo.setCurrentIndex(max(0, combo.findData(operation.operation)))
            if operation.location == "dropoff":
                combo.setEnabled(False)
            combo.currentIndexChanged.connect(
                lambda _index, location=operation.location, waypoint=operation.waypoint_index, control=combo: self._route_operation_control_changed(
                    location, waypoint, str(control.currentData())
                )
            )
            combo.setToolTip(f"Ordered operation {row + 1}: {labels.get(operation.operation, operation.operation)}")
            self.route_operations_table.setItem(row, 0, order_item)
            self.route_operations_table.setItem(row, 1, point_item)
            self.route_operations_table.setCellWidget(row, 2, combo)
            if (
                operation.location == "waypoint"
                and operation.waypoint_index == self._selected_route_point_index
            ):
                self.route_operations_table.selectRow(row)
        self._updating_operation_table = False
        self._update_route_point_order_buttons()

    def _operation_selection_changed(self) -> None:
        if self._updating_operation_table:
            return
        selected = self.route_operations_table.selectedItems()
        if not selected:
            self._selected_route_point_index = None
            self._update_route_point_order_buttons()
            return
        point_item = self.route_operations_table.item(selected[0].row(), 1)
        location, waypoint_index = point_item.data(Qt.ItemDataRole.UserRole)
        if location == "waypoint" and waypoint_index is not None:
            self._select_route_point(int(waypoint_index), select_table=False)
        else:
            self._selected_route_point_index = None
            for item in self.route_point_items:
                item.setSelected(False)
            self._update_route_point_order_buttons()

    def _select_route_point(self, index: int, *, select_table: bool = True) -> None:
        if not 0 <= index < len(self.route_waypoints):
            return
        self._selected_route_point_index = index
        section = "pre" if index < self._effective_dropoff_waypoint_index() else "post"
        section_changed = section != self._current_route_section
        self._current_route_section = section
        for item in self.route_point_items:
            item.setSelected(item.index == index)
        if select_table:
            self._updating_operation_table = True
            for row in range(self.route_operations_table.rowCount()):
                point_item = self.route_operations_table.item(row, 1)
                if point_item is None:
                    continue
                location, waypoint_index = point_item.data(Qt.ItemDataRole.UserRole)
                if location == "waypoint" and waypoint_index == index:
                    self.route_operations_table.selectRow(row)
                    break
            self._updating_operation_table = False
        self._update_route_point_order_buttons()
        if section_changed and self.current_section_only_checkbox.isChecked():
            self.redraw_dynamic_layers(self.form_profile())
            self.redraw_route_handles()

    def _route_poses_for_current_section(self, poses: list[Pose]) -> list[Pose]:
        if (
            not self.current_section_only_checkbox.isChecked()
            or self.dropoff_pose is None
            or len(poses) < 2
        ):
            return poses
        split_index = min(
            range(len(poses)),
            key=lambda index: (
                (poses[index].x - self.dropoff_pose.x) ** 2
                + (poses[index].y - self.dropoff_pose.y) ** 2
            ),
        )
        if self._current_route_section == "post":
            return poses[split_index:]
        return poses[: split_index + 1]

    def _update_route_point_order_buttons(self) -> None:
        if not hasattr(self, "move_route_point_up_button"):
            return
        index = self._selected_route_point_index
        dropoff_index = self._effective_dropoff_waypoint_index()
        section_start = dropoff_index if index is not None and index >= dropoff_index else 0
        section_end = (
            len(self.route_waypoints) - 1
            if index is not None and index >= dropoff_index
            else dropoff_index - 1
        )
        self.move_route_point_up_button.setEnabled(
            index is not None and index > section_start
        )
        self.move_route_point_down_button.setEnabled(
            index is not None and index < section_end
        )

    def move_selected_route_point(self, offset: int) -> None:
        index = self._selected_route_point_index
        row = self.route_operations_table.currentRow()
        if row >= 0:
            point_item = self.route_operations_table.item(row, 1)
            if point_item is not None:
                location, waypoint_index = point_item.data(Qt.ItemDataRole.UserRole)
                if location == "waypoint" and waypoint_index is not None:
                    index = int(waypoint_index)
        if index is None or offset not in {-1, 1}:
            return
        destination = index + offset
        dropoff_index = self._effective_dropoff_waypoint_index()
        same_section = (index < dropoff_index) == (destination < dropoff_index)
        if not 0 <= destination < len(self.route_waypoints) or not same_section:
            return
        self.stop_route_animation()
        moved_point = self.route_waypoints.pop(index)
        self.route_waypoints.insert(destination, moved_point)
        remap = {index: destination, destination: index}
        self.route_point_turns = {remap.get(item, item) for item in self.route_point_turns}
        self.route_reversing_actions = {
            remap.get(item, item) for item in self.route_reversing_actions
        }
        self.route_tangent_handles = {
            remap.get(item, item): vector
            for item, vector in self.route_tangent_handles.items()
        }
        self.route_point_path_modes = {
            remap.get(item, item): mode
            for item, mode in self.route_point_path_modes.items()
        }
        self._selected_route_point_index = destination
        self.redraw_dynamic_layers(self.form_profile())
        self.redraw_route_handles()
        self._refresh_route_operations_table()
        self._update_navigation_bounds()
        self.statusBar().showMessage(
            f"Moved route point to operation position {destination + 1}."
        )

    def _route_operation_changed(
        self, location: str, waypoint_index: int | None, operation: str
    ) -> None:
        if self._updating_operation_table:
            return
        if location == "start":
            self.route_start_operation = operation
        elif location == "end":
            self.route_end_operation = operation
        elif waypoint_index is not None:
            self.route_point_turns.discard(waypoint_index)
            self.route_reversing_actions.discard(waypoint_index)
            self.route_point_path_modes.pop(waypoint_index, None)
            if operation == "point_turn":
                self.route_point_turns.add(waypoint_index)
            elif operation == "reverse":
                self.route_reversing_actions.add(waypoint_index)
            elif operation in {"straight", "turn", "minimum_radius"}:
                self.route_point_path_modes[waypoint_index] = operation
        self.stop_route_animation()
        self.redraw_dynamic_layers(self.form_profile())
        self.redraw_route_handles()
        self._refresh_route_operations_table()

    def _route_operation_control_changed(
        self, location: str, waypoint_index: int | None, operation: str
    ) -> None:
        if location == "waypoint" and waypoint_index is not None:
            self._select_route_point(int(waypoint_index))
        self._route_operation_changed(location, waypoint_index, operation)

    @staticmethod
    def _axis_heading_error(first_deg: float, second_deg: float) -> float:
        return abs(((first_deg - second_deg + 90.0) % 180.0) - 90.0)

    @staticmethod
    def _operation_at(route: RoutePlan, location: str) -> str:
        return next(
            (
                operation.operation
                for operation in route.ordered_operations()
                if operation.location == location
            ),
            "stop" if location == "end" else "travel",
        )

    @staticmethod
    def _route_starts_reversing(route: RoutePlan) -> bool:
        return any(
            operation.location == "start" and operation.operation == "reverse"
            for operation in route.ordered_operations()
        )

    def _payload_pickup_analysis(
        self,
        route: RoutePlan,
        profile: VehicleProfile,
        poses: list[Pose] | None = None,
    ) -> PayloadPickupAnalysis | None:
        pickup_location = next(
            (
                location
                for location in ("start", "end")
                if self._operation_at(route, location) == "pickup"
            ),
            None,
        )
        if pickup_location is None:
            return None
        if pickup_location == "start":
            return PayloadPickupAnalysis(
                False,
                "Pickup is at the path start, so no approach is represented. Set Pick up payload on the path end to calculate the straight approach.",
            )
        pickup_pose = route.end_pose
        pickup_center = pickup_pose.transformed_point(profile.payload_x, profile.payload_y)
        tolerance = max(0.001, min(profile.payload_length, profile.payload_width) * 0.02)
        candidates: list[tuple[float, RoutePlan]] = []
        for candidate in self.saved_routes:
            if candidate is route or candidate.level_name != route.level_name:
                continue
            if candidate.dropoff_pose is None:
                continue
            drop_center = candidate.dropoff_pose.transformed_point(
                profile.payload_x, profile.payload_y
            )
            error = hypot(pickup_center[0] - drop_center[0], pickup_center[1] - drop_center[1])
            candidates.append((error, candidate))
        if not candidates:
            return PayloadPickupAnalysis(
                False,
                "No saved drop-off point exists on this floor for the pickup endpoint.",
            )
        position_error, dropoff_route = min(candidates, key=lambda item: item[0])
        if position_error > tolerance:
            return PayloadPickupAnalysis(
                False,
                f"Nearest drop-off is {position_error:.3f} away; pickup tolerance is {tolerance:.3f}.",
                dropoff_route.name,
                position_error=position_error,
            )
        dropoff_pose = dropoff_route.dropoff_pose
        assert dropoff_pose is not None
        payload_drop_heading = dropoff_pose.heading_deg + profile.payload_rotation_deg
        payload_pickup_heading = pickup_pose.heading_deg + profile.payload_rotation_deg
        alignment_error = self._axis_heading_error(
            payload_pickup_heading, payload_drop_heading
        )
        alignment_tolerance = 2.0
        if alignment_error > alignment_tolerance:
            return PayloadPickupAnalysis(
                False,
                f"Vehicle/payload alignment differs by {alignment_error:.1f} deg; maximum is {alignment_tolerance:.1f} deg.",
                dropoff_route.name,
                position_error,
                alignment_error,
            )
        if poses is None:
            poses = self._planned_route_poses_for(
                route.end_pose,
                route.waypoints,
                set(route.point_turn_indices),
                set(route.reversing_action_indices),
                self._start_pose_for_route(route),
                route.tangent_handles,
                route.dropoff_pose,
                route.point_path_modes,
                route.dropoff_waypoint_index,
                self._route_starts_reversing(route),
            )
        straight_distance = self._straight_inline_approach_distance(
            poses, pickup_pose.heading_deg
        )
        required_distance = max(profile.length, profile.payload_length)
        if straight_distance + 1e-9 < required_distance:
            return PayloadPickupAnalysis(
                False,
                f"Only {straight_distance:.3f} of straight inline approach is available; {required_distance:.3f} is required.",
                dropoff_route.name,
                position_error,
                alignment_error,
                straight_distance,
                required_distance,
            )
        return PayloadPickupAnalysis(
            True,
            f"Pickup possible from '{dropoff_route.name}': payload position matches and {straight_distance:.3f} of straight inline approach is available.",
            dropoff_route.name,
            position_error,
            alignment_error,
            straight_distance,
            required_distance,
        )

    def _straight_inline_approach_distance(
        self, poses: list[Pose], target_heading_deg: float
    ) -> float:
        straight_distance = 0.0
        for first, second in reversed(list(zip(poses, poses[1:]))):
            dx, dy = second.x - first.x, second.y - first.y
            distance = hypot(dx, dy)
            if distance < 1e-9:
                continue
            motion_heading = degrees(atan2(dy, dx))
            if self._axis_heading_error(motion_heading, target_heading_deg) > 2.0:
                break
            straight_distance += distance
        return straight_distance

    def _payload_dropoff_analysis(
        self, route: RoutePlan, profile: VehicleProfile, poses: list[Pose]
    ) -> PayloadPickupAnalysis | None:
        if route.dropoff_pose is None:
            return None
        marker_index = next(
            (index for index, pose in enumerate(poses) if pose.maneuver == "dropoff"),
            None,
        )
        if marker_index is None:
            return PayloadPickupAnalysis(
                False,
                "The route does not reach its configured drop-off point.",
            )
        approach_poses = poses[: marker_index + 1]
        straight_distance = self._straight_inline_approach_distance(
            approach_poses, route.dropoff_pose.heading_deg
        )
        alignment_error = self._axis_heading_error(
            approach_poses[-1].heading_deg, route.dropoff_pose.heading_deg
        )
        # A drop-off only requires the vehicle to reach the configured pose inline.
        # Unlike pickup, there is no fork/payload engagement travel that justifies
        # requiring a full vehicle length of perfectly straight approach.
        possible = alignment_error <= 2.0
        if possible:
            message = (
                f"Drop-off pose is aligned within {alignment_error:.1f} deg"
                f" ({straight_distance:.3f} straight approach measured); payload is released, "
                "then the vehicle follows the configured egress."
            )
        else:
            message = (
                f"Drop-off heading has {alignment_error:.1f} deg alignment error; "
                "require at most 2.0 deg."
            )
        return PayloadPickupAnalysis(
            possible,
            message,
            alignment_error_deg=alignment_error,
            straight_approach_distance=straight_distance,
            required_straight_distance=0.0,
        )

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
        stopped = " (stopped)" if value == 0 else ""
        self.direction_value_label.setText(f"Direction of travel: {direction}{stopped}")
        QtBootstrap.style_semantic(
            self.direction_value_label,
            "success" if self.travel_direction > 0 else "danger",
        )
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
        if self.project_file_path is None:
            self.store.upsert(profile)
            self.vehicles = self.store.vehicles
        else:
            for index, vehicle in enumerate(self.vehicles):
                if vehicle.name == profile.name:
                    self.vehicles[index] = profile
                    break
            else:
                self.vehicles.append(profile)
            self._write_project()
        self.vehicle_combo.blockSignals(True)
        self.vehicle_combo.clear()
        self.vehicle_combo.addItems([vehicle.name for vehicle in self.vehicles])
        self.vehicle_combo.setCurrentText(profile.name)
        self.vehicle_combo.blockSignals(False)
        self.current_profile = profile
        destination = self.project_file_path.name if self.project_file_path else "vehicles.json"
        self.statusBar().showMessage(f"Saved vehicle profile '{profile.name}' to {destination}")
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
            load_distance=self.load_distance_spin.value(),
            aisle_clearance=self.aisle_clearance_spin.value(),
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
        self.load_distance_spin.setValue(profile.load_distance)
        self.aisle_clearance_spin.setValue(profile.aisle_clearance)
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
        self.route_tangent_items.clear()
        self.route_tangent_lines.clear()
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
        if self.dropoff_pose is not None:
            active_route_name = self.route_name_edit.text().strip() or "Active path"
            self.position_items.append(
                self._add_dropoff_vehicle_block(
                    self.dropoff_pose,
                    active_route_name,
                    active=True,
                )
            )
            self.position_items.extend(
                self._add_payload_dropoff_footprint(
                    self.dropoff_pose,
                    active_route_name,
                    marker_size,
                    active=True,
                )
            )
            dropoff_handle = PoseHandleItem(
                "dropoff",
                self.dropoff_pose.x,
                -self.dropoff_pose.y,
                self.dropoff_pose.heading_deg,
                marker_size,
                QColor("#a21caf"),
                True,
                self._pose_handle_moved,
                self._pose_handle_released,
            )
            dropoff_handle.setToolTip(
                f"Active payload drop-off: {active_route_name}; drag to reposition"
            )
            self.scene.addItem(dropoff_handle)
            self.position_items.append(dropoff_handle)
        saved_pen = QPen(QColor("#2563eb"), 0)
        adjacent_dropoff_pen = QPen(QColor("#c026d3"), 0)
        adjacent_dropoff_brush = QBrush(QColor(192, 38, 211, 70))
        if self.show_other_paths_checkbox.isChecked():
            for index, route in enumerate(self.saved_routes):
                if index == self.active_route_index or route.level_name != self.current_level_name:
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
                if route.dropoff_pose is None:
                    continue
                dropoff = route.dropoff_pose
                self.position_items.append(
                    self._add_dropoff_vehicle_block(
                        dropoff,
                        route.name,
                        active=False,
                    )
                )
                self.position_items.extend(
                    self._add_payload_dropoff_footprint(
                        dropoff,
                        route.name,
                        marker_size,
                        active=False,
                    )
                )
                dropoff_marker = self.scene.addEllipse(
                    dropoff.x - marker_size * 0.55,
                    -dropoff.y - marker_size * 0.55,
                    marker_size * 1.1,
                    marker_size * 1.1,
                    adjacent_dropoff_pen,
                    adjacent_dropoff_brush,
                )
                dropoff_heading = radians(dropoff.heading_deg)
                dropoff_direction = self.scene.addLine(
                    dropoff.x,
                    -dropoff.y,
                    dropoff.x + marker_size * 2.0 * cos(dropoff_heading),
                    -dropoff.y - marker_size * 2.0 * sin(dropoff_heading),
                    adjacent_dropoff_pen,
                )
                label = QGraphicsTextItem(
                    f"{route.name}\nDrop-off {dropoff.heading_deg:.1f} deg"
                )
                label.setDefaultTextColor(QColor("#a21caf"))
                label.setFont(QFont(QApplication.font().family(), 9))
                label_scale = max(marker_size / 28.0, 0.01)
                label.setScale(label_scale)
                label.setPos(
                    dropoff.x + marker_size * 0.75,
                    -dropoff.y - marker_size * 1.5,
                )
                tooltip = (
                    f"Adjacent path drop-off: {route.name}; "
                    f"X {dropoff.x:.3f}, Y {dropoff.y:.3f}, heading {dropoff.heading_deg:.1f} deg"
                )
                for item in (dropoff_marker, dropoff_direction, label):
                    item.setZValue(18.0)
                    item.setToolTip(tooltip)
                self.position_items.extend(
                    [dropoff_marker, dropoff_direction, label]
                )

    def _add_dropoff_vehicle_block(
        self,
        dropoff: Pose,
        route_name: str,
        *,
        active: bool,
    ) -> QGraphicsItemGroup:
        block = self.draw_vehicle(
            self.form_profile(),
            Pose(dropoff.x, dropoff.y, dropoff.heading_deg, 0.0, "dropoff"),
            ghost=True,
            detailed=True,
            direction_override=0,
        )
        block.setOpacity(0.72 if active else 0.38)
        block.setZValue(15.5 if active else 14.5)
        block.setData(0, "active-dropoff-vehicle" if active else "saved-dropoff-vehicle")
        block.setData(1, route_name)
        block.setToolTip(
            f"{'Active' if active else 'Saved'} vehicle at payload drop-off: {route_name}; "
            f"X {dropoff.x:.3f}, Y {dropoff.y:.3f}, heading {dropoff.heading_deg:.1f} deg"
        )
        return block

    def _add_payload_dropoff_footprint(
        self,
        dropoff: Pose,
        route_name: str,
        marker_size: float,
        *,
        active: bool,
    ) -> list:
        profile = self.form_profile()
        payload_points = [
            dropoff.transformed_point(local_x, local_y)
            for local_x, local_y in payload_outline_points(profile)
        ]
        color = QColor("#0891b2" if active else "#c026d3")
        fill = QColor(color)
        fill.setAlpha(65 if active else 32)
        pen = QPen(color, 2.0)
        pen.setCosmetic(True)
        pen.setStyle(Qt.PenStyle.DashLine)
        footprint = QGraphicsPolygonItem(
            QPolygonF([QPointF(x, -y) for x, y in payload_points])
        )
        footprint.setPen(pen)
        footprint.setBrush(QBrush(fill))
        footprint.setZValue(17.0 if active else 16.0)
        center = dropoff.transformed_point(profile.payload_x, profile.payload_y)
        cross_size = max(marker_size * 0.35, 0.05)
        horizontal = self.scene.addLine(
            center[0] - cross_size,
            -center[1],
            center[0] + cross_size,
            -center[1],
            pen,
        )
        vertical = self.scene.addLine(
            center[0],
            -center[1] - cross_size,
            center[0],
            -center[1] + cross_size,
            pen,
        )
        label = QGraphicsTextItem(
            f"{route_name}\nPayload drop-off\n{dropoff.heading_deg:.1f} deg"
        )
        label.setDefaultTextColor(color)
        label.setFont(QFont(QApplication.font().family(), 9))
        label.setScale(max(marker_size / 28.0, 0.01))
        label.setPos(
            max(point[0] for point in payload_points) + marker_size * 0.35,
            -max(point[1] for point in payload_points),
        )
        tooltip = (
            f"{'Active' if active else 'Saved'} payload drop-off: {route_name}; "
            f"vehicle X {dropoff.x:.3f}, Y {dropoff.y:.3f}, "
            f"heading {dropoff.heading_deg:.1f} deg; "
            f"payload centre X {center[0]:.3f}, Y {center[1]:.3f}"
        )
        for item in (footprint, horizontal, vertical, label):
            if item.scene() is None:
                self.scene.addItem(item)
            item.setZValue(17.0 if active else 16.0)
            item.setToolTip(tooltip)
        return [footprint, horizontal, vertical, label]

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
                self._set_run_ui(False)
            self.start_pose = Pose(x, y, self.start_pose.heading_deg, 0.0)
            self.poses = [self.start_pose]
            self.speed = 0.0
            self.speed_slider.setValue(0)
        elif kind == "dropoff" and self.dropoff_pose is not None:
            self.dropoff_pose.x = x
            self.dropoff_pose.y = y
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
        pose = (
            self.start_pose
            if kind == "start"
            else self.dropoff_pose
            if kind == "dropoff"
            else self.end_pose
        )
        if pose is not None:
            self.statusBar().showMessage(
                f"{kind.title()} moved to X {pose.x:.3f}, Y {pose.y:.3f}, heading {pose.heading_deg:.1f}°"
            )

    def redraw_route_handles(self) -> None:
        self._clear_route_graphics_items(self.route_point_items)
        self._clear_route_graphics_items(self.route_tangent_items)
        self._clear_route_graphics_items(self.route_tangent_lines)
        if not self.show_route_checkbox.isChecked() or self.end_pose is None:
            return
        profile = self.form_profile()
        size = max(profile.width * 0.065, profile.length * 0.022, 0.05)
        drawing_rect = self._drawing_scene_rect()
        if drawing_rect is not None:
            size = max(size, max(drawing_rect.width(), drawing_rect.height()) / 1200.0)
        for index, (x, y) in enumerate(self.route_waypoints):
            if self.current_section_only_checkbox.isChecked():
                dropoff_index = self._effective_dropoff_waypoint_index()
                if (self._current_route_section == "pre") != (index < dropoff_index):
                    continue
            item = RoutePointHandleItem(
                index,
                x,
                -y,
                size,
                self._route_point_moved,
                self._route_point_released,
                self._select_route_point,
                index in self.route_point_turns,
                self._set_route_point_turn,
                index in self.route_reversing_actions,
                self._set_route_reversing_action,
                self.route_point_path_modes.get(index) == "straight",
            )
            self.scene.addItem(item)
            self.route_point_items.append(item)
            if index == self._selected_route_point_index:
                item.setSelected(True)
            if (
                index in self.route_point_turns
                or index in self.route_reversing_actions
                or self.route_point_path_modes.get(index) in {"straight", "minimum_radius"}
            ):
                continue
            vector = self.route_tangent_handles.get(index) or self._default_tangent_handle(index)
            if hypot(vector[0], vector[1]) < 1e-9:
                continue
            tangent_pen = QPen(QColor("#0ea5e9"), 0)
            tangent_pen.setStyle(Qt.PenStyle.DashLine)
            line = self.scene.addLine(
                x - vector[0],
                -(y - vector[1]),
                x + vector[0],
                -(y + vector[1]),
                tangent_pen,
            )
            line.setZValue(23.0)
            line.setToolTip(f"Curve tangent for route point {index + 1}")
            self.route_tangent_lines.append(line)
            handle_size = size * 0.65
            for sign in (-1, 1):
                handle = CurveTangentHandleItem(
                    index,
                    sign,
                    x + sign * vector[0],
                    -(y + sign * vector[1]),
                    handle_size,
                    self._curve_tangent_moved,
                    self._curve_tangent_released,
                )
                self.scene.addItem(handle)
                self.route_tangent_items.append(handle)

    @staticmethod
    def _clear_route_graphics_items(items: list) -> None:
        old_items = list(items)
        items.clear()
        for item in old_items:
            if not isValid(item):
                continue
            scene = item.scene()
            if scene is not None:
                scene.removeItem(item)

    def _default_tangent_handle(self, index: int) -> tuple[float, float]:
        dropoff_index = self._effective_dropoff_waypoint_index()
        nodes = [
            (self.start_pose.x, self.start_pose.y),
            *self.route_waypoints[:dropoff_index],
        ]
        if self.dropoff_pose is not None:
            nodes.append((self.dropoff_pose.x, self.dropoff_pose.y))
        nodes.extend(self.route_waypoints[dropoff_index:])
        nodes.append((self.end_pose.x, self.end_pose.y))
        node_index = index + 1 + (
            1 if self.dropoff_pose is not None and index >= dropoff_index else 0
        )
        previous, point, following = (
            nodes[node_index - 1], nodes[node_index], nodes[node_index + 1]
        )
        incoming_length = hypot(point[0] - previous[0], point[1] - previous[1])
        outgoing_length = hypot(following[0] - point[0], following[1] - point[1])
        chord = (following[0] - previous[0], following[1] - previous[1])
        chord_length = hypot(*chord)
        length = min(incoming_length, outgoing_length) * 0.75
        if chord_length < 1e-9 or length < 1e-9:
            return (0.0, 0.0)
        return (chord[0] / chord_length * length, chord[1] / chord_length * length)

    def _curve_tangent_moved(self, index: int, sign: int, scene_position: QPointF) -> None:
        if not 0 <= index < len(self.route_waypoints):
            return
        point_x, point_y = self.route_waypoints[index]
        handle_x, handle_y = float(scene_position.x()), float(-scene_position.y())
        vector = ((handle_x - point_x) * sign, (handle_y - point_y) * sign)
        if hypot(*vector) > 1e-9:
            self.route_tangent_handles[index] = vector
            self.redraw_dynamic_layers(self.form_profile())

    def _curve_tangent_released(self, index: int, sign: int, scene_position: QPointF) -> None:
        self._curve_tangent_moved(index, sign, scene_position)
        self.redraw_route_handles()
        vector = self.route_tangent_handles.get(index, (0.0, 0.0))
        self.statusBar().showMessage(
            f"Curve handle {index + 1}: X {vector[0]:.3f}, Y {vector[1]:.3f}"
        )

    def _route_point_moved(self, index: int, scene_position: QPointF) -> None:
        if not 0 <= index < len(self.route_waypoints):
            return
        self.stop_route_animation()
        self.route_waypoints[index] = (float(scene_position.x()), float(-scene_position.y()))
        self.redraw_dynamic_layers(self.form_profile())

    def _route_point_released(self, index: int, scene_position: QPointF) -> None:
        self._route_point_moved(index, scene_position)
        self.redraw_route_handles()
        self._update_navigation_bounds()
        x, y = self.route_waypoints[index]
        self.statusBar().showMessage(f"Route point {index + 1} moved to X {x:.3f}, Y {y:.3f}")

    def _set_route_point_turn(self, index: int, enabled: bool) -> None:
        if not 0 <= index < len(self.route_waypoints):
            return
        self.stop_route_animation()
        if enabled:
            self.route_point_turns.add(index)
            self.route_reversing_actions.discard(index)
        else:
            self.route_point_turns.discard(index)
        self.redraw_dynamic_layers(self.form_profile())
        self.redraw_route_handles()
        self._refresh_route_operations_table()
        state = "enabled" if enabled else "disabled"
        self.statusBar().showMessage(f"Driven-wheel point turn {state} at route point {index + 1}.")

    def _set_route_reversing_action(self, index: int, enabled: bool) -> None:
        if not 0 <= index < len(self.route_waypoints):
            return
        self.stop_route_animation()
        if enabled:
            self.route_reversing_actions.add(index)
            self.route_point_turns.discard(index)
        else:
            self.route_reversing_actions.discard(index)
        self.redraw_dynamic_layers(self.form_profile())
        self.redraw_route_handles()
        self._refresh_route_operations_table()
        state = "enabled" if enabled else "disabled"
        self.statusBar().showMessage(f"Reversing action {state} at route point {index + 1}.")

    def remove_selected_route_point(self) -> None:
        selected_index = self._selected_route_point_index
        if selected_index is None:
            selected = next((item for item in self.route_point_items if item.isSelected()), None)
            selected_index = selected.index if selected is not None else None
        if selected_index is None:
            QMessageBox.information(self, "Select a route point", "Select an orange route point to remove.")
            return
        self.stop_route_animation()
        del self.route_waypoints[selected_index]
        if (
            self.route_dropoff_waypoint_index is not None
            and selected_index < self.route_dropoff_waypoint_index
        ):
            self.route_dropoff_waypoint_index -= 1
        self.route_point_turns = {
            index - 1 if index > selected_index else index
            for index in self.route_point_turns
            if index != selected_index
        }
        self.route_reversing_actions = {
            index - 1 if index > selected_index else index
            for index in self.route_reversing_actions
            if index != selected_index
        }
        self.route_tangent_handles = {
            (index - 1 if index > selected_index else index): vector
            for index, vector in self.route_tangent_handles.items()
            if index != selected_index
        }
        self.route_point_path_modes = {
            (index - 1 if index > selected_index else index): mode
            for index, mode in self.route_point_path_modes.items()
            if index != selected_index
        }
        self._selected_route_point_index = (
            min(selected_index, len(self.route_waypoints) - 1)
            if self.route_waypoints
            else None
        )
        self.redraw_dynamic_layers(self.form_profile())
        self.redraw_route_handles()
        self._refresh_route_operations_table()
        self.statusBar().showMessage("Removed the selected route point.")

    def clear_route_points(self) -> None:
        if not self.route_waypoints:
            return
        self.stop_route_animation()
        self.route_waypoints.clear()
        self.route_point_turns.clear()
        self.route_reversing_actions.clear()
        self.route_tangent_handles.clear()
        self.route_point_path_modes.clear()
        self._selected_route_point_index = None
        if self.dropoff_pose is not None:
            self.route_dropoff_waypoint_index = 0
        self.redraw_dynamic_layers(self.form_profile())
        self.redraw_route_handles()
        self._refresh_route_operations_table()
        self.statusBar().showMessage("Cleared all route control points.")

    def _route_store_path(self) -> Path | None:
        return self.project_dxf_path

    def _persist_routes(self) -> None:
        if self.project_file_path is not None:
            self._write_project()
            return
        self.route_store.save_configuration(
            self._route_store_path(),
            self.levels,
            self.start_positions,
            self.saved_routes,
            self.level_drawing_paths,
        )

    def _next_route_name(self) -> str:
        used = {route.name for route in self.saved_routes}
        number = 1
        while f"Path {number}" in used:
            number += 1
        return f"Path {number}"

    def _copied_route_name(self, source_name: str) -> str:
        base = f"{source_name} Copy"
        used = {route.name.casefold() for route in self.saved_routes}
        if base.casefold() not in used:
            return base
        number = 2
        while f"{base} {number}".casefold() in used:
            number += 1
        return f"{base} {number}"

    def _route_plan_from_editor(self, name: str) -> RoutePlan:
        assert self.end_pose is not None
        return RoutePlan(
            name,
            Pose(self.end_pose.x, self.end_pose.y, self.end_pose.heading_deg, 0.0),
            list(self.route_waypoints),
            sorted(self.route_point_turns),
            sorted(self.route_reversing_actions),
            self.current_level_name,
            self.current_start_name,
            Pose(self.start_pose.x, self.start_pose.y, self.start_pose.heading_deg),
            dict(self.route_tangent_handles),
            payload_action=self.route_end_operation,
            operations=self._current_ordered_operations(),
            dropoff_pose=(
                Pose(
                    self.dropoff_pose.x,
                    self.dropoff_pose.y,
                    self.dropoff_pose.heading_deg,
                )
                if self.dropoff_pose is not None
                else None
            ),
            point_path_modes=dict(self.route_point_path_modes),
            dropoff_waypoint_index=self.route_dropoff_waypoint_index,
        )

    def copy_current_route(self) -> None:
        if self.end_pose is None:
            QMessageBox.information(
                self,
                "Select a path",
                "Select or create a path with a final position before copying it.",
            )
            return
        source_name = self.route_name_edit.text().strip() or self._next_route_name()
        copy_name = self._copied_route_name(source_name)
        copied_route = RoutePlan.from_dict(
            self._route_plan_from_editor(copy_name).to_dict()
        )
        copied_route.name = copy_name
        self.saved_routes.append(copied_route)
        self.active_route_index = len(self.saved_routes) - 1
        self._persist_routes()
        self._refresh_route_combo(self.active_route_index)
        self.route_name_edit.setText(copy_name)
        self._redraw_route_layers()
        self.statusBar().showMessage(
            f"Copied '{source_name}' as '{copy_name}'. Move its positions and route points for the adjacent space, then save the path."
        )

    def _refresh_route_combo(self, selected_index: int | None = None) -> None:
        if not hasattr(self, "route_combo"):
            return
        self._updating_route_combo = True
        self.route_combo.clear()
        self.route_combo.addItem("New unsaved path", None)
        for index, route in enumerate(self.saved_routes):
            self.route_combo.addItem(f"{route.level_name} / {route.name}", index)
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
        route = self._route_plan_from_editor(name)
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
        self.dropoff_pose = None
        self.route_dropoff_waypoint_index = None
        self.route_waypoints.clear()
        self._selected_route_point_index = None
        self.route_point_turns.clear()
        self.route_reversing_actions.clear()
        self.route_tangent_handles.clear()
        self.route_point_path_modes.clear()
        self.route_start_operation = "travel"
        self.route_end_operation = "stop"
        self.end_heading_spin.blockSignals(True)
        self.end_heading_spin.setValue(0.0)
        self.end_heading_spin.blockSignals(False)
        self.dropoff_heading_spin.blockSignals(True)
        self.dropoff_heading_spin.setValue(0.0)
        self.dropoff_heading_spin.blockSignals(False)
        self.route_name_edit.setText(self._next_route_name())
        self._refresh_route_combo()
        self._update_position_label()
        self._refresh_route_operations_table()
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
        route_start = self._start_pose_for_route(route)
        self.start_pose = Pose(route_start.x, route_start.y, route_start.heading_deg)
        self.poses = [self.start_pose]
        self.current_start_name = route.start_position_name
        self.current_level_name = route.level_name
        self.end_pose = Pose(route.end_pose.x, route.end_pose.y, route.end_pose.heading_deg, 0.0)
        self.dropoff_pose = (
            Pose(route.dropoff_pose.x, route.dropoff_pose.y, route.dropoff_pose.heading_deg)
            if route.dropoff_pose is not None
            else None
        )
        self.route_dropoff_waypoint_index = (
            min(route.dropoff_waypoint_index, len(route.waypoints))
            if route.dropoff_pose is not None and route.dropoff_waypoint_index is not None
            else len(route.waypoints) if route.dropoff_pose is not None else None
        )
        self.route_waypoints = list(route.waypoints)
        self._selected_route_point_index = None
        self.route_point_turns = {
            index for index in route.point_turn_indices if 0 <= index < len(route.waypoints)
        }
        self.route_reversing_actions = {
            index
            for index in route.reversing_action_indices
            if 0 <= index < len(route.waypoints)
        }
        self.route_tangent_handles = {
            index: vector
            for index, vector in route.tangent_handles.items()
            if 0 <= index < len(route.waypoints)
        }
        self.route_point_path_modes = {
            index: mode
            for index, mode in route.point_path_modes.items()
            if 0 <= index < len(route.waypoints)
        }
        ordered_operations = route.ordered_operations()
        self.route_start_operation = next(
            (item.operation for item in ordered_operations if item.location == "start"),
            "travel",
        )
        self.route_end_operation = next(
            (item.operation for item in ordered_operations if item.location == "end"),
            route.payload_action if route.payload_action in {"pickup", "dropoff"} else "stop",
        )
        self.route_name_edit.setText(route.name)
        self.start_position_combo.blockSignals(True)
        self.start_position_combo.setCurrentText(self.current_start_name)
        self.start_position_combo.blockSignals(False)
        self.level_combo.blockSignals(True)
        self.level_combo.setCurrentText(self.current_level_name)
        self.level_combo.blockSignals(False)
        self.start_heading_spin.blockSignals(True)
        self.start_heading_spin.setValue(self.start_pose.heading_deg)
        self.start_heading_spin.blockSignals(False)
        self.end_heading_spin.blockSignals(True)
        self.end_heading_spin.setValue(route.end_pose.heading_deg)
        self.end_heading_spin.blockSignals(False)
        self.dropoff_heading_spin.blockSignals(True)
        self.dropoff_heading_spin.setValue(
            route.dropoff_pose.heading_deg if route.dropoff_pose is not None else 0.0
        )
        self.dropoff_heading_spin.blockSignals(False)
        self._update_position_label()
        self._refresh_route_operations_table()
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
        self.dropoff_pose = None
        self.route_dropoff_waypoint_index = None
        self.route_waypoints.clear()
        self._selected_route_point_index = None
        self.route_point_turns.clear()
        self.route_reversing_actions.clear()
        self.route_tangent_handles.clear()
        self.route_point_path_modes.clear()
        self.route_start_operation = "travel"
        self.route_end_operation = "stop"
        self.route_name_edit.setText(self._next_route_name())
        self._refresh_route_combo()
        self._update_position_label()
        self._refresh_route_operations_table()
        self._redraw_route_layers()
        self.statusBar().showMessage(f"Removed saved path '{name}'.")

    def _redraw_route_layers(self) -> None:
        profile = self.form_profile()
        self.redraw_dynamic_layers(profile)
        self.redraw_position_markers()
        self.redraw_route_handles()
        self._refresh_route_operations_table()
        self._update_navigation_bounds()

    def _start_pose_for_route(self, route: RoutePlan) -> Pose:
        if route.start_pose is not None:
            return route.start_pose
        configured = next(
            (start.pose for start in self.start_positions if start.name == route.start_position_name),
            None,
        )
        return configured or self.start_pose

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
            if index != self.active_route_index and route.level_name == self.current_level_name
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
        if self.show_other_paths_checkbox.isChecked():
            for index, route in enumerate(self.saved_routes):
                if index == self.active_route_index or route.level_name != self.current_level_name:
                    continue
                saved_vehicle = self.draw_vehicle(
                    profile, route.end_pose, ghost=True, detailed=True
                )
                saved_vehicle.setData(0, "saved-route-vehicle")
                saved_vehicle.setData(1, route.name)
                self.vehicle_items.append(saved_vehicle)
        self.vehicle_items.append(self.draw_vehicle(profile, self.poses[-1], ghost=False))

    def redraw_indicative_path(self, profile: VehicleProfile) -> None:
        if not self.show_route_checkbox.isChecked():
            self.route_feasibility_label.setText("Route check: hidden")
            QtBootstrap.style_semantic(self.route_feasibility_label, "muted")
            return
        inactive_routes = [
            (
                route.name,
                self._planned_route_poses_for(
                    route.end_pose,
                    route.waypoints,
                    set(route.point_turn_indices),
                    set(route.reversing_action_indices),
                    self._start_pose_for_route(route),
                    route.tangent_handles,
                    route.dropoff_pose,
                    route.point_path_modes,
                    route.dropoff_waypoint_index,
                    self._route_starts_reversing(route),
                ),
            )
            for index, route in enumerate(self.saved_routes)
            if self.show_other_paths_checkbox.isChecked()
            and index != self.active_route_index
            and route.level_name == self.current_level_name
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
            QtBootstrap.style_semantic(self.route_feasibility_label, "primary")
            return
        current = self.poses[-1]
        route_poses: list[Pose] = []
        if self.end_pose is not None:
            route_poses = self.planned_route_poses(profile)
        else:
            total_distance = max(profile.length * 4.0, profile.effective_min_turning_radius * 2.0)
            step_distance = total_distance / 60.0
            projected = Pose(current.x, current.y, current.heading_deg, self.steering)
            route_poses.append(projected)
            for _ in range(60):
                projected = step_pose(projected, profile, self.steering, step_distance)
                route_poses.append(projected)

        if len(route_poses) < 2:
            self.route_feasibility_label.setText("Route check: place a finish position")
            QtBootstrap.style_semantic(self.route_feasibility_label, "muted")
            return
        visible_route_poses = self._route_poses_for_current_section(route_poses)
        path = QPainterPath(QPointF(visible_route_poses[0].x, -visible_route_poses[0].y))
        for pose in visible_route_poses[1:]:
            path.lineTo(pose.x, -pose.y)
        pen = QPen(QColor("#d97706"), 0)
        pen.setStyle(Qt.PenStyle.DashLine)
        self.indicative_path_item = self.scene.addPath(path, pen)
        self.indicative_path_item.setZValue(2.0)
        tooltip = "Planned route from start to finish" if self.end_pose is not None else "Projected steering path"
        self.indicative_path_item.setToolTip(tooltip)

        if self.end_pose is not None:
            self.draw_route_failures(visible_route_poses, profile)
            pickup_route = RoutePlan(
                self.route_name_edit.text().strip() or "Unsaved Path",
                Pose(self.end_pose.x, self.end_pose.y, self.end_pose.heading_deg),
                list(self.route_waypoints),
                sorted(self.route_point_turns),
                sorted(self.route_reversing_actions),
                self.current_level_name,
                self.current_start_name,
                Pose(self.start_pose.x, self.start_pose.y, self.start_pose.heading_deg),
                dict(self.route_tangent_handles),
                payload_action=self.route_end_operation,
                operations=self._current_ordered_operations(),
                dropoff_pose=self.dropoff_pose,
                point_path_modes=dict(self.route_point_path_modes),
            )
            pickup = self._payload_pickup_analysis(pickup_route, profile, route_poses)
            dropoff = self._payload_dropoff_analysis(pickup_route, profile, route_poses)
            if dropoff is not None:
                self.route_feasibility_label.setText(
                    f"{self.route_feasibility_label.text()} Drop-off check: {dropoff.message}"
                )
                QtBootstrap.style_semantic(
                    self.route_feasibility_label,
                    "success" if dropoff.possible else "danger",
                )
            if pickup is not None:
                self.route_feasibility_label.setText(
                    f"{self.route_feasibility_label.text()} Pickup check: {pickup.message}"
                )
                QtBootstrap.style_semantic(
                    self.route_feasibility_label, "success" if pickup.possible else "danger"
                )
            sweep_color = QColor("#f59e0b")
            sweep_color.setAlpha(150)
            sweep_pen = QPen(sweep_color, 0)
            sweep_pen.setStyle(Qt.PenStyle.DotLine)
            for side in (-1.0, 1.0):
                sweep = QPainterPath()
                for index, pose in enumerate(visible_route_poses):
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
            self.draw_planned_block_outline(visible_route_poses, profile)
            if profile.payload_enabled:
                self.draw_payload_traces(visible_route_poses, profile, planned=True)
        else:
            self.route_feasibility_label.setText("Route check: projected steering path")
            QtBootstrap.style_semantic(self.route_feasibility_label, "muted")

    def draw_route_failures(self, route_poses: list[Pose], profile: VehicleProfile) -> None:
        invalid, required_curvatures, unsupported_point_turn = self._route_section_analysis(
            route_poses, profile
        )
        limit = profile.max_turn_curvature
        worst_curvature = max(
            (value for value in required_curvatures if value != float("inf")),
            default=0.0,
        )
        available_radius = 1.0 / max(limit, 1e-12)
        if not any(invalid):
            required_radius = 1.0 / max(worst_curvature, 1e-12)
            radius_text = "straight" if worst_curvature < 1e-9 else f"minimum required radius {required_radius:.3f}"
            pivot_count = sum(1 for pose in route_poses if pose.maneuver == "point_turn")
            has_reverse = any(pose.maneuver == "reverse" for pose in route_poses)
            maneuver_notes = []
            if pivot_count:
                maneuver_notes.append("driven-wheel point turn included")
            if has_reverse:
                maneuver_notes.append("reversing action included")
            maneuver_text = f"; {', '.join(maneuver_notes)}" if maneuver_notes else ""
            self.route_feasibility_label.setText(
                f"Route feasible — {radius_text}{maneuver_text}"
            )
            QtBootstrap.style_semantic(self.route_feasibility_label, "success")
            return

        reverse_suggestion = ""
        if self.end_pose is not None and not self.route_reversing_actions:
            realignment = self._reverse_alignment_candidate(
                self.end_pose, self.route_waypoints, self.route_point_turns, profile
            )
            if realignment is not None:
                reverse_suggestion = (
                    " One reverse movement appears feasible; add or adjust the final route point "
                    f"near X {realignment.x:.3f}, Y {realignment.y:.3f} to realign if required."
                )

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
        if unsupported_point_turn:
            reason = "point turn requires a driven steerable wheel (or differential drive)"
        else:
            reason = f"available minimum radius {available_radius:.3f}"
        prefix = "Forward-only route impossible" if reverse_suggestion else "Route impossible"
        self.route_feasibility_label.setText(
            f"{prefix} in {invalid_count} section(s) — {reason}.{reverse_suggestion}"
        )
        QtBootstrap.style_semantic(
            self.route_feasibility_label,
            "warning" if reverse_suggestion else "danger",
        )

    @staticmethod
    def _route_section_analysis(
        route_poses: list[Pose], profile: VehicleProfile
    ) -> tuple[list[bool], list[float], bool]:
        invalid: list[bool] = []
        required_curvatures: list[float] = []
        unsupported_point_turn = False
        for first, second in zip(route_poses, route_poses[1:]):
            distance = hypot(second.x - first.x, second.y - first.y)
            heading_change = ((second.heading_deg - first.heading_deg + 180.0) % 360.0) - 180.0
            if distance < 1e-9 and second.maneuver == "point_turn":
                supported = profile.supports_point_turn
                invalid.append(not supported)
                required_curvatures.append(0.0 if supported else float("inf"))
                unsupported_point_turn |= not supported
                continue
            curvature = abs(radians(heading_change)) / max(distance, 1e-9)
            required_curvatures.append(curvature)
            invalid.append(curvature > profile.max_turn_curvature * 1.02)
        return invalid, required_curvatures, unsupported_point_turn

    def _reverse_alignment_candidate(
        self,
        end: Pose,
        waypoints: list[tuple[float, float]],
        point_turn_indices: set[int],
        profile: VehicleProfile,
    ) -> Pose | None:
        """Find a physically continuous forward-to-reverse cusp before the finish."""
        candidates: list[
            tuple[tuple[float, float], list[tuple[float, float]], set[int]]
        ] = []
        if waypoints:
            candidates.append((waypoints[-1], waypoints[:-1], {
                index for index in point_turn_indices if index < len(waypoints) - 1
            }))

        radius = max(profile.effective_min_turning_radius, profile.length * 0.5)
        heading = radians(end.heading_deg)
        forward = (cos(heading), sin(heading))
        left = (-forward[1], forward[0])
        for distance_factor in (1.5, 2.0, 3.0, 4.0):
            distance = radius * distance_factor
            for lateral_factor in (0.0, 0.5, -0.5, 1.0, -1.0):
                lateral = radius * lateral_factor
                point = (
                    end.x + forward[0] * distance + left[0] * lateral,
                    end.y + forward[1] * distance + left[1] * lateral,
                )
                candidates.append((point, list(waypoints), set(point_turn_indices)))

        seen: set[tuple[int, int]] = set()
        for (gear_x, gear_y), prior_waypoints, prior_turns in candidates:
            key = (round(gear_x * 1_000_000), round(gear_y * 1_000_000))
            if key in seen:
                continue
            seen.add(key)
            if hypot(gear_x - end.x, gear_y - end.y) < 1e-9:
                continue
            gear_heading = degrees(atan2(gear_y - end.y, gear_x - end.x))
            gear_pose = Pose(gear_x, gear_y, gear_heading)
            forward_route = self._planned_route_poses_for(
                gear_pose, prior_waypoints, prior_turns
            )
            reverse_route = self._hermite_motion_segment(
                gear_pose,
                end,
                gear_heading + 180.0,
                end.heading_deg + 180.0,
            )
            forward_invalid, _, _ = self._route_section_analysis(forward_route, profile)
            reverse_invalid, _, _ = self._route_section_analysis(reverse_route, profile)
            if forward_route and reverse_route and not any(forward_invalid) and not any(reverse_invalid):
                return gear_pose
        return None

    @staticmethod
    def _hermite_motion_segment(
        start: Pose,
        end: Pose,
        start_motion_heading_deg: float,
        end_motion_heading_deg: float,
    ) -> list[Pose]:
        distance = hypot(end.x - start.x, end.y - start.y)
        if distance < 1e-9:
            return []
        scale = distance * 1.2
        start_heading = radians(start_motion_heading_deg)
        end_heading = radians(end_motion_heading_deg)
        m0 = (cos(start_heading) * scale, sin(start_heading) * scale)
        m1 = (cos(end_heading) * scale, sin(end_heading) * scale)
        route: list[Pose] = []
        for sample in range(41):
            t = sample / 40.0
            t2 = t * t
            t3 = t2 * t
            h00 = 2.0 * t3 - 3.0 * t2 + 1.0
            h10 = t3 - 2.0 * t2 + t
            h01 = -2.0 * t3 + 3.0 * t2
            h11 = t3 - t2
            x = h00 * start.x + h10 * m0[0] + h01 * end.x + h11 * m1[0]
            y = h00 * start.y + h10 * m0[1] + h01 * end.y + h11 * m1[1]
            dh00 = 6.0 * t2 - 6.0 * t
            dh10 = 3.0 * t2 - 4.0 * t + 1.0
            dh01 = -6.0 * t2 + 6.0 * t
            dh11 = 3.0 * t2 - 2.0 * t
            dx = dh00 * start.x + dh10 * m0[0] + dh01 * end.x + dh11 * m1[0]
            dy = dh00 * start.y + dh10 * m0[1] + dh01 * end.y + dh11 * m1[1]
            fallback = route[-1].heading_deg if route else start_motion_heading_deg
            motion_heading = degrees(atan2(dy, dx)) if hypot(dx, dy) > 1e-9 else fallback
            route.append(Pose(x, y, motion_heading, 0.0, "reverse"))
        return route

    def block_outline_points(self, profile: VehicleProfile) -> list[tuple[float, float]]:
        if profile.dxf_block_name:
            _drawing, geometry = self._shared_block_geometry(profile.dxf_block_name)
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
        dropoff_index = next(
            (index for index, pose in enumerate(poses) if pose.maneuver == "dropoff"),
            None,
        )
        if dropoff_index is not None:
            poses = poses[: dropoff_index + 1]
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
        center_item.setData(0, "planned-payload-trace" if planned else "driven-payload-trace")
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
            item.setData(0, "planned-payload-trace" if planned else "driven-payload-trace")
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
            item.setData(0, "planned-payload-trace" if planned else "driven-payload-trace")
            self.payload_trace_items.append(item)

    def planned_route_poses(self, _profile: VehicleProfile) -> list[Pose]:
        if self.end_pose is None:
            return []
        return self._planned_route_poses_for(
            self.end_pose,
            self.route_waypoints,
            self.route_point_turns,
            self.route_reversing_actions,
            None,
            self.route_tangent_handles,
            self.dropoff_pose,
            self.route_point_path_modes,
            self.route_dropoff_waypoint_index,
            self.route_start_operation == "reverse",
        )

    def all_planned_route_poses(self, _profile: VehicleProfile) -> list[list[Pose]]:
        return [poses for _name, poses in self.all_planned_route_exports()]

    def all_planned_route_exports(self) -> list[tuple[str, list[Pose]]]:
        routes: list[tuple[str, list[Pose]]] = []
        if self.end_pose is not None:
            active = self._planned_route_poses_for(
                self.end_pose,
                self.route_waypoints,
                self.route_point_turns,
                self.route_reversing_actions,
                None,
                self.route_tangent_handles,
                self.dropoff_pose,
                self.route_point_path_modes,
                self.route_dropoff_waypoint_index,
                self.route_start_operation == "reverse",
            )
            if active:
                current_name = self.route_name_edit.text().strip()
                if not current_name and self.active_route_index is not None:
                    current_name = self.saved_routes[self.active_route_index].name
                routes.append((current_name or "Unsaved Path", active))
        for index, route in enumerate(self.saved_routes):
            if index == self.active_route_index:
                continue
            poses = self._planned_route_poses_for(
                route.end_pose,
                route.waypoints,
                set(route.point_turn_indices),
                set(route.reversing_action_indices),
                self._start_pose_for_route(route),
                route.tangent_handles,
                route.dropoff_pose,
                route.point_path_modes,
                route.dropoff_waypoint_index,
                self._route_starts_reversing(route),
            )
            if poses:
                routes.append((route.name, poses))
        return routes

    def all_planned_route_names(self) -> list[str]:
        return [name for name, _poses in self.all_planned_route_exports()]

    def _planned_route_poses_for(
        self,
        end: Pose,
        waypoints: list[tuple[float, float]],
        point_turn_indices: set[int] | None = None,
        reversing_action_indices: set[int] | None = None,
        start_pose: Pose | None = None,
        tangent_handles: dict[int, tuple[float, float]] | None = None,
        dropoff_pose: Pose | None = None,
        point_path_modes: dict[int, str] | None = None,
        dropoff_insert_index: int | None = None,
        start_reversing: bool = False,
    ) -> list[Pose]:
        start = start_pose or self.start_pose
        point_turn_indices = point_turn_indices or set()
        reversing_action_indices = reversing_action_indices or set()
        tangent_handles = tangent_handles or {}
        point_path_modes = point_path_modes or {}
        route_waypoints = list(waypoints)
        reversing_action_indices = set(reversing_action_indices)
        dropoff_waypoint_index: int | None = None
        if dropoff_pose is not None:
            dropoff_waypoint_index = min(
                max(
                    len(route_waypoints) if dropoff_insert_index is None else dropoff_insert_index,
                    0,
                ),
                len(route_waypoints),
            )
            remap = lambda index: index + 1 if index >= dropoff_waypoint_index else index
            point_turn_indices = {remap(index) for index in point_turn_indices}
            reversing_action_indices = {
                remap(index) for index in reversing_action_indices
            }
            tangent_handles = {
                remap(index): vector for index, vector in tangent_handles.items()
            }
            point_path_modes = {
                remap(index): mode for index, mode in point_path_modes.items()
            }
            route_waypoints.insert(
                dropoff_waypoint_index, (dropoff_pose.x, dropoff_pose.y)
            )
            reversing_action_indices.add(dropoff_waypoint_index)
        nodes = [(start.x, start.y), *route_waypoints, (end.x, end.y)]
        if all(hypot(b[0] - a[0], b[1] - a[1]) < 1e-9 for a, b in zip(nodes, nodes[1:])):
            return []
        segment_directions: list[int] = []
        direction = -1 if start_reversing else 1
        for segment in range(len(nodes) - 1):
            segment_directions.append(direction)
            if segment in reversing_action_indices:
                direction *= -1
        start_heading = radians(start.heading_deg + (180.0 if start_reversing else 0.0))
        final_direction = segment_directions[-1]
        end_heading = radians(end.heading_deg + (180.0 if final_direction < 0 else 0.0))
        first_distance = hypot(nodes[1][0] - nodes[0][0], nodes[1][1] - nodes[0][1])
        last_distance = hypot(nodes[-1][0] - nodes[-2][0], nodes[-1][1] - nodes[-2][1])
        tangents: list[tuple[float, float]] = [
            (cos(start_heading) * first_distance * 1.2, sin(start_heading) * first_distance * 1.2)
        ]
        for index in range(1, len(nodes) - 1):
            incoming = (
                nodes[index][0] - nodes[index - 1][0],
                nodes[index][1] - nodes[index - 1][1],
            )
            outgoing = (
                nodes[index + 1][0] - nodes[index][0],
                nodes[index + 1][1] - nodes[index][1],
            )
            incoming_length = hypot(*incoming)
            outgoing_length = hypot(*outgoing)
            chord = (
                nodes[index + 1][0] - nodes[index - 1][0],
                nodes[index + 1][1] - nodes[index - 1][1],
            )
            chord_length = hypot(*chord)
            tangent_length = min(incoming_length, outgoing_length) * 0.75
            if chord_length > 1e-9 and tangent_length > 1e-9:
                tangents.append(
                    (
                        chord[0] / chord_length * tangent_length,
                        chord[1] / chord_length * tangent_length,
                    )
                )
            elif incoming_length > 1e-9:
                tangents.append(
                    (
                        incoming[0] / incoming_length * tangent_length,
                        incoming[1] / incoming_length * tangent_length,
                    )
                )
            else:
                tangents.append((0.0, 0.0))
        tangents.append(
            (cos(end_heading) * last_distance * 1.2, sin(end_heading) * last_distance * 1.2)
        )
        incoming_tangents = list(tangents)
        outgoing_tangents = list(tangents)
        for waypoint_index, vector in tangent_handles.items():
            node_index = waypoint_index + 1
            if 0 < node_index < len(nodes) - 1 and hypot(vector[0], vector[1]) > 1e-9:
                incoming_tangents[node_index] = vector
                outgoing_tangents[node_index] = vector
        # A linear segment must meet adjoining Hermite curves tangentially.
        # Without this, the curve retains its bisecting auto-tangent and appears
        # to overshoot or kink where a straight section follows it.
        for segment in range(len(nodes) - 1):
            if point_path_modes.get(segment) != "straight":
                continue
            vector = (
                nodes[segment + 1][0] - nodes[segment][0],
                nodes[segment + 1][1] - nodes[segment][1],
            )
            if hypot(*vector) <= 1e-9:
                continue
            outgoing_tangents[segment] = vector
            incoming_tangents[segment + 1] = vector
            if segment > 0:
                incoming_tangents[segment] = vector
            if segment + 1 < len(nodes) - 1:
                outgoing_tangents[segment + 1] = vector
        for waypoint_index in reversing_action_indices:
            node_index = waypoint_index + 1
            if not 0 < node_index < len(nodes) - 1:
                continue
            previous = nodes[node_index - 1]
            point = nodes[node_index]
            following = nodes[node_index + 1]
            incoming_x = point[0] - previous[0]
            incoming_y = point[1] - previous[1]
            incoming_length = hypot(incoming_x, incoming_y)
            outgoing_length = hypot(following[0] - point[0], following[1] - point[1])
            if incoming_length > 1e-9:
                if waypoint_index == dropoff_waypoint_index and dropoff_pose is not None:
                    heading = radians(dropoff_pose.heading_deg)
                    ux, uy = cos(heading), sin(heading)
                else:
                    ux, uy = incoming_x / incoming_length, incoming_y / incoming_length
                incoming_tangents[node_index] = (incoming_x, incoming_y)
                if waypoint_index == dropoff_waypoint_index and dropoff_pose is not None:
                    incoming_tangents[node_index] = (ux * incoming_length, uy * incoming_length)
                outgoing_tangents[node_index] = (-ux * outgoing_length, -uy * outgoing_length)
        for waypoint_index in point_turn_indices:
            node_index = waypoint_index + 1
            if not 0 < node_index < len(nodes) - 1:
                continue
            previous = nodes[node_index - 1]
            point = nodes[node_index]
            following = nodes[node_index + 1]
            incoming_tangents[node_index] = (
                point[0] - previous[0], point[1] - previous[1]
            )
            outgoing_tangents[node_index] = (
                following[0] - point[0], following[1] - point[1]
            )
        route: list[Pose] = []
        for segment in range(len(nodes) - 1):
            p0 = nodes[segment]
            p1 = nodes[segment + 1]
            m0 = outgoing_tangents[segment]
            m1 = incoming_tangents[segment + 1]
            segment_direction = segment_directions[segment]
            straight_segment = (
                point_path_modes.get(segment) == "straight"
            )
            minimum_radius_segment = (
                point_path_modes.get(segment) == "minimum_radius"
            )
            if minimum_radius_segment:
                arc = self._minimum_radius_arc_poses(
                    p0,
                    p1,
                    m0,
                    m1,
                    self.form_profile().effective_min_turning_radius,
                    segment_direction,
                )
                if segment > 0:
                    arc = arc[1:]
                route.extend(arc)
                continue
            for sample in range(41):
                if segment > 0 and sample == 0:
                    continue
                t = sample / 40.0
                if straight_segment:
                    x = p0[0] + (p1[0] - p0[0]) * t
                    y = p0[1] + (p1[1] - p0[1]) * t
                    dx = p1[0] - p0[0]
                    dy = p1[1] - p0[1]
                    motion_heading = degrees(atan2(dy, dx))
                    vehicle_heading = motion_heading + (
                        180.0 if segment_direction < 0 else 0.0
                    )
                    maneuver = "reverse" if segment_direction < 0 else ""
                    route.append(Pose(x, y, vehicle_heading, 0.0, maneuver))
                    continue
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
                fallback = (
                    route[-1].heading_deg - (180.0 if segment_direction < 0 else 0.0)
                    if route
                    else start.heading_deg
                )
                motion_heading = degrees(atan2(dy, dx)) if hypot(dx, dy) > 1e-9 else fallback
                vehicle_heading = motion_heading + (180.0 if segment_direction < 0 else 0.0)
                maneuver = "reverse" if segment_direction < 0 else ""
                route.append(Pose(x, y, vehicle_heading, 0.0, maneuver))
            waypoint_index = segment
            if (
                dropoff_waypoint_index is not None
                and waypoint_index == dropoff_waypoint_index
                and route
            ):
                route[-1].maneuver = "dropoff"
            if waypoint_index in point_turn_indices and segment < len(nodes) - 2:
                incoming_motion_heading = degrees(
                    atan2(incoming_tangents[segment + 1][1], incoming_tangents[segment + 1][0])
                )
                outgoing_motion_heading = degrees(
                    atan2(outgoing_tangents[segment + 1][1], outgoing_tangents[segment + 1][0])
                )
                incoming_heading = incoming_motion_heading + (
                    180.0 if segment_directions[segment] < 0 else 0.0
                )
                outgoing_heading = outgoing_motion_heading + (
                    180.0 if segment_directions[segment + 1] < 0 else 0.0
                )
                heading_delta = ((outgoing_heading - incoming_heading + 180.0) % 360.0) - 180.0
                pivot_steps = max(2, ceil(abs(heading_delta) / 7.5))
                steering = (1.0 if heading_delta >= 0.0 else -1.0) * self.form_profile().max_steering_angle_deg
                for pivot_step in range(1, pivot_steps + 1):
                    route.append(
                        Pose(
                            p1[0],
                            p1[1],
                            incoming_heading + heading_delta * pivot_step / pivot_steps,
                            steering,
                            "point_turn",
                        )
                    )
        initial_maneuver = "reverse" if start_reversing else ""
        route[0] = Pose(start.x, start.y, start.heading_deg, 0.0, initial_maneuver)
        final_maneuver = "reverse" if final_direction < 0 else ""
        route[-1] = Pose(end.x, end.y, end.heading_deg, 0.0, final_maneuver)
        return route

    @staticmethod
    def _minimum_radius_arc_poses(
        start: tuple[float, float],
        end: tuple[float, float],
        start_tangent: tuple[float, float],
        end_tangent: tuple[float, float],
        minimum_radius: float,
        travel_direction: int,
    ) -> list[Pose]:
        chord_x, chord_y = end[0] - start[0], end[1] - start[1]
        chord = hypot(chord_x, chord_y)
        if chord < 1e-9:
            return [Pose(start[0], start[1], 0.0)]
        radius = max(minimum_radius, chord / 2.0 + 1e-9)
        mid_x, mid_y = (start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0
        normal_x, normal_y = -chord_y / chord, chord_x / chord
        center_offset = sqrt(max(radius * radius - (chord / 2.0) ** 2, 0.0))

        def unit(vector: tuple[float, float]) -> tuple[float, float]:
            length = hypot(*vector)
            return (vector[0] / length, vector[1] / length) if length > 1e-9 else (0.0, 0.0)

        expected_start, expected_end = unit(start_tangent), unit(end_tangent)
        candidates = []
        for center_sign in (-1.0, 1.0):
            center = (
                mid_x + normal_x * center_offset * center_sign,
                mid_y + normal_y * center_offset * center_sign,
            )
            start_angle = atan2(start[1] - center[1], start[0] - center[0])
            end_angle = atan2(end[1] - center[1], end[0] - center[0])
            for sweep_sign in (-1.0, 1.0):
                sweep = (end_angle - start_angle) % (2.0 * 3.141592653589793)
                if sweep_sign < 0.0:
                    sweep -= 2.0 * 3.141592653589793
                if abs(sweep) < 1e-9:
                    continue
                start_motion = unit((
                    -sin(start_angle) * sweep_sign,
                    cos(start_angle) * sweep_sign,
                ))
                end_motion = unit((
                    -sin(end_angle) * sweep_sign,
                    cos(end_angle) * sweep_sign,
                ))
                alignment = (
                    start_motion[0] * expected_start[0]
                    + start_motion[1] * expected_start[1]
                    + end_motion[0] * expected_end[0]
                    + end_motion[1] * expected_end[1]
                )
                candidates.append((alignment - abs(sweep) * 0.01, center, start_angle, sweep))
        _score, center, start_angle, sweep = max(candidates, key=lambda item: item[0])
        samples = max(12, ceil(abs(sweep) * radius / max(radius / 20.0, 1e-6)))
        maneuver = "reverse" if travel_direction < 0 else ""
        poses = []
        for sample in range(samples + 1):
            angle = start_angle + sweep * sample / samples
            motion_heading = degrees(angle + (3.141592653589793 / 2.0 if sweep > 0 else -3.141592653589793 / 2.0))
            vehicle_heading = motion_heading + (180.0 if travel_direction < 0 else 0.0)
            poses.append(Pose(
                center[0] + radius * cos(angle),
                center[1] + radius * sin(angle),
                vehicle_heading,
                0.0,
                maneuver,
            ))
        poses[0].x, poses[0].y = start
        poses[-1].x, poses[-1].y = end
        return poses

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
        block_drawing = None
        block_geometry = None
        if (not ghost or detailed) and profile.dxf_block_name:
            block_drawing, block_geometry = self._shared_block_geometry(
                profile.dxf_block_name
            )
        corners = [QPointF(x, -y) for x, y in vehicle_corners(profile, pose)]
        fill = QColor(37, 99, 235, 20 if block_geometry is not None else (60 if ghost else 180))
        outline = QColor("#2563eb") if not ghost else QColor(37, 99, 235, 110)
        body = QGraphicsPolygonItem(QPolygonF(corners))
        body.setBrush(QBrush(fill))
        body.setPen(QPen(outline, 0.04))
        group.addToGroup(body)

        if block_geometry is not None:
            cache_key = (id(block_drawing), profile.dxf_block_name)
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

        if not ghost and direction_override != 0:
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
            angled_for_maneuver = wheel.steerable and (
                pose.maneuver != "point_turn" or wheel.drive
            )
            wheel_angle = pose.heading_deg + (pose.steering_deg if angled_for_maneuver else 0.0)
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
    settings = QSettings("OpenAI", "Vehicle Tracking")
    QtBootstrap.apply(
        app,
        theme=str(settings.value("appearance/theme", "system")),
        dxf_background=str(settings.value("appearance/dxf_background", "")) or None,
    )
    window = VehicleTrackerWindow()
    window.show()
    sys.exit(app.exec())
