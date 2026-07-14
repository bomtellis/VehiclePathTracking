from __future__ import annotations

from PySide6.QtCore import QPointF, QSize, Qt
from PySide6.QtGui import QColor, QIcon, QPainter, QPen, QPixmap, QPolygonF


class QtBootstrap:
    PRIMARY = "#2563eb"
    SUCCESS = "#16a34a"
    WARNING = "#d97706"
    DANGER = "#dc2626"
    SURFACE = "#ffffff"
    INK = "#172033"
    MUTED = "#667085"

    @classmethod
    def apply(cls, app) -> None:
        app.setStyle("Fusion")
        app.setStyleSheet(
            f"""
            QMainWindow, QDialog {{
                background: #f3f6fb;
                color: {cls.INK};
                font-family: Segoe UI, Arial;
                font-size: 10pt;
            }}
            QWidget {{
                color: {cls.INK};
            }}
            QToolBar {{
                background: {cls.SURFACE};
                border: 0;
                border-bottom: 1px solid #d7deea;
                spacing: 8px;
                padding: 8px;
            }}
            QToolButton, QPushButton {{
                background: {cls.PRIMARY};
                color: white;
                border: 0;
                border-radius: 6px;
                padding: 7px 11px;
                min-height: 24px;
                font-weight: 600;
            }}
            QToolButton:hover, QPushButton:hover {{
                background: #1d4ed8;
            }}
            QToolButton:pressed, QPushButton:pressed {{
                background: #1e40af;
            }}
            QPushButton[variant="secondary"], QToolButton[variant="secondary"] {{
                background: #e8eef8;
                color: {cls.INK};
            }}
            QPushButton[variant="secondary"]:hover, QToolButton[variant="secondary"]:hover {{
                background: #d9e4f4;
            }}
            QPushButton[variant="success"] {{
                background: {cls.SUCCESS};
            }}
            QPushButton[variant="warning"] {{
                background: {cls.WARNING};
            }}
            QPushButton[variant="danger"] {{
                background: {cls.DANGER};
            }}
            QFrame#SidePanel {{
                background: {cls.SURFACE};
                border-left: 1px solid #d7deea;
            }}
            QLabel#PanelTitle {{
                font-size: 13pt;
                font-weight: 700;
                color: {cls.INK};
                padding: 4px 0 8px 0;
            }}
            QLabel#SectionTitle {{
                font-weight: 700;
                color: {cls.INK};
                padding-top: 8px;
            }}
            QLineEdit, QComboBox, QDoubleSpinBox, QSpinBox {{
                background: white;
                color: {cls.INK};
                border: 1px solid #cfd8e7;
                border-radius: 6px;
                padding: 5px 7px;
                min-height: 24px;
            }}
            QComboBox QAbstractItemView {{
                background: white;
                color: {cls.INK};
                selection-background-color: #dbeafe;
                selection-color: {cls.INK};
            }}
            QLineEdit:focus, QComboBox:focus, QDoubleSpinBox:focus, QSpinBox:focus {{
                border: 1px solid {cls.PRIMARY};
            }}
            QTableWidget {{
                background: white;
                color: {cls.INK};
                border: 1px solid #d7deea;
                border-radius: 6px;
                gridline-color: #e7ecf4;
                selection-background-color: #dbeafe;
                selection-color: {cls.INK};
            }}
            QHeaderView::section {{
                background: #eef3fb;
                color: {cls.MUTED};
                border: 0;
                border-bottom: 1px solid #d7deea;
                padding: 5px;
                font-weight: 700;
            }}
            QGraphicsView {{
                background: #f8fafc;
                border: 0;
            }}
            QSlider::groove:horizontal {{
                height: 6px;
                background: #d7deea;
                border-radius: 3px;
            }}
            QSlider::handle:horizontal {{
                background: {cls.PRIMARY};
                width: 16px;
                margin: -5px 0;
                border-radius: 8px;
            }}
            QStatusBar {{
                background: {cls.SURFACE};
                color: {cls.MUTED};
                border-top: 1px solid #d7deea;
            }}
            """
        )


def line_icon(name: str, color: str = "#172033") -> QIcon:
    pixmap = QPixmap(QSize(32, 32))
    pixmap.fill(Qt.GlobalColor.transparent)
    painter = QPainter(pixmap)
    painter.setRenderHint(QPainter.RenderHint.Antialiasing)
    pen = QPen(QColor(color), 2.3, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap, Qt.PenJoinStyle.RoundJoin)
    painter.setPen(pen)
    painter.setBrush(Qt.BrushStyle.NoBrush)

    if name == "open":
        painter.drawPolyline(_polyline([(6, 12), (13, 12), (16, 16), (26, 16), (23, 25), (7, 25), (6, 12)]))
        painter.drawLine(8, 9, 15, 9)
    elif name == "export":
        painter.drawRect(8, 18, 16, 7)
        painter.drawLine(16, 6, 16, 18)
        painter.drawLine(11, 11, 16, 6)
        painter.drawLine(21, 11, 16, 6)
    elif name == "save":
        painter.drawRoundedRect(7, 6, 18, 20, 2, 2)
        painter.drawLine(11, 6, 11, 13)
        painter.drawLine(20, 6, 20, 13)
        painter.drawRect(11, 18, 10, 6)
    elif name == "reset":
        painter.drawArc(7, 7, 18, 18, 40 * 16, 280 * 16)
        painter.drawLine(8, 12, 8, 7)
        painter.drawLine(8, 7, 13, 7)
    elif name == "play":
        painter.drawPolygon(_polyline([(11, 8), (24, 16), (11, 24)]))
    elif name == "stop":
        painter.drawRect(10, 10, 12, 12)
    elif name == "left":
        painter.drawLine(22, 8, 10, 16)
        painter.drawLine(10, 16, 22, 24)
    elif name == "right":
        painter.drawLine(10, 8, 22, 16)
        painter.drawLine(22, 16, 10, 24)
    elif name == "vehicle":
        painter.drawRoundedRect(7, 10, 18, 12, 3, 3)
        painter.drawEllipse(9, 20, 5, 5)
        painter.drawEllipse(18, 20, 5, 5)
        painter.drawLine(18, 10, 24, 7)
    elif name == "add":
        painter.drawLine(16, 8, 16, 24)
        painter.drawLine(8, 16, 24, 16)
    elif name == "start":
        painter.drawEllipse(7, 7, 18, 18)
        painter.drawLine(16, 10, 16, 22)
        painter.drawLine(10, 16, 22, 16)
    elif name == "end":
        painter.drawEllipse(7, 7, 18, 18)
        painter.drawLine(11, 11, 21, 21)
        painter.drawLine(11, 21, 21, 11)
    elif name == "fit":
        painter.drawLine(7, 13, 7, 7)
        painter.drawLine(7, 7, 13, 7)
        painter.drawLine(25, 13, 25, 7)
        painter.drawLine(25, 7, 19, 7)
        painter.drawLine(7, 19, 7, 25)
        painter.drawLine(7, 25, 13, 25)
        painter.drawLine(25, 19, 25, 25)
        painter.drawLine(25, 25, 19, 25)
    elif name == "wheel":
        painter.drawEllipse(7, 7, 18, 18)
        painter.drawEllipse(12, 12, 8, 8)
        painter.drawLine(16, 7, 16, 12)
        painter.drawLine(16, 20, 16, 25)
        painter.drawLine(7, 16, 12, 16)
        painter.drawLine(20, 16, 25, 16)
    elif name == "direction":
        painter.drawLine(6, 16, 23, 16)
        painter.drawLine(17, 10, 23, 16)
        painter.drawLine(17, 22, 23, 16)
    else:
        painter.drawEllipse(8, 8, 16, 16)
    painter.end()
    return QIcon(pixmap)


def _polyline(points: list[tuple[float, float]]) -> QPolygonF:
    return QPolygonF([QPointF(x, y) for x, y in points])
