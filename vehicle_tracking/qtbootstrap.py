from __future__ import annotations

from PySide6.QtCore import QPointF, QSize, Qt
from PySide6.QtGui import QColor, QIcon, QPainter, QPalette, QPen, QPixmap, QPolygonF
from PySide6.QtWidgets import QPushButton


class QtBootstrap:
    PRIMARY = "#2563eb"
    SUCCESS = "#16a34a"
    WARNING = "#d97706"
    DANGER = "#dc2626"
    SURFACE = "#ffffff"
    INK = "#172033"
    MUTED = "#667085"
    _dark = False
    _connected_apps: set[int] = set()
    _theme_mode = "system"
    _dxf_background: str | None = None

    @classmethod
    def apply(
        cls,
        app,
        dark: bool | None = None,
        theme: str | None = None,
        dxf_background: str | None = None,
    ) -> None:
        app.setStyle("Fusion")
        if theme is not None:
            cls._theme_mode = theme if theme in {"system", "light", "dark"} else "system"
        elif dark is not None:
            cls._theme_mode = "dark" if dark else "light"
        cls._dxf_background = dxf_background or None
        effective_dark = (
            cls._detect_dark(app) if cls._theme_mode == "system" else cls._theme_mode == "dark"
        )
        cls._apply_theme(app, effective_dark)
        app_id = id(app)
        if app_id not in cls._connected_apps:
            app.styleHints().colorSchemeChanged.connect(
                lambda scheme, target=app: (
                    cls._apply_theme(
                        target,
                        scheme == Qt.ColorScheme.Dark
                        if scheme != Qt.ColorScheme.Unknown
                        else cls._detect_dark(target),
                    )
                    if cls._theme_mode == "system"
                    else None
                )
            )
            cls._connected_apps.add(app_id)

    @classmethod
    def _detect_dark(cls, app) -> bool:
        scheme = app.styleHints().colorScheme()
        if scheme != Qt.ColorScheme.Unknown:
            return scheme == Qt.ColorScheme.Dark
        return app.palette().color(QPalette.ColorRole.Window).lightness() < 128

    @classmethod
    def is_dark(cls) -> bool:
        return cls._dark

    @classmethod
    def icon_color(cls, variant: str = "") -> str:
        if variant == "secondary":
            return "#e5e7eb" if cls._dark else "#475467"
        return "#ffffff"

    @classmethod
    def semantic_color(cls, role: str) -> str:
        colors = {
            "muted": "#a9b4c4" if cls._dark else "#667085",
            "primary": "#60a5fa" if cls._dark else "#2563eb",
            "success": "#4ade80" if cls._dark else "#15803d",
            "warning": "#fbbf24" if cls._dark else "#b45309",
            "danger": "#f87171" if cls._dark else "#dc2626",
        }
        return colors.get(role, colors["muted"])

    @classmethod
    def style_semantic(cls, widget, role: str) -> None:
        widget.setProperty("themeSemanticRole", role)
        widget.setStyleSheet(f"color: {cls.semantic_color(role)}; font-weight: 700;")

    @classmethod
    def _apply_theme(cls, app, dark: bool) -> None:
        cls._dark = dark
        if dark:
            canvas = "#111827"
            surface = "#1f2937"
            raised = "#273449"
            field = "#111827"
            ink = "#f3f4f6"
            muted = "#a9b4c4"
            border = "#46546a"
            secondary = "#344258"
            secondary_hover = "#43536b"
            selected = "#1e3a5f"
            header = "#29364a"
            groove = "#4b5563"
            canvas_view = cls._dxf_background or "#172033"
            disabled_bg = "#2b3546"
            disabled_ink = "#7f8a9b"
        else:
            canvas = "#f3f6fb"
            surface = "#ffffff"
            raised = "#f5f7fa"
            field = "#ffffff"
            ink = "#172033"
            muted = "#667085"
            border = "#cfd8e7"
            secondary = "#e8eef8"
            secondary_hover = "#d9e4f4"
            selected = "#dbeafe"
            header = "#eef3fb"
            groove = "#d7deea"
            canvas_view = cls._dxf_background or "#f8fafc"
            disabled_bg = "#edf1f7"
            disabled_ink = "#98a2b3"

        palette = QPalette()
        palette.setColor(QPalette.ColorRole.Window, QColor(canvas))
        palette.setColor(QPalette.ColorRole.WindowText, QColor(ink))
        palette.setColor(QPalette.ColorRole.Base, QColor(field))
        palette.setColor(QPalette.ColorRole.AlternateBase, QColor(raised))
        palette.setColor(QPalette.ColorRole.Text, QColor(ink))
        palette.setColor(QPalette.ColorRole.Button, QColor(surface))
        palette.setColor(QPalette.ColorRole.ButtonText, QColor(ink))
        palette.setColor(QPalette.ColorRole.Highlight, QColor(cls.PRIMARY))
        palette.setColor(QPalette.ColorRole.HighlightedText, QColor("#ffffff"))
        palette.setColor(QPalette.ColorRole.PlaceholderText, QColor(muted))
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Text, QColor(disabled_ink))
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ButtonText, QColor(disabled_ink))
        app.setPalette(palette)
        app.setStyleSheet(
            f"""
            QMainWindow, QDialog, QWidget#CentralWidget {{
                background: {canvas};
                color: {ink};
                font-family: Segoe UI, Arial;
                font-size: 10pt;
            }}
            QWidget {{
                color: {ink};
            }}
            QScrollArea, QScrollArea > QWidget > QWidget, QSplitter {{
                background: {canvas};
                border: 0;
            }}
            QToolBar {{
                background: {surface};
                border: 0;
                border-bottom: 1px solid {border};
                spacing: 8px;
                padding: 8px;
            }}
            QTabWidget#RibbonBar::pane {{
                background: {surface};
                border: 0;
                border-bottom: 1px solid {border};
            }}
            QTabWidget#RibbonBar QTabBar::tab {{
                background: {secondary};
                color: {ink};
                border: 0;
                border-right: 1px solid {border};
                padding: 7px 18px;
                font-weight: 700;
            }}
            QTabWidget#RibbonBar QTabBar::tab:selected {{
                background: {surface};
                color: {cls.semantic_color("primary")};
            }}
            QFrame#RibbonGroup {{
                background: {surface};
                border-right: 1px solid {border};
            }}
            QLabel#RibbonGroupTitle {{
                color: {muted};
                font-size: 8pt;
                padding-top: 1px;
            }}
            QLabel#FloorDrawingStatus {{
                background: {raised};
                border: 1px solid {border};
                border-radius: 6px;
                padding: 7px;
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
                background: {secondary};
                color: {ink};
            }}
            QPushButton[variant="secondary"]:hover, QToolButton[variant="secondary"]:hover {{
                background: {secondary_hover};
            }}
            QPushButton:disabled, QToolButton:disabled {{
                background: {disabled_bg};
                color: {disabled_ink};
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
                background: {surface};
                border-left: 1px solid {border};
            }}
            QLabel#PanelTitle {{
                font-size: 13pt;
                font-weight: 700;
                color: {ink};
                padding: 4px 0 8px 0;
            }}
            QLabel#SectionTitle {{
                font-weight: 700;
                color: {ink};
                padding-top: 8px;
            }}
            QLineEdit, QComboBox, QDoubleSpinBox, QSpinBox {{
                background: {field};
                color: {ink};
                border: 1px solid {border};
                border-radius: 6px;
                padding: 5px 7px;
                min-height: 24px;
            }}
            QComboBox QAbstractItemView {{
                background: {field};
                color: {ink};
                border: 1px solid {border};
                selection-background-color: {selected};
                selection-color: {ink};
            }}
            QLineEdit:focus, QComboBox:focus, QDoubleSpinBox:focus, QSpinBox:focus {{
                border: 1px solid {cls.PRIMARY};
            }}
            QTableWidget {{
                background: {field};
                alternate-background-color: {raised};
                color: {ink};
                border: 1px solid {border};
                border-radius: 6px;
                gridline-color: {border};
                selection-background-color: {selected};
                selection-color: {ink};
            }}
            QHeaderView::section {{
                background: {header};
                color: {muted};
                border: 0;
                border-bottom: 1px solid {border};
                padding: 5px;
                font-weight: 700;
            }}
            QGraphicsView {{
                background: {canvas_view};
                border: 0;
            }}
            QMenu {{
                background: {surface};
                color: {ink};
                border: 1px solid {border};
            }}
            QMenu::item:selected {{ background: {selected}; }}
            QCheckBox::indicator {{
                width: 15px;
                height: 15px;
                border: 1px solid {border};
                background: {field};
            }}
            QCheckBox::indicator:checked {{
                background: {cls.PRIMARY};
                border-color: {cls.PRIMARY};
            }}
            QSlider::groove:horizontal {{
                height: 6px;
                background: {groove};
                border-radius: 3px;
            }}
            QSlider::handle:horizontal {{
                background: {cls.PRIMARY};
                width: 16px;
                margin: -5px 0;
                border-radius: 8px;
            }}
            QStatusBar {{
                background: {surface};
                color: {muted};
                border-top: 1px solid {border};
            }}
            QScrollBar:vertical, QScrollBar:horizontal {{
                background: {raised};
                border: 0;
            }}
            QScrollBar::handle:vertical, QScrollBar::handle:horizontal {{
                background: {border};
                border-radius: 5px;
                min-height: 24px;
                min-width: 24px;
            }}
            QToolTip {{
                background: {surface};
                color: {ink};
                border: 1px solid {border};
            }}
            """
        )
        cls._refresh_button_icons(app)
        for widget in app.allWidgets():
            role = widget.property("themeSemanticRole")
            if role:
                cls.style_semantic(widget, str(role))

    @classmethod
    def _refresh_button_icons(cls, app) -> None:
        for button in app.allWidgets():
            if not isinstance(button, QPushButton):
                continue
            icon_name = button.property("themeIconName")
            if icon_name:
                button.setIcon(
                    line_icon(str(icon_name), cls.icon_color(str(button.property("variant") or "")))
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
