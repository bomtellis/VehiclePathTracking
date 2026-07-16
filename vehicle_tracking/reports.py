from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from math import cos, hypot, radians, sin

from .models import Pose
from .dxf_io import DxfPrimitive


@dataclass
class RouteReportEntry:
    name: str
    level_name: str
    start_position_name: str
    start_pose: Pose
    end_pose: Pose
    poses: list[Pose]
    feasible: bool
    impossible_sections: int
    notes: str = ""
    drawing_primitives: list[DxfPrimitive] | None = None
    vehicle_block_primitives: list[DxfPrimitive] | None = None
    vehicle_outline: list[tuple[float, float]] | None = None
    block_forward_angle_deg: float = 0.0

    @property
    def distance(self) -> float:
        return sum(
            hypot(second.x - first.x, second.y - first.y)
            for first, second in zip(self.poses, self.poses[1:])
        )


def generate_route_report_pdf(
    output_path: Path,
    entries: list[RouteReportEntry],
    drawing_name: str = "Untitled drawing",
) -> None:
    try:
        from reportlab.lib import colors
        from reportlab.lib.pagesizes import A4, landscape
        from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
        from reportlab.lib.units import mm
        from reportlab.platypus import (
            KeepTogether,
            PageBreak,
            Paragraph,
            SimpleDocTemplate,
            Spacer,
            Table,
            TableStyle,
        )
        from reportlab.graphics.shapes import Circle, Drawing, Line, PolyLine, Rect, String
    except ImportError as exc:
        raise RuntimeError("PDF reports require reportlab. Install the project requirements first.") from exc

    output_path.parent.mkdir(parents=True, exist_ok=True)
    page_size = landscape(A4)
    styles = getSampleStyleSheet()
    styles["Title"].textColor = colors.HexColor("#172033")
    styles["Heading2"].textColor = colors.HexColor("#2563eb")
    styles["BodyText"].fontSize = 8.5
    styles["BodyText"].leading = 11
    detail_label = ParagraphStyle(
        "DetailLabel",
        parent=styles["BodyText"],
        fontName="Helvetica-Bold",
        fontSize=7.2,
        leading=8.5,
        textColor=colors.HexColor("#172033"),
    )
    detail_value = ParagraphStyle(
        "DetailValue",
        parent=styles["BodyText"],
        fontName="Helvetica",
        fontSize=7.1,
        leading=8.4,
        textColor=colors.HexColor("#172033"),
        splitLongWords=True,
    )

    def paragraph(value, style=detail_value) -> Paragraph:
        escaped = (
            str(value)
            .replace("&", "&amp;")
            .replace("<", "&lt;")
            .replace(">", "&gt;")
        )
        return Paragraph(escaped, style)

    def footer(canvas, document) -> None:
        canvas.saveState()
        canvas.setStrokeColor(colors.HexColor("#d7dee8"))
        canvas.line(15 * mm, 11 * mm, page_size[0] - 15 * mm, 11 * mm)
        canvas.setFont("Helvetica", 7.5)
        canvas.setFillColor(colors.HexColor("#667085"))
        canvas.drawString(15 * mm, 7 * mm, "Vehicle Tracking - Route feasibility report")
        canvas.drawRightString(page_size[0] - 15 * mm, 7 * mm, f"Page {document.page}")
        canvas.restoreState()

    doc = SimpleDocTemplate(
        str(output_path),
        pagesize=page_size,
        leftMargin=15 * mm,
        rightMargin=15 * mm,
        topMargin=13 * mm,
        bottomMargin=15 * mm,
        title="Vehicle Tracking Route Report",
    )
    feasible_count = sum(entry.feasible for entry in entries)
    story = [
        Paragraph("Vehicle Tracking Route Report", styles["Title"]),
        Paragraph(f"Drawing: {drawing_name}", styles["BodyText"]),
        Spacer(1, 4 * mm),
        Table(
            [
                ["Routes", "Possible", "Impossible", "Levels", "Start positions"],
                [
                    str(len(entries)),
                    str(feasible_count),
                    str(len(entries) - feasible_count),
                    str(len({entry.level_name for entry in entries})),
                    str(len({entry.start_position_name for entry in entries})),
                ],
            ],
            colWidths=[35 * mm] * 5,
            style=TableStyle(
                [
                    ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#172033")),
                    ("TEXTCOLOR", (0, 0), (-1, 0), colors.white),
                    ("BACKGROUND", (0, 1), (-1, 1), colors.HexColor("#eef4ff")),
                    ("ALIGN", (0, 0), (-1, -1), "CENTER"),
                    ("FONTNAME", (0, 0), (-1, 0), "Helvetica-Bold"),
                    ("GRID", (0, 0), (-1, -1), 0.5, colors.HexColor("#c7d2e3")),
                    ("TOPPADDING", (0, 0), (-1, -1), 7),
                    ("BOTTOMPADDING", (0, 0), (-1, -1), 7),
                ]
            ),
        ),
        Spacer(1, 6 * mm),
    ]

    for index, entry in enumerate(entries):
        status = "POSSIBLE" if entry.feasible else "IMPOSSIBLE"
        status_color = colors.HexColor("#15803d" if entry.feasible else "#dc2626")
        details = Table(
            [
                [paragraph("Status", detail_label), paragraph(status), paragraph("Level", detail_label), paragraph(entry.level_name), paragraph("Start", detail_label), paragraph(entry.start_position_name)],
                [
                    paragraph("Start position", detail_label),
                    paragraph(f"X {entry.start_pose.x:.3f}, Y {entry.start_pose.y:.3f}, heading {entry.start_pose.heading_deg:.1f} deg"),
                    paragraph("End position", detail_label),
                    paragraph(f"X {entry.end_pose.x:.3f}, Y {entry.end_pose.y:.3f}, heading {entry.end_pose.heading_deg:.1f} deg"),
                    paragraph("Tracking distance", detail_label),
                    paragraph(f"{entry.distance:.3f}"),
                ],
                [paragraph("Impossible sections", detail_label), paragraph(entry.impossible_sections), paragraph("Tracking samples", detail_label), paragraph(len(entry.poses)), paragraph("Notes", detail_label), paragraph(entry.notes or "-")],
            ],
            colWidths=[29 * mm, 51 * mm, 29 * mm, 51 * mm, 29 * mm, 52 * mm],
            style=TableStyle(
                [
                    ("GRID", (0, 0), (-1, -1), 0.35, colors.HexColor("#c7d2e3")),
                    ("BACKGROUND", (0, 0), (-1, -1), colors.white),
                    ("BACKGROUND", (0, 0), (0, -1), colors.HexColor("#f5f7fa")),
                    ("BACKGROUND", (2, 0), (2, -1), colors.HexColor("#f5f7fa")),
                    ("BACKGROUND", (4, 0), (4, -1), colors.HexColor("#f5f7fa")),
                    ("TEXTCOLOR", (1, 0), (1, 0), status_color),
                    ("VALIGN", (0, 0), (-1, -1), "TOP"),
                    ("TOPPADDING", (0, 0), (-1, -1), 5),
                    ("BOTTOMPADDING", (0, 0), (-1, -1), 5),
                    ("LEFTPADDING", (0, 0), (-1, -1), 6),
                    ("RIGHTPADDING", (0, 0), (-1, -1), 6),
                ]
            ),
        )
        drawing = Drawing(735, 190)
        drawing.add(Rect(0, 0, 735, 190, fillColor=colors.HexColor("#f8fafc"), strokeColor=colors.HexColor("#c7d2e3")))
        if entry.poses:
            sample_count = min(8, len(entry.poses))
            sample_indices = sorted(
                {
                    round(sample * (len(entry.poses) - 1) / max(sample_count - 1, 1))
                    for sample in range(sample_count)
                }
                | {
                    pose_index
                    for pose_index, pose in enumerate(entry.poses)
                    if pose.maneuver in {"dropoff", "point_turn"}
                }
            )

            def vehicle_world_point(pose, point):
                angle = radians(pose.heading_deg - entry.block_forward_angle_deg)
                return (
                    pose.x + point[0] * cos(angle) - point[1] * sin(angle),
                    pose.y + point[0] * sin(angle) + point[1] * cos(angle),
                )

            vehicle_samples: list[tuple[Pose, list[list[tuple[float, float]]]]] = []
            for pose_index in sample_indices:
                pose = entry.poses[pose_index]
                primitive_paths = [
                    [vehicle_world_point(pose, point) for point in primitive.points]
                    for primitive in entry.vehicle_block_primitives or []
                    if primitive.kind != "text" and primitive.points
                ]
                if not primitive_paths and entry.vehicle_outline:
                    angle = radians(pose.heading_deg)
                    primitive_paths = [[
                        (
                            pose.x + point[0] * cos(angle) - point[1] * sin(angle),
                            pose.y + point[0] * sin(angle) + point[1] * cos(angle),
                        )
                        for point in [*entry.vehicle_outline, entry.vehicle_outline[0]]
                    ]]
                vehicle_samples.append((pose, primitive_paths))
            xs = [pose.x for pose in entry.poses]
            ys = [pose.y for pose in entry.poses]
            for _pose, paths in vehicle_samples:
                for path in paths:
                    xs.extend(point[0] for point in path)
                    ys.extend(point[1] for point in path)
            route_min_x, route_max_x = min(xs), max(xs)
            route_min_y, route_max_y = min(ys), max(ys)
            route_span_x = max(route_max_x - route_min_x, 1e-6)
            route_span_y = max(route_max_y - route_min_y, 1e-6)
            pad_x = max(route_span_x * 0.08, route_span_y * 0.02)
            pad_y = max(route_span_y * 0.12, route_span_x * 0.02)
            min_x, max_x = route_min_x - pad_x, route_max_x + pad_x
            min_y, max_y = route_min_y - pad_y, route_max_y + pad_y
            span_x = max(max_x - min_x, 1e-6)
            span_y = max(max_y - min_y, 1e-6)
            scale = min(690 / span_x, 145 / span_y)
            offset_x = 22 + (690 - span_x * scale) / 2
            offset_y = 22 + (145 - span_y * scale) / 2
            def map_point(point):
                return (
                    offset_x + (point[0] - min_x) * scale,
                    offset_y + (point[1] - min_y) * scale,
                )
            def clipped_segment(first, second):
                x0, y0 = first
                x1, y1 = second
                dx, dy = x1 - x0, y1 - y0
                lower, upper = 0.0, 1.0
                for p, q in (
                    (-dx, x0 - min_x),
                    (dx, max_x - x0),
                    (-dy, y0 - min_y),
                    (dy, max_y - y0),
                ):
                    if abs(p) < 1e-12:
                        if q < 0:
                            return None
                        continue
                    amount = q / p
                    if p < 0:
                        lower = max(lower, amount)
                    else:
                        upper = min(upper, amount)
                    if lower > upper:
                        return None
                return (
                    (x0 + lower * dx, y0 + lower * dy),
                    (x0 + upper * dx, y0 + upper * dy),
                )
            background_color = colors.HexColor("#aeb8c7")
            for primitive in entry.drawing_primitives or []:
                if not primitive.points or primitive.kind == "text":
                    continue
                primitive_x = [point[0] for point in primitive.points]
                primitive_y = [point[1] for point in primitive.points]
                if max(primitive_x) < min_x or min(primitive_x) > max_x or max(primitive_y) < min_y or min(primitive_y) > max_y:
                    continue
                if len(primitive.points) == 1:
                    point = primitive.points[0]
                    if min_x <= point[0] <= max_x and min_y <= point[1] <= max_y:
                        mapped = map_point(point)
                        drawing.add(Circle(mapped[0], mapped[1], 0.7, fillColor=background_color, strokeColor=None))
                    continue
                for first, second in zip(primitive.points, primitive.points[1:]):
                    segment = clipped_segment(first, second)
                    if segment is None:
                        continue
                    mapped_first, mapped_second = map_point(segment[0]), map_point(segment[1])
                    drawing.add(Line(mapped_first[0], mapped_first[1], mapped_second[0], mapped_second[1], strokeColor=background_color, strokeWidth=0.35))
            points = [
                map_point((pose.x, pose.y))
                for pose in entry.poses
            ]
            drawing.add(PolyLine(points, strokeColor=status_color, strokeWidth=2))
            block_color = colors.HexColor("#2563eb")
            for pose, paths in vehicle_samples:
                for path in paths:
                    if len(path) == 1:
                        mapped = map_point(path[0])
                        drawing.add(Circle(mapped[0], mapped[1], 0.6, fillColor=block_color, strokeColor=None))
                    else:
                        mapped = [map_point(point) for point in path]
                        drawing.add(PolyLine(mapped, strokeColor=block_color, strokeWidth=0.65))
            drawing.add(Circle(points[0][0], points[0][1], 4, fillColor=colors.HexColor("#16a34a"), strokeColor=None))
            drawing.add(Circle(points[-1][0], points[-1][1], 4, fillColor=colors.HexColor("#2563eb"), strokeColor=None))
            drawing.add(String(8, 174, "Route centreline (green=start, blue=end); vehicle block shown along path", fontName="Helvetica", fontSize=7, fillColor=colors.HexColor("#475467")))
        story.append(
            KeepTogether([
                Paragraph(f"{index + 1}. {entry.name}", styles["Heading2"]),
                details,
                Spacer(1, 2 * mm),
                drawing,
            ])
        )
        if index < len(entries) - 1:
            story.append(PageBreak())

    if not entries:
        story.append(Paragraph("No saved routes are available for reporting.", styles["BodyText"]))
    doc.build(story, onFirstPage=footer, onLaterPages=footer)
