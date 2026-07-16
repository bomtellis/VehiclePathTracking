from __future__ import annotations

from dataclasses import dataclass, field
from math import cos, radians, sin
from pathlib import Path
from typing import Callable, Iterable
import re

import ezdxf
from ezdxf import bbox, disassemble
from ezdxf.addons import Importer
from ezdxf.document import Drawing

from .models import Pose, VehicleProfile


@dataclass
class DxfPrimitive:
    kind: str
    points: list[tuple[float, float]]
    radius: float = 0.0
    start_angle: float = 0.0
    end_angle: float = 0.0
    layer: str = "0"
    text: str = ""
    text_height: float = 0.0
    rotation_deg: float = 0.0
    horizontal_alignment: str = "left"
    vertical_alignment: str = "baseline"
    width_factor: float = 1.0


@dataclass
class DxfDrawing:
    path: Path
    doc: Drawing
    primitives: list[DxfPrimitive]
    block_names: list[str]
    bounds: tuple[float, float, float, float] | None = None
    unsupported_types: tuple[str, ...] = ()
    block_geometries: dict[str, "DxfBlockGeometry"] = field(default_factory=dict)


@dataclass
class DxfBlockGeometry:
    name: str
    primitives: list[DxfPrimitive]
    bounds: tuple[float, float, float, float]


def load_dxf(path: Path, progress_callback: Callable[[int, str], None] | None = None) -> DxfDrawing:
    def report(value: int, message: str) -> None:
        if progress_callback is not None:
            progress_callback(value, message)

    report(5, "Reading DXF file")
    doc = ezdxf.readfile(path)
    primitives: list[DxfPrimitive] = []
    block_names: set[str] = set()
    msp = doc.modelspace()
    report(20, "Scanning blocks and inserts")
    for entity in msp:
        if entity.dxftype() == "INSERT":
            block_names.add(entity.dxf.name)

    for block in doc.blocks:
        if not block.name.startswith("*"):
            block_names.add(block.name)

    report(35, "Calculating drawing bounds")
    drawing_bounds = _drawing_bounds(msp)
    flattening_distance = _flattening_distance(drawing_bounds)
    report(50, "Converting drawing entities")
    primitives, unsupported = _convert_entities(msp, flattening_distance, report)

    if drawing_bounds is None and primitives:
        drawing_bounds = _primitive_bounds(primitives)
    report(92, "Finalising drawing")
    drawing = DxfDrawing(
        path=path,
        doc=doc,
        primitives=primitives,
        block_names=sorted(block_names),
        bounds=drawing_bounds,
        unsupported_types=tuple(sorted(unsupported)),
    )
    report(100, "DXF ready")
    return drawing


def get_block_geometry(drawing: DxfDrawing, block_name: str) -> DxfBlockGeometry | None:
    if not block_name or block_name not in drawing.doc.blocks:
        return None
    cached = drawing.block_geometries.get(block_name)
    if cached is not None:
        return cached

    block = drawing.doc.blocks.get(block_name)
    raw_bounds = _drawing_bounds(block)
    flattening_distance = _flattening_distance(raw_bounds)
    primitives, _unsupported = _convert_entities(block, flattening_distance)
    if not primitives:
        return None

    base = block.block.dxf.base_point
    base_x = float(base.x)
    base_y = float(base.y)
    for primitive in primitives:
        primitive.points = [(x - base_x, y - base_y) for x, y in primitive.points]
    bounds = _primitive_bounds(primitives)
    geometry = DxfBlockGeometry(block_name, primitives, bounds)
    drawing.block_geometries[block_name] = geometry
    return geometry


def _convert_entities(
    entities,
    flattening_distance: float,
    progress_callback: Callable[[int, str], None] | None = None,
) -> tuple[list[DxfPrimitive], set[str]]:
    primitives: list[DxfPrimitive] = []
    unsupported: set[str] = set()
    decomposed = list(disassemble.recursive_decompose(entities))
    if progress_callback is not None:
        progress_callback(58, f"Preparing {len(decomposed):,} entities")
    geometry_entities = []
    for entity in decomposed:
        text_primitive = _text_primitive(entity)
        if text_primitive is not None:
            primitives.append(text_primitive)
        else:
            geometry_entities.append(entity)
    total = max(len(geometry_entities), 1)
    for converted_index, primitive in enumerate(
        disassemble.to_primitives(geometry_entities, flattening_distance), start=1
    ):
        if progress_callback is not None and (
            converted_index == 1 or converted_index % 250 == 0
        ):
            progress_callback(
                min(90, 60 + int(30 * converted_index / total)),
                f"Converting entity {converted_index:,} of approximately {total:,}",
            )
        entity = primitive.entity
        layer = entity.dxf.layer if entity.dxf.hasattr("layer") else "0"
        if primitive.is_empty:
            unsupported.add(entity.dxftype())
            continue

        if primitive.path is not None:
            for sub_path in primitive.path.sub_paths():
                points = [(float(vertex.x), float(vertex.y)) for vertex in sub_path.flattening(flattening_distance)]
                if len(points) >= 2:
                    primitives.append(DxfPrimitive("polyline", points, layer=layer))
                elif points:
                    primitives.append(DxfPrimitive("point", points, layer=layer))
        elif primitive.mesh is not None:
            for face in primitive.mesh.faces:
                points = [(float(vertex.x), float(vertex.y)) for vertex in face]
                if len(points) >= 2:
                    if points[0] != points[-1]:
                        points.append(points[0])
                    primitives.append(DxfPrimitive("polyline", points, layer=layer))
        else:
            points = [(float(vertex.x), float(vertex.y)) for vertex in primitive.vertices()]
            if len(points) >= 2:
                primitives.append(DxfPrimitive("polyline", points, layer=layer))
            elif points:
                primitives.append(DxfPrimitive("point", points, layer=layer))
    return primitives, unsupported


def _text_primitive(entity) -> DxfPrimitive | None:
    kind = entity.dxftype()
    if kind not in {"TEXT", "MTEXT", "ATTRIB", "ATTDEF"}:
        return None
    layer = entity.dxf.layer if entity.dxf.hasattr("layer") else "0"
    if kind == "MTEXT":
        text = entity.plain_text() if hasattr(entity, "plain_text") else str(entity.text)
        insertion = entity.dxf.insert
        height = float(entity.dxf.char_height)
        rotation = float(entity.get_rotation())
        attachment = int(entity.dxf.attachment_point)
        horizontal = {1: "left", 2: "center", 3: "right", 4: "left", 5: "center", 6: "right", 7: "left", 8: "center", 9: "right"}.get(attachment, "left")
        vertical = {1: "top", 2: "top", 3: "top", 4: "middle", 5: "middle", 6: "middle", 7: "bottom", 8: "bottom", 9: "bottom"}.get(attachment, "top")
        return DxfPrimitive(
            "text",
            [(float(insertion.x), float(insertion.y))],
            layer=layer,
            text=text,
            text_height=max(height, 1e-9),
            rotation_deg=rotation,
            horizontal_alignment=horizontal,
            vertical_alignment=vertical,
        )

    text = str(entity.dxf.text)
    insert = entity.dxf.insert
    halign = int(entity.dxf.halign) if entity.dxf.hasattr("halign") else 0
    valign = int(entity.dxf.valign) if entity.dxf.hasattr("valign") else 0
    if (halign or valign) and entity.dxf.hasattr("align_point"):
        align_point = entity.dxf.align_point
        if align_point is not None:
            insert = align_point
    horizontal = {0: "left", 1: "center", 2: "right", 3: "center", 4: "center", 5: "center"}.get(halign, "left")
    vertical = {0: "baseline", 1: "bottom", 2: "middle", 3: "top"}.get(valign, "baseline")
    return DxfPrimitive(
        "text",
        [(float(insert.x), float(insert.y))],
        layer=layer,
        text=text,
        text_height=max(float(entity.dxf.height), 1e-9),
        rotation_deg=float(entity.dxf.rotation) if entity.dxf.hasattr("rotation") else 0.0,
        horizontal_alignment=horizontal,
        vertical_alignment=vertical,
        width_factor=max(float(entity.dxf.width), 1e-9) if entity.dxf.hasattr("width") else 1.0,
    )


def _drawing_bounds(modelspace) -> tuple[float, float, float, float] | None:
    try:
        extents = bbox.extents(modelspace, fast=True)
    except Exception:
        return None
    if not extents.has_data:
        return None
    return (
        float(extents.extmin.x),
        float(extents.extmin.y),
        float(extents.extmax.x),
        float(extents.extmax.y),
    )


def _flattening_distance(bounds: tuple[float, float, float, float] | None) -> float:
    if bounds is None:
        return 0.01
    min_x, min_y, max_x, max_y = bounds
    diagonal = ((max_x - min_x) ** 2 + (max_y - min_y) ** 2) ** 0.5
    return max(diagonal / 20_000.0, 0.001)


def _primitive_bounds(primitives: list[DxfPrimitive]) -> tuple[float, float, float, float]:
    points = []
    for primitive in primitives:
        points.extend(primitive.points)
        if primitive.kind == "text" and primitive.points:
            x, y = primitive.points[0]
            estimated_width = max(len(line) for line in primitive.text.splitlines() or [""])
            estimated_width *= primitive.text_height * 0.65 * primitive.width_factor
            estimated_height = max(1, len(primitive.text.splitlines())) * primitive.text_height * 1.25
            points.append((x + estimated_width, y + estimated_height))
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    return min(xs), min(ys), max(xs), max(ys)


def vehicle_corners(profile: VehicleProfile, pose: Pose) -> list[tuple[float, float]]:
    half_l = profile.length / 2.0
    half_w = profile.width / 2.0
    local = [(-half_l, -half_w), (half_l, -half_w), (half_l, half_w), (-half_l, half_w)]
    return [pose.transformed_point(x, y) for x, y in local]


def envelope_edges(profile: VehicleProfile, poses: Iterable[Pose]) -> tuple[list[tuple[float, float]], list[tuple[float, float]]]:
    left: list[tuple[float, float]] = []
    right: list[tuple[float, float]] = []
    for pose in poses:
        heading = radians(pose.heading_deg)
        left.append((pose.x - profile.width / 2.0 * sin(heading), pose.y + profile.width / 2.0 * cos(heading)))
        right.append((pose.x + profile.width / 2.0 * sin(heading), pose.y - profile.width / 2.0 * cos(heading)))
    return left, right


def export_tracking_dxf(
    source_path: Path | None,
    output_path: Path,
    profile: VehicleProfile,
    poses: list[Pose],
    planned_poses: list[Pose] | None = None,
    block_outline: list[tuple[float, float]] | None = None,
    progress_callback: Callable[[int, str], None] | None = None,
    planned_routes: list[list[Pose]] | None = None,
    planned_route_names: list[str] | None = None,
    block_source_path: Path | None = None,
) -> None:
    def report(value: int, message: str) -> None:
        if progress_callback is not None:
            progress_callback(value, message)

    report(0, "Preparing clean DXF")
    doc = ezdxf.new("R2010")
    if source_path and source_path.exists():
        report(10, "Reading source coordinate metadata")
        source_doc = ezdxf.readfile(source_path)
        _copy_coordinate_metadata(source_doc, doc)
    block_doc = None
    if block_source_path and block_source_path.exists():
        block_doc = ezdxf.readfile(block_source_path)
    elif source_path and source_path.exists():
        block_doc = source_doc
    if block_doc is not None and profile.dxf_block_name in block_doc.blocks:
        report(20, "Importing selected vehicle block")
        importer = Importer(block_doc, doc)
        importer.import_block(profile.dxf_block_name, rename=False)
        importer.finalize()
    msp = doc.modelspace()
    report(30, "Creating tracking layers")
    _ensure_layers(doc)

    report(40, "Adding driven path and payload")
    driven_entities = []
    centers = [(pose.x, pose.y) for pose in poses]
    if len(centers) >= 2:
        driven_entities.append(
            msp.add_lwpolyline(centers, dxfattribs={"layer": "VT_PATH", "color": 3})
        )
    left, right = envelope_edges(profile, poses)
    if len(left) >= 2:
        driven_entities.append(
            msp.add_lwpolyline(left, dxfattribs={"layer": "VT_SWEEP", "color": 1})
        )
        driven_entities.append(
            msp.add_lwpolyline(right, dxfattribs={"layer": "VT_SWEEP", "color": 1})
        )
    if profile.payload_enabled and len(poses) >= 2:
        driven_entities.extend(_add_payload_trace(msp, poses, profile, "VT_PAYLOAD_PATH", 4))

    source_routes = planned_routes if planned_routes is not None else ([planned_poses] if planned_poses else [])
    route_entries = [
        (
            route,
            planned_route_names[index]
            if planned_route_names and index < len(planned_route_names)
            else f"Path {index + 1}",
        )
        for index, route in enumerate(source_routes)
        if len(route) >= 2
    ]
    routes = [route for route, _name in route_entries]
    report(55, f"Adding {len(routes)} planned route(s) and swept geometry")
    for route_index, (route, route_name) in enumerate(route_entries):
        route_entities = []
        planned_centers = [(pose.x, pose.y) for pose in route]
        route_entities.append(
            msp.add_lwpolyline(
                planned_centers,
                dxfattribs={"layer": "VT_PLANNED_ROUTE", "color": 30},
            )
        )
        planned_left, planned_right = envelope_edges(profile, route)
        route_entities.append(
            msp.add_lwpolyline(
                planned_left,
                dxfattribs={"layer": "VT_PLANNED_SWEEP", "color": 2},
            )
        )
        route_entities.append(
            msp.add_lwpolyline(
                planned_right,
                dxfattribs={"layer": "VT_PLANNED_SWEEP", "color": 2},
            )
        )
        if block_outline and len(block_outline) >= 3:
            route_entities.extend(_add_block_outline_trace(msp, route, block_outline))
        if profile.payload_enabled:
            route_entities.extend(
                _add_payload_trace(msp, route, profile, "VT_PLANNED_PAYLOAD", 140)
            )
        route_entities.extend(_add_route_action_markers(msp, route, profile))
        route_entities.extend(
            _add_vehicle_pose(
                msp,
                doc,
                profile,
                route[-1],
                vehicle_layer="VT_FINISH_VEHICLE",
                wheel_layer="VT_FINISH_WHEELS",
            )
        )
        if profile.payload_enabled:
            route_entities.extend(
                _add_payload_footprint(msp, route[-1], profile, "VT_FINISH_PAYLOAD", 4)
            )
        _create_entity_group(
            doc,
            f"VT_PATH_{route_index + 1:03d}_{route_name}",
            route_entities,
            f"Vehicle Tracking planned path: {route_name}",
        )
        report(
            55 + int(14 * (route_index + 1) / max(1, len(routes))),
            f"Adding planned route {route_index + 1} of {len(routes)}",
        )

    report(70, "Adding vehicle position history")
    stride = max(1, int(round(profile.pose_spacing / 0.25)))
    exported_poses = [
        pose
        for index, pose in enumerate(poses)
        if index % stride == 0 or index == len(poses) - 1
    ]
    for index, pose in enumerate(exported_poses):
        driven_entities.extend(_add_vehicle_pose(msp, doc, profile, pose))
        report(70 + int(20 * (index + 1) / max(1, len(exported_poses))), "Adding vehicle position history")

    if not routes and poses:
        report(92, "Adding finish vehicle and payload position")
        driven_entities.extend(
            _add_vehicle_pose(
                msp,
                doc,
                profile,
                poses[-1],
                vehicle_layer="VT_FINISH_VEHICLE",
                wheel_layer="VT_FINISH_WHEELS",
            )
        )
        if profile.payload_enabled:
            driven_entities.extend(
                _add_payload_footprint(msp, poses[-1], profile, "VT_FINISH_PAYLOAD", 4)
            )
    _create_entity_group(
        doc,
        "VT_DRIVEN_PATH",
        driven_entities,
        "Vehicle Tracking driven path",
    )

    report(98, "Writing DXF file")
    doc.saveas(output_path)
    report(100, "Export complete")


def _copy_coordinate_metadata(source: Drawing, target: Drawing) -> None:
    target.units = source.units
    for name in (
        "$INSBASE",
        "$INSUNITS",
        "$MEASUREMENT",
        "$LUNITS",
        "$LUPREC",
        "$AUNITS",
        "$AUPREC",
    ):
        try:
            target.header[name] = source.header[name]
        except (KeyError, TypeError, ValueError):
            continue


def _ensure_layers(doc: Drawing) -> None:
    for name, color in (
        ("VT_PATH", 3),
        ("VT_SWEEP", 1),
        ("VT_PLANNED_ROUTE", 30),
        ("VT_PLANNED_SWEEP", 2),
        ("VT_BLOCK_OUTLINE", 6),
        ("VT_PAYLOAD_PATH", 4),
        ("VT_PLANNED_PAYLOAD", 140),
        ("VT_ROUTE_ACTIONS", 1),
        ("VT_VEHICLE_POSES", 5),
        ("VT_WHEELS", 6),
        ("VT_FINISH_VEHICLE", 2),
        ("VT_FINISH_WHEELS", 6),
        ("VT_FINISH_PAYLOAD", 4),
    ):
        if name not in doc.layers:
            doc.layers.add(name, color=color)


def _add_route_action_markers(msp, poses: list[Pose], profile: VehicleProfile) -> list:
    entities = []
    marker_radius = max(profile.width * 0.12, profile.length * 0.05)
    for previous, current in zip(poses, poses[1:]):
        was_reverse = previous.maneuver == "reverse"
        is_reverse = current.maneuver == "reverse"
        if was_reverse == is_reverse:
            continue
        x, y = previous.x, previous.y
        attributes = {"layer": "VT_ROUTE_ACTIONS", "color": 1}
        entities.append(msp.add_circle((x, y), marker_radius, dxfattribs=attributes))
        entities.append(
            msp.add_line(
                (x - marker_radius, y - marker_radius),
                (x + marker_radius, y + marker_radius),
                dxfattribs=attributes,
            )
        )
        entities.append(
            msp.add_line(
                (x - marker_radius, y + marker_radius),
                (x + marker_radius, y - marker_radius),
                dxfattribs=attributes,
            )
        )
    return entities


def _create_entity_group(
    doc: Drawing,
    requested_name: str,
    entities: list,
    description: str,
) -> None:
    if not entities:
        return
    safe_name = re.sub(r"[^A-Za-z0-9_-]+", "_", requested_name).strip("_")
    safe_name = (safe_name or "VT_PATH")[:250]
    candidate = safe_name
    suffix = 2
    while doc.groups.get(candidate) is not None:
        candidate = f"{safe_name[:244]}_{suffix}"
        suffix += 1
    doc.groups.new(candidate, description=description, selectable=True).extend(entities)


def _add_block_outline_trace(
    msp,
    poses: list[Pose],
    outline: list[tuple[float, float]],
) -> list:
    entities = []
    transformed = [
        [pose.transformed_point(local_x, local_y) for local_x, local_y in outline]
        for pose in poses
    ]
    for corner_index in range(len(outline)):
        trail = [points[corner_index] for points in transformed]
        entities.append(
            msp.add_lwpolyline(
                trail,
                dxfattribs={"layer": "VT_BLOCK_OUTLINE", "color": 6},
            )
        )
    for points in (transformed[0], transformed[-1]):
        entities.append(
            msp.add_lwpolyline(
                points + [points[0]],
                dxfattribs={"layer": "VT_BLOCK_OUTLINE", "color": 6},
            )
        )
    return entities


def payload_outline_points(profile: VehicleProfile) -> list[tuple[float, float]]:
    half_length = profile.payload_length / 2.0
    half_width = profile.payload_width / 2.0
    angle = radians(profile.payload_rotation_deg)
    result: list[tuple[float, float]] = []
    for local_x, local_y in (
        (-half_length, -half_width),
        (half_length, -half_width),
        (half_length, half_width),
        (-half_length, half_width),
    ):
        result.append(
            (
                profile.payload_x + local_x * cos(angle) - local_y * sin(angle),
                profile.payload_y + local_x * sin(angle) + local_y * cos(angle),
            )
        )
    return result


def _add_payload_trace(
    msp,
    poses: list[Pose],
    profile: VehicleProfile,
    layer: str,
    color: int,
) -> list:
    dropoff_index = next(
        (index for index, pose in enumerate(poses) if pose.maneuver == "dropoff"),
        None,
    )
    if dropoff_index is not None:
        poses = poses[: dropoff_index + 1]
    entities = []
    outline = payload_outline_points(profile)
    centers = [pose.transformed_point(profile.payload_x, profile.payload_y) for pose in poses]
    entities.append(msp.add_lwpolyline(centers, dxfattribs={"layer": layer, "color": color}))
    transformed = [
        [pose.transformed_point(local_x, local_y) for local_x, local_y in outline]
        for pose in poses
    ]
    for corner_index in range(4):
        entities.append(
            msp.add_lwpolyline(
                [points[corner_index] for points in transformed],
                dxfattribs={"layer": layer, "color": color},
            )
        )
    for points in (transformed[0], transformed[-1]):
        entities.append(
            msp.add_lwpolyline(
                points + [points[0]],
                dxfattribs={"layer": layer, "color": color},
            )
        )
    return entities


def _add_payload_footprint(
    msp,
    pose: Pose,
    profile: VehicleProfile,
    layer: str,
    color: int,
) -> list:
    points = [pose.transformed_point(x, y) for x, y in payload_outline_points(profile)]
    return [msp.add_lwpolyline(points, close=True, dxfattribs={"layer": layer, "color": color})]


def _add_vehicle_pose(
    msp,
    doc: Drawing,
    profile: VehicleProfile,
    pose: Pose,
    vehicle_layer: str = "VT_VEHICLE_POSES",
    wheel_layer: str = "VT_WHEELS",
) -> list:
    entities = []
    if profile.dxf_block_name and profile.dxf_block_name in doc.blocks:
        entities.append(
            msp.add_blockref(
                profile.dxf_block_name,
                (pose.x, pose.y),
                dxfattribs={
                    "layer": vehicle_layer,
                    "rotation": pose.heading_deg - profile.block_forward_angle_deg,
                },
            )
        )
        entities.extend(_add_wheels(msp, profile, pose, wheel_layer))
        return entities

    corners = vehicle_corners(profile, pose)
    entities.append(
        msp.add_lwpolyline(
            corners + [corners[0]], dxfattribs={"layer": vehicle_layer, "color": 5}
        )
    )
    nose = [
        pose.transformed_point(profile.length / 2.0, 0),
        pose.transformed_point(profile.length / 2.0 - profile.length * 0.18, profile.width * 0.18),
        pose.transformed_point(profile.length / 2.0 - profile.length * 0.18, -profile.width * 0.18),
        pose.transformed_point(profile.length / 2.0, 0),
    ]
    entities.append(msp.add_lwpolyline(nose, dxfattribs={"layer": vehicle_layer, "color": 5}))
    entities.extend(_add_wheels(msp, profile, pose, wheel_layer))
    return entities


def _add_wheels(msp, profile: VehicleProfile, pose: Pose, layer: str = "VT_WHEELS") -> list:
    entities = []
    for wheel in profile.wheels:
        wx, wy = pose.transformed_point(wheel.x, wheel.y)
        entities.append(
            msp.add_circle((wx, wy), wheel.radius, dxfattribs={"layer": layer, "color": 6})
        )
    return entities
