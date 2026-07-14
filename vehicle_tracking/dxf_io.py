from __future__ import annotations

from dataclasses import dataclass, field
from math import cos, radians, sin
from pathlib import Path
from typing import Callable, Iterable

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


def load_dxf(path: Path) -> DxfDrawing:
    doc = ezdxf.readfile(path)
    primitives: list[DxfPrimitive] = []
    block_names: set[str] = set()
    msp = doc.modelspace()
    for entity in msp:
        if entity.dxftype() == "INSERT":
            block_names.add(entity.dxf.name)

    for block in doc.blocks:
        if not block.name.startswith("*"):
            block_names.add(block.name)

    drawing_bounds = _drawing_bounds(msp)
    flattening_distance = _flattening_distance(drawing_bounds)
    primitives, unsupported = _convert_entities(msp, flattening_distance)

    if drawing_bounds is None and primitives:
        drawing_bounds = _primitive_bounds(primitives)
    return DxfDrawing(
        path=path,
        doc=doc,
        primitives=primitives,
        block_names=sorted(block_names),
        bounds=drawing_bounds,
        unsupported_types=tuple(sorted(unsupported)),
    )


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


def _convert_entities(entities, flattening_distance: float) -> tuple[list[DxfPrimitive], set[str]]:
    primitives: list[DxfPrimitive] = []
    unsupported: set[str] = set()
    decomposed = disassemble.recursive_decompose(entities)
    for primitive in disassemble.to_primitives(decomposed, flattening_distance):
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
    points = [point for primitive in primitives for point in primitive.points]
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
        if profile.dxf_block_name and profile.dxf_block_name in source_doc.blocks:
            report(20, "Importing selected vehicle block")
            importer = Importer(source_doc, doc)
            importer.import_block(profile.dxf_block_name, rename=False)
            importer.finalize()
    msp = doc.modelspace()
    report(30, "Creating tracking layers")
    _ensure_layers(doc)

    report(40, "Adding driven path and payload")
    centers = [(pose.x, pose.y) for pose in poses]
    if len(centers) >= 2:
        msp.add_lwpolyline(centers, dxfattribs={"layer": "VT_PATH", "color": 3})
    left, right = envelope_edges(profile, poses)
    if len(left) >= 2:
        msp.add_lwpolyline(left, dxfattribs={"layer": "VT_SWEEP", "color": 1})
        msp.add_lwpolyline(right, dxfattribs={"layer": "VT_SWEEP", "color": 1})
    if profile.payload_enabled and len(poses) >= 2:
        _add_payload_trace(msp, poses, profile, "VT_PAYLOAD_PATH", 4)

    routes = planned_routes if planned_routes is not None else ([planned_poses] if planned_poses else [])
    routes = [route for route in routes if len(route) >= 2]
    report(55, f"Adding {len(routes)} planned route(s) and swept geometry")
    for route_index, route in enumerate(routes):
        planned_centers = [(pose.x, pose.y) for pose in route]
        msp.add_lwpolyline(
            planned_centers,
            dxfattribs={"layer": "VT_PLANNED_ROUTE", "color": 30},
        )
        planned_left, planned_right = envelope_edges(profile, route)
        msp.add_lwpolyline(
            planned_left,
            dxfattribs={"layer": "VT_PLANNED_SWEEP", "color": 2},
        )
        msp.add_lwpolyline(
            planned_right,
            dxfattribs={"layer": "VT_PLANNED_SWEEP", "color": 2},
        )
        if block_outline and len(block_outline) >= 3:
            _add_block_outline_trace(msp, route, block_outline)
        if profile.payload_enabled:
            _add_payload_trace(msp, route, profile, "VT_PLANNED_PAYLOAD", 140)
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
        _add_vehicle_pose(msp, doc, profile, pose)
        report(70 + int(20 * (index + 1) / max(1, len(exported_poses))), "Adding vehicle position history")

    finish_poses = [route[-1] for route in routes]
    if not finish_poses and poses:
        finish_poses = [poses[-1]]
    if finish_poses:
        report(92, f"Adding {len(finish_poses)} finish vehicle and payload position(s)")
    for finish_pose in finish_poses:
        _add_vehicle_pose(
            msp,
            doc,
            profile,
            finish_pose,
            vehicle_layer="VT_FINISH_VEHICLE",
            wheel_layer="VT_FINISH_WHEELS",
        )
        if profile.payload_enabled:
            _add_payload_footprint(msp, finish_pose, profile, "VT_FINISH_PAYLOAD", 4)

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
        ("VT_VEHICLE_POSES", 5),
        ("VT_WHEELS", 6),
        ("VT_FINISH_VEHICLE", 2),
        ("VT_FINISH_WHEELS", 6),
        ("VT_FINISH_PAYLOAD", 4),
    ):
        if name not in doc.layers:
            doc.layers.add(name, color=color)


def _add_block_outline_trace(
    msp,
    poses: list[Pose],
    outline: list[tuple[float, float]],
) -> None:
    transformed = [
        [pose.transformed_point(local_x, local_y) for local_x, local_y in outline]
        for pose in poses
    ]
    for corner_index in range(len(outline)):
        trail = [points[corner_index] for points in transformed]
        msp.add_lwpolyline(
            trail,
            dxfattribs={"layer": "VT_BLOCK_OUTLINE", "color": 6},
        )
    for points in (transformed[0], transformed[-1]):
        msp.add_lwpolyline(
            points + [points[0]],
            dxfattribs={"layer": "VT_BLOCK_OUTLINE", "color": 6},
        )


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
) -> None:
    outline = payload_outline_points(profile)
    centers = [pose.transformed_point(profile.payload_x, profile.payload_y) for pose in poses]
    msp.add_lwpolyline(centers, dxfattribs={"layer": layer, "color": color})
    transformed = [
        [pose.transformed_point(local_x, local_y) for local_x, local_y in outline]
        for pose in poses
    ]
    for corner_index in range(4):
        msp.add_lwpolyline(
            [points[corner_index] for points in transformed],
            dxfattribs={"layer": layer, "color": color},
        )
    for points in (transformed[0], transformed[-1]):
        msp.add_lwpolyline(
            points + [points[0]],
            dxfattribs={"layer": layer, "color": color},
        )


def _add_payload_footprint(
    msp,
    pose: Pose,
    profile: VehicleProfile,
    layer: str,
    color: int,
) -> None:
    points = [pose.transformed_point(x, y) for x, y in payload_outline_points(profile)]
    msp.add_lwpolyline(points, close=True, dxfattribs={"layer": layer, "color": color})


def _add_vehicle_pose(
    msp,
    doc: Drawing,
    profile: VehicleProfile,
    pose: Pose,
    vehicle_layer: str = "VT_VEHICLE_POSES",
    wheel_layer: str = "VT_WHEELS",
) -> None:
    if profile.dxf_block_name and profile.dxf_block_name in doc.blocks:
        msp.add_blockref(
            profile.dxf_block_name,
            (pose.x, pose.y),
            dxfattribs={
                "layer": vehicle_layer,
                "rotation": pose.heading_deg - profile.block_forward_angle_deg,
            },
        )
        _add_wheels(msp, profile, pose, wheel_layer)
        return

    corners = vehicle_corners(profile, pose)
    msp.add_lwpolyline(corners + [corners[0]], dxfattribs={"layer": vehicle_layer, "color": 5})
    nose = [
        pose.transformed_point(profile.length / 2.0, 0),
        pose.transformed_point(profile.length / 2.0 - profile.length * 0.18, profile.width * 0.18),
        pose.transformed_point(profile.length / 2.0 - profile.length * 0.18, -profile.width * 0.18),
        pose.transformed_point(profile.length / 2.0, 0),
    ]
    msp.add_lwpolyline(nose, dxfattribs={"layer": vehicle_layer, "color": 5})
    _add_wheels(msp, profile, pose, wheel_layer)


def _add_wheels(msp, profile: VehicleProfile, pose: Pose, layer: str = "VT_WHEELS") -> None:
    for wheel in profile.wheels:
        wx, wy = pose.transformed_point(wheel.x, wheel.y)
        msp.add_circle((wx, wy), wheel.radius, dxfattribs={"layer": layer, "color": 6})
