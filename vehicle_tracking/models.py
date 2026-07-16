from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from math import cos, hypot, radians, sin, sqrt, tan
from pathlib import Path
from typing import Any
import json
import os


class SteeringMode(str, Enum):
    ACKERMANN_FRONT = "ackermann_front"
    ACKERMANN_REAR = "ackermann_rear"
    FOUR_WHEEL = "four_wheel"
    DIFFERENTIAL = "differential"
    OMNI = "omni"

    @property
    def label(self) -> str:
        return {
            SteeringMode.ACKERMANN_FRONT: "Ackermann front steer",
            SteeringMode.ACKERMANN_REAR: "Rear steer",
            SteeringMode.FOUR_WHEEL: "Four-wheel steer",
            SteeringMode.DIFFERENTIAL: "Differential drive",
            SteeringMode.OMNI: "Omni / holonomic",
        }[self]


@dataclass
class WheelSpec:
    name: str
    x: float
    y: float
    radius: float = 0.18
    width: float = 0.12
    steerable: bool = False
    drive: bool = True

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "WheelSpec":
        return cls(
            name=str(data.get("name", "Wheel")),
            x=float(data.get("x", 0.0)),
            y=float(data.get("y", 0.0)),
            radius=float(data.get("radius", 0.18)),
            width=float(data.get("width", 0.12)),
            steerable=bool(data.get("steerable", False)),
            drive=bool(data.get("drive", True)),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "x": self.x,
            "y": self.y,
            "radius": self.radius,
            "width": self.width,
            "steerable": self.steerable,
            "drive": self.drive,
        }


@dataclass
class VehicleProfile:
    name: str = "Forklift"
    length: float = 2.8
    width: float = 1.2
    steering_mode: SteeringMode = SteeringMode.ACKERMANN_REAR
    wheelbase: float = 1.8
    max_steering_angle_deg: float = 70.0
    min_turning_radius: float = 1.4
    pose_spacing: float = 0.75
    dxf_block_name: str = ""
    block_forward_angle_deg: float = 0.0
    payload_enabled: bool = False
    payload_x: float = 0.0
    payload_y: float = 0.0
    payload_length: float = 1.2
    payload_width: float = 1.0
    payload_rotation_deg: float = 0.0
    load_distance: float = 0.0
    aisle_clearance: float = 200.0
    wheels: list[WheelSpec] = field(default_factory=list)

    def __post_init__(self) -> None:
        if not self.wheels:
            half_width = self.width / 2.0 - 0.16
            half_base = self.wheelbase / 2.0
            steer_rear = self.steering_mode in {
                SteeringMode.ACKERMANN_REAR,
                SteeringMode.FOUR_WHEEL,
            }
            steer_front = self.steering_mode in {
                SteeringMode.ACKERMANN_FRONT,
                SteeringMode.FOUR_WHEEL,
            }
            self.wheels = [
                WheelSpec("Front left", half_base, half_width, steerable=steer_front),
                WheelSpec("Front right", half_base, -half_width, steerable=steer_front),
                WheelSpec("Rear left", -half_base, half_width, steerable=steer_rear),
                WheelSpec("Rear right", -half_base, -half_width, steerable=steer_rear),
            ]

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "VehicleProfile":
        mode_value = data.get("steering_mode", SteeringMode.ACKERMANN_REAR.value)
        try:
            mode = SteeringMode(mode_value)
        except ValueError:
            mode = SteeringMode.ACKERMANN_REAR
        return cls(
            name=str(data.get("name", "Vehicle")),
            length=float(data.get("length", 2.8)),
            width=float(data.get("width", 1.2)),
            steering_mode=mode,
            wheelbase=float(data.get("wheelbase", 1.8)),
            max_steering_angle_deg=float(data.get("max_steering_angle_deg", 70.0)),
            min_turning_radius=float(data.get("min_turning_radius", 1.4)),
            pose_spacing=float(data.get("pose_spacing", 0.75)),
            dxf_block_name=str(data.get("dxf_block_name", "")),
            block_forward_angle_deg=float(data.get("block_forward_angle_deg", 0.0)),
            payload_enabled=bool(data.get("payload_enabled", False)),
            payload_x=float(data.get("payload_x", 0.0)),
            payload_y=float(data.get("payload_y", 0.0)),
            payload_length=float(data.get("payload_length", 1.2)),
            payload_width=float(data.get("payload_width", 1.0)),
            payload_rotation_deg=float(data.get("payload_rotation_deg", 0.0)),
            load_distance=float(data.get("load_distance", 0.0)),
            aisle_clearance=float(data.get("aisle_clearance", 200.0)),
            wheels=[WheelSpec.from_dict(item) for item in data.get("wheels", [])],
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "length": self.length,
            "width": self.width,
            "steering_mode": self.steering_mode.value,
            "wheelbase": self.wheelbase,
            "max_steering_angle_deg": self.max_steering_angle_deg,
            "min_turning_radius": self.min_turning_radius,
            "pose_spacing": self.pose_spacing,
            "dxf_block_name": self.dxf_block_name,
            "block_forward_angle_deg": self.block_forward_angle_deg,
            "payload_enabled": self.payload_enabled,
            "payload_x": self.payload_x,
            "payload_y": self.payload_y,
            "payload_length": self.payload_length,
            "payload_width": self.payload_width,
            "payload_rotation_deg": self.payload_rotation_deg,
            "load_distance": self.load_distance,
            "aisle_clearance": self.aisle_clearance,
            "wheels": [wheel.to_dict() for wheel in self.wheels],
        }

    @property
    def calculated_min_turning_radius(self) -> float:
        """Theoretical radius followed by the vehicle origin."""
        angle = max(0.1, min(abs(self.max_steering_angle_deg), 89.0))
        tangent = max(0.001, abs(tan(radians(angle))))
        wheelbase = max(abs(self.wheelbase), 0.001)
        if self.steering_mode == SteeringMode.ACKERMANN_FRONT:
            return wheelbase / tangent
        if self.steering_mode == SteeringMode.ACKERMANN_REAR:
            fixed_axle_x = self.fixed_axle_x
            axle_radius = wheelbase / tangent
            return hypot(axle_radius, fixed_axle_x)
        if self.steering_mode == SteeringMode.FOUR_WHEEL:
            # Equal front/rear steering in opposite phase doubles yaw contribution.
            return wheelbase / (2.0 * tangent)
        # Differential and omni running gear can rotate/reorient about the centre.
        return 0.0

    @property
    def turning_radius_calculation(self) -> str:
        angle = max(0.1, min(abs(self.max_steering_angle_deg), 89.0))
        if self.steering_mode == SteeringMode.ACKERMANN_FRONT:
            return f"R = wheelbase / tan({angle:.1f} deg)"
        if self.steering_mode == SteeringMode.ACKERMANN_REAR:
            return "Rcentre = sqrt((wheelbase / tan(steer))^2 + fixed-axle offset^2)"
        if self.steering_mode == SteeringMode.FOUR_WHEEL:
            return f"R = wheelbase / (2 x tan({angle:.1f} deg))"
        if self.steering_mode == SteeringMode.DIFFERENTIAL:
            return "R = 0 (counter-rotating driven wheels permit an in-place turn)"
        return "R = 0 (holonomic motion permits in-place reorientation)"

    @property
    def effective_min_turning_radius(self) -> float:
        return max(0.0, self.min_turning_radius, self.calculated_min_turning_radius)

    @property
    def fixed_axle_x(self) -> float:
        fixed = [wheel.x for wheel in self.wheels if not wheel.steerable]
        if fixed:
            return sum(fixed) / len(fixed)
        return self.wheelbase / 2.0

    @property
    def calculated_outer_turning_radius(self) -> float:
        """Toyota Wa-style radius to the furthest truck/load corner for rear steer."""
        if self.steering_mode != SteeringMode.ACKERMANN_REAR:
            return self.calculated_min_turning_radius + self.width / 2.0
        icr_x = self.fixed_axle_x
        centre_radius = self.effective_min_turning_radius
        lateral_icr = sqrt(max(centre_radius * centre_radius - icr_x * icr_x, 0.0))
        corners = [
            (x, y)
            for x in (-self.length / 2.0, self.length / 2.0)
            for y in (-self.width / 2.0, self.width / 2.0)
        ]
        if self.payload_enabled:
            rotation = radians(self.payload_rotation_deg)
            for local_x in (-self.payload_length / 2.0, self.payload_length / 2.0):
                for local_y in (-self.payload_width / 2.0, self.payload_width / 2.0):
                    corners.append((
                        self.payload_x + local_x * cos(rotation) - local_y * sin(rotation),
                        self.payload_y + local_x * sin(rotation) + local_y * cos(rotation),
                    ))
        return max(
            hypot(x - icr_x, y - lateral_icr)
            for x, y in corners
        )

    @property
    def pallet_truck_aisle_width(self) -> float:
        """Toyota pallet/reach-truck Ast formula."""
        rotation = radians(self.payload_rotation_deg)
        load_length = (
            abs(cos(rotation)) * self.payload_length
            + abs(sin(rotation)) * self.payload_width
            if self.payload_enabled else 0.0
        )
        load_width = (
            abs(sin(rotation)) * self.payload_length
            + abs(cos(rotation)) * self.payload_width
            if self.payload_enabled else self.width
        )
        load_distance = self.load_distance
        if self.payload_enabled and load_distance <= 0.0:
            load_rear_face_x = self.payload_x - load_length / 2.0
            load_distance = max(0.0, load_rear_face_x - self.fixed_axle_x)
        return (
            self.calculated_outer_turning_radius
            + sqrt((load_length - load_distance) ** 2 + (load_width / 2.0) ** 2)
            + max(0.0, self.aisle_clearance)
        )

    @property
    def max_turn_curvature(self) -> float:
        radius = self.effective_min_turning_radius
        return 1_000_000.0 if radius < 1e-9 else 1.0 / radius

    @property
    def supports_point_turn(self) -> bool:
        """Whether the configured running gear can rotate the vehicle at a stop."""
        if self.steering_mode == SteeringMode.DIFFERENTIAL:
            return sum(1 for wheel in self.wheels if wheel.drive) >= 2
        return any(wheel.drive and wheel.steerable for wheel in self.wheels)


@dataclass
class Pose:
    x: float
    y: float
    heading_deg: float
    steering_deg: float = 0.0
    maneuver: str = ""

    def transformed_point(self, local_x: float, local_y: float) -> tuple[float, float]:
        heading = radians(self.heading_deg)
        return (
            self.x + local_x * cos(heading) - local_y * sin(heading),
            self.y + local_x * sin(heading) + local_y * cos(heading),
        )


@dataclass
class StartPosition:
    name: str
    level_name: str
    pose: Pose

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "StartPosition":
        pose = data.get("pose", {})
        return cls(
            str(data.get("name", "Start 1")),
            str(data.get("level_name", "Level 1")),
            Pose(
                float(pose.get("x", 0.0)),
                float(pose.get("y", 0.0)),
                float(pose.get("heading_deg", 0.0)),
            ),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "level_name": self.level_name,
            "pose": {
                "x": self.pose.x,
                "y": self.pose.y,
                "heading_deg": self.pose.heading_deg,
            },
        }


@dataclass
class RouteOperation:
    location: str
    operation: str
    waypoint_index: int | None = None

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "RouteOperation":
        index = data.get("waypoint_index")
        return cls(
            str(data.get("location", "waypoint")),
            str(data.get("operation", "travel")),
            int(index) if isinstance(index, (int, float)) else None,
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "location": self.location,
            "operation": self.operation,
            "waypoint_index": self.waypoint_index,
        }


@dataclass
class RoutePlan:
    name: str
    end_pose: Pose
    waypoints: list[tuple[float, float]] = field(default_factory=list)
    point_turn_indices: list[int] = field(default_factory=list)
    reversing_action_indices: list[int] = field(default_factory=list)
    level_name: str = "Level 1"
    start_position_name: str = "Start 1"
    start_pose: Pose | None = None
    tangent_handles: dict[int, tuple[float, float]] = field(default_factory=dict)
    payload_action: str = "none"
    operations: list[RouteOperation] = field(default_factory=list)
    dropoff_pose: Pose | None = None
    point_path_modes: dict[int, str] = field(default_factory=dict)
    dropoff_waypoint_index: int | None = None

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "RoutePlan":
        end = data.get("end_pose", {})
        plan = cls(
            name=str(data.get("name", "Path")),
            end_pose=Pose(
                float(end.get("x", 0.0)),
                float(end.get("y", 0.0)),
                float(end.get("heading_deg", 0.0)),
                0.0,
            ),
            waypoints=[
                (float(point[0]), float(point[1]))
                for point in data.get("waypoints", [])
                if isinstance(point, (list, tuple)) and len(point) >= 2
            ],
            point_turn_indices=sorted(
                {
                    int(index)
                    for index in data.get("point_turn_indices", [])
                    if isinstance(index, (int, float)) and int(index) >= 0
                }
            ),
            reversing_action_indices=sorted(
                {
                    int(index)
                    for index in data.get("reversing_action_indices", [])
                    if isinstance(index, (int, float)) and int(index) >= 0
                }
            ),
            level_name=str(data.get("level_name", "Level 1")),
            start_position_name=str(data.get("start_position_name", "Start 1")),
            start_pose=(
                Pose(
                    float(data["start_pose"].get("x", 0.0)),
                    float(data["start_pose"].get("y", 0.0)),
                    float(data["start_pose"].get("heading_deg", 0.0)),
                )
                if isinstance(data.get("start_pose"), dict)
                else None
            ),
            tangent_handles={
                int(index): (float(value[0]), float(value[1]))
                for index, value in data.get("tangent_handles", {}).items()
                if str(index).lstrip("-").isdigit()
                and int(index) >= 0
                and isinstance(value, (list, tuple))
                and len(value) >= 2
            },
            payload_action=(
                str(data.get("payload_action", "none"))
                if str(data.get("payload_action", "none")) in {"none", "dropoff", "pickup"}
                else "none"
            ),
            operations=[
                RouteOperation.from_dict(item)
                for item in data.get("operations", [])
                if isinstance(item, dict)
            ],
            dropoff_pose=(
                Pose(
                    float(data["dropoff_pose"].get("x", 0.0)),
                    float(data["dropoff_pose"].get("y", 0.0)),
                    float(data["dropoff_pose"].get("heading_deg", 0.0)),
                )
                if isinstance(data.get("dropoff_pose"), dict)
                else None
            ),
            point_path_modes={
                int(index): str(mode)
                for index, mode in data.get("point_path_modes", {}).items()
                if str(index).isdigit() and str(mode) in {"straight", "turn", "minimum_radius"}
            },
            dropoff_waypoint_index=(
                max(0, int(data["dropoff_waypoint_index"]))
                if isinstance(data.get("dropoff_waypoint_index"), (int, float))
                else None
            ),
        )
        if plan.operations and not (
            data.get("point_turn_indices") or data.get("reversing_action_indices")
        ):
            plan.point_turn_indices = sorted(
                operation.waypoint_index
                for operation in plan.operations
                if operation.location == "waypoint"
                and operation.operation == "point_turn"
                and operation.waypoint_index is not None
            )
            plan.reversing_action_indices = sorted(
                operation.waypoint_index
                for operation in plan.operations
                if operation.location == "waypoint"
                and operation.operation == "reverse"
                and operation.waypoint_index is not None
            )
        if plan.operations and not data.get("point_path_modes"):
            plan.point_path_modes = {
                operation.waypoint_index: operation.operation
                for operation in plan.operations
                if operation.location == "waypoint"
                and operation.operation in {"straight", "turn", "minimum_radius"}
                and operation.waypoint_index is not None
            }
        return plan

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "end_pose": {
                "x": self.end_pose.x,
                "y": self.end_pose.y,
                "heading_deg": self.end_pose.heading_deg,
            },
            "waypoints": [[x, y] for x, y in self.waypoints],
            "point_turn_indices": list(self.point_turn_indices),
            "reversing_action_indices": list(self.reversing_action_indices),
            "level_name": self.level_name,
            "start_position_name": self.start_position_name,
            "start_pose": (
                {
                    "x": self.start_pose.x,
                    "y": self.start_pose.y,
                    "heading_deg": self.start_pose.heading_deg,
                }
                if self.start_pose is not None
                else None
            ),
            "tangent_handles": {
                str(index): [vector[0], vector[1]]
                for index, vector in sorted(self.tangent_handles.items())
            },
            "payload_action": self.payload_action,
            "operations": [operation.to_dict() for operation in self.ordered_operations()],
            "dropoff_pose": (
                {
                    "x": self.dropoff_pose.x,
                    "y": self.dropoff_pose.y,
                    "heading_deg": self.dropoff_pose.heading_deg,
                }
                if self.dropoff_pose is not None
                else None
            ),
            "point_path_modes": {
                str(index): mode for index, mode in sorted(self.point_path_modes.items())
            },
            "dropoff_waypoint_index": self.dropoff_waypoint_index,
        }

    def ordered_operations(self) -> list[RouteOperation]:
        if self.operations:
            return list(self.operations)
        operations = [RouteOperation("start", "travel")]
        point_turns = set(self.point_turn_indices)
        reverses = set(self.reversing_action_indices)
        dropoff_index = (
            min(self.dropoff_waypoint_index, len(self.waypoints))
            if self.dropoff_pose is not None and self.dropoff_waypoint_index is not None
            else len(self.waypoints)
        )
        for index in range(len(self.waypoints)):
            if self.dropoff_pose is not None and index == dropoff_index:
                operations.append(RouteOperation("dropoff", "dropoff"))
                operations.append(RouteOperation("dropoff", "reverse"))
            operation = (
                "point_turn"
                if index in point_turns
                else "reverse"
                if index in reverses
                else self.point_path_modes.get(index, "turn")
            )
            operations.append(RouteOperation("waypoint", operation, index))
        if self.dropoff_pose is not None and dropoff_index == len(self.waypoints):
            operations.append(RouteOperation("dropoff", "dropoff"))
            operations.append(RouteOperation("dropoff", "reverse"))
        endpoint_operation = self.payload_action if self.payload_action in {"pickup", "dropoff"} else "stop"
        operations.append(RouteOperation("end", endpoint_operation))
        return operations


def step_pose(
    pose: Pose,
    profile: VehicleProfile,
    steering_deg: float,
    distance: float,
    lateral_distance: float = 0.0,
) -> Pose:
    steering = max(
        -profile.max_steering_angle_deg,
        min(profile.max_steering_angle_deg, steering_deg),
    )
    heading = radians(pose.heading_deg)
    if profile.steering_mode == SteeringMode.OMNI:
        dx = distance * cos(heading) - lateral_distance * sin(heading)
        dy = distance * sin(heading) + lateral_distance * cos(heading)
        return Pose(pose.x + dx, pose.y + dy, pose.heading_deg, steering)

    curvature = tan(radians(steering)) / max(profile.wheelbase, 0.001)
    if profile.steering_mode == SteeringMode.ACKERMANN_REAR:
        curvature *= -1.0
    elif profile.steering_mode == SteeringMode.FOUR_WHEEL:
        curvature *= 2.0
    elif profile.steering_mode == SteeringMode.DIFFERENTIAL:
        curvature = steering / max(profile.max_steering_angle_deg, 0.001)
        curvature *= profile.max_turn_curvature

    curvature = max(-profile.max_turn_curvature, min(profile.max_turn_curvature, curvature))
    delta_heading = curvature * distance
    if abs(curvature) < 1e-6:
        dx = distance * cos(heading)
        dy = distance * sin(heading)
    else:
        radius = 1.0 / curvature
        dx = radius * (sin(heading + delta_heading) - sin(heading))
        dy = -radius * (cos(heading + delta_heading) - cos(heading))
    return Pose(
        pose.x + dx,
        pose.y + dy,
        pose.heading_deg + delta_heading * 180.0 / 3.141592653589793,
        steering,
    )


class VehicleStore:
    def __init__(self, path: Path) -> None:
        self.path = path
        self.vehicles: list[VehicleProfile] = []

    def load(self) -> list[VehicleProfile]:
        if not self.path.exists():
            self.vehicles = [VehicleProfile()]
            self.save()
            return self.vehicles
        data = json.loads(self.path.read_text(encoding="utf-8"))
        self.vehicles = [VehicleProfile.from_dict(item) for item in data.get("vehicles", [])]
        if not self.vehicles:
            self.vehicles = [VehicleProfile()]
        return self.vehicles

    def save(self) -> None:
        payload = {"vehicles": [vehicle.to_dict() for vehicle in self.vehicles]}
        self.path.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    def upsert(self, profile: VehicleProfile) -> None:
        for index, vehicle in enumerate(self.vehicles):
            if vehicle.name == profile.name:
                self.vehicles[index] = profile
                self.save()
                return
        self.vehicles.append(profile)
        self.save()


class RouteStore:
    def __init__(self, path: Path) -> None:
        self.path = path

    @staticmethod
    def _key(drawing_path: Path | None) -> str:
        return str(drawing_path.resolve()).casefold() if drawing_path else "__blank_canvas__"

    def load(self, drawing_path: Path | None) -> tuple[Pose | None, list[RoutePlan]]:
        _levels, starts, routes = self.load_configuration(drawing_path)
        return (starts[0].pose if starts else None), routes

    def load_configuration(
        self, drawing_path: Path | None
    ) -> tuple[list[str], list[StartPosition], list[RoutePlan]]:
        if not self.path.exists():
            return ["Level 1"], [StartPosition("Start 1", "Level 1", Pose(0.0, 0.0, 0.0))], []
        data = json.loads(self.path.read_text(encoding="utf-8"))
        route_set = data.get("drawings", {}).get(self._key(drawing_path), {})
        levels = [str(value) for value in route_set.get("levels", []) if str(value).strip()]
        if not levels:
            levels = ["Level 1"]
        starts = [
            StartPosition.from_dict(item)
            for item in route_set.get("start_positions", [])
            if isinstance(item, dict)
        ]
        start_data = route_set.get("start_pose")
        if not starts:
            pose = Pose(0.0, 0.0, 0.0)
            if isinstance(start_data, dict):
                pose = Pose(
                    float(start_data.get("x", 0.0)),
                    float(start_data.get("y", 0.0)),
                    float(start_data.get("heading_deg", 0.0)),
                )
            starts = [StartPosition("Start 1", levels[0], pose)]
        for start in starts:
            if start.level_name not in levels:
                levels.append(start.level_name)
        routes = [RoutePlan.from_dict(item) for item in route_set.get("routes", [])]
        for route in routes:
            if route.start_pose is None:
                matching = next(
                    (start for start in starts if start.name == route.start_position_name),
                    starts[0],
                )
                route.start_pose = Pose(
                    matching.pose.x, matching.pose.y, matching.pose.heading_deg
                )
                route.start_position_name = matching.name
                route.level_name = matching.level_name
        return levels, starts, routes

    def load_level_drawings(self, drawing_path: Path | None) -> dict[str, Path]:
        if not self.path.exists():
            return {}
        data = json.loads(self.path.read_text(encoding="utf-8"))
        route_set = data.get("drawings", {}).get(self._key(drawing_path), {})
        drawings: dict[str, Path] = {}
        for level, value in route_set.get("level_drawings", {}).items():
            if str(level).strip() and str(value).strip():
                drawings[str(level)] = Path(str(value))
        return drawings

    def save_configuration(
        self,
        drawing_path: Path | None,
        levels: list[str],
        start_positions: list[StartPosition],
        routes: list[RoutePlan],
        level_drawings: dict[str, Path] | None = None,
    ) -> None:
        data: dict[str, Any] = {"drawings": {}}
        if self.path.exists():
            data = json.loads(self.path.read_text(encoding="utf-8"))
            data.setdefault("drawings", {})
        primary = start_positions[0] if start_positions else StartPosition(
            "Start 1", levels[0] if levels else "Level 1", Pose(0.0, 0.0, 0.0)
        )
        key = self._key(drawing_path)
        existing_drawings = data["drawings"].get(key, {}).get("level_drawings", {})
        drawing_values = (
            {level: str(path.resolve()) for level, path in level_drawings.items()}
            if level_drawings is not None
            else existing_drawings
        )
        data["drawings"][key] = {
            "source_dxf": str(drawing_path.resolve()) if drawing_path else "",
            "levels": list(dict.fromkeys(levels or ["Level 1"])),
            "start_positions": [start.to_dict() for start in start_positions],
            "level_drawings": drawing_values,
            "start_pose": {
                "x": primary.pose.x,
                "y": primary.pose.y,
                "heading_deg": primary.pose.heading_deg,
            },
            "routes": [route.to_dict() for route in routes],
        }
        self.path.write_text(json.dumps(data, indent=2), encoding="utf-8")

    def save(self, drawing_path: Path | None, start_pose: Pose, routes: list[RoutePlan]) -> None:
        self.save_configuration(
            drawing_path,
            ["Level 1"],
            [StartPosition("Start 1", "Level 1", start_pose)],
            routes,
        )


@dataclass
class VehicleTrackingProject:
    levels: list[str]
    level_drawings: dict[str, Path]
    start_positions: list[StartPosition]
    routes: list[RoutePlan]
    vehicles: list[VehicleProfile]
    active_level: str
    active_start: str


class ProjectStore:
    VERSION = 1

    @classmethod
    def save(cls, path: Path, project: VehicleTrackingProject) -> None:
        path = path.resolve()

        def portable_path(value: Path) -> str:
            resolved = value.resolve()
            try:
                return os.path.relpath(resolved, path.parent)
            except ValueError:
                return str(resolved)

        payload = {
            "format": "vehicle-tracking-project",
            "version": cls.VERSION,
            "levels": list(project.levels),
            "level_drawings": {
                level: portable_path(drawing)
                for level, drawing in project.level_drawings.items()
            },
            "start_positions": [start.to_dict() for start in project.start_positions],
            "routes": [route.to_dict() for route in project.routes],
            "vehicles": [vehicle.to_dict() for vehicle in project.vehicles],
            "active_level": project.active_level,
            "active_start": project.active_start,
        }
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    @classmethod
    def load(cls, path: Path) -> VehicleTrackingProject:
        path = path.resolve()
        data = json.loads(path.read_text(encoding="utf-8"))
        if data.get("format") != "vehicle-tracking-project":
            raise ValueError("This is not a Vehicle Tracking project file.")
        version = int(data.get("version", 0))
        if version > cls.VERSION:
            raise ValueError(
                f"Project version {version} is newer than this application supports ({cls.VERSION})."
            )
        levels = [str(value) for value in data.get("levels", []) if str(value).strip()]
        if not levels:
            raise ValueError("The project does not contain any floor levels.")
        drawings: dict[str, Path] = {}
        for level, value in data.get("level_drawings", {}).items():
            candidate = Path(str(value))
            if not candidate.is_absolute():
                candidate = (path.parent / candidate).resolve()
            drawings[str(level)] = candidate
        starts = [
            StartPosition.from_dict(item)
            for item in data.get("start_positions", [])
            if isinstance(item, dict)
        ]
        if not starts:
            starts = [StartPosition("Start 1", levels[0], Pose(0.0, 0.0, 0.0))]
        routes = [
            RoutePlan.from_dict(item)
            for item in data.get("routes", [])
            if isinstance(item, dict)
        ]
        vehicles = [
            VehicleProfile.from_dict(item)
            for item in data.get("vehicles", [])
            if isinstance(item, dict)
        ]
        if not vehicles:
            vehicles = [VehicleProfile()]
        active_level = str(data.get("active_level", levels[0]))
        if active_level not in levels:
            active_level = levels[0]
        active_start = str(data.get("active_start", starts[0].name))
        if not any(start.name == active_start for start in starts):
            active_start = starts[0].name
        return VehicleTrackingProject(
            levels, drawings, starts, routes, vehicles, active_level, active_start
        )
