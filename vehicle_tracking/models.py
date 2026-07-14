from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from math import cos, radians, sin, tan
from pathlib import Path
from typing import Any
import json


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
            "wheels": [wheel.to_dict() for wheel in self.wheels],
        }

    @property
    def max_turn_curvature(self) -> float:
        angle = max(0.1, min(abs(self.max_steering_angle_deg), 89.0))
        radius_from_angle = self.wheelbase / max(0.001, tan(radians(angle)))
        radius = max(self.min_turning_radius, abs(radius_from_angle))
        return 1.0 / radius


@dataclass
class Pose:
    x: float
    y: float
    heading_deg: float
    steering_deg: float = 0.0

    def transformed_point(self, local_x: float, local_y: float) -> tuple[float, float]:
        heading = radians(self.heading_deg)
        return (
            self.x + local_x * cos(heading) - local_y * sin(heading),
            self.y + local_x * sin(heading) + local_y * cos(heading),
        )


@dataclass
class RoutePlan:
    name: str
    end_pose: Pose
    waypoints: list[tuple[float, float]] = field(default_factory=list)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "RoutePlan":
        end = data.get("end_pose", {})
        return cls(
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
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "end_pose": {
                "x": self.end_pose.x,
                "y": self.end_pose.y,
                "heading_deg": self.end_pose.heading_deg,
            },
            "waypoints": [[x, y] for x, y in self.waypoints],
        }


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
        curvature *= 1.75
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
        if not self.path.exists():
            return None, []
        data = json.loads(self.path.read_text(encoding="utf-8"))
        route_set = data.get("drawings", {}).get(self._key(drawing_path), {})
        start_data = route_set.get("start_pose")
        start_pose = None
        if isinstance(start_data, dict):
            start_pose = Pose(
                float(start_data.get("x", 0.0)),
                float(start_data.get("y", 0.0)),
                float(start_data.get("heading_deg", 0.0)),
                0.0,
            )
        routes = [RoutePlan.from_dict(item) for item in route_set.get("routes", [])]
        return start_pose, routes

    def save(self, drawing_path: Path | None, start_pose: Pose, routes: list[RoutePlan]) -> None:
        data: dict[str, Any] = {"drawings": {}}
        if self.path.exists():
            data = json.loads(self.path.read_text(encoding="utf-8"))
            data.setdefault("drawings", {})
        data["drawings"][self._key(drawing_path)] = {
            "source_dxf": str(drawing_path.resolve()) if drawing_path else "",
            "start_pose": {
                "x": start_pose.x,
                "y": start_pose.y,
                "heading_deg": start_pose.heading_deg,
            },
            "routes": [route.to_dict() for route in routes],
        }
        self.path.write_text(json.dumps(data, indent=2), encoding="utf-8")
