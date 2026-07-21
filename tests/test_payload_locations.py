from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest

from vehicle_tracking.models import (
    PayloadLocation,
    FinishPosition,
    Pose,
    ProjectStore,
    RoutePlan,
    StartPosition,
    VehicleProfile,
    VehicleTrackingProject,
)
from vehicle_tracking.app import VehicleTrackerWindow


class PayloadLocationTests(unittest.TestCase):
    def test_project_round_trips_payload_locations_and_route_association(self) -> None:
        route = RoutePlan(
            "Bay A Path",
            Pose(20.0, 0.0, 0.0),
            dropoff_pose=Pose(10.0, 3.0, 90.0),
            dropoff_waypoint_index=0,
            payload_location_name="Bay A",
            finish_position_name="Finish A",
        )
        project = VehicleTrackingProject(
            ["Level 1"],
            {},
            [StartPosition("Start 1", "Level 1", Pose(0.0, 0.0, 0.0))],
            [route],
            [VehicleProfile()],
            "Level 1",
            "Start 1",
            [PayloadLocation("Bay A", "Level 1", Pose(10.0, 3.0, 90.0))],
            [FinishPosition("Finish A", "Level 1", Pose(20.0, 0.0, 0.0))],
        )

        with TemporaryDirectory() as directory:
            path = Path(directory) / "payloads.vtproject"
            ProjectStore.save(path, project)
            restored = ProjectStore.load(path)

        self.assertEqual("Bay A", restored.payload_locations[0].name)
        self.assertEqual(10.0, restored.payload_locations[0].pose.x)
        self.assertEqual("Bay A", restored.routes[0].payload_location_name)
        self.assertEqual("Finish A", restored.finish_positions[0].name)
        self.assertEqual("Finish A", restored.routes[0].finish_position_name)

    def test_older_project_without_payload_locations_remains_valid(self) -> None:
        project = VehicleTrackingProject(
            ["Level 1"],
            {},
            [StartPosition("Start 1", "Level 1", Pose(0.0, 0.0, 0.0))],
            [],
            [VehicleProfile()],
            "Level 1",
            "Start 1",
        )
        with TemporaryDirectory() as directory:
            path = Path(directory) / "legacy.vtproject"
            ProjectStore.save(path, project)
            text = path.read_text(encoding="utf-8").replace(
                ',\n  "payload_locations": []', ""
            )
            path.write_text(text, encoding="utf-8")
            restored = ProjectStore.load(path)

        self.assertEqual([], restored.payload_locations)

    def test_payload_final_pose_converts_to_and_from_vehicle_dropoff_pose(self) -> None:
        profile = VehicleProfile(
            payload_enabled=True,
            payload_x=120.0,
            payload_y=-35.0,
            payload_rotation_deg=20.0,
            payload_length=800.0,
            payload_width=500.0,
        )
        vehicle_pose = Pose(1000.0, 2000.0, 35.0)

        payload_pose = VehicleTrackerWindow._payload_pose_from_vehicle_pose(
            vehicle_pose,
            profile,
        )
        restored_vehicle = VehicleTrackerWindow._vehicle_pose_for_payload_location(
            PayloadLocation("Payload A", "Level 1", payload_pose),
            profile,
        )

        self.assertAlmostEqual(55.0, payload_pose.heading_deg)
        self.assertAlmostEqual(vehicle_pose.x, restored_vehicle.x)
        self.assertAlmostEqual(vehicle_pose.y, restored_vehicle.y)
        self.assertAlmostEqual(vehicle_pose.heading_deg, restored_vehicle.heading_deg)

    def test_payload_group_name_round_trips(self) -> None:
        location = PayloadLocation(
            "Payload A",
            "Level 1",
            Pose(10.0, 20.0, 0.0),
            "Level 1:Payload:1",
        )

        restored = PayloadLocation.from_dict(location.to_dict())

        self.assertEqual("Level 1:Payload:1", restored.group_name)

    def test_rotating_group_preserves_spacing_and_rotates_headings(self) -> None:
        locations = [
            PayloadLocation("A", "Level 1", Pose(10.0, 20.0, 0.0), "group"),
            PayloadLocation("B", "Level 1", Pose(110.0, 20.0, 15.0), "group"),
        ]
        harness = SimpleNamespace(
            payload_locations=locations,
            _sync_payload_locations_to_routes=lambda _affected: None,
        )

        affected = VehicleTrackerWindow._rotate_payload_location_model(
            harness,
            0,
            90.0,
        )

        self.assertEqual({0, 1}, affected)
        self.assertAlmostEqual(10.0, locations[1].pose.x)
        self.assertAlmostEqual(120.0, locations[1].pose.y)
        self.assertAlmostEqual(90.0, locations[0].pose.heading_deg)
        self.assertAlmostEqual(105.0, locations[1].pose.heading_deg)


if __name__ == "__main__":
    unittest.main()
