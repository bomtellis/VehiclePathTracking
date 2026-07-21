import os
from pathlib import Path
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication, QGraphicsPolygonItem

from vehicle_tracking.app import VehicleTrackerWindow
from vehicle_tracking.dxf_io import DxfBlockGeometry, DxfDrawing, DxfPrimitive
from vehicle_tracking.models import (
    Obstacle,
    Pose,
    RoutePlan,
    SteeringMode,
    VehicleProfile,
    WheelSpec,
)


class VehicleBlockBoundsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self) -> None:
        self.window = VehicleTrackerWindow()
        self.profile = VehicleProfile(
            name="Block vehicle",
            length=100.0,
            width=100.0,
            dxf_block_name="TEST_VEHICLE",
            block_forward_angle_deg=90.0,
        )
        primitives = [
            DxfPrimitive(
                "polyline",
                [(-2.0, -1.0), (1.0, -1.0), (1.0, 4.0), (-2.0, 4.0)],
            )
        ]
        drawing = DxfDrawing(Path("test.dxf"), None, [], ["TEST_VEHICLE"])
        geometry = DxfBlockGeometry(
            "TEST_VEHICLE", primitives, (-2.0, -1.0, 1.0, 4.0)
        )
        self.window._shared_block_cache["TEST_VEHICLE"] = (drawing, geometry)
        self.window.current_level_name = "Level 1"

    def tearDown(self) -> None:
        self.window.close()

    def test_drawn_vehicle_box_uses_oriented_block_bounds(self) -> None:
        pose = Pose(10.0, 20.0, 90.0)
        outline = self.window.block_outline_points(self.profile)

        expected_outline = [(-1.0, -1.0), (4.0, -1.0), (4.0, 2.0), (-1.0, 2.0)]
        for actual_point, expected_point in zip(outline, expected_outline):
            self.assertAlmostEqual(actual_point[0], expected_point[0])
            self.assertAlmostEqual(actual_point[1], expected_point[1])

        group = self.window.draw_vehicle(self.profile, pose, ghost=True)
        body = next(
            item for item in group.childItems() if isinstance(item, QGraphicsPolygonItem)
        )
        actual = [(point.x(), -point.y()) for point in body.polygon()]
        expected = [pose.transformed_point(x, y) for x, y in outline]
        for actual_point, expected_point in zip(actual, expected):
            self.assertAlmostEqual(actual_point[0], expected_point[0])
            self.assertAlmostEqual(actual_point[1], expected_point[1])

    def test_clearance_check_uses_block_box_instead_of_profile_dimensions(self) -> None:
        poses = [Pose(0.0, 0.0, 0.0), Pose(0.1, 0.0, 0.0)]
        self.window.obstacles = [
            Obstacle(
                "Wall",
                "Level 1",
                "wall",
                -70.0,
                -10.0,
                20.0,
                1.0,
                end_x=-70.0,
                end_y=10.0,
            )
        ]

        self.assertEqual(
            [False],
            self.window._route_obstacle_collision_flags(poses, self.profile),
        )

        self.window.obstacles[0].x = 3.0
        self.window.obstacles[0].end_x = 3.0
        self.assertEqual(
            [True],
            self.window._route_obstacle_collision_flags(poses, self.profile),
        )

    def test_zero_radius_auto_route_uses_point_turns_at_corners(self) -> None:
        profile = VehicleProfile(
            steering_mode=SteeringMode.OMNI,
            min_turning_radius=0.0,
            wheels=[WheelSpec("Drive steer", 0.0, 0.0, 0.2, steerable=True, drive=True)],
        )

        point_turns, reverses, modes = self.window._auto_route_leg_maneuvers(
            [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)], profile
        )

        self.assertEqual({0}, point_turns)
        self.assertEqual(set(), reverses)
        self.assertEqual("turn", modes[0])

    def test_minimum_radius_segment_preserves_dropoff_marker(self) -> None:
        profile = VehicleProfile(min_turning_radius=5.0)
        self.window._load_profile_to_form(profile)

        poses = self.window._planned_route_poses_for(
            Pose(20.0, 0.0, 0.0),
            [],
            start_pose=Pose(0.0, 0.0, 0.0),
            dropoff_pose=Pose(10.0, 0.0, 0.0),
            point_path_modes={0: "minimum_radius"},
            dropoff_insert_index=0,
        )

        self.assertTrue(any(pose.maneuver == "dropoff" for pose in poses))

    def test_auto_router_can_reuse_and_adapt_a_feasible_saved_example(self) -> None:
        profile = VehicleProfile(
            steering_mode=SteeringMode.OMNI,
            min_turning_radius=0.0,
        )
        self.window._load_profile_to_form(profile)
        self.window.current_level_name = "Level 1"
        self.window.current_start_name = "Start 1"
        self.window.start_pose = Pose(0.0, 0.0, 0.0)
        self.window.end_pose = Pose(20.0, 0.0, 0.0)
        self.window.dropoff_pose = Pose(10.0, 2.0, 0.0)
        self.window.obstacles = []
        self.window.saved_routes = [
            RoutePlan(
                "Working example",
                Pose(20.0, 0.0, 0.0),
                [(5.0, 0.0), (15.0, 0.0)],
                [],
                [],
                "Level 1",
                "Start 1",
                Pose(0.0, 0.0, 0.0),
                dropoff_pose=Pose(10.0, 0.0, 0.0),
                dropoff_waypoint_index=1,
            )
        ]

        result = self.window._auto_route_example_candidate(profile)

        self.assertIsNotNone(result)
        candidate, source = result
        self.assertEqual("Working example", source)
        self.assertEqual(1, candidate.dropoff_waypoint_index)
        self.assertIn((5.0, 2.0), candidate.waypoints)
        self.assertTrue(self.window._auto_route_candidate_is_feasible(candidate, profile))

    def test_auto_router_can_return_an_infeasible_saved_example_as_a_suggestion(self) -> None:
        profile = VehicleProfile(
            steering_mode=SteeringMode.OMNI,
            min_turning_radius=0.0,
        )
        self.window._load_profile_to_form(profile)
        self.window.current_start_name = "Start 1"
        self.window.start_pose = Pose(0.0, 0.0, 0.0)
        self.window.end_pose = Pose(20.0, 0.0, 0.0)
        self.window.obstacles = [
            Obstacle(
                "Blocking wall",
                "Level 1",
                "wall",
                10.0,
                -10.0,
                0.1,
                20.0,
            )
        ]
        self.window.saved_routes = [
            RoutePlan(
                "Useful example",
                Pose(20.0, 0.0, 0.0),
                [(10.0, 0.0)],
                [],
                [],
                "Level 1",
                "Start 1",
                Pose(0.0, 0.0, 0.0),
            )
        ]

        self.assertIsNone(self.window._auto_route_example_candidate(profile))
        suggestion = self.window._auto_route_example_candidate(
            profile, allow_infeasible=True
        )

        self.assertIsNotNone(suggestion)
        candidate, source = suggestion
        self.assertEqual("Useful example", source)
        self.assertFalse(self.window._auto_route_candidate_is_feasible(candidate, profile))

    def test_auto_router_applies_a_direct_suggestion_when_search_cannot_reach_finish(self) -> None:
        profile = VehicleProfile(
            name="Suggestion vehicle",
            length=2.0,
            width=1.0,
            steering_mode=SteeringMode.OMNI,
            min_turning_radius=0.0,
        )
        self.window._load_profile_to_form(profile)
        self.window.current_start_name = "Start 1"
        self.window.start_pose = Pose(0.0, 0.0, 0.0)
        self.window.end_pose = Pose(20.0, 0.0, 0.0)
        self.window.saved_routes = []
        self.window.route_waypoints = [(99.0, 99.0)]

        with (
            patch.object(self.window, "_obstacle_path", return_value=None),
            patch("vehicle_tracking.app.QMessageBox.warning") as warning,
        ):
            self.window.auto_route_current_path()

        self.assertEqual([(10.0, 0.0)], self.window.route_waypoints)
        self.assertEqual("travel", self.window.route_start_operation)
        warning.assert_called_once()
        self.assertEqual("Best route suggestion applied", warning.call_args.args[1])
        self.assertIn(
            "best-effort suggestion", self.window.statusBar().currentMessage()
        )

    def test_auto_router_builds_point_turn_locked_crab_then_forward_finish(self) -> None:
        profile = VehicleProfile(
            name="Crab vehicle",
            length=2.0,
            width=1.0,
            wheelbase=1.0,
            steering_mode=SteeringMode.CRAB,
            max_steering_angle_deg=90.0,
            min_turning_radius=0.0,
        )
        self.window._load_profile_to_form(profile)
        self.window.current_start_name = "Start 1"
        self.window.start_pose = Pose(0.0, 10.0, -90.0)
        self.window.end_pose = Pose(20.0, 0.0, 0.0)
        self.window.obstacles = []
        base = RoutePlan(
            "Base corridor",
            self.window.end_pose,
            [(0.0, 5.0)],
            level_name="Level 1",
            start_position_name="Start 1",
            start_pose=self.window.start_pose,
        )

        result = self.window._auto_route_locked_crab_finish_candidate(base, profile)

        self.assertIsNotNone(result)
        candidate, issues = result
        self.assertEqual([], issues)
        turn_index = candidate.point_turn_indices[-1]
        crab_mode = candidate.point_path_modes[turn_index + 1]
        self.assertEqual("crab:0.000000:0.000000", crab_mode)
        self.assertEqual("straight", candidate.point_path_modes[len(candidate.waypoints)])
        poses = self.window._auto_route_candidate_poses(candidate)
        self.assertTrue(any(pose.maneuver == "point_turn" for pose in poses))
        self.assertTrue(any(pose.maneuver == "crab" for pose in poses))
        self.assertEqual("", poses[-1].maneuver)

    def test_crab_turn_locks_heading_and_aligns_next_point_to_selected_axis(self) -> None:
        profile = VehicleProfile(
            name="Axis crab vehicle",
            length=2.0,
            width=1.0,
            wheelbase=1.0,
            steering_mode=SteeringMode.CRAB,
            max_steering_angle_deg=90.0,
            min_turning_radius=0.0,
        )
        self.window._load_profile_to_form(profile)
        self.window.start_pose = Pose(0.0, 5.0, 0.0)
        self.window.end_pose = Pose(20.0, 5.0, 0.0)
        self.window.route_waypoints = [(5.0, 5.0), (12.0, 9.0)]

        changed = self.window._set_route_crab_turn_axis(0, "x", 30.0)

        self.assertTrue(changed)
        self.assertEqual((12.0, 5.0), self.window.route_waypoints[1])
        self.assertIn(0, self.window.route_point_turns)
        self.assertEqual("crab:30:30:x", self.window.route_point_path_modes[1])
        operations = self.window._current_ordered_operations()
        self.assertEqual("crab_turn_x", operations[1].operation)
        poses = self.window.planned_route_poses(profile)
        crab_poses = [pose for pose in poses if pose.maneuver == "crab_x"]
        self.assertTrue(crab_poses)
        self.assertTrue(
            all(abs(pose.y - crab_poses[0].y) < 1e-9 for pose in crab_poses)
        )
        self.assertTrue(all(abs(pose.heading_deg - 30.0) < 1e-9 for pose in crab_poses))


if __name__ == "__main__":
    unittest.main()
