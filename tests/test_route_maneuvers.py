import os
import unittest
from math import hypot
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QPointF

from vehicle_tracking.app import VehicleTrackerWindow
from vehicle_tracking.models import Pose, RouteOperation, RoutePlan, SteeringMode, VehicleProfile, WheelSpec, step_pose


class RouteManeuverTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self) -> None:
        self.window = VehicleTrackerWindow()
        self.window.start_pose = Pose(0.0, 0.0, 0.0)

    def tearDown(self) -> None:
        self.window.close()

    def test_minimum_turning_radius_is_calculated_per_steering_method(self) -> None:
        expected = {
            SteeringMode.ACKERMANN_FRONT: 2.0,
            SteeringMode.ACKERMANN_REAR: 2.0,
            SteeringMode.FOUR_WHEEL: 1.0,
            SteeringMode.DIFFERENTIAL: 0.0,
            SteeringMode.OMNI: 0.0,
        }
        for mode, radius in expected.items():
            with self.subTest(mode=mode):
                profile = VehicleProfile(
                    steering_mode=mode,
                    wheelbase=2.0,
                    max_steering_angle_deg=45.0,
                    min_turning_radius=0.0,
                )
                self.assertAlmostEqual(profile.calculated_min_turning_radius, radius, places=6)

    def test_configured_radius_is_a_safety_floor_for_planner_curvature(self) -> None:
        profile = VehicleProfile(
            steering_mode=SteeringMode.ACKERMANN_FRONT,
            wheelbase=2.0,
            max_steering_angle_deg=45.0,
            min_turning_radius=3.0,
        )
        self.assertAlmostEqual(profile.effective_min_turning_radius, 3.0)
        self.assertAlmostEqual(profile.max_turn_curvature, 1.0 / 3.0)

    def test_four_wheel_motion_uses_the_same_two_axle_formula(self) -> None:
        profile = VehicleProfile(
            steering_mode=SteeringMode.FOUR_WHEEL,
            wheelbase=2.0,
            max_steering_angle_deg=45.0,
            min_turning_radius=0.0,
        )
        pose = step_pose(Pose(0.0, 0.0, 0.0), profile, 45.0, 1.0)
        self.assertAlmostEqual(pose.heading_deg, 180.0 / 3.141592653589793, places=5)

    def test_ui_can_apply_the_calculated_radius(self) -> None:
        index = self.window.steering_mode_combo.findData(SteeringMode.FOUR_WHEEL.value)
        self.window.steering_mode_combo.setCurrentIndex(index)
        self.window.wheelbase_spin.setValue(2.0)
        self.window.max_steer_spin.setValue(45.0)
        self.window.min_radius_spin.setValue(9.0)
        self.window.apply_calculated_turning_radius()
        self.assertAlmostEqual(self.window.min_radius_spin.value(), 1.0)
        self.assertIn("Four-wheel steer", self.window.calculated_radius_label.text())

    def test_dropoff_point_is_followed_by_reverse_exit_to_final_position(self) -> None:
        dropoff = Pose(10.0, 0.0, 0.0)
        poses = self.window._planned_route_poses_for(
            Pose(5.0, 0.0, 0.0),
            [],
            start_pose=Pose(0.0, 0.0, 0.0),
            dropoff_pose=dropoff,
        )
        marker = next(index for index, pose in enumerate(poses) if pose.maneuver == "dropoff")
        self.assertAlmostEqual(poses[marker].x, 10.0)
        self.assertTrue(all(pose.maneuver == "reverse" for pose in poses[marker + 1 :]))
        self.assertAlmostEqual(poses[-1].x, 5.0)

    def test_placing_dropoff_adds_ordered_drop_and_reverse_operations(self) -> None:
        self.window.end_pose = Pose(5.0, 0.0, 0.0)
        self.window.place_position("dropoff", QPointF(10.0, 0.0), 0.0)
        self.assertIsNotNone(self.window.dropoff_pose)
        self.assertEqual(
            [
                (item.location, item.operation)
                for item in self.window._current_ordered_operations()
            ],
            [
                ("start", "travel"),
                ("dropoff", "dropoff"),
                ("dropoff", "reverse"),
                ("end", "stop"),
            ],
        )
        self.assertEqual(self.window.route_operations_table.rowCount(), 4)

    def test_point_can_be_added_to_reverse_out_section(self) -> None:
        self.window.end_pose = Pose(5.0, 0.0, 0.0)
        self.window.place_position("dropoff", QPointF(10.0, 0.0), 0.0)

        # The approach and reverse exit overlap here; prefer the reverse section.
        self.window.place_position("route", QPointF(7.5, 0.0), 0.0)

        self.assertEqual(self.window.route_dropoff_waypoint_index, 0)
        self.assertEqual(len(self.window.route_waypoints), 1)
        self.assertEqual(
            [(item.location, item.waypoint_index) for item in self.window._current_ordered_operations()],
            [
                ("start", None),
                ("dropoff", None),
                ("dropoff", None),
                ("waypoint", 0),
                ("end", None),
            ],
        )
        poses = self.window.planned_route_poses(self.window.form_profile())
        dropoff_index = next(
            index for index, pose in enumerate(poses) if pose.maneuver == "dropoff"
        )
        self.assertTrue(any(pose.maneuver == "reverse" for pose in poses[dropoff_index + 1 :]))

        plan = RoutePlan(
            "reverse exit point",
            self.window.end_pose,
            list(self.window.route_waypoints),
            dropoff_pose=self.window.dropoff_pose,
            dropoff_waypoint_index=self.window.route_dropoff_waypoint_index,
        )
        loaded = RoutePlan.from_dict(plan.to_dict())
        self.assertEqual(loaded.dropoff_waypoint_index, 0)
        self.assertEqual(
            [item.location for item in loaded.ordered_operations()],
            ["start", "dropoff", "dropoff", "waypoint", "end"],
        )

    def test_alignment_suggestions_create_delivery_and_egress_points(self) -> None:
        self.window.length_spin.setValue(4.0)
        self.window.wheelbase_spin.setValue(2.0)
        self.window.min_radius_spin.setValue(3.0)
        self.window.payload_length_spin.setValue(2.0)
        self.window.start_pose = Pose(10.0, 0.0, 90.0)
        self.window.end_pose = Pose(10.0, -20.0, 90.0)
        self.window.place_position("dropoff", QPointF(10.0, -10.0), 90.0)
        self.assertTrue(self.window.suggest_alignment_button.isEnabled())
        reverse_only = self.window.alignment_strategy_combo.findData("reverse_to_final")
        self.window.alignment_strategy_combo.setCurrentIndex(reverse_only)

        self.window.create_alignment_point_suggestions()

        self.assertEqual(len(self.window.route_waypoints), 2)
        self.assertEqual(self.window.route_dropoff_waypoint_index, 1)
        approach, egress = self.window.route_waypoints
        self.assertAlmostEqual(approach[0], 10.0, places=6)
        self.assertAlmostEqual(egress[0], 10.0, places=6)
        self.assertLess(egress[1], approach[1])
        self.assertEqual(self.window.route_point_path_modes, {0: "straight", 1: "straight"})
        self.assertEqual(
            [item.location for item in self.window._current_ordered_operations()],
            ["start", "waypoint", "dropoff", "dropoff", "waypoint", "end"],
        )

        # Re-running the suggestion updates the same boundary points.
        self.window.create_alignment_point_suggestions()
        self.assertEqual(len(self.window.route_waypoints), 2)
        poses = self.window.planned_route_poses(self.window.form_profile())
        marker = next(index for index, pose in enumerate(poses) if pose.maneuver == "dropoff")
        self.assertTrue(all(pose.maneuver == "reverse" for pose in poses[marker + 1 :]))

        strategy = self.window.alignment_strategy_combo.findData("resume_forward")
        self.window.alignment_strategy_combo.setCurrentIndex(strategy)
        self.window.end_pose = Pose(10.0, 8.0, 90.0)
        self.window.create_alignment_point_suggestions()
        self.assertEqual(self.window.route_reversing_actions, {1})
        poses = self.window.planned_route_poses(self.window.form_profile())
        marker = next(index for index, pose in enumerate(poses) if pose.maneuver == "dropoff")
        self.assertTrue(any(pose.maneuver == "reverse" for pose in poses[marker + 1 :]))
        self.assertNotEqual(poses[-1].maneuver, "reverse")

    def test_dropoff_heading_field_controls_approach_and_persists(self) -> None:
        self.window.end_pose = Pose(5.0, 0.0, 0.0)
        self.window.place_position("dropoff", QPointF(10.0, 0.0), 15.0)
        self.assertAlmostEqual(self.window.dropoff_heading_spin.value(), 15.0)
        self.window.dropoff_heading_spin.setValue(30.0)
        self.assertAlmostEqual(self.window.dropoff_pose.heading_deg, 30.0)
        plan = RoutePlan(
            "headed drop",
            self.window.end_pose,
            dropoff_pose=self.window.dropoff_pose,
        )
        loaded = RoutePlan.from_dict(plan.to_dict())
        self.assertAlmostEqual(loaded.dropoff_pose.heading_deg, 30.0)

    def test_dropoff_does_not_require_a_full_vehicle_length_straight_approach(self) -> None:
        profile = VehicleProfile(
            length=4.0,
            payload_enabled=True,
            payload_length=2.0,
        )
        dropoff = Pose(1.0, 0.0, 0.0)
        plan = RoutePlan("short approach", Pose(0.0, 0.0, 0.0), dropoff_pose=dropoff)
        poses = [
            Pose(0.75, 0.0, 0.0),
            Pose(1.0, 0.0, 0.0, maneuver="dropoff"),
            Pose(0.0, 0.0, 0.0, maneuver="reverse"),
        ]

        result = self.window._payload_dropoff_analysis(plan, profile, poses)

        self.assertTrue(result.possible, result.message)
        self.assertAlmostEqual(result.straight_approach_distance, 0.25)
        self.assertEqual(result.required_straight_distance, 0.0)

        poses[1].heading_deg = 5.0
        result = self.window._payload_dropoff_analysis(plan, profile, poses)
        self.assertFalse(result.possible)

    def test_point_can_be_configured_as_straight_or_curved_turn(self) -> None:
        end = Pose(10.0, 5.0, 45.0)
        waypoint = [(5.0, 0.0)]
        straight = self.window._planned_route_poses_for(
            end,
            waypoint,
            start_pose=Pose(0.0, 0.0, 0.0),
            point_path_modes={0: "straight"},
        )
        curved = self.window._planned_route_poses_for(
            end,
            waypoint,
            start_pose=Pose(0.0, 0.0, 0.0),
            point_path_modes={0: "turn"},
        )
        self.assertTrue(all(abs(pose.y) < 1e-9 for pose in straight[:41]))
        self.assertTrue(any(abs(pose.y) > 1e-4 for pose in curved[1:40]))
        plan = RoutePlan("modes", end, waypoint, point_path_modes={0: "straight"})
        loaded = RoutePlan.from_dict(plan.to_dict())
        self.assertEqual(loaded.point_path_modes, {0: "straight"})

    def test_route_drawing_polar_snap_projects_to_nearest_angle(self) -> None:
        horizontal = self.window.view._polar_snap_point(
            QPointF(0.0, 0.0), QPointF(10.0, 1.0)
        )
        diagonal = self.window.view._polar_snap_point(
            QPointF(2.0, 3.0), QPointF(7.1, 8.0)
        )
        self.assertAlmostEqual(horizontal.x(), 10.0, places=6)
        self.assertAlmostEqual(horizontal.y(), 0.0, places=6)
        self.assertAlmostEqual(diagonal.x() - 2.0, diagonal.y() - 3.0, places=6)

    def test_perpendicular_drawn_lines_are_joined_by_a_radius_arc(self) -> None:
        nodes, modes = self.window._connect_straight_sketch(
            [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)], 2.0
        )
        self.assertEqual(modes, {0: "straight", 1: "minimum_radius", 2: "straight"})
        expected = [(0.0, 0.0), (8.0, 0.0), (10.0, 2.0), (10.0, 10.0)]
        for actual, wanted in zip(nodes, expected):
            self.assertAlmostEqual(actual[0], wanted[0], places=6)
            self.assertAlmostEqual(actual[1], wanted[1], places=6)

        poses = self.window._minimum_radius_arc_poses(
            nodes[1], nodes[2], (8.0, 0.0), (0.0, 8.0), 2.0, 1
        )
        center = (8.0, 2.0)
        for pose in poses:
            self.assertAlmostEqual(hypot(pose.x - center[0], pose.y - center[1]), 2.0, places=5)

    def test_existing_connected_straight_modes_are_filleted(self) -> None:
        nodes, modes, index_map = self.window._fillet_connected_straights(
            [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)],
            {0: "straight", 1: "straight"},
            2.0,
            set(),
        )
        self.assertEqual(modes, {0: "straight", 1: "minimum_radius", 2: "straight"})
        self.assertEqual(index_map, {0: 1})
        self.assertAlmostEqual(nodes[1][0], 8.0)
        self.assertAlmostEqual(nodes[1][1], 0.0)
        self.assertAlmostEqual(nodes[2][0], 10.0)
        self.assertAlmostEqual(nodes[2][1], 2.0)

    def test_isolated_straights_are_extended_to_their_corner(self) -> None:
        intersection = self.window._infinite_line_intersection(
            ((0.0, 0.0), (8.0, 0.0)),
            ((10.0, 3.0), (10.0, 10.0)),
        )
        self.assertIsNotNone(intersection)
        self.assertAlmostEqual(intersection[0], 10.0)
        self.assertAlmostEqual(intersection[1], 0.0)
        self.assertIsNone(self.window._infinite_line_intersection(
            ((0.0, 0.0), (8.0, 0.0)),
            ((0.0, 2.0), (8.0, 2.0)),
        ))

    def test_drawing_pre_dropoff_preserves_post_dropoff_section(self) -> None:
        mode = self.window.steering_mode_combo.findData(SteeringMode.DIFFERENTIAL.value)
        self.window.steering_mode_combo.setCurrentIndex(mode)
        self.window.min_radius_spin.setValue(1.0)
        self.window.end_pose = Pose(10.0, 10.0, 90.0)
        self.window.dropoff_pose = Pose(5.0, 5.0, 90.0)
        self.window.route_waypoints = [(2.0, 0.0), (8.0, 5.0)]
        self.window.route_dropoff_waypoint_index = 1
        self.window.route_point_turns = {1}
        self.window.route_reversing_actions = {1}
        self.window.route_tangent_handles = {1: (2.0, 0.0)}
        self.window.route_point_path_modes = {0: "straight", 1: "turn"}

        self.window._replace_route_section_from_vertices(
            [(0.0, 0.0), (5.0, 0.0), (5.0, 5.0)], "pre"
        )

        self.assertEqual(self.window.route_waypoints[-1], (8.0, 5.0))
        self.assertEqual(self.window.route_dropoff_waypoint_index, 2)
        self.assertEqual(self.window.route_point_turns, {2})
        self.assertEqual(self.window.route_reversing_actions, {2})
        self.assertEqual(self.window.route_tangent_handles, {2: (2.0, 0.0)})
        self.assertEqual(self.window.route_point_path_modes[1], "minimum_radius")

    def test_drawing_post_dropoff_preserves_pre_dropoff_section(self) -> None:
        mode = self.window.steering_mode_combo.findData(SteeringMode.DIFFERENTIAL.value)
        self.window.steering_mode_combo.setCurrentIndex(mode)
        self.window.min_radius_spin.setValue(1.0)
        self.window.end_pose = Pose(10.0, 10.0, 90.0)
        self.window.dropoff_pose = Pose(5.0, 5.0, 0.0)
        self.window.route_waypoints = [(2.0, 0.0), (8.0, 5.0)]
        self.window.route_dropoff_waypoint_index = 1
        self.window.route_point_turns = {0}
        self.window.route_tangent_handles = {0: (2.0, 0.0)}
        self.window.route_point_path_modes = {0: "turn", 1: "straight"}

        self.window._replace_route_section_from_vertices(
            [(5.0, 5.0), (10.0, 5.0), (10.0, 10.0)], "post"
        )

        self.assertEqual(self.window.route_waypoints[0], (2.0, 0.0))
        self.assertEqual(self.window.route_dropoff_waypoint_index, 1)
        self.assertEqual(self.window.route_point_turns, {0})
        self.assertEqual(self.window.route_tangent_handles, {0: (2.0, 0.0)})
        self.assertEqual(self.window.route_point_path_modes[2], "minimum_radius")

    def test_route_operations_are_saved_in_traversal_order(self) -> None:
        plan = RoutePlan(
            "transfer",
            Pose(5.0, 0.0, 0.0),
            [(3.0, 0.0)],
            dropoff_pose=Pose(10.0, 0.0, 0.0),
            operations=[
                RouteOperation("start", "travel"),
                RouteOperation("waypoint", "point_turn", 0),
                RouteOperation("dropoff", "dropoff"),
                RouteOperation("dropoff", "reverse"),
                RouteOperation("end", "stop"),
            ],
        )
        loaded = RoutePlan.from_dict(plan.to_dict())
        self.assertEqual(
            [(item.location, item.operation) for item in loaded.ordered_operations()],
            [
                ("start", "travel"),
                ("waypoint", "point_turn"),
                ("dropoff", "dropoff"),
                ("dropoff", "reverse"),
                ("end", "stop"),
            ],
        )
        self.assertEqual(loaded.dropoff_pose.x, 10.0)

    def test_route_point_selection_is_synchronized_with_operations_list(self) -> None:
        self.window.end_pose = Pose(10.0, 0.0, 0.0)
        self.window.route_waypoints = [(3.0, 1.0), (6.0, 2.0)]
        self.window.redraw_route_handles()
        self.window._refresh_route_operations_table()

        self.window.route_operations_table.selectRow(1)
        self.app.processEvents()
        self.assertEqual(self.window._selected_route_point_index, 0)
        self.assertTrue(self.window.route_point_items[0].isSelected())

        self.window._select_route_point(1)
        self.assertEqual(self.window.route_operations_table.currentRow(), 2)
        self.assertTrue(self.window.route_point_items[1].isSelected())
        self.assertFalse(self.window.route_point_items[0].isSelected())

    def test_reordering_points_keeps_each_points_operation_and_handles(self) -> None:
        self.window.end_pose = Pose(12.0, 0.0, 0.0)
        self.window.route_waypoints = [(2.0, 0.0), (5.0, 1.0), (8.0, 2.0)]
        self.window.route_point_turns = {0}
        self.window.route_reversing_actions = {1}
        self.window.route_tangent_handles = {0: (1.0, 0.5), 2: (2.0, 0.75)}
        self.window.route_point_path_modes = {1: "straight", 2: "turn"}
        self.window.redraw_route_handles()
        self.window._refresh_route_operations_table()
        self.window._select_route_point(1)

        self.window.move_selected_route_point(-1)

        self.assertEqual(
            self.window.route_waypoints,
            [(5.0, 1.0), (2.0, 0.0), (8.0, 2.0)],
        )
        self.assertEqual(self.window._selected_route_point_index, 0)
        self.assertEqual(self.window.route_point_turns, {1})
        self.assertEqual(self.window.route_reversing_actions, {0})
        self.assertEqual(
            self.window.route_tangent_handles,
            {1: (1.0, 0.5), 2: (2.0, 0.75)},
        )
        self.assertEqual(self.window.route_point_path_modes, {0: "straight", 2: "turn"})
        self.assertEqual(self.window.route_operations_table.currentRow(), 1)

    def test_copy_path_creates_an_independent_editable_duplicate(self) -> None:
        original = RoutePlan(
            "Bay 1",
            Pose(10.0, 0.0, 180.0),
            [(4.0, 1.0), (7.0, 2.0)],
            reversing_action_indices=[1],
            start_pose=Pose(0.0, 0.0, 0.0),
            tangent_handles={0: (2.0, 0.5)},
            dropoff_pose=Pose(8.0, 2.0, 90.0),
            point_path_modes={0: "turn", 1: "straight"},
            dropoff_waypoint_index=1,
        )
        self.window.saved_routes = [original]
        self.window.active_route_index = 0
        self.window.end_pose = Pose(10.0, 0.0, 180.0)
        self.window.dropoff_pose = Pose(8.0, 2.0, 90.0)
        self.window.route_waypoints = [(4.0, 1.0), (7.0, 2.0)]
        self.window.route_reversing_actions = {1}
        self.window.route_tangent_handles = {0: (2.0, 0.5)}
        self.window.route_point_path_modes = {0: "turn", 1: "straight"}
        self.window.route_dropoff_waypoint_index = 1
        self.window.route_name_edit.setText("Bay 1")

        with patch.object(self.window, "_persist_routes"):
            self.window.copy_current_route()

        self.assertEqual(len(self.window.saved_routes), 2)
        copied = self.window.saved_routes[1]
        self.assertEqual(copied.name, "Bay 1 Copy")
        self.assertEqual(self.window.active_route_index, 1)
        self.assertEqual(copied.waypoints, original.waypoints)
        self.assertEqual(copied.dropoff_waypoint_index, 1)
        self.assertEqual(copied.reversing_action_indices, [1])
        self.window.route_waypoints[0] = (20.0, 20.0)
        self.assertEqual(original.waypoints[0], (4.0, 1.0))
        self.assertEqual(copied.waypoints[0], (4.0, 1.0))

    def test_adjacent_saved_path_dropoff_is_shown_with_heading(self) -> None:
        self.window.current_level_name = "Level 1"
        self.window.saved_routes = [
            RoutePlan(
                "Adjacent Bay",
                Pose(12.0, 4.0, 180.0),
                level_name="Level 1",
                dropoff_pose=Pose(10.0, 5.0, 90.0),
            ),
            RoutePlan(
                "Other Floor",
                Pose(20.0, 4.0, 180.0),
                level_name="Level 2",
                dropoff_pose=Pose(18.0, 5.0, 45.0),
            ),
        ]
        self.window.active_route_index = None

        self.window.redraw_position_markers()

        adjacent = [
            item
            for item in self.window.position_items
            if "Adjacent path drop-off" in item.toolTip()
        ]
        self.assertEqual(len(adjacent), 3)
        self.assertTrue(all("Adjacent Bay" in item.toolTip() for item in adjacent))
        self.assertTrue(all("90.0 deg" in item.toolTip() for item in adjacent))
        self.assertFalse(
            any("Other Floor" in item.toolTip() for item in self.window.position_items)
        )

    def test_payload_dropoff_footprints_are_shown_in_dxf_viewer(self) -> None:
        self.window.current_level_name = "Level 1"
        self.window.route_name_edit.setText("Active Bay")
        self.window.dropoff_pose = Pose(5.0, 2.0, 90.0)
        self.window.saved_routes = [
            RoutePlan(
                "Adjacent Bay",
                Pose(12.0, 4.0, 180.0),
                level_name="Level 1",
                dropoff_pose=Pose(10.0, 5.0, 45.0),
            )
        ]
        self.window.active_route_index = None

        self.window.redraw_position_markers()

        active = [
            item
            for item in self.window.position_items
            if "Active payload drop-off: Active Bay" in item.toolTip()
        ]
        saved = [
            item
            for item in self.window.position_items
            if "Saved payload drop-off: Adjacent Bay" in item.toolTip()
        ]
        self.assertEqual(len(active), 5)
        self.assertEqual(len(saved), 4)
        self.assertTrue(any("payload centre" in item.toolTip() for item in active))
        active_blocks = [
            item
            for item in self.window.position_items
            if item.data(0) == "active-dropoff-vehicle"
        ]
        saved_blocks = [
            item
            for item in self.window.position_items
            if item.data(0) == "saved-dropoff-vehicle"
        ]
        self.assertEqual(len(active_blocks), 1)
        self.assertEqual(len(saved_blocks), 1)
        self.assertIn("heading 90.0 deg", active_blocks[0].toolTip())
        self.assertIn("heading 45.0 deg", saved_blocks[0].toolTip())

    def test_pickup_matches_dropoff_and_requires_straight_inline_approach(self) -> None:
        profile = VehicleProfile(payload_enabled=True, payload_length=2.0, payload_width=1.0)
        dropoff_route = RoutePlan(
            "Drop route",
            Pose(5.0, 0.0, 0.0),
            level_name="Ground",
            start_pose=Pose(0.0, 0.0, 0.0),
            dropoff_pose=Pose(10.0, 0.0, 0.0),
        )
        pickup_route = RoutePlan(
            "Pickup route",
            Pose(10.0, 0.0, 0.0),
            level_name="Ground",
            start_pose=Pose(0.0, 0.0, 0.0),
            operations=[
                RouteOperation("start", "travel"),
                RouteOperation("end", "pickup"),
            ],
        )
        self.window.saved_routes = [dropoff_route]
        poses = self.window._planned_route_poses_for(
            pickup_route.end_pose, [], start_pose=pickup_route.start_pose
        )
        result = self.window._payload_pickup_analysis(pickup_route, profile, poses)
        self.assertTrue(result.possible, result.message)
        self.assertGreaterEqual(result.straight_approach_distance, profile.length)

        pickup_route.end_pose.heading_deg = 90.0
        misaligned_poses = self.window._planned_route_poses_for(
            pickup_route.end_pose, [], start_pose=pickup_route.start_pose
        )
        result = self.window._payload_pickup_analysis(
            pickup_route, profile, misaligned_poses
        )
        self.assertFalse(result.possible)
        self.assertIn("alignment", result.message)

    def test_point_turn_rotates_at_marked_waypoint(self) -> None:
        profile = VehicleProfile(
            steering_mode=SteeringMode.ACKERMANN_REAR,
            wheels=[WheelSpec("driven steer", 0.0, 0.0, steerable=True, drive=True)],
        )
        end = Pose(5.0, 5.0, 90.0)
        poses = self.window._planned_route_poses_for(end, [(5.0, 0.0)], {0})

        pivot_poses = [pose for pose in poses if pose.maneuver == "point_turn"]
        self.assertTrue(pivot_poses)
        self.assertTrue(all((pose.x, pose.y) == (5.0, 0.0) for pose in pivot_poses))
        invalid, _, unsupported = self.window._route_section_analysis(poses, profile)
        self.assertFalse(unsupported)
        self.assertFalse(any(invalid))

    def test_point_turn_requires_compatible_driven_wheels(self) -> None:
        profile = VehicleProfile(
            steering_mode=SteeringMode.ACKERMANN_FRONT,
            wheels=[WheelSpec("fixed drive", 0.0, 0.0, steerable=False, drive=True)],
        )
        poses = [Pose(0.0, 0.0, 0.0), Pose(0.0, 0.0, 10.0, 70.0, "point_turn")]

        invalid, _, unsupported = self.window._route_section_analysis(poses, profile)
        self.assertTrue(unsupported)
        self.assertTrue(any(invalid))

    def test_reverse_final_approach_can_resolve_alignment(self) -> None:
        profile = VehicleProfile()
        end = Pose(2.0, 0.0, -90.0)
        direct = self.window._planned_route_poses_for(end, [], set())
        realignment = self.window._reverse_alignment_candidate(end, [], set(), profile)

        self.assertTrue(any(self.window._route_section_analysis(direct, profile)[0]))
        self.assertIsNotNone(realignment)
        self.window.end_pose = end
        self.window.route_waypoints = []
        self.window.route_point_turns = set()
        self.window.draw_route_failures(direct, profile)
        self.assertIn("One reverse movement appears feasible", self.window.route_feasibility_label.text())

    def test_point_turn_setting_round_trips(self) -> None:
        plan = RoutePlan(
            "turn",
            Pose(5.0, 5.0, 90.0),
            [(5.0, 0.0)],
            point_turn_indices=[0],
            reversing_action_indices=[0],
            tangent_handles={0: (2.5, 1.0)},
        )
        loaded = RoutePlan.from_dict(plan.to_dict())
        self.assertEqual(loaded.point_turn_indices, [0])
        self.assertEqual(loaded.reversing_action_indices, [0])
        self.assertEqual(loaded.tangent_handles, {0: (2.5, 1.0)})

    def test_reversing_action_changes_direction_without_rotating_vehicle(self) -> None:
        profile = VehicleProfile()
        end = Pose(2.0, 0.0, 0.0)
        poses = self.window._planned_route_poses_for(end, [(5.0, 0.0)], set(), {0})

        forward = [pose for pose in poses if pose.maneuver != "reverse"]
        reverse = [pose for pose in poses if pose.maneuver == "reverse"]
        self.assertTrue(forward)
        self.assertTrue(reverse)
        self.assertAlmostEqual(forward[-1].heading_deg % 360.0, 0.0, places=5)
        self.assertAlmostEqual(reverse[0].heading_deg % 360.0, 0.0, places=5)
        self.assertLess(reverse[-1].x, reverse[0].x)
        self.assertFalse(any(self.window._route_section_analysis(poses, profile)[0]))

    def test_place_reverse_action_inserts_a_red_action_point(self) -> None:
        self.window.end_pose = Pose(10.0, 0.0, 0.0)
        self.window.place_position("reverse_action", QPointF(5.0, 0.0))

        self.assertEqual(len(self.window.route_waypoints), 1)
        self.assertEqual(self.window.route_reversing_actions, {0})
        self.assertTrue(self.window.route_point_items[0].reversing_action)

    def test_second_reversing_action_returns_to_forward_travel(self) -> None:
        poses = self.window._planned_route_poses_for(
            Pose(6.0, 0.0, 0.0),
            [(5.0, 0.0), (2.0, 0.0)],
            set(),
            {0, 1},
        )

        self.assertTrue(any(pose.maneuver == "reverse" for pose in poses))
        self.assertEqual(poses[-1].maneuver, "")
        self.assertAlmostEqual(poses[-1].heading_deg % 360.0, 0.0, places=5)


if __name__ == "__main__":
    unittest.main()
