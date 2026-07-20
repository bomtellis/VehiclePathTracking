import math
import unittest

from vehicle_tracking.models import Pose, RoutePlan, SteeringMode, VehicleProfile, step_pose
from vehicle_tracking.app import VehicleTrackerWindow


class CrabSteeringTests(unittest.TestCase):
    def setUp(self) -> None:
        self.profile = VehicleProfile(
            name="Crab carrier",
            steering_mode=SteeringMode.CRAB,
            max_steering_angle_deg=90.0,
            min_turning_radius=0.0,
        )

    def test_default_crab_running_gear_steers_all_wheels(self) -> None:
        self.assertEqual(4, len(self.profile.wheels))
        self.assertTrue(all(wheel.steerable for wheel in self.profile.wheels))

    def test_crab_profile_uses_normal_4ws_radius_between_crab_legs(self) -> None:
        expected = self.profile.wheelbase / (
            2.0 * math.tan(math.radians(89.0))
        )
        self.assertAlmostEqual(expected, self.profile.calculated_min_turning_radius)
        self.assertIn("selected crab legs: R = infinite", self.profile.turning_radius_calculation)
        self.assertTrue(self.profile.supports_crab_movement)
        self.assertFalse(self.profile.supports_point_turn)

    def test_ninety_degree_crab_moves_sideways_without_yaw(self) -> None:
        result = step_pose(Pose(10.0, 20.0, 0.0), self.profile, 90.0, 5.0)
        self.assertAlmostEqual(10.0, result.x, places=9)
        self.assertAlmostEqual(25.0, result.y, places=9)
        self.assertEqual(0.0, result.heading_deg)
        self.assertEqual(90.0, result.steering_deg)
        self.assertEqual("crab", result.maneuver)

    def test_diagonal_crab_respects_existing_chassis_heading(self) -> None:
        result = step_pose(Pose(0.0, 0.0, 30.0), self.profile, -30.0, 4.0)
        self.assertAlmostEqual(4.0, result.x, places=9)
        self.assertAlmostEqual(0.0, result.y, places=9)
        self.assertEqual(30.0, result.heading_deg)

    def test_crab_mode_round_trips_through_profile_json_data(self) -> None:
        restored = VehicleProfile.from_dict(self.profile.to_dict())
        self.assertEqual(SteeringMode.CRAB, restored.steering_mode)

    def test_crab_section_round_trips_through_route_data(self) -> None:
        route = RoutePlan("Side entry", Pose(0.0, 5.0, 0.0))
        route.point_path_modes = {0: "crab:15:30"}
        restored = RoutePlan.from_dict(route.to_dict())
        self.assertEqual({0: "crab:15:30"}, restored.point_path_modes)

    def test_sideways_crab_leg_with_fixed_heading_is_feasible(self) -> None:
        route = [
            Pose(0.0, 0.0, 0.0, 90.0, "crab"),
            Pose(0.0, 5.0, 0.0, 90.0, "crab"),
        ]
        invalid, _required, _unsupported = VehicleTrackerWindow._route_section_analysis(
            route, self.profile
        )
        self.assertEqual([False], invalid)

    def test_planner_applies_defined_headings_to_selected_final_section(self) -> None:
        class PlannerHarness:
            _planned_route_poses_for = VehicleTrackerWindow._planned_route_poses_for
            _fillet_connected_straights = staticmethod(
                VehicleTrackerWindow._fillet_connected_straights
            )
            _is_fillet_mode = staticmethod(VehicleTrackerWindow._is_fillet_mode)
            _is_crab_mode = staticmethod(VehicleTrackerWindow._is_crab_mode)
            _crab_headings_from_mode = staticmethod(
                VehicleTrackerWindow._crab_headings_from_mode
            )
            _fillet_radius_from_mode = staticmethod(
                VehicleTrackerWindow._fillet_radius_from_mode
            )
            _minimum_radius_arc_poses = staticmethod(
                VehicleTrackerWindow._minimum_radius_arc_poses
            )

            def __init__(self, profile: VehicleProfile) -> None:
                self.start_pose = Pose(0.0, 0.0, 0.0)
                self.profile = profile

            def form_profile(self) -> VehicleProfile:
                return self.profile

        planner = PlannerHarness(self.profile)
        route = planner._planned_route_poses_for(
            Pose(0.0, 5.0, 0.0),
            [],
            point_path_modes={0: "crab:10:25"},
        )
        self.assertTrue(route)
        self.assertTrue(all(pose.maneuver == "crab" for pose in route))
        self.assertAlmostEqual(10.0, route[0].heading_deg)
        self.assertAlmostEqual(25.0, route[-1].heading_deg)
        self.assertTrue(
            all(
                first.heading_deg <= second.heading_deg
                for first, second in zip(route, route[1:])
            )
        )

    def test_planned_crab_heading_change_is_rejected(self) -> None:
        restricted = VehicleProfile(
            steering_mode=SteeringMode.CRAB,
            max_steering_angle_deg=10.0,
            min_turning_radius=0.0,
        )
        route = [
            Pose(0.0, 0.0, 0.0, 0.0, "crab"),
            Pose(0.1, 0.0, 90.0, 0.0, "crab"),
        ]
        invalid, _required, _unsupported = VehicleTrackerWindow._route_section_analysis(
            route, restricted
        )
        self.assertTrue(any(invalid))


if __name__ == "__main__":
    unittest.main()
