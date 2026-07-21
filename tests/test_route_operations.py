import unittest

from vehicle_tracking.models import Pose, RouteOperation, RoutePlan


class RouteOperationTests(unittest.TestCase):
    def test_reverse_then_turn_round_trips(self) -> None:
        route = RoutePlan(
            "Combined maneuver",
            Pose(10.0, 10.0, 90.0),
            waypoints=[(5.0, 0.0)],
            reversing_action_indices=[0],
            point_path_modes={0: "reverse_then_turn"},
        )

        restored = RoutePlan.from_dict(route.to_dict())

        self.assertEqual([0], restored.reversing_action_indices)
        self.assertEqual("reverse_then_turn", restored.point_path_modes[0])
        self.assertEqual(
            "reverse_then_turn",
            restored.ordered_operations()[1].operation,
        )

    def test_operation_only_data_restores_combined_maneuver(self) -> None:
        data = RoutePlan(
            "Operations",
            Pose(10.0, 0.0, 0.0),
            waypoints=[(5.0, 0.0)],
        ).to_dict()
        data.pop("reversing_action_indices")
        data.pop("point_path_modes")
        data["operations"] = [
            RouteOperation("start", "travel").to_dict(),
            RouteOperation("waypoint", "reverse_then_turn", 0).to_dict(),
            RouteOperation("end", "stop").to_dict(),
        ]

        restored = RoutePlan.from_dict(data)

        self.assertEqual([0], restored.reversing_action_indices)
        self.assertEqual({0: "reverse_then_turn"}, restored.point_path_modes)

    def test_continue_reversing_points_round_trip_and_legacy_global_migrates(self) -> None:
        route = RoutePlan(
            "Single reverse leg",
            Pose(10.0, 0.0, 0.0),
            waypoints=[(5.0, 0.0)],
            reversing_action_indices=[0],
            continue_reversing_indices=[],
        )

        restored = RoutePlan.from_dict(route.to_dict())
        legacy = RoutePlan.from_dict(
            {
                "name": "Legacy",
                "end_pose": {},
                "waypoints": [[5.0, 0.0]],
                "reversing_action_indices": [0],
                "continue_reversing": True,
            }
        )

        self.assertEqual([], restored.continue_reversing_indices)
        self.assertEqual([0], legacy.continue_reversing_indices)


if __name__ == "__main__":
    unittest.main()
