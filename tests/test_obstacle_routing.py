from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from math import cos, radians, sin

from vehicle_tracking.app import VehicleTrackerWindow
from vehicle_tracking.models import (
    Obstacle,
    Pose,
    ProjectStore,
    StartPosition,
    VehicleProfile,
    VehicleTrackingProject,
    SteeringMode,
    WheelSpec,
)


class ObstacleRoutingTests(unittest.TestCase):
    def test_astar_routes_around_closed_obstacle(self) -> None:
        wall = Obstacle("Wall 1", "Level 1", "wall", 4.0, -2.0, 2.0, 4.0)

        path = VehicleTrackerWindow._obstacle_path(
            (0.0, 0.0),
            (10.0, 0.0),
            [wall],
            clearance=1.0,
            grid_hint=0.5,
        )

        self.assertIsNotNone(path)
        self.assertGreater(len(path), 2)
        self.assertTrue(any(abs(y) > 3.0 for _x, y in path[1:-1]))

    def test_open_door_does_not_block_direct_path(self) -> None:
        door = Obstacle(
            "Door 1",
            "Level 1",
            "door",
            4.0,
            -2.0,
            2.0,
            4.0,
            open=True,
        )

        path = VehicleTrackerWindow._obstacle_path(
            (0.0, 0.0),
            (10.0, 0.0),
            [door],
            clearance=1.0,
            grid_hint=0.5,
        )

        self.assertEqual([(0.0, 0.0), (10.0, 0.0)], path)

    def test_hosted_door_is_a_cutout_in_chained_wall(self) -> None:
        wall = Obstacle(
            "Wall Chain 1 / Segment 1",
            "Level 1",
            "wall",
            0.0,
            0.0,
            10.0,
            0.2,
            end_x=10.0,
            end_y=0.0,
            chain_name="Wall Chain 1",
        )
        door = Obstacle(
            "Door 1",
            "Level 1",
            "door",
            4.0,
            0.0,
            2.0,
            0.2,
            open=True,
            end_x=6.0,
            end_y=0.0,
            chain_name="Wall Chain 1",
            host_wall_name=wall.name,
        )

        open_path = VehicleTrackerWindow._obstacle_path(
            (5.0, -3.0),
            (5.0, 3.0),
            [wall, door],
            clearance=0.2,
            grid_hint=0.1,
        )
        door.open = False
        closed_path = VehicleTrackerWindow._obstacle_path(
            (5.0, -3.0),
            (5.0, 3.0),
            [wall, door],
            clearance=0.2,
            grid_hint=0.1,
        )

        self.assertEqual([(5.0, -3.0), (5.0, 3.0)], open_path)
        self.assertIsNotNone(closed_path)
        self.assertGreater(len(closed_path), 2)

    def test_project_round_trips_walls_and_door_state(self) -> None:
        project = VehicleTrackingProject(
            ["Level 1"],
            {},
            [StartPosition("Start 1", "Level 1", Pose(0.0, 0.0, 0.0))],
            [],
            [VehicleProfile()],
            "Level 1",
            "Start 1",
            obstacles=[
                Obstacle("Wall 1", "Level 1", "wall", 1.0, 2.0, 3.0, 4.0),
                Obstacle("Door 1", "Level 1", "door", 5.0, 6.0, 7.0, 8.0, True),
            ],
        )

        with TemporaryDirectory() as directory:
            path = Path(directory) / "obstacles.vtproject"
            ProjectStore.save(path, project)
            restored = ProjectStore.load(path)

        self.assertEqual(2, len(restored.obstacles))
        self.assertEqual("wall", restored.obstacles[0].kind)
        self.assertTrue(restored.obstacles[1].open)

    def test_chained_wall_and_host_relationship_round_trip(self) -> None:
        wall = Obstacle(
            "Wall Chain 1 / Segment 1",
            "Level 1",
            "wall",
            1.0,
            2.0,
            1000.0,
            100.0,
            end_x=1001.0,
            end_y=2.0,
            chain_name="Wall Chain 1",
        )
        door = Obstacle(
            "Door 1",
            "Level 1",
            "door",
            400.0,
            2.0,
            900.0,
            100.0,
            end_x=1300.0,
            end_y=2.0,
            chain_name="Wall Chain 1",
            host_wall_name=wall.name,
        )

        restored_wall = Obstacle.from_dict(wall.to_dict())
        restored_door = Obstacle.from_dict(door.to_dict())

        self.assertEqual("Wall Chain 1", restored_wall.chain_name)
        self.assertEqual(1001.0, restored_wall.end_x)
        self.assertEqual(wall.name, restored_door.host_wall_name)

    def test_moving_wall_moves_whole_chain_and_hosted_door(self) -> None:
        first = Obstacle(
            "Chain / 1", "Level 1", "wall", 0.0, 0.0, 10.0, 1.0,
            end_x=10.0, end_y=0.0, chain_name="Chain",
        )
        second = Obstacle(
            "Chain / 2", "Level 1", "wall", 10.0, 0.0, 10.0, 1.0,
            end_x=10.0, end_y=10.0, chain_name="Chain",
        )
        door = Obstacle(
            "Door", "Level 1", "door", 4.0, 0.0, 2.0, 1.0,
            end_x=6.0, end_y=0.0, chain_name="Chain", host_wall_name=first.name,
        )
        harness = SimpleNamespace(obstacles=[first, second, door])

        affected = VehicleTrackerWindow._translate_obstacle_model(
            harness, 0, 3.0, -2.0
        )

        self.assertEqual({0, 1, 2}, affected)
        self.assertEqual((3.0, -2.0, 13.0, -2.0), (first.x, first.y, first.end_x, first.end_y))
        self.assertEqual((13.0, -2.0), (second.x, second.y))
        self.assertEqual((7.0, -2.0, 9.0, -2.0), (door.x, door.y, door.end_x, door.end_y))

    def test_moving_hosted_door_stays_on_wall(self) -> None:
        wall = Obstacle(
            "Wall", "Level 1", "wall", 0.0, 0.0, 10.0, 1.0,
            end_x=10.0, end_y=0.0, chain_name="Chain",
        )
        door = Obstacle(
            "Door", "Level 1", "door", 4.0, 0.0, 2.0, 1.0,
            end_x=6.0, end_y=0.0, chain_name="Chain", host_wall_name=wall.name,
        )
        harness = SimpleNamespace(obstacles=[wall, door])

        VehicleTrackerWindow._translate_obstacle_model(harness, 1, 1.0, 5.0)

        self.assertEqual((5.0, 0.0, 7.0, 0.0), (door.x, door.y, door.end_x, door.end_y))

    def test_auto_route_classifies_sharp_turns_and_reversing_cusps(self) -> None:
        point_turn_profile = VehicleProfile(min_turning_radius=10.0)

        point_turns, reverses, modes = VehicleTrackerWindow._auto_route_leg_maneuvers(
            [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)],
            point_turn_profile,
        )
        _u_turns, u_reverses, u_modes = VehicleTrackerWindow._auto_route_leg_maneuvers(
            [(0.0, 0.0), (1.0, 0.0), (0.0, 0.0)],
            point_turn_profile,
        )
        crab_profile = VehicleProfile(
            steering_mode=SteeringMode.CRAB,
            min_turning_radius=10.0,
        )
        _crab_turns, crab_reverses, crab_modes = VehicleTrackerWindow._auto_route_leg_maneuvers(
            [(0.0, 0.0), (1.0, 0.0), (0.0, 0.0)],
            crab_profile,
        )

        self.assertEqual({0}, point_turns)
        self.assertEqual(set(), reverses)
        self.assertEqual("turn", modes[0])
        self.assertEqual({0}, u_reverses)
        self.assertEqual("reverse_then_turn", u_modes[0])
        self.assertEqual({0}, crab_reverses)
        self.assertEqual("turn", crab_modes[0])

    def test_auto_router_can_select_user_available_motion_actions(self) -> None:
        fixed_profile = VehicleProfile(
            min_turning_radius=10.0,
            wheels=[
                WheelSpec(
                    "Fixed",
                    0.0,
                    0.0,
                    0.1,
                    steerable=False,
                    drive=False,
                )
            ],
        )
        crab_profile = VehicleProfile(
            steering_mode=SteeringMode.CRAB,
            max_steering_angle_deg=90.0,
            min_turning_radius=10.0,
        )
        five_degrees = radians(5.0)

        _, _, line_modes = VehicleTrackerWindow._auto_route_leg_maneuvers(
            [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)], fixed_profile
        )
        _, _, straight_modes = VehicleTrackerWindow._auto_route_leg_maneuvers(
            [(0.0, 0.0), (1.0, 0.0), (1.0 + cos(five_degrees), sin(five_degrees))],
            fixed_profile,
        )
        _, _, radius_modes = VehicleTrackerWindow._auto_route_leg_maneuvers(
            [(0.0, 0.0), (100.0, 0.0), (100.0, 100.0)], fixed_profile
        )
        _, _, turn_modes = VehicleTrackerWindow._auto_route_leg_maneuvers(
            [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)], fixed_profile
        )
        _, _, crab_modes = VehicleTrackerWindow._auto_route_leg_maneuvers(
            [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)], crab_profile
        )

        self.assertEqual("line", line_modes[0])
        self.assertEqual("straight", straight_modes[0])
        self.assertEqual("minimum_radius", radius_modes[0])
        self.assertEqual("turn", turn_modes[0])
        self.assertTrue(crab_modes[0].startswith("crab:"))
        self.assertEqual(
            "reverse",
            VehicleTrackerWindow._auto_route_start_operation(
                [(0.0, 0.0), (-10.0, 0.0)], 0.0, fixed_profile
            ),
        )
        self.assertEqual(
            "travel",
            VehicleTrackerWindow._auto_route_start_operation(
                [(0.0, 0.0), (0.0, 10.0)], 0.0, crab_profile
            ),
        )


if __name__ == "__main__":
    unittest.main()
