from pathlib import Path
import unittest
from uuid import uuid4

import ezdxf

from vehicle_tracking.dxf_io import export_tracking_dxf
from vehicle_tracking.models import Pose, VehicleProfile


class DxfGroupTests(unittest.TestCase):
    def test_each_path_exports_as_a_complete_named_group(self) -> None:
        output = Path.cwd() / f".test_grouped_paths_{uuid4().hex}.dxf"
        profile = VehicleProfile(payload_enabled=True)
        driven = [Pose(0.0, 0.0, 0.0), Pose(1.0, 0.0, 0.0)]
        routes = [
            [Pose(0.0, 0.0, 0.0), Pose(5.0, 0.0, 0.0, maneuver="reverse")],
            [Pose(0.0, 0.0, 0.0), Pose(0.0, 5.0, 90.0)],
        ]
        try:
            export_tracking_dxf(
                None,
                output,
                profile,
                driven,
                planned_routes=routes,
                planned_route_names=["Loading Bay A", "Loading/Bay A"],
            )
            document = ezdxf.readfile(output)
            groups = {name: list(group) for name, group in document.groups}

            self.assertEqual(
                set(groups),
                {
                    "VT_PATH_001_Loading_Bay_A",
                    "VT_PATH_002_Loading_Bay_A",
                    "VT_DRIVEN_PATH",
                },
            )
            for name in ("VT_PATH_001_Loading_Bay_A", "VT_PATH_002_Loading_Bay_A"):
                layers = {entity.dxf.layer for entity in groups[name]}
                self.assertIn("VT_PLANNED_ROUTE", layers)
                self.assertIn("VT_PLANNED_SWEEP", layers)
                self.assertIn("VT_FINISH_VEHICLE", layers)
                self.assertIn("VT_FINISH_PAYLOAD", layers)
            self.assertIn(
                "VT_ROUTE_ACTIONS",
                {entity.dxf.layer for entity in groups["VT_PATH_001_Loading_Bay_A"]},
            )
            driven_layers = {entity.dxf.layer for entity in groups["VT_DRIVEN_PATH"]}
            self.assertIn("VT_PATH", driven_layers)
            self.assertIn("VT_VEHICLE_POSES", driven_layers)
        finally:
            output.unlink(missing_ok=True)


if __name__ == "__main__":
    unittest.main()
