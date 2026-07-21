import os
from pathlib import Path
import re
import unittest
from unittest.mock import patch
from uuid import uuid4

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import ezdxf
from PySide6.QtCore import QPointF, QRectF
from PySide6.QtGui import QColor, QImage, QPainter, QPalette
from PySide6.QtWidgets import QApplication

from vehicle_tracking.app import (
    CurveTangentHandleItem,
    FloorDxfManagerDialog,
    VehicleTrackerWindow,
)
from vehicle_tracking.models import (
    Pose,
    ProjectStore,
    RoutePlan,
    RouteStore,
    StartPosition,
    VehicleProfile,
    VehicleTrackingProject,
)
from vehicle_tracking.qtbootstrap import QtBootstrap
from vehicle_tracking.reports import RouteReportEntry, generate_route_report_pdf
from vehicle_tracking.dxf_io import DxfPrimitive
from vehicle_tracking.video_export import export_qimages_to_mp4


class RouteConfigurationAndExportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def test_levels_starts_and_route_start_round_trip(self) -> None:
        path = Path.cwd() / f".test_routes_{uuid4().hex}.json"
        ground_dxf = Path.cwd() / f".test_ground_{uuid4().hex}.dxf"
        level_two_dxf = Path.cwd() / f".test_level_two_{uuid4().hex}.dxf"
        store = RouteStore(path)
        starts = [
            StartPosition("Goods In", "Ground", Pose(1.0, 2.0, 10.0)),
            StartPosition("Lift Lobby", "Level 2", Pose(30.0, 40.0, 90.0)),
        ]
        route = RoutePlan(
            "L2 Store",
            Pose(50.0, 60.0, 180.0),
            level_name="Level 2",
            start_position_name="Lift Lobby",
            start_pose=Pose(30.0, 40.0, 90.0),
        )
        try:
            ezdxf.new().saveas(ground_dxf)
            ezdxf.new().saveas(level_two_dxf)
            store.save_configuration(
                None,
                ["Ground", "Level 2"],
                starts,
                [route],
                {"Ground": ground_dxf, "Level 2": level_two_dxf},
            )
            levels, loaded_starts, routes = store.load_configuration(None)
            self.assertEqual(levels, ["Ground", "Level 2"])
            self.assertEqual([start.name for start in loaded_starts], ["Goods In", "Lift Lobby"])
            self.assertEqual(routes[0].level_name, "Level 2")
            self.assertEqual(routes[0].start_position_name, "Lift Lobby")
            self.assertEqual((routes[0].start_pose.x, routes[0].start_pose.y), (30.0, 40.0))
            drawings = store.load_level_drawings(None)
            self.assertEqual(drawings["Ground"], ground_dxf.resolve())
            self.assertEqual(drawings["Level 2"], level_two_dxf.resolve())
        finally:
            path.unlink(missing_ok=True)
            ground_dxf.unlink(missing_ok=True)
            level_two_dxf.unlink(missing_ok=True)

    def test_floor_dxf_dialog_adds_updates_and_removes_assignments(self) -> None:
        ground_dxf = Path.cwd() / f".test_ground_{uuid4().hex}.dxf"
        upper_dxf = Path.cwd() / f".test_upper_{uuid4().hex}.dxf"
        ezdxf.new().saveas(ground_dxf)
        ezdxf.new().saveas(upper_dxf)
        dialog = FloorDxfManagerDialog(["Ground"], {"Ground": ground_dxf})
        try:
            dialog.add_floor()
            dialog.table.item(1, 0).setText("Upper")
            dialog.table.item(1, 1).setText(str(upper_dxf))
            dialog._refresh_row(1)
            levels, drawings = dialog.configuration()
            self.assertEqual(levels, ["Ground", "Upper"])
            self.assertEqual(drawings["Upper"], upper_dxf)
            dialog.table.selectRow(0)
            dialog.remove_floor()
            self.assertEqual(dialog.configuration()[0], ["Upper"])
        finally:
            dialog.close()
            ground_dxf.unlink(missing_ok=True)
            upper_dxf.unlink(missing_ok=True)

    def test_floor_dialog_assignment_does_not_parse_dxf_until_saved(self) -> None:
        drawing = Path.cwd() / f".test_lazy_assign_{uuid4().hex}.dxf"
        ezdxf.new().saveas(drawing)
        dialog = FloorDxfManagerDialog(["Ground"], {})
        try:
            dialog.table.selectRow(0)
            with patch(
                "vehicle_tracking.app.QFileDialog.getOpenFileName",
                return_value=(str(drawing), "DXF files (*.dxf)"),
            ), patch("vehicle_tracking.app.load_dxf") as loader:
                dialog.assign_dxf()
                loader.assert_not_called()
            self.assertEqual(dialog.configuration()[1]["Ground"], drawing)
        finally:
            dialog.close()
            drawing.unlink(missing_ok=True)

    def test_switching_floor_loads_its_assigned_dxf(self) -> None:
        ground_dxf = Path.cwd() / f".test_ground_{uuid4().hex}.dxf"
        upper_dxf = Path.cwd() / f".test_upper_{uuid4().hex}.dxf"
        ground_doc = ezdxf.new()
        ground_doc.modelspace().add_line((0, 0), (10, 0))
        ground_doc.saveas(ground_dxf)
        upper_doc = ezdxf.new()
        upper_doc.modelspace().add_line((100, 100), (120, 100))
        upper_doc.saveas(upper_dxf)
        window = VehicleTrackerWindow()
        try:
            window.levels = ["Ground", "Upper"]
            window.level_drawing_paths = {"Ground": ground_dxf, "Upper": upper_dxf}
            window.level_combo.blockSignals(True)
            window.level_combo.clear()
            window.level_combo.addItems(window.levels)
            window.level_combo.blockSignals(False)
            window.change_level("Ground")
            self.assertEqual(window.current_dxf.path, ground_dxf)
            window.change_level("Upper")
            self.assertEqual(window.current_dxf.path, upper_dxf)
            self.assertEqual(window.current_dxf.bounds, (100.0, 100.0, 120.0, 100.0))
        finally:
            window.close()
            ground_dxf.unlink(missing_ok=True)
            upper_dxf.unlink(missing_ok=True)

    def test_project_file_contains_floor_dxfs_routes_starts_and_vehicles(self) -> None:
        project_path = Path.cwd() / f".test_project_{uuid4().hex}.vtproject"
        ground_dxf = Path.cwd() / f".test_project_ground_{uuid4().hex}.dxf"
        ezdxf.new().saveas(ground_dxf)
        project = VehicleTrackingProject(
            ["Ground"],
            {"Ground": ground_dxf},
            [StartPosition("Goods In", "Ground", Pose(1.0, 2.0, 15.0))],
            [
                RoutePlan(
                    "Stores",
                    Pose(10.0, 20.0, 90.0),
                    [(5.0, 7.0)],
                    level_name="Ground",
                    start_position_name="Goods In",
                    start_pose=Pose(1.0, 2.0, 15.0),
                )
            ],
            [VehicleProfile(name="Project Forklift")],
            "Ground",
            "Goods In",
        )
        try:
            ProjectStore.save(project_path, project)
            loaded = ProjectStore.load(project_path)
            self.assertEqual(loaded.level_drawings["Ground"], ground_dxf.resolve())
            self.assertEqual(loaded.routes[0].name, "Stores")
            self.assertEqual(loaded.start_positions[0].name, "Goods In")
            self.assertEqual(loaded.vehicles[0].name, "Project Forklift")
        finally:
            project_path.unlink(missing_ok=True)
            ground_dxf.unlink(missing_ok=True)

    def test_window_opens_project_and_uses_embedded_vehicle_and_floor(self) -> None:
        project_path = Path.cwd() / f".test_open_project_{uuid4().hex}.vtproject"
        drawing_path = Path.cwd() / f".test_open_project_{uuid4().hex}.dxf"
        document = ezdxf.new()
        document.modelspace().add_line((20, 30), (40, 30))
        document.saveas(drawing_path)
        ProjectStore.save(
            project_path,
            VehicleTrackingProject(
                ["Upper"],
                {"Upper": drawing_path},
                [StartPosition("Lift", "Upper", Pose(20.0, 30.0, 90.0))],
                [],
                [VehicleProfile(name="Project AMR")],
                "Upper",
                "Lift",
            ),
        )
        window = VehicleTrackerWindow()
        try:
            window._open_project_file(project_path)
            self.assertEqual(window.project_file_path, project_path.resolve())
            self.assertEqual(window.current_level_name, "Upper")
            self.assertEqual(window.current_dxf.path, drawing_path.resolve())
            self.assertEqual(window.vehicles[0].name, "Project AMR")
            self.assertEqual(window.current_start_name, "Lift")
        finally:
            window.close()
            project_path.unlink(missing_ok=True)
            drawing_path.unlink(missing_ok=True)

    def test_qtbootstrap_supplies_complete_light_and_dark_palettes(self) -> None:
        QtBootstrap.apply(self.app, dark=True, dxf_background="#123456")
        self.assertTrue(QtBootstrap.is_dark())
        self.assertEqual(
            self.app.palette().color(QPalette.ColorRole.Window).name(), "#111827"
        )
        self.assertIn("QScrollArea", self.app.styleSheet())
        self.assertIn("background: #1f2937", self.app.styleSheet())
        self.assertIn("background: #123456", self.app.styleSheet())
        QtBootstrap.apply(self.app, dark=False)
        self.assertFalse(QtBootstrap.is_dark())
        self.assertEqual(
            self.app.palette().color(QPalette.ColorRole.Window).name(), "#f3f6fb"
        )

    def test_waypoint_tangent_stays_smooth_with_uneven_sections(self) -> None:
        window = VehicleTrackerWindow()
        try:
            poses = window._planned_route_poses_for(
                Pose(12.0, 3.0, 0.0),
                [(9.0, 0.0), (9.2, 3.0)],
                start_pose=Pose(0.0, 0.0, 0.0),
            )
            for join in (40, 80):
                incoming = (poses[join].x - poses[join - 1].x, poses[join].y - poses[join - 1].y)
                outgoing = (poses[join + 1].x - poses[join].x, poses[join + 1].y - poses[join].y)
                dot = incoming[0] * outgoing[0] + incoming[1] * outgoing[1]
                self.assertGreater(dot, 0.0, "An ordinary waypoint must not form a pointed V/cusp")
        finally:
            window.close()

    def test_curve_handle_changes_and_persists_waypoint_tangent(self) -> None:
        window = VehicleTrackerWindow()
        try:
            window.start_pose = Pose(0.0, 0.0, 0.0)
            window.end_pose = Pose(10.0, 5.0, 30.0)
            window.route_waypoints = [(5.0, 1.0)]
            automatic = window.planned_route_poses(window.form_profile())
            window.route_tangent_handles = {0: (0.0, 4.0)}
            adjusted = window.planned_route_poses(window.form_profile())
            self.assertNotAlmostEqual(automatic[30].y, adjusted[30].y)
            window.redraw_route_handles()
            self.assertEqual(len(window.route_tangent_items), 2)
            self.assertEqual(len(window.route_tangent_lines), 1)
        finally:
            window.close()

    def test_curve_handles_do_not_duplicate_when_route_point_moves(self) -> None:
        window = VehicleTrackerWindow()
        try:
            window.start_pose = Pose(0.0, 0.0, 0.0)
            window.poses = [window.start_pose]
            window.end_pose = Pose(10.0, 5.0, 30.0)
            window.route_waypoints = [(5.0, 1.0)]
            window.redraw_scene()
            for offset in range(1, 6):
                window._route_point_released(
                    0, QPointF(5.0 + offset, -(1.0 + offset * 0.1))
                )
            scene_handles = [
                item
                for item in window.scene.items()
                if isinstance(item, CurveTangentHandleItem)
            ]
            self.assertEqual(len(window.route_tangent_items), 2)
            self.assertEqual(len(scene_handles), 2)
            self.assertEqual(len(window.route_tangent_lines), 1)
        finally:
            window.close()

    def test_route_animation_can_pause_and_scrub(self) -> None:
        window = VehicleTrackerWindow()
        try:
            window.start_pose = Pose(0.0, 0.0, 0.0)
            window.end_pose = Pose(10.0, 0.0, 0.0)
            window.toggle_route_animation()
            window.pause_route_animation()
            self.assertFalse(window.route_animation_timer.isActive())
            self.assertTrue(window.route_animation_paused)
            window.route_animation_slider.setValue(20)
            self.assertEqual(window.route_animation_index, 20)
            self.assertIsNotNone(window.route_animation_item)
        finally:
            window.close()

    def test_mp4_route_information_box_avoids_the_route(self) -> None:
        window = VehicleTrackerWindow()
        try:
            route = [Pose(float(x), 90.0, 0.0) for x in range(0, 101, 5)]
            source = QRectF(0.0, -100.0, 100.0, 100.0)
            panel = window._route_video_overlay_rect(
                route, source, 1280, 720, 460.0, 130.0
            )
            self.assertGreater(panel.top(), 300.0)

            image = QImage(1280, 720, QImage.Format.Format_RGB888)
            image.fill(QColor("#ffffff"))
            painter = QPainter(image)
            window._draw_route_video_overlay(
                painter,
                route,
                source,
                1280,
                720,
                "Delivery route",
                "Level 3",
                "Travel -> Drop off payload -> Reverse -> Stop",
            )
            painter.end()
            self.assertNotEqual(image.pixelColor(panel.center().toPoint()), QColor("#ffffff"))
        finally:
            window.close()

    def test_mp4_export_hides_non_selected_route_traces_then_restores_them(self) -> None:
        window = VehicleTrackerWindow()
        try:
            driven_path = window.scene.addLine(0.0, 0.0, 5.0, 0.0)
            driven_sweep = window.scene.addLine(0.0, 1.0, 5.0, 1.0)
            saved_path = window.scene.addLine(0.0, 2.0, 5.0, 2.0)
            saved_marker = window.scene.addEllipse(1.0, 1.0, 1.0, 1.0)
            saved_marker.setToolTip("Adjacent path drop-off: Other path")
            static_vehicle = window.scene.createItemGroup([])
            planned_trace = window.scene.addLine(0.0, 3.0, 5.0, 3.0)
            planned_trace.setData(0, "planned-payload-trace")
            driven_payload = window.scene.addLine(0.0, 4.0, 5.0, 4.0)
            driven_payload.setData(0, "driven-payload-trace")
            window.path_item = driven_path
            window.sweep_items = [driven_sweep]
            window.saved_route_items = [saved_path]
            window.position_items = [saved_marker]
            window.vehicle_items = [static_vehicle]
            window.payload_trace_items = [planned_trace, driven_payload]

            states = window._hide_non_selected_route_items_for_export()

            self.assertFalse(driven_path.isVisible())
            self.assertFalse(driven_sweep.isVisible())
            self.assertFalse(saved_path.isVisible())
            self.assertFalse(saved_marker.isVisible())
            self.assertFalse(static_vehicle.isVisible())
            self.assertFalse(driven_payload.isVisible())
            self.assertTrue(planned_trace.isVisible())

            window._restore_items_after_route_export(states)
            self.assertTrue(all(item.isVisible() for item, _visible in states))
        finally:
            window.close()

    def test_pdf_and_mp4_exports_create_real_files(self) -> None:
        pdf = Path.cwd() / f".test_report_{uuid4().hex}.pdf"
        mp4 = Path.cwd() / f".test_video_{uuid4().hex}.mp4"
        poses = [Pose(0.0, 0.0, 0.0), Pose(3.0, 1.0, 20.0), Pose(5.0, 2.0, 0.0)]
        try:
            generate_route_report_pdf(
                pdf,
                [
                    RouteReportEntry(
                        "Test path",
                        "Ground",
                        "Goods In",
                        poses[0],
                        poses[-1],
                        poses,
                        True,
                        0,
                    )
                ],
            )
            self.assertTrue(pdf.read_bytes().startswith(b"%PDF"))
            frames = []
            for color in ("#ffffff", "#2563eb", "#16a34a"):
                image = QImage(64, 64, QImage.Format.Format_RGB888)
                image.fill(QColor(color))
                frames.append(image)
            self.assertEqual(export_qimages_to_mp4(mp4, frames, 64, 64, 10), 3)
            self.assertGreater(mp4.stat().st_size, 500)
        finally:
            pdf.unlink(missing_ok=True)
            mp4.unlink(missing_ok=True)

    def test_pdf_report_wraps_long_notes_and_draws_dxf_context_on_one_page(self) -> None:
        pdf = Path.cwd() / f".test_report_layout_{uuid4().hex}.pdf"
        poses = [Pose(0.0, 0.0, 0.0), Pose(5.0, 1.0, 0.0), Pose(10.0, 0.0, 0.0)]
        long_notes = "operations: " + " -> ".join(["curved turn", "reverse"] * 12)
        try:
            generate_route_report_pdf(
                pdf,
                [
                    RouteReportEntry(
                        "Context path",
                        "Ground",
                        "Goods In",
                        poses[0],
                        poses[-1],
                        poses,
                        True,
                        0,
                        long_notes,
                        [
                            DxfPrimitive("line", [(-5.0, -2.0), (15.0, -2.0)]),
                            DxfPrimitive("line", [(2.0, -5.0), (2.0, 5.0)]),
                        ],
                        [
                            DxfPrimitive(
                                "line",
                                [(-1.0, -0.5), (1.0, -0.5), (1.0, 0.5), (-1.0, 0.5), (-1.0, -0.5)],
                            )
                        ],
                        [(-1.0, -0.5), (1.0, -0.5), (1.0, 0.5), (-1.0, 0.5)],
                    )
                ],
                "floor.dxf",
            )
            content = pdf.read_bytes()
            self.assertTrue(content.startswith(b"%PDF"))
            self.assertEqual(len(re.findall(rb"/Type\s+/Page\b", content)), 1)
            self.assertGreater(len(content), 2_000)
        finally:
            pdf.unlink(missing_ok=True)


if __name__ == "__main__":
    unittest.main()
