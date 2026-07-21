import os
from pathlib import Path
import unittest
from uuid import uuid4

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import ezdxf
from ezdxf.enums import TextEntityAlignment
from PySide6.QtGui import QPen
from PySide6.QtWidgets import QApplication, QGraphicsScene, QGraphicsTextItem

from vehicle_tracking.app import _add_primitives_to_scene
from vehicle_tracking.dxf_io import load_dxf


class DxfTextRenderingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QApplication.instance() or QApplication([])

    def test_text_mtext_and_attrib_are_preserved_as_arial_text_objects(self) -> None:
        path = Path.cwd() / f".test_text_{uuid4().hex}.dxf"
        document = ezdxf.new()
        modelspace = document.modelspace()
        modelspace.add_line((0, 0), (20, 0))
        modelspace.add_text(
            "Loading Bay",
            height=2.5,
            rotation=30.0,
        ).set_placement((10, 5), align=TextEntityAlignment.MIDDLE_CENTER)
        modelspace.add_mtext(
            r"Upper\PFloor",
            dxfattribs={"insert": (30, 12), "char_height": 3.0, "attachment_point": 1},
        )
        document.saveas(path)
        progress = []
        try:
            drawing = load_dxf(path, lambda value, message: progress.append((value, message)))
            text_primitives = [item for item in drawing.primitives if item.kind == "text"]
            self.assertEqual(len(text_primitives), 2)
            self.assertEqual(text_primitives[0].text, "Loading Bay")
            self.assertAlmostEqual(text_primitives[0].text_height, 2.5)
            self.assertAlmostEqual(text_primitives[0].rotation_deg, 30.0)
            self.assertEqual(text_primitives[0].horizontal_alignment, "center")
            self.assertEqual(text_primitives[1].text, "Upper\nFloor")
            self.assertEqual(progress[0][0], 5)
            self.assertEqual(progress[-1][0], 100)

            scene = QGraphicsScene()
            items = _add_primitives_to_scene(scene, drawing.primitives, QPen())
            rendered_text = [item for item in items if isinstance(item, QGraphicsTextItem)]
            self.assertEqual(len(rendered_text), 2)
            self.assertTrue(all(item.font().family() == "Arial" for item in rendered_text))
            self.assertTrue(all(not item.boundingRect().isEmpty() for item in rendered_text))
        finally:
            path.unlink(missing_ok=True)


if __name__ == "__main__":
    unittest.main()
