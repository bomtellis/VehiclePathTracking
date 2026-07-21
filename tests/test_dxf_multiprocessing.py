from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import ezdxf

from vehicle_tracking.dxf_io import load_dxf_process_safe


class DxfMultiprocessingTests(unittest.TestCase):
    @staticmethod
    def _create_dxf(path: Path, offset: float) -> None:
        document = ezdxf.new("R2010")
        document.modelspace().add_line(
            (offset, offset),
            (offset + 10.0, offset + 5.0),
        )
        block = document.blocks.new("TEST_VEHICLE")
        block.add_lwpolyline([(0.0, 0.0), (2.0, 0.0), (2.0, 1.0)], close=True)
        document.saveas(path)

    def test_drawings_can_be_loaded_and_returned_by_worker_processes(self) -> None:
        with TemporaryDirectory() as directory:
            paths = [Path(directory) / f"floor-{index}.dxf" for index in range(2)]
            for index, path in enumerate(paths):
                self._create_dxf(path, float(index * 20))

            with ProcessPoolExecutor(max_workers=2) as executor:
                futures = [
                    executor.submit(
                        load_dxf_process_safe,
                        str(path),
                        ("TEST_VEHICLE",),
                    )
                    for path in paths
                ]
                drawings = [future.result() for future in futures]

        self.assertEqual(paths, [drawing.path for drawing in drawings])
        self.assertTrue(all(drawing.doc is None for drawing in drawings))
        self.assertTrue(all(drawing.primitives for drawing in drawings))
        self.assertTrue(
            all("TEST_VEHICLE" in drawing.block_geometries for drawing in drawings)
        )


if __name__ == "__main__":
    unittest.main()
