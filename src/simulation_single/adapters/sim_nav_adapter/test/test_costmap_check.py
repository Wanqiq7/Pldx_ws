import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import unittest


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "check_sim_costmap.py"
SPEC = importlib.util.spec_from_file_location("check_sim_costmap", SCRIPT_PATH)
CHECK_SIM_COSTMAP = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CHECK_SIM_COSTMAP)


def quaternion_from_yaw(yaw):
    return SimpleNamespace(
        x=0.0,
        y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0),
    )


class CostmapCheckTest(unittest.TestCase):
    def test_transform_xy_applies_rotation_and_translation(self):
        transform = SimpleNamespace(
            translation=SimpleNamespace(x=10.0, y=20.0),
            rotation=quaternion_from_yaw(math.pi / 2.0),
        )

        actual = CHECK_SIM_COSTMAP.transform_xy(transform, (2.0, 1.0))

        self.assertAlmostEqual(actual[0], 9.0)
        self.assertAlmostEqual(actual[1], 22.0)

    def test_grid_cell_accounts_for_rotated_origin(self):
        grid = SimpleNamespace(
            info=SimpleNamespace(
                origin=SimpleNamespace(
                    position=SimpleNamespace(x=10.0, y=20.0),
                    orientation=quaternion_from_yaw(math.pi / 2.0),
                ),
                resolution=0.5,
                width=20,
                height=20,
            )
        )

        actual = CHECK_SIM_COSTMAP.grid_cell(grid, (9.25, 21.25))

        self.assertEqual(actual, (2, 1))

    def test_eight_connected_false_wall_spans_lateral_bounds(self):
        cells = {(index, index) for index in range(11)}
        bounds = {
            cell: (-0.5 + index * 0.1, -0.5 + index * 0.1)
            for index, cell in enumerate(sorted(cells))
        }

        self.assertTrue(
            CHECK_SIM_COSTMAP.spanning_false_wall(cells, bounds, -0.5, 0.5, 0.1)
        )

    def test_disconnected_false_cells_do_not_form_wall(self):
        cells = {(0, 0), (1, 1), (8, 8), (9, 9)}
        bounds = {
            (0, 0): (-0.5, -0.5),
            (1, 1): (-0.4, -0.4),
            (8, 8): (0.4, 0.4),
            (9, 9): (0.5, 0.5),
        }

        self.assertFalse(
            CHECK_SIM_COSTMAP.spanning_false_wall(cells, bounds, -0.5, 0.5, 0.1)
        )


if __name__ == "__main__":
    unittest.main()
