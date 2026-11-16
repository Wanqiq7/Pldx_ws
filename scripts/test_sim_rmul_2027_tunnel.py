"""Static checks for the RMUL 2027 tunnel acceptance contract."""

import math
import unittest

from check_sim_rmul_2027_tunnel import (
    ENTRY,
    PASSAGE_SPEED,
    Rmul2027TunnelAcceptance,
    TUNNEL_EXIT_X,
    TUNNEL_PATH_Y,
)


class Rmul2027TunnelContractTest(unittest.TestCase):
    def test_route_uses_step_tunnel_coordinates(self):
        self.assertEqual(ENTRY[:2], (2.0, 7.46))
        self.assertEqual(TUNNEL_PATH_Y, 7.46)
        self.assertEqual(TUNNEL_EXIT_X, 4.2)
        self.assertGreater(PASSAGE_SPEED, 0.0)

    def test_folded_clearance_report_is_finite_and_has_margin_field(self):
        report = Rmul2027TunnelAcceptance.clearance_check()
        self.assertTrue(math.isfinite(report["folded_max_z_m"]))
        self.assertTrue(math.isfinite(report["folded_clearance_m"]))
        self.assertGreater(report["required_clearance_margin_m"], 0.0)


if __name__ == "__main__":
    unittest.main()
