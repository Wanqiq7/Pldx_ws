#!/usr/bin/env python3
"""Regression checks for the tunnel acceptance verdict and wheel-wall contacts."""

import copy
import math
import unittest

from gz.msgs10.contact_pb2 import Contact

from check_sim_tunnel import wheel_contact_is_obstacle
from check_sim_tunnel_navigation import assess


class TunnelAcceptanceTest(unittest.TestCase):
    def test_exit_reached_is_insufficient_when_robot_rolls_back(self):
        samples = [{"phase": "autonomous_passage", "t": 1.0, "truth": [13.5, 13.475],
                    "truth_velocity": [0.1, 0.0, 0.0], "localization_error": 0.02,
                    "front_heading_error_rad": 0.01}]
        samples += [{"phase": "exit_hold", "t": t, "truth": [15.8, 13.475],
                     "truth_velocity": [0.0, 0.0, 0.0], "localization_error": 0.02}
                    for t in (2.0, 7.0)]
        self.assertTrue(assess(samples, [15.8, 13.475], 0.05, 0.15, 5.0)["passed"])
        rollback = copy.deepcopy(samples)
        rollback[-1]["truth"][0] -= 0.03
        self.assertFalse(assess(rollback, [15.8, 13.475], 0.05, 0.15, 5.0)["passed"])
        self.assertFalse(assess(samples[:-1], [15.8, 13.475], 0.05, 0.15, 5.0)["passed"])
        invalid = copy.deepcopy(samples)
        invalid[-1]["truth"][0] = math.nan
        self.assertFalse(assess(invalid, [15.8, 13.475], 0.05, 0.15, 5.0)["passed"])
        samples[0]["localization_error"] = 0.06
        self.assertFalse(assess(samples, [15.8, 13.475], 0.05, 0.15, 5.0)["passed"])

    def test_wheel_wall_and_missing_normals_fail_contact_check(self):
        contact = Contact()
        contact.position.add()
        self.assertTrue(wheel_contact_is_obstacle(contact))
        contact.normal.add(z=1.0)
        self.assertFalse(wheel_contact_is_obstacle(contact))
        contact.normal[0].z = math.cos(math.radians(12))
        contact.normal[0].x = math.sin(math.radians(12))
        self.assertFalse(wheel_contact_is_obstacle(contact))
        contact.normal[0].z = 0.0
        contact.normal[0].x = 1.0
        self.assertTrue(wheel_contact_is_obstacle(contact))


if __name__ == "__main__":
    unittest.main()
