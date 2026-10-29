#!/usr/bin/env python3
"""Regression checks for the tunnel acceptance verdict and wheel-wall contacts."""

import copy
import math
import unittest
from types import SimpleNamespace

import numpy as np

from gz.msgs10.contact_pb2 import Contact
from geometry_msgs.msg import Twist, Wrench
from nav_msgs.msg import Odometry
from ros_interfaces.msg import MpcPositionCommand, PositionCommand
from sentry_simulation.cmd_vel_adapter import SentrySimCmdAdapter

import check_sim_tunnel
from check_sim_tunnel import wheel_contact_is_obstacle
from check_sim_tunnel_navigation import assess


class TunnelAcceptanceTest(unittest.TestCase):
    def test_folded_adapter_rotates_reference_and_preserves_lateral_correction(self):
        commands, forces = [], []
        rotation = SimpleNamespace(x=0, y=0, z=math.sin(-math.pi / 8), w=math.cos(-math.pi / 8))
        node = SimpleNamespace(
            _passage_reference=None, _input_frame="camera_init",
            _tf_buffer=SimpleNamespace(lookup_transform=lambda *args: SimpleNamespace(
                transform=SimpleNamespace(rotation=rotation))),
            _body_yaw=lambda: 0.0, _gimbal_ready=lambda: True, _gimbal_topic="/status",
            _gimbal_state={"target": "folded", "front_yaw_in_chassis": -math.pi / 4},
            _publish_command=commands.append, _force_publisher=SimpleNamespace(publish=forces.append),
            _now_seconds=lambda: 1.0,
        )
        reference = MpcPositionCommand(command_flag=MpcPositionCommand.NORMAL_COMMAND)
        reference.header.frame_id = "map"
        reference.cmds = [PositionCommand(), PositionCommand()]
        reference.cmds[1].position.x = 0.5
        SentrySimCmdAdapter._reference_callback(node, reference)
        node._reference_heading = lambda: SentrySimCmdAdapter._reference_heading(node)
        self.assertAlmostEqual(node._reference_heading(), -math.pi / 4)
        twist = Twist()
        twist.linear.x, twist.linear.y = 0.08, -0.05
        SentrySimCmdAdapter._forward(node, twist, Wrench())
        self.assertAlmostEqual(commands[-1].linear.x, 0.08)
        self.assertAlmostEqual(commands[-1].linear.y, -0.05)
        self.assertAlmostEqual(commands[-1].angular.z, 0.0)
        reference.command_flag = MpcPositionCommand.BLOCK_COMMAND
        SentrySimCmdAdapter._reference_callback(node, reference)
        SentrySimCmdAdapter._forward(node, twist, Wrench())
        self.assertEqual(commands[-1], Twist())

    def test_restore_requires_stationary_platform_and_all_wheels_supported(self):
        allowed = check_sim_tunnel.platform_restore_allowed
        wheels = [{"gap_m": 0.001, "slope_deg": 1.5, "terrain_z": 0.142}] * 4
        self.assertFalse(allowed([14.4, 13.475], [0, 0, 0], wheels))
        self.assertFalse(allowed([15.5, 13.475], [0, 0, 0], wheels))
        self.assertTrue(allowed([15.9, 13.475], [0, 0, 0], wheels))
        self.assertFalse(allowed([15.9, 13.475], [0.03, 0, 0], wheels))
        self.assertFalse(allowed([15.9, 13.475], [0, 0, 0], wheels[:3]))
        for key, value in (("slope_deg", 11.5), ("terrain_z", -0.058), ("gap_m", 0.03)):
            bad = copy.deepcopy(wheels)
            bad[0] = {**bad[0], key: value}
            self.assertFalse(allowed([15.9, 13.475], [0, 0, 0], bad))

    def test_horizontal_contact_normal_needs_terrain_geometry(self):
        classifier = check_sim_tunnel.TerrainContacts
        floor = np.array([[[-1, -1, 0], [1, -1, 0], [0, 1, 0]]], dtype=float)
        contact = Contact()
        contact.position.add(z=0.001)
        contact.normal.add(x=1.0)
        self.assertTrue(wheel_contact_is_obstacle(contact))
        self.assertFalse(wheel_contact_is_obstacle(contact, classifier(floor)))
        wall = np.array([[[0, -1, -1], [0, 1, -1], [0, 0, 1]]], dtype=float)
        self.assertTrue(wheel_contact_is_obstacle(contact, classifier(np.concatenate([floor, wall]))))
        # A millimetre-thick floor marking has side faces, but still supports a wheel.
        seam = wall.copy()
        seam[:, :, 2] *= 0.001
        self.assertFalse(wheel_contact_is_obstacle(contact, classifier(np.concatenate([floor, seam]))))
        contact.position[0].z = 0.00205
        self.assertFalse(wheel_contact_is_obstacle(contact, classifier(floor)))
        self.assertTrue(wheel_contact_is_obstacle(contact, classifier(np.concatenate([floor, wall]))))
        contact.position[0].x = 3.0
        self.assertTrue(wheel_contact_is_obstacle(contact, classifier(floor)))

    def test_platform_restore_waits_for_a_full_second_of_support(self):
        wheels = [{"gap_m": 0.001, "slope_deg": 1.5, "terrain_z": 0.142}] * 4
        restored = []
        node = SimpleNamespace(gt=[], platform_support=lambda: wheels)

        def wait(condition, timeout):
            for timestamp, x in ((0, 14.4), (1, 15.5), (2, 15.9), (3, 15.9)):
                sample = Odometry()
                sample.header.stamp.sec = timestamp
                sample.pose.pose.position.x, sample.pose.pose.position.y = x, 13.475
                sample.pose.pose.orientation.w = 1.0
                node.gt[:] = [sample]
                if condition():
                    return
                self.assertFalse(restored)
            self.fail("platform never became eligible")

        node.wait = wait
        node.request_posture = restored.append
        node.await_state = lambda target: self.assertEqual(target, "normal")
        evidence = check_sim_tunnel.TunnelAccuracy.restore_on_platform(node)
        self.assertEqual(restored, ["normal"])
        self.assertEqual(evidence["sim_time_s"], 3)
        self.assertEqual(evidence["stable_seconds"], 1)

    def test_platform_hold_measures_drift_after_restoration(self):
        for drift, expected in ((0.0, True), (0.05, False)):
            initial = Odometry()
            initial.pose.pose.position.x = 16.0
            initial.pose.pose.position.y = 13.425
            initial.pose.pose.orientation.w = 1.0
            node = SimpleNamespace(gt=[initial])

            def wait(condition, timeout):
                self.assertFalse(condition())
                finish = copy.deepcopy(initial)
                finish.header.stamp.sec = 6
                finish.pose.pose.position.y += drift
                node.gt[:] = [finish]
                self.assertTrue(condition())

            node.wait = wait
            result = check_sim_tunnel.TunnelAccuracy.hold_on_platform(node)
            self.assertEqual(result["passed"], expected)
            self.assertAlmostEqual(result["drift_m"], drift)

    def test_exit_reached_is_insufficient_when_robot_rolls_back(self):
        samples = [{"phase": "autonomous_passage", "t": 1.0, "truth": [13.5, 13.475],
                    "truth_velocity": [0.1, 0.0, 0.0], "localization_error": 0.02,
                    "front_heading_error_rad": 0.01,
                    "gimbal": {"target": "folded", "ready": True}}]
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
        premature = copy.deepcopy(samples)
        premature[0]["gimbal"]["target"] = "normal"
        self.assertFalse(assess(premature, [15.8, 13.475], 0.05, 0.15, 5.0)["passed"])
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
