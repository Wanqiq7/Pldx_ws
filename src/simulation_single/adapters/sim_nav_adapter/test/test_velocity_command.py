import math
import unittest

from sim_nav_adapter.velocity_command import (
    CommandMode,
    PlanarCommand,
    VelocityCommandGate,
    global_to_body,
)


class VelocityCommandTransformTest(unittest.TestCase):
    def assert_command_almost_equal(self, actual, expected):
        self.assertAlmostEqual(actual.vx, expected.vx, places=9)
        self.assertAlmostEqual(actual.vy, expected.vy, places=9)
        self.assertAlmostEqual(actual.wz, expected.wz, places=9)

    def test_global_velocity_is_rotated_into_body_frame(self):
        command = PlanarCommand(1.0, 0.0, 0.7)

        cases = (
            (0.0, PlanarCommand(1.0, 0.0, 0.7)),
            (math.pi / 2.0, PlanarCommand(0.0, -1.0, 0.7)),
            (-math.pi / 2.0, PlanarCommand(0.0, 1.0, 0.7)),
            (math.pi, PlanarCommand(-1.0, 0.0, 0.7)),
        )
        for yaw, expected in cases:
            with self.subTest(yaw=yaw):
                self.assert_command_almost_equal(global_to_body(command, yaw), expected)

    def test_rotation_preserves_linear_norm_and_angular_velocity(self):
        command = PlanarCommand(1.2, -0.4, -2.3)

        transformed = global_to_body(command, 1.1)

        self.assertAlmostEqual(
            math.hypot(transformed.vx, transformed.vy),
            math.hypot(command.vx, command.vy),
            places=9,
        )
        self.assertEqual(transformed.wz, command.wz)


class VelocityCommandGateTest(unittest.TestCase):
    def setUp(self):
        self.gate = VelocityCommandGate(source_timeout_sec=0.2)

    def test_controller_uses_smoothed_global_command_after_raw_health_signal(self):
        self.assertIsNone(
            self.gate.accept_controller_raw(PlanarCommand(1.0, 0.0, 0.5), 1.0)
        )

        output = self.gate.accept_controller_smoothed(
            PlanarCommand(1.0, 0.0, 0.5),
            now_sec=1.01,
            yaw=math.pi / 2.0,
            odom_age_sec=0.01,
        )

        self.assertEqual(self.gate.mode, CommandMode.CONTROLLER)
        self.assertAlmostEqual(output.vx, 0.0, places=9)
        self.assertAlmostEqual(output.vy, -1.0, places=9)
        self.assertEqual(output.wz, 0.5)

    def test_raw_zero_stops_immediately_and_rejects_residual_smoother_output(self):
        self.gate.accept_controller_raw(PlanarCommand(1.0, 0.0, 0.5), 1.0)
        self.gate.accept_controller_smoothed(
            PlanarCommand(0.4, 0.0, 0.2), 1.01, yaw=0.0, odom_age_sec=0.01
        )

        stop = self.gate.accept_controller_raw(PlanarCommand.zero(), 1.02)
        residual = self.gate.accept_controller_smoothed(
            PlanarCommand(0.3, 0.0, 0.1), 1.03, yaw=0.0, odom_age_sec=0.01
        )

        self.assertEqual(stop, PlanarCommand.zero())
        self.assertIsNone(residual)
        self.assertEqual(self.gate.mode, CommandMode.STOPPED)

    def test_behavior_is_body_frame_passthrough_and_controller_reclaims_after_stop(
        self,
    ):
        behavior = PlanarCommand(-0.3, 0.2, 0.4)

        output = self.gate.accept_behavior(behavior, now_sec=2.0, odom_age_sec=0.01)
        self.assertEqual(output, behavior)
        self.assertEqual(self.gate.mode, CommandMode.BEHAVIOR)

        self.assertEqual(
            self.gate.accept_behavior(PlanarCommand.zero(), 2.005, odom_age_sec=0.01),
            PlanarCommand.zero(),
        )
        self.gate.accept_controller_raw(PlanarCommand(0.5, 0.0, 0.0), 2.01)
        controller_output = self.gate.accept_controller_smoothed(
            PlanarCommand(0.5, 0.0, 0.0),
            now_sec=2.02,
            yaw=0.0,
            odom_age_sec=0.01,
        )
        self.assertEqual(controller_output, PlanarCommand(0.5, 0.0, 0.0))
        self.assertEqual(self.gate.mode, CommandMode.CONTROLLER)

    def test_behavior_ignores_delayed_controller_samples(self):
        behavior = PlanarCommand(-0.3, 0.2, 0.4)
        self.gate.accept_behavior(behavior, now_sec=2.0, odom_age_sec=0.01)

        delayed_nonzero = self.gate.accept_controller_raw(
            PlanarCommand(0.5, 0.0, 0.0), 2.01
        )
        delayed_zero = self.gate.accept_controller_raw(PlanarCommand.zero(), 2.02)

        self.assertIsNone(delayed_nonzero)
        self.assertIsNone(delayed_zero)
        self.assertEqual(self.gate.mode, CommandMode.BEHAVIOR)
        self.assertEqual(
            self.gate.accept_behavior(behavior, 2.03, odom_age_sec=0.01),
            behavior,
        )

    def test_controller_ignores_delayed_behavior_stop(self):
        self.gate.accept_controller_raw(PlanarCommand(0.5, 0.0, 0.0), 2.0)
        self.gate.accept_controller_smoothed(
            PlanarCommand(0.5, 0.0, 0.0), 2.01, yaw=0.0, odom_age_sec=0.01
        )

        delayed_zero = self.gate.accept_behavior(
            PlanarCommand.zero(), 2.02, odom_age_sec=0.01
        )

        self.assertIsNone(delayed_zero)
        self.assertEqual(self.gate.mode, CommandMode.CONTROLLER)

    def test_stale_controller_or_odometry_fails_closed_once(self):
        self.gate.accept_controller_raw(PlanarCommand(0.5, 0.0, 0.0), 3.0)
        self.gate.accept_controller_smoothed(
            PlanarCommand(0.5, 0.0, 0.0),
            now_sec=3.01,
            yaw=0.0,
            odom_age_sec=0.01,
        )

        self.assertEqual(
            self.gate.poll(now_sec=3.21, odom_age_sec=0.01),
            PlanarCommand.zero(),
        )
        self.assertIsNone(self.gate.poll(now_sec=3.22, odom_age_sec=0.01))

        self.gate.accept_behavior(PlanarCommand(0.2, 0.0, 0.0), 4.0, odom_age_sec=0.01)
        self.assertEqual(
            self.gate.poll(now_sec=4.05, odom_age_sec=0.21),
            PlanarCommand.zero(),
        )

    def test_invalid_command_fails_closed(self):
        invalid = PlanarCommand(float("nan"), 0.0, 0.0)

        self.assertEqual(
            self.gate.accept_behavior(invalid, 5.0, odom_age_sec=0.0),
            PlanarCommand.zero(),
        )
        self.assertEqual(self.gate.mode, CommandMode.STOPPED)


if __name__ == "__main__":
    unittest.main()
