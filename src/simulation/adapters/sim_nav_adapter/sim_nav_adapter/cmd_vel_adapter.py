#!/usr/bin/env python3
"""Convert MINCO global commands to the Gazebo chassis body-frame contract."""

import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.node import Node

from sim_nav_adapter.transform_math import yaw_from_quaternion
from sim_nav_adapter.velocity_command import PlanarCommand, VelocityCommandGate


class SimulationCmdVelAdapter(Node):
    def __init__(self):
        super().__init__("sim_cmd_vel_adapter")
        self.declare_parameter(
            "controller_raw_topic", "/simulation/controller_cmd_vel_global_raw"
        )
        self.declare_parameter(
            "controller_smoothed_topic",
            "/simulation/controller_cmd_vel_global_smoothed",
        )
        self.declare_parameter("behavior_topic", "/simulation/behavior_cmd_vel_body")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("global_frame", "sim_odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("source_timeout_sec", 0.2)
        self.declare_parameter("odom_timeout_sec", 0.2)
        self.declare_parameter("watchdog_period_sec", 0.02)

        self.global_frame = self.get_parameter("global_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        source_timeout = float(self.get_parameter("source_timeout_sec").value)
        odom_timeout = float(self.get_parameter("odom_timeout_sec").value)
        watchdog_period = float(self.get_parameter("watchdog_period_sec").value)
        if watchdog_period <= 0.0:
            raise ValueError("watchdog_period_sec must be positive")

        self.gate = VelocityCommandGate(source_timeout, odom_timeout)
        self.latest_yaw = math.nan
        self.last_odom_monotonic = None
        self.last_warning_monotonic = {}

        self.publisher = self.create_publisher(
            Twist, self.get_parameter("output_topic").value, 10
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            self._odom_callback,
            10,
        )
        self.raw_subscription = self.create_subscription(
            Twist,
            self.get_parameter("controller_raw_topic").value,
            self._controller_raw_callback,
            10,
        )
        self.smoothed_subscription = self.create_subscription(
            Twist,
            self.get_parameter("controller_smoothed_topic").value,
            self._controller_smoothed_callback,
            10,
        )
        self.behavior_subscription = self.create_subscription(
            Twist,
            self.get_parameter("behavior_topic").value,
            self._behavior_callback,
            10,
        )
        self.watchdog_timer = self.create_timer(
            watchdog_period, self._watchdog_callback
        )

        self.get_logger().info(
            "Simulation cmd_vel adapter ready: controller commands are global, "
            "behavior commands and /cmd_vel are base_link"
        )

    def _warn_throttled(self, key, message):
        now = time.monotonic()
        if now - self.last_warning_monotonic.get(key, -math.inf) >= 2.0:
            self.get_logger().warning(message)
            self.last_warning_monotonic[key] = now

    def _odom_callback(self, message):
        if message.header.frame_id != self.global_frame:
            self._warn_throttled(
                "odom_parent",
                f"Ignoring odom parent '{message.header.frame_id}', "
                f"expected '{self.global_frame}'",
            )
            return
        if message.child_frame_id != self.base_frame:
            self._warn_throttled(
                "odom_child",
                f"Ignoring odom child '{message.child_frame_id}', "
                f"expected '{self.base_frame}'",
            )
            return

        orientation = message.pose.pose.orientation
        try:
            yaw = yaw_from_quaternion(
                (orientation.x, orientation.y, orientation.z, orientation.w)
            )
        except ValueError as error:
            self._warn_throttled("odom_quaternion", f"Ignoring invalid odom: {error}")
            return
        if not math.isfinite(yaw):
            self._warn_throttled("odom_yaw", "Ignoring non-finite odom yaw")
            return

        self.latest_yaw = yaw
        self.last_odom_monotonic = time.monotonic()

    @staticmethod
    def _command_from_twist(message):
        values = (
            message.linear.x,
            message.linear.y,
            message.linear.z,
            message.angular.x,
            message.angular.y,
            message.angular.z,
        )
        if not all(math.isfinite(value) for value in values):
            return PlanarCommand(math.nan, 0.0, 0.0)
        return PlanarCommand(message.linear.x, message.linear.y, message.angular.z)

    def _odom_age(self, now):
        if self.last_odom_monotonic is None:
            return math.inf
        return max(0.0, now - self.last_odom_monotonic)

    def _publish(self, command):
        message = Twist()
        message.linear.x = command.vx
        message.linear.y = command.vy
        message.angular.z = command.wz
        self.publisher.publish(message)

    def _controller_raw_callback(self, message):
        now = time.monotonic()
        command = self._command_from_twist(message)
        output = self.gate.accept_controller_raw(command, now)
        if not command.is_finite():
            self._warn_throttled(
                "controller_raw", "Invalid raw controller command; stopping"
            )
        if output is not None:
            self._publish(output)

    def _controller_smoothed_callback(self, message):
        now = time.monotonic()
        command = self._command_from_twist(message)
        output = self.gate.accept_controller_smoothed(
            command,
            now,
            self.latest_yaw,
            self._odom_age(now),
        )
        if not command.is_finite():
            self._warn_throttled(
                "controller_smoothed", "Invalid smoothed controller command; stopping"
            )
        if output is not None:
            self._publish(output)

    def _behavior_callback(self, message):
        now = time.monotonic()
        command = self._command_from_twist(message)
        output = self.gate.accept_behavior(command, now, self._odom_age(now))
        if not command.is_finite():
            self._warn_throttled("behavior", "Invalid behavior command; stopping")
        if output is not None:
            self._publish(output)

    def _watchdog_callback(self):
        now = time.monotonic()
        output = self.gate.poll(now, self._odom_age(now))
        if output is not None:
            self._warn_throttled("watchdog", "Velocity input became stale; stopping")
            self._publish(output)

    def publish_stop(self):
        self._publish(PlanarCommand.zero())


def main():
    rclpy.init()
    node = SimulationCmdVelAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except _rclpy.RCLError:
        if rclpy.ok():
            raise
    finally:
        # rclpy may invalidate the context from its SIGINT handler before the
        # executor unwinds. Do not publish through an invalid context during
        # shutdown; Gazebo's own command watchdog remains the final stop path.
        if rclpy.ok():
            node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
