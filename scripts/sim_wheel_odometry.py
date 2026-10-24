#!/usr/bin/env python3
"""Emulate chassis wheel odometry in simulation.

The omni simulation drives the chassis with a wrench and passive wheels, so there are
no wheel encoder readings. This node republishes the ground-truth body-frame twist as
the body-frame chassis velocity the real MC02 bridge publishes, with encoder-like
errors (white noise, per-axis scale error) so the controller cannot see perfect truth.
It is a stand-in for validating the controller-side fusion, not a wheel model: slip
and wheel lift are not represented.
"""

import random

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class SimWheelOdometry(Node):
    def __init__(self) -> None:
        super().__init__("sim_wheel_odometry")
        self.declare_parameter("input_topic", "/sim/ground_truth/odom")
        self.declare_parameter("output_topic", "/sentry/chassis_velocity")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("linear_noise_std", 0.01)   # m/s
        self.declare_parameter("angular_noise_std", 0.02)  # rad/s
        self.declare_parameter("scale", [1.02, 0.98, 1.01])  # vx, vy, wz
        self.declare_parameter("seed", 1)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._linear_std = float(self.get_parameter("linear_noise_std").value)
        self._angular_std = float(self.get_parameter("angular_noise_std").value)
        self._scale = [float(v) for v in self.get_parameter("scale").value]
        if len(self._scale) != 3:
            raise ValueError("scale must have three entries: vx, vy, wz")
        self._rng = random.Random(int(self.get_parameter("seed").value))
        self._publisher = self.create_publisher(
            TwistStamped, str(self.get_parameter("output_topic").value), qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("input_topic").value),
            self._odom_callback,
            qos_profile_sensor_data,
        )

    def _odom_callback(self, odom: Odometry) -> None:
        # Ground-truth twist is already expressed in the chassis body frame.
        out = TwistStamped()
        out.header.stamp = odom.header.stamp
        out.header.frame_id = self._frame_id
        twist = odom.twist.twist
        out.twist.linear.x = self._scale[0] * twist.linear.x + self._rng.gauss(0.0, self._linear_std)
        out.twist.linear.y = self._scale[1] * twist.linear.y + self._rng.gauss(0.0, self._linear_std)
        out.twist.angular.z = self._scale[2] * twist.angular.z + self._rng.gauss(0.0, self._angular_std)
        self._publisher.publish(out)


def main() -> None:
    rclpy.init()
    node = SimWheelOdometry()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
