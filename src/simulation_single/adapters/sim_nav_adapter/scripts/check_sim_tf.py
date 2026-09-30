#!/usr/bin/env python3
"""Runtime acceptance check for the simulation navigation TF contract."""

import math
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
import yaml


class SimulationTfCheck(Node):
    def __init__(self):
        super().__init__("check_sim_tf")
        self.declare_parameter("timeout_sec", 20.0)
        self.declare_parameter("ground_truth_xy_tolerance", 0.10)
        self.declare_parameter("ground_truth_yaw_tolerance", 0.05)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.ground_truth = None
        self.ground_truth_subscription = self.create_subscription(
            Odometry,
            "/ground_truth/odom",
            self._ground_truth_callback,
            10,
        )

    def _ground_truth_callback(self, message):
        self.ground_truth = message

    def run(self):
        required_transforms = (
            ("map", "camera_init"),
            ("camera_init", "body"),
            ("map", "sim_odom"),
            ("sim_odom", "base_link"),
            ("base_link", "base_footprint"),
            ("base_footprint", "chassis"),
            ("base_link", "front_mid360"),
        )
        timeout_sec = float(self.get_parameter("timeout_sec").value)
        deadline = time.monotonic() + timeout_sec
        missing = list(required_transforms)
        invalid_topic_publishers = {
            "/clock": 0,
            "/livox/imu": 0,
        }

        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            missing = [
                frames
                for frames in required_transforms
                if not self.buffer.can_transform(
                    frames[0], frames[1], Time(), timeout=Duration()
                )
            ]
            invalid_topic_publishers = {
                topic: len(self.get_publishers_info_by_topic(topic))
                for topic in invalid_topic_publishers
            }
            odom_publishers = self.get_publishers_info_by_topic("/odom")
            if (
                not missing
                and len(odom_publishers) == 1
                and all(count == 1 for count in invalid_topic_publishers.values())
                and self.ground_truth is not None
            ):
                if not self._check_level_transforms():
                    time.sleep(0.2)
                    continue
                if not self._check_tf_topology():
                    time.sleep(0.2)
                    continue
                if not self._check_ground_truth_error():
                    time.sleep(0.2)
                    continue
                publisher = odom_publishers[0]
                owner = f"{publisher.node_namespace.rstrip('/')}/{publisher.node_name}"
                self.get_logger().info(
                    "PASS: TF tree is connected and /odom is owned by " f"{owner}"
                )
                return True

        if missing:
            self.get_logger().error(
                "Missing transforms: "
                + ", ".join(f"{parent} -> {child}" for parent, child in missing)
            )
        odom_publishers = self.get_publishers_info_by_topic("/odom")
        if len(odom_publishers) != 1:
            owners = [
                f"{publisher.node_namespace.rstrip('/')}/{publisher.node_name}"
                for publisher in odom_publishers
            ]
            self.get_logger().error(
                f"Expected one /odom publisher, found {len(owners)}: {owners}"
            )
        for topic, count in invalid_topic_publishers.items():
            if count != 1:
                self.get_logger().error(
                    f"Expected one {topic} publisher, found {count}; "
                    "duplicate simulation instances can make time non-monotonic"
                )
        if self.ground_truth is None:
            self.get_logger().error("No /ground_truth/odom message received")
        return False

    def _check_level_transforms(self):
        for parent, child in (("map", "sim_odom"), ("sim_odom", "base_link")):
            try:
                transform = self.buffer.lookup_transform(parent, child, Time())
            except Exception as error:  # tf2 can raise several TransformException types
                self.get_logger().error(
                    f"Unable to inspect {parent} -> {child}: {error}"
                )
                return False

            rotation = transform.transform.rotation
            x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
            roll = math.atan2(
                2.0 * (w * x + y * z),
                1.0 - 2.0 * (x * x + y * y),
            )
            sin_pitch = 2.0 * (w * y - z * x)
            pitch = (
                math.copysign(math.pi / 2.0, sin_pitch)
                if abs(sin_pitch) >= 1.0
                else math.asin(sin_pitch)
            )
            if abs(roll) > 1.0e-3 or abs(pitch) > 1.0e-3:
                self.get_logger().error(
                    f"Tilted navigation transform {parent} -> {child}: "
                    f"roll={roll:.6f}, pitch={pitch:.6f}"
                )
                return False
        return True

    def _check_tf_topology(self):
        frames = yaml.safe_load(self.buffer.all_frames_as_yaml()) or {}
        expected_parents = {
            "camera_init": "map",
            "body": "camera_init",
            "sim_odom": "map",
            "base_link": "sim_odom",
        }
        for child, expected_parent in expected_parents.items():
            actual_parent = frames.get(child, {}).get("parent")
            if actual_parent != expected_parent:
                self.get_logger().error(
                    f"Expected direct TF {expected_parent} -> {child}, "
                    f"found parent '{actual_parent}'"
                )
                return False

        for child in frames:
            visited = set()
            current = child
            while current in frames:
                if current in visited:
                    self.get_logger().error(f"TF cycle detected through '{current}'")
                    return False
                visited.add(current)
                current = frames[current].get("parent")
        return True

    @staticmethod
    def _yaw(rotation):
        return math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )

    def _check_ground_truth_error(self):
        try:
            estimate = self.buffer.lookup_transform("map", "base_link", Time())
        except Exception as error:
            self.get_logger().error(f"Unable to inspect map -> base_link: {error}")
            return False

        estimate_translation = estimate.transform.translation
        truth_pose = self.ground_truth.pose.pose
        xy_error = math.hypot(
            estimate_translation.x - truth_pose.position.x,
            estimate_translation.y - truth_pose.position.y,
        )
        yaw_error = abs(
            math.atan2(
                math.sin(
                    self._yaw(estimate.transform.rotation)
                    - self._yaw(truth_pose.orientation)
                ),
                math.cos(
                    self._yaw(estimate.transform.rotation)
                    - self._yaw(truth_pose.orientation)
                ),
            )
        )
        xy_tolerance = float(self.get_parameter("ground_truth_xy_tolerance").value)
        yaw_tolerance = float(self.get_parameter("ground_truth_yaw_tolerance").value)
        if xy_error >= xy_tolerance or yaw_error >= yaw_tolerance:
            self.get_logger().error(
                "Point-LIO navigation pose exceeds ground-truth tolerance: "
                f"xy={xy_error:.4f} m (limit {xy_tolerance:.4f}), "
                f"yaw={yaw_error:.4f} rad (limit {yaw_tolerance:.4f})"
            )
            return False
        return True


def main():
    rclpy.init()
    node = SimulationTfCheck()
    try:
        success = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()
