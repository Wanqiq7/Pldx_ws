#!/usr/bin/env python3
"""Adapt Point-LIO tracking odometry to the navigation base-frame contract."""

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import (
    Buffer,
    StaticTransformBroadcaster,
    TransformBroadcaster,
    TransformException,
    TransformListener,
)

from sim_nav_adapter.transform_math import (
    compute_map_to_base,
    compute_map_to_camera,
    compose_transform,
    inverse_transform,
    project_to_navigation_plane,
    project_twist_covariance_to_navigation_plane,
    quaternion_from_yaw,
    transform_twist,
    transform_twist_covariance,
)


def _pose_to_transform(pose):
    return (
        (pose.position.x, pose.position.y, pose.position.z),
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ),
    )


def _message_to_transform(message):
    return (
        (
            message.transform.translation.x,
            message.transform.translation.y,
            message.transform.translation.z,
        ),
        (
            message.transform.rotation.x,
            message.transform.rotation.y,
            message.transform.rotation.z,
            message.transform.rotation.w,
        ),
    )


def _set_pose(pose, transform):
    translation, rotation = transform
    pose.position.x, pose.position.y, pose.position.z = translation
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (
        rotation
    )


def _make_transform(stamp, parent, child, transform):
    message = TransformStamped()
    message.header.stamp = stamp
    message.header.frame_id = parent
    message.child_frame_id = child
    translation, rotation = transform
    (
        message.transform.translation.x,
        message.transform.translation.y,
        message.transform.translation.z,
    ) = translation
    (
        message.transform.rotation.x,
        message.transform.rotation.y,
        message.transform.rotation.z,
        message.transform.rotation.w,
    ) = rotation
    return message


class PointLioOdomAdapter(Node):
    def __init__(self):
        super().__init__("point_lio_odom_adapter")
        self.declare_parameter("input_topic", "/aft_mapped_to_init")
        self.declare_parameter("output_topic", "/odom")
        self.declare_parameter("input_parent_frame", "camera_init")
        self.declare_parameter("input_child_frame", "body")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "sim_odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tracking_frame", "front_mid360")
        self.declare_parameter("spawn_x", 0.0)
        self.declare_parameter("spawn_y", 0.0)
        self.declare_parameter("spawn_yaw", 0.0)

        self.input_parent_frame = self.get_parameter("input_parent_frame").value
        self.input_child_frame = self.get_parameter("input_child_frame").value
        self.map_frame = self.get_parameter("map_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.tracking_frame = self.get_parameter("tracking_frame").value
        self.spawn_pose = (
            (
                float(self.get_parameter("spawn_x").value),
                float(self.get_parameter("spawn_y").value),
                0.0,
            ),
            quaternion_from_yaw(float(self.get_parameter("spawn_yaw").value)),
        )

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.dynamic_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.publisher = self.create_publisher(
            Odometry, self.get_parameter("output_topic").value, 10
        )
        self.subscription = self.create_subscription(
            Odometry,
            self.get_parameter("input_topic").value,
            self._odometry_callback,
            10,
        )
        self.base_to_tracking = None
        self.map_to_camera = None
        self.last_warning_ns = -2_000_000_000

    def _warn_throttled(self, message):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_warning_ns >= 2_000_000_000:
            self.get_logger().warning(message)
            self.last_warning_ns = now_ns

    def _initialize_alignment(self, stamp, camera_to_tracking_initial):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.tracking_frame,
                Time.from_msg(stamp),
                timeout=Duration(seconds=0.1),
            )
        except TransformException as error:
            self._warn_throttled(
                f"Waiting for {self.base_frame} -> {self.tracking_frame}: {error}"
            )
            return False

        self.base_to_tracking = _message_to_transform(transform)
        self.map_to_camera = compute_map_to_camera(
            self.spawn_pose,
            self.base_to_tracking,
            camera_to_tracking_initial,
        )
        self.static_broadcaster.sendTransform(
            [
                _make_transform(
                    stamp,
                    self.map_frame,
                    self.input_parent_frame,
                    self.map_to_camera,
                ),
                _make_transform(
                    stamp, self.map_frame, self.odom_frame, self.spawn_pose
                ),
            ]
        )
        self.get_logger().info(
            "Initialized full map -> camera_init alignment and planar map -> "
            f"{self.odom_frame} spawn transform"
        )
        return True

    def _odometry_callback(self, message):
        if message.header.stamp.sec == 0 and message.header.stamp.nanosec == 0:
            self._warn_throttled("Ignoring Point-LIO odometry with a zero timestamp")
            return
        if message.header.frame_id != self.input_parent_frame:
            self._warn_throttled(
                f"Ignoring Point-LIO parent frame '{message.header.frame_id}', "
                f"expected '{self.input_parent_frame}'"
            )
            return
        if message.child_frame_id != self.input_child_frame:
            self._warn_throttled(
                f"Ignoring Point-LIO child frame '{message.child_frame_id}', "
                f"expected '{self.input_child_frame}'"
            )
            return
        camera_to_tracking = _pose_to_transform(message.pose.pose)
        if self.map_to_camera is None and not self._initialize_alignment(
            message.header.stamp, camera_to_tracking
        ):
            return

        map_to_base = compute_map_to_base(
            self.map_to_camera,
            camera_to_tracking,
            self.base_to_tracking,
        )
        map_to_base_navigation = project_to_navigation_plane(map_to_base)
        odom_to_base = project_to_navigation_plane(
            compose_transform(
                inverse_transform(self.spawn_pose),
                map_to_base_navigation,
            )
        )
        input_linear = (
            message.twist.twist.linear.x,
            message.twist.twist.linear.y,
            message.twist.twist.linear.z,
        )
        input_angular = (
            message.twist.twist.angular.x,
            message.twist.twist.angular.y,
            message.twist.twist.angular.z,
        )
        output_linear, output_angular = transform_twist(
            input_linear, input_angular, self.base_to_tracking
        )
        output_linear = (output_linear[0], output_linear[1], 0.0)
        output_angular = (0.0, 0.0, output_angular[2])

        output = Odometry()
        output.header.stamp = message.header.stamp
        output.header.frame_id = self.odom_frame
        output.child_frame_id = self.base_frame
        _set_pose(output.pose.pose, odom_to_base)
        output.pose.covariance = message.pose.covariance
        (
            output.twist.twist.linear.x,
            output.twist.twist.linear.y,
            output.twist.twist.linear.z,
        ) = output_linear
        (
            output.twist.twist.angular.x,
            output.twist.twist.angular.y,
            output.twist.twist.angular.z,
        ) = output_angular
        output.twist.covariance = project_twist_covariance_to_navigation_plane(
            transform_twist_covariance(
                message.twist.covariance,
                self.base_to_tracking,
            )
        )

        self.publisher.publish(output)
        self.dynamic_broadcaster.sendTransform(
            [
                _make_transform(
                    output.header.stamp,
                    self.input_parent_frame,
                    self.input_child_frame,
                    camera_to_tracking,
                ),
                _make_transform(
                    output.header.stamp,
                    self.odom_frame,
                    self.base_frame,
                    odom_to_base,
                ),
            ]
        )


def main():
    rclpy.init()
    node = PointLioOdomAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
