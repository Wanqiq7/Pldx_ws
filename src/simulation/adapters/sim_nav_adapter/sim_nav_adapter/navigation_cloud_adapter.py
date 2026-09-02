#!/usr/bin/env python3
"""Transform Point-LIO registered clouds into the planar navigation frame."""

from collections import deque
from functools import partial

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


def remove_trailing_point_padding(message):
    """Repack records when tf2_sensor_msgs cannot serialize trailing padding."""
    packed_dtype = point_cloud2.dtype_from_fields(message.fields)
    if message.point_step == packed_dtype.itemsize:
        return message

    source_points = point_cloud2.read_points(message)
    packed_points = np.empty(source_points.shape, dtype=packed_dtype)
    for field_name in packed_dtype.names:
        packed_points[field_name] = source_points[field_name]
    packed = point_cloud2.create_cloud(message.header, message.fields, packed_points)
    packed.is_dense = message.is_dense
    return packed


class NavigationCloudAdapter(Node):
    def __init__(self):
        super().__init__("navigation_cloud_adapter")
        self.declare_parameter("input_frame", "camera_init")
        self.declare_parameter("output_frame", "sim_odom")
        self.declare_parameter("registered_input_topic", "/cloud_registered")
        self.declare_parameter(
            "registered_output_topic", "/simulation/cloud_registered_nav"
        )
        self.declare_parameter("registered_full_input_topic", "/cloud_registered_full")
        self.declare_parameter(
            "registered_full_output_topic",
            "/simulation/cloud_registered_full_nav",
        )
        self.declare_parameter("transform_timeout_sec", 0.05)
        self.declare_parameter("warning_period_sec", 2.0)
        self.declare_parameter("readiness_frame", "front_mid360")
        self.declare_parameter("pending_queue_size", 8)
        self.declare_parameter("retry_period_sec", 0.005)

        self.input_frame = self.get_parameter("input_frame").value
        self.output_frame = self.get_parameter("output_frame").value
        self.readiness_frame = self.get_parameter("readiness_frame").value
        self.pending_queue_size = int(self.get_parameter("pending_queue_size").value)
        if self.pending_queue_size < 1:
            raise ValueError("pending_queue_size must be at least 1")
        self.pending_clouds = deque()
        self.transform_timeout = Duration(
            seconds=float(self.get_parameter("transform_timeout_sec").value)
        )
        self.warning_period_ns = int(
            float(self.get_parameter("warning_period_sec").value) * 1.0e9
        )
        self.last_warning_ns = {}

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cloud_subscriptions = []
        routes = (
            (
                self.get_parameter("registered_input_topic").value,
                self.get_parameter("registered_output_topic").value,
            ),
            (
                self.get_parameter("registered_full_input_topic").value,
                self.get_parameter("registered_full_output_topic").value,
            ),
        )
        for input_topic, output_topic in routes:
            publisher = self.create_publisher(
                PointCloud2, output_topic, qos_profile_sensor_data
            )
            subscription = self.create_subscription(
                PointCloud2,
                input_topic,
                partial(self._cloud_callback, publisher=publisher),
                qos_profile_sensor_data,
            )
            self.cloud_subscriptions.append(subscription)
        self.retry_timer = self.create_timer(
            float(self.get_parameter("retry_period_sec").value),
            self._publish_ready_clouds,
        )

        self.get_logger().info(
            f"Transforming Point-LIO clouds {self.input_frame} -> {self.output_frame}: "
            + ", ".join(f"{source} -> {target}" for source, target in routes)
            + f"; waiting for {self.output_frame} <- {self.readiness_frame} "
            "before publication"
        )

    def _warn_throttled(self, key, message):
        now_ns = self.get_clock().now().nanoseconds
        last_ns = self.last_warning_ns.get(key)
        if (
            last_ns is None
            or now_ns < last_ns
            or now_ns - last_ns >= self.warning_period_ns
        ):
            self.get_logger().warning(message)
            self.last_warning_ns[key] = now_ns

    def _cloud_callback(self, message, publisher):
        if message.header.frame_id != self.input_frame:
            self._warn_throttled(
                "input_frame",
                f"Ignoring cloud in frame '{message.header.frame_id}', "
                f"expected '{self.input_frame}'",
            )
            return

        stamp = Time.from_msg(message.header.stamp)
        if not self.tf_buffer.can_transform(
            self.output_frame,
            self.readiness_frame,
            stamp,
        ):
            if len(self.pending_clouds) >= self.pending_queue_size:
                self.pending_clouds.popleft()
                self._warn_throttled(
                    "readiness_queue",
                    f"Dropping oldest cloud while waiting for {self.output_frame} <- "
                    f"{self.readiness_frame}; queue limit={self.pending_queue_size}",
                )
            self.pending_clouds.append((message, publisher))
            return

        self._transform_and_publish(message, publisher)

    def _publish_ready_clouds(self):
        while self.pending_clouds:
            message, publisher = self.pending_clouds[0]
            if not self.tf_buffer.can_transform(
                self.output_frame,
                self.readiness_frame,
                Time.from_msg(message.header.stamp),
            ):
                return
            self.pending_clouds.popleft()
            self._transform_and_publish(message, publisher)

    def _transform_and_publish(self, message, publisher):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.output_frame,
                self.input_frame,
                Time.from_msg(message.header.stamp),
                timeout=self.transform_timeout,
            )
            output = do_transform_cloud(
                remove_trailing_point_padding(message), transform
            )
        except (TransformException, AssertionError, ValueError) as error:
            self._warn_throttled(
                "transform",
                f"Dropping cloud because {self.output_frame} <- {self.input_frame} "
                f"is unavailable or invalid: {error}",
            )
            return

        output.header.frame_id = self.output_frame
        output.is_dense = message.is_dense
        publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationCloudAdapter()
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
