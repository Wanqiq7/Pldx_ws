#!/usr/bin/env python3
"""Expose Gazebo base odometry using the navigation workspace contract."""

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class GroundTruthOdom(Node):
    def __init__(self):
        super().__init__("ground_truth_odom")
        self.declare_parameter("input_topic", "robot_base/odom")
        self.declare_parameter("output_topic", "odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.publisher = self.create_publisher(
            Odometry, self.get_parameter("output_topic").value, 10
        )
        self.tf = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry,
            self.get_parameter("input_topic").value,
            self.callback,
            10,
        )

    def callback(self, message: Odometry):
        message.header.frame_id = self.odom_frame
        message.child_frame_id = self.base_frame
        self.publisher.publish(message)

        transform = TransformStamped()
        transform.header = message.header
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = message.pose.pose.position.x
        transform.transform.translation.y = message.pose.pose.position.y
        transform.transform.translation.z = message.pose.pose.position.z
        transform.transform.rotation = message.pose.pose.orientation
        self.tf.sendTransform(transform)


def main():
    rclpy.init()
    node = GroundTruthOdom()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
