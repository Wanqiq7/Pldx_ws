#!/usr/bin/env python3
"""Exercise the built TF publisher with synthetic LIO and gimbal feedback."""
import math
import os
import subprocess
import time

os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
os.environ['ROS_LOCALHOST_ONLY'] = '1'

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped
from ros_interfaces.msg import SentryInfoOffline
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))


def near(actual, expected):
    assert abs(actual - expected) < 1e-5, (actual, expected)


def main():
    rclpy.init()
    node = rclpy.create_node('check_chassis_tf')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    broadcaster = TransformBroadcaster(node)
    feedback = node.create_publisher(SentryInfoOffline, '/sentry/offline_info', 10)
    executable = os.path.join(get_package_prefix('fake_vel_transform'),
                              'lib', 'fake_vel_transform', 'fake_vel_transform_node')
    process = subprocess.Popen([executable], stdout=subprocess.DEVNULL)

    def check(chassis_yaw, joint_yaw=None):
        deadline = time.monotonic() + 8
        last_error = None
        while time.monotonic() < deadline:
            assert process.poll() is None, 'TF publisher exited'
            transform = TransformStamped()
            transform.header.stamp = node.get_clock().now().to_msg()
            transform.header.frame_id = 'camera_init'
            transform.child_frame_id = 'body'
            transform.transform.translation.x = 2.0
            transform.transform.translation.y = 3.0
            transform.transform.rotation.z = math.sin(chassis_yaw / 2)
            transform.transform.rotation.w = math.cos(chassis_yaw / 2)
            broadcaster.sendTransform(transform)
            if joint_yaw is not None:
                msg = SentryInfoOffline()
                msg.yaw_camerainit_to_gimbal = float(joint_yaw)
                feedback.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.02)
            try:
                fixed = buffer.lookup_transform('body', 'base_link', rclpy.time.Time())
                near(fixed.transform.translation.y, 0.20)
                near(fixed.transform.translation.z, 0.0)
                near(yaw(fixed.transform.rotation), 0.0)
                base = buffer.lookup_transform('camera_init', 'base_link', rclpy.time.Time())
                near(base.transform.translation.x, 2 - 0.20 * math.sin(chassis_yaw))
                near(base.transform.translation.y, 3 + 0.20 * math.cos(chassis_yaw))
                fake = buffer.lookup_transform('camera_init', 'base_link_fake', rclpy.time.Time())
                near(yaw(fake.transform.rotation), 0.0)
                if joint_yaw is not None:
                    gimbal = buffer.lookup_transform('base_link', 'vision_gimbal', rclpy.time.Time())
                    near(yaw(gimbal.transform.rotation), math.radians(joint_yaw))
                    near(gimbal.transform.translation.z, 0.5)
                return
            except (TransformException, AssertionError) as error:
                last_error = error
        raise AssertionError(f'TF did not converge: {last_error}')

    try:
        check(0.0)  # Navigation works without offline/gimbal messages.
        assert not buffer.can_transform('base_link', 'vision_gimbal', rclpy.time.Time())
        for chassis_yaw in (0.0, math.pi / 2, -math.pi / 2):
            for joint_yaw in (0, 90, -90):
                check(chassis_yaw, joint_yaw)
        check(math.pi)  # Navigation continues without fresh gimbal messages.
        print('PASS: static lever arm, yaw cancellation, gimbal independence, visual TF')
    finally:
        process.terminate()
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
