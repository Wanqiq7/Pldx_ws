"""Public-topic replay of all four chassis scenarios in a private ROS domain."""
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = str(110 + os.getpid() % 80)
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from ros_interfaces.msg import YawJointState
from scipy.spatial.transform import Rotation as R
from tf2_msgs.msg import TFMessage
import yaml


class ReplayTest(unittest.TestCase):
    def test_four_scenarios_and_unique_physical_tf(self):
        rclpy.init()
        node = rclpy.create_node('chassis_replay')
        odom_pub = node.create_publisher(Odometry, '/aft_mapped_to_init', 50)
        joint_pub = node.create_publisher(YawJointState, '/sentry/yaw_joint_state', 200)
        observed, differences, transforms = {}, [], {}
        def receive(msg):
            observed[msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec] = msg
        node.create_subscription(Odometry, '/chassis/odom', receive, 200)
        node.create_subscription(TwistStamped, '/chassis/pose_difference', differences.append, 200)
        def receive_tf(msg):
            for transform in msg.transforms:
                transforms[transform.child_frame_id] = transform
        node.create_subscription(TFMessage, '/tf', receive_tf, 200)
        geometry = dict(pivot_xyz=[.2, -.1, .3], axis_rpy=[.1, -.2, .3],
                        mount_to_imu_xyz=[.12, -.24, .08],
                        mount_to_imu_rpy=[.4, -.3, .2], joint_zero=.17,
                        joint_direction=-1.)
        report = []
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp) / 'geometry.yaml'
            config.write_text(yaml.safe_dump({'chassis_state': {'ros__parameters': geometry}}))
            process = subprocess.Popen(['ros2', 'run', 'chassis_state', 'chassis_state',
                                        '--ros-args', '--params-file', str(config)])
            try:
                deadline = time.monotonic() + 15
                while (odom_pub.get_subscription_count() == 0 or joint_pub.get_subscription_count() == 0
                       or node.count_publishers('/chassis/odom') == 0):
                    self.assertIsNone(process.poll())
                    self.assertLess(time.monotonic(), deadline, 'node discovery timeout')
                    rclpy.spin_once(node, timeout_sec=.05)
                # Complete DDS writer/reader matching before the first one-shot sample.
                ready = time.monotonic() + .5
                while time.monotonic() < ready:
                    rclpy.spin_once(node, timeout_sec=.01)
                t0 = node.get_clock().now().nanoseconds
                for scenario, base_rate, joint_rate, vx in [
                    ('stationary_sweep', 0., 1.2, 0.), ('locked_joint_rotation', .8, 0., 0.),
                    ('simultaneous_rotation', .8, 1.2, 0.), ('world_translation', .8, 1.2, .3)]:
                    errors = []
                    for k in range(35):
                        t = k * .02
                        stamp = rclpy.time.Time(nanoseconds=t0 + k * 20_000_000).to_msg()
                        base = R.from_euler('xyz', [.13, -.11, base_rate * t])
                        q = 3.05 + joint_rate * t
                        joint = YawJointState()
                        joint.header.stamp, joint.header.frame_id = stamp, 'base_link'
                        joint.child_frame_id = 'lidar_yaw_mount'
                        joint.angle = (q + np.pi) % (2 * np.pi) - np.pi
                        joint.velocity = joint_rate
                        joint_pub.publish(joint)
                        def pose(at):
                            b = R.from_euler('xyz', [.13, -.11, base_rate * at])
                            y = R.from_euler('xyz', geometry['axis_rpy']) * R.from_euler(
                                'z', -(3.05 + joint_rate * at) + .17)
                            p = np.array([vx * at, 0., 0.])
                            return (p + b.apply(geometry['pivot_xyz'])
                                    + (b * y).apply(geometry['mount_to_imu_xyz']),
                                    b * y * R.from_euler('xyz', geometry['mount_to_imu_rpy']))
                        p, rotation = pose(t)
                        pm, rm = pose(t - 1e-5)
                        pp, rp = pose(t + 1e-5)
                        msg = Odometry()
                        msg.header.stamp, msg.header.frame_id = stamp, 'camera_init'
                        msg.child_frame_id = 'body'
                        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = p
                        (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                         msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) = rotation.as_quat()
                        velocity = rotation.inv().apply((pp - pm) / 2e-5)
                        omega = rotation.inv().apply((rp * rm.inv()).as_rotvec() / 2e-5)
                        (msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                         msg.twist.twist.linear.z) = velocity
                        (msg.twist.twist.angular.x, msg.twist.twist.angular.y,
                         msg.twist.twist.angular.z) = omega
                        odom_pub.publish(msg)
                        key = stamp.sec * 10**9 + stamp.nanosec
                        deadline = time.monotonic() + 3
                        while key not in observed and time.monotonic() < deadline:
                            rclpy.spin_once(node, timeout_sec=.01)
                        self.assertIn(key, observed)
                        output = observed[key]
                        self.assertEqual(output.header.frame_id, 'camera_init')
                        self.assertEqual(output.child_frame_id, 'base_link')
                        actual_p = output.pose.pose.position
                        actual_q = output.pose.pose.orientation
                        actual_r = R.from_quat([actual_q.x, actual_q.y, actual_q.z, actual_q.w])
                        actual_v, actual_w = output.twist.twist.linear, output.twist.twist.angular
                        error = max(np.max(np.abs(np.array([actual_p.x, actual_p.y, actual_p.z]) - [vx*t,0,0])),
                                    np.linalg.norm((actual_r * base.inv()).as_rotvec()),
                                    np.max(np.abs(base.apply([actual_v.x,actual_v.y,actual_v.z]) - [vx,0,0])),
                                    np.max(np.abs(base.apply([actual_w.x,actual_w.y,actual_w.z]) - [0,0,base_rate])))
                        self.assertLess(error, 1e-7)
                        errors.append(float(error))
                    report.append({'scenario': scenario, 'samples': len(errors), 'max_error': max(errors)})
                    t0 += 2_000_000_000
                deadline = time.monotonic() + 3
                while 'base_link_fake' not in transforms and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=.01)
                self.assertEqual({child: tr.header.frame_id for child, tr in transforms.items()},
                                 {'base_link': 'camera_init', 'lidar_yaw_mount': 'base_link',
                                  'base_link_fake': 'base_link'})
                physical = transforms['base_link'].transform.rotation
                virtual = transforms['base_link_fake'].transform.rotation
                identity = R.from_quat([physical.x,physical.y,physical.z,physical.w]) * R.from_quat(
                    [virtual.x,virtual.y,virtual.z,virtual.w])
                self.assertLess(np.linalg.norm(identity.as_rotvec()), 1e-9)
                self.assertEqual(node.count_publishers('/chassis/odom'), 1)
                self.assertEqual(node.count_publishers('/tf'), 1)
                self.assertGreater(len(differences), 120)
                # Last scenario has constant world velocities, independent of tilted rotating IMU.
                diff = differences[-1].twist
                np.testing.assert_allclose([diff.linear.x,diff.linear.y,diff.linear.z], [.3,0,0], atol=1e-7)
                np.testing.assert_allclose([diff.angular.x,diff.angular.y,diff.angular.z], [0,0,.8], atol=1e-7)
                print('CHASSIS_REPLAY_REPORT=' + json.dumps(report))
            finally:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
                node.destroy_node()
                rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
