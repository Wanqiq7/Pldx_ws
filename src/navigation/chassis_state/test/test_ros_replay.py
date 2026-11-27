"""Public-topic replay of all four chassis scenarios in a private ROS domain."""
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time
import unittest

os.environ['ROS_DOMAIN_ID'] = str(110 + os.getpid() % 80)
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rosgraph_msgs.msg import Clock
from std_msgs.msg import UInt8MultiArray
from nav_msgs.msg import Odometry
from ros_interfaces.msg import YawJointState
from scipy.spatial.transform import Rotation as R
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
import yaml


class ReplayTest(unittest.TestCase):
    def test_four_scenarios_and_unique_physical_tf(self):
        rclpy.init()
        node = rclpy.create_node('chassis_replay')
        odom_pub = node.create_publisher(Odometry, '/aft_mapped_to_init', 50)
        joint_pub = node.create_publisher(YawJointState, '/sentry/yaw_joint_state', 200)
        observed, differences, transforms, packets = {}, [], {}, []
        command_pub = node.create_publisher(Twist, "/cmd_vel_mpc", 20)
        clock_pub = node.create_publisher(Clock, "/clock", 20)
        node.create_subscription(UInt8MultiArray, "/chassis/test/nav_packet", packets.append, 100)
        def receive(msg):
            observed[msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec] = msg
        node.create_subscription(Odometry, '/chassis/odom', receive, 200)
        node.create_subscription(TwistStamped, '/chassis/pose_difference', differences.append, 200)
        def receive_tf(msg):
            for transform in msg.transforms:
                transforms[transform.child_frame_id] = transform
        node.create_subscription(TFMessage, '/tf', receive_tf, 200)
        static = {}
        node.create_subscription(TFMessage, '/tf_static',
                                 lambda msg: static.update({t.child_frame_id: t for t in msg.transforms}),
                                 QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        geometry = dict(pivot_xyz=[.2, -.1, .3], axis_rpy=[.1, -.2, .3],
                        mount_to_imu_xyz=[.12, -.24, .08],
                        mount_to_imu_rpy=[.4, -.3, .2], joint_zero=.17,
                        joint_direction=-1., use_sim_time=True)
        report = []
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp) / 'geometry.yaml'
            config.write_text(yaml.safe_dump({'chassis_state': {'ros__parameters': geometry}}))
            process = subprocess.Popen(['ros2', 'run', 'chassis_state', 'chassis_state',
                                        '--ros-args', '--params-file', str(config)])
            capture = subprocess.Popen(['ros2', 'run', 'ros2_libxr', 'chassis_command_capture',
                                        '--ros-args', '-p', 'rotate_latency:=0.0',
                                        '-p', 'use_sim_time:=true'])
            try:
                deadline = time.monotonic() + 15
                while (odom_pub.get_subscription_count() == 0 or joint_pub.get_subscription_count() == 0
                       or node.count_publishers('/chassis/odom') == 0
                       or command_pub.get_subscription_count() == 0):
                    self.assertIsNone(process.poll())
                    self.assertLess(time.monotonic(), deadline, 'node discovery timeout')
                    rclpy.spin_once(node, timeout_sec=.05)
                # Complete DDS writer/reader matching before the first one-shot sample.
                ready = time.monotonic() + .5
                while time.monotonic() < ready:
                    rclpy.spin_once(node, timeout_sec=.01)
                t0 = 100_000_000_000
                for scenario, base_rate, joint_rate, vx in [
                    ('stationary_sweep', 0., 1.2, 0.), ('locked_joint_rotation', .8, 0., 0.),
                    ('simultaneous_rotation', .8, 1.2, 0.), ('world_translation', .8, 1.2, .3)]:
                    errors = []
                    for k in range(35):
                        t = k * .02
                        stamp = rclpy.time.Time(nanoseconds=t0 + k * 20_000_000).to_msg()
                        clock = Clock()
                        clock.clock = stamp
                        clock_pub.publish(clock)
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
                        # Exercise the production bridge with a fixed world direction.
                        packets.clear()
                        command = Twist()
                        command.linear.x = 1.0
                        command.angular.z = .2
                        command_pub.publish(command)
                        deadline = time.monotonic() + 1.
                        match = None
                        while time.monotonic() < deadline:
                            rclpy.spin_once(node, timeout_sec=.01)
                            for packet in packets:
                                if len(packet.data) == 30:
                                    x, y, w, valid = struct.unpack('<fffB', bytes(packet.data[16:29]))
                                    if valid and abs(w - .2) < 1e-6:
                                        match = x, y
                            if match is not None:
                                break
                        self.assertIsNotNone(match, 'no valid encoded chassis command')
                        np.testing.assert_allclose(match, [np.cos(base_rate*t), -np.sin(base_rate*t)],
                                                   atol=2e-6)

                    report.append({'scenario': scenario, 'samples': len(errors), 'max_error': max(errors)})
                    t0 += 2_000_000_000
                deadline = time.monotonic() + 3
                while (('base_link_fake' not in transforms
                        or transforms['base_link'].header.stamp != stamp)
                       and time.monotonic() < deadline):
                    rclpy.spin_once(node, timeout_sec=.01)
                self.assertEqual({child: tr.header.frame_id for child, tr in transforms.items()},
                                 {'base_link': 'camera_init', 'lidar_yaw_mount': 'base_link',
                                  'base_link_fake': 'base_link'})
                physical = transforms['base_link'].transform.rotation
                virtual = transforms['base_link_fake'].transform.rotation
                identity = R.from_quat([physical.x,physical.y,physical.z,physical.w]) * R.from_quat(
                    [virtual.x,virtual.y,virtual.z,virtual.w])
                self.assertLess(np.linalg.norm(identity.as_rotvec()), 1e-9)
                self.assertEqual(static['body'].header.frame_id, 'lidar_yaw_mount')
                self.assertEqual(static['vision_gimbal'].header.frame_id, 'lidar_yaw_mount')
                def matrix(transform):
                    q = transform.transform.rotation
                    p = transform.transform.translation
                    result = np.eye(4)
                    result[:3,:3] = R.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
                    result[:3,3] = [p.x,p.y,p.z]
                    return result
                reconstructed = (matrix(transforms['base_link'])
                                 @ matrix(transforms['lidar_yaw_mount']) @ matrix(static['body']))
                expected = np.eye(4)
                expected[:3,:3], expected[:3,3] = rotation.as_matrix(), p
                np.testing.assert_allclose(reconstructed, expected, atol=1e-9)
                self.assertEqual(node.count_publishers('/chassis/odom'), 1)
                self.assertEqual(node.count_publishers('/tf'), 1)
                self.assertGreater(len(differences), 120)
                # Last scenario has constant world velocities, independent of tilted rotating IMU.
                diff = differences[-1].twist
                np.testing.assert_allclose([diff.linear.x,diff.linear.y,diff.linear.z], [.3,0,0], atol=1e-7)
                np.testing.assert_allclose([diff.angular.x,diff.angular.y,diff.angular.z], [0,0,.8], atol=1e-7)
                print('CHASSIS_REPLAY_REPORT=' + json.dumps(report))
            finally:
                capture.send_signal(signal.SIGINT)
                try:
                    capture.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    capture.kill()
                    capture.wait()
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
