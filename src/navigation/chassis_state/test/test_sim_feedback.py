"""Simulation feedback retains the measured joint sample and timestamp."""
import os
os.environ['ROS_DOMAIN_ID'] = str(110 + os.getpid() % 80)
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
import signal
import subprocess
import time
import unittest
import rclpy
from sensor_msgs.msg import JointState
from ros_interfaces.msg import YawJointState


class SimulationFeedbackTest(unittest.TestCase):
    def test_measured_joint_velocity_and_time_reach_contract(self):
        rclpy.init()
        node = rclpy.create_node('test_sim_feedback')
        pub = node.create_publisher(JointState, '/sim/gimbal/joint_states', 10)
        values = []
        node.create_subscription(YawJointState, '/sentry/yaw_joint_state', values.append, 10)
        process = subprocess.Popen(['ros2','run','chassis_state','simulation_feedback'], start_new_session=True)
        try:
            msg = JointState()
            msg.header.stamp.sec, msg.header.stamp.nanosec = 123, 456
            msg.name, msg.position, msg.velocity = ['360_ref_ref1_frame'], [.3], [.7]
            end = time.monotonic() + 8
            while not values and time.monotonic() < end:
                pub.publish(msg)
                rclpy.spin_once(node, timeout_sec=.05)
            self.assertTrue(values)
            self.assertEqual(values[-1].header.stamp, msg.header.stamp)
            self.assertEqual(values[-1].angle, .3)
            self.assertEqual(values[-1].velocity, .7)
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
