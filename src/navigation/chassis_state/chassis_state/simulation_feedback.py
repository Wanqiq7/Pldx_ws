"""Adapt measured simulator joints, or an explicitly locked installation, to #6."""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from ros_interfaces.msg import YawJointState


class SimulationFeedback(Node):
    def __init__(self):
        super().__init__('simulation_yaw_feedback')
        self.locked = self.declare_parameter('joint_locked', False).value
        self.name = self.declare_parameter('joint_name', '360_ref_ref1_frame').value
        self.publisher = self.create_publisher(YawJointState, '/sentry/yaw_joint_state', 200)
        if self.locked:
            self.create_subscription(Odometry, '/aft_mapped_to_init', self.on_locked,
                                     qos_profile_sensor_data)
        else:
            self.create_subscription(JointState, '/sim/gimbal/joint_states', self.on_joint,
                                     qos_profile_sensor_data)

    def publish(self, stamp, angle, velocity):
        if not math.isfinite(angle) or not math.isfinite(velocity):
            return
        msg = YawJointState()
        msg.header.stamp = stamp
        msg.header.frame_id = 'base_link'
        msg.child_frame_id = 'lidar_yaw_mount'
        msg.angle = math.remainder(angle, 2 * math.pi)
        msg.velocity = velocity
        # No MCU clock exists in simulation; diagnostics remain zero.
        self.publisher.publish(msg)

    def on_joint(self, msg):
        if self.name not in msg.name:
            return
        index = msg.name.index(self.name)
        if index < len(msg.position) and index < len(msg.velocity):
            self.publish(msg.header.stamp, msg.position[index], msg.velocity[index])

    def on_locked(self, msg):
        # Zero is a known rigid-installation constraint, not a sensor estimate.
        self.publish(msg.header.stamp, 0., 0.)


def main():
    rclpy.init()
    node = SimulationFeedback()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
