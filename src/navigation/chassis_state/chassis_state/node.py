"""ROS adapter for the physical chassis state contract in ADR-0003."""
from collections import deque

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from ros_interfaces.msg import YawJointState
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from .alignment import JointBuffer
from .kinematics import Geometry, recover


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def vector(msg):
    return np.array([msg.x, msg.y, msg.z])


def set_vector(msg, value):
    msg.x, msg.y, msg.z = map(float, value)


def set_rotation(msg, value):
    msg.x, msg.y, msg.z, msg.w = map(float, value.as_quat())


class ChassisStateNode(Node):
    def __init__(self):
        super().__init__('chassis_state')
        param = lambda name, default: self.declare_parameter(name, default).value
        self.world = param('world_frame', 'camera_init')
        self.base = param('base_frame', 'base_link')
        self.mount = param('mount_frame', 'lidar_yaw_mount')
        self.body = param('body_frame', 'body')
        self.fake = param('fake_frame', 'base_link_fake')
        self.geometry = Geometry(
            pivot=np.array(param('pivot_xyz', [0., 0., 0.])),
            axis_rotation=Rotation.from_euler('xyz', param('axis_rpy', [0., 0., 0.])),
            imu_translation=np.array(param('mount_to_imu_xyz', [0., 0., 0.])),
            imu_rotation=Rotation.from_euler('xyz', param('mount_to_imu_rpy', [0., 0., 0.])),
            joint_zero=param('joint_zero', 0.),
            joint_direction=param('joint_direction', 1.))
        if self.geometry.joint_direction not in (-1., 1.):
            raise ValueError('joint_direction must be +1 or -1')
        self.publish_tf = param('publish_tf', True)
        self.joints = JointBuffer()
        self.pending = deque(maxlen=2000)
        self.previous = None
        self.last_lio_stamp = None
        self.odom_pub = self.create_publisher(Odometry, param('odom_topic', '/chassis/odom'), 20)
        self.difference_pub = self.create_publisher(TwistStamped, '/chassis/pose_difference', 20)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        if self.publish_tf:
            self.static_tf.sendTransform(self.transform(
                self.get_clock().now().to_msg(), self.mount, self.body,
                self.geometry.imu_translation, self.geometry.imu_rotation))
        self.create_subscription(YawJointState, param('joint_topic', '/sentry/yaw_joint_state'),
                                 self.on_joint, 200)
        self.create_subscription(Odometry, param('lio_topic', '/aft_mapped_to_init'),
                                 self.on_lio, qos_profile_sensor_data)

    def transform(self, stamp, parent, child, translation, rotation):
        msg = TransformStamped()
        msg.header.stamp, msg.header.frame_id, msg.child_frame_id = stamp, parent, child
        set_vector(msg.transform.translation, translation)
        set_rotation(msg.transform.rotation, rotation)
        return msg

    def on_joint(self, msg):
        if msg.header.frame_id != self.base or msg.child_frame_id != self.mount:
            self.get_logger().error('YawJointState frames do not match configured geometry',
                                    throttle_duration_sec=5.)
            return
        self.joints.add(seconds(msg.header.stamp), msg.angle, msg.velocity)
        self.drain()

    def on_lio(self, msg):
        if msg.header.frame_id != self.world or msg.child_frame_id != self.body:
            self.get_logger().error('LIO Odometry frames do not match configured geometry',
                                    throttle_duration_sec=5.)
            return
        stamp = seconds(msg.header.stamp)
        if self.last_lio_stamp is not None and stamp < self.last_lio_stamp - 1.:
            self.pending.clear()
            self.joints.samples.clear()
            self.previous = None
        self.last_lio_stamp = stamp
        self.pending.append(msg)
        self.drain()

    def drain(self):
        remaining = deque(maxlen=self.pending.maxlen)
        while self.pending:
            msg = self.pending.popleft()
            stamp = seconds(msg.header.stamp)
            joint = self.joints.at(stamp)
            if joint is None:
                if not self.joints.samples or stamp >= self.joints.samples[0][0]:
                    remaining.append(msg)
                continue
            try:
                self.publish(msg, joint)
            except ValueError as error:
                self.get_logger().error(str(error), throttle_duration_sec=5.)
        self.pending = remaining

    def publish(self, msg, joint):
        pose, twist = msg.pose.pose, msg.twist.twist
        q = pose.orientation
        rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])
        state = recover(self.geometry, vector(pose.position), rotation,
                        vector(twist.linear), vector(twist.angular), *joint)
        if not all(np.isfinite(value).all() for value in
                   (state.position, state.linear, state.angular)):
            raise ValueError('Non-finite LIO state')
        output = Odometry()
        output.header = msg.header
        output.child_frame_id = self.base
        set_vector(output.pose.pose.position, state.position)
        set_rotation(output.pose.pose.orientation, state.rotation)
        set_vector(output.twist.twist.linear, state.linear)
        set_vector(output.twist.twist.angular, state.angular)
        self.odom_pub.publish(output)
        if self.publish_tf:
            # The virtual frame shares the physical reference point, with identity world rotation.
            self.tf.sendTransform([
                self.transform(msg.header.stamp, self.world, self.base,
                               state.position, state.rotation),
                self.transform(msg.header.stamp, self.base, self.mount,
                               self.geometry.pivot, self.geometry.mount_rotation(joint[0])),
                self.transform(msg.header.stamp, self.base, self.fake,
                               np.zeros(3), state.rotation.inv())])
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if self.previous is not None:
            previous_stamp, previous_state = self.previous
            dt = (stamp - previous_stamp) * 1e-9
            if 0. < dt < 1.:
                difference = TwistStamped()
                difference.header.frame_id = self.world
                midpoint = (stamp + previous_stamp) // 2
                difference.header.stamp = rclpy.time.Time(nanoseconds=midpoint).to_msg()
                set_vector(difference.twist.linear, (state.position - previous_state.position) / dt)
                set_vector(difference.twist.angular,
                           (state.rotation * previous_state.rotation.inv()).as_rotvec() / dt)
                self.difference_pub.publish(difference)
        if self.previous is None or stamp > self.previous[0]:
            self.previous = stamp, state


def main(args=None):
    rclpy.init(args=args)
    node = ChassisStateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
