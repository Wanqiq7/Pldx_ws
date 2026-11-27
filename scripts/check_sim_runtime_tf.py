#!/usr/bin/env python3
"""Passively audit an existing simulation's TF, sensor timing and publishers.

Never launches Gazebo or sends commands. Source ROS/the workspace and select the
running simulation's ROS_DOMAIN_ID. Touch DIRECTORY/STOP to finish early.
"""
import argparse
from collections import Counter, deque
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
from ros_interfaces.msg import YawJointState
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener


def stamp(value):
    return value.sec + value.nanosec * 1e-9


def xyz(value):
    return np.array([value.x, value.y, value.z])


def rotation(value):
    return Rotation.from_quat([value.x, value.y, value.z, value.w])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--seconds', type=float, default=600)
    args = parser.parse_args()
    args.directory.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('runtime_tf_audit', parameter_overrides=[
        Parameter('use_sim_time', value=True)])
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    truth = deque(maxlen=1000)
    counts, last, publishers = Counter(), {}, {}
    errors, yaw_errors, ages = [], [], []
    tf_misses = 0
    def receive(topic):
        def callback(message):
            counts[topic] += 1
            last[topic] = message
            if topic == '/sim/ground_truth/odom':
                truth.append(message)
        return callback
    for topic, kind in [('/sim/ground_truth/odom', Odometry),
                        ('/chassis/odom', Odometry), ('/aft_mapped_to_init', Odometry),
                        ('/sim/imu', Imu), ('/cloud_registered_full', PointCloud2),
                        ('/sentry/yaw_joint_state', YawJointState), ('/sim/cmd_vel', Twist)]:
        node.create_subscription(kind, topic, receive(topic), qos_profile_sensor_data)
    def tf_received(message, info):
        # Jazzy's message info may omit the DDS writer GUID. Keep that
        # distinction explicit; graph endpoint counts cannot identify an edge.
        guid = info.get('publisher_gid')
        gid = bytes(guid).hex() if guid is not None else 'unavailable'
        for transform in message.transforms:
            edge = transform.header.frame_id + ' -> ' + transform.child_frame_id
            publishers.setdefault(edge, set()).add(gid)
    node.create_subscription(TFMessage, '/tf', tf_received, 100)
    node.create_subscription(TFMessage, '/tf_static', tf_received,
                             QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    started = time.monotonic()
    previous = started
    report_at = started
    start_sim = None
    with (args.directory / 'samples.jsonl').open('w') as stream:
        try:
            while time.monotonic() - started < args.seconds and not (args.directory / 'STOP').exists():
                rclpy.spin_once(node, timeout_sec=.02)
                if time.monotonic() - previous < .2:
                    continue
                previous = time.monotonic()
                now = node.get_clock().now().nanoseconds * 1e-9
                gt = next((m for m in reversed(truth) if stamp(m.header.stamp) <= now - .3), None)
                if gt is None:
                    continue
                if start_sim is None:
                    start_sim = now
                record = {'t': stamp(gt.header.stamp), 'now': now, 'counts': dict(counts)}
                try:
                    transform = buffer.lookup_transform('map', 'base_link', rclpy.time.Time.from_msg(gt.header.stamp)).transform
                    delta = xyz(transform.translation) - xyz(gt.pose.pose.position)
                    angle = float((rotation(transform.rotation) * rotation(gt.pose.pose.orientation).inv()).magnitude())
                    errors.append(float(np.linalg.norm(delta)))
                    yaw_errors.append(angle)
                    record.update(truth=xyz(gt.pose.pose.position).tolist(), delta=delta.tolist(), orientation_error=angle)
                except Exception as error:
                    tf_misses += 1
                    record['tf_error'] = str(error)
                for topic in ['/chassis/odom', '/aft_mapped_to_init', '/sim/imu', '/sentry/yaw_joint_state']:
                    message = last.get(topic)
                    if message is not None:
                        record[topic + '_age'] = now - stamp(message.header.stamp)
                        if topic == '/chassis/odom':
                            ages.append(record[topic + '_age'])
                stream.write(json.dumps(record) + '\n')
                stream.flush()
                if previous - report_at >= 20:
                    print(json.dumps(record), flush=True)
                    report_at = previous
        finally:
            summary = dict(samples=len(errors), tf_misses=tf_misses,
                           max_position_error_m=max(errors, default=None),
                           max_orientation_error_rad=max(yaw_errors, default=None),
                           max_chassis_age_sec=max(ages, default=None), counts=dict(counts),
                           sim_seconds=now - start_sim if start_sim is not None else 0,
                           wall_seconds=time.monotonic() - started,
                           edge_publishers={k: sorted(v) for k, v in publishers.items()},
                           duplicate_edges=[k for k, v in publishers.items() if len(v) > 1],
                           publisher_identity_available=all('unavailable' not in v for v in publishers.values()),
                           tf_publishers=[e.node_name for e in node.get_publishers_info_by_topic('/tf')])
            (args.directory / 'summary.json').write_text(json.dumps(summary, indent=2))
            print(json.dumps(summary), flush=True)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
