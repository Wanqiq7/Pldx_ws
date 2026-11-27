#!/usr/bin/env python3
"""Record #11 evidence without commanding any hardware. Ctrl-C finishes the record."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', required=True, choices=[
        'stationary_baseline', 'stationary_sweep', 'locked_joint_rotation',
        'simultaneous_rotation', 'world_translation'])
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--geometry', type=Path, required=True)
    parser.add_argument('--operator', required=True)
    parser.add_argument('--duration', type=float, default=60.)
    args = parser.parse_args()
    if not args.geometry.is_file() or args.duration <= 0:
        parser.error('geometry must exist and duration must be positive')
    args.output.mkdir(parents=True, exist_ok=False)
    shutil.copyfile(args.geometry, args.output / 'geometry.yaml')
    manifest = {'case': args.case, 'operator': args.operator,
                'utc': datetime.now(timezone.utc).isoformat(), 'status': 'recording',
                'hardware_id': None, 'firmware_revisions': None,
                'locked_mechanisms': None, 'motor_zero_rad': None,
                'direction_verified': None, 'gear_ratio': None,
                'measured_geometry': False, 'ground_truth_method': None,
                'tolerances_and_basis': None, 'operator_verdict': None}
    manifest_path = args.output / 'manifest.json'
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2))

    import rclpy
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist, TwistStamped
    from ros_interfaces.msg import YawJointState, Behavior
    from std_msgs.msg import UInt8MultiArray
    from tf2_msgs.msg import TFMessage
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from rosidl_runtime_py.convert import message_to_ordereddict

    rclpy.init()
    node = rclpy.create_node('record_chassis_bench')
    stream = (args.output / 'samples.jsonl').open('w')
    counts, tf_sources = {}, {}

    def callback(topic):
        def record(message, info):
            counts[topic] = counts.get(topic, 0) + 1
            # Jazzy rclpy MessageInfo exposes timestamps but not the writer GID.
            # Record endpoint identities separately; do not invent child ownership.
            gid = info.get("publisher_gid")
            source = bytes(gid).hex() if gid is not None else None
            if topic in ('/tf', '/tf_static'):
                for transform in message.transforms:
                    child = transform.child_frame_id
                    tf_sources.setdefault(child, set()).add(transform.header.frame_id)
            record = {'topic': topic, 'receive_ns': node.get_clock().now().nanoseconds,
                      'publisher_gid': source, 'transport_info': info,
                      'message': message_to_ordereddict(message)}
            stream.write(json.dumps(record) + '\n')
        return record

    for topic, kind in [('/aft_mapped_to_init', Odometry), ('/chassis/odom', Odometry),
                        ('/chassis/pose_difference', TwistStamped),
                        ('/sentry/yaw_joint_state', YawJointState),
                        ('/cmd_vel_mpc', Twist), ('/sentry/behaivor_send', Behavior),
                        ('/chassis/test/nav_packet', UInt8MultiArray), ('/tf', TFMessage)]:
        node.create_subscription(kind, topic, callback(topic), qos_profile_sensor_data)
    static_qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(TFMessage, '/tf_static', callback('/tf_static'), static_qos)
    import time
    end = time.monotonic() + args.duration
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        stream.close()
        endpoints = {}
        for topic in ('/tf', '/tf_static'):
            endpoints[topic] = [
                {'node': endpoint.node_namespace + '/' + endpoint.node_name,
                 'gid': bytes(endpoint.endpoint_gid).hex()}
                for endpoint in node.get_publishers_info_by_topic(topic)]
        manifest.update(status='recorded_unreviewed', counts=counts,
                        tf_publisher_endpoints=endpoints,
                        tf_child_ownership='requires_manual_verification_if_gid_unavailable',
                        tf_sources={key: sorted(value) for key, value in tf_sources.items()},
                        conflicting_tf_parents=[key for key, value in tf_sources.items() if len(value) > 1])
        manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2))
        node.destroy_node()
        rclpy.shutdown()
    print(args.output / 'manifest.json')


if __name__ == '__main__':
    main()
