#!/usr/bin/env python3
"""Check rmuc_2025 goal accuracy against synchronized Gazebo truth and TF.

Run with the simulation's ROS_DOMAIN_ID after sourcing the workspace. Writes
samples.jsonl and result.json, then exits nonzero on localization/goal errors.
Use --stationary-only to isolate localization from planning and control.
"""
import argparse
import json
import math
import time
from collections import deque
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener


def stamp(s):
    return s.sec + s.nanosec * 1e-9


def pose(p):
    q = p.orientation
    return [
        p.position.x,
        p.position.y,
        p.position.z,
        math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)),
    ]


class Accuracy(Node):
    def __init__(self, directory):
        super().__init__(
            "goal_accuracy_probe",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.samples = (self.directory / "samples.jsonl").open("w")
        self.gt = deque(maxlen=1000)
        self.lio = deque(maxlen=1000)
        self.cmd = [0.0, 0.0, 0.0]
        self.phase = "initializing"
        self.latest = None
        self.records = []
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_subscription(
            Odometry,
            "/sim/ground_truth/odom",
            lambda m: self.gt.append(m),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            "/aft_mapped_to_init",
            lambda m: self.lio.append(m),
            qos_profile_sensor_data,
        )
        self.create_subscription(Twist, "/sim/cmd_vel", self.command, 50)
        self.create_timer(0.1, self.sample)
        self.action = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    def command(self, msg):
        self.cmd = [msg.linear.x, msg.linear.y, msg.angular.z]

    def tf(self, parent, child, when):
        t = self.buffer.lookup_transform(parent, child, when)
        p, q = t.transform.translation, t.transform.rotation
        return {
            "stamp": stamp(t.header.stamp),
            "pose": [
                p.x,
                p.y,
                p.z,
                math.atan2(
                    2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
                ),
            ],
        }

    def sample(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        gt = next(
            (m for m in reversed(self.gt) if stamp(m.header.stamp) <= now - 0.15), None
        )
        if gt is None:
            return
        record = self.measurement(gt)
        if record is not None:
            self.latest = record
            self.samples.write(json.dumps(record) + "\n")
            self.samples.flush()

    def measurement(self, gt):
        now = self.get_clock().now().nanoseconds * 1e-9
        t = rclpy.time.Time.from_msg(gt.header.stamp)
        frames = {}
        try:
            for parent, child in [
                ("map", "base_link"),
                ("map", "body"),
                ("map", "camera_init"),
                ("camera_init", "body"),
                ("body", "base_link"),
                ("map", "pcd_map"),
            ]:
                frames[parent + "/" + child] = self.tf(parent, child, t)
        except Exception:
            return
        truth = pose(gt.pose.pose)
        mapped = frames["map/base_link"]["pose"]
        lio = (
            min(
                self.lio,
                key=lambda m: abs(stamp(m.header.stamp) - stamp(gt.header.stamp)),
            )
            if self.lio
            else None
        )
        record = {
            "t": stamp(gt.header.stamp),
            "receipt_t": now,
            "phase": self.phase,
            "truth": truth,
            "truth_frame": gt.header.frame_id,
            "truth_child": gt.child_frame_id,
            "truth_velocity": [
                gt.twist.twist.linear.x,
                gt.twist.twist.linear.y,
                gt.twist.twist.angular.z,
            ],
            "tf": frames,
            "cmd": self.cmd,
            "localization_delta": [mapped[0] - truth[0], mapped[1] - truth[1]],
            "localization_error": math.dist(mapped[:2], truth[:2]),
        }
        if lio is not None:
            record["lio"] = {
                "t": stamp(lio.header.stamp),
                "parent": lio.header.frame_id,
                "child": lio.child_frame_id,
                "pose": pose(lio.pose.pose),
                "velocity": [
                    lio.twist.twist.linear.x,
                    lio.twist.twist.linear.y,
                    lio.twist.twist.angular.z,
                ],
            }
        return record

    def wait(self, condition, timeout):
        end = time.monotonic() + timeout
        while not condition() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.03)
        assert condition(), "timeout during " + self.phase

    def spin_for(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.03)

    def snapshot(self, label, target=None, status=None, at_time=None):
        assert self.latest is not None
        if at_time is None:
            data = dict(self.latest)
        else:
            gt = min(self.gt, key=lambda m: abs(stamp(m.header.stamp) - at_time))
            data = self.measurement(gt)
            assert data is not None, "missing TF at action completion"
            data["action_result_t"] = at_time
            data["sample_time_error_sec"] = stamp(gt.header.stamp) - at_time
            assert abs(data["sample_time_error_sec"]) < 0.015
        assert self.get_clock().now().nanoseconds * 1e-9 - data["t"] < 0.5
        data.update(label=label, status=status, target=target)
        if target:
            data["map_error"] = math.dist(
                data["tf"]["map/base_link"]["pose"][:2], target
            )
            data["truth_error"] = math.dist(data["truth"][:2], target)
        self.records.append(data)
        print(
            json.dumps({k: v for k, v in data.items() if k not in ["tf", "lio"]}),
            flush=True,
        )

    def ready(self):
        self.wait(
            lambda: self.action.server_is_ready() and self.latest is not None, 120
        )
        self.spin_for(3)
        self.phase = "stationary"
        self.spin_for(1)
        self.snapshot("initial_stationary")

    def navigate(self, target):
        self.phase = "goal_" + str(target)
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = target
        goal.pose.pose.orientation.w = 1.0
        sent = self.action.send_goal_async(goal)
        self.wait(sent.done, 10)
        handle = sent.result()
        assert handle.accepted
        result = handle.get_result_async()
        try:
            self.wait(result.done, 90)
        finally:
            if not result.done():
                canceled = handle.cancel_goal_async()
                self.wait(canceled.done, 10)
        status = result.result().status
        result_t = self.get_clock().now().nanoseconds * 1e-9
        # Allow TF and truth publishers to cover the result timestamp.
        self.spin_for(0.3)
        self.snapshot("action_result", target, status, at_time=result_t)
        self.phase = "settled_" + str(target)
        self.spin_for(1.5)
        self.snapshot("settled", target, status)
        assert status == 4, f"navigation failed: {status}"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("directory")
    parser.add_argument("--stationary-only", action="store_true")
    parser.add_argument("--max-localization-error", type=float, default=0.12)
    parser.add_argument("--max-goal-error", type=float, default=0.15)
    parser.add_argument("--max-vertical-drift", type=float, default=0.2)
    parser.add_argument("--goals", default="[[4.5,8.5],[4.5,7.3]]")
    args = parser.parse_args()
    rclpy.init()
    node = Accuracy(args.directory)
    violations = []
    try:
        node.ready()
        if not args.stationary_only:
            for target in json.loads(args.goals):
                node.navigate(target)
        initial = node.records[0]
        initial_z_offset = (
            initial["tf"]["map/base_link"]["pose"][2] - initial["truth"][2]
        )
        for r in node.records:
            r["vertical_drift"] = (
                r["tf"]["map/base_link"]["pose"][2] - r["truth"][2] - initial_z_offset
            )
            if abs(r["vertical_drift"]) > args.max_vertical_drift:
                violations.append(
                    f"{r['label']} vertical drift {r['vertical_drift']:.3f}m > {args.max_vertical_drift}m"
                )
            if r["localization_error"] > args.max_localization_error:
                violations.append(
                    f"{r['label']} localization error {r['localization_error']:.3f}m > {args.max_localization_error}m"
                )
            if (
                r["label"] in ["action_result", "settled"]
                and r["truth_error"] > args.max_goal_error
            ):
                violations.append(
                    f"{r['target']} true goal error {r['truth_error']:.3f}m > {args.max_goal_error}m"
                )
            if r["label"] == "action_result" and (
                math.hypot(*r["truth_velocity"][:2]) > 0.15
                or abs(r["truth_velocity"][2]) > 0.15
            ):
                violations.append(f"{r['target']} succeeded while moving")
            if r["label"] == "settled" and (
                math.hypot(*r["truth_velocity"][:2]) > 0.02
                or abs(r["truth_velocity"][2]) > 0.02
                or max(abs(v) for v in r["cmd"]) > 0.01
            ):
                violations.append(f"{r['target']} did not stop")
        assert not violations, violations
        print("PASS accuracy", flush=True)
    except (Exception, KeyboardInterrupt) as error:
        if not violations:
            violations.append(str(error) or type(error).__name__)
        raise
    finally:
        (node.directory / "result.json").write_text(
            json.dumps({"records": node.records, "violations": violations}, indent=2)
        )
        node.samples.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
