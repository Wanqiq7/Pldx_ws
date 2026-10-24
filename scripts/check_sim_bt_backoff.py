#!/usr/bin/env python3
"""Exercise the installed bt_manager against an isolated rmuc_2025 simulation.

Requires a running simulation, a sourced workspace and a non-default
ROS_DOMAIN_ID. Starts only bt_manager; records natural Nav2 aborts, retry UUIDs,
stop measurements and a reachable replacement with repeated successful goals.
"""
import argparse
import json
import math
import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import Log
from ros_interfaces.msg import GameInfo, MpcPositionCommand
from tf2_ros import StaticTransformBroadcaster

from check_sim_goal_accuracy import Accuracy, stamp


class BackoffProbe(Accuracy):
    def __init__(self, directory):
        super().__init__(directory)
        self.events = (self.directory / "events.jsonl").open("w", buffering=1)
        self.goals = {}
        self.goal_cutoff_sim = float("inf")
        self.failures = []
        self.target = None
        self.game_info = self.create_publisher(GameInfo, "/sentry/game_info", 10)
        self.create_timer(0.1, self.publish_manual_goal)
        self.create_subscription(
            GoalStatusArray, "/navigate_to_pose/_action/status", self.statuses, 100
        )
        self.create_subscription(MpcPositionCommand, "/opt_path", self.reference, 100)
        self.create_subscription(Log, "/rosout", self.rosout, 100)

    def event(self, kind, **fields):
        row = dict(kind=kind, wall=time.monotonic(), sim=self.sim_time(), **fields)
        self.events.write(json.dumps(row) + "\n")
        return row

    def sim_time(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def statuses(self, message):
        for entry in message.status_list:
            if stamp(entry.goal_info.stamp) < self.goal_cutoff_sim:
                continue
            uuid = bytes(entry.goal_info.goal_id.uuid).hex()
            goal = self.goals.setdefault(
                uuid, dict(uuid=uuid, accepted_sim=stamp(entry.goal_info.stamp))
            )
            if goal.get("status") != entry.status:
                goal.update(self.event("status", uuid=uuid, status=entry.status))

    def reference(self, message):
        if message.command_flag == MpcPositionCommand.PLANNING_FAILED_COMMAND:
            self.failures.append(self.event("planning_failed"))

    def rosout(self, message):
        if message.level >= Log.WARN:
            self.event("rosout", node=message.name, message=message.msg)

    def set_target(self, target):
        # Convert map coordinates using the actual map -> minimap TF.
        transform = self.buffer.lookup_transform("minimap", "map", rclpy.time.Time())
        translation, rotation = transform.transform.translation, transform.transform.rotation
        yaw = math.atan2(
            2 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1 - 2 * (rotation.y**2 + rotation.z**2),
        )
        x, y = target
        self.target = (
            translation.x + math.cos(yaw) * x - math.sin(yaw) * y,
            translation.y + math.sin(yaw) * x + math.cos(yaw) * y,
        )
        self.event("manual_target", map=target, minimap=self.target)
        self.publish_manual_goal()

    def publish_manual_goal(self):
        if self.target is None:
            return
        message = GameInfo()
        message.header.stamp = self.get_clock().now().to_msg()
        message.game_time_remaining = 300
        message.game_status = 4
        message.manual_key = 65
        message.manual_point_x, message.manual_point_y = self.target
        self.game_info.publish(message)

    def no_new_goals(self, count, until_sim):
        end = time.monotonic() + 30
        while self.sim_time() < until_sim:
            rclpy.spin_once(self, timeout_sec=0.03)
            assert len(self.goals) == count, "new Nav2 goal inside abort backoff"
            assert time.monotonic() < end, "simulation clock stalled during backoff"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--unreachable", type=json.loads, default=[19.975, 3.975])
    parser.add_argument("--replacement", type=json.loads, default=[4.5, 8.5])
    args = parser.parse_args()
    assert os.environ.get("ROS_DOMAIN_ID", "0") != "0", "use an isolated ROS_DOMAIN_ID"
    rclpy.init()
    node = BackoffProbe(args.directory)
    child = None
    report = dict(passed=False, unreachable=args.unreachable, replacement=args.replacement)
    try:
        node.ready()
        node.goal_cutoff_sim = node.sim_time()
        assert "ros_interface_bt" not in node.get_node_names(), "bt_manager is already running"
        # simulation.launch.py has no minimap publisher. Add only this missing
        # test transform, matching the blue-side transform in navigation2.launch.py.
        node.minimap = StaticTransformBroadcaster(node)
        if not node.buffer.can_transform("map", "minimap", rclpy.time.Time()):
            transform = TransformStamped()
            transform.header.stamp = node.get_clock().now().to_msg()
            transform.header.frame_id, transform.child_frame_id = "map", "minimap"
            transform.transform.translation.x = 28.7
            transform.transform.translation.y = 15.0
            transform.transform.rotation.z = math.sin(3.14 / 2)
            transform.transform.rotation.w = math.cos(3.14 / 2)
            node.minimap.sendTransform(transform)
            node.wait(lambda: node.buffer.can_transform("map", "minimap", rclpy.time.Time()), 5)
        node.set_target(args.unreachable)
        executable = Path(get_package_prefix("bt_manager")) / "lib/bt_manager/bt_manager_node"
        with (args.directory / "bt_manager.log").open("w") as log:
            child = subprocess.Popen(
                [str(executable), "--ros-args", "-p", "use_sim_time:=true",
                 "-p", "bt_debug_logs:=false", "-p", "publish_area_markers:=false",
                 "-p", "groot2_port:=1967"],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
            )
        node.phase = "unreachable_first_attempt"
        node.wait(lambda: len(node.goals) >= 1 or child.poll() is not None, 15)
        assert child.poll() is None, "bt_manager exited; inspect bt_manager.log"
        node.wait(lambda: any(g["status"] == 6 for g in node.goals.values()), 90)
        assert len(node.goals) == 1, "unexpected goal before the first natural abort"
        first = dict(next(iter(node.goals.values())))
        assert node.failures, "no asynchronous planning-failure notification observed"
        node.phase = "first_backoff"
        node.no_new_goals(1, first["sim"] + 1.8)
        node.snapshot("stopped_after_abort", args.unreachable, 6)
        stopped = node.records[-1]
        assert max(map(abs, stopped["cmd"])) < 0.01, "nonzero command after abort"
        assert math.hypot(*stopped["truth_velocity"][:2]) < 0.03, "robot did not stop"
        node.wait(lambda: len(node.goals) >= 2, 15)
        second = list(node.goals.values())[1]
        retry_gap = second["accepted_sim"] - first["sim"]
        report["first_retry_gap_sim_sec"] = retry_gap
        assert 1.8 <= retry_gap <= 3.0, f"unexpected first retry gap: {retry_gap}"
        node.phase = "unreachable_second_attempt"
        node.wait(lambda: second["status"] == 6, 90)
        assert len(node.goals) == 2, "unexpected new goal during second attempt"
        report["natural_aborts"] = [first, dict(second)]
        node.phase = "second_backoff"
        node.no_new_goals(2, second["sim"] + 3.5)
        changed_at = node.sim_time()
        node.set_target(args.replacement)
        node.wait(lambda: len(node.goals) >= 3, 10)
        replacement = list(node.goals.values())[2]
        replacement_delay = replacement["accepted_sim"] - changed_at
        report["replacement_delay_sim_sec"] = replacement_delay
        assert replacement_delay < 1.0, "new target was delayed by old target's backoff"
        assert replacement["accepted_sim"] < second["sim"] + 4.0
        node.phase = "reachable_replacement"
        node.wait(lambda: replacement["status"] in (4, 5, 6), 60)
        assert replacement["status"] == 4, "reachable replacement failed"
        node.phase = "successful_resends"
        node.spin_for(2.0)
        node.snapshot("replacement_hold", args.replacement, 4)
        assert node.records[-1]["truth_error"] < 0.15, "replacement goal error exceeds 0.15 m"
        replacement_goals = list(node.goals.values())[2:]
        assert len(replacement_goals) >= 2, "successful goal was not re-sent"
        assert all(g["status"] in (1, 2, 4) for g in replacement_goals)
        report.update(
            passed=True,
            planning_failure_notifications=len(node.failures),
            replacement_truth_error_m=node.records[-1]["truth_error"],
            successful_resends=sum(g["status"] == 4 for g in replacement_goals) - 1,
        )
        print(json.dumps(report, indent=2), flush=True)
    except BaseException as error:
        report["failure"] = str(error) or type(error).__name__
        raise
    finally:
        # Stop only the decision process started here, then cancel its last goal.
        node.target = None
        if child is not None and child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM)
                child.wait(timeout=5)
        cancel = node.create_client(CancelGoal, "/navigate_to_pose/_action/cancel_goal")
        if cancel.wait_for_service(timeout_sec=2):
            future = cancel.call_async(CancelGoal.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=3)
        report["goals"] = list(node.goals.values())
        (args.directory / "result.json").write_text(json.dumps(report, indent=2) + "\n")
        node.events.close()
        node.samples.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
