#!/usr/bin/env python3
"""Regress terminal pose tracking in a running headless rmul_2027_3v3 sim.

Source ROS and the workspace before running. Uses NavigateToPose and synchronized
Gazebo truth/TF measurements; never changes navigation tolerances or publishes
manual velocity commands. A failed assertion exits nonzero and retains evidence.
"""

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
from rcl_interfaces.msg import Log
from ros_interfaces.msg import MpcPositionCommand

from check_sim_goal_accuracy import Accuracy, pose, stamp


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class TerminalPoseProbe(Accuracy):
    def __init__(self, directory, timeout):
        super().__init__(directory, timeout)
        self.observations = []
        self.commands = {}
        self.reference = None
        self.events = []
        self.goal_handle = None
        for topic in ("/cmd_vel_mpc", "/cmd_vel_nav", "/sim/cmd_vel"):
            self.create_subscription(
                Twist, topic,
                lambda msg, key=topic: self.commands.update(
                    {key: [msg.linear.x, msg.linear.y, msg.angular.z]}), 50)
        self.create_subscription(MpcPositionCommand, "/opt_path", self.on_reference, 50)
        self.create_subscription(Log, "/rosout", self.on_log, 100)

    def measurement(self, truth_msg):
        # Some exploration worlds have no static pcd_map frame. Only the
        # robot transform at the truth stamp is required for these assertions.
        try:
            mapped = self.tf("map", "base_link", rclpy.time.Time.from_msg(truth_msg.header.stamp))
        except Exception:
            return None
        truth = pose(truth_msg.pose.pose)
        velocity = truth_msg.twist.twist
        return {"t": stamp(truth_msg.header.stamp), "phase": self.phase,
                "truth": truth, "tf": {"map/base_link": mapped}, "cmd": list(self.cmd),
                "truth_velocity": [velocity.linear.x, velocity.linear.y, velocity.angular.z],
                "localization_error": math.dist(mapped["pose"][:2], truth[:2])}

    def ready(self):
        super().ready()
        # Action discovery precedes lifecycle activation. Do not send a goal
        # until Nav2 can accept it, even when TF and the action service exist.
        client = self.create_client(GetState, "/bt_navigator/get_state")
        self.wait(client.service_is_ready, 120)
        deadline = time.monotonic() + 120
        while time.monotonic() < deadline:
            state = client.call_async(GetState.Request())
            self.wait(state.done, 10)
            if state.result().current_state.id == 3:  # lifecycle ACTIVE
                return
            self.spin_for(0.2)
        raise AssertionError("bt_navigator did not become active")

    def on_reference(self, msg):
        self.reference = {"flag": msg.command_flag, "count": len(msg.cmds),
                          "terminal_pose": msg.terminal_pose}
        if msg.cmds:
            last = msg.cmds[-1]
            self.reference["end"] = [last.position.x, last.position.y, last.yaw]

    def on_log(self, msg):
        if msg.level >= Log.WARN:
            self.events.append({"phase": self.phase, "node": msg.name,
                                "message": msg.msg})

    def sample(self):
        super().sample()
        if self.latest and (not self.observations or self.latest["t"] > self.observations[-1]["t"]):
            self.observations.append({**self.latest, "commands": dict(self.commands),
                                      "reference": self.reference})

    def current_pose(self):
        assert self.latest is not None
        assert self.get_clock().now().nanoseconds * 1e-9 - self.latest["t"] < 0.5, "stale pose sample"
        assert self.latest["localization_error"] < 0.2, "localization drift exceeds 0.2 m"
        p = self.latest["tf"]["map/base_link"]["pose"]
        return [p[0], p[1], p[3]]

    def navigate_pose(self, name, target, cancel_after=None):
        self.phase = name
        first = len(self.observations)
        started = self.get_clock().now().nanoseconds * 1e-9
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = target[:2]
        goal.pose.pose.orientation.z = math.sin(target[2] / 2)
        goal.pose.pose.orientation.w = math.cos(target[2] / 2)
        sent = self.action.send_goal_async(goal)
        self.wait(sent.done, 10)
        self.goal_handle = sent.result()
        assert self.goal_handle.accepted, name + ": goal rejected"
        result = self.goal_handle.get_result_async()
        try:
            if cancel_after is not None:
                self.spin_for(cancel_after)
                assert not result.done(), name + ": finished before cancellation"
                canceled = self.goal_handle.cancel_goal_async()
                self.wait(canceled.done, 10)
            self.wait(result.done, self.goal_timeout)
        finally:
            if not result.done():
                canceled = self.goal_handle.cancel_goal_async()
                self.wait(canceled.done, 10)
                self.wait(result.done, 10)
        status = result.result().status
        ended = self.get_clock().now().nanoseconds * 1e-9
        self.goal_handle = None
        # Let delayed truth/TF cover the end of the action.
        self.spin_for(0.4)
        final = self.current_pose()
        truth = self.latest["truth"]
        rows = self.observations[first:]
        near = [row for row in rows
                if math.dist(row["tf"]["map/base_link"]["pose"][:2], target[:2]) < 0.1]
        report = {"case": name, "target": target, "status": status,
                  "duration_sim_s": ended - started,
                  "terminal_sim_s": ended - near[0]["t"] if near else None,
                  "map_xy_error_m": math.dist(final[:2], target[:2]),
                  "map_yaw_error_rad": abs(wrap(target[2] - final[2])),
                  "truth_xy_error_m": math.dist(truth[:2], target[:2]),
                  "truth_yaw_error_rad": abs(wrap(target[2] - truth[3]))}
        if cancel_after is None:
            assert status == 4, report
            assert report["map_xy_error_m"] <= 0.11, report
            assert report["map_yaw_error_rad"] <= 0.11, report
            assert report["truth_xy_error_m"] <= 0.15, report
            assert report["truth_yaw_error_rad"] <= 0.15, report
            assert report["terminal_sim_s"] is not None and report["terminal_sim_s"] < 10, report
        else:
            assert status == 5, report
        self.phase = name + "_idle"
        idle_first = len(self.observations)
        self.spin_for(8.0)
        idle = self.observations[idle_first:]
        assert len(idle) >= 10 and idle[-1]["t"] - idle[0]["t"] >= 2.0, "insufficient idle measurements"
        assert self.get_clock().now().nanoseconds * 1e-9 - idle[-1]["t"] < 0.5, "stale idle measurements"
        expected_topics = {"/cmd_vel_mpc", "/cmd_vel_nav", "/sim/cmd_vel"}
        assert all(set(row["commands"]) == expected_topics for row in idle), "missing velocity channel"
        report["idle_localization_max_m"] = max(row["localization_error"] for row in idle)
        yaw0 = idle[0]["truth"][3]
        report["idle_yaw_range_rad"] = max(abs(wrap(row["truth"][3] - yaw0)) for row in idle)
        report["idle_command_max"] = max(
            abs(value) for row in idle for cmd in row["commands"].values() for value in cmd)
        assert report["idle_yaw_range_rad"] < 0.01, report
        assert report["idle_command_max"] < 1e-4, report
        assert report["idle_localization_max_m"] < 0.2, report
        print(json.dumps(report), flush=True)
        return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--goal-timeout", type=float, default=90.0)
    parser.add_argument("--scenario", choices=("all", "routes", "rotations", "adjustment", "cancel"), default="all")
    args = parser.parse_args()
    rclpy.init()
    node = TerminalPoseProbe(args.directory, args.goal_timeout)
    reports = []
    failure = None
    try:
        node.ready()
        if args.scenario in ("all", "routes"):
            reports.append(node.navigate_pose("outbound", [5.43, 5.02, 0.0]))
            reports.append(node.navigate_pose("return", [1.03, 6.55, 0.0]))
            reports.append(node.navigate_pose("opposite_heading", [5.43, 5.02, math.pi]))
        if args.scenario in ("all", "rotations"):
            p = node.current_pose()
            reports.append(node.navigate_pose("yaw_only", [p[0], p[1], wrap(p[2] + math.pi / 2)]))
            p = node.current_pose()
            reports.append(node.navigate_pose("positive_pi", [p[0], p[1], 3.0]))
            p = node.current_pose()
            reports.append(node.navigate_pose("cross_pi", [p[0], p[1], -3.0]))
        if args.scenario in ("all", "rotations", "adjustment"):
            p = node.current_pose()
            reports.append(node.navigate_pose("near_goal_adjustment",
                                              [p[0] + 0.06, p[1] + 0.06, wrap(p[2] + 0.4)]))
        destination = [1.03, 6.55, 0.0] if args.scenario in ("all", "routes") else [5.43, 5.02, 0.0]
        reports.append(node.navigate_pose("cancel", destination, cancel_after=1.0))
    except Exception as exc:
        failure = str(exc)
        raise
    finally:
        try:
            if node.goal_handle is not None:
                future = node.goal_handle.cancel_goal_async()
                node.wait(future.done, 10)
        except Exception as exc:
            failure = failure or "cleanup failed: " + str(exc)
        summary = {"passed": failure is None, "failure": failure, "cases": reports}
        (args.directory / "terminal_results.json").write_text(json.dumps(summary, indent=2) + "\n")
        (args.directory / "terminal_trace.json").write_text(json.dumps(node.observations) + "\n")
        (args.directory / "terminal_warnings.json").write_text(json.dumps(node.events, indent=2) + "\n")
        node.samples.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
