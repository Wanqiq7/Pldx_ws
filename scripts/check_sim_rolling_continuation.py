#!/usr/bin/env python3
"""Record local trajectory handoffs; run alongside check_sim_goal_accuracy.py.

Stop with Ctrl-C after navigation. Full references, commands and truth speeds
are saved for inspecting coverage and continuity; summary.json reports handoffs.
"""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from ros_interfaces.msg import MpcPositionCommand
from rosidl_runtime_py.convert import message_to_ordereddict
from tf2_ros import Buffer, TransformListener


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def progress(points, position):
    index = min(
        range(len(points)),
        key=lambda i: math.dist(points[i][:2], position) ** 2,
    )
    fraction = 0.0
    if index + 1 < len(points):
        delta = [points[index + 1][i] - points[index][i] for i in range(2)]
        length = sum(v * v for v in delta)
        if length > 1e-6:
            projection = sum(
                delta[i] * (position[i] - points[index][i]) for i in range(2)
            ) / length
            if -0.5 < projection < 1.0:
                fraction = projection
    return max(0.0, index + fraction) * 0.05


def velocity(points, at):
    index = max(0, min(int(at / 0.05), len(points) - 1))
    dt = max(0.0, min(0.05, at - index * 0.05))
    point = points[index]
    return [point[i + 2] + point[i + 4] * dt + point[i + 6] * dt**2 / 2 for i in range(2)]


def predicted_position(points, at):
    index = max(0, min(int(at / 0.05), len(points) - 1))
    dt = max(0.0, min(0.05, at - index * 0.05))
    point = points[index]
    return [
        point[i] + point[i + 2] * dt + point[i + 4] * dt**2 / 2 + point[i + 6] * dt**3 / 6
        for i in range(2)
    ]


def handoff_metrics(old, event):
    position = event["position"]
    points = event["points"]
    projected = progress(old["points"], position)
    start_progress = progress(old["points"], points[0][:2])
    elapsed = event["origin"] - old["origin"]
    tail = old["points"][-1]
    return dict(
        t=event["t"], previous_id=old["id"], id=event["id"],
        goal=event["goal"], position=position,
        tracking_error=math.dist(position, predicted_position(old["points"], elapsed)),
        distance_from_previous_start=math.dist(position, old["points"][0][:2]),
        old_tail_speed=math.hypot(*tail[2:4]),
        old_tail_goal_distance=math.dist(tail[:2], event["goal"]) if event["goal"] else None,
        nominal_remaining_lower=(len(old["points"]) - 2) * 0.05 - elapsed,
        spatial_remaining_lower=(len(old["points"]) - 2) * 0.05 - projected,
        # Compare both trajectories at the new start position. The robot moves
        # between the worker snapshot and submission at the next FSM tick.
        velocity_jump=math.dist(velocity(old["points"], start_progress), points[0][2:4]),
        commit_velocity_difference=math.dist(velocity(old["points"], projected), points[0][2:4]),
        progress_since_snapshot=projected - start_progress,
    )


class RollingProbe(Node):
    def __init__(self, directory):
        super().__init__(
            "rolling_continuation_probe",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.stream = (self.directory / "events.jsonl").open("w", buffering=1)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.seen = {}
        self.previous = None
        self.goal = None
        self.truth_speed = None
        self.motion = []
        self.handoffs = []
        self.plans = []
        self.keepalives = []
        self.violations = []
        self.create_subscription(MpcPositionCommand, "/opt_path", self.reference, 100)
        self.create_subscription(NavPath, "/plan", self.plan, 20)
        self.create_subscription(Twist, "/sim/cmd_vel", self.command, 100)
        self.create_subscription(
            Odometry, "/sim/ground_truth/odom", self.truth, qos_profile_sensor_data
        )

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def write(self, event):
        self.stream.write(json.dumps(event) + "\n")

    def truth(self, message):
        self.truth_speed = math.hypot(
            message.twist.twist.linear.x, message.twist.twist.linear.y
        )

    def command(self, message):
        event = dict(
            kind="motion", t=self.now(), truth_speed=self.truth_speed,
            command_speed=math.hypot(message.linear.x, message.linear.y),
        )
        self.motion.append(event)
        self.write(event)

    def plan(self, message):
        self.previous = None
        if not message.poses:
            return
        self.goal = [message.poses[-1].pose.position.x, message.poses[-1].pose.position.y]
        event = dict(kind="plan", t=self.now(), goal=self.goal)
        self.plans.append(event)
        self.write(event)

    def reference(self, message):
        event = dict(kind="block", t=self.now(), stamp=seconds(message.header.stamp))
        if message.command_flag == message.BLOCK_COMMAND or not message.cmds:
            self.previous = None
            self.write(event)
            return
        normalized = copy.deepcopy(message)
        normalized.header.stamp.sec = normalized.header.stamp.nanosec = 0
        fingerprint = hashlib.sha256(
            json.dumps(message_to_ordereddict(normalized), sort_keys=True).encode()
        ).hexdigest()
        identity = message.cmds[0].trajectory_id
        event.update(
            id=identity, origin=seconds(message.cmds[0].header.stamp),
            fingerprint=fingerprint,
        )
        if identity in self.seen:
            event.update(kind="keepalive", age=event["stamp"] - event["origin"])
            if self.seen[identity] != fingerprint:
                self.violations.append(f"trajectory {identity} changed during keepalive")
            self.keepalives.append(event)
            self.write(event)
            return
        self.seen[identity] = fingerprint
        points = [
            [p.position.x, p.position.y, p.velocity.x, p.velocity.y,
             p.acceleration.x, p.acceleration.y, p.jerk.x, p.jerk.y]
            for p in message.cmds
        ]
        event.update(kind="new", points=points, goal=self.goal)
        try:
            transform = self.buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            position = [transform.transform.translation.x, transform.transform.translation.y]
            event["position"] = position
        except Exception:
            position = None
        if self.previous is not None and position is not None:
            handoff = handoff_metrics(self.previous, event)
            self.handoffs.append(handoff)
            print(json.dumps(handoff), flush=True)
        self.previous = event
        self.write(event)

    def save(self):
        for handoff in self.handoffs:
            window = [m for m in self.motion if abs(m["t"] - handoff["t"]) <= 0.25]
            if window:
                handoff["minimum_command_speed"] = min(m["command_speed"] for m in window)
                speeds = [m["truth_speed"] for m in window if m["truth_speed"] is not None]
                handoff["minimum_truth_speed"] = min(speeds) if speeds else None
        # Check moving intermediate handoffs with a stable hot start. Startup,
        # tracking-error corrections and the final braking segment remain visible
        # in the full handoff list, but have different speed requirements.
        rolling = [
            h for h in self.handoffs
            if h["old_tail_speed"] > 0.2 and h["tracking_error"] < 0.45
            and h["distance_from_previous_start"] > 0.5
        ]
        for handoff in rolling:
            if handoff["spatial_remaining_lower"] < 0.55:
                self.violations.append(f"MPC coverage exhausted at {handoff['t']}")
            if handoff["velocity_jump"] > 0.1:
                self.violations.append(f"hot-start velocity discontinuity at {handoff['t']}")
            if handoff.get("minimum_command_speed", 0.0) < 0.2 or (
                handoff.get("minimum_truth_speed") is None
                or handoff["minimum_truth_speed"] < 0.2
            ):
                self.violations.append(f"intermediate stop at {handoff['t']}")
        summary = dict(
            plans=self.plans, handoffs=self.handoffs, keepalives=self.keepalives,
            checked_rolling_handoffs=len(rolling),
            violations=self.violations,
        )
        (self.directory / "summary.json").write_text(json.dumps(summary, indent=2))
        self.stream.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory")
    args = parser.parse_args()
    rclpy.init()
    node = RollingProbe(args.directory)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
