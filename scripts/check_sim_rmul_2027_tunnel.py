#!/usr/bin/env python3
"""Run the RMUL 2027 red low-tunnel integration acceptance.

The running simulation must use ``robot_model:=real`` and
``world:=rmul_2027_3v3``. The route is derived from the STEP measurements:
approach the west side, fold, drive through the 0.25 m roof section, then
restore only after the chassis clears the tunnel.
"""

import argparse
import json
import math
from pathlib import Path
import time
import sys
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

PACKAGE = Path(__file__).resolve().parents[1] / "src/nav-simulation/sentry_simulation"
if str(PACKAGE) not in __import__("sys").path:
    __import__("sys").path.insert(0, str(PACKAGE))
if str(Path(__file__).parent) not in sys.path:
    sys.path.insert(0, str(Path(__file__).parent))
if str(PACKAGE / "scripts") not in sys.path:
    sys.path.insert(0, str(PACKAGE / "scripts"))
from sentry_simulation.folding import DEFAULT_GIMBAL_SPEED, DEFAULT_PHYSICS_STEP
from check_gimbal_fold import envelope, make_world, sequenced_targets


ENTRY = (2.0, 7.46, math.pi / 4)
TUNNEL_EXIT_X = 4.2
TUNNEL_PATH_Y = 7.46
PASSAGE_SPEED = 0.08
PASSAGE_TIMEOUT_S = 90.0


def stamp(message):
    return message.sec + message.nanosec * 1e-9


def yaw_from_pose(pose):
    q = pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def roll_pitch_from_pose(pose):
    q = pose.orientation
    roll = math.atan2(
        2.0 * (q.w * q.x + q.y * q.z),
        1.0 - 2.0 * (q.x * q.x + q.y * q.y),
    )
    pitch_input = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.asin(max(-1.0, min(1.0, pitch_input)))
    return roll, pitch


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class Rmul2027TunnelAcceptance(Node):
    def __init__(self, output, timeout):
        super().__init__(
            "rmul_2027_tunnel_acceptance",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=True)
        self.timeout = timeout
        self.truth = deque(maxlen=2000)
        self.gimbal = None
        self.mpc = None
        self.sim_cmd = None
        self.samples = []
        self.action = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.posture = self.create_publisher(String, "/sentry/gimbal/posture", 10)
        self.passage = self.create_publisher(
            Twist, "/sentry/gimbal/passage_cmd_vel", 10
        )
        self.create_subscription(
            Odometry,
            "/sim/ground_truth/odom",
            lambda message: self.truth.append(message),
            qos_profile_sensor_data,
        )
        self.create_subscription(String, "/sentry/gimbal/status", self.gimbal_cb, 10)
        self.create_subscription(Twist, "/cmd_vel_mpc", self.mpc_cb, 50)
        self.create_subscription(Twist, "/sim/cmd_vel", self.sim_cb, 50)
        self.create_timer(0.05, self.sample)

    @staticmethod
    def clearance_check():
        _, model, reference, profile = make_world(
            DEFAULT_GIMBAL_SPEED,
            with_yaw=True,
            physics_step=DEFAULT_PHYSICS_STEP,
        )
        report = envelope(model, reference, profile, sequenced_targets(model, reference))
        folded = next(item for item in report["endpoints"] if item["phase"] == "folded")
        measurements = json.loads(
            (PACKAGE / "maps/rmul_2027_3v3/step_measurements.json").read_text()
        )
        roof_bounds = measurements["measurements"]["red_tunnel_roof_underside"]["bounds_m"]
        roof_underside = roof_bounds[2]
        clearance = roof_underside - folded["maximum_xyz_m"][2]
        folded_width = folded["maximum_xyz_m"][1] - folded["minimum_xyz_m"][1]
        tunnel_span = json.loads(
            (PACKAGE / "maps/rmul_2027_3v3/parameters.json").read_text()
        )["manual_reference"]["specified"]["tunnel_clear_span"]
        lateral_clearance = tunnel_span - folded_width
        return {
            "roof_underside_z_m": roof_underside,
            "folded_max_z_m": folded["maximum_xyz_m"][2],
            "folded_clearance_m": clearance,
            "folded_width_m": folded_width,
            "tunnel_clear_span_m": tunnel_span,
            "lateral_clearance_m": lateral_clearance,
            "required_clearance_margin_m": 0.005,
            "passed": clearance >= 0.005 and lateral_clearance >= 0.005,
        }

    def gimbal_cb(self, message):
        try:
            self.gimbal = json.loads(message.data)
        except json.JSONDecodeError:
            self.gimbal = {"fault": "invalid gimbal JSON"}

    def mpc_cb(self, message):
        self.mpc = [message.linear.x, message.linear.y, message.angular.z]

    def sim_cb(self, message):
        self.sim_cmd = [message.linear.x, message.linear.y, message.angular.z]

    def sample(self):
        if not self.truth:
            return
        message = self.truth[-1]
        pose = message.pose.pose
        self.samples.append(
            {
                "t": stamp(message.header.stamp),
                "truth": [pose.position.x, pose.position.y, yaw_from_pose(pose)],
                "velocity": [
                    message.twist.twist.linear.x,
                    message.twist.twist.linear.y,
                    message.twist.twist.angular.z,
                ],
                "gimbal": self.gimbal,
                "cmd_vel_mpc": self.mpc,
                "sim_cmd_vel": self.sim_cmd,
            }
        )

    def spin_until(self, predicate, timeout, label):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() >= deadline:
                raise RuntimeError(f"timeout waiting for {label}")
            rclpy.spin_once(self, timeout_sec=0.03)

    def navigate(self, target):
        if self.truth:
            current = self.truth[-1].pose.pose
            current_pose = [
                current.position.x,
                current.position.y,
                yaw_from_pose(current),
            ]
            if (
                (math.dist(current_pose[:2], target[:2]) < 0.6
                 or 1.8 <= current_pose[0] <= 3.2)
                and 7.06 <= current_pose[1] <= 7.86
            ):
                return
        self.spin_until(self.action.server_is_ready, 60.0, "navigate_to_pose action")
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = target[:2]
        goal.pose.pose.orientation.z = math.sin(target[2] / 2.0)
        goal.pose.pose.orientation.w = math.cos(target[2] / 2.0)
        sent = self.action.send_goal_async(goal)
        self.spin_until(sent.done, 10.0, "goal response")
        handle = sent.result()
        if not handle.accepted:
            raise RuntimeError("entry goal rejected")
        result = handle.get_result_async()
        self.spin_until(result.done, self.timeout, "entry navigation")
        if result.result().status != 4:
            raise RuntimeError(f"entry navigation failed with status {result.result().status}")

    def wait_gimbal(self, target):
        deadline = time.monotonic() + 45.0
        next_request = 0.0
        while not (
            self.gimbal is not None
            and self.gimbal.get("target") == target
            and self.gimbal.get("ready")
            and not self.gimbal.get("fault")
        ):
            now = time.monotonic()
            if now >= deadline:
                raise RuntimeError(f"timeout waiting for gimbal {target}")
            if now >= next_request:
                self.publish_posture(target)
                next_request = now + 0.25
            rclpy.spin_once(self, timeout_sec=0.03)

    def publish_posture(self, target):
        self.posture.publish(String(data=target))

    def passage_drive(self):
        self.spin_until(lambda: bool(self.truth), 20.0, "ground truth")
        start = self.truth[-1].pose.pose
        start_xy = [start.position.x, start.position.y]
        path_y = start.position.y
        deadline = time.monotonic() + PASSAGE_TIMEOUT_S
        max_abs_roll = 0.0
        max_abs_pitch = 0.0
        min_z = float("inf")
        max_z = -float("inf")
        while True:
            current = self.truth[-1].pose.pose
            x, y = current.position.x, current.position.y
            roll, pitch = roll_pitch_from_pose(current)
            max_abs_roll = max(max_abs_roll, abs(roll))
            max_abs_pitch = max(max_abs_pitch, abs(pitch))
            min_z = min(min_z, current.position.z)
            max_z = max(max_z, current.position.z)
            if x >= TUNNEL_EXIT_X:
                break
            if time.monotonic() >= deadline:
                raise RuntimeError(f"tunnel drive timeout at ({x:.3f}, {y:.3f})")
            if not 7.06 <= y <= 7.86 or abs(y - path_y) > 0.25:
                raise RuntimeError(f"tunnel lateral deviation at ({x:.3f}, {y:.3f})")
            if self.gimbal is None or self.gimbal.get("target") != "folded" or not self.gimbal.get("ready"):
                raise RuntimeError("folded posture was lost during tunnel passage")
            if self.gimbal.get("fault"):
                raise RuntimeError(f"gimbal fault during tunnel passage: {self.gimbal['fault']}")
            if not all(math.isfinite(value) for value in (x, y)):
                raise RuntimeError("non-finite ground truth during tunnel passage")
            if abs(roll) > 0.35 or abs(pitch) > 0.35:
                raise RuntimeError("excessive roll/pitch during tunnel passage")
            command = Twist()
            command.linear.x = PASSAGE_SPEED
            command.linear.y = max(-0.02, min(0.02, -(y - path_y)))
            self.passage.publish(command)
            rclpy.spin_once(self, timeout_sec=0.02)
        for _ in range(20):
            self.passage.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.03)
        finish = self.truth[-1].pose.pose
        return {
            "start": start_xy,
            "finish": [finish.position.x, finish.position.y],
            "forward_m": finish.position.x - start.position.x,
            "lateral_error_m": finish.position.y - path_y,
            "path_y_m": path_y,
            "max_abs_roll_rad": max_abs_roll,
            "max_abs_pitch_rad": max_abs_pitch,
            "min_z_m": min_z,
            "max_z_m": max_z,
        }

    def run(self):
        clearance = self.clearance_check()
        if not clearance["passed"]:
            raise RuntimeError(
                "RMUL 2027 tunnel roof is lower than the folded CAD envelope: "
                f"clearance={clearance['folded_clearance_m']:.3f} m"
            )
        self.spin_until(lambda: bool(self.truth), 30.0, "ground truth")
        self.publish_posture("normal")
        self.wait_gimbal("normal")
        self.navigate(ENTRY)
        self.publish_posture("folded")
        self.wait_gimbal("folded")
        passage = self.passage_drive()
        if passage["forward_m"] < TUNNEL_EXIT_X - passage["start"][0]:
            raise RuntimeError("passage ended before the measured tunnel exit")
        self.publish_posture("normal")
        self.wait_gimbal("normal")
        nonzero_mpc = sum(
            1
            for sample in self.samples
            if sample["cmd_vel_mpc"] and max(abs(v) for v in sample["cmd_vel_mpc"]) > 1e-4
        )
        nonzero_sim = sum(
            1
            for sample in self.samples
            if sample["sim_cmd_vel"] and max(abs(v) for v in sample["sim_cmd_vel"]) > 1e-4
        )
        if nonzero_sim == 0:
            raise RuntimeError("no non-zero /sim/cmd_vel samples during passage")
        result = {
            "passed": True,
            "world": "rmul_2027_3v3",
            "entry": ENTRY,
            "tunnel_floor_bounds": {"x": [2.5, 4.5], "y": [7.06, 7.86]},
            "roof_underside_z_m": 0.25,
            "clearance": clearance,
            "passage": passage,
            "samples": len(self.samples),
            "nonzero_cmd_vel_mpc_samples": nonzero_mpc,
            "nonzero_sim_cmd_vel_samples": nonzero_sim,
            "final_gimbal": self.gimbal,
            "collision_evidence_available": False,
            "acceptance_scope": "position_pose_and_command_chain; contact sensors unavailable",
        }
        (self.output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        (self.output / "samples.json").write_text(json.dumps(self.samples) + "\n")
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--timeout", type=float, default=180.0)
    args = parser.parse_args()
    rclpy.init()
    node = Rmul2027TunnelAcceptance(args.directory, args.timeout)
    result = {
        "passed": False,
        "world": "rmul_2027_3v3",
        "clearance": node.clearance_check(),
    }
    try:
        result = node.run()
        print(json.dumps(result), flush=True)
    except Exception as error:
        result.update(error=f"{type(error).__name__}: {error}")
        (args.directory / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        raise
    finally:
        (args.directory / "samples.json").write_text(json.dumps(node.samples) + "\n")
        for _ in range(10):
            node.passage.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.02)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
