#!/usr/bin/env python3
"""Validate folding, localization and the temporary passage front in simulation.

Run in the ROS_DOMAIN_ID used by simulation.launch.py, after sourcing the workspace.
"""

import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data

from check_sim_goal_accuracy import Accuracy, pose


class GimbalAccuracy(Accuracy):
    def __init__(self, directory):
        super().__init__(directory)
        self.gimbal = None
        self.faults, self.stages, self.joints = [], [], []
        self.ready_samples = []
        self.posture_pub = self.create_publisher(String, "/sentry/gimbal/posture", 10)
        self.passage_pub = self.create_publisher(
            Twist, "/sentry/gimbal/passage_cmd_vel", 10
        )
        self.create_subscription(String, "/sentry/gimbal/status", self.status, 50)
        self.create_subscription(
            JointState,
            "/sim/gimbal/joint_states",
            self.joint_sample,
            qos_profile_sensor_data,
        )

    def status(self, message):
        state = json.loads(message.data)
        self.gimbal = state
        if state.get("fault"):
            self.faults.append(state)
        if "stage" not in state:
            return
        key = (state["target"], state["stage"])
        if not self.stages or tuple(self.stages[-1][:2]) != key:
            self.stages.append([*key, state["sim_time_s"]])
            print(json.dumps(dict(stage=key, t=state["sim_time_s"])), flush=True)
        if state["stage"] not in ("normal", "folded"):
            assert not state["ready"], state

    def joint_sample(self, message):
        q = dict(zip(message.name, message.position))
        v = dict(zip(message.name, message.velocity))
        self.joints.append(
            dict(
                t=message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
                position=q,
                velocity=v,
            )
        )
        if (
            "yaw_joint2" in q
            and -1.58 < q["yaw_joint2"] < -0.02
            and v["yaw_joint2"] > 0.1
        ):
            assert abs(q["360_ref_ref1_frame"]) < 0.01, q
        # A contact impulse can perturb yaw velocity at its folded zero without
        # starting a commanded yaw sweep. Check sequencing once yaw leaves zero.
        if abs(v.get("360_ref_ref1_frame", 0)) > 0.1 and abs(q["360_ref_ref1_frame"]) > 0.01:
            assert q["yaw_joint2"] < -1.58, q

    def await_state(self, target):
        self.phase = "gimbal_" + target
        self.wait(
            lambda: bool(self.faults)
            or (
                self.gimbal is not None
                and self.gimbal.get("target") == target
                and self.gimbal.get("ready")
            ),
            35,
        )
        assert not self.faults, self.faults[-1:]
        self.spin_for(0.5)
        self.ready_samples.append(dict(self.gimbal))
        self.snapshot("gimbal_" + target)

    def request_posture(self, target):
        self.posture_pub.publish(String(data=target))

    def cycle(self):
        self.request_posture("folded")
        self.await_state("folded")
        self.request_posture("normal")
        self.await_state("normal")

    def passage_forward(self):
        self.request_posture("folded")
        self.await_state("folded")
        assert abs(self.gimbal["front_yaw_in_chassis"] + math.pi / 4) < 0.01
        self.spin_for(0.5)
        start = pose(self.gt[-1].pose.pose)
        front = start[3] + self.gimbal["front_yaw_in_chassis"]
        command = Twist()
        command.linear.x = 0.12
        end = self.get_clock().now().nanoseconds * 1e-9 + 1.0
        deadline = time.monotonic() + 15
        while self.get_clock().now().nanoseconds * 1e-9 < end:
            assert time.monotonic() < deadline
            self.passage_pub.publish(command)
            self.spin_for(0.04)
        for _ in range(8):
            self.passage_pub.publish(Twist())
            self.spin_for(0.05)
        self.spin_for(1.0)
        finish = pose(self.gt[-1].pose.pose)
        dx, dy = finish[0] - start[0], finish[1] - start[1]
        forward = math.cos(front) * dx + math.sin(front) * dy
        lateral = -math.sin(front) * dx + math.cos(front) * dy
        result = dict(
            start=start,
            finish=finish,
            front_yaw=front,
            forward_m=forward,
            lateral_m=lateral,
        )
        print(json.dumps(dict(passage_direction=result)), flush=True)
        assert forward > 0.05 and abs(lateral) < 0.025, result
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--cycles", type=int, default=3)
    parser.add_argument(
        "--navigation",
        action="store_true",
        help="Navigate folded to [5.2,7.3], then restore and return to [4.6,7.3]",
    )
    args = parser.parse_args()
    rclpy.init()
    node = GimbalAccuracy(args.directory)
    result = {"passed": False}
    try:
        node.ready()
        node.request_posture("normal")
        node.await_state("normal")
        for _ in range(args.cycles):
            node.cycle()
        direction = node.passage_forward()
        if args.navigation:
            node.navigate([5.2, 7.3])
        node.request_posture("normal")
        node.await_state("normal")
        if args.navigation:
            node.navigate([4.6, 7.3])
        assert not node.faults
        for record in node.records:
            assert all(math.isfinite(v) for v in record["truth_velocity"]), record
            assert record["localization_error"] < 0.12, record
            if record["label"] in ("action_result", "settled"):
                assert record["truth_error"] < 0.15, record
        assert len(node.joints) > 500
        samples = [
            json.loads(line)
            for line in (args.directory / "samples.jsonl").read_text().splitlines()
        ]
        assert samples
        front_alignment = []
        turning_samples = 0
        for sample in samples:
            assert all(math.isfinite(v) for v in sample["truth_velocity"]), sample
            assert sample["localization_error"] < 0.12, sample
            if sample["phase"] == "goal_[5.2, 7.3]":
                vx, vy, wz = sample["cmd"]
                if math.hypot(vx, vy) < 0.001 and abs(wz) > 0.1:
                    turning_samples += 1
                if math.hypot(vx, vy) > 0.03:
                    error = math.atan2(vy, vx) + math.pi / 4
                    assert (
                        abs(math.atan2(math.sin(error), math.cos(error))) < 0.12
                    ), sample
                    # Compare the physical small-gimbal front with the requested
                    # world direction, including the estimated/true chassis yaw.
                    travel_yaw = (
                        sample["tf"]["map/base_link"]["pose"][3]
                        + math.atan2(vy, vx)
                    )
                    small_yaw = sample["truth"][3] - math.pi / 4
                    delta = travel_yaw - small_yaw
                    front_alignment.append(
                        abs(math.atan2(math.sin(delta), math.cos(delta)))
                    )
        if args.navigation:
            assert front_alignment, "no folded navigation translation samples"
            assert turning_samples, "no chassis alignment before folded navigation"
            assert max(front_alignment) < 0.15, front_alignment
        result.update(
            passed=True,
            cycles=args.cycles,
            passage_direction=direction,
            navigation_front_alignment={
                "translation_samples": len(front_alignment),
                "turning_samples": turning_samples,
                "max_small_gimbal_heading_error_rad": max(front_alignment, default=None),
            },
            stages=node.stages,
            records=node.records,
            joint_samples=len(node.joints),
            ready_samples=node.ready_samples,
            max_localization_error_m=max(r["localization_error"] for r in samples),
            joint_peak_speed_rad_s={
                name: max(abs(sample["velocity"][name]) for sample in node.joints)
                for name in node.joints[0]["velocity"]
            },
        )
    except BaseException as error:
        result.update(
            error=str(error),
            faults=node.faults,
            stages=node.stages,
            records=node.records,
        )
        raise
    finally:
        (args.directory / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        (args.directory / "joint_samples.json").write_text(
            json.dumps(node.joints) + "\n"
        )
        node.samples.close()
        node.destroy_node()
        rclpy.shutdown()
    print("PASS folding integration", flush=True)


if __name__ == "__main__":
    main()
