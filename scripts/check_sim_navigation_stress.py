#!/usr/bin/env python3
"""Stress a running simulation; fail on stalled motion or unsuccessful Nav2 goals.

Source ROS and install/setup.bash. Sends Nav2 goals only, never raw velocities.
Samples truth, odometry age, command age, trajectory age and execution gating.
Timeouts use wall time; stalls use simulation time so slow physics is not a stall.
"""
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import Log
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from ros_interfaces.msg import MpcPositionCommand
from std_msgs.msg import Bool, String


def stamp(msg):
    return msg.sec + msg.nanosec * 1e-9


class Stress(Node):
    def __init__(self, args):
        super().__init__('navigation_stress_probe', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.args = args
        args.directory.mkdir(parents=True, exist_ok=True)
        self.stream = (args.directory / 'samples.jsonl').open('w', buffering=1)
        self.logs = (args.directory / 'rosout.jsonl').open('w', buffering=1)
        self.truth = None
        self.odom_stamp = None
        self.command = {}
        self.path = None
        self.enabled = None
        self.gimbal = None
        self.target = None
        self.stall_start = None
        self.stall_anchor = None
        self.stalls = []
        self.legs = []
        self.failure = None
        self.goal_handle = None
        self.create_subscription(Odometry, '/sim/ground_truth/odom', self.on_truth, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/aft_mapped_to_init', self.on_odom, qos_profile_sensor_data)
        for topic in ['/cmd_vel_mpc', '/sim/cmd_vel']:
            self.create_subscription(Twist, topic, lambda m, t=topic: self.on_command(t, m), 50)
        self.create_subscription(MpcPositionCommand, '/opt_path', self.on_path, 20)
        self.create_subscription(Bool, '/minco/execution_enabled', self.on_enabled, 20)
        self.create_subscription(Log, '/rosout', self.on_log, 100)
        self.create_subscription(String, '/sentry/gimbal/status', self.on_gimbal, 10)
        self.create_timer(0.1, self.sample)
        self.action = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.lifecycle = self.create_client(GetState, '/bt_navigator/get_state')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_truth(self, msg):
        self.truth = msg

    def on_odom(self, msg):
        self.odom_stamp = stamp(msg.header.stamp)

    def on_command(self, topic, msg):
        self.command[topic] = dict(t=self.now(), velocity=[msg.linear.x, msg.linear.y, msg.angular.z])

    def on_path(self, msg):
        self.path = dict(t=self.now(), stamp=stamp(msg.header.stamp), size=len(msg.cmds), flag=msg.command_flag)

    def on_enabled(self, msg):
        self.enabled = msg.data

    def on_gimbal(self, msg):
        try:
            self.gimbal = json.loads(msg.data)
        except json.JSONDecodeError:
            self.gimbal = {"fault": "invalid gimbal status JSON"}

    def on_log(self, msg):
        if msg.level >= Log.WARN:
            self.logs.write(json.dumps(dict(t=stamp(msg.stamp), node=msg.name, message=msg.msg)) + '\n')

    def sample(self):
        if self.truth is None:
            return
        msg = self.truth
        p, v = msg.pose.pose.position, msg.twist.twist
        t = stamp(msg.header.stamp)
        xy = [p.x, p.y]
        distance = math.dist(xy, self.target) if self.target else None
        record = dict(t=t, wall_t=time.monotonic(), leg=len(self.legs), target=self.target,
                      truth=xy, velocity=[v.linear.x, v.linear.y, v.angular.z],
                      odom_age=self.now() - self.odom_stamp if self.odom_stamp else None,
                      commands=self.command, path=self.path, execution_enabled=self.enabled,
                      distance=distance, gimbal=self.gimbal)
        self.stream.write(json.dumps(record) + '\n')
        # A moving anchor catches a robot stuck despite nonzero commanded velocity.
        if self.target and distance > self.args.goal_radius:
            if self.stall_anchor is None or math.dist(xy, self.stall_anchor) > 0.05:
                self.stall_anchor, self.stall_start = xy, t
            if t - self.stall_start >= self.args.stall_seconds:
                self.stalls.append(record)
                raise AssertionError(f'stalled for {t - self.stall_start:.2f} sim seconds, {distance:.2f} m from goal')
        else:
            self.stall_anchor = self.stall_start = None

    def wait(self, predicate, timeout):
        end = time.monotonic() + timeout
        while not predicate() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not predicate():
            raise TimeoutError(f'wall timeout after {timeout} seconds')

    def save(self):
        (self.args.directory / 'summary.json').write_text(json.dumps(dict(
            goals=self.args.goals, cycles=self.args.cycles, legs=self.legs,
            stall_seconds=self.args.stall_seconds, goal_radius=self.args.goal_radius,
            stalls=self.stalls, failure=self.failure), indent=2))

    def run(self):
        try:
            self.wait(lambda: self.truth is not None and self.odom_stamp is not None
                      and self.action.server_is_ready() and self.lifecycle.service_is_ready(), 90)
            deadline = time.monotonic() + 90
            while True:
                state = self.lifecycle.call_async(GetState.Request())
                self.wait(state.done, 5)
                if state.result().current_state.id == 3:
                    break
                if time.monotonic() > deadline:
                    raise TimeoutError("Nav2 did not become active")
            for cycle in range(self.args.cycles):
                for target in self.args.goals:
                    self.target = target
                    self.stall_anchor = None
                    leg = dict(cycle=cycle + 1, target=target, start_t=self.now())
                    self.legs.append(leg)
                    print(json.dumps(dict(event='start', **leg)), flush=True)
                    goal = NavigateToPose.Goal()
                    goal.pose.header.frame_id = 'map'
                    goal.pose.header.stamp = self.get_clock().now().to_msg()
                    goal.pose.pose.position.x, goal.pose.pose.position.y = target
                    goal.pose.pose.orientation.w = 1.0
                    sent = self.action.send_goal_async(goal)
                    self.wait(sent.done, 10)
                    self.goal_handle = sent.result()
                    assert self.goal_handle.accepted, 'goal rejected'
                    result = self.goal_handle.get_result_async()
                    self.wait(result.done, self.args.timeout)
                    leg.update(status=result.result().status, end_t=self.now(),
                               truth=[self.truth.pose.pose.position.x, self.truth.pose.pose.position.y])
                    leg['truth_error'] = math.dist(leg['truth'], target)
                    self.goal_handle = None
                    assert leg['status'] == 4, f"navigation status {leg['status']}"
                    assert leg['truth_error'] <= self.args.goal_radius, 'goal succeeded away from truth target'
                    self.target = None
                    self.save()
                    print(json.dumps(dict(event='finish', **leg)), flush=True)
            print('PASS navigation stress', flush=True)
        except BaseException as exc:
            self.failure = str(exc) or type(exc).__name__
            if self.legs and 'status' not in self.legs[-1]:
                self.legs[-1].update(error=self.failure, end_t=self.now())
            raise
        finally:
            self.target = None
            self.save()
            if self.goal_handle and self.goal_handle.accepted:
                cancel = self.goal_handle.cancel_goal_async()
                self.wait(cancel.done, 5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--goals', type=json.loads, default=[[1.25, 2.0], [5.5, 2.0], [5.5, 5.5], [1.25, 6.75]])
    parser.add_argument('--cycles', type=int, default=3)
    parser.add_argument('--timeout', type=float, default=120)
    parser.add_argument('--stall-seconds', type=float, default=2)
    parser.add_argument('--goal-radius', type=float, default=0.3)
    args = parser.parse_args()
    if (args.cycles < 1 or any(not math.isfinite(x) or x <= 0 for x in
        [args.timeout, args.stall_seconds, args.goal_radius]) or not args.goals or
        any(len(g) != 2 or any(not math.isfinite(x) for x in g) for g in args.goals)):
        parser.error('provide finite [x,y] goals and positive cycles/timeouts/radius')
    rclpy.init()
    node = Stress(args)
    try:
        node.run()
    finally:
        node.stream.close()
        node.logs.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
