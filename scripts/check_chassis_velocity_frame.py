#!/usr/bin/env python3
"""Check that the chassis wheel odometry uses the controller's body frame.

Records /sentry/chassis_velocity (TwistStamped, body frame) and the LIO odometry,
converts the LIO body twist to the chassis reference point with the same lever arm
as MincoMpcController::compensateLeverArm, and fits

    [vx_w, vy_w] = M [vx_lio, vy_lio],   wz_w = k * wz_lio

over moving samples. A consistent frame gives M close to identity (any scale error
such as a wrong reduction ratio shows on its diagonal) and k close to +1. A swapped
or mirrored axis shows as off-diagonal or negative entries; k < 0 means the yaw rate
sign is opposite to REP-103 (counter-clockwise positive).

Wheel slip and contact impulses produce outliers, so both fits are trimmed: samples
whose residual exceeds 4 median absolute deviations are dropped and the fit repeated.

Drive the robot while this runs, with translation in several directions and some
rotation. Run with the stack's ROS_DOMAIN_ID; it does not command the chassis.
"""
import argparse
import bisect
import json
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data


def stamp(s):
    return s.sec + s.nanosec * 1e-9


class Recorder(Node):
    def __init__(self, args):
        overrides = [Parameter("use_sim_time", value=args.use_sim_time)]
        super().__init__("chassis_velocity_frame_check", parameter_overrides=overrides)
        self.wheel = []
        self.lio = []
        self.create_subscription(
            TwistStamped, args.wheel_topic, self.on_wheel, qos_profile_sensor_data
        )
        self.create_subscription(Odometry, args.odom_topic, self.on_odom, qos_profile_sensor_data)

    def on_wheel(self, m):
        t = m.twist
        self.wheel.append((stamp(m.header.stamp), t.linear.x, t.linear.y, t.angular.z))

    def on_odom(self, m):
        t = m.twist.twist
        self.lio.append((stamp(m.header.stamp), t.linear.x, t.linear.y, t.angular.z))


def trimmed_fit(X, Y, fit, rounds=3, k=4.0):
    """Least squares with iterative rejection of residuals beyond k robust sigmas."""
    keep = np.ones(len(X), dtype=bool)
    for _ in range(rounds):
        params = fit(X[keep], Y[keep])
        res = np.linalg.norm(np.atleast_2d((Y - predict(X, params)).T).T.reshape(len(Y), -1), axis=1)
        mad = np.median(np.abs(res[keep] - np.median(res[keep]))) * 1.4826
        new_keep = res <= np.median(res[keep]) + k * max(mad, 1e-6)
        if (new_keep == keep).all():
            break
        keep = new_keep
    params = fit(X[keep], Y[keep])
    return params, keep


def predict(X, params):
    return X @ params.T if np.ndim(params) == 2 else X * params


def fit_matrix(L, W):
    return np.linalg.lstsq(L, W, rcond=None)[0].T


def fit_scale(lw, ww):
    return float(np.dot(lw, ww) / np.dot(lw, lw))


def analyse(wheel, lio, offset_x, offset_y, max_dt, min_speed, min_rate, dump=None):
    lio.sort()
    lio_t = [s[0] for s in lio]
    pairs = []
    for t, vx, vy, wz in wheel:
        i = bisect.bisect_left(lio_t, t)
        best = min(
            (j for j in (i - 1, i) if 0 <= j < len(lio)),
            key=lambda j: abs(lio_t[j] - t),
            default=None,
        )
        if best is None or abs(lio_t[best] - t) > max_dt:
            continue
        _, lvx, lvy, lwz = lio[best]
        # Same lever-arm compensation as the controller (LIO body -> chassis point).
        bx = lvx + lwz * offset_y
        by = lvy - lwz * offset_x
        pairs.append((vx, vy, wz, bx, by, lwz))
    if not pairs:
        return None
    p = np.array(pairs)
    if dump:
        np.savetxt(dump, p, delimiter=",", header="wheel_vx,wheel_vy,wheel_wz,lio_vx,lio_vy,lio_wz")
    moving = np.hypot(p[:, 3], p[:, 4]) > min_speed
    turning = np.abs(p[:, 5]) > min_rate
    result = {"pairs": int(len(p)), "moving": int(moving.sum()), "turning": int(turning.sum())}
    if moving.sum() >= 20:
        # Least squares W = L M^T  ->  M = (lstsq(L, W))^T
        L = p[moving][:, 3:5]
        W = p[moving][:, 0:2]
        M, keep = trimmed_fit(L, W, fit_matrix)
        residual = W[keep] - L[keep] @ M.T
        result["linear_outliers"] = int((~keep).sum())
        result["M"] = M.round(4).tolist()
        result["rotation_deg"] = math.degrees(math.atan2(M[1, 0] - M[0, 1], M[0, 0] + M[1, 1]))
        result["scale"] = float(math.sqrt(abs(np.linalg.det(M))))
        result["mirrored"] = bool(np.linalg.det(M) < 0)
        result["linear_rms"] = float(np.sqrt((residual**2).mean()))
    if turning.sum() >= 20:
        lw = p[turning][:, 5]
        ww = p[turning][:, 2]
        k, keep = trimmed_fit(lw, ww, fit_scale)
        result["wz_outliers"] = int((~keep).sum())
        result["wz_max_abs_wheel"] = float(np.abs(ww).max())
        result["wz_scale"] = k
        result["wz_rms"] = float(np.sqrt(((ww[keep] - k * lw[keep]) ** 2).mean()))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seconds", type=float, default=60.0)
    parser.add_argument("--wheel-topic", default="/sentry/chassis_velocity")
    parser.add_argument("--odom-topic", default="/aft_mapped_to_init")
    parser.add_argument("--lidar-offset-x", type=float, default=0.0)
    parser.add_argument("--lidar-offset-y", type=float, default=-0.20)
    parser.add_argument("--max-dt", type=float, default=0.01, help="max stamp gap between paired samples, s")
    parser.add_argument("--min-speed", type=float, default=0.2)
    parser.add_argument("--min-rate", type=float, default=0.3)
    parser.add_argument("--max-scale-error", type=float, default=0.1)
    parser.add_argument("--max-rotation-deg", type=float, default=10.0)
    parser.add_argument("--use-sim-time", action="store_true")
    parser.add_argument("--json", help="write the result here")
    parser.add_argument("--dump", help="write the time-matched pairs here (CSV)")
    args = parser.parse_args()

    rclpy.init()
    node = Recorder(args)
    end = time.monotonic() + args.seconds
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        wheel, lio = list(node.wheel), list(node.lio)
        node.destroy_node()
        rclpy.shutdown()

    print(f"samples: wheel={len(wheel)} lio={len(lio)}")
    result = analyse(
        wheel,
        lio,
        args.lidar_offset_x,
        args.lidar_offset_y,
        args.max_dt,
        args.min_speed,
        args.min_rate,
        args.dump,
    )
    if result is None:
        print("no time-matched samples; check topics, ROS_DOMAIN_ID and clocks")
        return 2
    print(json.dumps(result, indent=2))
    if args.json:
        with open(args.json, "w") as f:
            json.dump(result, f, indent=2)

    problems = []
    if "M" not in result:
        problems.append("not enough translation (drive faster or longer)")
    else:
        if result["mirrored"]:
            problems.append("translation axes are mirrored")
        if abs(result["rotation_deg"]) > args.max_rotation_deg:
            problems.append(f"translation axes rotated by {result['rotation_deg']:.1f} deg")
        if abs(result["scale"] - 1.0) > args.max_scale_error:
            problems.append(f"translation scale {result['scale']:.3f}")
    if "wz_scale" not in result:
        problems.append("not enough rotation to check the yaw rate")
    elif result["wz_scale"] < 0:
        problems.append("yaw rate sign is opposite to the LIO (REP-103) convention")
    elif abs(result["wz_scale"] - 1.0) > args.max_scale_error:
        problems.append(f"yaw rate scale {result['wz_scale']:.3f}")
    if problems:
        print("CHECK: " + "; ".join(problems))
        return 1
    print("CHECK: chassis velocity frame consistent with the controller")
    return 0


if __name__ == "__main__":
    sys.exit(main())
