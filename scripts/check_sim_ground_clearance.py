#!/usr/bin/env python3
"""Measure wheel collision support points against the original terrain STL.

Run with the simulation ROS_DOMAIN_ID after settling. A gap above --max-gap
fails the check, catching physics collision margins that make the robot float.
"""

import argparse
import json
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation

PACKAGE = Path(__file__).resolve().parents[1] / "src/nav-simulation/sentry_simulation"
sys.path.insert(0, str(PACKAGE))
sys.path.insert(0, str(PACKAGE / "resource/models/sentry_real"))
from generate_collisions import load_mesh
from sentry_simulation.folding import frame_pose, transform


def terrain_heights(triangles, xy):
    keep = np.all(triangles[:, :, :2].min(axis=1) <= xy, axis=1)
    keep &= np.all(triangles[:, :, :2].max(axis=1) >= xy, axis=1)
    t = triangles[keep]
    a, b, p = t[:, 1, :2] - t[:, 0, :2], t[:, 2, :2] - t[:, 0, :2], xy - t[:, 0, :2]
    det = a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]
    keep = abs(det) > 1e-14
    t, a, b, p, det = t[keep], a[keep], b[keep], p[keep], det[keep]
    u = (p[:, 0] * b[:, 1] - p[:, 1] * b[:, 0]) / det
    v = (a[:, 0] * p[:, 1] - a[:, 1] * p[:, 0]) / det
    keep = (u >= -1e-8) & (v >= -1e-8) & (u + v <= 1 + 1e-8)
    return (t[:, 0, 2] + u * (t[:, 1, 2] - t[:, 0, 2])
            + v * (t[:, 2, 2] - t[:, 0, 2]))[keep]


def measure(model, terrain, body_pose):
    root = np.eye(4)
    p, q = body_pose.position, body_pose.orientation
    root[:3, 3] = [p.x, p.y, p.z]
    root[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    wheels = []
    for link in model.findall("link"):
        if not link.get("name").endswith("_Wheel"):
            continue
        collision = link.find("collision")  # CAD generator emits tread first.
        cylinder = collision.find("geometry/cylinder")
        frame = (root @ frame_pose(model, link.get("name"))
                 @ transform(collision.findtext("pose")))
        axle = frame[:3, 2]
        down = np.array([0., 0., -1.])
        radial = down - axle * np.dot(down, axle)
        radial /= np.linalg.norm(radial)
        support = (frame[:3, 3] + radial * float(cylinder.findtext("radius"))
                   + axle * np.sign(np.dot(down, axle))
                   * float(cylinder.findtext("length")) / 2)
        heights = terrain_heights(terrain, support[:2])
        # Select the supporting surface below this wheel, allowing small margins.
        floor = float(max(heights[heights < support[2] + 0.04]))
        wheels.append(dict(name=link.get("name"), support_xyz=support.tolist(),
                           terrain_z=floor, gap_m=float(support[2] - floor)))
    assert len(wheels) == 4, "expected four CAD tread cylinders"
    return dict(root_xyz=root[:3, 3].tolist(), wheels=wheels)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--max-gap", type=float, default=0.003)
    args = parser.parse_args()
    model = ET.parse(PACKAGE / "resource/models/sentry_real/model.sdf").getroot().find("model")
    terrain = load_mesh(PACKAGE / "resource/models/rmuc_2026/meshes/rmuc_2026.stl")
    rclpy.init()
    node = rclpy.create_node("wheel_ground_clearance_probe")
    truth = []
    node.create_subscription(Odometry, "/sim/ground_truth/odom", truth.append,
                             qos_profile_sensor_data)
    try:
        deadline = time.monotonic() + 10
        while len(truth) < 10 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        assert truth, "no simulation ground truth"
        result = measure(model, terrain, truth[-1].pose.pose)
        result["max_gap_m"] = args.max_gap
        result["passed"] = all(-0.002 <= w["gap_m"] <= args.max_gap
                               for w in result["wheels"])
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result), flush=True)
        return 0 if result["passed"] else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
