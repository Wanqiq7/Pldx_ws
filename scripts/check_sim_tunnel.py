#!/usr/bin/env python3
"""Exercise the RMUC_2026 home-left roof using the running real simulation.

Start at the documented entrance test spawn (12.3, 13.425), yaw pi/4.
Contact sensors in the acceptance share use Gazebo's native sensor topic names.
Navigation and low-speed physical passage are reported separately.
"""

import argparse
import json
import math
from pathlib import Path
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from gz.msgs10.contacts_pb2 import Contacts
from gz.transport13 import Node as GzNode

from check_sim_gimbal import GimbalAccuracy
from check_sim_goal_accuracy import pose, stamp
from check_sim_ground_clearance import PACKAGE, load_mesh, measure


class TerrainContacts:
    """Resolve suspicious wheel normals against the unchanged RMUC2026 mesh."""

    def __init__(self, triangles):
        self.triangles = np.asarray(triangles)
        self.minimum = self.triangles.min(axis=1)
        self.maximum = self.triangles.max(axis=1)
        normals = np.cross(self.triangles[:, 1] - self.triangles[:, 0],
                           self.triangles[:, 2] - self.triangles[:, 0])
        lengths = np.linalg.norm(normals, axis=1)
        self.normals = normals / np.maximum(lengths[:, None], 1e-30)

    # Contact points can lie outside the mesh by the two 1 mm collision margins.
    # Include 0.1 mm for float32 contact coordinates and the solver tolerance.
    def nearby_indices(self, point, tolerance=0.0021):
        point = np.asarray(point)
        candidates = np.all(self.minimum - tolerance <= point, axis=1)
        candidates &= np.all(self.maximum + tolerance >= point, axis=1)
        indices = np.flatnonzero(candidates)
        triangles = self.triangles[indices]
        if not len(triangles):
            return indices
        a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
        ab, ac, ap = b - a, c - a, point - a
        aa = np.einsum("ij,ij->i", ab, ab)
        bb = np.einsum("ij,ij->i", ac, ac)
        cross = np.einsum("ij,ij->i", ab, ac)
        along_b = np.einsum("ij,ij->i", ap, ab)
        along_c = np.einsum("ij,ij->i", ap, ac)
        determinant = aa * bb - cross**2
        u = (bb * along_b - cross * along_c) / np.maximum(determinant, 1e-30)
        v = (aa * along_c - cross * along_b) / np.maximum(determinant, 1e-30)
        projected = a + u[:, None] * ab + v[:, None] * ac
        inside = (determinant > 1e-24) & (u >= 0) & (v >= 0) & (u + v <= 1)
        distance = np.where(inside, np.linalg.norm(projected - point, axis=1), np.inf)
        for start, end in ((a, b), (b, c), (c, a)):
            edge = end - start
            fraction = np.clip(np.einsum("ij,ij->i", point - start, edge)
                               / np.maximum(np.einsum("ij,ij->i", edge, edge), 1e-30), 0, 1)
            distance = np.minimum(distance,
                                  np.linalg.norm(start + fraction[:, None] * edge - point, axis=1))
        return indices[distance <= tolerance]

    def nearby(self, point, tolerance=0.0021):
        return self.normals[self.nearby_indices(point, tolerance)]

    def is_support(self, point):
        indices = self.nearby_indices(point)
        support = indices[abs(self.normals[indices, 2]) >= math.cos(math.pi / 6)]
        if not len(support):
            return False
        for index in indices:
            if index in support:
                continue
            # The field mesh includes thin floor markings. Accept their side faces
            # only if the entire face stays within 2 mm of a nearby support plane.
            if not any(np.max(abs((self.triangles[index] - self.triangles[s, 0])
                                  @ self.normals[s])) <= 0.002 for s in support):
                return False
        return True


def platform_restore_allowed(position, velocity, wheels):
    """Require the complete chassis beyond the crest, on the upper level deck."""
    values = [*position, *velocity, *(w[k] for w in wheels
                                    for k in ("gap_m", "slope_deg", "terrain_z"))]
    return (all(math.isfinite(v) for v in values) and len(wheels) == 4
            and position[0] >= 15.8 and abs(position[1] - 13.475) <= 0.12
            and math.hypot(*velocity[:2]) <= 0.02 and abs(velocity[2]) <= 0.02
            and all(-0.003 <= w["gap_m"] <= 0.01 and w["slope_deg"] <= 3.0
                    and w["terrain_z"] >= 0.10 for w in wheels))


def wheel_contact_is_obstacle(contact, terrain=None):
    """Accept wheel support within 30 degrees of vertical; reject missing normals."""
    if len(contact.normal) != len(contact.position):
        return True
    for point, normal in zip(contact.position, contact.normal):
        magnitude = math.sqrt(normal.x**2 + normal.y**2 + normal.z**2)
        if not math.isfinite(magnitude) or magnitude < 1e-6:
            return True
        if abs(normal.z) / magnitude < math.cos(math.pi / 6):
            if terrain is None or not terrain.is_support([point.x, point.y, point.z]):
                return True
    return False


class TunnelAccuracy(GimbalAccuracy):
    def __init__(self, directory, model):
        super().__init__(directory)
        self.goal_timeout = 65.0
        self.contacts_lock = threading.Lock()
        self.contact_messages = 0
        self.wheel_contact_points = 0
        self.upper_contacts = []
        self.wheel_obstacle_contacts = []
        self.wheel_support_normal_anomalies = 0
        self.wheel_support_examples = []
        self.model = ET.parse(model).getroot().find("model")
        self.terrain = TerrainContacts(load_mesh(
            PACKAGE / "resource/models/rmuc_2026/meshes/rmuc_2026.stl"))
        self.transport = GzNode()
        self.contact_topics = []
        self.contact_callbacks = []
        for link in self.model.findall("link"):
            name = link.get("name")
            if not link.findall("collision"):
                continue
            topic = (
                f"/world/default/model/sentry/link/{name}"
                "/sensor/acceptance_contacts/contact"
            )

            def receive(message, link_name=name):
                entries = []
                wheel_obstacles = []
                count = 0
                for contact in message.contact:
                    if not contact.position:
                        continue
                    count += len(contact.position)
                    if link_name.endswith("_Wheel") and wheel_contact_is_obstacle(contact):
                        entry = {
                            "link": link_name, "phase": self.phase,
                            "collision1": contact.collision1.name,
                            "collision2": contact.collision2.name,
                            "position": [[p.x, p.y, p.z] for p in contact.position],
                            "normal": [[n.x, n.y, n.z] for n in contact.normal],
                            "depth": list(contact.depth),
                        }
                        # Only the known terrain collision may be reclassified as support.
                        terrain_contact = any(c.name == "rmuc_2026::body::collision"
                                              for c in (contact.collision1, contact.collision2))
                        if terrain_contact and not wheel_contact_is_obstacle(contact, self.terrain):
                            with self.contacts_lock:
                                self.wheel_support_normal_anomalies += 1
                                if len(self.wheel_support_examples) < 20:
                                    self.wheel_support_examples.append(entry)
                        else:
                            wheel_obstacles.append(entry)
                    if not link_name.endswith("_Wheel"):
                        truth = self.gt[-1].pose.pose if self.gt else None
                        entries.append({
                            "link": link_name,
                            "phase": self.phase,
                            "collision1": contact.collision1.name,
                            "collision2": contact.collision2.name,
                            "position": [[p.x, p.y, p.z] for p in contact.position],
                            "depth": list(contact.depth),
                            "truth_position": [truth.position.x, truth.position.y, truth.position.z]
                            if truth else None,
                            "truth_orientation_xyzw": [truth.orientation.x, truth.orientation.y,
                                                       truth.orientation.z, truth.orientation.w]
                            if truth else None,
                        })
                with self.contacts_lock:
                    self.contact_messages += 1
                    if link_name.endswith("_Wheel"):
                        self.wheel_contact_points += count
                    self.upper_contacts.extend(entries)
                    self.wheel_obstacle_contacts.extend(wheel_obstacles)

            self.contact_callbacks.append(receive)
            if not self.transport.subscribe(Contacts, topic, receive):
                raise RuntimeError("contact subscription failed: " + topic)
            self.contact_topics.append(topic)

    def drive(self, target_x, speed=0.10, stop_on_contact=True):
        start = pose(self.gt[-1].pose.pose)
        direction = 1 if target_x > start[0] else -1
        end = time.monotonic() + 300
        last_progress_x = start[0]
        last_progress_time = self.get_clock().now().nanoseconds * 1e-9
        with self.contacts_lock:
            initial_contacts = len(self.upper_contacts) + len(self.wheel_obstacle_contacts)
        while True:
            current = pose(self.gt[-1].pose.pose)
            if direction * (target_x - current[0]) <= 0:
                return {"reached": True, "start": start, "finish": current}
            with self.contacts_lock:
                hit = len(self.upper_contacts) + len(self.wheel_obstacle_contacts) > initial_contacts
            if stop_on_contact and hit:
                return {"reached": False, "reason": "obstacle_contact", "finish": current}
            if self.faults:
                return {"reached": False, "reason": "gimbal_fault", "finish": current}
            if time.monotonic() > end:
                return {"reached": False, "reason": "drive_timeout", "finish": current}
            now = self.get_clock().now().nanoseconds * 1e-9
            if abs(current[0] - last_progress_x) > 0.01:
                last_progress_x, last_progress_time = current[0], now
            elif now - last_progress_time > 3.0:
                return {"reached": False, "reason": "no_progress", "finish": current}
            if abs(current[1] - 13.425) > 0.18:
                return {"reached": False, "reason": "lateral_deviation", "finish": current}
            yaw = self.tf("map", "base_link", rclpy.time.Time())["pose"][3]
            front = yaw + self.gimbal["front_yaw_in_chassis"]
            front = math.atan2(math.sin(front), math.cos(front))
            command = Twist()
            command.linear.x = direction * speed
            command.linear.y = max(-0.025, min(0.025, -(current[1] - 13.425)))
            command.angular.z = max(-0.3, min(0.3, -1.5 * front))
            self.passage_pub.publish(command)
            rclpy.spin_once(self, timeout_sec=0.02)

    def stop(self):
        for _ in range(12):
            self.passage_pub.publish(Twist())
            self.spin_for(0.03)

    def platform_support(self):
        if not self.gt:
            return []
        wheels = measure(self.model, self.terrain.triangles, self.gt[-1].pose.pose)["wheels"]
        for wheel in wheels:
            point = [*wheel["support_xyz"][:2], wheel["terrain_z"]]
            normals = self.terrain.nearby(point)
            wheel["slope_deg"] = max((math.degrees(math.acos(min(1.0, abs(n[2]))))
                                      for n in normals), default=90.0)
        return wheels

    def restore_on_platform(self):
        self.phase = "platform_hold"
        stable_since = None
        evidence = {}

        def ready():
            nonlocal stable_since, evidence
            truth = self.gt[-1]
            position = pose(truth.pose.pose)
            velocity = [truth.twist.twist.linear.x, truth.twist.twist.linear.y,
                        truth.twist.twist.angular.z]
            now = truth.header.stamp.sec + truth.header.stamp.nanosec * 1e-9
            wheels = self.platform_support()
            eligible = platform_restore_allowed(position, velocity, wheels)
            if not eligible:
                stable_since = None
                return False
            if stable_since is None:
                stable_since = now
            evidence = dict(position=position, velocity=velocity, wheels=wheels,
                            stable_seconds=now - stable_since, sim_time_s=now)
            return now - stable_since >= 1.0

        self.wait(ready, 15)
        print(json.dumps({"platform_restore_gate": evidence}), flush=True)
        self.request_posture("normal")
        self.await_state("normal")
        return evidence

    def hold_on_platform(self, seconds=5.0):
        self.phase = "exit_hold"
        origin = pose(self.gt[-1].pose.pose)[:2]
        start = stamp(self.gt[-1].header.stamp)
        result = dict(drift_m=0.0, max_speed_mps=0.0, max_yaw_rate_radps=0.0)

        def held():
            truth = self.gt[-1]
            position = pose(truth.pose.pose)[:2]
            velocity = truth.twist.twist
            values = [math.dist(position, origin), math.hypot(velocity.linear.x, velocity.linear.y),
                      abs(velocity.angular.z)]
            assert all(math.isfinite(v) for v in values), "non-finite platform measurement"
            for key, value in zip(("drift_m", "max_speed_mps", "max_yaw_rate_radps"), values):
                result[key] = max(result[key], value)
            result["sim_seconds"] = stamp(truth.header.stamp) - start
            return result["sim_seconds"] >= seconds + 0.2

        self.wait(held, max(60.0, seconds * 20))
        result["passed"] = all(result[key] <= 0.02
                               for key in ("drift_m", "max_speed_mps", "max_yaw_rate_radps"))
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--model", required=True, type=Path)
    parser.add_argument("--physical-only", action="store_true",
                        help="Check mechanical passage only; does not certify autonomous navigation")
    args = parser.parse_args()
    rclpy.init()
    node = TunnelAccuracy(args.directory, args.model)
    result = {"passed": False, "entry": [12.3, 13.425], "exit": [16.0, 13.425],
              "scope": "physical_passage" if args.physical_only else "navigation_and_physical_passage"}
    try:
        node.ready()
        node.request_posture("normal")
        node.await_state("normal")
        start = pose(node.gt[-1].pose.pose)
        assert math.dist(start[:2], result["entry"]) < 0.15, start
        node.request_posture("folded")
        node.await_state("folded")
        with node.contacts_lock:
            assert node.contact_messages and node.wheel_contact_points, "contact sensors inactive"
            assert not node.upper_contacts and not node.wheel_obstacle_contacts, "obstacle contact before passage"
        if args.physical_only:
            result["navigation"] = {"skipped": True}
        else:
            node.phase = "tunnel_navigation"
            target = [13.65, 13.425]
            status, stamp = node.send_and_wait(target)
            node.spin_for(0.4)
            node.snapshot("tunnel_navigation_result", target, status, at_time=stamp)
            result["navigation"] = {"passed": status == 4, "status": status, "target": target}
            print(json.dumps({"navigation": result["navigation"]}), flush=True)
        node.phase = "physical_passage"
        result["physical_passage"] = node.drive(result["exit"][0])
        node.stop()
        print(json.dumps({"physical_passage": result["physical_passage"]}), flush=True)
        if result["physical_passage"]["reached"]:
            node.phase = "restore_after_exit"
            result["platform_restore_gate"] = node.restore_on_platform()
            result["restored"] = True
            result["exit_hold"] = node.hold_on_platform()
        else:
            node.phase = "back_out"
            result["back_out"] = node.drive(result["entry"][0], speed=0.06, stop_on_contact=False)
            node.stop()
            if result["back_out"]["reached"]:
                node.request_posture("normal")
                node.await_state("normal")
                result["restored"] = True
        result["passed"] = bool(
            (args.physical_only or result["navigation"]["passed"])
            and result["physical_passage"]["reached"]
            and result.get("restored")
            and result.get("exit_hold", {}).get("passed")
            and not node.upper_contacts
            and not node.wheel_obstacle_contacts
            and not node.faults
        )
    except Exception as error:
        result["error"] = f"{type(error).__name__}: {error}"
        raise
    finally:
        try:
            node.stop()
        finally:
            with node.contacts_lock:
                result.update(
                    contact_messages=node.contact_messages,
                    wheel_contact_points=node.wheel_contact_points,
                    non_wheel_contacts=node.upper_contacts,
                    wheel_obstacle_contacts=node.wheel_obstacle_contacts,
                    wheel_support_normal_anomalies=node.wheel_support_normal_anomalies,
                    wheel_support_examples=node.wheel_support_examples,
                )
            result.update(faults=node.faults, stages=node.stages, records=node.records)
            (args.directory / "result.json").write_text(json.dumps(result, indent=2) + "\n")
            node.samples.close()
            for topic in node.contact_topics:
                node.transport.unsubscribe(topic)
            node.destroy_node()
            rclpy.shutdown()
    print(json.dumps({"passed": result["passed"]}), flush=True)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
