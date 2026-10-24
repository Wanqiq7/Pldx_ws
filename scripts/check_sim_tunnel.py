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

import rclpy
from geometry_msgs.msg import Twist
from gz.msgs10.contacts_pb2 import Contacts
from gz.transport13 import Node as GzNode

from check_sim_gimbal import GimbalAccuracy
from check_sim_goal_accuracy import pose


def wheel_contact_is_obstacle(contact):
    """Accept wheel support within 30 degrees of vertical; reject missing normals."""
    if len(contact.normal) != len(contact.position):
        return True
    for normal in contact.normal:
        magnitude = math.sqrt(normal.x**2 + normal.y**2 + normal.z**2)
        if not math.isfinite(magnitude) or magnitude < 1e-6:
            return True
        if abs(normal.z) / magnitude < math.cos(math.pi / 6):
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
        self.transport = GzNode()
        self.contact_topics = []
        self.contact_callbacks = []
        for link in ET.parse(model).getroot().findall("model/link"):
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
                        wheel_obstacles.append({
                            "link": link_name, "phase": self.phase,
                            "collision1": contact.collision1.name,
                            "collision2": contact.collision2.name,
                            "position": [[p.x, p.y, p.z] for p in contact.position],
                            "normal": [[n.x, n.y, n.z] for n in contact.normal],
                            "depth": list(contact.depth),
                        })
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
        end = time.monotonic() + 180
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--model", required=True, type=Path)
    parser.add_argument("--physical-only", action="store_true",
                        help="Check mechanical passage only; does not certify autonomous navigation")
    args = parser.parse_args()
    rclpy.init()
    node = TunnelAccuracy(args.directory, args.model)
    result = {"passed": False, "entry": [12.3, 13.425], "exit": [14.35, 13.425],
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
            node.request_posture("normal")
            node.await_state("normal")
            result["restored"] = True
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
