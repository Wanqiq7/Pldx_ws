#!/usr/bin/env python3
"""Repeat long simulation routes, retaining truth/TF samples and ROS warnings.

Source ROS and the workspace, then use the simulation's ROS_DOMAIN_ID.
Stops at the first failed leg and always writes summary.json. Samples and
action snapshots reuse check_sim_goal_accuracy.py's synchronized measurements.
"""

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from rcl_interfaces.msg import Log
from visualization_msgs.msg import Marker

from check_sim_goal_accuracy import Accuracy, stamp


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--cycles", type=int, default=5)
    parser.add_argument("--goal-timeout", type=float, default=90.0,
                        help="wall-clock timeout per navigation action, in seconds")
    parser.add_argument("--goals", default="[[24.0,7.3],[4.6,7.3]]")
    parser.add_argument("--launch-log", type=Path,
                        help="also count planner stdout diagnostics omitted from /rosout")
    parser.add_argument("--max-localization-error", type=float, default=0.12)
    parser.add_argument("--max-goal-error", type=float, default=0.15)
    args = parser.parse_args()
    goals = json.loads(args.goals)
    if (not math.isfinite(args.goal_timeout) or args.goal_timeout <= 0
            or args.cycles < 1 or not goals or any(
        len(goal) != 2 or not all(math.isfinite(x) for x in goal) for goal in goals
    )):
        parser.error("provide positive cycles/timeout and finite [x, y] goals")

    rclpy.init()
    node = Accuracy(args.directory, goal_timeout=args.goal_timeout)
    warnings = (args.directory / "rosout.jsonl").open("w", buffering=1)
    candidate_stream = (args.directory / "candidate_status.jsonl").open("w", buffering=1)
    errors = []
    candidate_events = []
    previous_candidate_status = None
    legs = []
    failure = None
    started = time.monotonic()
    launch_line_offset = len(args.launch_log.read_text().splitlines()) if args.launch_log else 0

    def sample_metrics():
        node.samples.flush()
        errors_xy = []
        distance = 0.0
        previous = None
        for line in (args.directory / "samples.jsonl").read_text().splitlines():
            record = json.loads(line)
            errors_xy.append(record["localization_error"])
            if previous:
                distance += math.dist(record["truth"][:2], previous["truth"][:2])
            previous = record
        errors_xy.sort()
        return dict(sample_count=len(errors_xy), truth_distance_m=distance,
                    localization_max_m=errors_xy[-1] if errors_xy else None,
                    localization_p95_m=errors_xy[int((len(errors_xy) - 1) * .95)] if errors_xy else None)

    def log(message):
        if message.level < Log.WARN:
            return
        event = dict(t=stamp(message.stamp), node=message.name,
                     level=message.level, message=message.msg, phase=node.phase)
        if node.latest:
            event["truth"] = node.latest["truth"]
            event["localization_error"] = node.latest["localization_error"]
        warnings.write(json.dumps(event) + "\n")
        if message.level >= Log.ERROR:
            errors.append(event)

    node.create_subscription(Log, "/rosout", log, 100)

    def candidate(message):
        nonlocal previous_candidate_status
        if message.ns != "minco_candidate_status":
            return
        status = message.text if message.action == Marker.ADD else "CLEARED"
        if status == previous_candidate_status:
            return
        previous_candidate_status = status
        event = dict(t=stamp(message.header.stamp), frame=message.header.frame_id,
                     status=status, phase=node.phase,
                     candidate_start=[message.pose.position.x, message.pose.position.y])
        if node.latest:
            event["truth"] = node.latest["truth"]
            event["localization_error"] = node.latest["localization_error"]
        candidate_events.append(event)
        candidate_stream.write(json.dumps(event) + "\n")

    node.create_subscription(Marker, "/minco_control_points_vis", candidate, 50)

    def save():
        summary = dict(cycles_requested=args.cycles, route=goals, legs=legs,
                       completed_legs=sum(leg["success"] for leg in legs),
                       elapsed_wall_seconds=time.monotonic() - started,
                       ros_errors=errors, candidate_status_transitions=candidate_events,
                       failure=failure)
        if args.launch_log:
            lines = args.launch_log.read_text().splitlines()[launch_line_offset:]
            patterns = {"collision": "validateTrajectory: collision detected",
                        "dynamics": "validateTrajectory: severe dynamics violation",
                        "rejected": "Trajectory validation failed",
                        "yaw_fallback": "Yaw optimization failed",
                        "large_tracking_error": "Large tracking error"}
            summary["planner_stdout_counts"] = {
                key: sum(pattern in line for line in lines) for key, pattern in patterns.items()
            }
            summary["launch_log"] = str(args.launch_log)
            summary["launch_log_start_line"] = launch_line_offset + 1
        (args.directory / "summary.json").write_text(json.dumps(summary, indent=2))

    try:
        node.ready()
        for cycle in range(1, args.cycles + 1):
            for index, goal in enumerate(goals, 1):
                before = len(node.records)
                start_t = node.get_clock().now().nanoseconds * 1e-9
                leg = dict(cycle=cycle, leg=index, target=goal, start_t=start_t,
                           start_truth=node.latest["truth"], success=False)
                legs.append(leg)
                print(json.dumps(dict(event="start", **leg)), flush=True)
                save()
                try:
                    node.navigate(goal)
                    records = node.records[before:]
                    result = next(r for r in records if r["label"] == "action_result")
                    settled = next(r for r in records if r["label"] == "settled")
                    leg.update(status=result["status"], truth_error=settled["truth_error"],
                               localization_error=settled["localization_error"])
                    assert settled["truth_error"] <= args.max_goal_error, "goal accuracy exceeded"
                    assert settled["localization_error"] <= args.max_localization_error, "localization error exceeded"
                    leg["success"] = True
                except Exception as error:
                    leg["error"] = str(error) or type(error).__name__
                    raise
                finally:
                    leg["end_t"] = node.get_clock().now().nanoseconds * 1e-9
                    leg["records"] = node.records[before:]
                    print(json.dumps(dict(event="finish", **leg)), flush=True)
                    save()
        metrics = sample_metrics()
        assert metrics["localization_max_m"] is not None, "no synchronized pose samples"
        assert metrics["localization_max_m"] <= args.max_localization_error, "in-motion localization error exceeded"
        print("PASS navigation loop", flush=True)
    except (Exception, KeyboardInterrupt) as error:
        failure = str(error) or type(error).__name__
        raise
    finally:
        save()
        summary_path = args.directory / "summary.json"
        summary = json.loads(summary_path.read_text())
        summary["sample_metrics"] = sample_metrics()
        summary_path.write_text(json.dumps(summary, indent=2))
        warnings.close()
        candidate_stream.close()
        node.samples.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
