# Recovery reference clock contract

Recovery references use the owning ROS node's clock, just like committed
normal trajectories and trajectory invalidation messages. With `use_sim_time`
enabled, their timestamps follow `/clock`; otherwise they follow the node's
normal ROS time source.

An independently constructed `rclcpp::Clock()` defaults to system time. Using
it for escape references in simulation produces future-dated commands. The
controller correctly rejects negative reference ages and publishes zero speed.
This can turn an intended escape into a stop until the recovery window expires.

The fix changes only the timestamp source in the escape and emergency-stop
publishers. Message flags, trajectory points, recovery decisions, collision
checks, controller freshness limits, topics and frames retain their behavior.
The emergency-stop message remains a blocking command.

## Automated checks

Run from the workspace root after sourcing ROS and the workspace:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=229 build/minco_planner/test_planning_budget \
    --gtest_filter=PlanningBudgetTest.RecoveryReferencesUseNodeClock
ROS_DOMAIN_ID=229 build/minco_controller/controller_freshness_test \
    --gtest_filter=ControllerFreshnessTest.ShortRecoveryReferenceHonorsSimulationClockAndFreshnessGuards
ROS_DOMAIN_ID=229 colcon test --parallel-workers 1 --event-handlers console_cohesion+
colcon test-result --verbose
```

The planner test subscribes to messages from the real escape and emergency-stop
publishers. It checks node-time bounds and point timestamps in normal time and
at simulated times 251 and 252 seconds, together with message flags and escape
motion. The emergency-stop test uses the existing constant-stop fallback without
initializing the backup optimizer; it tests publication metadata, not optimizer
performance.

The controller test supplies a short ten-point escape-shaped reference at
simulated time 251 seconds and observes positive velocity through the controller
interface. Future references, stale references and stale odometry still produce
zero velocity. New valid input at 252 seconds restores movement. This is a
controller boundary test; it does not connect a live planner to the controller.

## Simulation acceptance

Launch the original real robot configuration with its default moving gimbal:

```bash
ROS_DOMAIN_ID=228 GZ_PARTITION=recovery_clock_check ./start_sim.sh \
    robot_model:=real world:=rmul_2027_3v3 headless:=true rviz:=false
```

In another sourced terminal, run:

```bash
ROS_DOMAIN_ID=228 python3 scripts/check_sim_navigation_stress.py \
    log/recovery_clock_check --cycles 1 \
    --goals '[[1.25,2.0],[5.5,2.0]]' --stall-seconds 2
```

Use one Gazebo instance in the chosen Gazebo transport partition. A distinct ROS
domain alone does not isolate Gazebo transport. The probe waits for the Nav2
lifecycle to become active, sends action goals, and records physical truth,
commands, reference metadata and odometry age. It fails if the robot stays within
5 cm of its progress anchor for two simulated seconds while more than 0.3 m from
the target. Wall time separately bounds action completion. On failure it saves
the summary and requests cancellation.

## Ticket 01 verification (2026-11-17)

- Production changes: two recovery publisher timestamp assignments.
- Earlier red evidence: before the fix, both recovery publisher branches failed
  the fixed-simulation-time regression. The original navigation repro stopped
  2.96 m from the goal with reference stamp 1794903563.97 s at simulation 251.2 s.
- Expanded publisher clock test and five focused controller cases passed.
- Full workspace run: 21 packages, 186 reported tests, zero errors/failures,
  two skipped tests, using `colcon test-result` counts.
- New two-goal simulation run: 2/2 successful actions, no stall assertion,
  final truth errors 0.03888 m and 0.10736 m, no future reference samples,
  maximum sampled odometry age 0.14229 s.
- This particular new simulation run did not enter recovery. Recovery publication
  is covered by the publisher regression and execution by the controller test;
  the route check verifies the original trigger scenario still completes.

The timestamp audit also found independent clocks in global-search path metadata
and an A* visualization message. These label spatial search output and visualization;
the local seed uses waypoint geometry, and normal execution references receive
node timestamps at commit. Those values are not the recovery reference timestamps
checked here. They were not changed.

Moving-gimbal dynamics, fault-state observation publishing and long-run acceptance
remain separate work. Ticket 01 does not claim that the moving-gimbal failure
observed in the earlier long stress run has been fixed.
