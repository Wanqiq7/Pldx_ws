# MincoMpcController

Nav2 controller plugin for the omni chassis. MINCO supplies `/opt_path`; the
controller solves a force-input MPC and publishes the first optimized drive wrench
on `/chassis_cmd_mpc` (`ros_interfaces/msg/ChassisCommand`).

The state is `[x, y, yaw, vx, vy, omega]`; inputs are `[Fx/m, Fy/m, Mz/Iz]`.
Translation and yaw are solved as separate qpOASES QPs with exact double-integrator
discretization. Position/velocity tracking costs distinguish along-track and
cross-track errors. Input differences are penalized; `r_jerk` is a soft cost.
The translation input respects an octagon inside the friction/force circle; yaw
respects `max_torque`. Velocity constraints recover gradually from an initially
out-of-range measured speed. Nav2 speed limits tighten the prediction bounds;
absolute limits use a conservative inscribed square.

The default control frequency is 100 Hz, prediction step 0.02 s, horizon 0.8 s.
Point-LIO supplies measured state; lever-arm compensation and a configurable
control delay project that state to the actuation time. Yaw comes from LIO rather
than the yaw-cancelled Nav2 proxy frame. Optional wheel odometry integrates the
pose to now. Optional slope gravity and the force-input disturbance observer add
external acceleration to the model; the optimized force compensates it.

The ROS command contains a stamp, a global frame and a wrench in N/N*m.
The bridge rotates this wrench to the calibrated chassis axes before USB output.
`computeVelocityCommands()` remains the required Nav2 API and returns zero;
actuation is supplied through the wrench publisher. Prediction velocities remain
internal state and visualization data.

Cancel, BLOCK, goal completion, stale odometry/reference and failed solves publish
zero wrench. Reference replacement during a solve discards the in-flight command.
Message stamps and steady-clock receipt intervals both participate in freshness
checks. The performance CSV records `cmd_fx`, `cmd_fy`, `cmd_mz` in N/N*m.

Configuration: `src/navigation/navi2_bringup/params/sentry1.yaml`,
`controller_server.ros__parameters.FollowPath`. The configured physical model is
28 kg, Iz=0.58 kg*m², 56 N and 2.32 N*m; these match the MC02 model/acceleration
settings and require hardware calibration. The transport and firmware behavior
are described in [force control integration](../../../docs/force_control/README.md).

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select ros_interfaces minco_controller ros2_libxr navi2 --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select minco_controller ros2_libxr
colcon test-result --verbose
```
