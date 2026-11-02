# MincoMpcController

Nav2 controller plugin for the omni chassis. MINCO supplies `/opt_path`; the
controller solves a force-input MPC and publishes the first optimized drive wrench
on `/chassis_cmd_mpc` (`ros_interfaces/msg/ChassisCommand`).

The state is `[x, y, yaw, vx, vy, omega]`; inputs are world-frame
`[Fx, Fy, Mz]`. The acados solver uses the forward Euler model, state tracking
cost and component-wise input and velocity bounds from
`model/omnidirectional_report_tracking.py`. It solves twice, resampling the
reference after the first prediction. The published planar force is additionally
limited by magnitude. Nav2 speed limits tighten the prediction bounds.

The default control frequency is 100 Hz, prediction step 0.02 s, horizon 0.8 s.
Point-LIO supplies measured state; lever-arm compensation and a configurable
control delay project that state to the actuation time. Yaw comes from LIO rather
than the yaw-cancelled Nav2 proxy frame.

The ROS command contains a stamp, a global frame and a wrench in N/N*m.
The bridge rotates this wrench to the calibrated chassis axes before USB output.
`computeVelocityCommands()` remains the required Nav2 API and returns zero;
the navigation command is supplied through the wrench publisher. The checked-in
MC02 firmware module adds this force to its local velocity and wheel-speed PID
outputs, so the commanded wrench is not the entire motor output. Prediction
velocities remain internal state and visualization data.

Cancel, BLOCK, goal completion, stale odometry/reference and failed solves publish
zero wrench. Reference replacement during a solve discards the in-flight command.
Message stamps and steady-clock receipt intervals both participate in freshness
checks. The performance CSV records `cmd_fx`, `cmd_fy`, `cmd_mz` in N/N*m.

Configuration: `src/navigation/navi2_bringup/params/sentry1.yaml`,
`controller_server.ros__parameters.FollowPath`. The configured physical model is
28 kg, Iz=0.58 kg*m², 56 N and 2.32 N*m; these match the MC02 model/acceleration
settings and require hardware calibration. The transport and firmware behavior
are described in [force control integration](../../../docs/force_control/README.md).

The model, acados dependency and code generation are
described in [report model](model/README.md). Validate the force path and physical
parameters on the robot before operation.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select ros_interfaces minco_controller ros2_libxr navi2 --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select minco_controller ros2_libxr
colcon test-result --verbose
```
