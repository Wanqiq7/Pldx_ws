# MPC chassis drive wrench

> Historical design: the active controller has been restored to the qpOASES
> velocity-input MPC. This document describes the former force-control path
> and its corresponding firmware, not the current ROS navigation output.

The actuation path is MINCO `/opt_path` -> force-input MPC -> `/chassis_cmd_mpc`
-> ros2_libxr world-to-chassis rotation -> USB `nav_data` -> HostDataLink
`chassis_ff` -> DualBoard CAN 0x32C -> Omni wheel force allocation -> motor torque
and power limits. The ROS command contains only Header and Wrench. USB carries
exactly three little-endian float32 values in the order Fx, Fy, Mz (12 bytes).

MPC publishes the first input at the configured controller frequency, after any
output limits. It includes position/velocity feedback; disturbance compensation
is not included in the report model. Nav2's required
`computeVelocityCommands()` returns zero and has no actuator subscriber here.

The available MC02 `Omni.hpp` treats the navigation wrench as feedforward:
`DynamicInverseSolution()` adds chassis velocity PID and slope compensation to
Fx/Fy and adds yaw-rate PID to Mz; `CalculateMotorCurrent()` then adds wheel
speed PID and wheel resistance torque. The host sets navigation velocity targets
to zero, so these local loops can oppose the requested MPC force. This firmware
is therefore not an implementation of pure MPC force actuation. ROTOR and FOLLOW
also supply their own yaw targets. The actual deployed firmware version must be
checked separately; changing these control loops requires hardware validation.
The omni simulator receives `/sim/cmd_force` and bypasses its velocity servo for
navigation, so its force response does not reproduce those MC02 feedback loops.
Folded-passage alignment retains a local yaw torque loop using
planned heading and measured yaw rate; `passage_yaw_kp`, `passage_yaw_kd` and
`passage_max_torque` configure it. Manual passage controls remain operator inputs.

The USB bridge requires matching command/odometry frames and transforms force to
the chassis axes using LIO yaw plus `chassis_yaw_offset`. Configure the mounting
rotation before a hardware run. The model's position/velocity reference point must
be the chassis centre of mass. The physical parameters in sentry1.yaml correspond
to the MC02 settings: m=28 kg, Iz=0.58 kg*m², Fmax=56 N, Mmax=2.32 N*m.
The firmware's additional force cap is 79.20 N; its yaw cap is 2.32 N*m.

Cancel, BLOCK, goal completion and controller failure publish zero force. The
bridge also clears stale/invalid commands and stale heading. HostDataLink and the
chassis CAN receiver retain their watchdogs. Zero wrench removes the MPC
contribution; the available firmware may still apply PID and compensation terms.
Emergency disable still enters firmware RELAX.

Build ros_interfaces, minco_controller, ros2_libxr and navi2 together, then build and
flash both sentry_gimbal and sentry_chassis firmware. The ROS schema, USB payload
length and navigation CAN control payload have changed together. Wheel-velocity
telemetry is a separate chassis-to-host feedback stream.

Validation:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --build-base build-force --install-base install-force --symlink-install --packages-select ros_interfaces minco_controller ros2_libxr navi2 --executor sequential --cmake-args -DBUILD_TESTING=ON
source install-force/setup.bash
colcon test --build-base build-force --install-base install-force --packages-select minco_controller ros2_libxr
colcon test-result --test-result-base build-force --verbose
```

The isolated build directory avoids mixed copy/symlink caches in existing builds.
For hardware startup, source the normal workspace install first, then select the
rebuilt overlay (the base install must contain the other navigation packages):

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
NAV_INSTALL_DIR="$PWD/install-force" ./start.bash
```

The launcher inherits the base workspace environment and sources the force overlay. Do not mix generated ChassisCommand types from
different installs in a process.

In the firmware workspace, validate the HostDataLink protocol and DualBoard
feedforward codecs and build both board configurations sequentially; verify that
the firmware installed on both boards matches the validated source and protocol.
