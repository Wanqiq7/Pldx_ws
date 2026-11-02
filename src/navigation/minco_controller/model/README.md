# Report MPC and force output

`omnidirectional_report_tracking.py` is the report-model simulation and retains
its original Python trajectory dependencies in this directory. The ROS plugin
uses the native acados `ReportMpcSolver`: state `[x, y, yaw, vx, vy, omega]`,
input `[Fx, Fy, Mz]` and forward Euler dynamics. A 40-step fixture compares the
C++ result with the Python qpOASES result. The reference Python file itself
uses CasADi/qpOASES; the acados model is generated separately from its equations.

The build requires an acados installation with HPIPM and BLASFEO. CMake searches
the standard prefixes and `ACADOS_SOURCE_DIR`; use
`-DACADOS_INSTALL_DIR=/path/to/acados` for another installation. The checked-in
generated C sources target the matching acados API. Generation was verified with
acados_template 0.5.1 and CasADi 3.7.2; other acados versions may require
regeneration. To regenerate them with a matching installation, run:

```bash
ACADOS_SOURCE_DIR=/path/to/acados python3 src/navigation/minco_controller/model/generate_report_acados.py
```

The controller performs a first solve,
resamples the MINCO reference based on the predicted/reference velocity angle,
and performs a second solve. The first solution is retained if the second fails.
`report_q` and `report_r` are diagonal state and
physical force/torque cost weights; they need tuning for the robot.

The QP uses independent bounds on each force component, as in the Python model.
Before publishing, the planar force is additionally clipped to the smaller of
`max_force` and `friction_coeff * mass * 9.81`; the applied wrench can differ from the exact
QP optimum when both components are large. Delay prediction uses the wrench
actually published. MINCO, unlike the standalone figure-eight example, supplies
the online reference. The ROS adapter uses a 0.05 m/s threshold for its
low-speed angle/resampling rules and uses planner yaw/yaw-rate information at
low speed. These are integration choices, not claims of exact equivalence to
the standalone trajectory example. The report mode does not add the existing
slope-gravity or disturbance-observer terms.

Velocity bounds are hard constraints from the first predicted state onward. If
the measured velocity is too far outside a bound to recover in one step, the
optimization is infeasible and the controller publishes zero wrench; it does
not command active braking in that case.

The controller publishes the first force and torque to `/chassis_cmd_mpc` as
`ros_interfaces/msg/ChassisCommand`, in the local costmap global frame (normally
`camera_init`), with units N and N*m. The existing `ros2_libxr` bridge requires
that frame to match the Point-LIO odometry frame and rotates the wrench to the
calibrated chassis axes before sending `nav_data`. Nav2's velocity return is
zero. MC02's NAVIGATION mode applies the external translation and yaw wrench;
the available MC02 `Omni.hpp` also adds local chassis and wheel-speed PID outputs
and slope/resistance compensation. Thus the firmware does not currently apply
only the MPC wrench, even in NAVIGATION mode. ROTOR and FOLLOW additionally
specify local yaw targets. The deployed bridge and both boards must use the
matching 12-byte force protocol; see `docs/force_control/README.md`.

## State inputs and simulation

Point-LIO `/aft_mapped_to_init` supplies the measured pose and velocity. The
controller projects this state using the configured control delay and the last
applied input. Physical parameters are starting values and require calibration.

For simulation, use `./start_sim.sh` and validate navigation with
`scripts/check_sim_goal_accuracy.py`. The real robot model uses its CAD mass
and inertia. Simulation does not validate the USB link or hardware force path.
