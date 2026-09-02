# Simulation

This directory contains the Gazebo Sim integration used to exercise the
navigation stack in this workspace. It is intentionally limited to one
`red_standard_robot1` robot and does not include the source workspace's
navigation, vision, or referee algorithms.

Install the simulation-only STVL dependency, then build after sourcing Jazzy:

```bash
sudo apt install ros-jazzy-spatio-temporal-voxel-layer
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to rmu_gazebo_simulator
source install/setup.bash
```

Start the simulator with:

```bash
ros2 launch rmu_gazebo_simulator simulation.launch.py
```

For repeatable navigation tests or machines where Gazebo rendering competes
with Point-LIO/ROGMap, run the server without the Gazebo GUI:

```bash
ros2 launch rmu_gazebo_simulator simulation.launch.py use_rviz:=false headless:=true
```

`headless:=true` only removes the Gazebo GUI. It does not change sensor topics,
TF ownership, map frames, or the navigation configuration.

The default local odometry source is Point-LIO. The simulator converts
`/livox/lidar` to the Velodyne-style `/velodyne_points` input and runs
Point-LIO with simulation-specific IMU gravity, zero sensor extrinsics, and
its native TF output disabled. Gazebo odometry remains available only as
`/ground_truth/odom` for diagnostics.

The navigation TF ownership is:

```text
map
|-- camera_init -> body                 point_lio_odom
`-- sim_odom -> base_link               point_lio_odom
                 |-- base_footprint     robot_state_publisher
                 `-- ... -> front_mid360 robot_state_publisher
```

`map -> camera_init` preserves the complete Point-LIO alignment, including
roll and pitch. `map -> sim_odom` is the level spawn pose and
`sim_odom -> base_link` is the final planar navigation pose. The adapter uses
the complete `base_link -> front_mid360` extrinsic for pose, twist, and twist
covariance conversion before suppressing z, roll, and pitch navigation axes.
No Gazebo ground-truth pose is injected into this tree.

Point-LIO's registered clouds remain unchanged on their original topics. The
TF-based navigation cloud adapter publishes these simulation-only views:

| Point-LIO input | Navigation output | Consumer |
| --- | --- | --- |
| `/cloud_registered` (`camera_init`) | `/simulation/cloud_registered_nav` (`sim_odom`) | local/global STVL |
| `/cloud_registered_full` (`camera_init`) | `/simulation/cloud_registered_full_nav` (`sim_odom`) | ROGMap |

The adapter uses the timestamped `sim_odom <- camera_init` transform and
preserves every PointCloud2 field. It defers publication until the unchanged
cloud timestamp is transformable at the physical `front_mid360` raytrace
origin, avoiding a cloud/TF delivery race in STVL. Both STVL layers reject
returns below 0.3 m so the arena's elevated traversable floor is not marked as
an obstacle. The local decay is 0.6 s and the global decay is 1.5 s.

Gazebo runs immediately by default. To inspect the initial state before
physics starts, launch it paused:

```bash
ros2 launch rmu_gazebo_simulator simulation.launch.py start_paused:=true
```

After the default launch starts producing Point-LIO odometry, validate the TF
contract, compare it with Gazebo truth, and confirm that `/odom` has exactly
one publisher:

```bash
ros2 run sim_nav_adapter check_sim_tf.py
```

Check that the static-map-free region directly in front of the robot contains
at most 64 false lethal Global Costmap cells and no 8-connected false wall:

```bash
ros2 run sim_nav_adapter check_sim_costmap.py
```

The complete runtime acceptance performs three cold starts and navigation
runs to `(8.41, 14.2)`, checking the Costmap with ROG visualization both on
and off:

```bash
src/simulation/simulator/rmu_gazebo_simulator/test/check_sim_navigation_runtime.sh
```

The simulator publishes camera, camera-info, IMU, laser scan, Livox point
cloud, joint state, ground-truth odometry, and `/clock` topics using the root
topic names consumed by the navigation bringup.

## Navigation command contract

MINCO solves its planar velocity in `sim_odom` (the local navigation
frame), while the Gazebo `MecanumDrive2` plugin consumes velocity components in
the `base_link` frame. The simulation keeps these contracts separate from the
shared navigation and real-robot code:

```text
controller_server -> /simulation/controller_cmd_vel_global_raw
velocity_smoother -> /simulation/controller_cmd_vel_global_smoothed
behavior_server   -> /simulation/behavior_cmd_vel_body
                         |
                         v
                 sim_cmd_vel_adapter -> /cmd_vel -> Gazebo
```

The adapter rotates only the smoothed controller command with `R(-yaw)` from
the latest `/odom` pose. Spin, BackUp, and other Nav2 behaviors are already
expressed in `base_link` and pass through unchanged. A raw controller zero
command immediately suppresses any residual smoother command. Missing or
invalid commands and stale odometry fail closed after 0.20 s; the Gazebo
Mecanum plugin has an independent 0.25 s command watchdog.

The Point-LIO odometry adapter publishes `/odom` at the `base_link` origin, so
the simulation planner and controller set `lidar_offset_x/y` to zero. The
simulation controller frequency is 20 Hz to match the MINCO integration step
of 0.05 s; the velocity smoother remains a 100 Hz output interpolator.

`/cmd_vel_mpc` remains the world-frame command used by the real-robot
communication contract and is intentionally not connected to the simulation
chassis.
