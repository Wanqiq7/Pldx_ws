# Physical chassis state (#7)

`chassis_state` converts `/aft_mapped_to_init` (`camera_init` / `body`, IMU-frame
linear and angular velocity) plus `/sentry/yaw_joint_state` into `/chassis/odom`
(`camera_init` / `base_link`, chassis-frame twist). The original LIO message and
point clouds are unchanged. Only the big Yaw may move relative to the chassis;
lock the other mechanisms during this first-stage validation.

The geometry is `B_T_Y(q) = [R_axis Rz(direction*q + zero), pivot_xyz]` and a
fixed full 3D `Y_T_I`. All distances are metres, all angles radians. Joint zero
is an installation correction **after** the firmware's motor zero, not a second
copy of that motor offset. Direction applies to both angle and rate.

The converter rotates IMU velocities into the world, subtracts joint angular
velocity about the configured axis, then subtracts both rigid-body lever-arm
terms. It finally expresses twist in `base_link`. Pose difference is published
separately on `/chassis/pose_difference` in `camera_init`, timestamped at the
interval midpoint; it never feeds the controller.

Joint samples bracket each LIO measurement timestamp. Linear interpolation uses
the shortest wrapped angle increment (successive measured angles must differ by
less than pi) and interpolates the same-sample angular rates. Out-of-order samples
within the 5 s history are inserted by measurement time. A backwards jump beyond
that history clears it. Pending LIO messages wait for the next joint sample; data
outside the history is discarded. There is no extrapolation or receive-time
restamping. The queue is bounded to 2000 LIO messages. These are synchronization
rules, not a control stop state machine; the existing consumers retain their
existing timeout handling.

TF ownership with the production launch:

| Parent | Child | Publisher |
| --- | --- | --- |
| camera_init | base_link | chassis_state, dynamic, LIO stamp |
| base_link | lidar_yaw_mount | chassis_state, dynamic, same stamp |
| lidar_yaw_mount | body | chassis_state, static installation |
| lidar_yaw_mount | vision_gimbal | chassis_state, static CAD small_pitch, other joints locked |
| body | livox_frame | single_livox_pointlio launch, static IMU/LiDAR extrinsic |
| base_link | base_link_fake | chassis_state, same stamp, inverse chassis rotation |

Thus `camera_init → base_link_fake` has identity orientation, including when the
physical chassis is tilted. A `map → camera_init` correction still rotates that
virtual frame in `map`. Disable the old `fake_vel_transform` publisher and the
Point-LIO `publish.tf_send_en` before enabling this chain.

## Run and verify

```bash
source /opt/ros/jazzy/setup.bash
source install-link/setup.bash
# Run live validation isolated from any vehicle controllers/TF publishers.
ROS_DOMAIN_ID=171 ros2 launch chassis_state chassis_state.launch.py
python3 -m unittest discover -s src/navigation/chassis_state/test -p 'test_kinematics.py' -v
python3 src/navigation/chassis_state/test/test_ros_replay.py
```

`config/cad_initial.yaml` reproduces the existing CAD zero profile: chassis-to-Yaw
pivot z=0.055 m, installation yaw=-pi/4, and the full tilted IMU transform. It is an
initial estimate, not a bench calibration. Motor zero, positive direction and
all geometry require #11 measurement. Isolated validation can supply another
`geometry:=/path/to/parameters.yaml`; `publish_tf:=false` as a node parameter
supports odometry-only comparison.

Numerical tests use independently differentiated forward sensor poses at ±10 us
and require error below 1e-8 for twist and 1e-9 for pose. Public ROS replay permits
1e-7 in SI units and tests wrap crossing, stationary chassis, locked joint,
simultaneous rotation and world translation. These tolerances are synthetic,
not real-hardware noise limits.

Simulation uses `simulation_feedback` to relay `JointState.position/velocity` from
the same simulator sample, retaining its timestamp. `joint_locked=true` publishes
the known rigid-installation constraint at each LIO timestamp for fixed models.
The simulation launch supplies its level equivalent-IMU geometry rather than the
physical MID360 tilted IMU calibration. The command adapter reads the resulting
physical `base_link` TF in `camera_init`. No consumer applies another lever arm.

The public ROS replay includes the real communication bridge with injected
transport (no USB access), decodes the complete LibXR `nav_data` packet and checks
fixed-world command direction in every scenario. Its capture executable requires
`ros2_libxr` built with `BUILD_TESTING=ON`. Bench recording instructions and the
unfilled real-hardware acceptance report are in
[the bench guide](../../../docs/validation/yaw-chassis-bench.md).
