# fake_vel_transform

Chassis-mounted LiDAR TF publisher, independent of `bt_manager`:

- `body → base_link`: static translation `(0, +0.20, 0)` m, identity rotation.
- `base_link → base_link_fake`: negative `camera_init → body` yaw, stamped with LIO TF time.
- `base_link → vision_gimbal`: relative chassis/gimbal feedback angle from
  `/sentry/offline_info.yaw_camerainit_to_gimbal` (degrees), stamped on receipt.
  No gimbal TF is published before valid feedback arrives.

Nav2/MPC retain `base_link_fake`. Its yaw is zero in `camera_init` for planar
motion; it is not fixed in `map` when localization adds a yaw correction.
The static offset reproduces the old `(0, +0.20, 0)` chassis-center convention.
MPC/planner `lidar_offset_y = -0.20` is the opposite vector (center to IMU).
Do not add a second publisher for any of these child frames.

Mounting translation is configurable with `body_to_base_x/y/z` at startup.
`joint_yaw_sign` defaults to +1. `base_to_gimbal_z` defaults to the old 0.5 m;
measure it on hardware. This yaw-only model assumes aligned axes at zero yaw
and does not compensate pitch, lifting motion, or feedback transport latency.
The offline message has no measurement timestamp; exact visual observation-time
TF coverage must be checked with live feedback and observation rates.

`body` is the built-in LiDAR IMU frame. The single-Livox launch publishes
`body → livox_frame` from the same internal extrinsics as `mid360.yaml`.
Chassis body-frame command conversion remains in `ros2_libxr`.

Regression check (after building and sourcing install/setup.bash):

```bash
python3 src/navigation/fake_vel_transform/test/check_chassis_tf.py
```

The check starts only this TF node and synthetic inputs, using an isolated ROS
domain. It checks the static lever arm, yaw cancellation, independence from
gimbal feedback, and dynamic visual gimbal rotation.
