# fake_vel_transform

Publishes the chassis TF that Nav2 needs, including when `bt_manager` is off:

- `body → base_link`: joint inverse from `/sentry/offline_info.yaw_camerainit_to_gimbal`
- `base_link → base_link_fake`: heading cancelled in `camera_init` (+X)

Fake yaw is `-(yaw(camera_init→body) − s·yaw_joint)`, the same chain as
`lookup(camera_init, base_link)`. Do not subscribe `/aft_mapped_to_init` for this
TF: that odometry is gimbal/radar yaw.

Chassis body-frame `vx/vy` conversion lives in `ros2_libxr`, not in this node.
