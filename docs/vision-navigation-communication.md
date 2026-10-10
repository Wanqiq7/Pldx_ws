# 视觉与导航 ROS 通信

目标观测使用 `/vision/target_observation`（`pldx_vision_interfaces/TargetObservation`，Reliable/KeepLast(1)）；目标选择使用 `/vision/target_directive`（`VisionTargetDirective`，Reliable/KeepLast(10)）。两端应使用同一工作区构建的消息定义及同一 ROS_DOMAIN_ID。

观测 header.stamp 为图像采集时间，header.frame_id 为 vision_gimbal。导航按采集时间查询 TF 转换到 vision_target_map_frame（默认 map），查询超时由 vision_target_tf_timeout_ms（默认 50）控制。目标位置单位为米；缺少 TF、非法目标或超过 vision_target_max_age_ms（默认 200）的观测会清除目标。

导航的目标指令周期为 100 ms；视觉 750 ms 未收到新指令即恢复默认选择。not_aim_enemy=true 时发送有效限制指令，优先目标为空，表示不瞄准任何目标。

正常启动必须保证 map→camera_init→body→vision_gimbal TF 链可查询，并开启 bt_manager。start_nav.sh 禁用 bt_manager，不能单独提供视觉目标指令生产者及该 launch 的 body→vision_gimbal 静态 TF。

2026-10-09 使用两端生产通信类，在独立 ROS 域完成 15 项收发、坐标、有效性及超时检查。测试入口与结果见工作区 artifacts/vision-navigation-check；未执行相机识别或实车运动。
