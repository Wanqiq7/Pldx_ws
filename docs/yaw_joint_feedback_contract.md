# 大 Yaw 关节反馈契约

对应工单 [#6](https://github.com/Wanqiq7/Pldx_ws/issues/6)。术语见 [GLOSSARY](../GLOSSARY.md) 的 Yaw Feedback 一节，设计背景见 [ADR-0003](adr/0003-physical-chassis-state-for-yaw-mounted-lidar.md)。

## ROS 消息

话题 `/sentry/yaw_joint_state`，类型 `ros_interfaces/msg/YawJointState`，由 `ros2_libxr` 发布（可靠，深度 200）。

| 字段 | 含义 |
| --- | --- |
| `header.stamp` | 电机反馈帧的测量时间，已映射到 ROS/LIO 时钟。不是接收时间 |
| `header.frame_id` | 角度的参考系，实体底盘 `base_link` |
| `child_frame_id` | 转动端，`lidar_yaw_mount` |
| `angle` | rad，转动端相对实体底盘，范围 [-π, π)，以电机标定零位 `yaw_zero` 为零，电机 `reverse` 生效后从上向下看逆时针为正 |
| `velocity` | rad/s，`angle` 的变化率，符号相同 |
| `mcu_stamp_us` | 48 位 LibXR 包头时间戳（MCU 微秒），仅用于诊断 |
| `receive_stamp` | 上位机解包时刻，仅用于诊断延迟 |

`angle` 和 `velocity` 不是雷达对世界的航向或转速。参数：`yaw_joint_topic`、`yaw_joint_frame`、`yaw_joint_child_frame`、`yaw_joint_clock_window`（秒，默认 5）。

## 采样与时间

- 下位机 `Gimbal` 每次解码到新的大 Yaw 电机 CAN 帧，用同一帧的 `abs_angle - yaw_zero` 与 `omega` 组成一个样本，所以两个字段同期采样。DM 电机 `omega` 单位 rad/s，路径上没有额外传动比换算，`reverse` 对角度与角速度同时生效。
- 样本时间是该帧的 CAN 接收时刻（MCU `Timebase` 微秒），写入 LibXR 包头的 48 位时间戳，负载只有 8 字节 `{float angle_rad, float velocity_rad_s}`。话题名 `nav_yaw_joint`，经 `navigation_tx` 发送。
- 电机离线、带错误码、数值非有限或没有新序号时不发送。

## MCU 时间到 ROS 时间

`SourceClockMap`（`ros2_libxr/yaw_joint.hpp`）：

1. 把 48 位计数器按模 2^48 解卷绕。回退超过 1 s 视为 MCU 重启，重新建立映射。
2. 偏移取滑动窗口内 `接收时间 - MCU 时间` 的最小值。传输延迟不会为负，所以最小值对应延迟最小的包。
3. `header.stamp = MCU 时间 + 偏移`。结果不会晚于接收时间，与真实测量时刻的偏差不超过窗口内最小传输延迟，不随单包抖动变化。接收时间只用于估计偏移，不当作采样时刻。

映射目标是节点所用的 ROS 时钟，LIO 与该节点使用同一时钟时可按 `header.stamp` 直接对齐。时钟漂移由窗口跟随，漂移 100 ppm 时窗口 5 s 内约引入 0.5 ms 滞后。

## 验证

- `ros2_libxr_yaw_joint_test`：经 LibXR 包头解析、带时间戳回调、解码与时间映射回放。覆盖已知角度/转速、2 到 5 ms 抖动传输、单包 60 ms 迟到、MCU 时间整体偏移、48 位计数器回绕、MCU 重启、角度跨越 ±π、错误长度/零时间戳/非有限值。
- 下位机：`Modules/GUET-PLDX/Gimbal/tests/run_yaw_joint_feedback_test.sh`，覆盖同帧取样、重复帧、离线/错误帧、48 位截断、序号回绕与负载字节。

实机零位、方向与安装标定由 [#11](https://github.com/Wanqiq7/Pldx_ws/issues/11) 验证；`yaw_zero` 当前仍是占位值。沿用现有超时处理，没有新增停止门控或降级状态。
