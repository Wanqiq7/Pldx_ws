# 力控 MPC 阶段 2：坡道扰动与轮速融合

日期：2026-10-23。前序记录 `20261022_force_control_mpc.md`。改动横跨三处：本仓库 `minco_controller`、`ros2_libxr`、脚本，以及 `~/Sentry/PLDX_Template_mc02` 固件（`Modules/pldx/{Chassis,DualBoard}`、`User/RobotConfig/sentry_{chassis,gimbal}.yaml`）。

## 控制器（`minco_controller`，均默认关闭）

- 扰动 `d`（map 系，m/s²）进入动力学模型 `v' = a + d`，前馈改为 `m·(a_ref − d)`，`wrench` 即抵消 `d` 的驱动力。底盘速度环看不到 `d`，所以输出速度额外提前 `command_lookahead·(−d)`。每轴按 `maxDisturbance()` 截断，保留一半对角恢复能力；速度边界的可行性放宽同时计入漂移。
- `slope_gravity`：由 odom 姿态求坡面切向重力 `−g·(e_z − n_z·n)`。`gravity_body_roll_offset` 用于去掉 body 的安装横滚；故意不复用 `lidar_roll_offset`，仿真 body 是水平的，误用会在平地上造成约 1.7 m/s² 的恒定横向推力。
- `disturbance_observer`：一阶滞后（时间常数 = `command_lookahead`）加集总扰动的 Luenberger 观测器，两个极点都在 `−bandwidth`。只依据速度残差更新，不对含噪速度求导。
- `wheel_odometry`：订阅底盘系 `TwistStamped`，从 LIO 时间戳积分到当前时刻（零阶保持，平移用段中点航向旋转）。旋转航向取 LIO yaw，因为 `base_link_fake` 抵消了 yaw。这一段替代固定延迟外推，`control_delay_compensation` 只预测剩余部分。数据缺失、中断或陈旧时退回 LIO，并节流告警。
- 删除从未调用的 `applyGravityCompensation` 及其相关成员。详细 CSV 增加 `dist_x/dist_y` 两列。
- 新测试：`disturbance_test`（重力几何、观测器收敛/噪声/截断/中断）、`wheel_odometry_test`（直线、圆弧闭式解、零阶保持、陈旧/中断拒绝）、`dynamic_mpc_solver_test` 中的漂移用例（与稠密 QP 一致、坡上静止驻点、截断后可行）、`controller_freshness_test` 中的轮速使用与退回。

## 固件（MC02，未烧录）

- `Omni::GetMeasuredState()`：在互斥锁内读出 `SelfResolution()` 的车体系 `vx/vy/wz`、四轮在线标志、实际模式和采样时刻，经 `Chassis<T>::GetMeasuredState()` 转发。
- `DualBoard`：底盘侧每 4 ms 在 `0x311 + 0x1a = 0x32B` 发送 `ChassisVelocityFrame`（`int16` ×1000、8 位序号、有效位/四轮在线/模式）。采样超过 10 ms 未更新时有效位清零。云台侧解码，把序号扩展为 32 位，补上收帧 µs 时间戳和最近一次底盘陀螺 z，作为 26 B 的 `ChassisFeedback` 发布到 `chassis_feedback`。该 topic 允许多发布者，因为 CAN 接收线程和看门狗都会发布。超过 100 ms 无帧时发布一次无效反馈。`0x1a` 位于裁判位置分片 `0x14–0x19` 与电容 `0x1e` 之间，用 `static_assert` 约束不重叠。
- `sentry_gimbal.yaml`：`navigation_tx` 增加 `chassis_feedback`。`sentry_chassis.yaml`：底盘 `reduction_ratio` 由 19 改为 19.2（M3508 实际为 3591/187 ≈ 19.203）。`PowerControl.reduction_ratio_3508` 仍为 19.0，它参与功率模型标定，这次未动。
- 新测试 `DualBoard/tests/chassis_velocity_frame_test.cpp`：编解码、非法值清除有效位、序号回绕与丢包、主机镜像布局。

## 桥接（`ros2_libxr`）

- `wire::ChassisFeedback` 镜像固件结构，用 `static_assert` 约束布局。`chassis_feedback` 加入 `SharedTopic` 接收列表；旧固件不发这个 topic，主机仍兼容。
- 发布 `/sentry/chassis_velocity`（`TwistStamped`，`base_link`）。`BoardClockMapper` 用接收偏移的滑动最小值把板端 µs 时钟映射到 ROS 时间，复位或跳变时重同步，输出严格递增；之后再减去 `chassis_velocity_latency_us`。无效、链路断开、重复的样本，以及默认情况下有轮离线的样本都不发布，序号跳号会告警。`chassis_yaw_rate_source` 可选 `wheel` 或 `gyro`。
- `ros2_libxr_wire_test` 增加布局与时钟映射用例。该包以 Release 构建，带 `-DNDEBUG`，所以测试中 `assert` 原本被编译掉、什么都没检查。现在加了 `#undef NDEBUG`，并故意改错断言确认测试确实会失败。

## 坐标系核对

命令通路 PC → `HostDataLink::BuildAI` → `CMD` → `DualBoard` → `Omni::UpdateCMD` 全程原样复制 `vx/vy/wz`，不做旋转。`SelfResolution()` 是 `InverseKinematicsSolution()` 的伪逆，所以回传速度与命令同轴同号。轴向是否等于 REP-103（x 前、y 左、z 上，wz 逆时针为正）由用户在实车确认。wz 的物理符号代码无法确定，需上车用 `scripts/check_chassis_velocity_frame.py` 实测：它按控制器的杠杆臂补偿把 LIO 速度换算到底盘参考点，拟合 2×2 平移矩阵和 wz 比例，并剔除超过 4 倍 MAD 的离群点。

## 验证

```bash
# 本仓库
colcon build --symlink-install --packages-select minco_controller ros2_libxr --cmake-args -DBUILD_TESTING=ON
ROS_DOMAIN_ID=232 colcon test --packages-select minco_controller ros2_libxr
# 固件（构建目录放在仓库外，不覆盖 build/；完成后恢复 User/xrobot_main.hpp）
bash tools/Linux/buildgimbal.sh --skip-format -b /tmp/mc02_build_gimbal
bash tools/Linux/buildchassis.sh --skip-format -b /tmp/mc02_build_chassis
```

- `minco_controller` 58 项、`ros2_libxr` 1 项，全部通过。DualBoard 主机测试：新增用例、`referee_can_codec`、`sentry_decision_frame` 通过。`dual_board_control_frame_test`、`sentry_decision_static_regression`、`use_capacitor_transport_static_regression` 在模块 HEAD 上同样失败，属于原有问题。
- 两块板均编译通过，无警告。gimbal 镜像中包含 `chassis_feedback` 字符串。
- 仿真：`SIM_WHEEL_ODOM=1` 跑六目标路线的同时运行校验脚本。仿真轮速注入的比例误差为 `[1.02, 0.98, 1.01]`，脚本拟合出 `M = [[1.0199, −0.0033], [−0.0002, 0.9804]]`、旋转 0.09°、未镜像、`wz_scale = 1.0084`，判为一致。仿真真值在接触冲击时出现约 ±630 rad/s 的瞬时尖峰（2535 对中有 3 个），已被剔除。
- 坡道补偿与轮速的闭环结果见上一条记录及本轮对话。全部打开时停稳误差平均约 0.034 m（无补偿约 0.06 m）。轮速融合在仿真中没有可见收益，因为仿真 LIO 本身已接近真值。

## 限制

- 固件未烧录、未上车。实车启用 `wheel_odometry` 前需要：烧录两块板，先运行校验脚本，再按结果设置 `chassis_velocity_latency_us` 和 yaw 来源。
- 固件模块改动是 `Modules/pldx/{Chassis,DualBoard}` 各自 git 仓库中未提交的工作区修改。`tools/module_snapshots/` 已被删除且按用户要求不恢复，所以不能重新打 bundle；不要执行 `xrobot setup --frozen`，它会用旧 bundle 覆盖这些修改。
- 已知但未处理（用户决定暂不处理）：电容帧与裁判链路状态帧共用偏移 `0x1E`；底盘 RFID 帧与云台 `use_capacitor` 帧同为 `0x323`。
