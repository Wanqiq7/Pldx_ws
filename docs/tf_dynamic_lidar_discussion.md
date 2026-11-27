# 大 Yaw 雷达的 TF 设计讨论

2026-11-27。状态：用户已确认完整设计和六张工单拆分；工单已发布，运行代码实施另行进行。执行依赖见[工单总览](plans/yaw-mounted-lidar/README.md)。

## 已确认的硬件前提

用户已更正并确认：参考工程 `navi_minco_bit` 的雷达固定在底盘；当前工程的 MID360 固定在大 Yaw 转动端。前者的雷达到底盘外参为常量，后者随大 Yaw 关节角变化。此前“参考工程雷达在云台”的说法已撤回。

雷达内置 IMU 是当前 LIO 的 `body` 参考系。位姿、平移速度必须明确参考点，姿态与角速度必须明确所属刚体；仅取消显示坐标系的 yaw 不会消除传感器的真实运动。

## ROSE 源码核对

以下路径和行号相对 `rose_navigation` 源码目录。

- `config/mid360_lm.yaml:8`：配置 `lidar_frame`、`robot_base_frame` 与 `base_frame`，`robo_to_lidar_dynamic` 为 `false`。
- `src/lm/small_point_lio.cpp:202`：初始化时按首帧点云时间查询雷达到机器人参考系的外参。
- `src/lm/small_point_lio.cpp:541`：动态分支按 LIO 输出时间查询当前外参，尝试恢复机器人参考系位姿。
- `src/lm/small_point_lio.cpp:580`：动态分支按消息时间差分位姿得到线速度和角速度，没有直接订阅关节角速度做减法。
- `src/utils/rcl_tf.hpp:41`：TF 包装直接调用 `lookupTransform(target, source, time, timeout)`。

可借鉴的设计意图是：保留传感器运动估计，结合同期关节外参恢复目标刚体位姿，然后在明确的参考系下求速度。动态模式默认未启用，不能据此宣称该实现已经验证了活动安装。

动态分支的矩阵方向需要修正。采用 `^A T_B` 表示把 B 中坐标变换到 A 的矩阵，令 `B` 为机器人参考系、`L` 为雷达参考系：

```text
E0 = ^B0 T_L0                 # 初始化查询结果
P(t) = ^L0 T_Lt               # LIO 在初始雷达参考系中的位姿
F(t) = ^Lt T_Bt               # 当前查询结果

初始世界系对齐 B0 时：
正确的机器人位姿 = E0 * P(t) * F(t)
ROSE 第 568 行实际计算 = inverse(F(t)) * P(t) * inverse(E0)
```

最简反例：初始外参为单位阵，底盘静止，零杆臂，雷达只转动 30°。此时 `P=R(30°)`、`F=R(-30°)`，正确结果为 0°，ROSE 现有表达式得到 60°。该结果已用平面旋转算例核对，尚未运行 ROSE 的 ROS 节点验证。

此外，动态 TF 查询失败时实现会复用缓存值；位姿差分速度直接写入带机器人 `child_frame_id` 的 Odometry，表达轴仍需核对。上述细节不适合直接复制到当前控制链。

## 当前工程与参考工程的差距

以下路径相对当前工程，另行注明者除外。

- 参考 `navi_minco_bit` 的 `src/perception/Point-LIO/src/laserMapping.cpp:659` 使用安装偏移恢复参考点位置，并把 `base_link` 朝向置为单位旋转。因此参考工程中的这一 frame 是去 yaw 的导航参考系，不能从名称推断它具有实体底盘朝向。
- `src/navigation/fake_vel_transform/src/fake_vel_transform.cpp:34` 把 `body → base_link` 发布为静态外参。对当前活动安装，这只能表达锁定关节时的安装状态。
- 同文件第 55 行使用关节角发布 `base_link → vision_gimbal`，但该角没有参与恢复实体底盘位姿；回调使用上位机接收时刻打戳。
- `src/navigation/minco_controller/src/minco_mpc_controller.cpp:263` 使用固定二维杆臂修正 LIO 线速度，第 925 行继续使用 LIO yaw 与角速度构造控制状态。
- `src/navigation/minco_planner/src/minco_core/minco_planner.cpp` 中的 `getCurrentSpeed()` 和 `getCurrentYawFromOdom()` 也读取原始 LIO，不能只修改 MPC。
- `src/navigation/ros2_libxr/src/ros2_libxr/src/ros2libxr.cpp:112` 从 LIO 读取 yaw 与角速度用于指令方向换算。若目标是实体底盘坐标系，这条链也应使用一致的底盘朝向。
- 导航桥现有 `offline_info` 有关节角与底盘 IMU yaw，但导航接入没有同期关节转速；ROS 消息的接收时间戳不是传感器测量时间戳。下位机 `NavLink` 另有 `GimbalFeedback.yaw_velocity`，传输接入、单位和时钟关系仍须核对。

## 已选择的设计方向

保留当前 Point-LIO 估计传感器运动，在独立的状态转换边界集中完成动态安装几何与底盘状态恢复，让规划器、MPC 和指令方向转换使用一致的物理参考点与朝向。继续保留导航虚拟参考系的兼容用途，避免把物理底盘朝向与虚拟零 yaw 混为一谈。

令 `W=camera_init`、`B=实体底盘参考系`、`Y=大 Yaw 转动端`、`I=body`：

```text
^B T_I(t) = ^B T_Y(q(t)) * ^Y T_I
^W T_B(t) = ^W T_I(t) * inverse(^B T_I(t))
```

第一项应由转轴位置、关节零位、方向和关节角决定，第二项为转动端到 IMU 的安装标定。雷达倾斜安装时应使用完整三维变换，不能把 IMU 的 z 轴角速度直接当作世界竖直轴转速。

用户选择由状态转换节点发布 `camera_init → base_link` 并禁用 LIO 对 `body` 的 TF 发布。继续发布 `base_link → lidar_yaw_mount → body → livox_frame` 安装链，保留 `base_link → base_link_fake` 导航虚拟参考系。`lidar_yaw_mount` 表示大 Yaw 转动端，沿用当前 CAD 零位讨论中的名称。安装链上的静态外参及动态关节变换必须各有唯一发布者。

用户选择解析刚体关系与同期关节转速作为主速度来源，对恢复后的底盘位姿按测量时间求导只用于验证。

解析关系可在完整三维几何下定义。令 `O` 为大 Yaw 转轴上的固定点，`r_BO` 为底盘参考点指向 O 的向量，`r_OI` 为 O 指向 IMU 的向量；以下向量统一表达在 W 中，`a_W` 为关节正方向的单位轴：

```text
omega_B = omega_I - a_W * q_dot
v_B = v_I - omega_B × r_BO - omega_I × r_OI
```

该表达式假定第一阶段仅有大 Yaw 关节运动，其他机构固定，IMU 与转动端刚性连接。`omega_I` 必须先由 IMU 轴旋转到 W；`r_BO` 与 `r_OI` 必须按同一时刻姿态旋转到 W。输出标准底盘 Odometry 时，pose 表达在 W，twist 再旋转到底盘 child frame。若底盘参考点与转轴水平重合，平面线速度恢复不单独依赖 `q_dot`，但底盘角速度仍需要分离关节转动。

规划器和 MPC 接收已经恢复的底盘速度后，应移除该路径上原有的雷达杆臂补偿，避免重复扣除。规划器、MPC 和串口桥的速度指令与朝向必须处于同一世界参考系；当前局部 costmap/MPC 使用 `camera_init`，后续若改为 `map`，通信转换也必须同步变更，不能只改消息名称。

## 设计树与第一轮已确认项

1. 状态语义：用户选择恢复完整实体底盘状态，保留导航虚拟参考系供现有 Nav2 使用。
2. 机械范围：用户选择第一阶段只处理大 Yaw 自由度，验证时固定其他会改变雷达安装的机构。
3. 反馈边界：用户允许完善上下位机反馈，接入关节角、转速与可对齐的测量时间。

保留现有 MINCO 规划器与隧道命令策略为讨论前提；`docs/adr/0002-controller-owns-tunnel-command-policy.md` 的策略归属不因 TF 变换而迁移。

## 第二轮已确认项

4. TF 拓扑：用户选择统一底盘状态转换节点发布物理 TF 与底盘里程计，关闭 Point-LIO 原有 `camera_init → body` TF，同时保留原始 LIO 里程计。
5. 速度来源：用户选择解析恢复作为控制输入，位姿差分用于验证。
6. 用户明确要求“不要增加冗余安全保护”。本次沿用现有命令与里程计超时处理，不新增停止门控、保护话题或降级状态机。正常数据转换仍按实际样本的时间与语义计算，不把旧值重新标为新测量。

下位机 `NavLink.hpp:100` 明确导航速度是 chassis-frame，`MakeAIChassis()` 直接使用收到的 XY，没有在此处再扣关节角。因此，通信出口必须使用实体底盘朝向，不能继续依赖雷达转动端朝向。

## 反馈与标定核对结果

- 下位机 `Gimbal/Gimbal.hpp:313` 的关节角为电机绝对角减 `yaw_zero`；第 355 行起发布角度和电机 `omega`。`DMMotor/DMMotor.hpp:339` 的 `omega` 解码为 rad/s，此路径未见额外传动比换算。
- `NavLink.GimbalFeedback` 已有角度和转速，但字段分别回调更新，不能仅因在同一包中就当作同期采样；当前 USB 发送配置与 ROS 导航接收列表还没有接通这一路。
- LibXR 消息包头已有 48 位微秒时间戳，但目前关节反馈未贯通电机测量时刻，ROS 回调还丢弃了源时间。实施应尽量复用现有时间戳能力，建立 MCU 到 ROS/LIO 时钟的映射，并在同一 LIO 时刻取得关节角和转速。
- 大 Yaw 电机配置中的零位与 CAN ID 尚有占位注释，实机零位、方向及转轴几何仍需标定验证。先以 CAD 为安装几何初值，不把它视为实机标定真值。
- 底盘 IMU yaw 可辅助对比方向和相对转角；当前没有证据证明它与 LIO 世界零位已经对齐，不将它直接替代 LIO 世界航向。

## 具体改动边界

1. 反馈链：下位机提供可对齐的关节角与转速，ROS 桥保留并换算源时间。
2. 状态转换节点：保留 `/aft_mapped_to_init` 的传感器里程计语义，新增底盘里程计（拟名 `/chassis/odom`，pose 父系为 `camera_init`，twist 子系为 `base_link`），集中完成一次几何和速度转换。
3. TF 发布：Point-LIO 关闭原始 TF 广播；统一转换节点与安装/关节发布者组成物理链；`base_link_fake` 保留其在 `camera_init` 中平面零 yaw 的用途。零位 CAD 验证 profile 继续明确只用于关节锁定验证。
4. 消费者：规划器、MPC、Nav2 停车速度检查与通信桥改用底盘状态；删除其新输入路径上重复的雷达杆臂补偿。相机/视觉参考系保留原名称，核对大 Yaw 与小云台的实际关系后正确挂接，不能把视觉 frame 名当成机械大 Yaw 的同义词。
5. 保持既有策略：隧道、逃逸与普通/小陀螺角速度仲裁仍归原有模块。这里只统一它们依赖的物理状态和指令坐标。

最低验收场景为底盘静止大 Yaw 转动、关节锁定底盘旋转、两者同时运动以及固定世界方向平移指令；期望分别消除关节引起的虚假底盘运动、保留真实底盘运动、满足解析分解关系并保持命令方向正确。先用合成数据/离线检查验证三维旋转、杆臂、单位、时间和唯一 TF 发布者，再在其他机构固定的台架条件下验证。
