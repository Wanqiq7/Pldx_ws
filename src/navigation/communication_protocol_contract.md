# 当前导航通信协议与字段契约

> 冻结基线：2026-08-31，协议 revision 3
>
> 本文件固化当前导航工程通信包的应用层协议，作为后续 LibXR/XR transport 迁移的输入。迁移时只替换串口承载和帧收发实现，不以其他工程的 payload 定义覆盖本契约。

## 1. 权威来源与范围

权威来源按以下顺序排列：

1. `src/navigation/communication/README.md` 的模块职责、ROS 接口、单位和时序说明；
2. `src/navigation/communication/include/utils/custom_protocol.hpp` 的枚举、结构体和字段顺序；
3. `src/navigation/communication/include/utils/protocol.hpp` 的旧帧格式和校验实现；
4. `src/navigation/communication/include/com_interface_ros.hpp` 与 `src/navigation/communication/src/com.cpp` 的实际打包、解析和 ROS 映射行为。

`PLDX_Template` 中的 `NavLinkProtocol.hpp` 和 `pldx_ws` 中的旧 XR bridge 不是本契约的协议来源。它们只能用于后续兼容性对照或复用 LibXR/Linux 实现。

本契约包含：

- 导航主机与固件之间的 payload 语义、字段顺序、单位和方向；
- 当前旧串口帧格式，作为迁移前的基线；
- ROS topic 与 payload 的映射；
- 当前时序、坐标转换和安全行为；
- 已知实现歧义和迁移前必须确认的事项。

## 2. 当前通信链路

```text
ROS 2 输入
  -> communication_sentry
  -> 控制/行为/odom 快照
  -> 10 ms 通信循环
  -> 串口协议打包
  -> STM32 固件

STM32 串口数据
  -> 帧解析
  -> payload 解析
  -> communication_sentry
  -> ROS 2 输出
```

当前实现使用 `FdManager + SerialPort + TimerManager`，串口接收在独立线程中运行，ROS callback 与通信循环使用独立 callback group。

当前配置基线：

| 项目 | 当前值 |
|---|---|
| 设备路径 | `/dev/ttyACM0`（源码静态值） |
| 波特率 | `115200` |
| 数据位 | 8 |
| 停止位 | 1 |
| 校验位 | 无（`N`） |
| 主通信周期 | 10 ms（约 100 Hz） |
| 全局路径辅助周期 | 1 s，但当前发送调用被注释 |
| 启动发送延迟 | 2 s |

设备路径是实车绑定值，不代表硬件枚举事实。迁移到 XR 时必须重新确认实际 Linux 设备节点，并将路径参数化或使用稳定 USB selector。

## 3. 当前旧串口帧格式

当前旧帧使用 8 字节、1 字节对齐的 `PacketHeader`：

| 偏移 | 字段 | 类型 | 说明 |
|---:|---|---|---|
| 0 | `start1` | `uint8_t` | 固定 `0xA5` |
| 1 | `start2` | `uint8_t` | 固定 `0x5A` |
| 2 | `from` | `uint8_t` | 发送方，当前导航发送为 `ENUM_ARM_SENTRY` |
| 3 | `to` | `uint8_t` | 接收方，底盘/下位机为 `ENUM_ARM_SLAVE_COMPUTER` |
| 4 | `packet_type` | `uint8_t` | `PacketTypeEnum` |
| 5 | `data_len` | `uint8_t` | payload 字节数，不含包头 |
| 6 | `checksum` | `uint16_t` | 16 位累加校验和 |

发送端的目标校验逻辑位于 `Communication::__send2stm32`：对包头（排除 checksum）和 payload 做 16 位累加。`calChecksum` 以 16 位小端字为单位累加，奇数长度时再累加最后一个字节。

当前接收校验不是有效的安全校验：`protocol.hpp` 中长度检查、帧头检查和 checksum 比较均被注释，`check()` 最终恒定返回 `true`。这是旧实现事实，不应作为 XR 实现的验收标准。

XR 迁移时的默认解释为：保留下文 payload 的语义和字段布局，由 XR packet 的 topic、长度和 CRC 取代 `PacketHeader` 的外层承载。若要求旧 `PacketHeader` 字节级保留，则应另行定义“XR 携带完整旧帧”的双层封装方案。

## 4. PacketType 与方向

| 枚举 | 数值 | 方向 | payload | 当前状态 |
|---|---:|---|---|---|
| `ENUM_PACKET_NAV_DATA` | 0 | 导航 -> STM32 | `ChassisTarget` | 每个通信周期发送 |
| `ENUM_PACKET_ALLY_STATUS` | 1 | STM32 -> 导航 | `TeamInfo` | 接收并发布 |
| `ENUM_PACKET_GAMESTATUS_DATA` | 2 | STM32 -> 导航 | `GameInfo` | 接收并发布 |
| `ENUM_PACKET_SENTRY_SERVER_DATA` | 3 | STM32 -> 导航 | `SentryInfoOnline` | 接收并发布 |
| `ENUM_PACKET_SENTRY_SELF_DATA` | 4 | STM32 -> 导航 | `SentryInfoOffline` | 接收并发布 |
| `ENUM_PACKET_RADAR` | 5 | STM32 -> 导航 | `RadarInfo` | 接收并发布 |
| `ENUM_PACKET_GLOBAL_PATH_X` | 6 | 导航 -> STM32 | `GlobalPathX` | 构造逻辑存在，发送已注释 |
| `ENUM_PACKET_GLOBAL_PATH_Y` | 7 | 导航 -> STM32 | `GlobalPathY` | 构造逻辑存在，发送已注释 |
| `ENUM_PACKET_BEHAVIOR_DATA` | 8 | 导航 -> STM32 | `BehaviorData` | 每个通信周期发送 |

数值以当前 `custom_protocol.hpp` 的枚举顺序为准，不应根据其他工程中的 packet ID 重新编号。

当前 Linux/GCC 编译环境下的 payload 大小如下。这里的 `ChassisTarget` 虽然没有在结构体声明处写 `packed`，但处于 `custom_protocol.hpp` 的外层 `#pragma pack(push, 1)` 范围内。

| 类型 | `sizeof`（字节） |
|---|---:|
| `PacketHeader` | 8 |
| `NavRes` | 13 |
| `GlobalPath` | 102 |
| `GlobalPathX` | 51 |
| `GlobalPathY` | 51 |
| `ChassisTarget` | 45 |
| `BehaviorData` | 26 |
| `AllyRobotStatus` | 11 |
| `TeamInfo` | 48 |
| `GameInfo` | 22 |
| `SentryInfoOnline` | 37 |
| `SentryInfoOffline` | 16 |
| `EnemyRobotStatus` | 9 |
| `RadarInfo` | 59 |

## 5. 发送 payload 字段

### 5.1 `ChassisTarget`

当前结构体字段顺序如下。该结构体声明处未写 `packed`，但受文件外层 `#pragma pack(push, 1)` 影响；迁移时仍必须显式冻结其实际大小和偏移，不能仅凭字段顺序推断 wire layout。

| 顺序 | 字段 | 类型 | 当前语义 |
|---:|---|---|---|
| 1 | `vx_mps` | `float` | 前进方向速度，m/s |
| 2 | `vy_mps` | `float` | 左侧方向速度，m/s |
| 3 | `vw_rad_s` | `float` | 目标偏航角速度，rad/s |
| 4 | `current_yaw` | `float` | 当前底盘航向，rad |
| 5 | `current_vx` | `float` | odom 当前线速度 x |
| 6 | `current_vy` | `float` | odom 当前线速度 y |
| 7 | `current_vw` | `float` | odom 当前角速度 z |
| 8 | `fx_global` | `float` | `/cmd_force_mpc.force.x`，全局系，N |
| 9 | `fy_global` | `float` | `/cmd_force_mpc.force.y` |
| 10 | `fw_global` | `float` | `/cmd_force_mpc.torque.z` |
| 11 | `use_speed_control` | `bool` | 速度控制/力控制标志；当前发送代码固定为 `true` |
| 12 | `delta_yaw` | `float` | odom 与底盘 IMU yaw 匹配得到的航向差 |

当前构造行为：

- `cmd_vel.angular.z` 直接写入 `vw_rad_s`，单位 rad/s，不把 `gyro_vel` 写入 `vw`；
- odom 的线速度和角速度直接写入 `current_vx/current_vy/current_vw`；
- `/cmd_force_mpc` 的 x/y 力和 z 力矩直接写入 `fx_global/fy_global/fw_global`；
- 主机恒发 `use_speed_control=true`，下位机走速度外环；`false` 时下位机才直接执行全局系合力/力矩。
- yaw 从 odom 四元数计算；
- 隧道覆盖 vx/vy/vw 在旋转前执行；`delta_yaw` 已初始化时，将 map/odom 系 vx/vy 转到底盘 IMU 车体系：θ = current_yaw - delta_yaw，vx_b = vx·cosθ + vy·sinθ，vy_b = -vx·sinθ + vy·cosθ；
- `delta_yaw` 未初始化时，将平移速度和 x/y 力清零；
- tunnel escape、tunnel prepare 等逻辑在通信循环中覆盖或修改待发送值。

### 5.2 `BehaviorData`

该结构体使用 1 字节对齐，字段顺序如下：

| 顺序 | 字段 | 类型 | 当前语义 |
|---:|---|---|---|
| 1 | `pitch_mode` | `uint8_t` | 云台抬头模式 |
| 2 | `desire_stance` | `uint8_t` | 哨兵姿态 |
| 3 | `desire_lifter_pos` | `uint8_t` | 云台升降位置 |
| 4 | `scan_yaw_min_rad` | `float` | 扫描最小偏航角，rad |
| 5 | `scan_yaw_max_rad` | `float` | 扫描最大偏航角，rad |
| 6 | `ammo_purchase_request` | `uint16_t` | 弹药兑换请求累计值 |
| 7 | `revive_request` | `uint8_t` | 本地复活请求 |
| 8 | `remote_revive_request` | `uint8_t` | 远程复活请求 |
| 9 | `remote_ammo_request` | `uint8_t` | 远程买弹请求 |
| 10 | `remote_health_request` | `uint8_t` | 远程买血请求 |
| 11 | `use_limited_scan` | `bool` | 限制性扫描模式 |
| 12 | `not_aim_enemy` | `bool` | 是否不瞄准敌方 |
| 13 | `use_capacitor` | `bool` | 是否使用电容 |
| 14 | `tunnel_align_active` | `bool` | 隧道航向对齐请求 |
| 15 | `tunnel_align_angle_rad` | `float` | 隧道目标角，rad |
| 16 | `use_gyro_mode` | `bool` | 小陀螺模式；`offsetof==25` |

`Behavior.msg` 还包含 `control_mode`、`gyro_vel`、隧道逃逸速度、准备状态等字段；这些字段不在 26 B `BehaviorData` 中，`use_gyro_mode` 已入 wire，其余必须记录其处理策略，不能默默丢弃。

### 5.3 可选路径 payload

| payload | 字段 |
|---|---|
| `GlobalPath` | `start_x:uint16_t`、`start_y:uint16_t`、`delta_x:int8_t[49]`、`delta_y:int8_t[49]` |
| `GlobalPathX` | `start_x:uint16_t`、`delta_x:int8_t[49]` |
| `GlobalPathY` | `start_y:uint16_t`、`delta_y:int8_t[49]` |

路径坐标以 minimap 为目标坐标系，当前代码将米转换为分米后量化，并将增量限制到 `int8_t`。当前实际发送调用被注释，因此 XR 第一阶段默认不启用。

## 6. 接收 payload 字段

### 6.1 `TeamInfo`

`TeamInfo` 使用 1 字节对齐，包含：

| 字段 | 类型 | 语义 |
|---|---|---|
| `ally_status[4]` | `AllyRobotStatus[4]` | 四名友方机器人 |
| `outpost_hp` | `uint16_t` | 己方前哨站血量 |
| `base_hp` | `uint16_t` | 己方基地血量 |

`AllyRobotStatus`：

| 字段 | 类型 | 语义 |
|---|---|---|
| `robot_id` | `uint8_t` | 机器人 ID |
| `robot_hp` | `uint16_t` | 血量 |
| `robot_pos_x` | `float` | x，m |
| `robot_pos_y` | `float` | y，m |

### 6.2 `GameInfo`

| 字段 | 类型 | 语义 |
|---|---|---|
| `game_time_remaining` | `uint16_t` | 比赛剩余时间，s |
| `coin_remaining` | `uint16_t` | 己方剩余金币 |
| `event_code` | `uint32_t` | 场地事件代码，未解码 |
| `game_status` | `uint8_t` | 比赛阶段枚举 |
| `manual_point_x` | `float` | 手动目标点 x，minimap/m |
| `manual_point_y` | `float` | 手动目标点 y，minimap/m |
| `manual_key` | `uint8_t` | 手动目标点快捷键 |
| `enemy_outpost_hp` | `uint16_t` | 敌方前哨站血量 |
| `enemy_base_hp` | `uint16_t` | 敌方基地血量 |

`game_status` 当前约定：`0` 未开始、`1` 准备、`2` 裁判系统自检、`3` 倒计时、`4` 比赛中、`5` 结算。

### 6.3 `SentryInfoOnline`

| 字段 | 类型 | 语义 |
|---|---|---|
| `self_health` | `uint16_t` | 机器人血量 |
| `bullets_remaining` | `uint16_t` | 剩余发弹量 |
| `cooling_value` | `uint16_t` | 每秒冷却值 |
| `heat_limit` | `uint16_t` | 热量上限 |
| `current_heat` | `uint16_t` | 当前热量 |
| `sentry_pos_x` | `float` | 哨兵位置 x，m |
| `sentry_pos_y` | `float` | 哨兵位置 y，m |
| `speed_monitor_angle` | `float` | 测速模块朝向，度 |
| `sentry_info_1` | `uint32_t` | 未解码信息 1 |
| `sentry_info_2` | `uint16_t` | 未解码信息 2 |
| `sentry_info_3` | `uint64_t` | 未解码信息 3 |
| `energy_ratio` | `uint8_t` | 底盘能量比例 |

### 6.4 `SentryInfoOffline`

| 字段 | 类型 | 语义 |
|---|---|---|
| `yaw_camerainit_to_gimbal` | `float` | camera_init 到 gimbal 的 yaw，度制约定需与视觉一致 |
| `lifter_current_pos` | `uint8_t` | `0` 顶部、`1` 底部、`2` 中间 |
| `is_transformable` | `bool` | 是否允许变形 |
| `transform_state` | `float` | 变形状态，0 到 1 |
| `capacitor_capacity` | `uint8_t` | 底盘 `PowerControl/SuperPower` 超电遥测，0..100%；255 表示不可用 |
| `chassis_imu_yaw` | `float` | 底盘 IMU yaw |
| `tunnel_yaw_aligned` | `bool` | 底盘航向是否完成对齐 |

`chassis_imu_yaw` 会进入有界历史队列，并与 odom stamp 在 `communication.imu_yaw_window_ms`（默认 40 ms）内做最近时间匹配，生成 `delta_yaw`。底盘 IMU 无效时发送 NaN，导航端不得将该样本加入历史队列。

revision 3 删除了由 `vision/target_observation` 提供的视觉目标字段。`bool` 按单字节
编码，字段顺序和偏移必须与 MCU 固件一致；revision 2 的 30 B payload 不得被 revision 3
实现解析或发送。

### 6.5 `RadarInfo`

| 字段 | 类型 | 语义 |
|---|---|---|
| `enemy_status[6]` | `EnemyRobotStatus[6]` | 六类敌方机器人 |
| `enemy_coin_left` | `uint16_t` | 敌方剩余金币 |
| `enemy_coin_accumulated` | `uint16_t` | 敌方累计金币 |
| `is_enemy_outpost_sensed` | `bool` | 是否感知到敌方前哨站 |

`EnemyRobotStatus`：

| 字段 | 类型 | 语义 |
|---|---|---|
| `robot_id` | `uint8_t` | 机器人 ID |
| `robot_hp` | `uint16_t` | 血量 |
| `allowed_projectile` | `uint16_t` | 允许发射的弹丸数 |
| `robot_pos_x` | `uint16_t` | x，cm |
| `robot_pos_y` | `uint16_t` | y，cm |

发布 ROS `RadarInfo` 时，`robot_pos_x/y` 会除以 100 转换为 m。

## 7. ROS 接口契约

### 7.1 输入

| Topic | 类型 | 通信层行为 |
|---|---|---|
| `/cmd_vel_mpc` | `geometry_msgs/msg/Twist` | 缓存世界系速度，通信循环读取 |
| `/cmd_force_mpc` | `geometry_msgs/msg/WrenchStamped` | 缓存力/力矩兼容输入 |
| `/aft_mapped_to_init` | `nav_msgs/msg/Odometry` | 更新 odom 和 yaw 时间匹配状态 |
| `/sentry/behaivor_send` | `ros_interfaces/msg/Behavior` | 缓存行为、资源和隧道指令 |

### 7.2 输出

| Topic | 来源 payload | 主要转换 |
|---|---|---|
| `/sentry/team_info` | `TeamInfo` | 队友数组、前哨站和基地直接映射 |
| `/sentry/game_info` | `GameInfo` | 字段直接映射 |
| `/sentry/online_info` | `SentryInfoOnline` | 位置字段写入 `geometry_msgs/Point` |
| `/sentry/offline_info` | `SentryInfoOffline` | 发布机构、云台 yaw 和底盘 IMU 状态，并记录 IMU yaw 历史 |
| `/sentry/radar_info` | `RadarInfo` | 敌方坐标 cm 转 m，其余字段直接映射 |

迁移时保留这些 topic 和消息类型，ROS 控制器、行为树和算法节点不应感知 XR 实现细节。

## 8. 坐标、单位和时序约束

- `/cmd_vel_mpc` 在 README 中定义为世界坐标系速度。
- communication / `ros2_libxr` 在 `use_speed_control=true` 时结合 `delta_yaw` 将 vx/vy 转到底盘 IMU 车体系后再下发。
- `delta_yaw` 由 odom stamp 与底盘 IMU yaw 历史做最近时间匹配；未初始化时抑制平移命令。
- `cmd_vel.angular.z` 和 `vw_rad_s` 均为 rad/s；不再进行 rpm 转换。
- 当前主循环每 10 ms 发送一次 NAV payload 和一次 Behavior payload，两次发送之间 sleep 2 ms。
- 控制和行为消息只保留最新值；odom 使用 KeepLast(1)、best effort、volatile。
- 性能诊断默认关闭；开启后会写 `/tmp/communication_logs/*.csv`，每行同步 flush。

## 9. 结构布局冻结项

实现 XR transport 前必须在主机侧加入编译期或测试期检查，至少覆盖：

```text
sizeof(PacketHeader)
sizeof(ChassisTarget)
sizeof(BehaviorData)
sizeof(TeamInfo)
sizeof(GameInfo)
sizeof(SentryInfoOnline)
sizeof(SentryInfoOffline)
sizeof(RadarInfo)
offsetof(all wire fields)
std::is_trivially_copyable<all wire payloads>
```

还必须冻结：

- 小端字节序假设；
- `bool` 的 wire 表示；
- `float` 的 IEEE-754 表示；
- 结构体对齐和尾部 padding；
- topic 名称到 `PacketTypeEnum` 的一一映射；
- payload 版本和兼容策略。

## 10. 已知歧义和迁移前决策

1. `ChassisTarget` 的所有速度、角度字段统一使用 SI：m/s、rad/s、rad。
2. 全局力字段单位固定为 N/N*m；下位机使用实时底盘 IMU yaw 转换到车体系。
3. `ChassisTarget` 虽受当前文件的外层 `#pragma pack(1)` 影响，但主机和固件仍必须确认实际布局；不能直接把未验证的 C++ 对象内存当作跨平台 wire 格式。
4. 当前接收 `check()` 恒真，XR 版本必须执行长度、topic、payload 类型和 CRC 检查。
5. `Behavior.msg` 字段多于 `BehaviorData`；缺少 wire 字段的行为必须建立保留、降级或扩展策略。
6. 当前协议基线为 115200、100 Hz。XR 固定 17 字节额外开销可能改变带宽预算，必须在发送频率和双向 payload 全量确定后验算。
7. 当前 `GlobalPathX/GlobalPathY` 发送被注释，不应在第一阶段隐式启用。

## 11. XR 迁移不变量

第一阶段 XR migration 应满足：

- 保留本文件定义的 payload 字段、单位、方向和 ROS 映射；
- 只替换外层串口承载、生命周期、设备选择和校验实现；
- 不修改 MINCO、Nav2、行为树、规划器或 ROS 消息定义；
- 旧 `communication` 包继续可作为回滚实现；
- 新 XR 节点与旧节点不能同时打开同一串口；
- 在当前下位机未实现对应 XR topic/payload 之前，只能完成主机侧和模拟链路验证，不能宣称已具备实车互通能力。
