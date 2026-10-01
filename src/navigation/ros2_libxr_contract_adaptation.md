# 导航通信契约

## 结论

导航工程只保留 ROS2 topic 和 payload 契约。LibXR topic 注册、USB CDC 收发和下位机反馈发布由 `PLDX_Sentry_Vision` 负责。当前为通信协议 revision 3，`SentryInfoOffline` 已移除视觉目标字段并缩短为 16 B。

当前节点使用：

| 方向 | LibXR topic | payload | ROS 接口 |
|---|---|---|---|
| 下位机 -> 主机 | `team_info` | `TeamInfo` | `/sentry/team_info` |
| 下位机 -> 主机 | `game_info` | `GameInfo` | `/sentry/game_info` |
| 下位机 -> 主机 | `online_info` | `SentryInfoOnline` | `/sentry/online_info` |
| 下位机 -> 主机 | `offline_info` | `SentryInfoOffline` | `/sentry/offline_info` |
| 下位机 -> 主机 | `radar_info` | `RadarInfo` | `/sentry/radar_info` |
| 主机 -> 下位机 | `nav_data` | `ChassisTarget` | `/cmd_vel_mpc`、`/cmd_force_mpc`、`/aft_mapped_to_init` |
| 主机 -> 下位机 | `behavior_data` | `BehaviorData` | `/sentry/behaivor_send` |

本工程契约要求的接口见 `communication_protocol_contract.md`：

- 输入：`/cmd_vel_mpc`、`/cmd_force_mpc`、`/aft_mapped_to_init`、`/sentry/behaivor_send`；
- 输出：`/sentry/team_info`、`/sentry/game_info`、`/sentry/online_info`、`/sentry/offline_info`、`/sentry/radar_info`；
- 发送 payload：`ChassisTarget`（45 字节）和 `BehaviorData`（26 字节，末尾 `use_gyro_mode`），每 10 ms 各发送一次；主机恒发 `use_speed_control=true` 的车体系速度；
- 接收 payload：`TeamInfo`（48）、`GameInfo`（22）、`SentryInfoOnline`（37）、`SentryInfoOffline`（16）、`RadarInfo`（59）。当前为通信协议 revision 3；revision 2 的 30 B payload 不兼容。

节点现在已完成主机侧话题和 payload 适配。节点可以正常启动或持续等待 USB 设备，并不代表协议互通；必须先确认固件端定义一致。

## 正确适配方式

视觉侧适配需要保持以下约束：

1. 当前注册表使用 `nav_data`、`behavior_data`、`team_info`、`game_info`、`online_info`、`offline_info`、`radar_info`。这些名称是本工程适配约定，仍必须与固件确认的 LibXR domain 和版本逐项核对。
2. 使用独立的 wire header 定义上述七个 payload，固定 `#pragma pack(1)`、小端序、IEEE-754 float、`bool` 的单字节表示，并加入 `sizeof`、`offsetof`、`std::is_trivially_copyable` 的静态断言或测试。
3. ROS 输入只保留最新快照。订阅 `/cmd_vel_mpc`、`/cmd_force_mpc`、`/aft_mapped_to_init` 和 `/sentry/behaivor_send`，在 10 ms 定时器中构造 `ChassisTarget` 与 `BehaviorData` 后分别 `Publish`。世界系 vx/vy 用 `θ = yaw(camera_init→base_link)` 折到 MCU 车体：`vx_mcu = vx·sinθ − vy·cosθ`（右），`vy_mcu = vx·cosθ + vy·sinθ`（前）。该 TF 不可用时抑制平移/力指令。
4. 为每个接收 topic 注册长度检查回调，将 payload 映射回 `ros_interfaces` 的五个 `/sentry/*` 消息。雷达坐标按契约从 cm 转 m；revision 3 的 `SentryInfoOffline` 仅映射机构、云台 yaw 和底盘 IMU 状态。`chassis_imu_yaw` 不再参与主机速度旋转变换。
5. 让串口 VID/PID、selector、波特率和周期成为 ROS 参数。官方当前硬编码 `16d0:1492`、selector `navigation`、115200 8N1；这些值只能作为默认值，必须用实车枚举结果确认。
6. 在协议双方完成前只做回环/模拟测试。

## 当前仓库状态

- 导航工程不构建或启动 USB 通信节点。
- 视觉工程只创建一个 `LinuxUART`，并注册完整导航 payload topic。

完成适配的验收顺序应为：payload 布局测试 -> LibXR 主机/固件回环 -> ROS topic 映射测试 -> 串口实测；每一步都应检查 topic 名、payload 长度和 CRC/传输错误。
