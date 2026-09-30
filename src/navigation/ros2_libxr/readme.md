# ROS2 与 LibXR 融合串口通信框架

本仓库实现 ROS2（上位机）与下位机（嵌入式控制板）之间的**双向串口通信**。
桥接核心借助 [LibXR](https://github.com/Jiu-xiao/libxr) 的 Topic 机制：把下位机发来的二进制数据解析成 ROS2 消息发布出去，同时把 ROS2 侧的指令打包通过 UART 下发给下位机。

整体数据流：

```
下位机  --UART-->  SharedTopic(接收/解析)  -->  LibXR Topic  -->  ROS2 Publisher  -->  ROS2 网络
ROS2 网络  -->  ROS2 Subscriber  -->  LibXR Topic  -->  SharedTopicClient(打包/发送)  --UART-->  下位机
```

## 仓库结构

这是一个 colcon 工作区，`src/` 下包含三个功能包：

| 包名 | 作用 |
| --- | --- |
| `ros2_libxr` | 核心桥接节点。串口收发、LibXR 话题与 ROS2 话题的转换都在这里 |
| `ros_interfaces` | 本工程通信契约消息（Behavior、TeamInformation、GameInfo、SentryInfo*、RadarInfo） |

`ros2_libxr` 通过 git submodule 引入三个外部依赖：

| 子模块 | 路径 | 说明 |
| --- | --- | --- |
| `libxr` | `src/ros2_libxr/libxr` | LibXR 主库，CMake 目标名为 `xr` |
| `SharedTopic` | `src/ros2_libxr/include/SharedTopic` | UART 多话题接收/解析服务端（下位机 → 上位机） |
| `SharedTopicClient` | `src/ros2_libxr/include/SharedTopicClient` | UART 多话题打包/转发客户端（上位机 → 下位机） |

核心代码位置：

- 节点实现：`src/ros2_libxr/src/ros2libxr.cpp`
- 接口、数据结构与话题定义：`src/ros2_libxr/include/ros2_libxr/ros2libxr.hpp`

## 快速开始

### 1. 克隆并初始化子模块

```bash
git clone <repo-url> ros2_libxr
cd ros2_libxr
git submodule update --init --recursive
```

### 2. 构建

在工作区根目录（包含 `src/` 的目录）构建。由于 `ros2_libxr` 依赖两个接口包，首次构建建议整包构建：

```bash
colcon build
source install/setup.bash
```

如只想单独编译桥接节点（接口包已编译过）：

```bash
colcon build --packages-select ros2_libxr
source install/setup.bash
```

### 3. 运行

```bash
# 方式一：直接运行节点
ros2 run ros2_libxr ros2_libxr_node

# 方式二：使用 launch 文件
ros2 launch ros2_libxr ros2_libxr_launch.py
```

> 构建后务必 `source install/setup.bash`，否则运行时可能找不到 `.so`。

## 串口配置

串口在 `RMSerialDriver` 构造函数中通过 `LibXR::LinuxUART` 初始化（`ros2libxr.cpp`）：

```cpp
uart_client = std::make_unique<LibXR::LinuxUART>(
    "16d0", "1492", "XRUSB CDC Control", 921600,
    LibXR::LinuxUART::Parity::NO_PARITY, 8, 1);
```

参数依次为：USB VID（`16d0`）、PID（`1492`）、CDC 控制接口名称（`XRUSB CDC Control`）、波特率（`921600`）、校验位、数据位、停止位。控制接口名称用于在同一个 USB Composite 设备的多个 CDC 中选中导航链路，不能写成 `ttyACM0` 或未限定的 VID/PID。

节点还会启动一个 LibXR 终端线程（`Terminal`），可通过它在运行时查看接收速率等调试信息。

## 话题接口

### 下位机 → ROS2（接收方向）

| LibXR 话题 | 数据结构 | 经过处理后发布的 ROS2 话题 | ROS2 消息类型 |
| --- | --- | --- | --- |
| `team_info` | `TeamInfo` | `/sentry/team_info` | `ros_interfaces/TeamInformation` |
| `game_info` | `GameInfo` | `/sentry/game_info` | `ros_interfaces/GameInfo` |
| `online_info` | `SentryInfoOnline` | `/sentry/online_info` | `ros_interfaces/SentryInfoOnline` |
| `offline_info` | `SentryInfoOffline` | `/sentry/offline_info` | `ros_interfaces/SentryInfoOffline` |
| `radar_info` | `RadarInfo` | `/sentry/radar_info` | `ros_interfaces/RadarInfo` |

说明：

- 接收回调严格检查 payload 长度，再将 revision 3 契约结构映射到五个 `/sentry/*` ROS 话题；revision 2 的 30 B `offline_info` payload 必须拒绝。

### ROS2 → 下位机（发送方向）

| 订阅的 ROS2 话题 | ROS2 消息类型 | 转发到的 LibXR 话题 | 数据结构 |
| --- | --- | --- | --- |
| `/cmd_vel_mpc`、`/cmd_force_mpc` | `geometry_msgs/Twist`、`WrenchStamped` | `nav_data` | `ChassisTarget` |
| `/sentry/behaivor_send` | `ros_interfaces/Behavior` | `behavior_data` | `BehaviorData` |

说明：

- 每 10 ms 保留最新 ROS 快照并发送一次 `ChassisTarget`（45 B，`use_speed_control=true`）和一次 `BehaviorData`（26 B，末尾 `use_gyro_mode`）。
- 用 `θ = yaw(lookup(camera_init, base_link))` 把 map/odom vx/vy 折到 MCU 车体：`vx = vx_w·sinθ − vy_w·cosθ`（右），`vy = vx_w·cosθ + vy_w·sinθ`（前）。该 TF 不可用时抑制平移和 x/y 力。`vw` 保持 MPC `angular.z`（rad/s）。小陀螺由 `use_gyro_mode` 走 ROTOR。

## 如何新增话题

新增话题需要在「话题注册表」和「收发逻辑」两处同步修改，二者的话题名必须与下位机约定一致。

### 接收方向（下位机 → ROS2）

1. 在 `ros2libxr.hpp` 的 `XRobotMain` 中，把话题名加入 **`SharedTopic`** 的列表：

   ```cpp
   static SharedTopic shared_topic_rx(
       hw, appmgr, "uart_client", 256,
       {{"team_info"}, {"game_info"}, {"online_info"}, {"offline_info"}, {"radar_info"}});
   ```

2. 在 `RMSerialDriver` 构造函数中创建对应的 LibXR 话题并赋值给成员变量：

   ```cpp
   your_new_topic_ = LibXR::Topic::CreateTopic<YourDataType>("your_new_topic");
   ```

3. 创建 ROS2 发布者，并注册回调，在回调中把数据转换后用 `self` 发布：

   ```cpp
   void (*cb_fun)(bool, RMSerialDriver *self, LibXR::RawData &data) =
       [](bool, RMSerialDriver *self, LibXR::RawData &data) {
         auto payload = reinterpret_cast<YourDataType *>(data.addr_);
         // ... 转换并 self->your_pub_->publish(...)
       };
   auto cb = LibXR::Topic::Callback::Create(cb_fun, this);
   your_new_topic_.RegisterCallback(cb);
   ```

### 发送方向（ROS2 → 下位机）

1. 在 `XRobotMain` 中，把话题名加入 **`SharedTopicClient`** 的列表：

   ```cpp
   static SharedTopicClient shared_topic_tx(
       hw, appmgr, "uart_client", 16,
       {{"nav_data"}, {"behavior_data"}, {"your_tx_topic"}});
   ```

2. 在构造函数中创建对应 LibXR 话题，并在 ROS2 订阅回调里 `Publish` 给该话题：

   ```cpp
   your_tx_topic_ = LibXR::Topic::CreateTopic<YourTxType>("your_tx_topic");
   // 订阅回调内：
   your_tx_topic_.Publish(payload);
   ```

> 若新增的数据需要对应新的 ROS2 消息类型，记得在 `ros_interfaces` 的 `CMakeLists.txt` 中注册 `.msg` 文件，并在 `ros2_libxr` 中添加依赖。

## 调试要点

- 回调函数必须通过传入的 `self`（`RMSerialDriver*`）访问类成员，不要在 lambda 中直接使用未捕获的 `this`。
- 调试打印三选一：`XR_LOG_INFO(...)`（LibXR）、`RCLCPP_INFO(self->get_logger(), ...)`（ROS2）、`std::cout`。
- LibXR 终端的 `shared_topic:<uart_name>` 命令支持 `monitor` 子命令，可实时查看接收带宽（Mbps），用于排查串口吞吐问题。
- 修改子模块后，注意 `git submodule update --recursive`，避免使用过期的头文件版本。
- 构建后记得 `source install/setup.bash`。
