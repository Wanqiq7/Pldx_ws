# 导航侧视觉接口消费清单

调研范围：当前仓库 `/home/wanqiq/桌面/navi_minco_bit_jazzy` 与视觉工程
`/home/wanqiq/2026Sentry/PLDX_Sentry_Vision` 的源码。本文区分“导航当前真实消费的接口”、
“导航代码预留但未接入的接口”和“视觉工程已提供但导航侧尚未消费的接口”。

## 结论

当前导航工程直接依赖 `pldx_vision_interfaces` 并订阅视觉工程发布的
`vision/target_observation`。目标有效性、位置和装甲板 ID 均由该类型化接口提供，
再由 `bt_manager` 按时间戳 TF 转换后写入行为树黑板。

`/sentry/offline_info` 仅承载 MCU 机构、云台 yaw 和底盘 IMU 状态，不再承载视觉目标字段。

导航感知侧另有一个实验性/预留接口 `/vision/targets_array`，类型为
`geometry_msgs/msg/PoseArray`。它只被 `dbscan_cluster` 用于把视觉位置与激光聚类目标关联，
当前没有下游规划器或行为树消费其输出，而且视觉工程源码没有发布这个话题。

`auto_aim_target_pos` 仍是视觉工程保留的旧兼容话题，但不属于导航正式输入；不能把它重新
接回 BT，也不能直接把旧 `PoseArray` 订阅当成 `TargetObservation` 的兼容接口。

## 1. 当前生产消费：`/sentry/offline_info`

| 项目 | 当前约定 |
|---|---|
| ROS 话题 | `/sentry/offline_info` |
| 类型 | `ros_interfaces/msg/SentryInfoOffline` |
| 发布者 | `PLDX_Sentry_Vision`，从 MCU/LibXR 接收后发布 |
| 直接消费者 | `decision/bt_manager` |
| MCU 状态字段 | `yaw_camerainit_to_gimbal`、升降、变形、电容、隧道航向、底盘 IMU yaw |

消息保留 `yaw_camerainit_to_gimbal` 作为 `camera_init` 到 `gimbal` 的 yaw 反馈；视觉目标字段
已从 ROS 消息和 MCU revision 3 payload 删除：

- [SentryInfoOffline.msg](/home/wanqiq/桌面/navi_minco_bit_jazzy/src/ros_interfaces/msg/SentryInfoOffline.msg:1)

行为树回调的实际消费行为：

- `lifter_current_pos`、`is_transformable`、`transform_state`、`tunnel_yaw_aligned` 写入对应状态；
- `yaw_camerainit_to_gimbal` 更新 `TransformUtils` 的云台初始 yaw；
- `chassis_imu_yaw` 仍出现在 `/sentry/offline_info`，通信层不再用它做速度旋转变换。

证据：[ros_interface.cpp](/home/wanqiq/桌面/navi_minco_bit_jazzy/src/decision/bt_manager/src/ros_interface.cpp:323)、
[tf_utils.cpp](/home/wanqiq/桌面/navi_minco_bit_jazzy/src/decision/bt_manager/src/utils/tf_utils.cpp:116)。

`/sentry/offline_info` 的发布者是视觉工程的 LibXR 通信节点：
它们从 STM32/LibXR 的 `SentryInfoOffline` payload 读取字段后再发布 ROS 话题，源码并未证明
这个 payload 一定由当前 `PLDX_Sentry_Vision` ROS 进程提供。因此这里是“当前 BT 使用的视觉
相关数据边界”，不是两个工程已经建立的直接 ROS 链路。旧通信映射见
XR 映射见
[ros2libxr.cpp](/home/wanqiq/桌面/navi_minco_bit_jazzy/src/navigation/ros2_libxr/src/ros2_libxr/src/ros2libxr.cpp:113)。

`SentryInfoOffline` 的目标字段已退出；BT 的正式目标输入只有
`vision/target_observation`，并执行有效标志、ID、置信度、时间戳和 TF 校验。

## 2. 预留但未接通：`/vision/targets_array`

`dbscan_cluster` 声明了以下订阅：

| 项目 | 当前约定 |
|---|---|
| ROS 话题 | `/vision/targets_array` |
| 类型 | `geometry_msgs/msg/PoseArray` |
| QoS | `rclcpp::SensorDataQoS()` |
| 消费者 | `dbscan_cluster_node` |
| 使用字段 | 每个 `Pose.position` 的 x/y/z |
| 丢失字段 | 目标 ID、置信度、有效标志、时间戳语义、速度 |

回调只保存位置，并把速度固定为零：

- [ros_interface.cpp](/home/wanqiq/桌面/navi_minco_bit_jazzy/src/perception/dbscan_cluster/src/ros_interface.cpp:67)
- [ros_interface.cpp](/home/wanqiq/桌面/navi_minco_bit_jazzy/src/perception/dbscan_cluster/src/ros_interface.cpp:75)

该回调没有读取 `PoseArray.header`，也没有记录接收时间或超时；只要缓存非空，后续点云回调
就会继续使用最后一组点。话题名以 `/` 开头，是绝对名称；而 PLDX 的类型化话题是相对名称，
会被 `ROS_NAMESPACE` 前缀（例如 `/red_standard_robot1/vision/target_observation`），不能
仅靠启动两个节点就自动匹配。

后续点云回调使用固定的 `camera_init <- gimbal` TF，把视觉点转换后与激光聚类目标按
0.4 m 阈值关联，并将关联速度送入跟踪器：

- [ros_interface.cpp](/home/wanqiq/桌面/navi_minco_bit_jazzy/src/perception/dbscan_cluster/src/ros_interface.cpp:111)
- [kalman_tracker.cpp](/home/wanqiq/桌面/navi_minco_bit_jazzy/src/perception/dbscan_cluster/src/kalman_tracker.cpp:149)

该节点发布 `/clustered_obstacles`（`ros_interfaces/msg/DynamicObstacleArray`），但当前
仓库没有搜索到导航规划器、控制器或行为树订阅该话题；它不是当前生产导航闭环的一部分。
`dbscan_cluster` 也没有出现在根 `start.bash` 或 Nav2 启动链路中。

## 3. 视觉工程已经提供、导航侧尚未消费：`TargetObservation`

`PLDX_Sentry_Vision` 在 ROS 依赖存在时发布：

| 项目 | 当前约定 |
|---|---|
| ROS 话题 | `vision/target_observation` |
| 类型 | `pldx_vision_interfaces/msg/TargetObservation` |
| QoS | Reliable、KeepLast(1) |
| 坐标系 | `header.frame_id = "vision_gimbal"` |
| 位置 | `position`，米 |
| 速度 | `velocity`，m/s；当前视觉发布代码明确填零，因为速度不是 map 绝对速度 |
| 目标标识 | `target_id`，1-based；0 表示无效 |
| 质量/有效性 | `confidence`、`valid` |

发布代码和字段填充：

- [publish2nav.cpp](/home/wanqiq/2026Sentry/PLDX_Sentry_Vision/io/ros2/publish2nav.cpp:15)
- [publish2nav.cpp](/home/wanqiq/2026Sentry/PLDX_Sentry_Vision/io/ros2/publish2nav.cpp:42)
- [TargetObservation.msg](/home/wanqiq/2026Sentry/pldx_ws/src/interfaces/pldx_vision_interfaces/msg/TargetObservation.msg:1)

视觉工程的 CMake 只有在发现 `ament_cmake`、`rclcpp`、`std_msgs`、
`rosidl_typesupport_cpp` 和 `pldx_vision_interfaces` 时才编译生产 ROS 可执行文件；
`sp_msgs` 仅控制旧兼容订阅是否额外编译，见
[CMakeLists.txt](/home/wanqiq/2026Sentry/PLDX_Sentry_Vision/CMakeLists.txt:165)。

当前导航仓库没有 `pldx_vision_interfaces`、`TargetObservation` 或
`vision/target_observation` 的源码引用。因此这个接口目前没有导航消费者。

要把它作为当前导航的正式视觉输入，至少需要明确以下适配边界：

1. 在导航侧引入 `pldx_vision_interfaces` 依赖或部署一个独立 bridge；
2. 按消息时间戳将 `vision_gimbal` 转换到导航使用的 `map`/`camera_init` 坐标系；
3. 把 `valid`、`target_id`、`confidence` 和超时转成行为树需要的 `target_valid`、
   `target_armor_id`、`target_pose`；
4. 决定是否把该接口同时接入动态障碍关联。直接转成旧 `PoseArray` 会丢失 ID、置信度和
   有效性，不应默认视为等价替换。

外部 `/home/wanqiq/2026Sentry/pldx_ws/src/vision_nav_bridge` 可作为独立适配器：它订阅
`vision/target_observation`（并可兼容 `auto_aim_target_pos`），默认拒绝置信度低于 `0.4`
或年龄超过 `200 ms` 的 typed 样本，按消息时间戳查找 `vision_gimbal -> map` TF（超时
`50 ms`），再发布 `vision/target_observation_map`。该 bridge 不在当前导航仓库中，且当前
仓库没有订阅其输出；部署 bridge 本身不会让 BT 或 DBSCAN 自动消费目标。

证据：[vision_nav_bridge.cpp](/home/wanqiq/2026Sentry/pldx_ws/src/vision_nav_bridge/src/vision_nav_bridge.cpp:6)。

## 4. 兼容输出：`auto_aim_target_pos`

视觉工程还发布：

```text
auto_aim_target_pos : std_msgs/msg/String
格式：x,y,valid,target_id
```

这是旧兼容接口，代码与类型化话题同时发布：

- [publish2nav.cpp](/home/wanqiq/2026Sentry/PLDX_Sentry_Vision/io/ros2/publish2nav.cpp:15)
- [publish2nav.cpp](/home/wanqiq/2026Sentry/PLDX_Sentry_Vision/io/ros2/publish2nav.cpp:48)

当前导航仓库同样没有该话题的订阅者。即使增加订阅，也需要自己处理字符串解析、单位、
坐标系和超时；它不包含 `header.frame_id`，不适合作为新的正式接口。

## 5. 反向接口：导航并不消费，而是应发布给视觉

视觉工程订阅：

```text
vision/target_directive : pldx_vision_interfaces/msg/VisionTargetDirective
```

它用于导航/裁判侧向视觉发送目标排除列表和优先目标列表：

- [subscribe2nav.cpp](/home/wanqiq/2026Sentry/PLDX_Sentry_Vision/io/ros2/subscribe2nav.cpp:23)
- [VisionTargetDirective.msg](/home/wanqiq/2026Sentry/pldx_ws/src/interfaces/pldx_vision_interfaces/msg/VisionTargetDirective.msg:1)

当前导航工程没有该消息类型，也没有发布 `vision/target_directive` 的代码。现有 BT
通信接口发布的是 `/sentry/behaivor_send`，它面向 MCU/底盘，不是视觉目标选择接口：

- [ros_interface.cpp](/home/wanqiq/桌面/navi_minco_bit_jazzy/src/decision/bt_manager/src/ros_interface.cpp:88)

所以 `VisionTargetDirective` 属于后续双向视觉集成的导航输出接口，不属于当前导航已经消费
的视觉输入。

## 接入优先级

| 优先级 | 接口 | 判断 |
|---|---|---|
| P0 | `TargetObservation` | 导航正式视觉输入，直接由 `bt_manager` 消费 |
| P1 | `VisionTargetDirective` | 若需要导航控制视觉选敌/禁选，增加导航到视觉的反向链路 |
| P2 | `PoseArray /vision/targets_array` | 仅在继续使用 DBSCAN 视觉-激光关联实验时接通 |
| P2 | `auto_aim_target_pos` | 仅作为旧系统兼容，不建议作为新正式接口 |

## 当前验收命令

在同一 ROS domain、同一命名空间下运行系统后，建议按以下顺序确认：

```bash
ros2 topic info /sentry/offline_info -v
ros2 topic info /vision/targets_array -v
ros2 topic info /vision/target_observation -v
ros2 topic info /vision/target_directive -v
ros2 node info /dbscan_cluster_node
```

预期现状是：`/sentry/offline_info` 只有 MCU 状态消费者；`vision/target_observation` 有视觉
发布者和 BT 单一消费者；`auto_aim_target_pos` 不应出现在导航订阅列表中。
