# 大 Yaw 底盘状态软件验收

日期：2026-11-27。基线：`935c3eb2`。实现：`ca5f8185` (#7)、`364ad3b5` (#8)、
`da5ea588` (#9/#10/#11工具)，通信子模块 `d51e824`，仿真子模块 `3448c8e`。

## 四工况公共 ROS 回放

`chassis_state/test/test_ros_replay.py` 在本机独立 ROS_DOMAIN_ID 下运行真实状态转换节点、
真实planner/MPC插件和注入传输的真实通信桥，不接 USB。传入同时间戳的原始 IMU odom 与关节反馈，使用
独立前向传感器位姿数值求导提供速度真值。包含倾斜 IMU、偏移转轴、反向关节及跨 ±pi。

| 工况 | 样本 | 最大状态误差（m、rad、m/s、rad/s） |
| --- | ---: | ---: |
| stationary_sweep | 35 | 2.01e-11 |
| locked_joint_rotation | 35 | 2.26e-11 |
| simultaneous_rotation | 35 | 3.11e-11 |
| world_translation | 35 | 3.11e-11 |

每个样本检查恢复状态、planner姿态/世界速度、MPC预测首点，容差 1e-7；完整 LibXR 包解析得到的 XY 与真实MPC世界指令经物理底盘 yaw
旋转的期望一致，容差 2e-6。验证物理 TF 重建原始 LIO 位姿、fake 在 camera_init 中
单位朝向、实际唯一动态 TF 发布者和静态 body/小云台挂接。差分线速度和角速度按测量
时间区间中点核对，容差 1e-7。30°雷达转动而底盘0°另有核心测试。

## 消费者和回归

- 真实 MPC 插件公共 ROS 输入测试：90°姿态、0.1s控制延迟、非零yaw率、干扰原始LIO、
  虚拟零yaw及旧offset参数；预测初始位置/方向容差1e-9。
- 真实 planner 插件：map 90°旋转和10m平移，同参考点、同测量stamp、world速度；1e-9。
- 通信桥：倾斜底盘、测量age、原始LIO扫转干扰、完整编码帧；既有超时/隧道/gyro回归。
- 仿真测量反馈：保持 JointState 同样本 position、velocity、timestamp。
- `colcon test --build-base build-link --executor sequential --event-handlers console_cohesion+`
  全工作区执行；首次 chassis_state 未发现测试，补充pytest声明后单包重跑通过。
- 最终 `colcon test-result --test-result-base build-link --verbose`：
  **170 tests, 0 errors, 0 failures, 1 skipped**（既有禁用benchmark）。
- 已构建 ros_interfaces、chassis_state、minco_controller、minco_planner、ros2_libxr、
  navi2、point_lio、sentry_simulation；Python语法、默认launch解析、diff检查通过。

本报告覆盖合成 ROS 链和公共插件输入测试，未运行真实硬件运动，也未把整套 Gazebo
场地导航作为本轮新增验收。真实零位、方向、传动、安装与时间映射误差仍交 #11。
实机方法与报告格式见 [台架验收](yaw-chassis-bench.md)。

## 审查修正

Standards轴无具体违规。Spec轴发现并修正两项：仿真此前将IMU固定底盘且点云去除
关节运动；现已将IMU移至大Yaw实体，点云输出保留同一测量时刻的关节运动，相关
转换和模型测试通过。四工况此前未经过真实导航插件；现已逐样本经过planner、MPC
和通信编码。两个问题均补充回归验证。

台架记录器经过合成静止样本烟测；报告保持待人工判定。Jazzy rclpy 不提供逐消息
writer GID，记录器明确标记 TF child 归属需人工结合端点列表核查。
