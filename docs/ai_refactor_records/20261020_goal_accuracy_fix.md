# 仿真到点误差诊断与修复

2026-10-20，rmuc_2025、omni、无界面 Gazebo、双 MID360、Point-LIO、GICP、MINCO/MPC。已复现“地图坐标判定成功，但物理到点偏差较大”的问题，分别处理固定定位偏差、运动中提前成功、末端搜索退化和上台后的 IMU 积分漂移。最终六目标路线及障碍移除后的原路线共八次导航全部成功，成功时真值误差为 **0.014～0.054 m**，停车后为 **0.015～0.055 m**。

## 测量方法与复现证据

新增 `scripts/check_sim_goal_accuracy.py`，同时记录 `/sim/ground_truth/odom`、`/aft_mapped_to_init`、`/sim/cmd_vel` 和完整 TF 链。对每个真值时间戳查 `map -> base_link`，区分：

- 地图到点误差：`map/base_link` 到地图目标的 XY 距离。
- 真值到点误差：Gazebo 模型原点到相同目标的 XY 距离。
- 定位误差：同一时间戳的 `map/base_link` 与 Gazebo 模型原点的 XY 距离。
- 高度漂移：`map/base_link.z - truth.z` 相对初始静止值的变化，消除导航 Z 原点约定产生的常数项。

收到 action 结果后继续收集 0.3 s 数据，再回查结果接收时刻的真值及 TF；八次导航的采样时差绝对值最大 5 ms。另在 1.5 s 后检查停车位置、速度和命令。成功时真值平面速度及角速度均须不超过 0.15，停车后不超过 0.02；停车命令各轴不超过 0.01。脚本默认要求真值到点误差不超过 0.15 m、定位 XY 误差不超过 0.12 m、高度漂移不超过 0.2 m。

原事件重规划验收记录的 `(4.5, 8.5)` 误差是地图 **0.215 m**、真值 **0.491 m**，见同目录 `20261020_event_driven_local_replanning.md`。本次重启后，同类偏差再次出现：

| 轮次 | `(4.5, 8.5)` 地图误差 | 真值误差 | 说明 |
| --- | ---: | ---: | --- |
| `baseline` | 0.246 m | 0.467 m | 初版探针取最近缓存，约落后结果时刻 0.15 s，保留为复现线索 |
| `baseline_exact` | 0.140 m | 0.321 m | 改为结果时刻同步采样；停车后真值误差仍为 0.292 m |
| `validated/recovery` | 0.026 m | 0.035 m | 最终修复后，插墙、撤销、移除墙体，再提交原目标 |

前两轮都在出生静止时测得约 0.22～0.23 m 定位偏差。复现的是同一类故障，数值随停车、定位瞬态和采样时刻变化；不能将不同测量方法的数据当作严格的同条件性能比较。

## 根因与改动

### 1. rmuc_2025 的固定地图配准偏差

静止回归 `regression_before` 报告定位误差 **0.222 m > 0.12 m**，无需导航即可触发。不同朝向停车后，误差主要沿世界 +Y，未随车体方向旋转，因此固定地图配准比雷达杆臂错误更符合证据。

用 58 个已停车样本估计平面变换：

```text
T_new_map_pcd = T_truth_base × inverse(T_old_map_base) × T_old_map_pcd
```

在仿真子模块 `sentry_simulation/config/worlds.yaml` 中，仅修正 `rmuc_2025.icp.pcd_to_map`：

```yaml
# 原 [x, y, yaw]
[-0.18495, 0.38684, -0.022672]
# 修正后
[-0.03036, 0.04827, 0.002257]
```

计算包含已有 `body -> base_link` 的 0.2 m 平面杆臂。保留样本的变换后 XY 误差为 0.003～0.043 m；独立重启的 `calibrated` 出生误差为 0.0053 m。没有修改实车 TF、雷达外参或在运行中使用真值修正定位；真值只用于离线标定和验收。当前点云适配器原有的仿真扫描运动处理保持原实现。

### 2. 运动中提前宣布成功，且原生停车检查缺少里程计

固定标定修复后，原 0.3 m 到点容差仍允许停车真值误差 0.102～0.162 m。仅将 SimpleGoalChecker 的容差收紧到 0.1 m 仍不充分：`simple_checker_failure/short` 返回 `(4.5, 7.3)` 时，地图误差 0.051 m，但真值误差 **0.214 m**、平面速度 **0.266 m/s**。停车后定位恢复，任务已结束，真值误差仍为 0.198 m。

进一步发现 `controller_server.odom_topic` 默认值为 `odom`，该话题发布者为零。MPC 自己订阅 `/aft_mapped_to_init`，因此车能正常行驶，但 Nav2 原生停车检查没有实际速度输入。只替换检查器而不修复该连接，仍不能依靠它确认停车。

在共享导航配置 `src/navigation/navi2_bringup/params/sentry1.yaml` 中使用原生能力：

```yaml
controller_server:
  ros__parameters:
    odom_topic: /aft_mapped_to_init
    general_goal_checker:
      plugin: "nav2_controller::StoppedGoalChecker"
      stateful: False
      xy_goal_tolerance: 0.1
      yaw_goal_tolerance: 6.28
      trans_stopped_velocity: 0.1
      rot_stopped_velocity: 0.1
```

位置和速度每次同时检查。`stateful: False` 防止位置在运动中短暂满足后被锁存；已有 `controller_odom_topic` remap 同时作用于原生检查和 MPC。实读参数与 `/controller_server` 订阅均已保存。yaw 继续允许任意朝向到达，但成功时要求角速度降下来。

### 3. 更严格到点要求暴露 SMAC 单点路径退化

距目标约 0.12 m 时，起点已在 SMAC 的 0.3 m 搜索容差内，搜索立即返回只有起点的路径；调用方要求至少两个点，因而末端一直无法重新规划。`approach_failure` 记录起点格 `(92,145)`、目标格 `(89,145)`、分辨率 0.05 m。

先加入回归 `SmacGoalApproach.PlansToGoalWhenStartAlreadyInsideSearchTolerance`，确认旧实现失败，再修改 `src/navigation/minco_planner/src/smac_search/smac_planner_2d_simple.cpp`：起点已经位于容差范围内时，搜索继续到目标栅格，避免仅返回起点。其余情况保留原搜索容差。测试检查真实搜索结果具有多个点并到达目标格，红、绿日志分别为 `smac_red.log`、`smac_green.log`。

### 4. 200 Hz 仿真 IMU 对台阶接触冲击欠采样

短程通过后，长程 `(4.5,14) -> (4.5,6.5)` 暴露另一个问题：实际只上了约 0.203 m 高台，LIO 却持续向上漂移。`final/long` 曾在真值误差 **0.316 m、0.409 m** 时返回成功，后续高度漂移达到数十米。该轮不作为验收通过数据。

隔离采集的 `covariance_baseline` 表明：

- raw 和适配后 IMU 的加速度、角速度完全相同，最大单轴加速度 617.4 m/s²，未触发已有的 1000 m/s² 裁剪。因此量程裁剪不符合这轮漂移证据。
- IMU 每 5 ms 采样，物理默认每 1 ms 更新。上台后，8～14 s 窗口的竖直加速度积分残留约 **0.643 m/s**，而末端真值速度为零。
- 该轮 XY 检查曾通过，但补充高度审计发现检查点漂移 0.72、1.13、9.60、8.75 m。早期脚本尚无高度断言，不能据其旧 `violations=[]` 宣称完整通过。
- 单变量尝试把 Point-LIO `vel_cov` 从 0.1 改为参考工程默认的 20，回程高度仍漂移到 13.59 m。因此恢复 0.1，不将该调整留在最终配置。

最终仅匹配仿真采样与物理步：

| 文件 | 修改 |
| --- | --- |
| `sentry_simulation/resource/models/sentry_omni/model.sdf` | IMU `update_rate`：200 → 1000 Hz |
| `sentry_simulation/config/point_lio_sim.yaml` | `mapping.imu_time_inte`：0.005 → 0.001 s |

最终轮前 60 s 中，raw IMU 收到 59,999 条，适配后 59,996 条，中位间隔 1 ms、最大间隔 2 ms；诊断订阅端分别少收约 1、4 个采样。用于积分的上下台窗口内 raw IMU 无缺样，间隔均为 1 ms。59,995 对同时间戳 raw/适配样本完全一致，最大单轴加速度 497.7 m/s²。

上台窗口 33～41 s、下台窗口 41～49 s 的两端真值线速度与角速度均为零；将 IMU 加速度按其姿态旋转并减去重力后，竖直积分残留分别为 **−0.000047 m/s、−0.000712 m/s**。姿态消息的初始 yaw 基准不影响竖直积分。此证据与高度检查一起支持欠采样的诊断；不同路线的单次积分数值不作为通用误差上界。

## pb2025_sentry_ws 参考核对

已核对参考工程的导航参数、机器人与 MID360 SDF、点云转换器、MecanumDrive2 插件及 launch：

| 项目 | 参考工程 | 当前工程的处理 |
| --- | --- | --- |
| 控制器里程计 | 显式配置 `odom_topic: odometry` | 补全当前原生控制器的 `/aft_mapped_to_init` 连接 |
| 到点检查 | SimpleGoalChecker、0.15 m | 根据已复现的运动中提前成功，使用 0.1 m StoppedGoalChecker |
| IMU | 200 Hz，单雷达倾斜安装 | 双雷达融合到水平公共帧；omni 提高到逐物理步采样 |
| 重力初始化 | 含 Y 分量，与约 30° 安装匹配 | 保留 `[0,0,-9.81]`，不移植倾斜传感器重力参数 |
| 点云 | 20 Hz 单雷达，转换器构造点时间 | 保留当前双雷达 10 Hz 公共帧及扫描运动处理，避免套用不同时间语义 |
| LIO 体素/速度协方差 | 0.2 m 体素；代码默认 `vel_cov=20` | 不移植未证明有效的参数；`vel_cov=20` 单变量实验未解决高度漂移 |
| 底盘插件 | 同源 P 控制 | 保留当前已有 PI、抗积分饱和和零指令清积分修复 |

没有修改参考工程。参考工程的 200 Hz 设置不能证明当前不同传感器、动力学与路线条件下同样适用。

## 最终验收

使用独立 `ROS_DOMAIN_ID=223`、`GZ_PARTITION=minco_goal_accuracy_1khz_run1`，重启并确认正确出生位姿和唯一 `/clock` 发布者。一次重启因旧 Gazebo 残留导致时钟串源，已标记为 `imu_restart_invalid` 并排除；最终轮在清理后使用新 partition。

| 目标，按实际顺序 | 成功时地图误差 | 成功时真值误差 | 停车后真值误差 |
| --- | ---: | ---: | ---: |
| (4.5, 8.5) | 0.051 m | 0.054 m | 0.055 m |
| (4.5, 7.3) | 0.026 m | 0.014 m | 0.015 m |
| (5.0, 5.5) | 0.025 m | 0.028 m | 0.029 m |
| (4.5, 6.5) | 0.032 m | 0.023 m | 0.024 m |
| (4.5, 14.0)，上台 | 0.022 m | 0.024 m | 0.024 m |
| (4.5, 6.5)，下台回程 | 0.024 m | 0.018 m | 0.019 m |
| 障碍移除后 (4.5, 8.5) | 0.026 m | 0.035 m | 0.035 m |
| 障碍移除后 (4.5, 7.3) | 0.034 m | 0.026 m | 0.026 m |

六目标路线的检查点定位 XY 误差最大 0.015 m、相对高度漂移绝对值最大 0.066 m。上台停稳时真值高度约 0.203 m，LIO 高度约 0.22～0.23 m，未再出现持续的米级漂移。所有目标 status=4，停车检查通过。

同步复查之前事件重规划行为：同一轨迹保活至原始年龄 2.20 s，保活间隔 0.498～0.553 s；58 次保活均保持规范化后的完整消息内容一致，原时间基准未重置。两次正常续接在旧轨迹剩余约 0.10～0.15 s 时提交。

前方约 0.65 m 插入 `3.0 × 0.5 × 2.0 m` 静态墙，轨迹 40 尚剩 **1.939～1.989 s** 时撤销；撤销到新参考约 0.600 s，期间 63 条底盘命令均为零。另有一个备份停车参考使用系统时钟，其与仿真时钟相减得到的异常剩余时间被排除，不作为提前撤销证据；该既有发布分支的时钟统一可单独处理。障碍测试覆盖撤销和移除后恢复，没有将其解释为墙体持续存在时的完整绕障到达验收。

本轮规划 CSV 有 52 条完成记录，35 条成功、15 条碰撞拒绝、2 条运动约束拒绝（包含插墙场景）。成功局部重规划的耗时 p50 **2.46 ms**、p95 **5.75 ms**、最大 **5.89 ms**；不含全局搜索、FSM 等待及未完成中断任务，不能作为完整规划链路的最坏延迟。

## 复跑与证据索引

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
# 终端一；每次重启先确认旧 Gazebo 已退出，并使用独立 partition。
ROS_DOMAIN_ID=223 GZ_PARTITION=goal_accuracy_recheck \
    ./start_sim.sh headless:=true rviz:=false
# 终端二；默认检查原往返目标。
ROS_DOMAIN_ID=223 python3 scripts/check_sim_goal_accuracy.py /tmp/goal_accuracy_recheck
# 覆盖短程、上台和下台。
ROS_DOMAIN_ID=223 python3 scripts/check_sim_goal_accuracy.py /tmp/goal_accuracy_route \
    --goals '[[4.5,8.5],[4.5,7.3],[5.0,5.5],[4.5,6.5],[4.5,14.0],[4.5,6.5]]'
# 只检查静止定位，隔离控制器和规划器。
ROS_DOMAIN_ID=223 python3 scripts/check_sim_goal_accuracy.py /tmp/goal_accuracy_stationary \
    --stationary-only
```

本次完整日志保存在 `/tmp/minco_goal_accuracy_20261020/`：

- 固定偏差：`baseline/`、`baseline_exact/`、`regression_before/`、`calibration.json`、`calibrated/`。
- 提前成功及缺少 odom：`simple_checker_failure/short/`、`stopped_without_odom/`、`controller_before_odom.txt`、`odom_before.txt`。
- 末端退化：`approach_failure/`、`smac_red.log`、`smac_green.log`。
- 高度漂移及单变量实验：`final/long/`、`covariance_baseline/sensors.jsonl`、`covariance_baseline/sensor_summary.json`、`covariance_probe/`。
- 最终有效轮：`validated/route/`、`validated/recovery/` 的 `result.json` 和 `samples.jsonl`；`validated/sensors.jsonl`、`sensor_summary.json`、`summary.json`、`performance_summary.json`。
- 运行状态与保活：`validated/controller_parameters.yaml`、`controller_topics.txt`、`lio_parameters.txt`、`clock_publishers.txt`、`references/`、`obstacle/`、`simulation.log`、性能 CSV。
- 离线复核：`summarize_validated.py`；完整测试：`final_workspace_test.log`、`final_test_result.log`。

## 构建、回归及影响范围

相关规划器、导航配置和仿真包构建通过。新增 SMAC 回归由红转绿；最终配置下执行全工作区测试：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=224 colcon test --event-handlers console_cohesion+ \
    --executor sequential --return-code-on-test-failure
colcon test-result --verbose
```

18 个包完成；汇总 **66 tests、0 errors、0 failures、1 skipped**。结果扫描仍提示既有 `build/sdformat_tools/package.xml` 缺失；未删除用户构建产物。Python 格式、C++ 格式、YAML/XML 解析及主仓/仿真子模块 diff 检查均通过。

地图标定和 IMU 频率改动仅作用于仿真；共享 `sentry1.yaml` 的到点容差、StoppedGoalChecker、原生 odom 连接，以及 SMAC 末端搜索修复，也影响使用此配置的实机导航。实机尚未验收，尤其需要核对真实里程计速度和停车阈值。1000 Hz 会增加仿真 IMU 消息处理量，本轮已有导航闭环与规划耗时数据，但未测定所有场景的 CPU 上界。

本轮通过不代表运动中所有 LIO 瞬态误差已消失，也不涵盖其他地图、其他底盘和更强接触冲击。验收完成后已关闭本次监测、记录及导航进程，并对 launch 未回收的 Gazebo 实体进程发送 SIGTERM，确认退出。
