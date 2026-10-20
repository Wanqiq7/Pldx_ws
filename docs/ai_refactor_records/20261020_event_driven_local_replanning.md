# 按需局部重规划与参考保活

> 历史记录：此策略已撤回，当前采用 1 秒强制局部重规划。下文实现和验收数据对应撤回前版本，现行规则见 [规划器说明](../../src/navigation/minco_planner/README.md#-状态机)。

局部轨迹在剩余时间不足、轨迹不安全或当前位置偏离轨迹时重新优化。目标位置变化继续走已有的全局搜索入口。FSM 保持 50 ms 轮询，只要 `planningInProgress()` 为真就调用 `planAsync()` 领取结果，即使当前轨迹已经恢复正常，也会提交已完成的有效候选。

## 续接与跟踪阈值

配置位于 `src/navigation/navi2_bringup/params/sentry1.yaml` 的 `MincoPlanner.scheduling`，均在 configure 阶段加载：

| 参数 | 默认值 | 含义 |
| --- | ---: | --- |
| `replan_lead_time_sec` | 0.8 s | 按时钟或机器人空间进度计算的剩余时间，任一低于此值时续接 |
| `tracking_error_threshold` | 0.5 m | 当前位置与轨迹当前时刻位置的平面距离上限 |
| `reference_keepalive_sec` | 0.5 s | 安全有效轨迹的重复发布间隔，使用稳态时钟 |

有效续接阈值取配置值与下式的较大值：

```text
planning_budget_sec + 2 × 50 ms + control_budget_sec + safety_budget_sec
```

两个周期分别覆盖跨过阈值后等待 FSM 派发，以及计算完成后等待 FSM 领取结果；控制与安全预算预留同一执行线程上的提交和回调时间。当前预算下下限为 0.135 s。滚动续接优化后，默认实际使用 0.8 s，同时覆盖当前 MPC 约 0.55 s 的前视与延迟补偿，避免控制器在新轨迹提交前读到旧轨迹尾端停车参考。调整 MPC 时应同步核对此值。该提前量提供正常调度下的余量，普通 Linux 调度和协作式计算预算不构成硬实时保证。

空间进度与控制器采用相同的最近采样点及相邻线段投影；热启动也从这一进度继承速度、加速度和剩余轨迹初值。中间截断点保持非零末速，仅全局路径实际终点使用零末速。最新实现与仿真结果见 [滚动续接验收](20261020_rolling_continuation.md)。

跟踪偏差超限时，从实际位姿和实测速度重新初始化，避免继承偏离轨迹的速度、加速度和优化初值。轨迹过期、安全检查失败或安全检查超预算继续撤销参考，控制器原有的参考/里程计超时保护继续生效。

## 保活协议

规划器缓存已提交的采样指令。20 Hz 安全检查通过后，若达到保活间隔，则重新发布缓存消息，仅更新外层 `MpcPositionCommand.header.stamp`。以下内容保持原值：

- 所有采样点及其 `PositionCommand.header.stamp`、`trajectory_id`；
- 位置、偏航轨迹的 `start_WT`；
- 轨迹总时长和自然递减的剩余时间。

因此外层时间戳表示本次发布时刻，采样点时间戳保留轨迹原始时间基准。控制器用轨迹 ID、采样点原始头、坐标系及采样数量识别同一轨迹；同一轨迹保活刷新接收时间，保留跟踪状态和正在计算的 MPC 输出。轨迹替换、时间基准变化、撤销和执行重启仍更新参考版本，使旧控制计算失效。

保活与工作线程优化独立，计算在途时安全旧轨迹仍可保活；失效、过期、停用和执行关闭后不能保活，撤销时清空指令缓存。保活间隔应显著小于控制器 `FollowPath.reference_timeout_sec`，当前为 0.5 s 对 2.0 s。`/opt_path` 消息频率包含保活，分析实际优化频率应查看规划性能日志或轨迹 ID 变化。

## 验证

回归用例覆盖按剩余时间续接及预算下限、安全轨迹持续使用、按当前轨迹时间计算跟踪偏差、目标更新、异步结果轮询、ROS 消息实际保活交付、时间基准和采样内容保持、碰撞与过期撤销，以及 MPC 计算期间的保活、替换和撤销。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --symlink-install \
    --packages-select minco_planner minco_controller --executor sequential
ROS_DOMAIN_ID=217 colcon test --packages-select minco_planner minco_controller \
    --event-handlers console_cohesion+ --executor sequential
ROS_DOMAIN_ID=217 colcon test --event-handlers console_cohesion+ --executor sequential
colcon test-result --verbose
```

验证结果：

- `minco_planner`、`minco_controller` 构建通过。
- 规划器 30 项用例通过；控制器 10 项参考新鲜度用例与 5 项求解器用例通过，既有 MPC 基准测试保持禁用。
- 全工作区 `colcon test` 完成 18 个包；`colcon test-result --verbose` 汇总 65 项测试、0 errors、0 failures、1 skipped。
- C++ 格式检查、`git diff --check` 和 YAML 配置解析通过。

测试结果扫描仍会提示既有 `build/sdformat_tools/package.xml` 缺失，不影响上述测试结果。本次未启动实机硬件。

## 初次仿真验收（2026-10-20，旧 0.2 s 提前量）

以下保留首次验收时的配置与数据；最新滚动续接和到点精度结果见上述独立记录。完整运行 rmuc_2025、无界面 Gazebo、Point-LIO、GICP、Nav2、MINCO/MPC。GICP 通过三个独立窗口确认定位，导航 lifecycle 节点全部 active；通过进程内存映射确认规划器和控制器加载当时工作区动态库。使用独立 `ROS_DOMAIN_ID=222`、`GZ_PARTITION=minco_event_replan_acceptance_20261020`。临时 YAML 仅开启规划器、控制器及地图性能记录；当时调度参数实读为 0.2 s、0.5 m、0.5 s。

```bash
ROS_DOMAIN_ID=222 GZ_PARTITION=minco_event_replan_acceptance_20261020 \
    ./start_sim.sh headless:=true rviz:=false \
    params_file:=/tmp/minco_event_replan_acceptance_20261020/params.yaml
```

订阅 `/opt_path`、`/plan`、`/sim/cmd_vel`、`/sim/ground_truth/odom` 并读取 TF。保活校验逐字段规范化整条消息，仅排除外层发布时间，确认采样点、点时间戳、轨迹编号和其他字段保持一致。

| 验收项 | 实测结果 |
| --- | --- |
| 安全轨迹持续使用 | 轨迹 50 连续使用 2.75 s，无每秒强制替换；保活间隔 0.501～0.553 s |
| 超过参考超时仍可跟踪 | 同一轨迹保活到原始年龄 2.65 s，超过控制器 2.0 s 的参考超时；轨迹年龄超过 2 s 后仍收到 75 条运动命令，采样内容和原始时间基准保持一致 |
| 到期前续接 | 共观察到 3 次旧轨迹剩余约 0.10～0.15 s 时提交新轨迹；以 0.05 s 采样间隔推导剩余时间区间，不假设末段恰为整采样周期 |
| 跟踪偏差 | 0.2 m/s 限速场景中，日志记录约 0.5 m 超限后转冷启动；此时仍可能接近每秒重规划，触发原因是跟踪偏差 |
| 固定目标及目标更新 | 固定目标 6 s 观察窗口 `/plan` 仅 1 次；更新为 (5.0, 5.5) 后恰新增 1 次，收到 `/plan` 约 19.8 ms；最终 NavigateToPose status=4 |
| 取消和同目标重启 | 取消 status=5；重新提交 (4.5, 6.5) 后仅新增 1 次 `/plan`，最终 status=4 |
| 不安全参考撤销 | 插入静态墙体后观察到 4 次旧轨迹未到期即撤销；各撤销到新参考区间内的底盘命令均为零 |
| 障碍移除后的恢复 | 移除墙体后，导航至 (4.5, 8.5) 和返回 (4.5, 7.3) 均 status=4；停车零命令到达后没有再次出现非零命令 |

墙体通过 Gazebo 服务插入，大小为 3.0 × 0.5 × 2.0 m，位于机器人前方约 0.65 m，经模拟激光雷达和地图链路生效。4 次撤销的具体记录如下：

| 轨迹编号 | 撤销时剩余时间区间 | 撤销至新参考 | 期间零命令数 |
| --- | --- | --- | ---: |
| 63 | 2.01～2.06 s | 554 ms | 57 |
| 66 | 4.00～4.05 s | 252 ms | 28 |
| 75 | 3.95～4.00 s | 53 ms | 6 |
| 77 | 2.40～2.45 s | 102 ms | 11 |

这些时间是观察节点的消息接收间隔，不能等同于完整感知制动延迟或物理停止时间。墙体存在期间观察撤销与重新规划，之后移除并重新提交目标；没有把这项检查当作墙体保留情况下的完整绕障到达验收。另一处初次墙体放置未触发撤销，不作为该分支通过的证据。

规划 CSV 最终记录 69 次局部重规划：65 次成功，4 次碰撞校验拒绝。成功记录的 `total_replan_time_ms` 为 p50 3.04 ms、p95 10.50 ms、最大 24.32 ms；该计时不含全局搜索及等待 FSM 提交的时间，也不覆盖未写入完成记录的中断任务，不作为整条规划链路的硬实时上界。

到点精度单独记录：最终通过的导航场景中，地图坐标误差为 0.075～0.215 m，Gazebo 真值误差为 0.042～0.491 m。障碍移除后的目标 (4.5, 8.5) 对应地图误差 0.215 m、真值误差 0.491 m。Nav2 的 0.3 m 目标检查在地图坐标中通过；本次没有将物理到点精度判定为通过，定位与真值偏差仍需单独诊断。

运行证据保存在本机临时目录 `/tmp/minco_event_replan_acceptance_20261020/`：`simulation.log`、`navigation.log`、`navigation_results.json`、`long_run/`、`obstacle_run2/`、`recovery_run/`、`references.jsonl`、`minco.csv`、`mpc.csv`、`loaded_plugins.json`、`summary.json` 及对应检查脚本。离线复查命令：

```bash
python3 /tmp/minco_event_replan_acceptance_20261020/summarize.py
```

核心导航、长距离、有效障碍、恢复导航和离线汇总检查均退出 0。验收后已关闭本次仿真，无导航或 Gazebo 进程残留。关闭阶段速度适配节点记录 Python 退出异常，Gazebo 在 SIGINT 超时后由 launch 发送 SIGTERM 终止；上述记录发生在验收完成后的关闭阶段。
