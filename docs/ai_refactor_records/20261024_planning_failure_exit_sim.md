# 规划失败出口：RMUC2025 仿真验收

日期：2026-10-24。使用 `rmuc_2025`、`robot_model:=real`，打开 Gazebo 和 RViz，
独立 ROS domain 93、Gazebo partition `planning_exit_rmuc2025_1024`。
终点外受控测试使用全新仿真、ROS domain 94、partition `planning_exit_terminal_1024`。
未提交代码。本次不调整规划算法、碰撞几何或半径。

## 已验证结果

**失败上报、行为树重试以及耗尽重试后的主动中止均已有闭环证据。持续长路线
导航仍出现无路，不能把失败出口验收等同于路线稳定性通过。**

| 场景 | 结果 | 判定 |
| --- | --- | --- |
| 第一轮返程，原物理卡点附近 | 约 `(5.8, 3.03)` 处再次无进展；10 s 检查触发 FollowPath 失败，重试后到点 | 恢复重试链路有效 |
| 第二轮去程持续 SMAC 无路 | 8 次规划失败通知分别使 FollowPath 失败，最终 NavigateToPose 返回 `ABORTED=6` | 主动退出链路有效；该段路线失败 |
| 第二组五轮复跑 | 完成 3/10 个目标后，第二轮返程无路，约 26.1 s 主动中止 | 重复观察到有界失败；循环未通过 |
| 障碍内部目标 `(19.975, 3.975)` | 23.54 s 墙钟时间主动中止，停车后命令和真值速度均为零 | 不可达退出通过 |
| 中止后换成 `(4.5, 8.5)` | 新导航成功 | 失败状态不阻止新任务 |
| 终点附近重复下发 | 两个目标共 115 次重发，均成功，最大稳定真值误差 0.07173 m | 正常到点与重复执行通过 |
| 终点容差外受控无进展 | 8 次进度失败，84.04 s 后主动中止，最终地图到点误差 0.19625 m | 未入 0.1 m 容差时也有明确出口 |

第一组循环原定八轮，首个失败即停止；实际完成 2/16 个目标，随后第二轮去程
失败。累计路程 127.062 m，最大同步 XY 定位误差 0.04680 m。失败目标在
约 59.23 s 后由系统主动返回 ABORTED，真值位置约 `(20.593, 14.447)`，
命令和真值速度均为零。测量脚本因此返回非零；并未触发脚本的 90 s 超时取消。

第二组复跑在同一仿真内继续进行，没有清空地图或重启节点。累计路程
144.264 m，最大定位误差 0.05197 m；中止时真值位置约 `(20.665, 12.232)`。
该段又记录到 8 次失败通知、8 次 FollowPath 失败及一次 NavigateToPose 中止。
两组长路线共完成 5 个目标、失败 2 个目标，累计采样路程 271.326 m。
中止后额外静止测量的命令始终为零，最大真值平移速度约 0.00012 m/s，
位置变化小于 0.5 mm。

第一轮返程的进度失败在仿真时间 96.636 s；FollowPath 重试立即重新执行，
NavigateToPose 在 103.390 s 成功，随后稳定真值误差 0.07998 m。

重复下发测试使用 `--resend-seconds 12`：`(4.5, 8.5)` 重发 57 次，
`(4.6, 7.3)` 重发 58 次，失败数均为零，`result.json` 的 violations 为空。
这覆盖了正常到点后的重复执行。

另用全新仿真补测“终点附近无进展、但未进入容差”：先正常到达 `(4.5, 8.5)`，
再以地图坐标中当前位置前方 0.20 m 作为目标，通过 Nav2 原生 `/speed_limit`
将平移速度限制为 `0.0001 m/s`，不暂停物理或仿真时钟。车辆几乎静止，最终
地图误差仍为 0.19625 m、真值误差 0.19200 m，未满足 0.1 m 到点条件。
观察到 8 次 `Failed to make progress`、8 次 FollowPath 失败，零次规划失败
通知；NavigateToPose 在 84.04 s 墙钟时间后主动返回 ABORTED，命令归零。
这是受控速度约束下的无进展验证，并非额外声称复现了一个新的自然物理碰撞。
脚本在 finally 中发布零值解除限速。
解除限速后再导航到 `(4.6, 7.3)` 并重复下发 10 次，全部成功，最大真值误差
0.02498 m，额外 `terminal_replacement/result.json` 的 violations 为空。

## 观测方式与边界

复用 `check_sim_navigation_loop.py` 和 `check_sim_goal_accuracy.py` 的同步
Gazebo 真值/TF 测量，并增加本地测试观察脚本，记录：

- `/opt_path` 的 command_flag 变化，特别是 `PLANNING_FAILED_COMMAND=138`。
- `/follow_path/_action/status` 和 `/navigate_to_pose/_action/status` 的目标 UUID 与状态。
- `/rosout` 中进度失败、规划器放弃和 Action 失败信息。

因此可以区分系统 ABORTED 与测量脚本主动 CANCELED。失败通知不是仅凭红色
可视化推断：本轮已实际观察到通知、控制器 `NoValidControl` 对应日志和 Action
失败。行为树 `number_of_retries=3` 是初次执行加三次重试；加上内层一次重试，
本次无路场景观察到了 8 次 FollowPath 失败，而非只执行三轮 FollowPath。
受控无进展测试进一步实测约 84 s：粗略基数为 `4 × 2 × 10 + 3 × 1 = 83 s`，
另有 Action、清图和调度开销。10 s 使用节点时钟；有效位移又会重置进度计时，
因此既不能宣称固定 63 s，也不能仅凭这些参数保证任意场景在 90 s 墙钟时间内结束。

原先试探点 `(6.0, 7.3)` 在本轮地图状态下实际可达（20.42 s 成功），不能沿用
此前代价 253 的观测将其固定称为“不可达”。故重新从静态 PGM 选择障碍内部点
`(19.975, 3.975)`，该点距自由区约 1.1 m，超过 0.3 m 搜索容差。
前一试探的预期中止断言失败是测试前提不成立，不是导航失败。

## 构建与参数

先保持接口包已有构建模式，再重建所有直接和传递依赖：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select ros_interfaces \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DAMENT_CMAKE_SYMLINK_INSTALL=OFF
source install/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build \
    --symlink-install --executor sequential --packages-above ros_interfaces \
    --packages-skip ros_interfaces --cmake-args -DCMAKE_BUILD_TYPE=Release
```

依赖重建 8 包全部完成：`bt_manager`、`dbscan_cluster`、`fake_vel_transform`、
`minco_controller`、`minco_planner`、`ros2_libxr`、`navi2`、`sentry_simulation`。
本次启动日志未出现 type-hash 或 incompatible-type 警告。

已从运行中节点导出参数确认：进度检测 10.0 s，`max_attempts=3`，MINCO
`collision_dist=0.321`、`safe_dist=0.55`，加载的是已修改的导航行为树。
planner 30/30、去重 9/9、controller_freshness 15/15 重新运行通过；
`sentry_simulation` 三个测试入口通过。

## 复现与产物

启动仿真并持续保存标准输出，然后运行：

```bash
ROS_DOMAIN_ID=93 python3 scripts/check_sim_navigation_loop.py \
    log/planning_failure_exit_sim/long_route --cycles 8 --goal-timeout 90 \
    --launch-log log/planning_failure_exit_sim/launch.log
ROS_DOMAIN_ID=93 python3 scripts/check_sim_goal_accuracy.py \
    log/planning_failure_exit_sim/resend \
    --goals '[[4.5,8.5],[4.6,7.3]]' --resend-seconds 12
```

需先 source ROS 和工作空间；两条测量命令依次执行。循环遇首个失败即停止，
并非预期一定完成八轮。无路目标由本地 `check_unreachable.py` 发起，观察上限
设为 120 s，另断言系统应在 90 s 内自然中止，避免由测试取消伪造“主动退出”。

产物在 `log/planning_failure_exit_sim/`，生成日志不纳入版本管理：
`launch.log`、`exit_events.jsonl`、各案例结果和同步样本、运行参数 YAML、
接口/依赖重建日志以及单元测试日志。[统计汇总](../../log/planning_failure_exit_sim/analysis.json)
和[路线与定位误差图](../../log/planning_failure_exit_sim/routes_and_errors.png)包含两组失败路线。
`terminal_exit_events.jsonl`、`terminal_no_progress/result.json` 和
`check_terminal_no_progress.py` 保存独立受控测试的状态证据、断言和复现入口。

两个仿真实例与观察进程均在测试结束后关闭。脚本语法检查、主仓库及仿真子模块
的 `git diff --check` 通过；本轮没有提交。

`bt_manager` 中止后每 tick 重发且没有退避的行为不在本次修改范围内。
上述目标由测量客户端发起，尚未覆盖 bt_manager 连续重提中止目标的系统级策略。

后续已补齐真实 `bt_manager` 的共享退避、Action 回归测试和仿真验收，见
[导航中止退避验收](20261024_bt_manager_abort_backoff.md)。
