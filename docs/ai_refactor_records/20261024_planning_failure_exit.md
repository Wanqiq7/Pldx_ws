# 规划失败的明确出口

## 原因

1. `createPlan()` 只提交目标，搜索在 FSM 中异步进行。规划失败从未传到 Nav2：控制器在参考过期后只输出零速，不会让 FollowPath 失败，行为树一直处于"运行中"。
2. 内部恢复失败后总是回到 `GENERATE_TRAJ`，`EMER_STOP` 分支被注释，没有失败出口。另有两个 bug：
   - 离开 `RECOVERING` 时 `changeState()` 调用 `RecoverServer::reset()`，失败计数被清零，"失败 → 脱困 → 失败"循环没有上限。
   - `handleReplanFailure()` 在 `onReplanFailure()` 里已经 `startRecovery()`，无脱困方向时返回 `ENTER_EMER_STOP` 却没有结束恢复。`recovery_active_` 一直为 true，之后的决策恒为 `NONE`，机器人原地停住直到换目标。
3. `progress_checker.movement_time_allowance` 为 1000 s，外层兜底实际不起作用。

## 修改

- `recovery_behaivor.cpp`：无脱困方向时先 `finishRecovery(false, now)` 再返回 `ENTER_EMER_STOP`。新增 `recovery_server.max_attempts`（默认 3）。
- `minco_fsm`：
  - 新增 `FAILED` 状态和按目标计数的 `recovery_attempts_`，只在新目标、取消或 `COMMITTED` 时清零。
  - 每次 `DO_ESCAPE` 或 `ENTER_EMER_STOP` 计一次，超过上限进入 `FAILED`，撤销参考并发布失败通知。
  - `EMER_STOP` 仍保持注释。
- `MincoPlanner`：
  - `invalidateTrajectory(true)` 在 `/opt_path` 发布 `PLANNING_FAILED_COMMAND`。
  - FSM 处于 `FAILED` 时，`createPlan()` 不对相同目标去重，而是重新提交。
- `ros_interfaces/MpcPositionCommand`：新增常量 `PLANNING_FAILED_COMMAND = 138`，同时改了 `.idl`（实际构建来源）和 `.msg`。
- `MincoMpcController`：收到该通知后锁存，发布零速并抛 `nav2_core::NoValidControl`。`reset()` 或新的 `setPlan()` 清除锁存。
- `sentry1.yaml`：`movement_time_allowance` 从 1000 改为 10 s，新增 `recovery_server.max_attempts: 3`。
- 两个导航 BT 的外层 `NavigateRecovery` 重试次数从 6 改为 3。注意这表示首次执行后再重试 3 次；内层 FollowPath 另有 1 次重试，持续失败时可执行 8 次 FollowPath。不能用 `3 × (2 × 10 + 1) = 63 s` 作为硬上限；进度检测还会随有效位移重新计时，总时长需由仿真测量。

## 验证

```bash
colcon build --packages-select ros_interfaces --cmake-args -DCMAKE_BUILD_TYPE=Release -DAMENT_CMAKE_SYMLINK_INSTALL=OFF
colcon build --symlink-install --packages-select minco_planner minco_controller --cmake-args -DCMAKE_BUILD_TYPE=Release
ROS_DOMAIN_ID=218 build/minco_planner/test_planning_budget --gtest_filter='-PlanningTimingBenchmark.*:*SafetyCheckP95P99'
ROS_DOMAIN_ID=218 build/minco_planner/test_goal_deduplication
ROS_DOMAIN_ID=218 build/minco_controller/controller_freshness_test
```

- planner 30/30、去重 9/9、controller_freshness 15/15 通过，dynamic_mpc_solver、disturbance、wheel_odometry 也全部通过。
- 新增用例：
  - `RepeatedRecoveryWithoutCommitFailsGoal`
  - `FailedGoalIsRetriedWhenResubmitted`
  - `RecoverServerTest.StopWithoutEscapeDirectionDoesNotDisableLaterRecovery`
  - `PlanningFailureFailsFollowPathUntilNextExecution`
- 去掉 `finishRecovery` 修复后，`RecoverServerTest` 会失败，说明该用例确实能抓到原来的 bug。

## 仿真验收

已重建全部消息依赖并完成 Gazebo + RViz 闭环验证，详见
[规划失败出口仿真验收](20261024_planning_failure_exit_sim.md)。

- 原物理卡点再次复现，10 s 进度失败使 FollowPath 失败，行为树重试后到点。
- 两组长路线仍因 SMAC 无路失败，但系统主动中止，不再等测量脚本取消。
- 静态障碍内部目标 23.54 s 主动中止；正常终点共 115 次重发全部成功。
- 受控终点外无进展：极低速度限制下始终未进入 0.1 m 容差，8 次 FollowPath 进度失败后 84.04 s 主动中止。此结果也纠正了原先 63 s 的估算。
- `bt_manager` 在 NavigateToPose 中止后每个 tick 重发同一目标，会立即重新开始，没有退避。
- `ros_interfaces` 及其 8 个直接/传递依赖包已重建，启动日志未出现类型哈希不一致警告。

后续已补齐真实 `bt_manager` 的共享退避、Action 回归测试和仿真验收，见
[导航中止退避验收](20261024_bt_manager_abort_backoff.md)。
