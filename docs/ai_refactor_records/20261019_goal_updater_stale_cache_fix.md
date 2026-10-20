# GoalUpdater 旧目标缓存重复告警修复

## 原因与实现

Jazzy 原生 GoalUpdater 每次 tick 将缓存的 topic 目标时间戳与 action 目标时间戳比较；当新 action 目标使旧 topic 目标过期后，旧缓存一直保留，因此每次 tick 都重复输出警告。另一个问题是 topic 与 action 可能使用不同 ROS 时钟，时间戳大小不能可靠表示消息是否属于当前导航目标。

在 navi2 包内增加 `minco_goal_updater_bt_node` 动态库，注册 `MincoGoalUpdater`，保持原节点的输入/输出端口、`goal_updater_topic` 参数（默认 `goal_update`）、订阅 QoS 和 Jazzy executor warm-up。检测到 action 目标切换时清空上一轮 topic 缓存；当前 action 期间收到的非零时间戳更新按到达顺序应用，不跨时钟域比较时间戳。有效 topic 目标保留供后续 tick 使用。

单目标 BT 使用该节点，原生 `GoalUpdatedController` 和恢复分支保持原设计；两个 Nav2 参数文件增加插件加载声明。实现保存在仓库中，通过 colcon 构建安装。

## 回归测试

新增 `navi2/test/test_goal_updater.cpp`，通过实际 DDS 发布和真实 BT 插件验证：有效 topic 更新跨 tick 保留；新 action 清除上轮缓存；当前 action 收到时间戳较旧的 topic 更新仍能生效。

同一测试切换到已安装原生节点时，单份旧缓存经过 21 次 tick 产生 21 条警告，断言失败；修复插件通过测试。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select navi2 --executor sequential
ROS_DOMAIN_ID=95 colcon test --packages-select navi2 --event-handlers console_cohesion+
colcon test-result --test-result-base build/navi2 --verbose
```

构建、回归测试通过，无测试失败。

## 完整仿真

以默认 rmuc_2025 无界面启动，独立 `ROS_DOMAIN_ID=94`、`GZ_PARTITION=minco_goal_updater_fix_20261019`。Nav2 lifecycle 全部 active，运行上一轮真实导航验收脚本。

- 固定目标 (4.5, 6.0)，限速 0.2 m/s 观察 6 秒：`/plan` 仅 1 次，MINCO 生成 7 个不同轨迹编号（2 至 8）。
- topic 动态目标更新至 (5.0, 5.5)：仅新增 1 次 `/plan`，后续 3 秒不重复提交。
- 更新目标导航 status=4；Gazebo 真值 (4.96154, 5.42673)，目标误差 0.08275 m，停车速度和底盘命令均为零。
- 向 (4.5, 6.5) 导航后取消 status=5；相同目标重启 status=4，仅新增 1 次规划。
- 新 action 取代旧 topic 缓存时，日志出现且仅出现 1 次 `Ignoring goal update with timestamp ...`，原生重复警告为 0 次。没有全局屏蔽 WARN。

运行证据位于本机临时目录 `/tmp/minco_goal_updater_check/`：`simulation.log`、`check_sim.py`、`check_sim.log`、`results.json`、`package_build.log`。临时产物不会随仓库提交持久保存。验收后关闭本次仿真。

本次没有主动注入障碍或规划/控制故障；验证范围是上述场景及目标序列。

## 后续回归修复（2026-10-20）

GoalUpdater 改为以 action 目标切换作为缓存边界，当前导航期间不比较不同来源时钟的时间戳。针对该行为的回归测试通过。仿真中动态目标仅新增一次规划，最终到达更新目标，Nav2 status=4；取消/重启场景仍通过。

同次生命周期复测发现 `ProbMap::initProbMap()` 的函数内 `static` 初始化标记跨实例保留，planner cleanup 后重新创建 ROGMap 会错误抛出 `ProbMap can only init once`。移除此进程级标记后，planner deactivate 触发 Nav2 lifecycle manager 自动恢复，重新初始化成功且 `/planner_server` 回到 active。仿真期间未再出现该异常。

验证命令：`colcon build --symlink-install --packages-up-to minco_planner --executor sequential`；`colcon test --packages-select minco_planner navi2 --event-handlers console_cohesion+`。仿真结果保存在本机 `/tmp/minco_bt_sim_acceptance/results.json`，使用 `ROS_DOMAIN_ID=221`，验收后关闭仿真。
