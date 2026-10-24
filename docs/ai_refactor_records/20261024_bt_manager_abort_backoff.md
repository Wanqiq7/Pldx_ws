# bt_manager 导航中止退避与闭环验收

日期：2026-10-24。

## 原因与范围

承接[规划失败出口](20261024_planning_failure_exit.md)和
[失败出口仿真验收](20261024_planning_failure_exit_sim.md)：MINCO 的异步失败现已能使
NavigateToPose 自然中止，但决策节点会在下一 tick 再次下发同一目标，立即启动下一轮导航。
仅给每个 action 加独立计时器也不够：响应式树可能切换到另一个 action 实例。

本次完成 `nav_action.hpp` 的共享退避验收，修复接手时未能编译/运行的测试夹具，
补充可重复运行的真实决策节点仿真脚本。没有改比赛树、地图、碰撞半径、传感器外参或通信协议。

## 行为

- blackboard `navigation_abort_retry` 存放最近中止的 XY 目标、连续中止计数及下次发送时间。
  目标比较沿用 1 cm 阈值，状态在共享同一 blackboard 的 action 实例之间生效。
- 连续同目标 `ABORTED` 默认退避为 `2、4、8、16、16…` 秒；计数饱和，避免指数溢出。
- `on_tick()` 仅在同目标退避未到期时设置 `should_send_goal_=false`，由 Nav2 基类返回
  FAILURE 而不发送 ROS goal。没有改写基类 tick 或引入新的后台线程。
- 换目标立即允许发送；新目标中止时重新计数。成功或活动导航取消后清除失败历史。
  halt 一个已失败或尚未发送的节点不会误清除共享退避。
- 两个 XML 输入端口 `abort_backoff_initial_sec` / `abort_backoff_max_sec` 默认 2 / 16。
  非有限或非正值回退到默认值，max 不小于 initial。
- 等待使用节点 ROS 时钟，所以仿真中的秒指 `/clock` 秒；既有巡逻超时仍使用 steady clock。
- 补齐 `rclcpp_action` 的直接构建/包依赖以及 `ament_cmake_gtest` 测试依赖。

这是对最近一条同目标失败链的限流，不是持久化不可达目标黑名单。退避时返回 FAILURE，
原 XML 仍可选择低优先级分支或其他目标。完整比赛策略的目标选择不由此处重写。

## 回归测试

`src/decision/bt_manager/test/test_nav_action.cpp` 使用真实 NavigateToPose Action 服务端，
以收到的 goal 数量作为判据，不把单纯出现“等待”日志当作通过。

| 场景 | 判据 |
| --- | --- |
| 跨两个 action 实例 | 第二个实例不能在窗口内重发；到期重试再中止后计数为 2 |
| 默认指数延迟封顶 | 服务端逐次 ABORTED，记录 2/4/8/16/16 秒，计数连续增长 |
| 真实计时窗口 | 0.3 秒退避中持续 tick，窗口内 goal 数保持不变，到期后增加一个 |
| 换目标 | 5 秒旧窗口未到期即发送新坐标；新目标首次中止计数为 1 |
| 参数保护 | 0、负数、NaN、Infinity 回退到安全默认值 |
| 成功与取消（两个参数化用例） | 先制造失败历史，再成功/主动取消，确认状态移除且同目标立即可发 |

指数序列测试只把已记录的截止时间移到过去来避免空等，保留目标和失败计数；真实等待另有独立用例。
接手时夹具存在枚举用作布尔断言、ROS 初始化前构造 executor、缺失 current_mode、
错误的 Fallback 短路和对同一句柄重复结束等问题，已收敛为上述七项测试。
取消服务端通过 timer 在进入 CANCELING 后结束句柄，避免非法状态迁移。

还做了变异验证：临时关闭生产代码的退避判断，真实计时测试立即因服务端收到
第二个 goal（预期仍为一个）失败。随后恢复生产实现、重新编译并重新通过测试。
证据在 `log/bt_manager_abort_backoff/mutation.log`。

## 构建与跨包验证

使用 `install/`，与上一轮规划失败出口验收保持一致：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build \
    --symlink-install --executor sequential \
    --packages-select bt_manager minco_controller minco_planner navi2 \
                      sentry_simulation fake_vel_transform ros2_libxr dbscan_cluster \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
ROS_DOMAIN_ID=221 GTEST_FILTER='-*Benchmark*:*SafetyCheckP95P99' \
    colcon test --executor sequential \
    --packages-select bt_manager minco_planner minco_controller navi2 \
                      fake_vel_transform ros2_libxr sentry_simulation \
    --event-handlers console_cohesion+
colcon test-result --test-result-base build/bt_manager --verbose
```

八包重建完成。新 bt_manager 七项测试、planner 30 + 9 项、controller 15 + 13 + 7 + 5 项、
Nav2 goal updater、TF 脚本以及 simulation 的三个 CTest 入口均通过。
性能基准被显式过滤，本次不把仿真负载下的功能回归当作 P95/P99 性能验收。
`ros2_libxr` 在当前 build 配置下没有注册测试，不能将“包测试命令完成”当作 wire test 已运行。

## 仿真方式

```bash
# 终端一：隔离的 Gazebo + Point-LIO + GICP + Nav2，使用真实车模型
ROS_DOMAIN_ID=95 GZ_PARTITION=bt_backoff_20261024 NAV_INSTALL_DIR="$PWD/install" \
    ./start_sim.sh world:=rmuc_2025 robot_model:=real headless:=true rviz:=false

# 终端二：source 同一份 ROS 和 install 环境，运行真实 bt_manager
ROS_DOMAIN_ID=95 python3 scripts/check_sim_bt_backoff.py \
    log/bt_manager_abort_backoff/actual_bt_final
```

脚本复用 `Accuracy` 的同步真值/TF 测量，启动已安装的 `bt_manager_node` 及原始比赛 XML，
不是另写一个 Python 重试器。通过 `/sentry/game_info` 的 A 键人工点输入，先从 map 反变换到
minimap。仿真 launch 没有 minimap TF 时，只补这一条测试用静态变换，采用实车 launch
蓝方的 `[28.7, 15, yaw=3.14]`，不重复发布 body/base_link 等关键 TF。

比赛剩余时间输入为 300 秒；实际走到现有 XML 的
`NavigateFallbackManualOverrideGoal` 分支（外层 ForceSuccess），策略树未修改。
观察 UUID/状态、`/opt_path` 失败通知、命令和 Gazebo 真值速度。
不可达阶段不调用 cancel 来伪造失败。脚本结束时关闭自己启动的决策进程并取消最后一次保持目标。

## 仿真结果

首轮完整验收 `actual_bt/result.json`：

- 障碍内部 `(19.975, 3.975)` 连续两次自然 `ABORTED=6`，共观察到 16 次规划失败通知。
- 首次中止到下一 UUID 被接受的间隔为 **2.076 秒（仿真时间）**；中间无新 UUID。
- 停车时 `/sim/cmd_vel` 与 Gazebo 真值速度均为零。
- 第二次失败记录 4 秒退避；在其中 1 秒后换成 `(4.5, 8.5)`，**0.013 秒**即接受新目标。
- 新目标成功，到点后继续重发成功；稳定真值误差 **0.009794 m**。

最终复跑 `actual_bt_final/result.json` 同样通过：

| 指标 | 实测 |
| --- | --- |
| 两次自然中止耗时 | 24.070 / 23.843 秒（仿真时间） |
| 首次中止至重试接受 | 2.084 秒，无窗口内新 UUID |
| 第二次 4 秒退避中的零重提观察 | 连续 3.5 秒，没有新 UUID |
| 在上述窗口末换目标后的接受延迟 | 0.002 秒，早于旧目标 4 秒截止时间 |
| 替代目标稳定真值误差 | 0.045535 m |
| 到点后额外成功重发 | 10 次；首个替代目标也成功 |
| 规划失败通知 | 16 次 |

复跑起点沿用首轮到点位置，因此替代目标本来已接近车辆；首次验收则覆盖了从原出生点到替代目标的实际运动。
两轮都使用真实决策节点。首轮原始 JSON 的 `successful_resends=10` 包含了第一次替代导航；
新增脚本已修正计数，最终结果的 10 次仅指到点后的额外重发。

## 产物与边界

构建、单元测试、变异验证、原始仿真日志、每次 Action 状态和真值采样均保存在
`log/bt_manager_abort_backoff/`，不纳入版本管理。新增脚本 `scripts/check_sim_bt_backoff.py`
和本记录保留复现方式。

本次只验收规划失败出口与决策退避链路。此前长路线 SMAC 无路/物理卡点的独立问题没有被
这组短距离到点结果证明解决，详见上一轮路线记录。没有启动实车 `start.bash`，没有提交或推送。

结束时只关闭本次启动的 decision/Gazebo/Nav2 进程，并在关闭仿真前额外确认零命令和真值停车。
SIGINT 关闭 decision 后，Nav2 基类会在 ROS context 关闭期间打印取消等待错误；
探针随后通过 Action cancel 服务清理最后一个保持目标，独立停车检查结果保存在
`cleanup_stop/result.json`，不把清理阶段的取消计为自然中止。
