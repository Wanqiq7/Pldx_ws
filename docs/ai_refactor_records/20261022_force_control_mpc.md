# 力控（动力学）MPC，阶段 1

日期：2026-10-22。修改范围为 `minco_controller` 与 `sentry1.yaml` 的 `FollowPath` 参数。参考《中科大哨兵 2025 技术报告》5.5.5 节。

## 背景

原控制器为 SE(2) 速度层 MPC：状态 `[x, y, yaw]`，输入 `[vx, vy, wz]`，模型 `A=I, B=dt·I`。加减速只能通过相邻速度差分约束间接表达，无法使用 MINCO 的加速度作力前馈，也无法表达摩擦约束。报告的方案是以 `[Fx, Fy, Mz]` 为输入的双积分模型，下发的仍是预测轨迹前瞻处的速度，力前馈留给下位机。阶段 1 只做 ROS 侧，不改协议与固件。

## 实现

新增 `DynamicMpcSolver`（`dynamic_mpc_solver.{hpp,cpp}`），由 `FollowPath.use_dynamic_model` 选择，默认 `false`，运动学求解器保持原样。

- 模型：状态 `[px, py, yaw, vx, vy, omega]`，输入为质量归一化的 `[Fx/m, Fy/m, Mz/Iz]`，零阶保持精确离散 `p += dt·v + dt²/2·a`。报告为欧拉离散，位置要滞后一步才感受到力。
- 分解：平移与 yaw 只通过代价耦合，拆为 2N 维平移 QP 与 N 维 yaw QP。yaw 的 Hessian 与约束矩阵恒定，只热启动向量；平移 Hessian 随参考切向变化，沿用 `SQProblem` 双缓冲热启动。Hessian 用后缀矩求和以 O(N²) 组装。
- 代价：位置与速度误差按参考速度切向分解为沿轨迹与横向权重（静止参考时退回参考 yaw）。输入相对 `a_ref`（相邻参考速度差商，即零阶保持下的精确平均加速度）的偏差加权，即 `m·a_ref` 力前馈；另加相邻输入差分权重。
- 约束：平移合力为摩擦圆 `min(mu·m·g, max_force)` 的内接八边形（盒约束加两组对角面），不超过摩擦圆；`|Mz| ≤ max_torque`；速度为状态约束。初速已越界时按可恢复量逐步放宽速度边界，保证 QP 可行。
- 输出：预测轨迹 `t0 + command_lookahead` 处的速度（步内线性插值），并给出首步力/力矩 `wrench` 供阶段 3 使用。
- 控制器：动力学模式下 `curr.omega` 使用 odom 角速度（运动学模式仍为 0）；延迟补偿用上一周期下发的加速度外推；参考从延迟补偿后位置的投影点开始，不再叠加 `control_delay_compensation`。死区只在整个参考时域都静止时生效，否则从静止起步的小命令会被截断。求解失败、命令被新鲜度检查置零或到点时清除已记录的加速度。
- 二次参考（`dynamic.secondary_reference`，默认关闭）：按首次预测速度与参考速度夹角余弦 `α` 缩短时间步重新取参考点后再求解一次；第二次失败时保留首次结果。
- 重构：`buildReferenceFromOptPath` 拆为 `prepareReference`（投影）与 `sampleReference`（按时间插值），运动学路径的取点与原实现一致。

## 验证

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select minco_controller --cmake-args -DBUILD_TESTING=ON
ROS_DOMAIN_ID=232 colcon test --packages-select minco_controller --event-handlers console_cohesion+
colcon test-result --test-result-base build/minco_controller --verbose
build/minco_controller/dynamic_mpc_solver_test --gtest_filter='*Benchmark' --gtest_also_run_disabled_tests
```

35 项测试通过（2 项为默认禁用的基准）。`dynamic_mpc_solver_test` 9 项：与独立的完整 6 状态稠密 QP 逐次冷启动对比（含活跃的速度、盒与对角约束，首步容差 1e-5），恒加速度参考下力前馈零滞后，静止保持，起步，初速越界可行性，前瞻插值，commit 语义，非法输入与配置，迭代耗尽后恢复。`controller_freshness_test` 参数化为运动学/动力学两组，8 项新鲜度与取消语义在两种模型下均通过。

基准（i7-14700HX，Release，热启动，2000 样本）：

| 求解器 | 步数 | p50 / p95 / p99（μs） |
| --- | --- | --- |
| 动力学 | 20 | 70 / 95 / 111 |
| 动力学 | 40 | 394 / 532 / 604 |
| 运动学（对照） | 20 | 225 / 284 / 308 |

## 仿真

rmuc_2025，六目标往返路线（含上下台），每轮单独启动、单独 domain/partition，结束后确认无残留 Gazebo 进程：

```bash
ROS_DOMAIN_ID=<id> GZ_PARTITION=<name> ./start_sim.sh headless:=true rviz:=false \
    params_file:=<sentry1.yaml 副本，use_dynamic_model: true>
ROS_DOMAIN_ID=<id> python3 scripts/check_sim_goal_accuracy.py <out> \
    --goals '[[4.5,8.5],[4.5,7.3],[5.0,5.5],[4.5,6.5],[4.5,14.0],[4.5,6.5]]'
```

| 配置 | 路线总时长 | 到达 0.1 m 后最大误差 | 停稳真值误差最大值 | QP 失败 |
| --- | ---: | ---: | ---: | ---: |
| 运动学（基线） | 24.8 s | 0.094 m | 0.043 m | 0 |
| 动力学，`command_lookahead=0.05` | 32.5 s | 0.261 m | 0.101 m | 0 |
| 动力学，`command_lookahead=0.2` | 19.6 s | 0.094 m | 0.075 m | 0 |
| 动力学，0.2 + 二次参考 | 20.4 s | 0.099 m | 0.070 m | 0 |

四轮均为全部 status=4、`PASS accuracy`。`0.05` 的问题定位：仿真底盘速度环表现为约 0.2 s 的一阶滞后（加速段命令与真值速度差约 0.3 m/s、实际加速度约 1.5 m/s²），前瞻过短时规划的 2.4 m/s² 减速只兑现约 1.4 m/s²，长距离到点超调 0.78 m 后回头。这与报告 5.5.5.6 节“前瞻速度过小、下位机推力不足”是同一机理。前瞻设为 0.2 s 后超调消失，路线用时比运动学基线少约 21%。

## 限制与影响

- `sentry1.yaml` 中 `use_dynamic_model` 默认关闭，实机行为不变。`start_sim.sh` 默认生成 `log/sim_params/sentry1_dynamic.yaml` 在仿真中启用，`SIM_MPC_MODEL=kinematic` 切回；显式 `params_file:=` 时不做改写。实机启用前需标定 `mass`、`inertia_z`、`friction_coeff`、`max_force`，并实测底盘速度环等效时间常数来设定 `command_lookahead`；仿真的 0.2 s 不能直接沿用。
- 停稳误差比运动学基线大（0.075 m 对 0.043 m），仍在 0.1 m 到点容差内；末端死区与停车检查的配合未专门调优。
- 测试路线以直线为主，二次参考的横向收益没有被区分出来，因此默认关闭。
- 动力学模式下 yaw 使用真实角速度，角速度命令会比运动学模式（`omega` 恒为 0）更活跃；小陀螺模式下不受影响。
- 未改 `/cmd_vel_mpc` 类型与坐标约定，`wrench` 尚未下发。
- 顺带发现：运动学求解器 `mpc_solver.cpp` 旋转 Q 的非对角项符号相反，45° 航向时沿轨迹/横向权重互换（0°/90° 正确），其测试使用同一公式。本次未修改。

## 后续修复：控制器内到点判断的坐标系

`computeVelocityCommands()` 的到点判断把 `setPlan()` 路径终点（MincoPlanner 输出 `map`）与机器人位姿（局部代价地图 `camera_init`）直接比较。controller_server 不转换交给插件的路径，仿真中两系相差约 (4.24, 7.09) m，车在目标 0.1 m 内时该判断算出约 8.3 m，从不触发；到点一直由 controller_server 自己（会做变换）的检查完成。

修复：`goalInPoseFrame()` 用 TF 把终点转到位姿所在坐标系后再调用 GoalChecker；终点无 frame 时取路径 header，变换不可用时跳过本次判断并节流告警。临时日志确认仿真中 `map` 终点 (4.50, 8.50) 转为 `camera_init` (0.32, 1.39)，到点时距离 0.016 m。该分支仍很少先于 controller_server 触发：两者同一周期判断，后者稍晚取位姿，通常先满足停车速度条件。

`controller_freshness_test` 新增两项（两种模型各一）：按仿真偏移构造 `map → camera_init`，站在终点原始坐标上判断距离 > 8 m 不到点，站在转换后坐标上判断到点并清空参考；无 TF 时不调用 GoalChecker。58 项测试通过。六目标路线运动学 1 轮、动力学 3 轮；动力学首轮在第三个目标超时，日志显示全局代价地图起终点均为 unknown、控制循环报 `inf Hz`、90 s 内仅 5 个真值样本，属于仿真时钟停滞，复跑两轮均通过（停稳误差最大 0.075/0.076 m）。
