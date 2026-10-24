# 力控 MPC 阶段 3：力前馈、仿真验收与删除运动学 MPC

日期：2026-10-23。前序记录 `20261022_force_control_mpc.md`、`20261023_force_mpc_stage2_disturbance_wheel_odometry.md`。

## 阶段 3：力前馈

用户确认的约定：速度与力前馈用同一坐标系（与 `/cmd_vel_mpc` 相同，桥接默认原样直通）；NAVIGATION 模式下固件忽略主机 wz 是有意设计；视觉与导航是同一 USB 上的两个独立 CDC。

- **消息**：`ros_interfaces/msg/ChassisCommand`（`header`、`twist`、`wrench_ff`）。`/cmd_vel_mpc` 保留给速度模式。
- **控制器**（`dynamic.force_ff.*`，默认关闭）：同一控制周期发布 `/chassis_cmd_mpc`（sensor-data QoS），`F_ff = gain·m·a_plan(t0 + lookahead)`，按 `max_force` 限幅，随 speed limit 缩放。速度提前量随 `gain` 从 `command_lookahead` 线性缩到 `lookahead`，避免底盘速度 PID 与前馈重复提供同一加速度。死区清零、输入过期、取消或求解失败时前馈同时清零。yaw 力矩默认关闭。
- **桥接**（`ros2_libxr`）：`command_arbiter.hpp` 在 `ChassisCommand` 新鲜时优先转发其速度与 `fx/fy/fw`，否则转发 `/cmd_vel_mpc` 且力为零，都不新鲜时发零。`rotate_world_to_body`（默认 false）可用 LIO yaw 外推把速度和力一起旋到车体系。
- **固件**（MC02，未提交、未烧录）：`CMD::NavigationFeedforward`；`HostDataLink` 与速度同源、同新鲜度发布 `chassis_ff`，非法或超过 1000 的值丢弃；`DualBoard` 云台侧每 10 ms 在 `0x312 + 0x1a = 0x32C` 发送；`Omni` 在 NAVIGATION/ROTOR/ROTOR_VARIABLE/FOLLOW 且来源为导航、30 ms 内新鲜时，把前馈加到速度 PID 输出的力上，再走力分配与功率控制。限幅 `ff_max_force_n: 80`、`ff_max_torque_nm: 0`（`sentry_chassis.yaml`）。新增 `DualBoard/tests/chassis_feedforward_frame_test.cpp`。两块板编译通过。
- **仿真**：`cmd_vel_adapter.py` 用与速度相同的 world→body 旋转转发力到 `/sim/cmd_force`；`MecanumDrive2` 把它加在速度伺服输出之后、执行器限幅之前，与固件位置一致，100 ms 超时。`start_sim.sh` 增加 `SIM_FORCE_FF=<gain>`、`SIM_FORCE_FF_LOOKAHEAD`（默认 0.15 s），仿真参数副本使用仿真底盘质量 38.15 kg、`max_force` 91.56 N（保持 2.4 m/s² 上限，速度模式不变）。

## 删除运动学 MPC

按用户要求删除速度层 SE(2) 运动学 MPC，只保留力控链路：

- 删除 `mpc_solver.{hpp,cpp}`、`mpc_solver_test.cpp`、`MPCConfig`、`MINCO_DEBUG` 选项、`buildReferenceFromOptPath()` 与 `computeVelocityCommands()` 中的运动学分支。`use_dynamic_model` 参数删除，控制器始终使用 `DynamicMpcSolver`。
- 顶层 `vx/vy/omega_min/max` 直接读入力控配置；`/opt_path` 采样周期改为常量 `kPlannerDt = 0.05`（原 `MPCConfig::planner_freq` 默认 20 Hz，从未作为参数暴露）。
- `sentry1.yaml` 删除 `dt`、`lookahead_time`、`Q`、`R`、`use_acc_constraints`、`ax/ay/alpha_*`、`use_dynamic_model`。`start_sim.sh` 删除 `SIM_MPC_MODEL`，副本改名为 `log/sim_params/sentry1_sim.yaml`。
- `controller_freshness_test` 由两种模型参数化改为单一 fixture。`minco_controller` 测试 42 项，0 失败（1 项为默认禁用的基准）。
- **实车影响**：此前实车默认 `use_dynamic_model: false`，即运动学 MPC。删除后实车直接使用力控 MPC，而 `mass`、`inertia_z`、`friction_coeff`、`max_force` 仍是占位值，`command_lookahead: 0.2` 来自仿真底盘。上车前必须标定。

## 仿真验收

rmuc_2025，无头，每轮独立 domain/partition，结束后确认无残留 Gazebo 进程。脚本在 `log/force_mpc_accept/`（`accept_run.sh`、`batch.sh`、`batch2.sh`、`accept_summary.py`、`force_probe.py`）。参数副本由 `start_sim.sh` 生成（`ros2 launch` 用桩替换），再打开详细 CSV。

```bash
SIM_WHEEL_ODOM=1 SIM_SLOPE_COMP=1 SIM_FORCE_FF=1.0 ./log/force_mpc_accept/accept_run.sh <label> <domain> route6|hold|rolling|complex
python3 log/force_mpc_accept/accept_summary.py <label>...
```

场景：`route6` 六目标往返（含上下台）；`hold` 坡中段目标 [4.7, 10.9]，每次到点后按 `bt_manager` 方式重发 15 s；`rolling` 滚动衔接五目标并运行 `check_sim_rolling_continuation.py`；`complex` 长距离往返 [24.0,10.0]↔[4.234,7.3]；`fault` 六目标中途 12 s 杀掉轮速模拟器。

配置：D = 力控；DS = 力控 + 轮速 + 坡度补偿；F05/F1 = 力前馈增益 0.5/1.0（前瞻 0.15 s）；F1L02 = 增益 1.0、前瞻 0.02 s；FA = 轮速 + 坡度补偿 + 力前馈 1.0。

### 六目标路线（删除前，每配置 2 轮，全部 PASS，QP 失败 0）

| 配置 | 路线总时长 | 停稳误差最大值 | 前馈力 p50 / max |
|---|---:|---:|---:|
| D | 21.3 / 22.2 s | 0.067 / 0.066 m | — |
| DS | 22.0 / 22.2 s | 0.079 / 0.082 m | — |
| F05 | 21.5 / 21.8 s | 0.079 / 0.062 m | 23 / 46 N |
| F1 | 21.6 / 21.0 s | 0.080 / 0.051 m | 47 / 92 N |
| F1L02 | 32.8 / 32.5 s | 0.100 / 0.086 m | 74–81 / 92 N |
| FA | 22.1 / 21.9 s | 0.045 / 0.047 m | 33 / 92 N |
| 运动学（删除前对照） | 24.2 / 26.3 s | 0.046 / 0.044 m | — |

- 前瞻 0.02 s 时路线用时多约 50%，默认 0.15 s。前馈 92 N 即 `m·a_max`，加速段处于上限。
- `/chassis_cmd_mpc` 间隔 p99 为 12–13 ms（100 Hz）；`hold` 场景中 p99 约 0.23 s，是到点后 Nav2 不再调用控制器的空档，符合预期。
- 轮速故障：12 s 时杀掉模拟器，控制器 5 次节流告警后退回 LIO，路线 PASS，停稳误差最大 0.039 m。

### 坡上驻点（重发 15 s，两次坡上停车的最大偏离）

| 配置 | 最大偏离 | 结果 |
|---|---:|---|
| D | 0.090 / 0.086 m | PASS |
| F1 | 0.084 / 0.088 m | PASS |
| FA | 0.135 / 0.113 m | PASS（判据 0.15 m，余量小） |
| 运动学（对照） | 0.286 / 0.257 m | 未通过 |

FA 偏离更大与阶段 2 观察一致：坡度补偿使停点维持在约 0.1 m，原因未分析。

### 滚动衔接与复杂路线

- 滚动衔接 F1、FA：到点均 PASS，无中途停车、MPC 覆盖未耗尽。`check_sim_rolling_continuation.py` 报热启动速度跳变：F1 2 次（0.10–0.11 m/s，判据 0.1），FA 4 次（0.25–0.34 m/s）。该判据针对已撤回的按空间进度热启动版本（见 `20261020_rolling_continuation.md` 头注），现行规划器按时间热启动，机器人落后参考约 0.3 m 时必然出现这种差值。FA 跳变明显更大，与坡度补偿加大跟踪滞后相符，未进一步分析。
- 删除前第一轮 `roll_K/roll_D` 因测试脚本把目标写成整数 `14` 触发 `check_sim_goal_accuracy.py` 的浮点断言而崩溃，修正为 `14.0` 后重跑。
- 复杂路线 D、F1、FA 均 PASS。运动学对照到点误差正常，但 LIO 高度漂移 0.55 m 超判据，属定位问题。
- **FA 复杂路线在台上 (5.75, 3.03) 卡住 43.5 s**，最后一段用时 78.5 s（其他段约 33 s）。D 和 F1 在同一位置也各停顿 1.4 s，运动学对照没有。卡住时车正在旋转（yaw −0.95，命令 wz ≈ 0.75 rad/s），0.40 m 方形车体的角外扩约 0.08 m，附近 y = 2.6 有台沿边界；参考速度约 1.2 m/s，但命令速度停在约 0.33 m/s，扰动观测器饱和在 −0.78 m/s²，前馈力饱和在 92 N。机理：力控 MPC 的输出速度是从实测速度按 `a_max·提前量` 外推得到的，车被挡住时速度接近 0，命令速度和驱动力都被加速度上限封顶，无法像运动学 MPC 那样直接给出接近参考的速度命令来“推出去”；`progress_checker` 容许 1000 s，Nav2 不会触发恢复。最终在规划器换了参考方向后脱困。控制律未改动，见下方待决事项。

### 删除运动学 MPC 后的回归（`batch2.sh`，各 1 轮）

| 用例 | 结果 | 要点 |
|---|---|---|
| `route6` D / F1 / FA | PASS | 总时长 20.7 / 20.7 / 22.3 s，停稳误差最大 0.068 / 0.092 / 0.051 m |
| `rolling` D | PASS | 到点误差最大 0.042 m；热启动跳变判据 1 次（同上，判据过时） |
| `hold` F1 / FA | PASS | 坡上最大偏离 0.092 / 0.117 m |
| `complex` F1 | 未通过 | 最后一段在坡上 (16.87, 12.06) 停 75 s 后超时：SMAC 2D 从该起点反复 “Failed to find path”，起终点代价均为 free，控制器无参考、输出零。前一段跟踪误差 p95 达 1.56 m，车偏离路线到了这个位置。删除前同配置一轮通过 |
| `complex` FA | 无效 | Point-LIO 组件加载超时，`camera_init` TF 缺失，`controller_server` 未能激活 |

两条复杂路线重跑时启动均失败：另一会话在同一工作区 15:53 修改了 `src/nav-simulation/.../simulation.launch.py`，引入只安装在 `install-real` 的 `sentry_simulation.real_robot`，并在 domain 191 运行实车模型仿真，内存与交换区已满。复杂路线需在机器空闲、子模块改动合入后重跑。

删除改动没有修改力控求解路径（`solveDynamic`、求解器、前馈），回归中的短路线与坡上用例和删除前一致。

## 限制与待决

- 实车：固件未烧录；物理参数与 `command_lookahead` 未标定；`wheel_odometry` 启用前需 `scripts/check_chassis_velocity_frame.py`；力前馈 `gain` 从 0 逐步加大，报告提示易振荡。
- 仿真底盘接受 wz，实车 NAVIGATION 模式忽略 wz，所以上面的旋转卡边在实车 NAVIGATION 下不会以同样方式出现，ROTOR 下仍可能出现。
- 待决：卡住时的升级策略（例如命令速度相对实测速度的下限、观测器饱和时放宽 `a_max`，或收紧 `progress_checker` 触发恢复），需要用户决定。
