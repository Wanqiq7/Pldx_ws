# 终端位姿跟踪与回归验证

## 控制契约

`NavigateToPose` 的目标位置和朝向是一组完整目标。重复发送同一位姿不会重启规划，但同位置的新 Yaw 会提交新任务；四元数 `q` 与 `-q` 仍视为同一朝向。

导航分为两种参考：

- 行进参考：保留原有路径方向、坡度相关 Yaw 策略，不因为局部路径已经包含终点就在数米外提前对齐最终朝向。
- 终端参考：距离最终目标不超过 `minco_optimizer.traj_goal_tolerance` 时进入，当前配置为 0.3 m。跳过短路径全局搜索，生成从实测平移速度到目标位置、末端速度为零的五次制动轨迹。航向使用原任务的最终 Yaw，按最短角度处理。

终端轨迹仍经过动力学、动态 ROG 安全间距、全局地图可通行性、规划预算和任务版本检查。地图任一路径不可用或阻塞时拒绝提交。全局地图同时按时间和空间分辨率采样，避免制动时跳过窄障碍。

`MpcPositionCommand.terminal_pose` 表示这一参考已通过终端规划验证。控制器按时间跟踪终端平移，而不是用近零位移寻找最近索引；最终 Yaw 为反馈目标，不使用短平移切线。终端角速度上限为 1 rad/s，并受已有更小角速度限制约束。启用加速度约束时，从更高角速度进入终端会依照 `alpha_min/max` 逐周期减速，不能通过硬限幅瞬间跳变。

任务成功仍由 Nav2 `StoppedGoalChecker` 决定，保留 XY/Yaw 的 0.1 m / 0.1 rad 容差及停车要求。进度检查改为 `PoseProgressChecker`，位置进展或 0.1 rad 角度进展都有效，10 秒进度超时不变。时效、BLOCK、取消及规划失败停车检查继续生效，Nav2 返回速度和 `/cmd_vel_mpc` 经过相同最终输出路径。

## 接口与部署

`MpcPositionCommand.idl` 和 `.msg` 同步新增了 `bool terminal_pose`。这是自定义 ROS 消息接口变化；部署时需要同步构建 `ros_interfaces`、`minco_planner`、`minco_controller`，重启有关节点和 Foxglove Bridge，不能混用新旧消息定义。没有更改话题、TF、QoS 或真实机器人地址。

Foxglove 可以观察 `/opt_path.terminal_pose`，并对比终端参考中的 `cmds[*].yaw`、`/cmd_vel_mpc.angular.z`、`/cmd_vel_nav.angular.z` 和里程计。`terminal_pose=false` 时，参考末端朝向可以是行进方向；最终目标朝向由终端阶段执行。

## Headless 回归

启动仿真：

```bash
./start_sim.sh robot_model:=real world:=rmul_2027_3v3 headless:=true rviz:=false
```

另一个已加载 ROS 环境的终端运行：

```bash
source /opt/ros/jazzy/setup.bash
source install-link/setup.bash
python3 scripts/check_sim_terminal_pose.py log/terminal_pose_regression
```

脚本提供 `--scenario routes` / `--scenario rotations` / `--scenario adjustment` / `--scenario cancel`，可以在独立的新仿真中隔离行进和旋转。使用真实导航 action，仿真仍运行 Point-LIO，不发布手工速度，也不放宽到位容差。

验收除 action 状态外，还记录同一时间戳的 Gazebo 真值与 `map → base_link`。检查终点位置、Yaw、到达位置容差后的耗时、三路速度、8 秒墙钟停车观察、采样新鲜度和定位误差。每个真值时间戳只计一次；定位漂移超过 0.2 m 时停止测试，避免向漂移后的估计位置发送新目标。结果、轨迹和告警分别保存为 `terminal_results.json`、`terminal_trace.json` 和 `terminal_warnings.json`。

现有折叠通道约束在仿真通行适配器中独立执行。相关静态回归为 `scripts/test_sim_tunnel_navigation.py` 和仿真包的 `test/test_folding_integration.py`；平地终端测试不等价于实体狗洞通行验收。

## 2026-11-14 验证结果

最终实现的专项通过记录如下。它们来自独立 headless 会话，不代表完整连续序列全绿；角度误差列为结果后的同时间真值/TF采样中的估计 Yaw 误差。

| 用例 | Action 状态 | 仿真耗时 | 估计 Yaw 误差 |
|---|---:|---:|---:|
| 去程 | 4 | 12.20 s | 0.087 rad |
| 回程 | 4 | 14.00 s | 0.087 rad |
| 同位置 90° | 4 | 3.60 s | 0.074 rad |
| 转向 π 附近 | 4 | 3.40 s | 0.091 rad |
| 跨 ±π | 4 | 1.78 s | 0.088 rad |
| 近目标位姿微调 | 4 | 1.88 s | 0.092 rad |
| 活动任务取消 | 5 | 1.02 s | — |

- 回程进入 0.1 m 位置容差后约 4.46 秒完成；先前调查中的同类回程约 36 秒。该比较是一次复现记录，不是统计性能基准。
- 上表每个用例均完成 8 秒墙钟停车观察，三路速度最大绝对值为 0；Gazebo 真值 Yaw 波动均小于 0.00004 rad。
- 反向到达的 action 也成功（状态 4，估计 Yaw 误差 0.093 rad），三路速度为 0，真实 Yaw 静止；但停车期间定位误差增至 0.36 m，触发 0.2 m 验收门限，**完整路线序列判定失败**。另一轮旋转长序列在后续微调时也出现定位漂移和样本陈旧。定位链稳定性仍需单独调查，不能据此宣称完整任务链已修复。
- 已构建的 18 个包完整测试：110 tests，0 errors，0 failures，1 skipped。`dbscan_cluster`、`dual_lidar_calibration`、`msg_convert` 未构建，首次全工作区测试因缺少这三包安装文件未能启动它们；随后显式跳过三包运行其余完整测试，不能把它们计入已通过范围。
- 通道/折叠静态回归：`scripts/test_sim_tunnel_navigation.py` 7 项、`test_folding_integration.py` 8 项通过。未用平地结果代替实体狗洞动态验收。
- Standards / Spec 双轴审查提出的静态地图检查、角加速度限制和脚本新鲜度/证据保存问题均已修复，最终复核无未解决的阻塞发现。

本次本地证据位于 `log/arrival_yaw_fix_v3/`、`log/arrival_yaw_fix_v3_rotations/`、`log/arrival_yaw_fix_v3_adjustment_ready/`。保留其中 `passed=false` 的记录，它们是定位稳定性问题的复现依据。
