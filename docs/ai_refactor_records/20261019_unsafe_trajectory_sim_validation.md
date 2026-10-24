# FOLLOW_TRAJ 不安全轨迹撤销：仿真验证

日期：2026-10-19。验证本次 `minco_fsm.cpp` 修复：局部重规划失败时，仅允许保留仍安全且未过期的旧轨迹；不安全轨迹通过 `invalidateTrajectory()` 撤销，控制器清除缓存并输出零命令。

## 环境

使用当前工作区安装产物，rmuc_2025、Gazebo、Point-LIO、GICP、Nav2、MINCO/MPC，RViz 开启。启动命令：

```bash
ROS_DOMAIN_ID=95 GZ_PARTITION=minco_unsafe_debug_20261019 ./start_sim.sh headless:=true rviz:=true
```

Nav2 lifecycle 全部 active，GICP 三个独立窗口完成定位确认。通过 planner_server 进程的 `/proc/<pid>/maps` 确认加载当前工作区的 `libminco_planner.so`。

## 正常导航基线

复用此前 BT 仿真验收脚本：固定目标 (4.5, 6.0) 的 6 秒观察窗口内 `/plan` 仅发布一次，内部轨迹编号从 2 更新至 8；更新目标 (5.0, 5.5) 后仅新增一次规划，约 20.6 ms 收到规划结果。导航最终 status=4，Gazebo 真值距目标约 0.064 m，平移速度与底盘命令为零。取消后重新提交相同目标也成功，新增一次规划。

## 动态障碍验证

向 (4.5, 8.0) 导航，在机器人前方约 0.85 m 处通过 Gazebo `/world/default/create` 插入静态墙体（3.0 × 0.35 × 1.2 m），由模拟激光雷达进入实际感知/地图链路。每轮观察约 12 秒，然后删除墙体并取消测试目标。

订阅 `/opt_path` 和 `/sim/cmd_vel`，同时保存轨迹碰撞、候选轨迹校验失败日志。两轮都观察到阻断消息，且阻断到新轨迹之间底盘命令全部为零。

第二轮特意记录了轨迹采样点数。正常轨迹以 0.05 秒间隔采样、点数为 `ceil(duration / 0.05) + 1`；因此用 `header.stamp + (点数 - 2) × 0.05` 给出旧轨迹结束时间的保守下界。阻断时的剩余时间如下：

| 旧轨迹编号 | 剩余时长下界 | 阻断至新轨迹的墙钟时间 | 新轨迹编号 | 等待期间零命令数 |
| --- | --- | --- | --- | --- |
| 54 | 1.462 s | 24.4 ms | 56 | 3 |
| 60 | 5.415 s | 119.1 ms | 62 | 12 |
| 65 | 3.222 s | 63.2 ms | 67 | 7 |

这验证了旧轨迹尚未到期时也会被撤销，而非继续执行到到期。首个零命令在接收阻断消息后不到 1 ms 被观察到；该数值是同一 Python 观察节点的消息接收间隔，不代表完整感知到制动延迟，也不代表物理底盘瞬时停止。

本次验证覆盖动态障碍触发的轨迹撤销、控制缓存清除和新轨迹恢复。墙体存在期间未等待绕行目标完成，因此不将结果视为完整绕障到达验收。

## 运行产物

本机临时目录 `/tmp/minco_unsafe_sim_debug/` 保存 `simulation.log`、`baseline.py`、`baseline_results.json`、`obstacle.py`、`obstacle_results.json`、`obstacle_round2.py`、`obstacle_round2_results.json`、`check_obstacle.py` 和 `obstacle_checks.json`。复查命令：

```bash
python3 /tmp/minco_unsafe_sim_debug/check_obstacle.py
```

检查断言：至少一次旧轨迹结束时间下界晚于阻断时间，且每个被检查的阻断至新轨迹区间内观测到的底盘命令全部为零。断言已通过。临时目录不会随仓库持久保存。
