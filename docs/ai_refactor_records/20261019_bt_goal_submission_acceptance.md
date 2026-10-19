# 单目标 BT 按目标变化提交：仿真验收

日期：2026-10-19。验收工作区当前单目标 BT：`GoalUpdater` 包住导航 `PipelineSequence`，`GoalUpdatedController goal="{updated_goal}"` 包住规划恢复分支，MINCO 内部重规划保持原实现。

## 环境与启动

默认 rmuc_2025，headless Gazebo + Point-LIO + GICP + Nav2 + MINCO/MPC。Nav2 lifecycle 全部 active，GICP 通过三个独立窗口确认定位。

```bash
ROS_DOMAIN_ID=94 GZ_PARTITION=minco_bt_acceptance_20261019 ./start_sim.sh headless:=true rviz:=false
```

首次启动受旧 Gazebo 残留进程影响：时钟串流、TF 未就绪，Nav2 激活失败。关闭旧实例及残留 Gazebo 后，使用上述 ROS 和 Gazebo 双重隔离重新启动，才执行下列验收。

## 结果

| 验收项 | 实测结果 |
| --- | --- |
| 固定目标 (4.5, 6.0) | 使用 `/speed_limit` 暂限速至 0.2 m/s；6 秒墙钟观察窗口内 `/plan` 仅发布 1 次 |
| MINCO 内部刷新 | 同期 `/opt_path` 收到 7 个不同轨迹编号：2 至 8；外层没有重复提交 |
| 动态目标 | 通过 `/goal_update` 更新为 (5.0, 5.5)，新增 `/plan` 恰好 1 次，目标发布到收到规划结果约 19.6 ms；随后 3 秒不再发布规划结果 |
| 更新目标到达 | 取消限速后 NavigateToPose status=4；Gazebo 真值 (4.93182, 5.46459)，到更新目标距离 0.07683 m；停车后真值平移速度为 0，底盘命令为 0 |
| 取消及同目标重启 | 向 (4.5, 6.5) 导航后取消，status=5；重新提交相同目标，恰好新增 1 次 `/plan`，最终 status=4 |

订阅 `/behavior_tree_log`、`/plan`、`/opt_path`、`/sim/ground_truth/odom` 和 `/sim/cmd_vel` 采集证据。验收脚本所有断言通过，退出码为 0。仅使用规划结果发布次数不能独立证明 action 调用次数，但本次同时采集了真实 BT 状态变化，且此前已用原生 BT 节点验证固定目标不重复执行规划 action。

运行产物保存在本机临时目录 `/tmp/minco_bt_sim_acceptance/`：`check.py`、`check.log`、`results.json`、`simulation_clean.log`；首次失败启动日志为 `simulation.log`。临时目录不会随仓库提交持久保存。

## 观察与覆盖边界

动态目标完成后启动更新的 action 目标，Nav2 原生 GoalUpdater 仍缓存旧的 topic 目标，并频繁记录旧时间戳忽略警告。本次未因此重复规划或影响到达；日志噪声仍存在。

本次是单目标 BT、上述目标及场景的实测。没有主动注入障碍或规划/控制故障；这些恢复分支此前通过原生 BT 节点加模拟 action 检查，本次不声称完成实物理故障恢复验证。多目标点 BT 不属于本次修改范围。

验收完成后关闭本次仿真，释放 Gazebo 和导航进程。
