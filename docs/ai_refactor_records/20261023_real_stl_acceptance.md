# 原始 STL 碰撞模型实跑验收（2026-10-23）

## 结论

**总体未通过。** 静止落地、前进、横移、正向旋转、传感器和定位检查通过；
默认控制参数下，第一个导航目标未到达，因此没有继续执行回程与远距离目标。
本次保留每个部件 visual/collision 共用原始 STL 的配置。

## 测试配置

- `robot_model:=real world:=rmuc_2025 headless:=true rviz:=false`。
- 12 对 visual/collision 引用相同 STL，局部 pose 和 scale 相同。
- 使用当前真实模型专用出生点 `(4.6, 7.3)`、17 顶点 footprint 和 0.44 m MINCO 圆包络。
- 主验收使用独立 ROS domain 81 和独立 Gazebo partition。
- 模型及网格 SHA-256 记录于 `log/sentry_real_stl_acceptance/manifest.json`。
- 三个仿真包 CTest 目标通过；CAD/profile 检查中的六项断言组通过。

## 基础验收

`python3 src/nav-simulation/sentry_simulation/scripts/validate_real.py log/sentry_real_stl_acceptance/axes`

| 检查 | 结果 |
| --- | --- |
| 静止观察期间 XY 漂移 / Z 变化 | 0 / 0 m（按真值采样） |
| 前进 | 0.1084 m，侧向偏移 -0.0367 m |
| 左移 | 0.1025 m，纵向偏移 -0.0049 m |
| 正向旋转 | 0.3518 rad，平移约 0.0005 m |
| 运动阶段最大 XY 定位误差 | 0.0210 m |
| IMU / 雷达频率 | 1000 / 10 Hz（仿真时间） |
| 每帧雷达点数 | 18524–19053 |

观察到 `body` 和 `base_link` 分别只有一个父坐标系；发布节点清单见 axes/result.json。
该检查不等同于通过 publisher GID 验证每个 TF 子帧的唯一发布者。

## 导航验收

```bash
python3 scripts/check_sim_goal_accuracy.py log/sentry_real_stl_acceptance/goals \
    --goals '[[4.5,8.5],[4.6,7.3],[12.611,10.593]]' --goal-timeout 300
```

第一个目标 `(4.5, 8.5)` 在 300 秒现实等待后超时，约经过 38.5 秒仿真时间。
最后真值位置约 `(4.7348, 7.8318)`，目标误差 **0.7083 m**；
全过程采样最大 XY 定位误差为 **0.0169 m**。
目标已取消，随后两个目标未执行。

新增的 `--goal-timeout` 只控制测试脚本的现实等待时间，默认仍为 90 秒。
目标误差 0.15 m、定位误差 0.12 m、停车等验收阈值没有放宽。

## 诊断观察与边界

- 五秒观察期间实时率约 0.12。
- 里程计批次间现实接收间隔约 0.75–0.83 秒，批次内消息时间戳按约 0.01 秒仿真间隔递增。
- 默认控制器同时检查 ROS 时间年龄和 steady-clock 接收年龄；后者限制为 0.3 秒。
  因此存在慢速仿真触发接收超时的条件。不能仅凭此断定它是唯一根因。
- 在独立 domain 82 的临时参数副本中，仅把 `odom_timeout_sec` 改为 1.5 秒做诊断。
  同一目标在 120 秒现实等待后仍超时，末端误差约 0.4941 m。
  两次初始姿态和等待时长不同，因此不作为严格的性能提升对比。
- 临时参数位于日志目录；共享控制参数与生产控制器代码没有因本次验收而调整。
- 本轮没有证明车体嵌入墙面；导航未通过也不能归结为单一碰撞几何问题。

下一步应分别检查仿真时间/现实接收看门狗、零速度输出占比，以及完整 STL 接触下的速度跟随。
这些问题解决后，应从干净出生状态重新执行三个导航目标。

所有本次启动的测试实例已请求退出；数据入口为
`log/sentry_real_stl_acceptance/summary.json`，原始命令、采样及日志保留在同目录。
