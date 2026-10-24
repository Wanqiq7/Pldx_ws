# 完整 STL 仿真反复停车修复（2026-10-23）

> 历史记录：外观/碰撞分离完成后，已按用户要求撤销本文的接收超时倍率参数及其
> 仿真覆盖配置。当前控制器对 ROS 年龄和现实接收间隔均使用原有 0.3 s / 2 s 阈值。
> 验收脚本保留同时间戳采样和按仿真时间判断停车的测量修正；目标截止恢复为 90 s。

## 原因

原始高精度 STL 同时用于 visual/collision 时，实测 Gazebo 实时率约为 0.12。
控制器此前用相同阈值检查两个不同时间尺度：

- ROS 消息时间戳年龄：轨迹 2 秒，里程计 0.3 秒。
- steady-clock 现实接收间隔：同样分别为 2 秒和 0.3 秒。

Point-LIO 会按扫描批次输出里程计，现实接收间隔可超过 0.8 秒；
轨迹刷新现实间隔可超过 7 秒。仿真时间内仍有效的数据因此被现实看门狗拒绝。
控制器持续发布零速度并重置求解器上一周期加速度，使 RViz 中轨迹正常但小车间歇爬行。

修复前，在干净出生状态执行 `(4.5, 8.5)` 会超时。
临时诊断记录捕获 `STALE_INPUT`，里程计 ROS 年龄约 0.05 秒、轨迹现实接收年龄约 7.9 秒。
诊断代码已从生产源码删除，证据保留在 `log/sentry_stl_motion_fix/baseline_simulation.log`。

## 修改

- `FollowPath.input_receive_timeout_scale` 默认 `1.0`，只乘在现实接收间隔阈值上。
- CAD 真实模型仿真 profile 使用 `100.0`，覆盖约 0.01 及以上实时率：
  里程计现实接收上限 30 秒，轨迹现实接收上限 200 秒。
  高台完整网格接触区实测实时率约为 0.026。
- ROS 消息年龄阈值仍为 0.3 秒 / 2 秒；未来时间戳、真正过期消息、BLOCK、取消、
  轨迹更新竞争以及最终命令提交处的保护继续生效。
- 实车和其他未指定倍率的配置保留默认 `1.0`。
- `check_sim_goal_accuracy.py` 按目标结果时间等待真值和 TF 到齐，不再依赖固定现实延迟；
  采样时间对齐误差仍必须小于 15 ms。

SDF 和全部 12 个 STL 的 SHA-256 与上一轮恢复后的模型完全一致。
碰撞几何、安全半径、误差阈值和物理动力学未因本次修复修改。

## 回归与命令

新增三个控制器回归测试，覆盖慢速接收批次、提交时检查和保留的过期保护。
其中两个测试在修复前失败，修复后全部通过。
控制器测试结果为 41 项、0 失败、1 个可选项跳过；仿真包三个 CTest 目标通过。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --symlink-install \
    --packages-select minco_controller --executor sequential
colcon test --packages-select minco_controller sentry_simulation

ROS_DOMAIN_ID=86 GZ_PARTITION=sentry_stl_final_verified \
    ./start_sim.sh robot_model:=real world:=rmuc_2025 headless:=true rviz:=false

# 在相同 ROS domain、已 source 环境的另一终端运行
ROS_DOMAIN_ID=86 python3 scripts/check_sim_goal_accuracy.py \
    log/sentry_stl_motion_fix/final_goals \
    --goals '[[4.5,8.5],[4.6,7.3],[12.611,10.593]]' --goal-timeout 300
```

主验收从 `(4.6, 7.3)` 的干净出生状态启动。结果采样和实际运行参数分别保存在
`log/sentry_stl_motion_fix/final_goals/` 和 `runtime_parameters.json`。
