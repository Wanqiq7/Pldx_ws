# sentry_real 仿真慢：网格碰撞体换成盒子

## 现象

`robot_model:=real` 在 Gazebo 中实时因子约 0.24（完整栈 + GUI），机器人看起来比
`sentry_omni` 慢约 4 倍。

## 原因

Gazebo server-only 对照（同一 rmuc_2025 世界与出生点，无 GUI/ROS，每次只改一项）：

| 变体 | RTF |
|---|---|
| sentry_omni | 1.00 |
| sentry_real 原样（16 个凸包网格 + 8 圆柱） | 0.41 |
| 去掉 GPU 雷达 / 低面数视觉 / IMU 200 Hz / bullet 检测器 | 0.42 |
| 只保留轮子圆柱 | 1.00 |
| 仅底盘 9 块网格 | 0.69 |
| 仅上部 7 块网格 | 0.87 |
| 16 块网格换成包围盒 | 1.00 |

每个网格碰撞体每步都要与 114k 面的地形网格做检测，开销随网格碰撞体个数增长，与其面数
（每块仅 92 面）关系不大。雷达、视觉网格和 IMU 不是瓶颈。

## 修复

`generate_collisions.py` 只输出图元：

- 底盘按导航轴对齐（CAD 坐标系 45°）切 3×3 网格，每格一个盒子；轴对齐 CAD 网格会把
  圆角整块包进去，footprint 涨到 0.86 m 见方。
- `360_ref_ref1` 按同样方向切 2×2；其余上部 link 各一个最小面积偏航盒子。
- 轮子圆柱与摩擦设置不变。共 19 个盒子 + 8 个圆柱，删除 `meshes/collision/*.stl`
  （备份在 `log/sentry_rtf_debug/before_box/`）。
- `test_real_robot_profile.py` 改为禁止网格碰撞体，按盒子检查顶点覆盖与底盘分块。

footprint 由 x −0.287…0.338、y −0.339…0.287 变为 x −0.287…0.338、y −0.347…0.287；
最大半径 0.4085 → 0.4106 m（`small_pitch`），仍小于 `collision_radius` 0.44。

## 验证

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
python3 src/nav-simulation/sentry_simulation/resource/models/sentry_real/generate_collisions.py  # 两次输出一致
python3 src/nav-simulation/sentry_simulation/test/test_real_robot_profile.py                      # 9/9
gz sdf -k src/nav-simulation/sentry_simulation/resource/models/sentry_real/model.sdf              # Valid
colcon build --symlink-install --packages-select sentry_simulation
ROS_DOMAIN_ID=218 colcon test --packages-select sentry_simulation                                 # 3/3
python3 log/sentry_rtf_debug/bench_variants.py real                                               # RTF 1.00
ROS_DOMAIN_ID=219 GZ_PARTITION=sentry_box_acceptance \
    ./start_sim.sh robot_model:=real world:=rmuc_2025 headless:=true rviz:=false
ROS_DOMAIN_ID=219 python3 src/nav-simulation/sentry_simulation/scripts/validate_real.py log/sentry_rtf_debug/accept/axes
ROS_DOMAIN_ID=219 python3 scripts/check_sim_goal_accuracy.py log/sentry_rtf_debug/accept/goals \
    --goals '[[4.5,8.5],[4.6,7.3],[12.611,10.593]]'
```

结果：完整栈无界面 RTF 1.00（静止与移动中）。轴向检查通过（前进/左移 0.226 m，
偏航 0.350 rad，IMU 988 Hz，雷达 10 Hz）。三目标全部 status 4，到点真值误差
0.0180 / 0.0425 / 0.0408 m。

## 未验证

- 带 GUI + RViz 的 RTF（原问题场景）。
- 盒子比原凸包更胖，部分上部轮廓更保守；不代表实车外形或轮级打滑动力学。
