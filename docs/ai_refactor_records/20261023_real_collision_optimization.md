# CAD 模型外观与碰撞分离（2026-10-23）

## 实施

修改 `src/nav-simulation/sentry_simulation/resource/models/sentry_real/model.sdf`：

- 12 个原始高精度 STL 外观保留，文件 SHA-256 未变；GPU 雷达仍使用这些外形。
- 底盘按实际 CAD 三角形裁切成 3×3 分区，各自拟合低面数凸体；不使用整车凸包。
- 上部结构的 7 个 link 分别拟合凸体，覆盖云台、俯仰机构及 MID360 主要轮廓。
- 每个轮子分别拟合轮胎和轮毂圆柱；半径约 62.98 mm、轮胎宽约 45.2–45.4 mm，
  中心和方向分别由实际 STL 与关节轴计算。
- 圆柱局部 Z 轴沿轮轴，`fdir1=0 0 1`，沿轮轴的 `mu=0.03`、切向 `mu2=0.6`。
  这些是被动滚子的等效仿真参数，不是实测轮胎标定；驱动仍由已有底盘力控制插件承担。

总计 16 个凸体网格、8 个原生圆柱；网格碰撞面数由 641,409 降至 1,472。
非轮子碰撞体最低点与轮底仍相隔至少 26 mm。导航 footprint 同步取原始外观与新碰撞体
投影的凸包，约 0.625×0.627 m；原有 0.02 m costmap padding、0.44 m 规划碰撞圆保持。
惯量、质量、所有非碰撞 SDF 元素、关节树及雷达/IMU 外参均保持原值。

模型目录中的 `generate_collisions.py` 可重建碰撞网格及 footprint；
`collision_geometry.json` 记录原始网格哈希和拟合尺寸，`collision_preview.png` 显示几何对照。
重复生成的 SDF、配置、报告和全部碰撞网格逐字节相同。

## 实跑结果

性能对比采用同一 RMUC2025 地形、出生位姿、原始外观和传感器配置，关闭 GUI 及 ROS 导航：
原始碰撞 RTF **0.1425**，优化碰撞 **0.4151**，提高 **2.91 倍**。
性能数值来自 `/clock` 的仿真时间/现实时间比；该轮基准的真值订阅未采到消息，
落地与运动验证由后续完整 ROS 验收单独完成。

完整导航运动采样 RTF 为 **0.2759**，该窗口最高真值速度 **1.319 m/s**。
这些数据不代表整套仿真已经达到实时率 1.0。

静止落地、前进、横移和正向旋转通过；前进/横移分别 0.2266/0.2258 m，
串轴偏移低于 4 mm，正向旋转 0.3501 rad。IMU/点云仿真时间频率为 1000/10 Hz。
三个目标在同一次运行中连续完成，均返回状态 4 并停车：

| 目标 | 停车后真值误差 | 残余线速度 |
| --- | --- | --- |
| (4.5, 8.5) | 0.02246 m | 0.000124 m/s |
| (4.6, 7.3) | 0.04545 m | 0.000151 m/s |
| (12.611, 10.593) | 0.03500 m | 0.000155 m/s |

远目标包含高台路段，最终底盘原点真值高度约 0.3388 m。
随后重新启动 Gazebo + RViz，目标 (4.5, 8.5) 再次通过，停车误差 0.01521 m；
有界面停车状态 RTF 采样 0.3561。最终保留该交互会话运行。

## 验证与复现

新增检查先在完整 STL 碰撞模型上失败，再随几何修改通过。
9 项 profile 回归覆盖原始网格哈希、轻量碰撞、原始顶点覆盖、轮轴/摩擦、
底盘分区/离地空间、footprint、质量惯量及已有坐标系关系。
仿真包 3 个 CTest 目标全部通过；`gz sdf -k` 通过，仅保留原有传感器扩展标签警告。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 src/nav-simulation/sentry_simulation/test/test_real_robot_profile.py
colcon test --packages-select sentry_simulation
ROS_DOMAIN_ID=87 GZ_PARTITION=sentry_collision_acceptance \
    ./start_sim.sh robot_model:=real world:=rmuc_2025 headless:=true rviz:=false
# 同域另一终端，按顺序运行
ROS_DOMAIN_ID=87 python3 src/nav-simulation/sentry_simulation/scripts/validate_real.py \
    log/sentry_collision_optimization/axes
ROS_DOMAIN_ID=87 python3 scripts/check_sim_goal_accuracy.py \
    log/sentry_collision_optimization/goals \
    --goals '[[4.5,8.5],[4.6,7.3],[12.611,10.593]]'
```

汇总在 `log/sentry_collision_optimization/summary.json`，逐项数据与启动日志在同目录。
逐滚子牵引、打滑及轮速反馈仍需单独的轮级动力学实现和硬件标定。
