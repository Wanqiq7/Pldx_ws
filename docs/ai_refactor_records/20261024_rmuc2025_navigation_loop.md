# RMUC2025 长路线循环导航与定位测试

日期：2026-10-24。地图 `rmuc_2025`，模型 `sentry_real`，Gazebo + RViz。

## 结论

已复现返程持续跟踪失败、最终导航超时。故障期间同步定位与 Gazebo
真值仍相差约 3 cm，未观察到定位丢失。测试不能判为整体通过。

需要区分两类现象：

- 候选轨迹碰撞/动力学校验失败，随后重规划成功：成功的五轮内也多次出现。
- 车辆实际停滞，但仍持续输出运动命令并执行 HOT_START：追加返程与原地重发
  目标均超时。上部外伸结构与低墙接触是有几何证据支持的原因，尚缺物理引擎
  接触遥测确认，不能把所有候选轨迹拒绝都归因于它。

将 MINCO 圆从底盘半径 `0.321 m` 改为全车包络 `0.44 m` 的对照也未通过：
返程 SMAC 反复报无路。已恢复原参数；本次交付是复现、测量和诊断，尚无已验证
的规划/卡死修复。

## 测试方法与结果

使用独立 ROS domain/Gazebo partition，避免连接旧世界或真实机器人。
目标在 `(24.0, 7.3)` 与 `(4.6, 7.3)` 之间交替；场内绕行后每个完整往返约
90 m。每段导航超时为 90 s 墙钟时间，成功后等待稳定再检查真值到点误差。
定位误差由同一时间戳的 `map/base_link` TF 与 Gazebo 真值比较，而非直接
相减不同时间的最新消息。

| 数据集 | 完成目标数 | 真值轨迹累计长度 | XY 定位误差最大值 | 结果 |
| --- | ---: | ---: | ---: | --- |
| `initial_route` | 2/2 | 89.286 m | 0.04490 m | 首次往返通过 |
| `repeated_route` | 10/10 | 451.329 m | 0.04734 m | 连续五轮通过 |
| `candidate_capture` | 1/2 | 84.342 m | 0.05102 m | 返程超时 |
| `cold_retry` | 0/1 | 2.660 m | 0.03538 m | 卡住位置重发目标仍超时 |
| `full_envelope_route` | 1/16 | 50.472 m | 0.04030 m | 原计划八轮，首轮返程超时即停止 |

长度是采样真值位置的累计路程，包含卡住期间的抖动，不等于有效前进距离。
基线首次往返、五轮循环与追加测试累计有 13 个目标成功，但随后两次失败；
不能用前五轮成功替代持续运行验收。

五轮主测试耗时 363.2 s 仿真时间、366.24 s 墙钟时间，定位误差 P95
为 0.03698 m，最大航向误差 0.3814°。十个目标稳定后的真值到点误差
为 0.0256–0.0851 m，均小于脚本 0.15 m 阈值；全程定位误差小于 0.12 m。

## 候选轨迹拒绝与实际卡死

五轮主测试的启动日志共记录：

- `validateTrajectory: collision detected`：26 次。
- `validateTrajectory: severe dynamics violation`：17 次；其中加速度约
  3.03–3.43 m/s²，超过该运行配置的 3.0 m/s² 严重违例门限。
- `Trajectory validation failed`：43 次，与前两项合计一致。
- `Yaw optimization failed`：12 次，触发回退。

这些候选拒绝后都完成了目标，故红色可视化/标准输出不能直接解释为导航最终
失败。该阶段 `/rosout` 的 ERROR 数为零，也不能据此认为完全没有 MINCO
校验失败：相关信息有一部分只写到标准输出。

追加返程停在约 `(5.83, 3.03)`，机器人几乎不前进，仍接收约
0.5–0.7 m/s 的速度命令。日志反复出现：

```text
Large tracking error ... Continuing HOT_START.
```

此时跟踪误差约 1.1–1.5 m，定位误差仍约 0.03 m。取消后在同一位置重发返程
目标也未解除卡死，因此仅重置目标/热启动状态不足以解决这一案例。

## 接触与碰撞圆证据

离线检查使用保存的完整 Gazebo 位姿、`sentry_real/model.sdf` 中的碰撞盒，
以及 RMUC2025 的 STL 三角形。地形网格必须叠加模型平移 `[14.5, 8, 0]`。

- 轮子位于约 `z=0.203 m` 的平台；非轮底盘盒底部约有 27 mm 以上净空。
- 严格穿透检测未发现碰撞盒与地形内部相交；这与物理引擎将机器人停在接触
  表面相容，不能据此排除接触。
- 将碰撞盒各半轴扩展 2 mm 后，只有 `small_pitch_collision` 与邻近地形
  三角形相交；含接触边界的检测也找到该盒与五个三角形接触。
- 相关低墙位于 `y=2.5–2.65 m`，顶部约 `z=0.403 m`；该外伸盒底部约
  `z=0.3609 m`，高度上存在干涉条件。
- MINCO 当前只使用底盘圆 `0.321 m`。全车投影最远处约 `0.4106 m`，
  profile 的全车包络圆为 `0.44 m`；该位置静态占用栅格边缘净空约
  `0.378 m`，可能通过底盘圆检查但无法容纳上部外伸。

因此，上部结构贴墙与导航包络不匹配是本次物理停滞的强候选原因。此检查是
离线 OBB/三角形 SAT，未采集 Gazebo 接触传感器数据；静态栅格、动态投影和
物理网格的局部差异也仍需进一步区分。

## 全车圆对照及恢复

独立重启同图同模型，将 `configure_navigation` 中 MINCO 半径来源从
`chassis_radius` 改成 `collision_radius`，即 `0.321 → 0.44 m`。
由此 corridor 半径同时变化，派生的 `safe_dist` 也从启动后的 `0.55` 增为
`0.59 m`；这是半径策略的整体对照，并非保持所有其他有效参数不变的单变量实验。

去程完成，但返程在约 `(21.57, 10.70)` 附近停滞，SMAC 反复在 30000 次
扩展后耗尽 open set；记录到 590 条 ROS ERROR，最终返程超时。不能将盲目
增大碰撞圆作为有效修复。

已恢复 `configure_navigation` 使用 `chassis_radius`，保留原碰撞几何、
`chassis_radius: 0.321` 与 `collision_radius: 0.44`。启动日志现明确打印
实际生效的 `planning_radius` 和 `chassis_geometry_radius`，便于核对配置。

后续修复应重点验证：规划轨迹对完整 footprint 的朝向/高度约束，以及在持续
无进展且跟踪误差增大时退出 HOT_START、执行有边界的恢复。当前证据尚不足以
选定最终实现，需用本次失败路线再次验收。

## 复现命令与产物

新增 `scripts/check_sim_navigation_loop.py`，复用已有同步测量及 action 取消
逻辑，支持循环次数、目标、超时、定位/到点阈值，并保存每段结果、真值/TF
样本、ROS 警告及候选状态变化。首个失败即停止并非零退出。

在工作空间根目录，终端一启动全新仿真：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
mkdir -p log/rmuc2025_navigation_loop/repro
export ROS_DOMAIN_ID=91
export GZ_PARTITION=minco_loop_rmuc2025_repro
export ROS_LOG_DIR="$PWD/log/rmuc2025_navigation_loop/repro/ros"
ros2 launch sentry_simulation simulation.launch.py \
    world:=rmuc_2025 robot_model:=real headless:=false rviz:=true \
    >log/rmuc2025_navigation_loop/repro/launch.log 2>&1
```

终端二使用相同 ROS domain，重复长路线：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=91 python3 scripts/check_sim_navigation_loop.py \
    log/rmuc2025_navigation_loop/repro/route \
    --cycles 10 --goal-timeout 90 \
    --goals '[[24.0,7.3],[4.6,7.3]]' \
    --launch-log log/rmuc2025_navigation_loop/repro/launch.log
```

卡死有偶发性，首次五轮成功后继续执行才捕获本例；上述命令提供可无人值守、
遇超时/误差超限即失败的回归入口，不保证每次在相同轮次触发。
`candidate_status.jsonl` 记录的是 marker 状态变化；15 Hz 发布可能漏掉短暂
候选拒绝，精确次数应同时查看启动标准输出。

本机诊断产物位于 `log/rmuc2025_navigation_loop/`（生成日志不纳入版本管理）：

- [汇总统计](../../log/rmuc2025_navigation_loop/analysis.json)
- [路线、停滞位置与定位误差图](../../log/rmuc2025_navigation_loop/route_and_localization.png)
- `baseline.log`、`full_envelope.log`：两种半径策略的启动与规划日志。
- 各数据集下的 `summary.json`、`samples.jsonl`、`rosout.jsonl`。
- `stalled_pose.json`、`contact_margin.json`、`stalled_near_contacts.json`：几何证据。
- `stalled_references/`：停滞期间路径交接与命令采样。

## 检查与收尾

- 两个测量脚本 `py_compile`、循环脚本 `--help` 检查通过。
- 恢复原配置后 `colcon test --packages-select sentry_simulation
  --event-handlers console_cohesion+`：3 个测试入口全部通过；profile 中 10 个
  Python 用例通过。`colcon test-result --test-result-base build/sentry_simulation
  --verbose`：6 tests，0 errors，0 failures。
- 主仓库与仿真子模块的 `git diff --check` 通过。
- 本次基线与对照仿真均已关闭，检查两个测试专属 Gazebo partition 没有遗留进程。

包级单元测试通过仅表示当前实现自检通过，不表示此次长路线故障已经修复。
