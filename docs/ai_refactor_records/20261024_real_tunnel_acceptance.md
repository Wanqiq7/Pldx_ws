# RMUC_2026 real 模型实际过洞验收

日期：2026-10-24。结论：**本次实际过洞验收未通过**。
Gazebo 和 RViz 已启动；测试结束后机器人已退回入口、恢复常态并停止。

## 测试条件

通过 `start_nav.sh --sim` 启动 `robot_model:=real world:=rmuc_2026`。
`start_nav.sh` 原有无参数入口启动实车链路，本次增加显式 `--sim` 分支，转交
`start_sim.sh`。本次命令如下：

```bash
DISPLAY=:1 ROS_DOMAIN_ID=171 \
GZ_PARTITION=sentry_tunnel_entrance_20261024 \
NAV_INSTALL_DIR="$PWD/install" \
SENTRY_SIMULATION_SHARE="$PWD/log/tunnel_acceptance_rmuc2026_20261024/entrance_share" \
./start_nav.sh --sim robot_model:=real world:=rmuc_2026 \
    use_icp:=true headless:=false rviz:=true log_level:=warn
```

测试选择己方左侧实际低顶通道，入口真值目标为 `(12.3, 13.425)`，
底盘航向为 `+π/4`；折叠后小云台临时前向约为地图 `+X`。
洞内导航目标为 `(13.65, 13.425)`，整车出洞目标为 `(14.35, 13.425)`。
这些坐标是本轮测试点，未修改正式地图或默认出生点。

`log/.../entrance_share` 是本轮测试资源副本：只覆盖入口出生位姿、增加各碰撞
link 的接触传感器及 Gazebo Contact 系统。地形网格、机器人原有碰撞几何及
规划安全参数保持原值。Contact 系统实际发布 Gazebo 原生完整传感器路径，
脚本按 `/world/default/model/sentry/link/<link>/sensor/acceptance_contacts/contact`
订阅，并用轮地接触确认传感器工作。

## 实测结果

| 检查 | 结果与证据 |
| --- | --- |
| 常态 → 折叠 → 恢复 | 到位成功，`faults=[]` |
| 折叠时临时前向 | 低速试穿记录 112 个采样，小云台真值前向与地图 +X 最大偏差约 0.682° |
| 洞内自主导航 | 失败，NavigateToPose 返回 `ABORTED=6`，机器人未进入通道 |
| 规划拒绝原因 | 目标栅格代价 `253`，SMAC 报告 `goal cell is not traversable` |
| 低速物理试穿 | 沿临时前向请求 0.10 m/s；出现非轮部接触后立即停止，未到达出口 |
| 首个接触时机器人真值 | `(13.26455, 13.42458)`，底盘航向约 0.77513 rad |
| 接触组件 | `sentry::360_ref_ref1::360_ref_ref1_00_collision` 与 `rmuc_2026::body::collision` |
| 首个接触世界坐标 | 约 `(13.46973, 13.42773, 0.18614)` m |
| 退回 | 以 0.06 m/s 请求倒退至入口，成功；随后恢复常态 |
| 结束状态 | `target=normal`、`stage=normal`、`ready=true`，最后测试采样命令与真值速度均为零 |

接触的是大 Yaw 基座组件当前使用的近似碰撞盒。初次接触深度处于引擎容差附近，
随后退回阶段同一碰撞对报告的最大正深度约为 4.85 mm。验收因此中止，
不能声明已经无接触穿过，也不能仅据碰撞盒结果断定真实 CAD 一定无法穿过。
需要对照网格、支撑地面和实际运行姿态复核这处包络。

**后续勘误**：原 Featherstone 将 Bullet 的有符号距离直接填入 `depth`，
并发布仍有间隙的持久流形点。因此上面的正数不能解释为穿透深度；
本轮基于“收到任意非轮接触点”的物理判据存在误报。
修正后的物理试穿已成功，悬空间隙、CAD 包络和自主导航限制见
[净空复核记录](20261024_real_tunnel_clearance.md)。原自主导航失败结论仍成立。

运行时确认 `MincoPlanner.minco_optimizer.collision_dist=0.44` m，
`MincoPlanner.rog_map.projection.tunnel_height_delta_min=0.34` m。
日志证明本次目标被代价地图拒绝；当前参数尚不能给这条折叠路线放行。
本轮没有缩小碰撞半径或高度阈值来强行获得成功结果。

GICP 本轮初始化超时并使用配置的默认位姿；导航结果时平面定位误差约 0.079 m。
因此本轮不作为高精定位通过证明。低速物理试穿的位置和接触判断分别使用
Gazebo 真值及物理接触传感器，与此定位回退独立记录。

## 产物与复跑

- [完整结果](../../log/tunnel_acceptance_rmuc2026_20261024/entrance_checked/result.json)
- [逐帧采样](../../log/tunnel_acceptance_rmuc2026_20261024/entrance_checked/samples.jsonl)
- [导航启动日志](../../log/tunnel_acceptance_rmuc2026_20261024/entrance_launch.log)
- [3 分钟过程录像](../../log/tunnel_acceptance_rmuc2026_20261024/tunnel_acceptance.mp4)
- [试穿画面](../../log/tunnel_acceptance_rmuc2026_20261024/physical_passage.png)
- [退回恢复后的画面](../../log/tunnel_acceptance_rmuc2026_20261024/final_normal.png)
- [结束状态复核](../../log/tunnel_acceptance_rmuc2026_20261024/final_status.json)

录像为过程节选，完整动作与退出状态以 JSON 和日志为准。在上述带接触传感器的
测试资源及入口姿态下，可复跑：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=171 GZ_PARTITION=sentry_tunnel_entrance_20261024 \
python3 scripts/check_sim_tunnel.py log/tunnel_acceptance_rerun \
    --model log/tunnel_acceptance_rmuc2026_20261024/entrance_share/resource/models/sentry_real/model.sdf
```

`check_sim_tunnel.py` 将自主导航和低速物理试穿分别记分，碰撞即停、失败倒退，
退回入口后才恢复常态；验收失败返回非零。脚本要求上述入口和传感器配置，
不能直接用于任意地图或出生点。

本轮改动的 `start_nav.sh`、`start_sim.sh` 已通过 `bash -n`，验收脚本通过
`python3 -m py_compile`，启动脚本的模拟/实车路由已用临时桩脚本验证。
未重构规划器或改变机器人模型；后续工作应先复核接触盒的准确性与机械净空，
再配置经过验证的折叠态导航包络，重跑整车完整过洞。
