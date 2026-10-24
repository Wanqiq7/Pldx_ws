# RMUC_2026 窄洞净空复核

日期：2026-10-24。

**低速物理穿越成功；完整自主导航过洞验收尚未完成。**
本轮采用原始地形与 CAD 视觉资源，通过真值反馈控制走过实际低顶通道。
“240 mm 车高 / 250 mm 洞高”有通过的几何可能，但 10 mm 名义余量不能直接
用作实际动态余量，也不能据一次仿真穿越保证真实机器可靠通过。

## 已定位并修正的问题

1. 原 Gazebo Physics 7.6.0 Featherstone 对地形 GImpact 网格固定使用 10 mm
   margin，轮子实际悬空约 10 mm。包内引擎改为 1 mm，未移动视觉或地形顶点。
   原始地形的轮底复核从约 10 mm 降为约 1 mm。
2. 上装碰撞盒填补 CAD 外部空角，原始首个报告点距真实 CAD 曲面约 110.75 mm。
   折叠运行时改为七个上装 link 各自的 CAD 凸包；底盘和轮子保留原基本体。
   原生 `convex_hull` 仍调用体素分解，带来约 3–5 mm 外扩，现直接使用 CAD 顶点。
   凸包仍会填补 CAD 凹陷，属于保守包络。
3. 原静态地形 GImpact 试穿在 x≈13.351 m 停滞，未发现上装几何相交。
   改用 Bullet 静态 triangle BVH 后越过该处；地图三角形未改变。
4. 原 Featherstone 发布所有持久接触流形点，包含仍有间隙的接近点，且把
   Bullet 有符号距离直接作为穿透深度。x≈13.597 m 的 7.5 mm 报告因此误触发
   停车。现仅发布距离 ≤0 的点，并将其转换为正穿透深度。
   回归测试先以负穿透深度失败，修正后通过。

上述修改限定于包内 `SentryBulletFeatherstone`，使用固定版本及 SHA-256 源码，
不替换系统 Gazebo。源固定模型仍为 38 个基本体，惯量、关节限制与地形资源保持原值。
构建与源码下载要求见模型的 `FOLDING_SIMULATION.md`。

## 最终物理试穿

| 项目 | 实测 |
| --- | --- |
| 模式 | `robot_model:=real world:=rmuc_2026`，完整导航栈启动；本轮跳过导航 action |
| 入口 | `(12.27838, 13.41899)` m，底盘航向约 44.76° |
| 折叠过洞 | 大 Yaw 先归零，再下折；沿小云台临时前向请求 0.10 m/s |
| 出口到达 | `(14.35086, 13.42280)` m，`physical_passage.reached=true` |
| 非轮接触 / 关节故障 | `[]` / `[]` |
| 轮接触反馈 | 653640 个轮接触点，证实传感器工作；脚本未区分轮地与轮墙接触 |
| 恢复 | `restored=true`，最终 `target=normal, stage=normal, ready=true` |
| 平面定位误差 | 本轮物理阶段最大约 0.13599 m |
| 出口停车 | 未验收；出口位于上坡段，恢复后回滑至 x≈14.02 m，需另选停车点及验证驻停 |

脚本 `passed=true` 的范围为本次低速物理到达出口并恢复，不包含自主导航、
重复通过、出口驻停或真实机器可靠性。位置与航向控制采用 Gazebo 真值；
不能把实际穿越归因于定位达到毫米精度。模拟轮接触也不是实际全向滚子的滑移模型。

## 净空与导航包络

折叠静态包络：高度 236.98 mm，最大半径 408.15 mm，沿临时车头方向的宽度
606.99 mm。洞顶和地面具有约 1.5° 的共同坡度，局部垂直开口约 247.2 mm；
250 mm 不能直接当作整个地图网格处处精确的开口高度。

记录实际底盘四元数和关节角，使用凸包半空间与原始洞顶三角形投影范围进行
线性规划，检查 48 个位于洞顶下方的时刻。最小上装垂直几何间隙为 **6.75 mm**，
发生在 t≈24.96 s、大 Yaw 基座处。此时底盘 roll≈−1.108°、pitch≈1.345°。
该结果未扣除 1 mm 地形和 1 mm 凸包接触 margin，关节状态为最近一次反馈，
采样检查也不能排除采样间更小间隙。0.6 m 尺度增加 1° 相对倾斜即可产生约
10 mm 的高度变化；因此还需要姿态、地形和机械公差预算。

对原占用栅格矩形进行多边形 SAT 检查：

- 现有 0.44 m 圆包络加 20 mm padding，在全部 83 个路线采样点受阻。
- 折叠多边形加 20 mm padding，在 y=13.425 m 名义路线通过，但增加 30 mm
  定位余量后受阻。
- y=13.475 m 路线在额外 50 mm 定位余量下静态通过，80 mm 时失败。

以上只是静态平面地图分析，不能自动给正式导航放行。MINCO 优化器与安全检查
仍使用圆包络，0.34 m 高度门槛也尚未随姿态切换。后续必须验证完整折叠多边形、
路径航向、动态高度、定位误差和出口停车；不可只缩小圆半径来取得通过结果。

## 证据与检查

- [物理试穿结果](../../log/tunnel_clearance_review_20261024/contact_final_physical/result.json)
- [逐帧导航采样](../../log/tunnel_clearance_review_20261024/contact_final_physical/samples.jsonl)
- [实际底盘及关节姿态](../../log/tunnel_clearance_review_20261024/contact_final_truth.jsonl)
- [运动中洞顶几何净空](../../log/tunnel_clearance_review_20261024/contact_final_roof_clearance.json)
- [静态导航包络结果](../../log/tunnel_clearance_review_20261024/navigation_envelope.json)
- [约 1 mm 轮地间隙复核](../../log/tunnel_clearance_review_20261024/static_ground_gap.json)
- [恢复后状态](../../log/tunnel_clearance_review_20261024/contact_final_status.json)
- [恢复后画面](../../log/tunnel_clearance_review_20261024/contact_final_normal.png)
- [构建日志](../../log/tunnel_clearance_review_20261024/contact_reporting_build.log)
- [7 项 CTest 全部通过](../../log/tunnel_clearance_review_20261024/contact_reporting_test.log)

测试使用 `start_nav.sh --sim`、入口测试资源副本和接触传感器；正式默认出生点未改为洞口。
同一启动环境设 `ROS_DOMAIN_ID=178`、`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`、
`GZ_PARTITION=sentry_tunnel_contact_final_20261024`；测试命令为：

```bash
python3 scripts/check_sim_tunnel.py log/tunnel_clearance_rerun --physical-only \
  --model log/tunnel_clearance_review_20261024/refined_share/resource/models/sentry_real/model.sdf
```

此脚本依赖文档中的入口出生位姿与接触传感器配置，不能直接用于任意出生位置。
