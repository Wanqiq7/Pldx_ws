# #10 — 启用物理 TF 链并完成整栈四工况回放验收

**GitHub:** https://github.com/Wanqiq7/Pldx_ws/issues/10

**Label:** ready-for-agent

## What to build

将已验证的底盘状态转换、导航消费者和通信出口接入当前工程正式启动链，统一启用物理 TF：`camera_init → base_link → lidar_yaw_mount → body → livox_frame`，保留 `base_link → base_link_fake`。从启动、定位、规划控制到实际出站指令完整使用一致的底盘状态，并提供可在实车前重复运行的验收入口。

这是整栈切换点：此前工单可在隔离验证配置下完成，各部分就绪后本工单一次性切换运行链，避免新 TF 与旧状态消费者混搭。

## Acceptance criteria

- [ ] Point-LIO 继续发布原始雷达里程计和原有点云，关闭其 `camera_init → body` TF 广播；统一转换节点发布实体底盘位姿，安装链与动态关节变换各有唯一发布者。
- [ ] 移除正式运行中冲突的静态 `body → base_link` 关系；完整启动后无重复 child、环路或断开的关键链路，重建得到的雷达世界位姿与原始 LIO 一致。
- [ ] `base_link_fake` 原点跟随底盘参考点，在 `camera_init` 中平面 yaw 为零；有 `map` 定位 yaw 修正时不把它误认为在 map 中固定。
- [ ] MINCO、MPC、Nav2 速度检查及通信桥正式使用底盘状态；仿真/回放命令适配器遵守相同状态和命令坐标语义，正常默认启动不再依赖旧雷达杆臂补偿。
- [ ] 核对现有视觉 frame 与大 Yaw 转动端的机械关系，保持视觉名称/接口并正确挂接其已建模运动；第一阶段不额外实现未纳入范围的升降或小云台自由度。零位 CAD 验证入口明确限定其他关节锁定。
- [ ] 提供四工况自动回放及结果汇总：底盘静止关节转动、关节锁定底盘旋转、两者同时运动、固定世界方向平移指令；同时检查位姿、解析速度、差分对照、虚拟朝向、唯一 TF 发布者和出站速度方向。
- [ ] 准备台架标定/数据记录入口及实机验收步骤，记录零位、方向、安装几何、测量时间对齐误差与静止噪声；给工单 06 提供明确的输入和报告格式。
- [ ] 所有改动包完成构建及适用的跨包验证；保持既有隧道、逃逸、自旋策略与超时处理，不新增保护状态机。

## Blocked by

- https://github.com/Wanqiq7/Pldx_ws/issues/8
- https://github.com/Wanqiq7/Pldx_ws/issues/9
