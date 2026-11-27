# 大 Yaw 雷达 TF 与底盘状态工单

2026-11-27。用户已确认设计和六张工单拆分，已发布到 GitHub Issues。GitHub 是状态与讨论的权威来源，本目录保留发布时的工单内容和执行依赖。

参考工程雷达固定在底盘，当前 MID360 固定在大 Yaw 转动端。恢复实体底盘状态供 MINCO、MPC 和指令转换一致使用，保留导航虚拟参考系；第一阶段固定其他会改变雷达安装的机构，允许完善上下位机反馈，沿用现有超时处理，不新增冗余保护。

| 工单 | 标题与本地正文 | 直接阻塞项 | 分诊标签 |
| --- | --- | --- | --- |
| [#6](https://github.com/Wanqiq7/Pldx_ws/issues/6) | [贯通带测量时间的大 Yaw 关节角与转速反馈](issues/01-joint-feedback.md) | 无 | `ready-for-agent` |
| [#7](https://github.com/Wanqiq7/Pldx_ws/issues/7) | [从大 Yaw 雷达运动解析恢复实体底盘状态](issues/02-chassis-state.md) | [#6](https://github.com/Wanqiq7/Pldx_ws/issues/6) | `ready-for-agent` |
| [#8](https://github.com/Wanqiq7/Pldx_ws/issues/8) | [让 MINCO、MPC 与 Nav2 使用统一底盘状态](issues/03-navigation-consumers.md) | [#7](https://github.com/Wanqiq7/Pldx_ws/issues/7) | `ready-for-agent` |
| [#9](https://github.com/Wanqiq7/Pldx_ws/issues/9) | [使用实体底盘朝向转换下位机平移指令](issues/04-command-conversion.md) | [#7](https://github.com/Wanqiq7/Pldx_ws/issues/7) | `ready-for-agent` |
| [#10](https://github.com/Wanqiq7/Pldx_ws/issues/10) | [启用物理 TF 链并完成整栈四工况回放验收](issues/05-stack-integration.md) | [#8](https://github.com/Wanqiq7/Pldx_ws/issues/8)、[#9](https://github.com/Wanqiq7/Pldx_ws/issues/9) | `ready-for-agent` |
| [#11](https://github.com/Wanqiq7/Pldx_ws/issues/11) | [完成大 Yaw 台架标定与实体底盘四工况验收](issues/06-hardware-acceptance.md) | [#10](https://github.com/Wanqiq7/Pldx_ws/issues/10) | `ready-for-human` |

```mermaid
flowchart LR
    A["#6 关节反馈"] --> B["#7 底盘状态"]
    B --> C["#8 导航消费者"]
    B --> D["#9 指令转换"]
    C --> E["#10 整栈验收"]
    D --> E
    E --> F["#11 实机验收"]
```

执行时仅领取阻塞项全部完成的工单。当前起点为 #6；#8 与 #9 在 #7 完成后可并行。#11 需要真实机构操作人员完成标定和验收，其余软件工单可由代理执行。

所有后续实现提交和推送使用 `develop`。开始本批工作前的已提交导航基线固定在 `9b284c61`，已通过 [PR #5](https://github.com/Wanqiq7/Pldx_ws/pull/5) 提请进入 `master`；本批文档和未来实现不追加到该固定 PR head。

设计依据：[动态雷达讨论与几何推导](../../tf_dynamic_lidar_discussion.md)、[ADR-0003](../../adr/0003-physical-chassis-state-for-yaw-mounted-lidar.md)、[领域术语](../../../GLOSSARY.md)。本次只发布工单并记录设计，运行代码实现由各工单推进。
