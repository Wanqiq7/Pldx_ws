# 恢复原工程的隧道准备与逃逸平移行为

日期：2026-11-15。

## 问题与约定

原工程在通信出口先用逃逸 XY 替换导航 XY，再在 `tunnel_prepare_active && !tunnel_ready` 时将两个分量乘以 0.15。迁移到 LibXR 后，行为树仍计算并发布这些字段，但桥接出口未消费它们，实际底盘命令缺少减速与逃逸替换。导航进度检查也从原工程 SimpleProgressChecker / 0.1 m / 1000 秒变为 PoseProgressChecker / 0.1 m / 0.1 rad / 10 秒。

用户确认沿用原工程的隧道逻辑；这取代此前讨论的“未就绪时清零 XY”。本次实现不回退独立的导航到达契约，不修改固件。

## 实现

- 保留行为树准备、机构就绪、航向对齐和逃逸动作源码。
- 通信桥保留 ROS 行为中的准备、就绪和逃逸 XY；在每个发送周期从原始导航输入生成选定命令，先替换逃逸 XY，再乘以 0.15，最后执行现有世界到车体转换。不会在缓存中累乘。
- 原始导航输入必须先通过现有时效、有限值、里程计和线协议范围检查；行为替换后再次验证选定命令，避免无效导航流被逃逸覆盖重新放行。
- 角速度、对齐目标、角度单位转换、固件小陀螺模式和线协议 ABI 保持原契约。逃逸角速度字段仍不参与替换。
- 主导航参数恢复 SimpleProgressChecker、0.1 m 和 1000 秒。1000 秒影响所有使用该参数文件的导航任务：位置停滞由该检查器识别的时间变长；输入失效、取消和确定性规划失败仍走原有处理。
- 通信节点可注入仅接收最终协议数据的内存发送端，测试使用真实 ROS 订阅与发送周期，不打开 USB。默认构造路径继续使用原有 USB 传输。

## 验证边界

- `bt_manager` 集成测试通过真实里程计与路线输入、机构/对齐反馈、准备条件和逃逸动作，观察真实 ROS Behavior 发布结果。
- `ros2_libxr` 集成测试通过真实 ROS 速度、Behavior 和里程计输入，观察最终 ChassisTarget / BehaviorData。准备与逃逸均覆盖准备/就绪四种组合、连续发送、角速度保持、非零航向转换和恢复导航。
- 过期导航、过期里程计、非有限值、原导航越界和无效逃逸输入不能生成有效非零指令。
- 导航进度测试加载主参数文件中的实际插件，用可控 ROS 时钟验证超过 10 秒但未满 1000 秒继续有效、旋转不重置位置进度、超时失败以及位置进展和 reset 重置基线。
- 两个通信测试进程分别使用 ROS domain 228 和 229，避免并行执行时全局话题相互干扰。

## 命令与结果

所有测试均在隔离 ROS domain 中执行，不启动实车或烧录。

```bash
source /opt/ros/jazzy/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build \
    --build-base build-link --install-base install-link --symlink-install \
    --packages-select ros2_libxr bt_manager navi2 --parallel-workers 1 \
    --cmake-args -DBUILD_TESTING=ON
source install-link/setup.bash
ROS_DOMAIN_ID=227 colcon test \
    --build-base build-link --install-base install-link \
    --event-handlers console_cohesion+
colcon test-result --test-result-base build-link --verbose
```

TDD 证据：首次准备减速测试在期望 0.15/-0.06 时失败；实现后通过。首次逃逸测试在期望逃逸 XY 时失败；恢复替换后通过。进度测试首次加载到 PoseProgressChecker / 10 秒并失败，恢复原配置后通过。

仿真派生检查：仅调用现有仿真 `_launch_setup` 生成动作与临时参数，不执行返回动作、不启动 Gazebo。对 `omni` 和 `real` 两种模型，实际生成的参数均为 SimpleProgressChecker / 0.1 m / 1000 秒；临时文件检查后删除。仿真子模块的已有改动未修改。

首次工作区全量运行：18 包完成，3 包因当前 install-link 下缺少安装产物无法启动；已产出的结果为 119 项、0 错误、0 失败、1 跳过。随后补构建缺少产物的 dual_lidar_calibration、msg_convert 和 dbscan_cluster，再完成最终全量验证。最终全量结果：21 个包完成，131 项测试，0 错误，0 失败，1 项跳过。

## 限制

验证在 Behavior 边界组合上下游的真实 ROS 集成测试，没有连接物理底盘；上位机发送对齐请求不能证明 MC02 已实现航向闭环。本次不新增行为心跳失效策略，行为状态沿用原工程的“保存至下一条消息”语义。没有改变地图、TF、标定或硬件 IP。

## 审查与提交

按代码审查技能分别检查仓库规范与规格符合性。已修正新增内部标识符的命名，并补齐真实 Behavior 发布链、逃逸准备/就绪四组合和持续发送验证；最终没有未处理的实质性审查发现。C++ clang-format 和 git diff --check 通过。

通信桥子模块提交：a4cc5e3（develop）。主仓库提交包含该子模块引用、导航进度参数、相关包测试与本记录。未推送远端，保留原有 README、仿真子模块与 start_sim.sh 本地改动。
