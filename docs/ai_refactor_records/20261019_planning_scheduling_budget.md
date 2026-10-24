# MINCO 调度预算、取消与输入失效保护

## 问题与参考

原先 20 Hz FSM、20 Hz 安全检查和执行启停订阅共用默认互斥回调组，FSM 同步进行搜索与优化，导致安全和停止处理等待计算结束。`activate()` / `deactivate()` 未管理执行状态，`createPlan()` 忽略取消检查，控制端缺少输入过期保护。

参考 `rose_navigation/src/planner/rose_planner.cpp` 的独立规划/控制线程及 A* 搜索超时方案，当前插件增加专用控制 executor，并将搜索、优化放入单个在途工作任务。保持启停、FSM 与结果提交的串行一致性。

## 执行与失效规则

- FSM 派发/轮询计算，工作线程负责搜索、初值、MINCO、偏航、校验和指令采样；这些步骤共享从派发时刻开始计算的 `steady_clock` 截止时间。
- 搜索扩展、路径回溯/处理、MINCO 目标函数与积分采样、偏航及轨迹采样检查预算和版本。A* 原来每 1000 个外层循环才检查取消，现在扩展前即检查。
- 目标更新、执行切换、停用和参考撤销递增规划版本。提交只接受当前版本；计算完成后排队超过 100 ms 的候选被丢弃。单条采样指令最多 4096 个点。
- 控制线程只用短锁取得轨迹快照。热启动判定和诊断在快照上执行，停止撤销不等待工作线程复制全局路径。
- 安全定时器检查剩余轨迹并包含终点；碰撞或检查超预算立即发布 `BLOCK_COMMAND`。FSM 包括恢复查询也有预算，耗尽后撤销参考并重新排入当前目标。
- `deactivate()` 取消定时器、停止执行、取消目标及在途版本并撤销参考；`cleanup()` 等工作任务退出后释放资源。重新 activate 后等待新的执行启用消息。
- 控制端同时检查消息时间戳和稳态接收间隔；任一参考/里程计缺失、超时或位于未来，输出全零速度。最终输出与取消/参考替换串行，并检查本轮执行版本，阻止旧计算在重启后输出。

## 常驻规划 worker

规划器配置完成后只创建一个常驻线程。FSM 调用 `planAsync()` 时提交包含位姿、目标、全局搜索标志、版本和截止时间的请求；worker 通过条件变量取请求并执行搜索、MINCO 和采样。请求槽位只保留最新待处理请求，结果槽位由 FSM 线程以非阻塞方式领取。

候选轨迹、结果状态和计算版本由规划互斥量同步，避免原先依赖 `future.get()` 建立的隐式同步。停用只递增版本、清空待处理请求并唤醒取消检查，不等待在途规划；清理阶段才停止并 join worker，然后释放地图和优化器。

## 默认配置与硬件影响

主参数文件：`src/navigation/navi2_bringup/params/sentry1.yaml`。

| 参数 | 默认值 |
|---|---:|
| `MincoPlanner.scheduling.planning_budget_sec` | 0.025 s |
| `MincoPlanner.scheduling.control_budget_sec` | 0.005 s |
| `MincoPlanner.scheduling.safety_budget_sec` | 0.005 s |
| `FollowPath.reference_timeout_sec` | 2.0 s |
| `FollowPath.odom_timeout_sec` | 0.3 s |

FSM 与安全检查周期均为 50 ms。安全行为变化：检查超预算及输入失效会停止输出，包括固定小陀螺角速度。规划计算超预算会丢弃结果，只有尚未过期且安全的已提交轨迹可继续跟踪。实机需要依据地图查询耗时、规划负载及里程计频率调整预算。

预算采用协作式检查，单次地图查询、内存分配、线性代数和 DDS 调用不能被强制抢占；普通 Linux 的线程调度也没有硬实时上界。因此不能把 5 ms 配置值或本机测量值当成实机最坏响应时间承诺。控制端过期检查在 Nav2 控制周期内执行。本次验证未启动实机硬件。

## 验证

修复前运行新增回归用例，确认停用后仍可消费目标、已取消请求仍被接受，两个用例均失败。修复后通过。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --symlink-install \
    --packages-select minco_planner minco_controller --executor sequential
ROS_DOMAIN_ID=217 colcon test --event-handlers console_cohesion+ --executor sequential
colcon test-result --verbose
```

两包构建成功。全工作区 18 个包完成测试；`colcon test-result` 汇总为 46 tests、0 errors、0 failures、1 skipped，跳过项为默认关闭的 MPC 基准。扫描结果时另有已有 `build/sdformat_tools/package.xml` 断链提示；不属于测试失败。

新增覆盖：版本匹配提交、停用/重启与目标更新丢弃旧候选、提交等待超时、停用不等待长计算、安全检查超预算撤销、A* 即时取消、真实 MINCO 积分采样预算/取消、参考与里程计的时间戳及接收超时、未来时间戳、控制计算途中重启。

专用 executor 集成用例不运行父节点 executor，在规划任务持续运行时确认安全查询执行，然后通过 ROS 发布停止消息；本轮测得停止处理 1.143 ms，并确认在途任务取消。该数值仅是本机本轮测试观测值。测试结果保存在 `build/minco_planner/test_results/minco_planner/test_planning_budget.gtest.xml`。

### 本机 P95/P99 校准（2026-10-19）

使用当前配置的 MINCO 参数，预热 5--10 次后各采样 100 次。全局搜索使用 200×200、0.05 m 分辨率地图和 20,20 到 180,180 的端点；A* 为自由地图，SMAC 使用带确定性墙体和缺口的地图并包含 ESDF 查询；局部优化使用 9 个路径点；安全检查复用已提交轨迹。结果如下：

| 阶段 | P95 | P99 | 最大值 |
| --- | ---: | ---: | ---: |
| A* 全局搜索（含 ESDF 查询） | 2.47 ms | 2.52 ms | 2.56 ms |
| SMAC 全局搜索（含 ESDF 查询） | 0.042 ms | 0.045 ms | 0.046 ms |
| MINCO 局部优化 | 0.35 ms | 0.39 ms | 0.41 ms |
| 安全检查 | 0.0027 ms | 0.0027 ms | 0.0028 ms |

统一规划预算据此设为 25 ms：约为最慢 A* P99 的 10 倍，用于覆盖局部路径处理、偏航/采样、线程唤醒和地图复杂度变化。该值是本机协作式预算，不是硬实时上界；部署到目标机器后应重新采样并关注超时率。

可重复运行基准：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=217 build/minco_planner/test_planning_budget \
  --gtest_filter='PlanningTimingBenchmark.*:PlanningBudgetTest.SafetyCheckP95P99'
```
