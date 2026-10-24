# MPC 固定矩阵与求解器复用

日期：2026-10-19。修改范围为 `minco_controller` 求解器、包级测试及文档。

## 实现

离散模型仍为 `A=I, B=dt·I`。配置时构建下三角预测矩阵、速度边界和加速度差分约束，并预分配加权预测矩阵、误差、Hessian、梯度和解缓冲。删除单位矩阵幂循环、稠密 Q/R 块矩阵及每周期 Eigen 到 qpOASES 数组的转换。

每周期根据各参考点航向重新计算 Q 对应的 Hessian 和梯度，保留原代价公式、全局系速度及加速度约束。使用持久的 `SQProblem`，通过其支持矩阵变化的 `hotstart()` 更新 QP。Hessian 使用两份缓冲交替构建，确保 qpOASES 浅引用的上一份矩阵在切换期间仍有效。

首次求解冷启动。热启动失败时重置并尝试一次冷启动；该故障路径最多执行两次各自受 `max_working_set_recalculations` 限制的尝试。两次均失败、解获取失败或解非有限时返回 false，不更新输出及上一成功控制。`setConfig()` 重建缓存和求解器，保持上一成功控制以延续既有加速度边界语义。

未更改 ROS 话题、坐标系、控制模型或硬件配置。复用的是应用层矩阵与求解器对象；qpOASES 内部仍可能分配临时存储，不声称整条求解链零分配。

## 验证

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select minco_controller --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select minco_controller --event-handlers console_cohesion+
colcon test-result --test-result-base build/minco_controller --verbose
build/minco_controller/mpc_solver_test --gtest_filter=MpcSolver.DISABLED_Benchmark --gtest_also_run_disabled_tests
```

编译及五项回归测试通过。测试用独立稠密批量公式、逐次冷启动作为对照，覆盖连续变航向及活跃约束、全预测时域输出、配置切换、非法参考、不可行 QP、迭代耗尽，以及热启动失败后不改配置恢复。控制与预测轨迹容差为 2e-6，另验证速度及加速度边界。微基准默认不参与 CTest，通过上述命令显式运行，10/20 步各 2000 个统计样本均无失败。

另外将本次修改前保存的原始 `mpc_solver.cpp/.hpp` 与新实现分别编译后，在同一进程对相同参考连续求解，均生成预测轨迹。Intel Core i7-14700HX，GCC，C++17，`-O3 -DNDEBUG`，使用仓库内同一 qpOASES 静态库。每种预测步数先预热 200 次，再统计 5000 次，开启加速度约束：

| 预测步数 | 原实现 p50/p95/p99（μs） | 新实现 p50/p95/p99（μs） | 原/新失败次数 | 最大控制分量差 |
| --- | --- | --- | --- | --- |
| 10 | 150.773 / 223.714 / 248.214 | 39.617 / 52.204 / 60.530 | 0 / 0 | 1.06218e-11 |
| 20 | 731.813 / 1097.150 / 1203.960 | 224.168 / 279.211 / 303.236 | 0 / 0 | 1.14325e-11 |

本次样本中，中位耗时分别下降约 74% 和 69%。原实现直接对比程序与源码备份位于本机临时目录 `/tmp/minco_mpc_reuse_before/`，不随仓库持久保存；仓库内保留独立公式回归与微基准用于后续复测。

这些结果是主机上的求解器合成基准，包含 QP 组装、求解和预测输出，不包含参考轨迹构建、ROS 调度、TF、通信或物理底盘延迟。未运行整栈仿真或实车，不将微基准视为实车控制周期或导航效果的验收。
