# 仿真 Nav2Goal 跑飞修复记录

## 实测原因

1. MPC 的平移速度在 `camera_init` 坐标系中，Gazebo 底盘接收机体坐标系速度。原适配器直接转发，车体转向后速度方向错误。新增 TF 查询并复用 `adapt_command` 完成世界到机体的二维旋转；缺少 TF 时输出零速度。
2. 仿真 Nav2 使用 `base_link_fake`。该帧抵消真实车体 yaw，导致控制器无法得到真实航向反馈。仿真生成参数时统一将 `robot_base_frame` 设为 `base_link`。
3. 仿真接触冲击超过原 IMU 的 3 g 限幅，过滤后的加速度与真实运动不一致；Point-LIO 还会在接近 `satu_acc` 时跳过对应轴的 IMU 观测更新。连续目标下出现停车后定位持续漂移。6941 个原始 IMU 样本中有 8 个样本超过 29.43 m/s²，各轴绝对峰值为 37.39、52.56、98.48 m/s²。将仿真过滤器和 Point-LIO 的加速度范围同步为 1000 m/s²，保留仿真接触脉冲。此参数只修改仿真配置。
4. 构建前已有的规划器时钟修改使 `createPlan` 在没有绑定 lifecycle node 的测试中解引用空指针。补充节点有效性检查；实际运行时仍使用节点时钟。

## 修改边界

本次只修改下面 patch 所列代码。进入调试前已有的 `start_sim.sh`、启动文件中的 sentry1 参数加载/仿真时间/地图选择/fake_vel_transform 节点，以及规划器其余系统时间到仿真时间的改动均予以保留。下面 patch 相对于本次调试开始时的工作区生成，包含新增回归脚本的全部代码，可逐行审阅。

## 验证

实际通过 `./start_sim.sh headless:=true rviz:=false` 启动完整 Gazebo、Point-LIO、Nav2、MINCO 链路，没有启动真实硬件。

- 修复前：同一个开放目标 `(4.5, 6.0)` 出现持续旋转、偏离目标。
- 回归脚本：真实适配器在 yaw=90° 时将世界速度 `(1, 0)` 转为机体 `(0, -1)`，保持角速度 `0.4`；缺少 TF 和命令超时均输出零速度。旧回调对此断言失败，新代码通过。
- 仅修复坐标系后，前三个目标可以到达，但第四个目标后 Point-LIO 高度误差达到约 -3.4 m。只切换 IMU 输入模式仍然发散；同步放宽两个加速度量程后消失。
- 首轮四个目标 `(4.5, 6.0)`、`(5.0, 5.5)`、`(4.5, 6.5)`、`(4.3, 8.8)` 全部成功；动作结果 status=4，真值目标距离 <0.45 m，停车速度 <0.06 m/s，底盘命令为零。
- 随后 30 秒静止检查：真值最大漂移 0.000000235 m，Point-LIO 最大漂移 0.0128 m，无非零底盘命令。0.45 m 是此测试容差，包含 Nav2 目标容差和定位偏差，不表示厘米级绝对定位精度。
- `colcon build --symlink-install --packages-select minco_planner sentry_simulation --executor sequential` 通过；存在依赖头文件未使用函数等编译警告。
- 全工作区 `colcon test --event-handlers console_cohesion+` 完成（18 个包）；`colcon test-result --verbose` 汇总 21 tests、0 errors、0 failures，但报告了旧构建目录 `build/sdformat_tools/package.xml` 失效链接的跳过警告。
- `colcon test --packages-select minco_planner sentry_simulation --event-handlers console_cohesion+` 通过：规划器 7 个 gtest 全通过；仿真包当前未注册 CTest，新增适配器检查单独执行。

复现适配器检查：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=93 python3 src/nav-simulation/sentry_simulation/scripts/check_command_frame.py
```

本次验证限定为默认 rmuc_2026 场景及上述目标序列；未对其他底盘和场景进行覆盖。

第二次全新启动后，同样四个目标全部成功。第四个目标后的首个 30 秒窗口中，Gazebo 真值继续缓慢移动了 0.178 m，未通过严格的 0.1 m 静止阈值；这是仿真底盘的低速滑移，不能将动作成功等同于完全静止。稳定后的第二个 30 秒窗口通过：真值最大漂移 0.000000253 m、Point-LIO 最大漂移 0.0165 m，无非零底盘命令。两轮均未再出现原来的持续旋转、偏离目标或停车后定位数米级发散。

## 完整修改代码

```diff
--- a/src/nav-simulation/sentry_simulation/launch/simulation.launch.py
+++ b/src/nav-simulation/sentry_simulation/launch/simulation.launch.py
@@ -104,6 +104,8 @@
             for key, child in value.items():
                 if key == "ros__parameters":
                     child["use_sim_time"] = True
+                if key == "robot_base_frame":
+                    value[key] = "base_link"
                 enable_sim_time(child)
 
     enable_sim_time(resolved_params)
@@ -164,7 +166,8 @@
             "use_sim_time": True,
             "input_topic": "/sim/imu_raw",
             "output_topic": "/sim/imu",
-            "acceleration_limit": 29.43,
+            # Preserve Gazebo contact impulses; match Point-LIO satu_acc.
+            "acceleration_limit": 1000.0,
             "angular_velocity_limit": 35.0,
         }],
     )
@@ -369,7 +372,8 @@
             executable="sentry_sim_cmd_adapter",
             name="sentry_sim_cmd_adapter",
             output="screen",
-            parameters=[{"use_sim_time": True, "chassis_type": vehicle_model}],
+            parameters=[{"use_sim_time": True, "chassis_type": vehicle_model,
+                         "input_frame": "camera_init"}],
         ),
         rviz,
     ]
--- a/src/nav-simulation/sentry_simulation/sentry_simulation/cmd_vel_adapter.py
+++ b/src/nav-simulation/sentry_simulation/sentry_simulation/cmd_vel_adapter.py
@@ -1,13 +1,15 @@
 #!/usr/bin/env python3
-"""Forward the controller's body-frame command to the simulator with a watchdog."""
+"""Transform controller velocity into the chassis frame, with a watchdog."""
 
+import math
 from geometry_msgs.msg import Twist, TwistStamped
 import rclpy
 from rclpy._rclpy_pybind11 import RCLError
 from rclpy.executors import ExternalShutdownException
 from rclpy.node import Node
+from tf2_ros import Buffer, TransformException, TransformListener
 
-from sentry_simulation.kinematics import command_timed_out
+from sentry_simulation.kinematics import adapt_command, command_timed_out
 
 
 class SentrySimCmdAdapter(Node):
@@ -17,6 +19,10 @@
         self.declare_parameter("input_topic", "/cmd_vel_mpc")
         self.declare_parameter("output_topic", "/sim/cmd_vel")
         self.declare_parameter("command_timeout", 0.25)
+        self.declare_parameter("input_frame", "base_link")
+        self._input_frame = str(self.get_parameter("input_frame").value)
+        self._tf_buffer = Buffer()
+        self._tf_listener = TransformListener(self._tf_buffer, self)
 
         chassis_type = str(self.get_parameter("chassis_type").value)
         if chassis_type not in ("omni", "ackermann", "swerve"):
@@ -41,6 +47,22 @@
         self.create_timer(0.05, self._watchdog_callback)
 
     def _command_callback(self, message: Twist) -> None:
+        if self._input_frame != "base_link":
+            try:
+                transform = self._tf_buffer.lookup_transform(
+                    self._input_frame, "base_link", rclpy.time.Time()
+                )
+            except TransformException:
+                self._publish_command(Twist())
+                return
+            q = transform.transform.rotation
+            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
+                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
+            command = Twist()
+            command.linear.x, command.linear.y, command.angular.z = adapt_command(
+                "omni", message.linear.x, message.linear.y, message.angular.z, yaw
+            )
+            message = command
         self._publish_command(message)
         self._last_command_time = self._now_seconds()
         self._sent_timeout_stop = False
--- a/src/nav-simulation/sentry_simulation/config/point_lio_sim.yaml
+++ b/src/nav-simulation/sentry_simulation/config/point_lio_sim.yaml
@@ -44,7 +44,8 @@
       extrinsic_est_en: false
       imu_time_inte: 0.005
       lidar_time_inte: 0.1
-      satu_acc: 29.43
+      # Match the IMU filter range: Gazebo contact impulses can exceed 3 g.
+      satu_acc: 1000.0
       satu_gyro: 35.0
       acc_norm: 9.81
       lidar_meas_cov: 0.01
--- a/src/navigation/minco_planner/src/minco_core/minco_planner.cpp
+++ b/src/navigation/minco_planner/src/minco_core/minco_planner.cpp
@@ -864,7 +864,9 @@
   // Nav2 interface: createPlan() only sets the goal flag for MincoFSM.
   // It must NOT run A* or optimization here.
   nav_msgs::msg::Path path;
-  path.header.stamp = node_.lock()->get_clock()->now();
+  if (const auto node = node_.lock()) {
+    path.header.stamp = node->get_clock()->now();
+  }
   path.header.frame_id = output_frame_;
 
   geometry_msgs::msg::PoseStamped normalized_start;
--- /dev/null
+++ b/src/nav-simulation/sentry_simulation/scripts/check_command_frame.py
@@ -0,0 +1,59 @@
+#!/usr/bin/env python3
+"""Exercise the real adapter callback: rotation, missing TF, and watchdog stop."""
+import math
+import time
+
+from geometry_msgs.msg import TransformStamped, Twist
+import rclpy
+
+from sentry_simulation.cmd_vel_adapter import SentrySimCmdAdapter
+
+
+def main():
+    rclpy.init(args=["--ros-args", "-p", "input_frame:=test_world"])
+    adapter = SentrySimCmdAdapter()
+    received = []
+    subscriber = adapter.create_subscription(Twist, "/sim/cmd_vel", received.append, 10)
+    try:
+        transform = TransformStamped()
+        transform.header.frame_id = "test_world"
+        transform.child_frame_id = "base_link"
+        transform.transform.rotation.z = math.sin(math.pi / 4)
+        transform.transform.rotation.w = math.cos(math.pi / 4)
+        adapter._tf_buffer.set_transform_static(transform, "test")
+        command = Twist()
+        command.linear.x = 1.0
+        command.angular.z = 0.4
+        # Wait for local DDS discovery before exercising the callback.
+        deadline = time.monotonic() + 5.0
+        while adapter._publisher.get_subscription_count() == 0:
+            assert time.monotonic() < deadline, "subscriber discovery timed out"
+            rclpy.spin_once(adapter, timeout_sec=0.05)
+
+        def check(expected):
+            deadline = time.monotonic() + 2.0
+            while not received:
+                assert time.monotonic() < deadline, "command was not published"
+                rclpy.spin_once(adapter, timeout_sec=0.05)
+            actual = received.pop(0)
+            assert all(abs(a - b) < 1e-6 for a, b in zip(
+                (actual.linear.x, actual.linear.y, actual.angular.z), expected
+            )), (actual, expected)
+
+        adapter._command_callback(command)
+        check((0.0, -1.0, 0.4))
+        adapter._input_frame = "missing_frame"
+        adapter._command_callback(command)
+        check((0.0, 0.0, 0.0))
+        adapter._last_command_time = adapter._now_seconds() - 1.0
+        adapter._watchdog_callback()
+        check((0.0, 0.0, 0.0))
+        print("PASS: world-to-body rotation, missing-TF stop, command-timeout stop")
+    finally:
+        adapter.destroy_subscription(subscriber)
+        adapter.destroy_node()
+        rclpy.shutdown()
+
+
+if __name__ == "__main__":
+    main()
```
