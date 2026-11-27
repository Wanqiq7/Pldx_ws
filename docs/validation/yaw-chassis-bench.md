# 大 Yaw 实机验收记录（#11）

当前状态：**待操作人员实测**。CAD 初值、离线回放、程序构建均不构成实机通过。
软件验收见 [chassis_state](../../src/navigation/chassis_state/README.md)。

## 开始前记录

保存机器人编号、电机型号/CAN ID、Gimbal/NavLink/ROS2_LIBXR/导航提交、实际传动比、
电机 reverse、yaw_zero、其他机构锁定位置及照片。`vision_gimbal` 是小云台；第一阶段
只随大 Yaw 运动，小云台/升降的其他自由度锁定。当前小云台 CAD 原点是
`small_pitch`，在 `lidar_yaw_mount` 中为 `[0.17282073,-0.042,0.092]` m，零位旋转单位阵。
这不是相机 optical frame 标定。当前实机几何文件中的 IMU 位置和旋转来自 CAD，必须
与实际安装复核。

先启动传感器、反馈和 chassis_state 进行静止记录，由操作员按现场流程控制真实机构。
下面的记录器只订阅，不发布运动指令：

```bash
source /opt/ros/jazzy/setup.bash
source install-link/setup.bash
python3 scripts/record_chassis_bench.py --case stationary_baseline \
  --operator NAME --duration 60 --geometry src/navigation/chassis_state/config/cad_initial.yaml \
  --output /tmp/chassis-baseline
python3 scripts/analyze_chassis_bench.py /tmp/chassis-baseline
```

每次记录使用新目录。记录器保存实际样本、测量时间、接收时间、TF 发布源 GID、参数副本
及 manifest。补全 manifest 中的硬件/固定机构/方向/传动/真值来源/容差依据字段。
额外保留原始 LiDAR/IMU 与通信抓包以便重新处理，可在独立终端录制：

```bash
ros2 bag record /tf /tf_static /aft_mapped_to_init /chassis/odom \
  /chassis/pose_difference /sentry/yaw_joint_state /cmd_vel_mpc \
  /sentry/behaivor_send /livox/lidar /livox/imu
```

## 标定和验收顺序

1. **静止基线**：所有机构固定，至少 60 s。记录底盘位置/姿态漂移与速度分布，结合
   地面标记、量角器/外部测量或已对齐 IMU 的精度确定容差。不能用合成测试的 1e-7
   作为硬件容差。底盘 IMU 未经零位对齐只能比较相对旋转。
2. **零位与方向**：机械定义零位后读回角度；分次正/负方向转动已知角度，确认 rad、
   rad/s、reverse 与真实传动关系。校正电机零位在固件处理；剩余安装偏角用
   `joint_zero`，避免重复扣零。杆臂由底盘参考点→转轴、转轴→IMU 分别测量。
3. **底盘静止扫转**（`stationary_sweep`）：覆盖正负角度与 ±pi 边界。原始 LIO 应观察
   雷达运动；恢复底盘的位姿/两种速度应在步骤1容差内。
4. **关节锁定底盘旋转**（`locked_joint_rotation`）：保留真实底盘角速度；解析速度与
   以测量时间求得的位姿差分比较，注明噪声、滤波和区间中点的影响。
5. **两者同时运动**（`simultaneous_rotation`）：在独立真值下重复步骤3/4，检验分离。
6. **固定世界方向指令**（`world_translation`）：由操作员按现场流程执行；保留实际
   USB 发送/下位机接收数据，验证 chassis-frame XY 经底盘朝向转回 `camera_init`
   仍沿预期世界方向。`/cmd_vel_mpc` 本身只能证明发送前控制意图，不能证明实际线缆
   数据。测试 capture 节点用于离线回放，不能与真实通信桥同时运行冒充实测数据。
7. 每工况确认关键 TF 每个 child 只有一个 publisher，重建的 `camera_init → body`
   与原始 LIO 一致，`camera_init → base_link_fake` 朝向单位阵；有 map 定位修正时
   不要求 fake 在 map 中零 yaw。重复隧道准备减速/逃逸/小陀螺既有策略验收。

时间统计中的“接收减测量”包含传输延迟与时钟映射偏差；只有同步参考或独立注入真值
才能测出绝对对齐误差。分别记录均值、抖动、极值与测量方法，不把接收时间当采样时间。

## 报告

每工况附：manifest、geometry、samples.jsonl、summary.json、bag/抓包路径、独立真值、
容差及其依据、实际最大误差、通过/失败、操作员与日期。自动分析始终标为
`requires_operator_review`。发现偏差先按证据修正标定或创建具体问题。未完成真实测量
前保持 #11 开放；#10 的视觉挂接仅验证锁定其他机构时的 CAD 几何。
