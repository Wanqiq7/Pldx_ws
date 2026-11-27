# Sentry Navigation Control

Terms for navigation arrival, chassis yaw arbitration, and tunnel passage preparation.

## Language

**位置到达**:
The robot is within the 0.3 m XY tolerance and its planar translational speed is at or below 0.3 m/s. Final goal yaw and zero angular velocity are not required for normal navigation success.

**普通导航角速度**:
The chassis yaw rate requested by normal navigation, used when no higher-priority strategy owns chassis yaw.

**小陀螺模式**:
A strategy-controlled chassis rotation that may continue after the navigation task succeeds; the chassis controller determines the physical rotation rate.

**隧道对齐**:
Preparing the chassis orientation for passage through a tunnel, with priority over small-gyro rotation and normal navigation yaw control.

**tunnel_ready**:
The tunnel-entry readiness state indicating that both chassis alignment and the required lifter posture are ready for passage.

**隧道准备减速**:
Reducing both planar velocity components to 15% while tunnel preparation is active and passage is not ready. This includes escape motion and preserves its direction.

## Yaw Feedback

**大 Yaw 关节角**:
大 Yaw 转动端相对实体底盘的转角，以电机标定零位为参考。
_Avoid_: 大 Yaw 世界航向、底盘 IMU 航向

**大 Yaw 关节角速度**:
大 Yaw 转动端相对实体底盘的转角变化率。
_Avoid_: 雷达对世界的角速度、底盘角速度

**底盘参考点**:
导航跟踪的实体底盘上的固定点，是底盘位置与平移速度的共同参考点。该点与大 Yaw 转轴的位置关系由安装标定确定。

**导航虚拟参考系**:
原点跟随底盘参考点、平面朝向在局部里程计参考系中保持固定的导航坐标系。
