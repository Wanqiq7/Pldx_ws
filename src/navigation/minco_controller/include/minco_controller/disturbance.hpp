#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace minco_controller {

// 坡面上重力沿地面切向的分量在 map 水平面的投影（m/s²）。
// q_world_body 为 odom 姿态；body_roll_offset 为 body 相对底盘的固定安装横滚
// （与旧 applyGravityCompensation 的 true_roll = roll - offset 约定一致）。
// 底盘法向 n = R·Rx(-offset)·ez，切向重力 g_t = -g·(ez - (ez·n)·n)，取其 xy 分量。
Eigen::Vector2d slopeGravity(
  const Eigen::Quaterniond & q_world_body, double body_roll_offset, double g = 9.81);

// Global-frame force-input disturbance observer: v_dot = a_cmd + d.
// Velocity residuals correct the velocity/disturbance estimates without differentiating noise.
struct DisturbanceObserverConfig
{
  double bandwidth{4.0};     // rad/s，越大跟随越快、越受速度噪声影响
  double max_estimate{1.0};  // m/s²，每轴截断
  double max_gap{0.2};       // s，样本间隔超过此值视为数据中断并重置
};

class DisturbanceObserver
{
public:
  explicit DisturbanceObserver(const DisturbanceObserverConfig & config = {}) : config_(config) {}

  void setConfig(const DisturbanceObserverConfig & config)
  {
    config_ = config;
    reset();
  }
  const DisturbanceObserverConfig & config() const { return config_; }
  void reset() { initialized_ = false; }

  // @param stamp 速度测量时间戳（s）
  // @param v_meas 测得的 map 系平移速度
  // @param a_cmd drive acceleration applied since the previous sample (map frame, m/s²)
  void update(double stamp, const Eigen::Vector2d & v_meas, const Eigen::Vector2d & a_cmd);

  bool initialized() const { return initialized_; }
  Eigen::Vector2d estimate() const { return initialized_ ? d_hat_ : Eigen::Vector2d::Zero(); }

private:
  DisturbanceObserverConfig config_;
  bool initialized_{false};
  double last_stamp_{0.0};
  Eigen::Vector2d v_hat_{Eigen::Vector2d::Zero()};
  Eigen::Vector2d d_hat_{Eigen::Vector2d::Zero()};
};

}  // namespace minco_controller
