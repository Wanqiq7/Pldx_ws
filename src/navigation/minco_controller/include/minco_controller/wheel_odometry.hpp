#pragma once

#include <deque>

#include <Eigen/Core>

namespace minco_controller {

// 底盘轮速里程计缓存与局部积分（报告 5.5.5.4.1）：以有延迟的 LIO 位姿为起点，
// 用高频轮速把底盘位姿积分到当前时刻。纯 C++，不加锁，由调用方串行化。
struct WheelOdometryConfig
{
  double max_age{1.0};   // s，缓存保留时长
  double max_gap{0.05};  // s，相邻样本间隔上限，超过视为中断
  double timeout{0.05};  // s，最新样本相对积分终点的最大陈旧度
};

class WheelOdometry
{
public:
  explicit WheelOdometry(const WheelOdometryConfig & config = {}) : config_(config) {}

  void setConfig(const WheelOdometryConfig & config)
  {
    config_ = config;
    samples_.clear();
  }
  const WheelOdometryConfig & config() const { return config_; }
  void clear() { samples_.clear(); }
  size_t size() const { return samples_.size(); }

  // @param stamp 测量时刻（s）；乱序或重复样本被丢弃
  // @param v_body 底盘系 [vx, vy, wz]
  void push(double stamp, const Eigen::Vector3d & v_body);

  // 把底盘位姿从 t0 积分到 t1。每个样本的速度保持到下一个样本（零阶保持），
  // 平移按段内中点航向旋到世界系。
  // 要求 t0 之前（含）有样本、区间内无中断、最新样本不陈旧。
  // @param v_body_at_t1 t1 时刻生效的底盘系速度
  bool propagate(double t0,
    double t1,
    const Eigen::Vector2d & p0,
    double yaw0,
    Eigen::Vector2d & p1,
    double & yaw1,
    Eigen::Vector3d & v_body_at_t1) const;

private:
  struct Sample
  {
    double stamp;
    Eigen::Vector3d v_body;
  };
  WheelOdometryConfig config_;
  std::deque<Sample> samples_;
};

}  // namespace minco_controller
