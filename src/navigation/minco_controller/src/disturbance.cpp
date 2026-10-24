#include "minco_controller/disturbance.hpp"

#include <algorithm>
#include <cmath>

namespace minco_controller {

Eigen::Vector2d slopeGravity(const Eigen::Quaterniond & q_world_body, double body_roll_offset, double g)
{
  const Eigen::Matrix3d R_world_chassis =
    q_world_body.normalized().toRotationMatrix() *
    Eigen::AngleAxisd(-body_roll_offset, Eigen::Vector3d::UnitX()).toRotationMatrix();
  const Eigen::Vector3d n = R_world_chassis.col(2);
  // Gravity minus its component along the ground normal; the normal force cancels the rest.
  const Eigen::Vector3d tangential = -g * (Eigen::Vector3d::UnitZ() - n.z() * n);
  return tangential.head<2>();
}

void DisturbanceObserver::update(
  double stamp, const Eigen::Vector2d & v_meas, const Eigen::Vector2d & v_cmd)
{
  if (!std::isfinite(stamp) || !v_meas.allFinite() || !v_cmd.allFinite() || config_.lag <= 0.0 ||
      config_.bandwidth <= 0.0) {
    reset();
    return;
  }
  const double dt = stamp - last_stamp_;
  if (!initialized_ || dt <= 0.0 || dt > config_.max_gap) {
    // Duplicate or out-of-order stamps are ignored; a gap restarts from the measurement.
    if (initialized_ && dt <= 0.0) {
      return;
    }
    initialized_ = true;
    last_stamp_ = stamp;
    v_hat_ = v_meas;
    d_hat_.setZero();
    return;
  }
  last_stamp_ = stamp;

  // Predict with the exact first-order lag, then correct both states from the velocity residual.
  // Gains place both error poles of [v̂, d̂] at -bandwidth.
  const double decay = std::exp(-dt / config_.lag);
  const Eigen::Vector2d v_pred = v_cmd + (v_hat_ - v_cmd) * decay + config_.lag * (1.0 - decay) * d_hat_;
  const double w = config_.bandwidth;
  const double l1 = std::max(0.0, 2.0 * w - 1.0 / config_.lag);
  const double l2 = w * w;
  const Eigen::Vector2d innovation = v_meas - v_pred;
  v_hat_ = v_pred + dt * l1 * innovation;
  d_hat_ += dt * l2 * innovation;
  d_hat_ = d_hat_.cwiseMax(-config_.max_estimate).cwiseMin(config_.max_estimate);
}

}  // namespace minco_controller
