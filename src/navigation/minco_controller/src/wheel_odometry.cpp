#include "minco_controller/wheel_odometry.hpp"

#include <algorithm>
#include <cmath>

namespace minco_controller {

void WheelOdometry::push(double stamp, const Eigen::Vector3d & v_body)
{
  if (!std::isfinite(stamp) || !v_body.allFinite() ||
      (!samples_.empty() && stamp <= samples_.back().stamp)) {
    return;
  }
  samples_.push_back({stamp, v_body});
  while (samples_.size() > 2 && samples_.front().stamp < stamp - config_.max_age) {
    samples_.pop_front();
  }
}

bool WheelOdometry::propagate(double t0,
  double t1,
  const Eigen::Vector2d & p0,
  double yaw0,
  Eigen::Vector2d & p1,
  double & yaw1,
  Eigen::Vector3d & v_body_at_t1) const
{
  if (samples_.empty() || !std::isfinite(t0) || !std::isfinite(t1) || t1 < t0 || !p0.allFinite() ||
      !std::isfinite(yaw0)) {
    return false;
  }
  // The sample in effect at t0 is the last one stamped at or before it.
  auto it = std::upper_bound(samples_.begin(), samples_.end(), t0, [](double t, const Sample & s) {
    return t < s.stamp;
  });
  if (it == samples_.begin()) {
    return false;
  }
  --it;
  if (t1 - samples_.back().stamp > config_.timeout || t0 - it->stamp > config_.max_gap) {
    return false;
  }

  Eigen::Vector2d p = p0;
  double yaw = yaw0;
  double t = t0;
  while (true) {
    const auto next = std::next(it);
    // Each sample holds until the next one, or until t1 for the newest sample.
    const bool last = next == samples_.end() || next->stamp > t1;
    const double t_end = last ? t1 : next->stamp;
    if (!last && next->stamp - it->stamp > config_.max_gap) {
      return false;
    }
    const double dt = t_end - t;
    if (dt > 0.0) {
      // Rotate the body velocity at the mid-segment heading.
      const double dyaw = it->v_body.z() * dt;
      const double mid = yaw + 0.5 * dyaw;
      const double c = std::cos(mid);
      const double s = std::sin(mid);
      p += dt * Eigen::Vector2d(
                  c * it->v_body.x() - s * it->v_body.y(), s * it->v_body.x() + c * it->v_body.y());
      yaw += dyaw;
      t = t_end;
    }
    if (last) {
      v_body_at_t1 = it->v_body;
      break;
    }
    it = next;
  }
  p1 = p;
  yaw1 = yaw;
  return true;
}

}  // namespace minco_controller
