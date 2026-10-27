#include "minco_controller/disturbance.hpp"

#include <cmath>
#include <limits>
#include <random>

#include <gtest/gtest.h>

namespace minco_controller {
namespace {

Eigen::Quaterniond rpy(double roll, double pitch, double yaw)
{
  return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
}

TEST(SlopeGravity, LevelGroundHasNoDrift)
{
  EXPECT_NEAR(slopeGravity(Eigen::Quaterniond::Identity(), 0.0).norm(), 0.0, 1e-12);
  EXPECT_NEAR(slopeGravity(rpy(0.0, 0.0, 1.3), 0.0).norm(), 0.0, 1e-12);
}

TEST(SlopeGravity, NoseUpPullsBackwardsDownTheSlope)
{
  const double slope = 10.0 * M_PI / 180.0;
  // REP-103: nose up is negative pitch about +y.
  for (double yaw : {0.0, 0.7, -2.1}) {
    SCOPED_TRACE(::testing::Message() << "yaw=" << yaw);
    const Eigen::Vector2d d = slopeGravity(rpy(0.0, -slope, yaw), 0.0);
    const Eigen::Vector2d forward(std::cos(yaw), std::sin(yaw));
    // Horizontal part of g·sin(θ) along the slope.
    EXPECT_NEAR(d.dot(forward), -9.81 * std::sin(slope) * std::cos(slope), 1e-9);
    EXPECT_NEAR(d.x() * forward.y() - d.y() * forward.x(), 0.0, 1e-9);
  }
}

TEST(SlopeGravity, RollPullsSideways)
{
  const double slope = 0.12;
  // Positive roll lifts the left (+y) side, so gravity pulls towards -y.
  const Eigen::Vector2d d = slopeGravity(rpy(slope, 0.0, 0.0), 0.0);
  EXPECT_NEAR(d.x(), 0.0, 1e-12);
  EXPECT_NEAR(d.y(), -9.81 * std::sin(slope) * std::cos(slope), 1e-9);
}

TEST(SlopeGravity, BodyRollOffsetRemovesMountingTilt)
{
  const double mount = 0.1745;
  EXPECT_NEAR(slopeGravity(rpy(mount, 0.0, 0.4), mount).norm(), 0.0, 1e-9);
  // On a nose-up slope the mount offset leaves only the slope component.
  const double slope = 0.15;
  const Eigen::Quaterniond on_slope = rpy(0.0, -slope, 0.0) * rpy(mount, 0.0, 0.0);
  const Eigen::Vector2d d = slopeGravity(on_slope, mount);
  EXPECT_NEAR(d.x(), -9.81 * std::sin(slope) * std::cos(slope), 1e-9);
  EXPECT_NEAR(d.y(), 0.0, 1e-9);
}

// A force-driven plant: acceleration is known input plus an unknown disturbance.
struct ForcePlant
{
  Eigen::Vector2d d;
  Eigen::Vector2d v{Eigen::Vector2d::Zero()};
  void step(const Eigen::Vector2d & cmd, double dt) { v += dt * (cmd + d); }
};

TEST(DisturbanceObserver, ConvergesToConstantDisturbance)
{
  DisturbanceObserverConfig config;
  DisturbanceObserver observer(config);
  ForcePlant plant{Eigen::Vector2d(-1.2, 0.4)};
  config.max_estimate = 2.0;
  observer.setConfig(config);
  const double dt = 0.01;
  Eigen::Vector2d executing = Eigen::Vector2d::Zero();
  for (int i = 0; i < 300; ++i) {
    const double t = i * dt;
    // The observer receives the command that ran since the previous sample, as the
    // controller passes the last published command.
    observer.update(t, plant.v, executing);
    // Varying commands so the result does not rely on steady state.
    executing = Eigen::Vector2d(0.8 * std::sin(1.3 * t), 0.5 * std::cos(0.9 * t));
    plant.step(executing, dt);
  }
  EXPECT_NEAR(observer.estimate().x(), -1.2, 0.02);
  EXPECT_NEAR(observer.estimate().y(), 0.4, 0.02);
}

TEST(DisturbanceObserver, StaysNearZeroWhenModelMatchesAndFiltersNoise)
{
  DisturbanceObserver observer;
  ForcePlant plant{Eigen::Vector2d::Zero()};
  std::mt19937 rng(7);
  std::normal_distribution<double> noise(0.0, 0.03);
  const double dt = 0.01;
  double worst = 0.0;
  Eigen::Vector2d executing = Eigen::Vector2d::Zero();
  for (int i = 0; i < 1000; ++i) {
    const double t = i * dt;
    observer.update(t, plant.v + Eigen::Vector2d(noise(rng), noise(rng)), executing);
    executing = Eigen::Vector2d(1.5 * std::sin(0.8 * t), 1.0 * std::sin(1.7 * t));
    plant.step(executing, dt);
    if (i > 200) {
      worst = std::max(worst, observer.estimate().cwiseAbs().maxCoeff());
    }
  }
  // 3 cm/s velocity noise at 100 Hz would be ~4 m/s² if differentiated directly.
  EXPECT_LT(worst, 0.35);
}

TEST(DisturbanceObserver, ClampsAndResetsOnGapsAndInvalidInput)
{
  DisturbanceObserverConfig config;
  config.max_estimate = 0.5;
  DisturbanceObserver observer(config);
  ForcePlant plant{Eigen::Vector2d(3.0, -3.0)};
  for (int i = 0; i < 300; ++i) {
    observer.update(i * 0.01, plant.v, Eigen::Vector2d::Zero());
    plant.step(Eigen::Vector2d::Zero(), 0.01);
  }
  EXPECT_DOUBLE_EQ(observer.estimate().x(), 0.5);
  EXPECT_DOUBLE_EQ(observer.estimate().y(), -0.5);

  // Duplicate stamps are ignored.
  const Eigen::Vector2d before = observer.estimate();
  observer.update(2.99, Eigen::Vector2d(9.0, 9.0), Eigen::Vector2d::Zero());
  EXPECT_EQ(observer.estimate(), before);
  // A data gap restarts from the measurement with no estimate.
  observer.update(3.5, plant.v, Eigen::Vector2d::Zero());
  EXPECT_TRUE(observer.initialized());
  EXPECT_EQ(observer.estimate(), Eigen::Vector2d::Zero());
  // Invalid input disables the estimate until the next valid sample.
  observer.update(
    3.51, Eigen::Vector2d(std::numeric_limits<double>::quiet_NaN(), 0.0), Eigen::Vector2d::Zero());
  EXPECT_FALSE(observer.initialized());
  EXPECT_EQ(observer.estimate(), Eigen::Vector2d::Zero());
}

}  // namespace
}  // namespace minco_controller
