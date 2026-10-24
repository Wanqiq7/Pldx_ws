#include "minco_controller/wheel_odometry.hpp"

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

namespace minco_controller {
namespace {

TEST(WheelOdometry, StraightLineInRotatedBodyFrame)
{
  WheelOdometry odom;
  for (int i = 0; i <= 100; ++i) {
    odom.push(i * 0.001, Eigen::Vector3d(1.0, 0.5, 0.0));
  }
  Eigen::Vector2d p;
  double yaw = 0.0;
  Eigen::Vector3d v;
  const double yaw0 = M_PI / 2;
  ASSERT_TRUE(odom.propagate(0.02, 0.1, Eigen::Vector2d(1.0, 2.0), yaw0, p, yaw, v));
  // Body +x points to world +y at yaw 90°.
  EXPECT_NEAR(p.x(), 1.0 - 0.5 * 0.08, 1e-12);
  EXPECT_NEAR(p.y(), 2.0 + 1.0 * 0.08, 1e-12);
  EXPECT_NEAR(yaw, yaw0, 1e-12);
  EXPECT_EQ(v, Eigen::Vector3d(1.0, 0.5, 0.0));
}

TEST(WheelOdometry, ArcMatchesClosedForm)
{
  // Constant body velocity and yaw rate traces a circle of radius v/w.
  WheelOdometry odom;
  const double v = 1.2;
  const double w = 2.0;
  for (int i = 0; i <= 300; ++i) {
    odom.push(i * 0.001, Eigen::Vector3d(v, 0.0, w));
  }
  Eigen::Vector2d p;
  double yaw = 0.0;
  Eigen::Vector3d vel;
  const double T = 0.25;
  ASSERT_TRUE(odom.propagate(0.0, T, Eigen::Vector2d::Zero(), 0.0, p, yaw, vel));
  EXPECT_NEAR(yaw, w * T, 1e-12);
  // Mid-segment heading is second-order: about 5e-8 m here at 1 kHz.
  EXPECT_NEAR(p.x(), v / w * std::sin(w * T), 1e-6);
  EXPECT_NEAR(p.y(), v / w * (1.0 - std::cos(w * T)), 1e-6);
}

TEST(WheelOdometry, HoldsEachSampleUntilTheNextOne)
{
  WheelOdometry odom;
  odom.push(0.000, Eigen::Vector3d(1.0, 0.0, 0.0));
  odom.push(0.010, Eigen::Vector3d(3.0, 0.0, 0.0));
  odom.push(0.020, Eigen::Vector3d(-1.0, 0.0, 0.0));
  Eigen::Vector2d p;
  double yaw = 0.0;
  Eigen::Vector3d v;
  // t0 between samples uses the sample in effect; t1 equal to a stamp uses that sample next.
  ASSERT_TRUE(odom.propagate(0.005, 0.020, Eigen::Vector2d::Zero(), 0.0, p, yaw, v));
  EXPECT_NEAR(p.x(), 0.005 * 1.0 + 0.010 * 3.0, 1e-12);
  EXPECT_EQ(v.x(), -1.0);
  ASSERT_TRUE(odom.propagate(0.015, 0.015, Eigen::Vector2d(4.0, 0.0), 0.3, p, yaw, v));
  EXPECT_EQ(p, Eigen::Vector2d(4.0, 0.0));
  EXPECT_EQ(v.x(), 3.0);
}

TEST(WheelOdometry, RejectsMissingStaleOrInterruptedData)
{
  WheelOdometryConfig config;
  config.max_gap = 0.02;
  config.timeout = 0.03;
  WheelOdometry odom(config);
  Eigen::Vector2d p;
  double yaw = 0.0;
  Eigen::Vector3d v;
  EXPECT_FALSE(odom.propagate(0.0, 0.01, Eigen::Vector2d::Zero(), 0.0, p, yaw, v));
  for (int i = 0; i <= 10; ++i) {
    odom.push(1.0 + i * 0.01, Eigen::Vector3d(1.0, 0.0, 0.0));
  }
  // Starts before the first sample.
  EXPECT_FALSE(odom.propagate(0.99, 1.05, Eigen::Vector2d::Zero(), 0.0, p, yaw, v));
  // Newest sample (1.10) too old for t1.
  EXPECT_FALSE(odom.propagate(1.05, 1.14, Eigen::Vector2d::Zero(), 0.0, p, yaw, v));
  EXPECT_TRUE(odom.propagate(1.05, 1.12, Eigen::Vector2d::Zero(), 0.0, p, yaw, v));
  // A gap inside the interval.
  odom.push(1.20, Eigen::Vector3d(1.0, 0.0, 0.0));
  EXPECT_FALSE(odom.propagate(1.05, 1.21, Eigen::Vector2d::Zero(), 0.0, p, yaw, v));
  // Reversed interval and non-finite inputs.
  EXPECT_FALSE(odom.propagate(1.21, 1.20, Eigen::Vector2d::Zero(), 0.0, p, yaw, v));
  EXPECT_FALSE(odom.propagate(1.20, 1.21, Eigen::Vector2d(std::nan(""), 0.0), 0.0, p, yaw, v));
}

TEST(WheelOdometry, DropsOutOfOrderAndInvalidSamplesAndPrunesOldOnes)
{
  WheelOdometryConfig config;
  config.max_age = 0.1;
  WheelOdometry odom(config);
  odom.push(1.0, Eigen::Vector3d(1.0, 0.0, 0.0));
  odom.push(1.0, Eigen::Vector3d(9.0, 0.0, 0.0));
  odom.push(0.9, Eigen::Vector3d(9.0, 0.0, 0.0));
  odom.push(1.01, Eigen::Vector3d(std::numeric_limits<double>::infinity(), 0.0, 0.0));
  EXPECT_EQ(odom.size(), 1u);
  for (int i = 1; i <= 50; ++i) {
    odom.push(1.0 + i * 0.01, Eigen::Vector3d(1.0, 0.0, 0.0));
  }
  EXPECT_LE(odom.size(), 12u);
}

}  // namespace
}  // namespace minco_controller
