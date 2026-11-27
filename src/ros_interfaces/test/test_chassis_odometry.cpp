#include "ros_interfaces/chassis_odometry.hpp"

#include <cmath>
#include <gtest/gtest.h>

TEST(ChassisOdometry, RotatesChildTwistWithoutLeverArmCompensation)
{
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "camera_init";
  odom.child_frame_id = "base_link";
  odom.header.stamp.sec = 12;
  odom.pose.pose.orientation.z = std::sqrt(0.5);
  odom.pose.pose.orientation.w = std::sqrt(0.5);
  odom.twist.twist.linear.x = 2.0;
  odom.twist.twist.angular.z = 3.0;
  const auto state = ros_interfaces::chassisStateInWorld(odom);
  EXPECT_NEAR(state.linear.x, 0.0, 1e-12);
  EXPECT_NEAR(state.linear.y, 2.0, 1e-12);
  EXPECT_NEAR(state.angular.z, 3.0, 1e-12);
  EXPECT_EQ(state.pose.header.stamp.sec, 12);
  EXPECT_EQ(state.pose.header.frame_id, "camera_init");
}

TEST(ChassisOdometry, TiltedChassisPreservesFullVelocityAndHeadingRate)
{
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "camera_init";
  odom.pose.pose.orientation.y = std::sin(M_PI / 8.0);
  odom.pose.pose.orientation.w = std::cos(M_PI / 8.0);
  odom.twist.twist.linear.z = std::sqrt(2.0);
  odom.twist.twist.angular.z = std::sqrt(2.0);
  const auto state = ros_interfaces::chassisStateInWorld(odom);
  EXPECT_NEAR(state.linear.x, 1.0, 1e-12);
  EXPECT_NEAR(state.linear.z, 1.0, 1e-12);
  EXPECT_NEAR(state.angular.x, 1.0, 1e-12);
  EXPECT_NEAR(state.angular.z, 1.0, 1e-12);
  EXPECT_NEAR(state.yawRate, 2.0, 1e-12);
}

TEST(ChassisOdometry, MapCorrectionTransformsPoseAndVelocityAtTheSameReferencePoint)
{
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "camera_init";
  odom.header.stamp.sec = 42;
  odom.pose.pose.orientation.w = 1.0;
  odom.pose.pose.position.x = 1.0;
  odom.twist.twist.linear.x = 2.0;
  geometry_msgs::msg::TransformStamped mapFromOdom;
  mapFromOdom.header.frame_id = "map";
  mapFromOdom.transform.translation.x = 10.0;
  mapFromOdom.transform.rotation.z = std::sqrt(0.5);
  mapFromOdom.transform.rotation.w = std::sqrt(0.5);
  const auto state = ros_interfaces::chassisStateInWorld(odom, mapFromOdom);
  EXPECT_EQ(state.pose.header.frame_id, "map");
  EXPECT_EQ(state.pose.header.stamp.sec, 42);
  EXPECT_NEAR(state.pose.pose.position.x, 10.0, 1e-12);
  EXPECT_NEAR(state.pose.pose.position.y, 1.0, 1e-12);
  EXPECT_NEAR(state.linear.x, 0.0, 1e-12);
  EXPECT_NEAR(state.linear.y, 2.0, 1e-12);
  EXPECT_NEAR(state.yaw, M_PI / 2.0, 1e-12);
}
