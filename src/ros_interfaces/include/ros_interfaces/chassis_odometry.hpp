#pragma once

#include <cmath>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace ros_interfaces {

// Pose and velocities at the same physical chassis reference point. Unlike
// Odometry::twist, both velocity vectors here are expressed in pose.header.frame_id.
struct ChassisWorldState
{
  geometry_msgs::msg::PoseStamped pose;
  geometry_msgs::msg::Vector3 linear;
  geometry_msgs::msg::Vector3 angular;
  double yaw;
  double yawRate;
};

inline ChassisWorldState chassisStateInWorld(
  const nav_msgs::msg::Odometry & odom, const geometry_msgs::msg::TransformStamped & worldFromOdom)
{
  tf2::Quaternion odomFromBase;
  tf2::fromMsg(odom.pose.pose.orientation, odomFromBase);
  tf2::Transform worldTransform;
  tf2::fromMsg(worldFromOdom.transform, worldTransform);
  const auto worldFromBase = worldTransform.getRotation() * odomFromBase;
  const auto & linear = odom.twist.twist.linear;
  const auto & angular = odom.twist.twist.angular;
  ChassisWorldState state;
  geometry_msgs::msg::PoseStamped pose;
  pose.header = odom.header;
  pose.pose = odom.pose.pose;
  tf2::doTransform(pose, state.pose, worldFromOdom);
  state.pose.header.stamp = odom.header.stamp;
  state.linear = tf2::toMsg(tf2::quatRotate(worldFromBase, tf2::Vector3(linear.x, linear.y, linear.z)));
  state.angular = tf2::toMsg(tf2::quatRotate(worldFromBase, tf2::Vector3(angular.x, angular.y, angular.z)));
  double roll, pitch;
  tf2::Matrix3x3(worldFromBase).getRPY(roll, pitch, state.yaw);
  state.yawRate = state.angular.z + std::tan(pitch) * (std::cos(state.yaw) * state.angular.x +
                                                        std::sin(state.yaw) * state.angular.y);
  return state;
}

inline ChassisWorldState chassisStateInWorld(const nav_msgs::msg::Odometry & odom)
{
  geometry_msgs::msg::TransformStamped identity;
  identity.header.frame_id = odom.header.frame_id;
  identity.transform.rotation.w = 1.0;
  return chassisStateInWorld(odom, identity);
}

}  // namespace ros_interfaces
