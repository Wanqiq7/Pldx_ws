#ifndef FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_
#define FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_

#include <atomic>
#include <memory>
#include <string>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>
#include <ros_interfaces/msg/sentry_info_offline.hpp>

namespace fake_vel_transform {
class FakeVelTransform : public rclcpp::Node
{
public:
  explicit FakeVelTransform(const rclcpp::NodeOptions & options);

private:
  void onOfflineInfo(const ros_interfaces::msg::SentryInfoOffline::SharedPtr msg);
  void publishTransforms();

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<ros_interfaces::msg::SentryInfoOffline>::SharedPtr offline_sub_;

  std::string odom_frame_;
  std::string body_frame_;
  std::string robot_base_frame_;
  std::string fake_robot_base_frame_;
  double joint_yaw_sign_{1.0};
  double body_to_base_x_{0.0};
  double body_to_base_y_{0.0};
  double body_to_base_z_{-0.5};
  std::atomic<float> latest_gimbal_yaw_deg_{0.0f};
};

}  // namespace fake_vel_transform

#endif  // FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_
