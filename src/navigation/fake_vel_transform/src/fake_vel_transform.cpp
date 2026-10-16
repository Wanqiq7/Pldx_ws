#include "fake_vel_transform/fake_vel_transform.hpp"

#include <chrono>
#include <cmath>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace fake_vel_transform {
FakeVelTransform::FakeVelTransform(const rclcpp::NodeOptions & options)
: Node("fake_vel_transform", options)
{
  odom_frame_ = declare_parameter<std::string>("odom_frame", "camera_init");
  body_frame_ = declare_parameter<std::string>("body_frame", "body");
  robot_base_frame_ = declare_parameter<std::string>("robot_base_frame", "base_link");
  fake_robot_base_frame_ = declare_parameter<std::string>("fake_robot_base_frame", "base_link_fake");
  joint_yaw_sign_ = declare_parameter<double>("joint_yaw_sign", 1.0);
  const double body_to_base_x = declare_parameter<double>("body_to_base_x", 0.0);
  const double body_to_base_y = declare_parameter<double>("body_to_base_y", 0.20);
  const double body_to_base_z = declare_parameter<double>("body_to_base_z", 0.0);
  gimbal_frame_ = declare_parameter<std::string>("gimbal_frame", "vision_gimbal");
  base_to_gimbal_z_ = declare_parameter<double>("base_to_gimbal_z", 0.5);
  const int period_ms = declare_parameter<int>("publish_period_ms", 10);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
  geometry_msgs::msg::TransformStamped body_to_base;
  body_to_base.header.stamp = now();
  body_to_base.header.frame_id = body_frame_;
  body_to_base.child_frame_id = robot_base_frame_;
  body_to_base.transform.translation.x = body_to_base_x;
  body_to_base.transform.translation.y = body_to_base_y;
  body_to_base.transform.translation.z = body_to_base_z;
  body_to_base.transform.rotation.w = 1.0;
  static_tf_broadcaster_->sendTransform(body_to_base);

  offline_sub_ = create_subscription<ros_interfaces::msg::SentryInfoOffline>("/sentry/offline_info",
    rclcpp::QoS(10),
    std::bind(&FakeVelTransform::onOfflineInfo, this, std::placeholders::_1));

  timer_ = create_wall_timer(std::chrono::milliseconds(period_ms), [this]() {
    publishTransforms();
  });
}

void FakeVelTransform::onOfflineInfo(const ros_interfaces::msg::SentryInfoOffline::SharedPtr msg)
{
  if (!std::isfinite(msg->yaw_camerainit_to_gimbal)) {
    return;
  }
  // The feedback angle is relative to the chassis, in degrees.
  geometry_msgs::msg::TransformStamped gimbal;
  gimbal.header.stamp = now();
  gimbal.header.frame_id = robot_base_frame_;
  gimbal.child_frame_id = gimbal_frame_;
  gimbal.transform.translation.z = base_to_gimbal_z_;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, joint_yaw_sign_ * msg->yaw_camerainit_to_gimbal * M_PI / 180.0);
  gimbal.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(gimbal);
}

void FakeVelTransform::publishTransforms()
{
  geometry_msgs::msg::TransformStamped body_in_odom;
  try {
    body_in_odom = tf_buffer_->lookupTransform(odom_frame_, body_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    return;
  }

  const double yaw_base = tf2::getYaw(body_in_odom.transform.rotation);
  geometry_msgs::msg::TransformStamped fake;
  fake.header.stamp = body_in_odom.header.stamp;
  fake.header.frame_id = robot_base_frame_;
  fake.child_frame_id = fake_robot_base_frame_;
  fake.transform.translation.x = 0.0;
  fake.transform.translation.y = 0.0;
  fake.transform.translation.z = 0.0;
  tf2::Quaternion fake_q;
  fake_q.setRPY(0.0, 0.0, -yaw_base);
  fake.transform.rotation = tf2::toMsg(fake_q);
  tf_broadcaster_->sendTransform(fake);
}

}  // namespace fake_vel_transform

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fake_vel_transform::FakeVelTransform)
