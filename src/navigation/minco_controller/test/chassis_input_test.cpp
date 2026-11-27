#include "minco_controller/minco_mpc_controller.hpp"

#include <gtest/gtest.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>

namespace {
using namespace std::chrono_literals;

TEST(ChassisInput, PublicRosInputUsesPhysicalPoseAndChildTwist)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("chassis_controller_test");
  node->declare_parameter("FollowPath.odom_topic", "/test/chassis_odom");
  node->declare_parameter("FollowPath.control_delay_compensation", 0.1);
  // Legacy overrides must not reintroduce a second installation compensation.
  node->declare_parameter("FollowPath.lidar_offset_y", -0.2);
  node->declare_parameter("FollowPath.use_small_gyro_mode", false);
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");
  costmap->set_parameter(rclcpp::Parameter("global_frame", "camera_init"));
  costmap->set_parameter(rclcpp::Parameter("robot_base_frame", "base_link_fake"));
  costmap->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{}));
  costmap->on_configure(rclcpp_lifecycle::State());
  auto buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  minco_controller::MincoMpcController controller;
  controller.configure(node, "FollowPath", buffer, costmap);
  auto io = std::make_shared<rclcpp::Node>("chassis_input_test_io");
  auto odomPublisher = io->create_publisher<nav_msgs::msg::Odometry>("/test/chassis_odom", 1);
  auto referencePublisher = io->create_publisher<ros_interfaces::msg::MpcPositionCommand>("/opt_path", 1);
  auto rawPublisher = io->create_publisher<nav_msgs::msg::Odometry>("/aft_mapped_to_init", 1);
  nav_msgs::msg::Path prediction;
  auto predictionSubscriber = io->create_subscription<nav_msgs::msg::Path>(
    "/mpc_predict_path", 1, [&](nav_msgs::msg::Path::ConstSharedPtr path) {
      prediction = *path;
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(io);
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (odomPublisher->get_subscription_count() == 0 && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_GT(odomPublisher->get_subscription_count(), 0u);
  nav_msgs::msg::Path plan;
  plan.poses.resize(1);
  controller.setPlan(plan);
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "camera_init";
  odom.child_frame_id = "base_link";
  odom.pose.pose.position.x = 2.0;
  odom.pose.pose.position.y = 3.0;
  odom.pose.pose.orientation.z = std::sqrt(0.5);
  odom.pose.pose.orientation.w = std::sqrt(0.5);
  odom.twist.twist.linear.x = 1.0;
  odom.twist.twist.angular.z = 1.0;
  ros_interfaces::msg::MpcPositionCommand reference;
  reference.header.frame_id = "camera_init";
  reference.command_flag = ros_interfaces::msg::MpcPositionCommand::NORMAL_COMMAND;
  reference.cmds.resize(40);
  for (size_t i = 0; i < reference.cmds.size(); ++i) {
    reference.cmds[i].position.x = 2.0;
    reference.cmds[i].position.y = 3.0 + 0.05 * i;
    reference.cmds[i].velocity.y = 1.0;
    reference.cmds[i].yaw = M_PI / 2.0 + 0.05 * i;
    reference.cmds[i].yaw_dot = 1.0;
    reference.cmds[i].trajectory_id = 1;
  }
  geometry_msgs::msg::PoseStamped virtualPose;
  virtualPose.header.frame_id = "camera_init";
  virtualPose.pose.orientation.w = 1.0;  // Nav2 virtual yaw must not replace physical yaw.
  virtualPose.pose.position.x = 100.0;   // Odometry also owns the reference point.
  for (int i = 0; i < 30 && prediction.poses.empty(); ++i) {
    odom.header.stamp = io->now();
    reference.header.stamp = io->now();
    odomPublisher->publish(odom);
    referencePublisher->publish(reference);
    auto raw = odom;
    raw.pose.pose.orientation.z = 0.0;
    raw.pose.pose.orientation.w = 1.0;
    raw.twist.twist.angular.z = 5.0;
    rawPublisher->publish(raw);
    executor.spin_some();
    controller.computeVelocityCommands(virtualPose, geometry_msgs::msg::Twist{}, nullptr);
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_FALSE(prediction.poses.empty());
  EXPECT_EQ(prediction.header.frame_id, "camera_init");
  const auto & current = prediction.poses.front().pose;
  EXPECT_NEAR(current.position.x, 2.0, 1e-9);
  EXPECT_NEAR(current.position.y, 3.1, 1e-9);
  EXPECT_NEAR(tf2::getYaw(current.orientation), M_PI / 2.0 + 0.1, 1e-9);
  EXPECT_NEAR(prediction.poses[1].pose.position.x, 2.0, 0.01);
  EXPECT_GT(prediction.poses[1].pose.position.y, 3.1);
  controller.cleanup();
  executor.remove_node(node->get_node_base_interface());
  executor.remove_node(io);
  costmap->on_cleanup(rclcpp_lifecycle::State());
  rclcpp::shutdown();
}
}  // namespace
