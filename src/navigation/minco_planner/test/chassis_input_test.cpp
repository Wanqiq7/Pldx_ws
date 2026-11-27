#include "minco_core/minco_planner.hpp"

#include <gtest/gtest.h>
#include <thread>

namespace {
using namespace std::chrono_literals;

TEST(ChassisInput, PlannerUsesMeasuredChassisPoseYawAndWorldVelocity)
{
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"GridBased.odom_topic", "/test/planner_chassis_odom"},
    {"GridBased.planner_mode", "PRIORMAP"},
    {"GridBased.rog_map.map_size", std::vector<double>{2.0, 2.0, 2.0}},
    {"GridBased.rog_map.resolution", 0.5},
    {"GridBased.rog_map.inflation_resolution", 0.5},
    {"GridBased.rog_map.decay.clear_time", 1.0},
    {"GridBased.rog_map.ros_callback.enable", false},
    {"GridBased.performance.enable", false},
    {"GridBased.performance.print_enable", false},
  });
  auto node = std::make_shared<nav2_util::LifecycleNode>("chassis_planner_test", "", options);
  auto buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  geometry_msgs::msg::TransformStamped correction;
  correction.header.frame_id = "map";
  correction.child_frame_id = "camera_init";
  correction.transform.translation.x = 10.0;
  correction.transform.rotation.z = std::sqrt(0.5);
  correction.transform.rotation.w = std::sqrt(0.5);
  ASSERT_TRUE(buffer->setTransform(correction, "chassis_test", true));
  minco_planner::MincoPlanner planner;
  planner.configure(node, "GridBased", buffer, nullptr);
  auto io = std::make_shared<rclcpp::Node>("chassis_planner_test_io");
  auto publisher = io->create_publisher<nav_msgs::msg::Odometry>("/test/planner_chassis_odom", 1);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(io);
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "camera_init";
  odom.child_frame_id = "base_link";
  odom.pose.pose.orientation.w = 1.0;
  odom.pose.pose.position.x = 1.0;
  odom.twist.twist.linear.x = 2.0;
  geometry_msgs::msg::PoseStamped pose;
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (!planner.getRobotPose(pose) && std::chrono::steady_clock::now() < deadline) {
    odom.header.stamp = io->now();
    publisher->publish(odom);
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_TRUE(planner.getRobotPose(pose));
  EXPECT_EQ(pose.header.frame_id, "map");
  EXPECT_EQ(pose.header.stamp, odom.header.stamp);
  EXPECT_NEAR(pose.pose.position.x, 10.0, 1e-9);
  EXPECT_NEAR(pose.pose.position.y, 1.0, 1e-9);
  EXPECT_NEAR(planner.getCurrentYawFromOdom(), M_PI / 2.0, 1e-9);
  EXPECT_NEAR(planner.getCurrentSpeed().x(), 0.0, 1e-9);
  EXPECT_NEAR(planner.getCurrentSpeed().y(), 2.0, 1e-9);
  planner.cleanup();
  executor.remove_node(node->get_node_base_interface());
  executor.remove_node(io);
  rclcpp::shutdown();
}
}  // namespace
