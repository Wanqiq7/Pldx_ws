#include <cmath>
#include <memory>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <gtest/gtest.h>
#include <nav2_core/progress_checker.hpp>
#include <pluginlib/class_loader.hpp>
#include <rcl/time.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

TEST(OriginalProgressContract, LoadedNavigationParametersUsePositionProgressAndOriginalTimeout)
{
  rclcpp::init(0, nullptr);
  {
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "--params-file", NAVIGATION_PARAMS_FILE});
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_server", options);
    const auto plugin = node->get_parameter("progress_checker.plugin").as_string();
    EXPECT_EQ(plugin, "nav2_controller::SimpleProgressChecker");
    EXPECT_DOUBLE_EQ(node->get_parameter("progress_checker.required_movement_radius").as_double(), 0.1);
    EXPECT_DOUBLE_EQ(node->get_parameter("progress_checker.movement_time_allowance").as_double(), 1000.0);
    pluginlib::ClassLoader<nav2_core::ProgressChecker> loader("nav2_core", "nav2_core::ProgressChecker");
    auto checker = loader.createSharedInstance(plugin);
    checker->initialize(node, "progress_checker");
    auto * clock = node->get_clock()->get_clock_handle();
    ASSERT_EQ(rcl_enable_ros_time_override(clock), RCL_RET_OK);
    auto setTime = [clock](int64_t seconds) {
      ASSERT_EQ(rcl_set_ros_time_override(clock, seconds * 1000000000LL), RCL_RET_OK);
    };
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.orientation.w = 1.0;
    setTime(1);
    EXPECT_TRUE(checker->check(pose));
    setTime(12);
    EXPECT_TRUE(checker->check(pose));  // More than the former 10 second allowance.
    setTime(1000);
    pose.pose.orientation.z = std::sin(0.5);
    pose.pose.orientation.w = std::cos(0.5);
    EXPECT_TRUE(checker->check(pose));
    setTime(1002);
    EXPECT_FALSE(checker->check(pose));  // Rotation must not reset position progress.
    pose.pose.position.x = 0.11;
    EXPECT_TRUE(checker->check(pose));
    setTime(2001);
    EXPECT_TRUE(checker->check(pose));
    setTime(2003);
    EXPECT_FALSE(checker->check(pose));
    checker->reset();
    EXPECT_TRUE(checker->check(pose));
  }
  rclcpp::shutdown();
}
