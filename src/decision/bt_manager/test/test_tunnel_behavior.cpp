#include "bt_manager/action/recovery_actions.hpp"
#include "bt_manager/condition/change_stance_condition.hpp"
#include "bt_manager/ros_interface.hpp"

#include <chrono>
#include <functional>
#include <gtest/gtest.h>
#include <thread>

namespace {
using namespace std::chrono_literals;

TEST(TunnelBehavior, FeedbackAndTunnelActionsReachPublishedBehavior)
{
  rclcpp::init(0, nullptr);
  {
    auto board = std::make_shared<Sentry_BT::Blackboard>();
    board->set<std::shared_ptr<Sentry_BT::TransformUtils>>("transform_utils", nullptr);
    auto interface = std::make_shared<Sentry_BT::ros_interface>(board);
    interface->set_parameter(rclcpp::Parameter("publish_area_markers", false));
    auto input = std::make_shared<rclcpp::Node>("test_tunnel_behavior_input");
    auto offline =
      input->create_publisher<ros_interfaces::msg::SentryInfoOffline>("/sentry/offline_info", 1);
    auto odometry = input->create_publisher<nav_msgs::msg::Odometry>("/aft_mapped_to_init", 1);
    auto route = input->create_publisher<ros_interfaces::msg::MpcPositionCommand>("/opt_path", 1);
    nav_msgs::msg::Odometry position;
    position.pose.pose.orientation.w = 1.0;
    ros_interfaces::msg::MpcPositionCommand path;
    path.cmds.resize(5);
    for (auto & command : path.cmds) {
      command.position.x = 9.8;
      command.position.y = 2.5;
    }
    ros_interfaces::msg::Behavior observed;
    int messages = 0;
    auto subscription = input->create_subscription<ros_interfaces::msg::Behavior>(
      "/sentry/behaivor_send", 10, [&](ros_interfaces::msg::Behavior::ConstSharedPtr message) {
        observed = *message;
        ++messages;
      });
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<Sentry_BT::CheckTunnelDeformation>("CheckTunnelDeformation");
    factory.registerNodeType<Sentry_BT::TunnelTimeoutBackoutAction>("TunnelTimeoutBackoutAction");
    auto preparation = factory.createTreeFromText("<root BTCPP_format=\"4\"><BehaviorTree ID=\"Prepare\">"
                                                  "<CheckTunnelDeformation/></BehaviorTree></root>",
      board->getBTBlackboard());
    auto escape =
      factory.createTreeFromText("<root BTCPP_format=\"4\"><BehaviorTree ID=\"Escape\">"
                                 "<TunnelTimeoutBackoutAction timeout_s=\"0.0\" max_backout_s=\"10.0\"/>"
                                 "</BehaviorTree></root>",
        board->getBTBlackboard());
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(interface);
    executor.add_node(input);
    auto waitFor = [&](const std::function<bool()> & predicate) {
      const auto deadline = std::chrono::steady_clock::now() + 2s;
      while (std::chrono::steady_clock::now() < deadline) {
        odometry->publish(position);
        route->publish(path);
        preparation.tickExactlyOnce();
        executor.spin_some();
        if (predicate()) {
          return true;
        }
        std::this_thread::sleep_for(2ms);
      }
      return false;
    };
    ASSERT_TRUE(waitFor([&] {
      return messages > 0 && offline->get_subscription_count() > 0;
    }));
    EXPECT_FALSE(observed.tunnel_prepare_active);
    position.pose.pose.position.x = 9.0;
    position.pose.pose.position.y = 2.5;
    ASSERT_TRUE(waitFor([&] {
      return observed.tunnel_prepare_active && !observed.tunnel_ready;
    }));
    EXPECT_EQ(observed.desire_lifter_pos, ros_interfaces::msg::Behavior::LIFTER_BOTTOM);
    ros_interfaces::msg::SentryInfoOffline feedback;
    feedback.lifter_current_pos = ros_interfaces::msg::Behavior::LIFTER_BOTTOM;
    feedback.tunnel_yaw_aligned = false;
    offline->publish(feedback);
    ASSERT_TRUE(waitFor([&] {
      return board->get<Sentry_BT::LifterPos>("lifter_current_pos") == Sentry_BT::LifterPos::BOTTOM;
    }));
    EXPECT_FALSE(observed.tunnel_ready);
    feedback.tunnel_yaw_aligned = true;
    offline->publish(feedback);
    ASSERT_TRUE(waitFor([&] {
      return observed.tunnel_ready;
    }));
    feedback.lifter_current_pos = ros_interfaces::msg::Behavior::LIFTER_TOP;
    offline->publish(feedback);
    ASSERT_TRUE(waitFor([&] {
      return !observed.tunnel_ready;
    }));
    position.pose.pose.position.x = 0.0;
    position.pose.pose.position.y = 0.0;
    ASSERT_TRUE(waitFor([&] {
      return !observed.tunnel_prepare_active;
    }));
    EXPECT_EQ(observed.desire_lifter_pos, ros_interfaces::msg::Behavior::LIFTER_TOP);
    geometry_msgs::msg::Pose pose;
    pose.position.x = 9.8;
    pose.position.y = 2.5;
    pose.orientation.w = 1.0;
    position.pose.pose = pose;
    ASSERT_TRUE(waitFor([&] {
      return observed.tunnel_prepare_active;
    }));
    escape.tickExactlyOnce();  // Record tunnel entry.
    escape.tickExactlyOnce();  // Zero test allowance activates the real escape action.
    ASSERT_TRUE(waitFor([&] {
      return observed.tunnel_escape_active;
    }));
    EXPECT_FLOAT_EQ(observed.tunnel_escape_vx, 0.0F);
    EXPECT_FLOAT_EQ(observed.tunnel_escape_vy, 0.5F);
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    position.pose.pose = pose;
    ASSERT_TRUE(waitFor([&] {
      return !observed.tunnel_prepare_active;
    }));
    escape.tickExactlyOnce();
    ASSERT_TRUE(waitFor([&] {
      return !observed.tunnel_escape_active && !observed.tunnel_prepare_active;
    }));
    executor.remove_node(interface);
    executor.remove_node(input);
  }
  rclcpp::shutdown();
}
}  // namespace
