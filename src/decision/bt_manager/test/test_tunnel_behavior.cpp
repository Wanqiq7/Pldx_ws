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
TEST(RefereeStance, ConfirmsRequestsOnlyFromFresh2027Feedback)
{
  rclcpp::init(0, nullptr);
  {
    auto board = std::make_shared<Sentry_BT::Blackboard>();
    auto interface = std::make_shared<Sentry_BT::ros_interface>(board);
    auto input = std::make_shared<rclcpp::Node>("test_referee_stance_input");
    auto online = input->create_publisher<ros_interfaces::msg::SentryInfoOnline>("/sentry/online_info", 1);
    uint8_t observed_request = 0;
    auto behavior = input->create_subscription<ros_interfaces::msg::Behavior>(
      "/sentry/behaivor_send", 10, [&](ros_interfaces::msg::Behavior::ConstSharedPtr message) {
        observed_request = message->desired_stance;
      });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(interface);
    executor.add_node(input);
    const auto waitFor = [&](const auto & ready, auto timeout) {
      const auto deadline = std::chrono::steady_clock::now() + timeout;
      do {
        executor.spin_some();
        if (ready())
          return true;
        std::this_thread::sleep_for(2ms);
      } while (std::chrono::steady_clock::now() < deadline);
      return false;
    };
    auto previous = Sentry_BT::UNKNOWN;
    for (const auto stance : {Sentry_BT::ATTACK, Sentry_BT::DEFEND, Sentry_BT::MOVE}) {
      board->set<Sentry_BT::SentryStance>("desired_stance", stance);
      ASSERT_TRUE(waitFor(
        [&] {
          return observed_request == stance;
        },
        2s));
      EXPECT_EQ(board->get<Sentry_BT::SentryStance>("current_stance"), previous);
      ros_interfaces::msg::SentryInfoOnline message;
      const bool energy_enabled = stance == Sentry_BT::DEFEND;
      const bool enhanced = stance == Sentry_BT::MOVE;
      message.sentry_info_2 = 0x032BU | (static_cast<uint16_t>(stance) << 12U) |
                              (energy_enabled ? 0x4000U : 0U) | (enhanced ? 0x8000U : 0U);
      message.sentry_info_1 = 0x00180049U;
      message.sentry_info_3 = 0x0006050400030201ULL;
      message.energy_ratio = 30U;
      ASSERT_TRUE(waitFor(
        [&] {
          online->publish(message);
          return board->get<Sentry_BT::SentryStance>("current_stance") == stance;
        },
        2s));
      EXPECT_EQ(board->get<bool>("can_activate_energy"), energy_enabled);
      EXPECT_TRUE(board->get<bool>("is_disengaged"));
      EXPECT_EQ(board->get<bool>("current_stance_is_enhanced"), enhanced);
      EXPECT_EQ(board->get<int>("remaining_ammo_exchange"), 405);
      previous = stance;
    }
    EXPECT_TRUE(waitFor(
      [&] {
        return board->get<Sentry_BT::SentryStance>("current_stance") == Sentry_BT::UNKNOWN;
      },
      4s));
    EXPECT_FALSE(board->get<bool>("current_stance_is_enhanced"));
    EXPECT_FALSE(board->get<bool>("can_activate_energy"));
    EXPECT_FALSE(board->get<bool>("can_free_resurrect"));
    EXPECT_FALSE(board->get<bool>("can_instant_resurrect"));
    EXPECT_EQ(board->get<int>("remaining_ammo_exchange"), 0);
    EXPECT_EQ(board->get<int>("enhanced_attack_remaining_time"), 0);
    EXPECT_EQ(board->get<int>("ammo_exchanged_local"), 73);
    EXPECT_EQ(observed_request, Sentry_BT::MOVE);
  }
  rclcpp::shutdown();
}
}  // namespace
