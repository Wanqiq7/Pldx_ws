#include "bt_manager/ros_interface.hpp"

#include <chrono>
#include <gtest/gtest.h>
#include <thread>

namespace {
using namespace std::chrono_literals;

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
