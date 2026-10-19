#include <atomic>
#include <chrono>
#include <cstdlib>
#include <thread>

#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>

namespace {
class Track : public BT::ActionNodeBase
{
public:
  using BT::ActionNodeBase::ActionNodeBase;
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override { return BT::NodeStatus::RUNNING; }
  void halt() override { resetStatus(); }
};
std::atomic<int> warnings{0};
void countWarnings(const rcutils_log_location_t *,
  int severity,
  const char *,
  rcutils_time_point_value_t,
  const char *,
  va_list *)
{
  if (severity == RCUTILS_LOG_SEVERITY_WARN) {
    ++warnings;
  }
}

TEST(GoalUpdater, DiscardsStaleCacheOnceAndKeepsAcceptedUpdates)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("test_goal_updater");
  auto board = BT::Blackboard::create();
  board->set("node", node);
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.header.stamp.sec = 10;
  goal.pose.position.x = 1.0;
  goal.pose.orientation.w = 1.0;
  board->set("goal", goal);
  BT::BehaviorTreeFactory factory;
  const char * native = std::getenv("TEST_NATIVE_GOAL_UPDATER");
  factory.registerFromPlugin(
    native ? "libnav2_goal_updater_node_bt_node.so" : "libminco_goal_updater_bt_node.so");
  factory.registerNodeType<Track>("Track");
  auto tree =
    factory.createTreeFromText(std::string("<root BTCPP_format=\"4\"><BehaviorTree ID=\"Test\"><") +
                                 (native ? "GoalUpdater" : "MincoGoalUpdater") +
                                 " input_goal=\"{goal}\" output_goal=\"{updated_goal}\"><Track/></" +
                                 (native ? "GoalUpdater" : "MincoGoalUpdater") + "></BehaviorTree></root>",
      board);
  auto publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>("goal_update", 10);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (publisher->get_subscription_count() == 0 && std::chrono::steady_clock::now() < deadline) {
    tree.tickOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_GT(publisher->get_subscription_count(), 0U);
  tree.tickOnce();
  auto previous_handler = rcutils_logging_get_output_handler();
  rcutils_logging_set_output_handler(countWarnings);
  warnings = 0;
  auto publishAndWait = [&](const geometry_msgs::msg::PoseStamped & update, auto received) {
    publisher->publish(update);
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (!received() && std::chrono::steady_clock::now() < end) {
      tree.tickOnce();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_TRUE(received());
  };
  // A topic update stays effective across ticks, even when the child keeps running.
  auto update = goal;
  update.header.stamp.sec = 20;
  update.pose.position.x = 2.0;
  publishAndWait(update, [&] {
    return board->get<geometry_msgs::msg::PoseStamped>("updated_goal").pose.position.x == 2.0;
  });
  for (int i = 0; i < 10; ++i) {
    tree.tickOnce();
    EXPECT_DOUBLE_EQ(board->get<geometry_msgs::msg::PoseStamped>("updated_goal").pose.position.x, 2.0);
  }
  EXPECT_EQ(warnings, 0);
  // A newer action makes that single cached update stale.
  goal.header.stamp.sec = 30;
  goal.pose.position.x = 3.0;
  board->set("goal", goal);
  tree.tickOnce();
  EXPECT_EQ(warnings, 1);
  for (int i = 0; i < 20; ++i) {
    tree.tickOnce();
  }
  EXPECT_EQ(warnings, 1) << "One stale cached message must not warn on every tick";
  EXPECT_DOUBLE_EQ(board->get<geometry_msgs::msg::PoseStamped>("updated_goal").pose.position.x, 3.0);
  // An explicitly stale/equal update is rejected once; later fresh updates still work.
  int before = warnings;
  update.header.stamp.sec = 30;
  publishAndWait(update, [&] {
    return warnings > before;
  });
  for (int i = 0; i < 10; ++i) {
    tree.tickOnce();
  }
  EXPECT_EQ(warnings, before + 1);
  update.header.stamp.sec = 40;
  update.pose.position.x = 4.0;
  publishAndWait(update, [&] {
    return board->get<geometry_msgs::msg::PoseStamped>("updated_goal").pose.position.x == 4.0;
  });
  EXPECT_EQ(warnings, before + 1);
  rcutils_logging_set_output_handler(previous_handler);
  tree.haltTree();
  rclcpp::shutdown();
}
}  // namespace
