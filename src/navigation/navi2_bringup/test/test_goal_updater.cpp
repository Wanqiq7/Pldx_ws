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

TEST(GoalUpdater, ResetsBetweenActionsAndAcceptsUpdatesFromAnotherClock)
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
  // A new action discards the previous action's cached topic update.
  goal.header.stamp.sec = 30;
  goal.pose.position.x = 3.0;
  board->set("goal", goal);
  tree.tickOnce();
  for (int i = 0; i < 20; ++i) {
    tree.tickOnce();
  }
  EXPECT_DOUBLE_EQ(board->get<geometry_msgs::msg::PoseStamped>("updated_goal").pose.position.x, 3.0);
  EXPECT_EQ(warnings, 0);
  // A message received during the current action remains valid even if its stamp
  // came from a clock that is behind the action server's clock.
  update.header.stamp.sec = 30;
  update.pose.position.x = 3.5;
  publishAndWait(update, [&] {
    return board->get<geometry_msgs::msg::PoseStamped>("updated_goal").pose.position.x == 3.5;
  });
  for (int i = 0; i < 10; ++i) {
    tree.tickOnce();
  }
  EXPECT_EQ(warnings, 0);
  // A later action still clears that update before processing its own messages.
  goal.header.stamp.sec = 40;
  goal.pose.position.x = 4.0;
  board->set("goal", goal);
  tree.tickOnce();
  EXPECT_DOUBLE_EQ(board->get<geometry_msgs::msg::PoseStamped>("updated_goal").pose.position.x, 4.0);
  update.header.stamp.sec = 1;
  update.pose.position.x = 5.0;
  publishAndWait(update, [&] {
    return board->get<geometry_msgs::msg::PoseStamped>("updated_goal").pose.position.x == 5.0;
  });
  EXPECT_EQ(warnings, 0);
  // An orientation-only action change also starts a new cache scope.
  goal.pose.orientation.z = 1.0;
  goal.pose.orientation.w = 0.0;
  board->set("goal", goal);
  tree.tickOnce();
  const auto oriented_goal = board->get<geometry_msgs::msg::PoseStamped>("updated_goal");
  EXPECT_DOUBLE_EQ(oriented_goal.pose.position.x, 4.0);
  EXPECT_DOUBLE_EQ(oriented_goal.pose.orientation.z, 1.0);
  EXPECT_DOUBLE_EQ(oriented_goal.pose.orientation.w, 0.0);
  EXPECT_EQ(warnings, 0);
  rcutils_logging_set_output_handler(previous_handler);
  tree.haltTree();
  rclcpp::shutdown();
}
}  // namespace
