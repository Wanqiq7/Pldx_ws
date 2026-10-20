// Adapted from Nav2 GoalUpdater.
// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Francisco Martin Rico
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <nav2_behavior_tree/bt_utils.hpp>
#include <rclcpp/rclcpp.hpp>

namespace navi2 {

// Keep accepted topic updates across ticks; discard a superseded update once.
class GoalUpdater : public BT::DecoratorNode
{
public:
  GoalUpdater(const std::string & name, const BT::NodeConfiguration & config)
  : BT::DecoratorNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
    std::string topic;
    node_->get_parameter_or<std::string>("goal_updater_topic", topic, "goal_update");
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    subscription_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      topic,
      rclcpp::SystemDefaultsQoS(),
      [this](geometry_msgs::msg::PoseStamped::SharedPtr message) {
        last_goal_ = *message;
      },
      options);
    // Retain Nav2's Jazzy executor warm-up for newly created subscriptions.
    executor_.spin_all(std::chrono::milliseconds(1));
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<geometry_msgs::msg::PoseStamped>("input_goal", "Original Goal"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("output_goal", "Updated Goal")};
  }

private:
  BT::NodeStatus tick() override
  {
    auto input = getInput<geometry_msgs::msg::PoseStamped>("input_goal");
    if (!input) {
      throw BT::RuntimeError("Missing input_goal: ", input.error());
    }
    auto goal = input.value();
    if (goal.header.stamp != last_action_goal_stamp_ ||
        goal.pose.position.x != last_action_goal_.pose.position.x ||
        goal.pose.position.y != last_action_goal_.pose.position.y ||
        goal.pose.position.z != last_action_goal_.pose.position.z ||
        goal.pose.orientation.x != last_action_goal_.pose.orientation.x ||
        goal.pose.orientation.y != last_action_goal_.pose.orientation.y ||
        goal.pose.orientation.z != last_action_goal_.pose.orientation.z ||
        goal.pose.orientation.w != last_action_goal_.pose.orientation.w ||
        goal.header.frame_id != last_action_goal_.header.frame_id) {
      last_goal_.reset();
      last_action_goal_ = goal;
      last_action_goal_stamp_ = goal.header.stamp;
    }
    executor_.spin_all(std::chrono::milliseconds(49));
    if (last_goal_ && rclcpp::Time(last_goal_->header.stamp).nanoseconds() != 0) {
      goal = *last_goal_;
    }
    setOutput("output_goal", goal);
    return child_node_->executeTick();
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  std::optional<geometry_msgs::msg::PoseStamped> last_goal_;
  geometry_msgs::msg::PoseStamped last_action_goal_;
  builtin_interfaces::msg::Time last_action_goal_stamp_{};
};

}  // namespace navi2

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navi2::GoalUpdater>("MincoGoalUpdater");
}
