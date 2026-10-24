#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>

#include "bt_manager/utils/area.hpp"
#include "bt_manager/utils/log.hpp"
#include "bt_manager/utils/nav_zone.hpp"
#include <behaviortree_cpp/behavior_tree.h>
#include <nav2_behavior_tree/bt_action_node.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

using namespace color_text;
namespace Sentry_BT {
class MapsToPoseAction : public nav2_behavior_tree::BtActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  MapsToPoseAction(
    const std::string & xml_tag_name, const std::string & action_name, const BT::NodeConfiguration & conf)
  : nav2_behavior_tree::BtActionNode<nav2_msgs::action::NavigateToPose>(xml_tag_name, action_name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({BT::InputPort<Sentry_BT::Point2D>("nav_goal"),
      BT::InputPort<double>(
        "abort_backoff_initial_sec", 2.0, "Delay before retrying the same aborted navigation goal"),
      BT::InputPort<double>(
        "abort_backoff_max_sec", 16.0, "Maximum exponential retry delay for an aborted navigation goal")});
  }

  struct NavigationRetryState
  {
    Point2D goal{};
    uint32_t consecutive_aborts{0};
    rclcpp::Time retry_not_before{0, 0, RCL_ROS_TIME};
  };

  // Read the current goal, but suppress only an actual ABORTED action. Goal
  // changes, successful arrival and operator cancellation never start a delay.
  void on_tick() override
  {
    Point2D nav_goal;
    if (!readGoalFromBlackboard(nav_goal)) {
      throw BT::RuntimeError("missing required input [nav_goal]");
    }

    setActionGoal(nav_goal);
    should_send_goal_ = true;

    NavigationRetryState retry;
    const auto blackboard = config().blackboard;
    const rclcpp::Time now = node_->now();
    if (blackboard->get(kRetryStateKey, retry) && !goalsDiffer(nav_goal, retry.goal) &&
        now < retry.retry_not_before) {
      // Nav2's base returns FAILURE without sending when should_send_goal_ is false.
      should_send_goal_ = false;
      const double remaining = (retry.retry_not_before - now).seconds();
      if (!backoff_logged_ || (now - last_backoff_log_time_).seconds() >= 1.0) {
        std::cout << YELLOW << "[NavigateToPoseAction:" << name() << "] retry same goal=(" << nav_goal.x
                  << ", " << nav_goal.y << ") in " << remaining << " s (abort " << retry.consecutive_aborts
                  << ")" << RESET << std::endl;
        backoff_logged_ = true;
        last_backoff_log_time_ = now;
      }
      return;
    }

    backoff_logged_ = false;
    if (isGoalChanged(nav_goal)) {
      last_goal_ = nav_goal;
      has_last_goal_ = true;
      nav_start_time_ = std::chrono::steady_clock::now();

      std::cout << GREEN << "[NavigateToPoseAction:" << name() << "] send initial goal=(" << nav_goal.x
                << ", " << nav_goal.y << ")" << RESET << std::endl;
    }
  }

  bool readGoalFromBlackboard(Sentry_BT::Point2D & nav_goal)
  {
    auto blackboard = config().blackboard;
    return blackboard->get<Sentry_BT::Point2D>("nav_goal", nav_goal) || getInput("nav_goal", nav_goal);
  }

  void setActionGoal(const Sentry_BT::Point2D & nav_goal)
  {
    goal_.pose.header.frame_id = "map";
    goal_.pose.pose.position.x = nav_goal.x;
    goal_.pose.pose.position.y = nav_goal.y;
    goal_.pose.pose.orientation.w = 1.0;
  }

  bool isGoalChanged(const Sentry_BT::Point2D & nav_goal) const
  {
    if (!has_last_goal_) {
      return true;
    }
    return goalsDiffer(nav_goal, last_goal_);
  }

  static bool goalsDiffer(const Point2D & a, const Point2D & b)
  {
    return std::hypot(a.x - b.x, a.y - b.y) > 0.01;
  }

  bool selectNextPatrolGoalFromBlackboard(Sentry_BT::Point2D & next_goal)
  {
    auto blackboard = config().blackboard;

    int patrol_index = 0;
    if (!blackboard->get<int>("patrol_index", patrol_index)) {
      blackboard->set("patrol_index", patrol_index);
    }

    TacticalMode tactical_mode = TacticalMode::BALANCED;
    if (!blackboard->get<TacticalMode>("tactical_mode", tactical_mode)) {
      tactical_mode = TacticalMode::BALANCED;
    }
    auto branch = blackboard->get<int>("patrol_branch");

    // --- 根据战术模式和分支索引查找对应的巡逻点列表 ---
    const PatrolList * patrol_points_ptr = nullptr;
    auto it = Sentry_BT::tactical_patrol_branches.find(tactical_mode);
    if (it != Sentry_BT::tactical_patrol_branches.end()) {
      const auto & branches = it->second;
      if (branch >= 0 && branch < static_cast<int>(branches.size())) {
        patrol_points_ptr = &branches[branch];
      }
    }
    // 回退到防守分支0
    if (!patrol_points_ptr) {
      patrol_points_ptr = &Sentry_BT::normal_patrol_branches.at(0);
    }

    const PatrolList & patrol_points = *patrol_points_ptr;
    if (patrol_points.empty()) {
      return false;
    }

    if (patrol_index < 0 || patrol_index >= static_cast<int>(patrol_points.size())) {
      patrol_index = 0;
    }

    next_goal = patrol_points[patrol_index].position;
    const int next_index = (patrol_index + 1) % static_cast<int>(patrol_points.size());

    blackboard->set("patrol_index", next_index);
    blackboard->set("nav_goal", next_goal);
    blackboard->set<NavMode>("current_mode", NavMode::PATROL);
    return true;
  }

  bool isPatrolNavigationContext() const
  {
    const auto current_mode = config().blackboard->get<NavMode>("current_mode");
    return current_mode == NavMode::PATROL;
  }

  void on_wait_for_result(
    std::shared_ptr<const typename nav2_msgs::action::NavigateToPose::Feedback>) override
  {
    auto blackboard = config().blackboard;

    Point2D nav_goal;
    if (!readGoalFromBlackboard(nav_goal)) {
      return;
    }

    if (isPatrolNavigationContext()) {
      const auto now = std::chrono::steady_clock::now();
      const double elapsed_sec = std::chrono::duration<double>(now - nav_start_time_).count();
      if (elapsed_sec > 30.0) {
        Point2D timeout_goal;
        if (selectNextPatrolGoalFromBlackboard(timeout_goal)) {
          setActionGoal(timeout_goal);
          goal_updated_ = true;
          last_goal_ = timeout_goal;
          has_last_goal_ = true;
          nav_start_time_ = now;

          std::cout << YELLOW << "[NavigateToPoseAction:" << name()
                    << "] patrol timeout(>15s), force next patrol goal=(" << timeout_goal.x << ", "
                    << timeout_goal.y << ")" << RESET << std::endl;
          return;
        }
      }
    }

    if (isGoalChanged(nav_goal)) {
      setActionGoal(nav_goal);
      goal_updated_ = true;
      last_goal_ = nav_goal;
      has_last_goal_ = true;
      nav_start_time_ = std::chrono::steady_clock::now();

      std::cout << GREEN << "[NavigateToPoseAction:" << name() << "] preempt goal=(" << nav_goal.x << ", "
                << nav_goal.y << ")" << RESET << std::endl;
    }
  }

  void halt() override
  {
    has_last_goal_ = false;
    nav2_behavior_tree::BtActionNode<nav2_msgs::action::NavigateToPose>::halt();
  }

  BT::NodeStatus on_success() override
  {
    clearRetryState();
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus on_cancelled() override
  {
    has_last_goal_ = false;
    clearRetryState();
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus on_aborted() override
  {
    const auto blackboard = config().blackboard;
    NavigationRetryState retry;

    Point2D nav_goal;
    const bool have_blackboard_goal = readGoalFromBlackboard(nav_goal);
    if (has_last_goal_) {
      nav_goal = last_goal_;
    } else if (!have_blackboard_goal) {
      RCLCPP_WARN(node_->get_logger(), "Cannot record retry state for aborted goal without nav_goal");
      return BT::NodeStatus::FAILURE;
    }
    if (!blackboard->get(kRetryStateKey, retry) || goalsDiffer(nav_goal, retry.goal)) {
      retry = NavigationRetryState{};
    }
    retry.goal = nav_goal;
    retry.consecutive_aborts = std::min(retry.consecutive_aborts + 1, kMaxRetryCount);

    double initial_sec = getInput<double>("abort_backoff_initial_sec").value_or(2.0);
    double max_sec = getInput<double>("abort_backoff_max_sec").value_or(16.0);
    if (!std::isfinite(initial_sec) || initial_sec <= 0.0) {
      initial_sec = 2.0;
    }
    if (!std::isfinite(max_sec) || max_sec <= 0.0) {
      max_sec = 16.0;
    }
    max_sec = std::max(initial_sec, max_sec);
    const int exponent = std::min(static_cast<int>(retry.consecutive_aborts) - 1, 30);
    const double delay_sec = std::min(max_sec, initial_sec * std::exp2(exponent));
    retry.retry_not_before = node_->now() + rclcpp::Duration::from_seconds(delay_sec);
    blackboard->set(kRetryStateKey, retry);

    std::cout << YELLOW << "[NavigateToPoseAction:" << name() << "] aborted goal=(" << retry.goal.x << ", "
              << retry.goal.y << "); next attempt in " << delay_sec << " s" << RESET << std::endl;
    has_last_goal_ = false;
    backoff_logged_ = false;
    return BT::NodeStatus::FAILURE;
  }

private:
  static constexpr const char * kRetryStateKey = "navigation_abort_retry";
  static constexpr uint32_t kMaxRetryCount = 31;

  void clearRetryState()
  {
    config().blackboard->unset(kRetryStateKey);
    backoff_logged_ = false;
  }

  Sentry_BT::Point2D last_goal_{};
  bool has_last_goal_ = false;
  bool backoff_logged_{false};
  rclcpp::Time last_backoff_log_time_{0, 0, RCL_ROS_TIME};
  std::chrono::time_point<std::chrono::steady_clock> nav_start_time_ = std::chrono::steady_clock::now();
};

using NavigateToPoseAction = MapsToPoseAction;

}  // namespace Sentry_BT
