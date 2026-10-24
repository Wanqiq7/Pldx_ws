#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "bt_manager/action/nav_action.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace {
using namespace std::chrono_literals;
using Sentry_BT::Point2D;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;
using RetryState = Sentry_BT::NavigateToPoseAction::NavigationRetryState;
constexpr const char * kRetryKey = "navigation_abort_retry";

class TestNavAction : public Sentry_BT::NavigateToPoseAction
{
public:
  using Sentry_BT::NavigateToPoseAction::NavigateToPoseAction;
  bool goalAccepted() const { return goal_handle_ && !future_goal_handle_; }
};

class NavActionTest : public testing::TestWithParam<rclcpp_action::ResultCode>
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_nav_action");
    server_node_ = std::make_shared<rclcpp::Node>("test_nav_action_server");
    server_ = rclcpp_action::create_server<NavigateToPose>(
      server_node_,
      "navigate_to_pose",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const NavigateToPose::Goal> goal) {
        const std::lock_guard<std::mutex> lock(mutex_);
        goals_.emplace_back(goal->pose.pose.position.x, goal->pose.pose.position.y);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<GoalHandle>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandle> handle) {
        const std::lock_guard<std::mutex> lock(mutex_);
        handles_.push_back(handle);
      });
    // Complete cancellation after rclcpp_action has entered CANCELING, not
    // inside the cancel request callback before the state transition occurs.
    cancel_timer_ = server_node_->create_wall_timer(5ms, [this]() {
      const std::lock_guard<std::mutex> lock(mutex_);
      for (const auto & handle : handles_) {
        if (handle->is_canceling()) {
          handle->canceled(std::make_shared<NavigateToPose::Result>());
        }
      }
    });
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(server_node_);
    spinner_ = std::thread([this]() {
      executor_->spin();
    });

    board_ = BT::Blackboard::create();
    board_->set("node", node_);
    board_->set("current_mode", Sentry_BT::NavMode::MANUAL);
    board_->set("bt_loop_duration", 10ms);
    board_->set("wait_for_service_timeout", 2000ms);
    board_->set("server_timeout", 1000ms);
    board_->set("cancel_timeout", 1000ms);
    board_->set("nav_goal", Point2D(1.0, 2.0));
    factory_.registerBuilder<TestNavAction>(
      "NavigateToPoseAction", [](const std::string & name, const BT::NodeConfiguration & config) {
        return std::make_unique<TestNavAction>(name, "navigate_to_pose", config);
      });
  }

  void TearDown() override
  {
    for (auto & tree : trees_) {
      tree->haltTree();
    }
    trees_.clear();
    executor_->cancel();
    if (spinner_.joinable()) {
      spinner_.join();
    }
    executor_.reset();
    cancel_timer_.reset();
    server_.reset();
    server_node_.reset();
    board_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  void addTree(const std::string & attributes = "")
  {
    const auto xml = "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Test\">"
                     "<NavigateToPoseAction " +
                     attributes + "/></BehaviorTree></root>";
    trees_.push_back(std::make_unique<BT::Tree>(factory_.createTreeFromText(xml, board_)));
  }

  bool waitUntil(const std::function<bool()> & condition, size_t tree = 0)
  {
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (std::chrono::steady_clock::now() < deadline) {
      last_status_ = trees_.at(tree)->tickExactlyOnce();
      if (condition()) {
        return true;
      }
      std::this_thread::sleep_for(5ms);
    }
    return false;
  }

  bool waitForGoal(size_t count, size_t tree = 0)
  {
    return waitUntil(
      [this, count, tree]() {
        const std::lock_guard<std::mutex> lock(mutex_);
        const auto * action = dynamic_cast<TestNavAction *>(trees_.at(tree)->rootNode());
        return goals_.size() == count && handles_.size() == count && action->goalAccepted();
      },
      tree);
  }

  void completeGoal(size_t index, rclcpp_action::ResultCode result)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    ASSERT_LT(index, handles_.size());
    const auto message = std::make_shared<NavigateToPose::Result>();
    if (result == rclcpp_action::ResultCode::SUCCEEDED) {
      handles_[index]->succeed(message);
    } else {
      ASSERT_EQ(result, rclcpp_action::ResultCode::ABORTED);
      handles_[index]->abort(message);
    }
  }

  bool waitForStatus(BT::NodeStatus wanted, size_t tree = 0)
  {
    return waitUntil(
      [this, wanted]() {
        return last_status_ == wanted;
      },
      tree);
  }

  size_t goalCount()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return goals_.size();
  }

  void assertNoGoalFor(std::chrono::milliseconds duration, size_t tree = 0)
  {
    const auto count = goalCount();
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      EXPECT_EQ(trees_.at(tree)->tickExactlyOnce(), BT::NodeStatus::FAILURE);
      ASSERT_EQ(goalCount(), count);
      std::this_thread::sleep_for(5ms);
    }
  }

  RetryState retryState()
  {
    RetryState state;
    EXPECT_TRUE(board_->get(kRetryKey, state));
    return state;
  }

  void expireRetry()
  {
    auto state = retryState();
    // Skip wall-clock waiting only; keep the goal and abort history intact.
    state.retry_not_before = node_->now() - rclcpp::Duration::from_seconds(1.0);
    board_->set(kRetryKey, state);
  }

  rclcpp::Node::SharedPtr node_, server_node_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr server_;
  rclcpp::TimerBase::SharedPtr cancel_timer_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spinner_;
  BT::Blackboard::Ptr board_;
  BT::BehaviorTreeFactory factory_;
  std::vector<std::unique_ptr<BT::Tree>> trees_;
  BT::NodeStatus last_status_{BT::NodeStatus::IDLE};
  std::mutex mutex_;
  std::vector<Point2D> goals_;
  std::vector<std::shared_ptr<GoalHandle>> handles_;
};

TEST_F(NavActionTest, BackoffIsSharedAcrossActionInstances)
{
  addTree("name=\"first\" abort_backoff_initial_sec=\"0.3\"");
  addTree("name=\"second\" abort_backoff_initial_sec=\"0.3\"");
  ASSERT_TRUE(waitForGoal(1));
  completeGoal(0, rclcpp_action::ResultCode::ABORTED);
  ASSERT_TRUE(waitForStatus(BT::NodeStatus::FAILURE));
  trees_[0]->haltTree();
  assertNoGoalFor(100ms, 1);
  ASSERT_EQ(goalCount(), 1U);
  ASSERT_TRUE(waitForGoal(2, 1));
  completeGoal(1, rclcpp_action::ResultCode::ABORTED);
  ASSERT_TRUE(waitForStatus(BT::NodeStatus::FAILURE, 1));
  EXPECT_EQ(retryState().consecutive_aborts, 2U);
  EXPECT_NEAR((retryState().retry_not_before - node_->now()).seconds(), 0.6, 0.1);
}

TEST_F(NavActionTest, DefaultExponentialBackoffIsCapped)
{
  addTree();
  const std::vector<double> delays{2.0, 4.0, 8.0, 16.0, 16.0};
  for (size_t i = 0; i < delays.size(); ++i) {
    ASSERT_TRUE(waitForGoal(i + 1));
    completeGoal(i, rclcpp_action::ResultCode::ABORTED);
    ASSERT_TRUE(waitForStatus(BT::NodeStatus::FAILURE));
    const auto state = retryState();
    EXPECT_EQ(state.consecutive_aborts, i + 1);
    EXPECT_NEAR((state.retry_not_before - node_->now()).seconds(), delays[i], 0.1);
    assertNoGoalFor(20ms);
    expireRetry();
  }
  EXPECT_EQ(goalCount(), delays.size());
}

TEST_F(NavActionTest, RealTimeBackoffSuppressesGoalsUntilDeadline)
{
  addTree("abort_backoff_initial_sec=\"0.3\" abort_backoff_max_sec=\"1.0\"");
  ASSERT_TRUE(waitForGoal(1));
  completeGoal(0, rclcpp_action::ResultCode::ABORTED);
  ASSERT_TRUE(waitForStatus(BT::NodeStatus::FAILURE));
  const auto deadline = retryState().retry_not_before;
  assertNoGoalFor(100ms);
  ASSERT_EQ(goalCount(), 1U);
  ASSERT_TRUE(waitForGoal(2));
  EXPECT_GE(node_->now(), deadline);
}

TEST_F(NavActionTest, ChangedGoalSendsImmediatelyAndResetsAbortCount)
{
  addTree("abort_backoff_initial_sec=\"5.0\"");
  ASSERT_TRUE(waitForGoal(1));
  completeGoal(0, rclcpp_action::ResultCode::ABORTED);
  ASSERT_TRUE(waitForStatus(BT::NodeStatus::FAILURE));
  const auto deadline = retryState().retry_not_before;
  board_->set("nav_goal", Point2D(3.0, 4.0));
  ASSERT_TRUE(waitForGoal(2));
  EXPECT_LT(node_->now(), deadline);
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_DOUBLE_EQ(goals_[1].x, 3.0);
    EXPECT_DOUBLE_EQ(goals_[1].y, 4.0);
  }
  completeGoal(1, rclcpp_action::ResultCode::ABORTED);
  ASSERT_TRUE(waitForStatus(BT::NodeStatus::FAILURE));
  EXPECT_EQ(retryState().consecutive_aborts, 1U);
}

TEST_F(NavActionTest, InvalidBackoffParametersUseSafeDefaults)
{
  addTree("abort_backoff_initial_sec=\"{initial}\" abort_backoff_max_sec=\"{maximum}\"");
  const std::vector<double> invalid{
    0.0, -1.0, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()};
  for (size_t i = 0; i < invalid.size(); ++i) {
    board_->set("initial", invalid[i]);
    board_->set("maximum", invalid[i]);
    board_->set("nav_goal", Point2D(1.0 + i, 2.0));
    ASSERT_TRUE(waitForGoal(i + 1));
    completeGoal(i, rclcpp_action::ResultCode::ABORTED);
    ASSERT_TRUE(waitForStatus(BT::NodeStatus::FAILURE));
    EXPECT_NEAR((retryState().retry_not_before - node_->now()).seconds(), 2.0, 0.1);
  }
}

TEST_P(NavActionTest, SuccessAndCancelClearPreviousAbortHistory)
{
  addTree();
  ASSERT_TRUE(waitForGoal(1));
  completeGoal(0, rclcpp_action::ResultCode::ABORTED);
  ASSERT_TRUE(waitForStatus(BT::NodeStatus::FAILURE));
  expireRetry();
  ASSERT_TRUE(waitForGoal(2));
  ASSERT_EQ(retryState().consecutive_aborts, 1U);
  if (GetParam() == rclcpp_action::ResultCode::CANCELED) {
    trees_[0]->haltTree();
    const std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_FALSE(handles_[1]->is_active());
  } else {
    completeGoal(1, GetParam());
    ASSERT_TRUE(waitForStatus(BT::NodeStatus::SUCCESS));
  }
  EXPECT_FALSE(board_->getEntry(kRetryKey));
  ASSERT_TRUE(waitForGoal(3));
  EXPECT_EQ(goalCount(), 3U);
}

INSTANTIATE_TEST_SUITE_P(NoBackoffResults,
  NavActionTest,
  testing::Values(rclcpp_action::ResultCode::SUCCEEDED, rclcpp_action::ResultCode::CANCELED));
}  // namespace
