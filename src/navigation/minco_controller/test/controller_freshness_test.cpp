#include "minco_controller/minco_mpc_controller.hpp"

#include <cmath>
#include <functional>
#include <limits>
#include <thread>

#include <gtest/gtest.h>

#include "nav2_core/controller_exceptions.hpp"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

namespace minco_controller {
class MincoMpcControllerTestPeer
{
public:
  static void initialize(MincoMpcController & p, const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
  {
    p.node_ = node;
    p.global_frame_ = "map";
    p.execution_enabled_ = true;
    p.execution_start_ = node->now();
    p.dynamic_solver_ = std::make_unique<DynamicMpcSolver>(p.dynamic_config_);
    p.execution_pub_ = node->create_publisher<std_msgs::msg::Bool>("test_execution", 1);
    auto publisher = node->create_publisher<ros_interfaces::msg::ChassisCommand>(
      "test_chassis_command", rclcpp::SensorDataQoS());
    publisher->on_activate();
    p.chassis_cmd_pub_ = publisher;
    p.global_plan_.poses.resize(1);
  }
  static void receive(MincoMpcController & p)
  {
    auto node = p.node_.lock();
    auto odom = std::make_shared<nav_msgs::msg::Odometry>();
    odom->header.stamp = node->now();
    odom->pose.pose.orientation.w = 1.0;
    p.onOdom(odom);
    auto reference = std::make_shared<ros_interfaces::msg::MpcPositionCommand>();
    reference->header.frame_id = "map";
    reference->header.stamp = node->now();
    reference->command_flag = ros_interfaces::msg::MpcPositionCommand::NORMAL_COMMAND;
    reference->cmds.resize(40);
    for (size_t i = 0; i < reference->cmds.size(); ++i) {
      reference->cmds[i].header = reference->header;
      reference->cmds[i].position.x = i * 0.05;
      reference->cmds[i].velocity.x = 1.0;
      reference->cmds[i].trajectory_id = 1;
    }
    p.onOptPath(reference);
  }
  static void receiveOdomOnly(MincoMpcController & p)
  {
    auto odom = std::make_shared<nav_msgs::msg::Odometry>();
    odom->header.stamp = p.node_.lock()->now();
    odom->pose.pose.orientation.w = 1.0;
    p.onOdom(odom);
  }
  static void refreshReference(MincoMpcController & p)
  {
    auto reference = std::make_shared<ros_interfaces::msg::MpcPositionCommand>(*p.latest_opt_path_);
    reference->header.stamp = p.node_.lock()->now();
    p.onOptPath(reference);
  }
  static void replaceReference(MincoMpcController & p)
  {
    auto reference = std::make_shared<ros_interfaces::msg::MpcPositionCommand>(*p.latest_opt_path_);
    reference->header.stamp = p.node_.lock()->now();
    for (auto & cmd : reference->cmds) {
      ++cmd.trajectory_id;
    }
    p.onOptPath(reference);
  }
  static void block(MincoMpcController & p)
  {
    auto reference = std::make_shared<ros_interfaces::msg::MpcPositionCommand>();
    reference->header.stamp = p.node_.lock()->now();
    reference->command_flag = ros_interfaces::msg::MpcPositionCommand::BLOCK_COMMAND;
    p.onOptPath(reference);
  }
  static void planningFailed(MincoMpcController & p)
  {
    auto notice = std::make_shared<ros_interfaces::msg::MpcPositionCommand>();
    notice->header.stamp = p.node_.lock()->now();
    notice->command_flag = ros_interfaces::msg::MpcPositionCommand::PLANNING_FAILED_COMMAND;
    p.onOptPath(notice);
  }
  static void expireReferenceStamp(MincoMpcController & p)
  {
    p.latest_opt_path_->header.stamp = p.node_.lock()->now() - rclcpp::Duration::from_seconds(3.0);
  }
  static void expireOdomStamp(MincoMpcController & p)
  {
    p.latest_odom_->header.stamp = p.node_.lock()->now() - rclcpp::Duration::from_seconds(1.0);
  }
  static void expireReceipt(MincoMpcController & p, bool odom, std::chrono::seconds age = 3s)
  {
    (odom ? p.odom_received_at_ : p.reference_received_at_) -= age;
  }
  static void futureStamp(MincoMpcController & p)
  {
    p.latest_opt_path_->header.stamp = p.node_.lock()->now() + rclcpp::Duration::from_seconds(1.0);
  }
  static bool hasReference(MincoMpcController & p)
  {
    MincoMpcController::ReferenceTrajectory traj;
    return p.prepareReference({}, traj);
  }
  // map -> camera_init as in the rmuc_2025 simulation; the plan ends at goal in map.
  static void useOffsetFrames(
    MincoMpcController & p, const Eigen::Vector2d & goal_in_map, bool publish_tf = true)
  {
    auto node = p.node_.lock();
    p.global_frame_ = "camera_init";
    p.tf_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    if (publish_tf) {
      geometry_msgs::msg::TransformStamped t;
      t.header.frame_id = "map";
      t.child_frame_id = "camera_init";
      t.transform.translation.x = 4.24;
      t.transform.translation.y = 7.09;
      t.transform.rotation.z = std::sin(0.039 / 2.0);
      t.transform.rotation.w = std::cos(0.039 / 2.0);
      p.tf_->setTransform(t, "test", true);
    }
    nav_msgs::msg::Path plan;
    plan.header.frame_id = "map";
    plan.poses.resize(1);
    plan.poses[0].pose.position.x = goal_in_map.x();
    plan.poses[0].pose.position.y = goal_in_map.y();
    plan.poses[0].pose.orientation.w = 1.0;
    std::lock_guard<std::mutex> lk(p.plan_mtx_);
    p.global_plan_ = plan;
  }
  static Eigen::Vector3d pendingWrench(const MincoMpcController & p) { return p.pending_wrench_; }
  static void stationaryReferenceWhileMoving(MincoMpcController & p)
  {
    auto reference = std::make_shared<ros_interfaces::msg::MpcPositionCommand>(*p.latest_opt_path_);
    reference->header.stamp = p.node_.lock()->now();
    for (auto & cmd : reference->cmds) {
      cmd.position.x = 0.0;
      cmd.velocity.x = 0.0;
    }
    p.onOptPath(reference);
    auto odom = std::make_shared<nav_msgs::msg::Odometry>(*p.latest_odom_);
    odom->header.stamp = p.node_.lock()->now();
    odom->twist.twist.linear.x = 0.5;
    p.onOdom(odom);
  }
  // Clears the committed input so consecutive cycles start from the same solver state.
  static void resetSolverHistory(MincoMpcController & p) { p.dynamic_solver_->resetLastAcceleration(); }
  static double mass(const MincoMpcController & p) { return p.dynamic_solver_->config().mass; }
  static void enableWheelOdometry(MincoMpcController & p) { p.wheel_odometry_enabled_ = true; }
  static void pushWheel(MincoMpcController & p, const rclcpp::Time & stamp, double vx)
  {
    auto msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
    msg->header.stamp = stamp;
    msg->twist.linear.x = vx;
    p.onWheelOdometry(msg);
  }
  static uint64_t wheelFallbacks(const MincoMpcController & p) { return p.wheel_odometry_fallbacks_; }
  static rclcpp::Time odomStamp(const MincoMpcController & p) { return p.latest_odom_->header.stamp; }
  static void restart(MincoMpcController & p)
  {
    p.reset();
    nav_msgs::msg::Path path;
    path.poses.resize(1);
    p.setPlan(path);
    receive(p);
  }
};

namespace {
class CallbackGoalChecker : public nav2_core::GoalChecker
{
public:
  explicit CallbackGoalChecker(std::function<void()> callback) : callback_(std::move(callback)) {}
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    const std::string &,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override
  {
  }
  void reset() override {}
  bool isGoalReached(const geometry_msgs::msg::Pose &,
    const geometry_msgs::msg::Pose &,
    const geometry_msgs::msg::Twist &) override
  {
    callback_();
    return false;
  }
  bool getTolerances(geometry_msgs::msg::Pose &, geometry_msgs::msg::Twist &) override { return false; }

private:
  std::function<void()> callback_;
};

// Position-only goal check that records the distance it was given.
class DistanceGoalChecker : public nav2_core::GoalChecker
{
public:
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    const std::string &,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override
  {
  }
  void reset() override {}
  bool isGoalReached(const geometry_msgs::msg::Pose & pose,
    const geometry_msgs::msg::Pose & goal,
    const geometry_msgs::msg::Twist &) override
  {
    ++calls;
    distance = std::hypot(pose.position.x - goal.position.x, pose.position.y - goal.position.y);
    return distance < 0.1;
  }
  bool getTolerances(geometry_msgs::msg::Pose &, geometry_msgs::msg::Twist &) override { return false; }
  int calls{0};
  double distance{std::numeric_limits<double>::quiet_NaN()};
};

class ControllerFreshnessTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override
  {
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_freshness_test");
    MincoMpcControllerTestPeer::initialize(controller, node);
    MincoMpcControllerTestPeer::receive(controller);
    pose.pose.orientation.w = 1.0;
  }
  void expectStopped(nav2_core::GoalChecker * checker = nullptr)
  {
    const auto cmd = controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist{}, checker);
    EXPECT_DOUBLE_EQ(cmd.twist.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(cmd.twist.linear.y, 0.0);
    EXPECT_DOUBLE_EQ(cmd.twist.angular.z, 0.0);
    EXPECT_EQ(MincoMpcControllerTestPeer::pendingWrench(controller), Eigen::Vector3d::Zero());
  }
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  MincoMpcController controller;
  geometry_msgs::msg::PoseStamped pose;
};

TEST_F(ControllerFreshnessTest, FreshInputProducesMotion)
{
  ASSERT_TRUE(MincoMpcControllerTestPeer::hasReference(controller));
  const auto cmd = controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist{}, nullptr);
  EXPECT_DOUBLE_EQ(cmd.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(cmd.twist.angular.z, 0.0);
  EXPECT_GT(MincoMpcControllerTestPeer::pendingWrench(controller).x(), 0.0);
}

TEST_F(ControllerFreshnessTest, StationaryReferenceBrakesMovingChassis)
{
  MincoMpcControllerTestPeer::stationaryReferenceWhileMoving(controller);
  controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist{}, nullptr);
  EXPECT_LT(MincoMpcControllerTestPeer::pendingWrench(controller).x(), 0.0);
}

TEST_F(ControllerFreshnessTest, PublishesForceAndCancelPublishesZero)
{
  auto receiver = std::make_shared<rclcpp::Node>("force_command_receiver");
  std::vector<ros_interfaces::msg::ChassisCommand> received;
  auto subscription =
    receiver->create_subscription<ros_interfaces::msg::ChassisCommand>("test_chassis_command",
      rclcpp::SensorDataQoS(),
      [&](ros_interfaces::msg::ChassisCommand::ConstSharedPtr msg) {
        received.push_back(*msg);
      });
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (received.empty() && std::chrono::steady_clock::now() < deadline) {
    MincoMpcControllerTestPeer::receive(controller);
    controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist{}, nullptr);
    rclcpp::spin_some(receiver);
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_FALSE(received.empty());
  EXPECT_EQ(received.back().header.frame_id, "map");
  EXPECT_GT(received.back().wrench.force.x, 0.0);
  received.clear();
  controller.cancel();
  const auto cancel_deadline = std::chrono::steady_clock::now() + 2s;
  while ((received.empty() || received.back().wrench.force.x != 0.0) &&
         std::chrono::steady_clock::now() < cancel_deadline) {
    rclcpp::spin_some(receiver);
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_FALSE(received.empty());
  EXPECT_DOUBLE_EQ(received.back().wrench.force.x, 0.0);
  EXPECT_DOUBLE_EQ(received.back().wrench.force.y, 0.0);
  EXPECT_DOUBLE_EQ(received.back().wrench.torque.z, 0.0);
}

TEST_F(ControllerFreshnessTest, ReferenceUpdateDuringComputationDiscardsPreviousCommand)
{
  CallbackGoalChecker checker([&] {
    MincoMpcControllerTestPeer::refreshReference(controller);
  });
  expectStopped(&checker);
  EXPECT_TRUE(MincoMpcControllerTestPeer::hasReference(controller));
}

TEST_F(ControllerFreshnessTest, ReplacementDuringComputationDiscardsPreviousCommand)
{
  CallbackGoalChecker checker([&] {
    MincoMpcControllerTestPeer::replaceReference(controller);
  });
  expectStopped(&checker);
  EXPECT_TRUE(MincoMpcControllerTestPeer::hasReference(controller));
}

TEST_F(ControllerFreshnessTest, BlockDuringComputationDiscardsCommand)
{
  CallbackGoalChecker checker([&] {
    MincoMpcControllerTestPeer::block(controller);
  });
  expectStopped(&checker);
  EXPECT_FALSE(MincoMpcControllerTestPeer::hasReference(controller));
}

TEST_F(ControllerFreshnessTest, StaleReferenceAndOdomSuppressAllMotion)
{
  MincoMpcControllerTestPeer::expireReferenceStamp(controller);
  EXPECT_FALSE(MincoMpcControllerTestPeer::hasReference(controller));
  expectStopped();
  MincoMpcControllerTestPeer::receive(controller);
  MincoMpcControllerTestPeer::expireOdomStamp(controller);
  EXPECT_FALSE(MincoMpcControllerTestPeer::hasReference(controller));
  expectStopped();
}

TEST_F(ControllerFreshnessTest, SteadyTimeoutWorksWithoutRosTimeAdvancing)
{
  MincoMpcControllerTestPeer::expireReceipt(controller, false);
  expectStopped();
  MincoMpcControllerTestPeer::receive(controller);
  MincoMpcControllerTestPeer::expireReceipt(controller, true);
  expectStopped();
}

TEST_F(ControllerFreshnessTest, ReceiptExpiryRejectsFreshMessageStamps)
{
  MincoMpcControllerTestPeer::expireReceipt(controller, true, 3s);
  expectStopped();
  MincoMpcControllerTestPeer::receive(controller);
  MincoMpcControllerTestPeer::expireReceipt(controller, false, 8s);
  expectStopped();
}

TEST_F(ControllerFreshnessTest, ReceiptExpiryDuringComputationDiscardsCommand)
{
  CallbackGoalChecker checker([&] {
    MincoMpcControllerTestPeer::expireReceipt(controller, true, 3s);
  });
  expectStopped(&checker);
}

TEST_F(ControllerFreshnessTest, FutureDatedReferenceIsRejected)
{
  MincoMpcControllerTestPeer::futureStamp(controller);
  EXPECT_FALSE(MincoMpcControllerTestPeer::hasReference(controller));
  expectStopped();
}

TEST_F(ControllerFreshnessTest, RestartDuringComputationDiscardsPreviousCommand)
{
  CallbackGoalChecker checker([&] {
    MincoMpcControllerTestPeer::restart(controller);
  });
  // New inputs are valid and the solver can succeed, but the in-flight call
  // belongs to the execution that reset() cancelled.
  expectStopped(&checker);
  EXPECT_TRUE(MincoMpcControllerTestPeer::hasReference(controller));
  controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist{}, nullptr);
  EXPECT_GT(MincoMpcControllerTestPeer::pendingWrench(controller).x(), 0.0);
}
TEST_F(ControllerFreshnessTest, PlanningFailureFailsFollowPathUntilNextExecution)
{
  MincoMpcControllerTestPeer::planningFailed(controller);
  EXPECT_FALSE(MincoMpcControllerTestPeer::hasReference(controller));
  EXPECT_THROW(controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist{}, nullptr),
    nav2_core::NoValidControl);
  // The failure is latched for this execution even if odometry keeps arriving.
  MincoMpcControllerTestPeer::receiveOdomOnly(controller);
  EXPECT_THROW(controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist{}, nullptr),
    nav2_core::NoValidControl);
  // A new FollowPath execution (reset + setPlan) starts clean.
  MincoMpcControllerTestPeer::restart(controller);
  controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist{}, nullptr);
  EXPECT_GT(MincoMpcControllerTestPeer::pendingWrench(controller).x(), 0.0);
}

TEST_F(ControllerFreshnessTest, WheelOdometryIsUsedWhenFreshAndFallsBackOtherwise)
{
  MincoMpcControllerTestPeer::enableWheelOdometry(controller);
  // No samples yet: falls back to the LIO state and still moves.
  controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist{}, nullptr);
  EXPECT_GT(MincoMpcControllerTestPeer::pendingWrench(controller).x(), 0.0);
  EXPECT_EQ(MincoMpcControllerTestPeer::wheelFallbacks(controller), 1u);
  // Samples covering the odom stamp up to now are used.
  const rclcpp::Time t0 = MincoMpcControllerTestPeer::odomStamp(controller);
  for (int i = -2; i <= 2; ++i) {
    MincoMpcControllerTestPeer::pushWheel(controller, t0 + rclcpp::Duration::from_seconds(0.005 * i), 0.2);
  }
  MincoMpcControllerTestPeer::pushWheel(controller, node->now(), 0.2);
  controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist{}, nullptr);
  EXPECT_GT(MincoMpcControllerTestPeer::pendingWrench(controller).x(), 0.0);
  EXPECT_EQ(MincoMpcControllerTestPeer::wheelFallbacks(controller), 1u);
  // Stale samples fall back again.
  std::this_thread::sleep_for(80ms);
  MincoMpcControllerTestPeer::receive(controller);
  controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist{}, nullptr);
  EXPECT_EQ(MincoMpcControllerTestPeer::wheelFallbacks(controller), 2u);
}

TEST_F(ControllerFreshnessTest, FirstOptimizedWrenchDrivesFromRestAndStopsWithStaleInput)
{
  MincoMpcControllerTestPeer::resetSolverHistory(controller);
  const auto cmd = controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist{}, nullptr);
  EXPECT_DOUBLE_EQ(cmd.twist.linear.x, 0.0);
  const auto wrench = MincoMpcControllerTestPeer::pendingWrench(controller);
  EXPECT_GT(wrench.x(), 0.0);
  EXPECT_LE(wrench.head<2>().norm(), 60.0);
  MincoMpcControllerTestPeer::expireOdomStamp(controller);
  expectStopped();
}

TEST_F(ControllerFreshnessTest, SpeedLimitConstrainsPredictionAndRejectsInvalidLimits)
{
  controller.setSpeedLimit(0.1, false);
  controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist{}, nullptr);
  const auto limited = MincoMpcControllerTestPeer::pendingWrench(controller);
  EXPECT_GT(limited.x(), 0.0);
  controller.setSpeedLimit(0.0, false);
  MincoMpcControllerTestPeer::resetSolverHistory(controller);
  controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist{}, nullptr);
  EXPECT_GT(MincoMpcControllerTestPeer::pendingWrench(controller).x(), limited.x());
  EXPECT_THROW(controller.setSpeedLimit(NAN, false), std::invalid_argument);
}

TEST_F(ControllerFreshnessTest, GoalCheckComparesInThePoseFrame)
{
  const Eigen::Vector2d goal(4.7, 10.9);
  MincoMpcControllerTestPeer::useOffsetFrames(controller, goal);
  DistanceGoalChecker checker;
  // Standing on the raw map coordinates of the goal is 8.3 m away from it.
  pose.header.frame_id = "camera_init";
  pose.pose.position.x = goal.x();
  pose.pose.position.y = goal.y();
  controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist{}, &checker);
  EXPECT_EQ(checker.calls, 1);
  EXPECT_GT(checker.distance, 8.0);
  EXPECT_TRUE(MincoMpcControllerTestPeer::hasReference(controller));
  // The goal expressed in camera_init (inverse of the map -> camera_init transform) is
  // reached and stops the vehicle.
  const double c = std::cos(0.039);
  const double s = std::sin(0.039);
  const Eigen::Vector2d d = goal - Eigen::Vector2d(4.24, 7.09);
  pose.pose.position.x = c * d.x() + s * d.y();
  pose.pose.position.y = -s * d.x() + c * d.y();
  expectStopped(&checker);
  EXPECT_EQ(checker.calls, 2);
  EXPECT_NEAR(checker.distance, 0.0, 1e-9);
  EXPECT_FALSE(MincoMpcControllerTestPeer::hasReference(controller));
}

TEST_F(ControllerFreshnessTest, GoalCheckIsSkippedWithoutTransform)
{
  // Without map -> camera_init neither the goal nor the map-frame reference can be used:
  // the goal check must not run on untransformed coordinates and the command is zero.
  MincoMpcControllerTestPeer::useOffsetFrames(controller, Eigen::Vector2d(4.7, 10.9), false);
  DistanceGoalChecker checker;
  pose.header.frame_id = "camera_init";
  expectStopped(&checker);
  EXPECT_EQ(checker.calls, 0);
}
}  // namespace
}  // namespace minco_controller
