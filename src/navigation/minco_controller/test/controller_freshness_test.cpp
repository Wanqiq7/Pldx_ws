#include "minco_controller/minco_mpc_controller.hpp"

#include "nav2_core/controller_exceptions.hpp"
#include <functional>
#include <gtest/gtest.h>
#include <rcl/time.h>

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
    p.fixed_wz_ = 1.0;  // stale inputs must also suppress fixed gyro commands
    p.solver_ = std::make_unique<MpcSolver>(p.mpc_config_);
    p.execution_pub_ = node->create_publisher<std_msgs::msg::Bool>("test_execution", 1);
    p.global_plan_.poses.resize(1);
  }
  static void receive(MincoMpcController & p, size_t steps = 40, double speed = 1.0)
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
    reference->cmds.resize(steps);
    for (size_t i = 0; i < reference->cmds.size(); ++i) {
      reference->cmds[i].header = reference->header;
      reference->cmds[i].position.x = i * 0.05 * speed;
      reference->cmds[i].velocity.x = speed;
      reference->cmds[i].trajectory_id = 1;
    }
    p.onOptPath(reference);
  }
  static void receiveStationaryRotation(MincoMpcController & p)
  {
    auto reference = std::make_shared<ros_interfaces::msg::MpcPositionCommand>(*p.latest_opt_path_);
    for (size_t i = 0; i < reference->cmds.size(); ++i) {
      auto & cmd = reference->cmds[i];
      cmd.position.x = 0.0;
      cmd.velocity.x = 0.0;
      cmd.yaw = static_cast<double>(i) * 0.05;
      cmd.yaw_dot = 1.0;
    }
    p.use_small_gyro_mode_ = false;
    p.onOptPath(reference);
  }
  static void receiveTerminalPose(MincoMpcController & p)
  {
    auto reference = std::make_shared<ros_interfaces::msg::MpcPositionCommand>(*p.latest_opt_path_);
    reference->terminal_pose = true;
    for (auto & cmd : reference->cmds) {
      cmd.position.x = 0.01;
      cmd.velocity.x = 0.0;
      cmd.yaw = 3.0;
      cmd.yaw_dot = 0.0;
    }
    p.use_small_gyro_mode_ = false;
    p.onOptPath(reference);
  }
  static void enterTerminalAtHighAngularSpeed(MincoMpcController & p)
  {
    p.mpc_config_.omega_min = -5.0;
    p.mpc_config_.omega_max = 5.0;
    p.mpc_config_.use_acc_constraints = true;
    p.mpc_config_.alpha_min = -5.0;
    p.mpc_config_.alpha_max = 5.0;
    p.mpc_config_.dt = 0.05;
    p.applied_control_.z() = 3.0;
    p.solver_ = std::make_unique<MpcSolver>(p.mpc_config_);
    receiveTerminalPose(p);
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
  static void failPlanning(MincoMpcController & p)
  {
    auto reference = std::make_shared<ros_interfaces::msg::MpcPositionCommand>();
    reference->header.stamp = p.node_.lock()->now();
    reference->command_flag = ros_interfaces::msg::MpcPositionCommand::PLANNING_FAILED_COMMAND;
    p.onOptPath(reference);
  }
  static void setOdomYaw(MincoMpcController & p, double yaw)
  {
    auto odom = std::make_shared<nav_msgs::msg::Odometry>(*p.latest_odom_);
    odom->header.stamp = p.node_.lock()->now();
    odom->pose.pose.orientation.z = std::sin(yaw / 2.0);
    odom->pose.pose.orientation.w = std::cos(yaw / 2.0);
    p.use_small_gyro_mode_ = false;
    p.onOdom(odom);
  }
  static void expireReferenceStamp(MincoMpcController & p)
  {
    p.latest_opt_path_->header.stamp = p.node_.lock()->now() - rclcpp::Duration::from_seconds(3.0);
  }
  static void expireOdomStamp(MincoMpcController & p)
  {
    p.latest_odom_->header.stamp = p.node_.lock()->now() - rclcpp::Duration::from_seconds(1.0);
  }
  static void expireReceipt(MincoMpcController & p, bool odom)
  {
    (odom ? p.odom_received_at_ : p.reference_received_at_) -= 3s;
  }
  static void futureStamp(MincoMpcController & p)
  {
    p.latest_opt_path_->header.stamp = p.node_.lock()->now() + rclcpp::Duration::from_seconds(1.0);
  }
  static void setTunnelState(MincoMpcController & p, bool prepare, bool ready, bool escape = false)
  {
    p.tunnel_prepare_active_ = prepare;
    p.tunnel_ready_ = ready;
    p.tunnel_escape_active_ = escape;
    p.tunnel_escape_vx_ = -0.4;
    p.tunnel_escape_vy_ = 0.2;
  }
  static void receiveBehavior(MincoMpcController & p, bool prepare, bool ready, bool escape = false)
  {
    auto message = std::make_shared<ros_interfaces::msg::Behavior>();
    message->tunnel_prepare_active = prepare;
    message->tunnel_ready = ready;
    message->tunnel_escape_active = escape;
    message->tunnel_escape_vx = -0.4F;
    message->tunnel_escape_vy = 0.2F;
    p.onBehavior(message);
  }
  static void applyTunnelPolicy(MincoMpcController & p, geometry_msgs::msg::Twist & command)
  {
    p.applyTunnelCommandPolicy(command);
  }
  static bool hasReference(MincoMpcController & p)
  {
    std::vector<ReferencePoint> ref;
    return p.buildReferenceFromOptPath({}, ref);
  }
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
  }
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  MincoMpcController controller;
  geometry_msgs::msg::PoseStamped pose;
};

TEST_F(ControllerFreshnessTest, FreshInputProducesMotion)
{
  ASSERT_TRUE(MincoMpcControllerTestPeer::hasReference(controller));
  const auto cmd = controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist{}, nullptr);
  EXPECT_GT(cmd.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(cmd.twist.angular.z, 1.0);
}

TEST_F(ControllerFreshnessTest, ShortRecoveryReferenceHonorsSimulationClockAndFreshnessGuards)
{
  node->set_parameter(rclcpp::Parameter("use_sim_time", true));
  ASSERT_EQ(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()), RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(), 251000000000LL), RCL_RET_OK);
  MincoMpcControllerTestPeer::restart(controller);
  MincoMpcControllerTestPeer::receive(controller, 10, 0.1);
  const auto escape = controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist{}, nullptr);
  EXPECT_GT(escape.twist.linear.x, 0.0);

  MincoMpcControllerTestPeer::futureStamp(controller);
  expectStopped();
  MincoMpcControllerTestPeer::receive(controller, 10, 0.1);
  MincoMpcControllerTestPeer::expireReferenceStamp(controller);
  expectStopped();
  MincoMpcControllerTestPeer::receive(controller, 10, 0.1);
  MincoMpcControllerTestPeer::expireOdomStamp(controller);
  expectStopped();

  ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(), 252000000000LL), RCL_RET_OK);
  MincoMpcControllerTestPeer::receive(controller, 10, 0.1);
  const auto refreshed = controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist{}, nullptr);
  EXPECT_GT(refreshed.twist.linear.x, 0.0);
}

TEST_F(ControllerFreshnessTest, TunnelPreparationScalesXYBeforeMpcOutput)
{
  geometry_msgs::msg::Twist command;
  command.linear.x = 1.0;
  command.linear.y = -0.4;
  command.angular.z = 0.5;

  MincoMpcControllerTestPeer::setTunnelState(controller, true, false);
  MincoMpcControllerTestPeer::applyTunnelPolicy(controller, command);
  EXPECT_DOUBLE_EQ(command.linear.x, 0.15);
  EXPECT_DOUBLE_EQ(command.linear.y, -0.06);
  EXPECT_DOUBLE_EQ(command.angular.z, 0.5);

  command.linear.x = 1.0;
  command.linear.y = -0.4;
  MincoMpcControllerTestPeer::setTunnelState(controller, true, true);
  MincoMpcControllerTestPeer::applyTunnelPolicy(controller, command);
  EXPECT_DOUBLE_EQ(command.linear.x, 1.0);
  EXPECT_DOUBLE_EQ(command.linear.y, -0.4);
  EXPECT_DOUBLE_EQ(command.angular.z, 0.5);
}

TEST_F(ControllerFreshnessTest, BehaviorStateShapesComputedVelocityCommand)
{
  MincoMpcController normal_controller;
  MincoMpcController tunnel_controller;
  MincoMpcControllerTestPeer::initialize(normal_controller, node);
  MincoMpcControllerTestPeer::initialize(tunnel_controller, node);
  MincoMpcControllerTestPeer::receive(normal_controller);
  MincoMpcControllerTestPeer::receive(tunnel_controller);
  MincoMpcControllerTestPeer::receiveBehavior(tunnel_controller, true, false);

  const auto normal = normal_controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist{}, nullptr);
  const auto tunnel = tunnel_controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist{}, nullptr);
  EXPECT_NEAR(tunnel.twist.linear.x, normal.twist.linear.x * 0.15, 1e-9);
  EXPECT_NEAR(tunnel.twist.linear.y, normal.twist.linear.y * 0.15, 1e-9);
  EXPECT_DOUBLE_EQ(tunnel.twist.angular.z, normal.twist.angular.z);
}

TEST_F(ControllerFreshnessTest, TunnelEscapeReplacesXYBeforePreparationScaling)
{
  geometry_msgs::msg::Twist command;
  command.linear.x = 1.0;
  command.linear.y = -0.4;
  command.angular.z = 0.5;

  MincoMpcControllerTestPeer::setTunnelState(controller, true, false, true);
  MincoMpcControllerTestPeer::applyTunnelPolicy(controller, command);
  EXPECT_DOUBLE_EQ(command.linear.x, -0.06);
  EXPECT_DOUBLE_EQ(command.linear.y, 0.03);
  EXPECT_DOUBLE_EQ(command.angular.z, 0.5);
}

TEST_F(ControllerFreshnessTest, InvalidMpcCommandIsClearedBeforeTunnelPolicy)
{
  geometry_msgs::msg::Twist command;
  command.linear.x = std::numeric_limits<double>::quiet_NaN();
  command.linear.y = 1.0;
  command.angular.z = 0.5;

  MincoMpcControllerTestPeer::setTunnelState(controller, true, false);
  MincoMpcControllerTestPeer::applyTunnelPolicy(controller, command);
  EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(command.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(command.angular.z, 0.0);
}

TEST_F(ControllerFreshnessTest, StationaryRotationAdvancesWhilePositionIsUnchanged)
{
  node->set_parameter(rclcpp::Parameter("use_sim_time", true));
  ASSERT_EQ(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()), RCL_RET_OK);
  const auto set_time = [&](int64_t nanoseconds) {
    ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(), nanoseconds), RCL_RET_OK);
  };
  set_time(10000000000LL);
  MincoMpcControllerTestPeer::restart(controller);
  MincoMpcControllerTestPeer::receiveStationaryRotation(controller);
  const auto initial = controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist{}, nullptr);
  set_time(10500000000LL);
  MincoMpcControllerTestPeer::setOdomYaw(controller, 0.0);
  const auto advanced = controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist{}, nullptr);
  EXPECT_GT(advanced.twist.angular.z, initial.twist.angular.z + 0.1);
  EXPECT_NEAR(advanced.twist.linear.x, 0.0, 1e-6);
  EXPECT_NEAR(advanced.twist.linear.y, 0.0, 1e-6);
}

TEST_F(ControllerFreshnessTest, TerminalPoseBoundsRotationAndStillHonorsBlockAndCancel)
{
  MincoMpcControllerTestPeer::receiveTerminalPose(controller);
  const auto cmd = controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist{}, nullptr);
  EXPECT_GT(cmd.twist.angular.z, 0.0);
  EXPECT_LE(cmd.twist.angular.z, 1.0);
  MincoMpcControllerTestPeer::block(controller);
  expectStopped();
  MincoMpcControllerTestPeer::receive(controller);
  MincoMpcControllerTestPeer::receiveTerminalPose(controller);
  ASSERT_TRUE(controller.cancel());
  expectStopped();
}

TEST_F(ControllerFreshnessTest, TerminalRotationDeceleratesWithinConfiguredAcceleration)
{
  MincoMpcControllerTestPeer::enterTerminalAtHighAngularSpeed(controller);
  const auto cmd = controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist{}, nullptr);
  EXPECT_GE(cmd.twist.angular.z, 2.75 - 1e-6);
  EXPECT_LE(cmd.twist.angular.z, 3.25 + 1e-6);
}

TEST_F(ControllerFreshnessTest, UsesOdomYawWhenNav2PoseHasZeroYaw)
{
  MincoMpcControllerTestPeer::setOdomYaw(controller, 1.0);
  const auto cmd = controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist{}, nullptr);
  EXPECT_LT(cmd.twist.angular.z, 0.0);
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

TEST_F(ControllerFreshnessTest, PlanningFailureStopsAndFailsFollowPath)
{
  MincoMpcControllerTestPeer::failPlanning(controller);
  EXPECT_FALSE(MincoMpcControllerTestPeer::hasReference(controller));
  EXPECT_THROW(controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist{}, nullptr),
    nav2_core::NoValidControl);
}

TEST_F(ControllerFreshnessTest, SpeedLimitChangeDuringComputationDiscardsCommand)
{
  CallbackGoalChecker checker([&] {
    controller.setSpeedLimit(0.2, false);
  });
  expectStopped(&checker);
  const auto limited = controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist{}, nullptr);
  EXPECT_LE(std::hypot(limited.twist.linear.x, limited.twist.linear.y), 0.2 + 1e-9);
  EXPECT_GT(limited.twist.linear.x, 0.0);
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
  EXPECT_GT(
    controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist{}, nullptr).twist.linear.x, 0.0);
}
}  // namespace
}  // namespace minco_controller
