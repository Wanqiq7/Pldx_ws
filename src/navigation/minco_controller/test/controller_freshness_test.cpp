#include "minco_controller/minco_mpc_controller.hpp"

#include <functional>
#include <gtest/gtest.h>

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
