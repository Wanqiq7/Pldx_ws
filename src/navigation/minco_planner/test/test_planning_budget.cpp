#include "minco_core/minco_planner.hpp"
#include "minco_core/planning_deadline.hpp"

#include <algorithm>
#include <future>
#include <gtest/gtest.h>
#include <numeric>
#include <rcl/time.h>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

namespace minco_planner {

class MincoPlannerTestPeer
{
public:
  static void initialize(
    MincoPlanner & p, const nav2_util::LifecycleNode::SharedPtr & node, bool start_worker = true)
  {
    p.node_ = node;
    p.output_frame_ = p.planning_frame_ = "map";
    p.active_ = p.execution_enabled_ = true;
    auto publisher = node->create_publisher<ros_interfaces::msg::MpcPositionCommand>("test_reference", 1);
    publisher->on_activate();
    p.opt_path_pub_ = publisher;
    if (start_worker) {
      p.startPlanningWorker();
    }
  }

  static void readyCandidate(MincoPlanner & p,
    bool old = false,
    double duration = 1.0,
    double velocity = 0.0,
    double acceleration = 0.0,
    double y = 0.0)
  {
    Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3, 6);
    coefficients(0, 5) = 1.0;
    coefficients(0, 4) = velocity;
    coefficients(0, 3) = 0.5 * acceleration;
    coefficients(1, 5) = y;
    traj_opt::Trajectory traj;
    traj.emplace_back(duration, coefficients);
    std_msgs::msg::Header header;
    header.frame_id = p.output_frame_;
    auto command = utils::sampleOptimizedTrajectory(
      traj, traj, 0, header, static_cast<int>(std::ceil(duration / 0.05)) + 1, 0.05);
    std::lock_guard<std::mutex> lock(p.planning_mutex_);
    p.pending_planning_request_.reset();
    p.candidate_ = MincoPlanner::Candidate{traj, traj, command};
    p.computing_version_ = p.planning_version_.load();
    p.computed_at_ = std::chrono::steady_clock::now() - (old ? 200ms : 0ms);
    p.planning_result_ = true;
  }

  static void followTrajectory(MincoPlanner & p, double duration = 5.0)
  {
    p.latest_odom_.header.frame_id = "map";
    p.latest_odom_.pose.pose.position.x = 1.0;
    p.latest_odom_.pose.pose.orientation.w = 1.0;
    p.has_latest_odom_ = true;
    p.recovery_server_ = std::make_shared<RecoverServer>();
    p.planner_handle_ = MincoPlanner::Ptr(&p, [](MincoPlanner *) {
    });
    p.fsm_ = std::make_unique<MincoFsm>(p.planner_handle_, p.recovery_server_);
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.pose.position.x = 10.0;
    goal.pose.orientation.w = 1.0;
    ASSERT_EQ(p.createPlan(goal, goal, {}).poses.size(), 2U);
    p.fsm_->callMainFsmOnce();
    readyCandidate(p, false, duration);
    p.fsm_->callMainFsmOnce();
    ASSERT_EQ(p.fsm_->getState(), MincoFsm::State::FOLLOW_TRAJ);
  }
  static void tick(MincoPlanner & p) { p.fsm_->callMainFsmOnce(); }
  static std::optional<bool> requestedGlobalSearch(MincoPlanner & p)
  {
    if (!p.pending_planning_request_) {
      return std::nullopt;
    }
    return p.pending_planning_request_->global_search;
  }
  static void setRobotX(MincoPlanner & p, double x) { p.latest_odom_.pose.pose.position.x = x; }
  static bool usesColdStart(MincoPlanner & p)
  {
    return p.determinePlanningState(p.latest_odom_.pose.pose, {}, p.last_traj_, true) ==
           MincoPlanner::PlanningState::COLD_START;
  }
  static void markUnsafe(MincoPlanner & p) { p.is_traj_safe_.store(false); }
  static void setLeadTime(MincoPlanner & p, double seconds) { p.replan_lead_time_sec_ = seconds; }
  static double trajectoryStart(const MincoPlanner & p) { return p.last_traj_.start_WT; }
  static auto command(const MincoPlanner & p) { return p.last_command_; }
  static void keepaliveDue(MincoPlanner & p) { p.last_reference_publish_at_ -= 1s; }
  static auto lastPublishTime(const MincoPlanner & p) { return p.last_reference_publish_at_; }

  static bool hasTrajectory(const MincoPlanner & p) { return p.has_last_traj_; }
  static void enable(MincoPlanner & p) { p.execution_enabled_ = true; }
  static void startCallbacks(MincoPlanner & p, const nav2_util::LifecycleNode::SharedPtr & node)
  {
    p.configureControlCallbacks(node);
    p.fsm_timer_->reset();
    p.safety_timer_->reset();
  }
  static bool stopped(MincoPlanner & p)
  {
    std::lock_guard<std::recursive_mutex> lock(p.control_mutex_);
    return !p.execution_enabled_ && !p.has_last_traj_;
  }

  static std::future<void> longComputation(MincoPlanner & p)
  {
    std::promise<void> started;
    auto ready = started.get_future();
    std::thread([&p, started = std::move(started)]() mutable {
      std::unique_lock<std::mutex> computation_lock(p.computation_mutex_);
      started.set_value();
      (void)p.planAsync(geometry_msgs::msg::PoseStamped{}, geometry_msgs::msg::PoseStamped{}, false);
      const auto version = p.planning_version_.load();
      while (version == p.planning_version_.load()) {
        std::this_thread::sleep_for(1ms);
      }
    }).detach();
    return ready;
  }
  static bool workerRunning(MincoPlanner & p)
  {
    std::lock_guard<std::mutex> lock(p.planning_mutex_);
    return p.planning_worker_busy_ || p.pending_planning_request_.has_value();
  }
  static bool workerStopped(MincoPlanner & p)
  {
    const auto deadline = std::chrono::steady_clock::now() + 100ms;
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(p.planning_mutex_);
        if (!p.planning_worker_busy_ && !p.pending_planning_request_.has_value()) {
          return true;
        }
      }
      std::this_thread::sleep_for(1ms);
    }
    return false;
  }

  static void installSafety(MincoPlanner & p, const std::shared_ptr<rog_map::MapQueryInterface> & query)
  {
    p.safety_checker_ = std::make_unique<TrajectorySafetyChecker>();
    p.safety_checker_->configure(0.1, 0.05, p.logger_);
    p.safety_checker_->setQuery(query);
    p.safety_budget_sec_ = 0.005;
  }

  static traj_opt::Trajectory optimizeStraightSegment(MincoPlanner & p,
    const std::shared_ptr<rog_map::MapQueryInterface> & query,
    double lookahead,
    double current_x = 1.0)
  {
    p.traj_goal_tolerance_ = 0.3;
    p.minco_config.max_vel = 3.0;
    p.minco_config.max_acc = 2.0;
    p.minco_config.safe_dist = 0.35;
    p.minco_config.rho = 100.0;
    p.minco_config.magnitudeBounds = Eigen::Vector3d(0.35, 3.0, 2.0);
    p.minco_config.penaltyWeights = Eigen::VectorXd(5);
    p.minco_config.penaltyWeights << 50000.0, 1500.0, 1500.0, 100.0, 50.0;
    p.minco_config.time_allocation_iters = 15;
    p.minco_config.opt_accuracy = 1e-3;
    p.minco_config.print_optimizer_log = false;
    p.minco_optimizer_ = std::make_unique<MincoOptimizer>(p.minco_config);
    p.minco_optimizer_->setMap(query);
    p.mode_params_.planner_mode = "EXPLORATION";
    p.mode_params_.rog_frame = "map";
    p.mode_params_.exploration_boundary_margin = 0.0;
    p.mode_context_ = std::make_unique<PlannerModeContext>();
    p.mode_context_->configure(p.mode_params_, query, nullptr, nullptr, p.logger_);
    p.local_path_processor_ = std::make_unique<LocalPathProcessor>();
    p.local_path_processor_->configure(lookahead, 3.0, 2.0, 0.3, p.logger_);
    installSafety(p, query);
    p.use_yaw_opt_ = false;
    p.latest_odom_.pose.pose.orientation.w = 1.0;
    p.latest_odom_.twist.twist.linear.x = 1.0;
    p.has_latest_odom_ = true;
    for (int i = 0; i <= 50; ++i) {
      geometry_msgs::msg::PoseStamped point;
      point.pose.position.x = 1.0 + 0.1 * i;
      point.pose.position.y = 1.0;
      p.latest_global_path_.push_back(point);
    }
    geometry_msgs::msg::PoseStamped current;
    current.pose.position.x = current_x;
    current.pose.position.y = 1.0;
    if (!p.ReplanLocal(current) || !p.candidate_) {
      return {};
    }
    return p.candidate_->position;
  }
};

namespace {

double percentileMs(std::vector<double> samples, double percentile)
{
  std::sort(samples.begin(), samples.end());
  const double rank = percentile * static_cast<double>(samples.size() - 1);
  const auto lower = static_cast<size_t>(rank);
  const auto upper = std::min(lower + 1, samples.size() - 1);
  const double fraction = rank - static_cast<double>(lower);
  return samples[lower] + fraction * (samples[upper] - samples[lower]);
}
class CountingQuery : public Nav2CostmapQuery
{
public:
  explicit CountingQuery(nav2_costmap_2d::Costmap2D * costmap) : Nav2CostmapQuery(costmap) {}
  rog_map::QueryResult query(const Eigen::Vector3d & pos) const override
  {
    ++calls;
    return Nav2CostmapQuery::query(pos);
  }
  mutable std::atomic_int calls{0};
};

class SlowQuery : public Nav2CostmapQuery
{
public:
  explicit SlowQuery(nav2_costmap_2d::Costmap2D * costmap) : Nav2CostmapQuery(costmap) {}
  rog_map::QueryResult query(const Eigen::Vector3d &) const override
  {
    ++calls;
    std::this_thread::sleep_for(1ms);
    rog_map::QueryResult result;
    result.ok = true;
    result.status = rog_map::QueryStatus::OK;
    result.distance = 5.0;
    result.gradient.setZero();
    return result;
  }
  mutable int calls{0};
};

class ClearanceQuery : public Nav2CostmapQuery
{
public:
  explicit ClearanceQuery(nav2_costmap_2d::Costmap2D * costmap) : Nav2CostmapQuery(costmap) {}
  rog_map::QueryResult query(const Eigen::Vector3d & pos) const override
  {
    auto result = Nav2CostmapQuery::query(pos);
    if (result.ok) {
      result.distance = 5.0;
    }
    return result;
  }
};

class PlanningBudgetTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void setTime(double seconds)
  {
    node->set_parameter(rclcpp::Parameter("use_sim_time", true));
    ASSERT_EQ(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(
                node->get_clock()->get_clock_handle(), static_cast<rcl_time_point_value_t>(seconds * 1e9)),
      RCL_RET_OK);
    ASSERT_NEAR(node->now().seconds(), seconds, 1e-9);
  }
  nav2_util::LifecycleNode::SharedPtr node = std::make_shared<nav2_util::LifecycleNode>("budget_test");
};

TEST_F(PlanningBudgetTest, SafeTrajectoryDoesNotReplanEverySecond)
{
  setTime(10.0);
  MincoPlanner p;
  MincoPlannerTestPeer::initialize(p, node, false);
  MincoPlannerTestPeer::followTrajectory(p);
  for (double time : {10.05, 11.1, 12.2, 13.3, 14.1}) {
    setTime(time);
    MincoPlannerTestPeer::tick(p);
    EXPECT_FALSE(p.planningInProgress());
  }
  setTime(14.21);
  EXPECT_FALSE(p.isTrajectoryTimeExpired(p.nowSeconds()));
  MincoPlannerTestPeer::tick(p);
  EXPECT_EQ(MincoPlannerTestPeer::requestedGlobalSearch(p), std::optional<bool>(false));
}

TEST_F(PlanningBudgetTest, ContinuationLeadCoversSchedulingBudgets)
{
  setTime(10.0);
  MincoPlanner p;
  MincoPlannerTestPeer::initialize(p, node, false);
  MincoPlannerTestPeer::followTrajectory(p);
  MincoPlannerTestPeer::setLeadTime(p, 0.01);
  setTime(14.85);  // 150 ms remaining, above the 135 ms scheduling floor.
  MincoPlannerTestPeer::tick(p);
  EXPECT_FALSE(p.planningInProgress());
  setTime(14.87);  // 130 ms remaining: poll + compute + commit require renewal.
  MincoPlannerTestPeer::tick(p);
  EXPECT_EQ(MincoPlannerTestPeer::requestedGlobalSearch(p), std::optional<bool>(false));
}

TEST_F(PlanningBudgetTest, RollingContinuationStartsBeforeMpcReadsTerminalHold)
{
  setTime(10.0);
  MincoPlanner p;
  MincoPlannerTestPeer::initialize(p, node, false);
  MincoPlannerTestPeer::readyCandidate(p, false, 5.0, 1.0);
  ASSERT_EQ(p.planAsync(geometry_msgs::msg::PoseStamped{}, geometry_msgs::msg::PoseStamped{}, false),
    std::optional<bool>(true));
  setTime(14.21);  // 790 ms left: renew before MPC's 550 ms coverage plus submission delay.
  geometry_msgs::msg::PoseStamped current;
  current.pose.position.x = 5.21;
  EXPECT_TRUE(p.needsLocalReplan(current));
}

TEST_F(PlanningBudgetTest, RollingContinuationAccountsForRobotAheadOfTrajectoryClock)
{
  setTime(10.0);
  MincoPlanner p;
  MincoPlannerTestPeer::initialize(p, node, false);
  MincoPlannerTestPeer::readyCandidate(p, false, 5.0, 1.0);
  ASSERT_EQ(p.planAsync(geometry_msgs::msg::PoseStamped{}, geometry_msgs::msg::PoseStamped{}, false),
    std::optional<bool>(true));
  setTime(14.1);  // Nominally 900 ms left, but the robot has only 600 ms of reference left.
  geometry_msgs::msg::PoseStamped current;
  current.pose.position.x = 5.4;  // 300 mm timing error, below the tracking-error threshold.
  EXPECT_TRUE(p.needsLocalReplan(current));
}

TEST_F(PlanningBudgetTest, NonfinalLocalEndpointRetainsPassThroughVelocity)
{
  setTime(10.0);
  nav2_costmap_2d::Costmap2D costmap(30, 30, 1.0, 0.0, 0.0);
  auto query = std::make_shared<ClearanceQuery>(&costmap);
  for (double lookahead : {4.5, 4.8}) {
    MincoPlanner p;
    MincoPlannerTestPeer::initialize(p, node, false);
    const auto trajectory = MincoPlannerTestPeer::optimizeStraightSegment(p, query, lookahead);
    ASSERT_FALSE(trajectory.empty());
    const double duration = trajectory.getTotalDuration();
    EXPECT_LT(trajectory.getPos(duration).x(), 6.0);
    EXPECT_GT(trajectory.getVel(duration).head<2>().norm(), 0.2);
  }
}

TEST_F(PlanningBudgetTest, FinalGoalUsesZeroTerminalVelocity)
{
  setTime(10.0);
  nav2_costmap_2d::Costmap2D costmap(30, 30, 1.0, 0.0, 0.0);
  MincoPlanner p;
  MincoPlannerTestPeer::initialize(p, node, false);
  const auto trajectory =
    MincoPlannerTestPeer::optimizeStraightSegment(p, std::make_shared<ClearanceQuery>(&costmap), 6.0);
  ASSERT_FALSE(trajectory.empty());
  const double duration = trajectory.getTotalDuration();
  EXPECT_NEAR(trajectory.getPos(duration).x(), 6.0, 1e-6);
  EXPECT_LT(trajectory.getVel(duration).norm(), 1e-6);
}

TEST_F(PlanningBudgetTest, RollingHotStartPreservesVelocityAtRobotProgress)
{
  setTime(10.0);
  nav2_costmap_2d::Costmap2D costmap(30, 30, 1.0, 0.0, 0.0);
  MincoPlanner p;
  MincoPlannerTestPeer::initialize(p, node, false);
  MincoPlannerTestPeer::readyCandidate(p, false, 5.0, 0.5, 0.5, 1.0);
  ASSERT_EQ(p.planAsync(geometry_msgs::msg::PoseStamped{}, geometry_msgs::msg::PoseStamped{}, false),
    std::optional<bool>(true));
  setTime(12.0);
  // Wall time is 2 s, but MPC is following the point at 1.75 s.
  constexpr double progress = 1.75;
  const double x = 1.0 + 0.5 * progress + 0.25 * progress * progress;
  const auto trajectory =
    MincoPlannerTestPeer::optimizeStraightSegment(p, std::make_shared<ClearanceQuery>(&costmap), 6.0, x);
  ASSERT_FALSE(trajectory.empty());
  EXPECT_NEAR(trajectory.getVel(0.0).x(), 0.5 + 0.5 * (progress + 0.005), 0.01);
  EXPECT_NEAR(trajectory.getAcc(0.0).x(), 0.5, 1e-6);
}

TEST_F(PlanningBudgetTest, TrackingDeviationRequestsLocalReplan)
{
  setTime(10.0);
  MincoPlanner p;
  MincoPlannerTestPeer::initialize(p, node, false);
  MincoPlannerTestPeer::followTrajectory(p);
  MincoPlannerTestPeer::setRobotX(p, 1.49);
  EXPECT_FALSE(MincoPlannerTestPeer::usesColdStart(p));
  MincoPlannerTestPeer::tick(p);
  EXPECT_FALSE(p.planningInProgress());
  MincoPlannerTestPeer::setRobotX(p, 1.51);
  EXPECT_TRUE(MincoPlannerTestPeer::usesColdStart(p));
  MincoPlannerTestPeer::tick(p);
  EXPECT_EQ(MincoPlannerTestPeer::requestedGlobalSearch(p), std::optional<bool>(false));
}

TEST_F(PlanningBudgetTest, TrackingErrorUsesCurrentTrajectoryTime)
{
  setTime(10.0);
  MincoPlanner p;
  MincoPlannerTestPeer::initialize(p, node, false);
  MincoPlannerTestPeer::readyCandidate(p, false, 5.0, 1.0);
  ASSERT_EQ(p.planAsync(geometry_msgs::msg::PoseStamped{}, geometry_msgs::msg::PoseStamped{}, false),
    std::optional<bool>(true));
  setTime(11.0);
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 2.0;
  EXPECT_FALSE(p.needsLocalReplan(pose));
  pose.pose.position.x = 1.0;
  EXPECT_TRUE(p.needsLocalReplan(pose));
}

TEST_F(PlanningBudgetTest, UnsafeTrajectoryRequestsLocalReplan)
{
  setTime(10.0);
  MincoPlanner p;
  MincoPlannerTestPeer::initialize(p, node, false);
  MincoPlannerTestPeer::followTrajectory(p);
  MincoPlannerTestPeer::markUnsafe(p);
  MincoPlannerTestPeer::tick(p);
  EXPECT_EQ(MincoPlannerTestPeer::requestedGlobalSearch(p), std::optional<bool>(false));
}

TEST_F(PlanningBudgetTest, ChangedGoalRequestsGlobalSearch)
{
  setTime(10.0);
  MincoPlanner p;
  MincoPlannerTestPeer::initialize(p, node, false);
  MincoPlannerTestPeer::followTrajectory(p);
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position.x = 20.0;
  p.createPlan(goal, goal, {});
  MincoPlannerTestPeer::tick(p);
  EXPECT_EQ(MincoPlannerTestPeer::requestedGlobalSearch(p), std::optional<bool>(true));
}

TEST_F(PlanningBudgetTest, PollsReadyResultWhenTrajectoryNoLongerNeedsReplanning)
{
  setTime(10.0);
  MincoPlanner p;
  MincoPlannerTestPeer::initialize(p, node, false);
  MincoPlannerTestPeer::followTrajectory(p);
  MincoPlannerTestPeer::setRobotX(p, 2.0);
  MincoPlannerTestPeer::tick(p);
  ASSERT_TRUE(p.planningInProgress());
  MincoPlannerTestPeer::readyCandidate(p, false, 5.0);
  MincoPlannerTestPeer::setRobotX(p, 1.0);  // Trigger cleared while the worker was computing.
  setTime(10.05);
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 1.0;
  ASSERT_FALSE(p.needsLocalReplan(pose));
  const auto old_id = MincoPlannerTestPeer::command(p)->cmds.front().trajectory_id;
  MincoPlannerTestPeer::tick(p);
  EXPECT_FALSE(p.planningInProgress());
  EXPECT_NE(MincoPlannerTestPeer::command(p)->cmds.front().trajectory_id, old_id);
  EXPECT_DOUBLE_EQ(MincoPlannerTestPeer::trajectoryStart(p), 10.05);
}

TEST_F(PlanningBudgetTest, KeepalivePreservesSamplesAndTrajectoryTimeOrigin)
{
  setTime(10.0);
  nav2_costmap_2d::Costmap2D costmap(30, 30, 1.0, 0.0, 0.0);
  MincoPlanner p;
  MincoPlannerTestPeer::initialize(p, node, false);
  std::promise<ros_interfaces::msg::MpcPositionCommand> delivered;
  auto result = delivered.get_future();
  const auto subscription = node->create_subscription<ros_interfaces::msg::MpcPositionCommand>(
    "test_reference", 1, [&](const ros_interfaces::msg::MpcPositionCommand::SharedPtr message) {
      if (message->header.stamp.sec == 11) {
        delivered.set_value(*message);
      }
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  MincoPlannerTestPeer::followTrajectory(p);
  MincoPlannerTestPeer::installSafety(p, std::make_shared<ClearanceQuery>(&costmap));
  const auto original = MincoPlannerTestPeer::command(p);
  ASSERT_TRUE(original.has_value());
  const auto published_at = MincoPlannerTestPeer::lastPublishTime(p);
  setTime(11.0);
  p.safetyTimerCallback();
  EXPECT_EQ(MincoPlannerTestPeer::lastPublishTime(p), published_at);
  MincoPlannerTestPeer::keepaliveDue(p);
  // An outstanding optimization does not block safe reference keepalive.
  MincoPlannerTestPeer::setRobotX(p, 2.0);
  MincoPlannerTestPeer::tick(p);
  ASSERT_TRUE(p.planningInProgress());
  p.safetyTimerCallback();
  const auto refreshed = MincoPlannerTestPeer::command(p);
  ASSERT_TRUE(refreshed.has_value());
  EXPECT_EQ(rclcpp::Time(refreshed->header.stamp), node->now());
  EXPECT_EQ(refreshed->cmds, original->cmds);
  EXPECT_EQ(refreshed->mpc_horizon, original->mpc_horizon);
  EXPECT_DOUBLE_EQ(MincoPlannerTestPeer::trajectoryStart(p), 10.0);
  EXPECT_DOUBLE_EQ(p.getTrajectoryRemainTime(), 4.0);
  EXPECT_GT(MincoPlannerTestPeer::lastPublishTime(p), published_at);
  EXPECT_TRUE(p.planningInProgress());
  ASSERT_EQ(executor.spin_until_future_complete(result, 1s), rclcpp::FutureReturnCode::SUCCESS);
  const auto received = result.get();
  EXPECT_EQ(received.header, refreshed->header);
  EXPECT_EQ(received.cmds, original->cmds);
}

TEST_F(PlanningBudgetTest, ExpiredAndRevokedTrajectoriesCannotBeKeptAlive)
{
  setTime(10.0);
  nav2_costmap_2d::Costmap2D costmap(30, 30, 1.0, 0.0, 0.0);
  MincoPlanner p;
  MincoPlannerTestPeer::initialize(p, node, false);
  MincoPlannerTestPeer::followTrajectory(p);
  MincoPlannerTestPeer::installSafety(p, std::make_shared<ClearanceQuery>(&costmap));
  MincoPlannerTestPeer::keepaliveDue(p);
  const auto published_at = MincoPlannerTestPeer::lastPublishTime(p);
  setTime(15.01);
  p.safetyTimerCallback();
  EXPECT_FALSE(p.isTrajSafe());
  EXPECT_FALSE(MincoPlannerTestPeer::command(p).has_value());
  EXPECT_EQ(MincoPlannerTestPeer::lastPublishTime(p), published_at);
  p.safetyTimerCallback();
  EXPECT_FALSE(MincoPlannerTestPeer::command(p).has_value());
  EXPECT_EQ(MincoPlannerTestPeer::lastPublishTime(p), published_at);
}

TEST_F(PlanningBudgetTest, CollisionRevokesCachedKeepalive)
{
  setTime(10.0);
  nav2_costmap_2d::Costmap2D costmap(30, 30, 1.0, 0.0, 0.0);
  MincoPlanner p;
  MincoPlannerTestPeer::initialize(p, node, false);
  MincoPlannerTestPeer::followTrajectory(p);
  MincoPlannerTestPeer::installSafety(p, std::make_shared<ClearanceQuery>(&costmap));
  costmap.setCost(1, 0, nav2_costmap_2d::LETHAL_OBSTACLE);
  MincoPlannerTestPeer::keepaliveDue(p);
  const auto published_at = MincoPlannerTestPeer::lastPublishTime(p);
  p.safetyTimerCallback();
  EXPECT_FALSE(p.isTrajSafe());
  EXPECT_FALSE(MincoPlannerTestPeer::command(p).has_value());
  EXPECT_EQ(MincoPlannerTestPeer::lastPublishTime(p), published_at);
}

TEST_F(PlanningBudgetTest, CommitsOnlyCurrentExecutionAndGoal)
{
  MincoPlanner p;
  MincoPlannerTestPeer::initialize(p, node);
  MincoPlannerTestPeer::readyCandidate(p);
  ASSERT_EQ(p.planAsync(geometry_msgs::msg::PoseStamped{}, geometry_msgs::msg::PoseStamped{}, false),
    std::optional<bool>(true));
  EXPECT_TRUE(MincoPlannerTestPeer::hasTrajectory(p));

  MincoPlannerTestPeer::readyCandidate(p);
  p.deactivate();
  EXPECT_FALSE(MincoPlannerTestPeer::command(p).has_value());
  p.activate();
  MincoPlannerTestPeer::enable(p);
  EXPECT_FALSE(
    p.planAsync(geometry_msgs::msg::PoseStamped{}, geometry_msgs::msg::PoseStamped{}, false).has_value());
  EXPECT_FALSE(MincoPlannerTestPeer::hasTrajectory(p));

  MincoPlannerTestPeer::readyCandidate(p);
  geometry_msgs::msg::PoseStamped goal;
  goal.pose.position.x = 2.0;
  p.createPlan(goal, goal, {});
  EXPECT_FALSE(
    p.planAsync(geometry_msgs::msg::PoseStamped{}, geometry_msgs::msg::PoseStamped{}, false).has_value());
  EXPECT_FALSE(MincoPlannerTestPeer::hasTrajectory(p));
}

TEST_F(PlanningBudgetTest, RejectsResultDelayedBeforeCommit)
{
  MincoPlanner p;
  MincoPlannerTestPeer::initialize(p, node);
  MincoPlannerTestPeer::readyCandidate(p, true);
  EXPECT_EQ(p.planAsync(geometry_msgs::msg::PoseStamped{}, geometry_msgs::msg::PoseStamped{}, false),
    std::optional<bool>(false));
  EXPECT_FALSE(MincoPlannerTestPeer::hasTrajectory(p));
}

TEST_F(PlanningBudgetTest, DeactivationDoesNotWaitForLongComputation)
{
  MincoPlanner p;
  MincoPlannerTestPeer::initialize(p, node);
  auto started = MincoPlannerTestPeer::longComputation(p);
  ASSERT_EQ(started.wait_for(1s), std::future_status::ready);
  const auto before = std::chrono::steady_clock::now();
  p.deactivate();
  EXPECT_LT(std::chrono::steady_clock::now() - before, 50ms);
  EXPECT_TRUE(MincoPlannerTestPeer::workerStopped(p));
  EXPECT_FALSE(MincoPlannerTestPeer::hasTrajectory(p));
}

TEST_F(PlanningBudgetTest, ExecutionSubscriptionRunsWhilePlanningWithoutParentExecutor)
{
  nav2_costmap_2d::Costmap2D costmap(30, 30, 1.0, 0.0, 0.0);
  auto query = std::make_shared<CountingQuery>(&costmap);
  MincoPlanner p;
  MincoPlannerTestPeer::initialize(p, node);
  MincoPlannerTestPeer::readyCandidate(p);
  ASSERT_EQ(p.planAsync(geometry_msgs::msg::PoseStamped{}, geometry_msgs::msg::PoseStamped{}, false),
    std::optional<bool>(true));
  MincoPlannerTestPeer::installSafety(p, query);
  MincoPlannerTestPeer::startCallbacks(p, node);
  auto publisher = node->create_publisher<std_msgs::msg::Bool>("/minco/execution_enabled", 10);
  publisher->on_activate();
  const auto discovery_deadline = std::chrono::steady_clock::now() + 1s;
  while (
    publisher->get_subscription_count() == 0 && std::chrono::steady_clock::now() < discovery_deadline) {
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_GT(publisher->get_subscription_count(), 0U);
  auto started = MincoPlannerTestPeer::longComputation(p);
  ASSERT_EQ(started.wait_for(1s), std::future_status::ready);
  const auto safety_deadline = std::chrono::steady_clock::now() + 200ms;
  while (query->calls.load() == 0 && std::chrono::steady_clock::now() < safety_deadline) {
    std::this_thread::sleep_for(1ms);
  }
  EXPECT_GT(query->calls.load(), 0);
  EXPECT_TRUE(MincoPlannerTestPeer::workerRunning(p));
  const auto before = std::chrono::steady_clock::now();
  std_msgs::msg::Bool stop;
  stop.data = false;
  publisher->publish(stop);
  while (!MincoPlannerTestPeer::stopped(p) && std::chrono::steady_clock::now() - before < 200ms) {
    std::this_thread::sleep_for(1ms);
  }
  RecordProperty("stop_response_ms",
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - before).count());
  EXPECT_TRUE(MincoPlannerTestPeer::stopped(p));
  EXPECT_TRUE(MincoPlannerTestPeer::workerStopped(p));
}

TEST_F(PlanningBudgetTest, SafetyBudgetExhaustionRevokesReference)
{
  nav2_costmap_2d::Costmap2D costmap(30, 30, 1.0, 0.0, 0.0);
  auto query = std::make_shared<SlowQuery>(&costmap);
  MincoPlanner p;
  MincoPlannerTestPeer::initialize(p, node);
  MincoPlannerTestPeer::readyCandidate(p);
  ASSERT_EQ(p.planAsync(geometry_msgs::msg::PoseStamped{}, geometry_msgs::msg::PoseStamped{}, false),
    std::optional<bool>(true));
  MincoPlannerTestPeer::installSafety(p, query);
  const auto before = std::chrono::steady_clock::now();
  p.safetyTimerCallback();
  EXPECT_LT(std::chrono::steady_clock::now() - before, 100ms);
  EXPECT_GT(query->calls, 0);
  EXPECT_LT(query->calls, 21);
  EXPECT_FALSE(p.isTrajSafe());
  EXPECT_FALSE(MincoPlannerTestPeer::hasTrajectory(p));
  EXPECT_FALSE(MincoPlannerTestPeer::command(p).has_value());
}

TEST(PlanningDeadlineTest, AstarChecksCancellationBeforeExpansion)
{
  std::vector<unsigned char> costs(100, 0);
  Astar astar(10, 10);
  astar.setCostmap(costs.data());
  astar.setStart(1, 1);
  astar.setGoal(8, 8);
  astar.setupNavFn(true);
  int calls = 0;
  EXPECT_FALSE(astar.propNavFnAstar(100, [&] {
    ++calls;
    return true;
  }));
  EXPECT_EQ(calls, 1);
}

TEST(PlanningDeadlineTest, MincoQuadratureHonorsDeadlineAndCancellation)
{
  nav2_costmap_2d::Costmap2D costmap(30, 30, 1.0, 0.0, 0.0);
  auto query = std::make_shared<SlowQuery>(&costmap);
  MincoOptimizer::Config cfg;
  cfg.magnitudeBounds = Eigen::Vector3d(0.1, 2.0, 4.0);
  cfg.penaltyWeights = Eigen::VectorXd::Ones(5);
  cfg.print_optimizer_log = false;
  MincoOptimizer optimizer(cfg);
  optimizer.setMap(query);
  const std::vector<Eigen::Vector3d> path{{1, 1, 0}, {5, 1, 0}, {10, 1, 0}};
  Eigen::Matrix3d start = Eigen::Matrix3d::Zero(), end = start;
  start.col(0) = path.front();
  end.col(0) = path.back();
  traj_opt::Trajectory result;
  {
    PlanningDeadline deadline(0.005, [] {
      return false;
    });
    EXPECT_THROW(optimizer.optimize(path, start, end, Eigen::Vector2d(2, 2), result), PlanningInterrupted);
  }
  EXPECT_GT(query->calls, 0);
  EXPECT_LT(query->calls, 34);
  query->calls = 0;
  {
    PlanningDeadline deadline(5.0, [&] {
      return query->calls >= 3;
    });
    EXPECT_THROW(optimizer.optimize(path, start, end, Eigen::Vector2d(2, 2), result), PlanningInterrupted);
  }
  EXPECT_EQ(query->calls, 3);
  EXPECT_NO_THROW(PlanningDeadline::check());  // worker scope has been removed
}

TEST(PlanningTimingBenchmark, LocalMincoP95P99)
{
  nav2_costmap_2d::Costmap2D costmap(400, 400, 0.05, -10.0, -10.0);
  auto query = std::make_shared<Nav2CostmapQuery>(&costmap);

  MincoOptimizer::Config cfg;
  cfg.safe_dist = 0.35;
  cfg.max_vel = 3.0;
  cfg.max_acc = 2.0;
  cfg.rho = 100.0;
  cfg.smooth_eps = 0.01;
  cfg.magnitudeBounds = Eigen::Vector3d(cfg.safe_dist, cfg.max_vel, cfg.max_acc);
  cfg.penaltyWeights = Eigen::VectorXd(5);
  cfg.penaltyWeights << 50000.0, 1500.0, 1500.0, 100.0, 50.0;
  cfg.time_allocation_iters = 15;
  cfg.integral_res = 16;
  cfg.opt_accuracy = 1.0e-3;
  cfg.print_optimizer_log = false;

  const std::vector<Eigen::Vector3d> path{{0.0, 0.0, 0.0},
    {1.0, 0.2, 0.0},
    {2.0, 0.8, 0.0},
    {3.0, 1.1, 0.0},
    {4.0, 1.0, 0.0},
    {5.0, 0.4, 0.0},
    {6.0, 0.0, 0.0},
    {7.0, -0.2, 0.0},
    {8.0, 0.0, 0.0}};
  Eigen::Matrix3d start = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d end = Eigen::Matrix3d::Zero();
  start.col(0) = path.front();
  end.col(0) = path.back();
  VecDf local_magnitudes = VecDf::Constant(static_cast<int>(path.size()) - 1, cfg.max_vel);

  constexpr int warmup = 10;
  constexpr int samples = 100;
  MincoOptimizer optimizer(cfg);
  optimizer.setMap(query);
  for (int i = 0; i < warmup; ++i) {
    traj_opt::Trajectory output;
    ASSERT_TRUE(std::isfinite(optimizer.optimize(path, start, end, local_magnitudes, output)));
  }

  std::vector<double> durations;
  durations.reserve(samples);
  for (int i = 0; i < samples; ++i) {
    traj_opt::Trajectory output;
    const auto before = std::chrono::steady_clock::now();
    ASSERT_TRUE(std::isfinite(optimizer.optimize(path, start, end, local_magnitudes, output)));
    durations.push_back(
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - before).count());
  }

  const double p50 = percentileMs(durations, 0.50);
  const double p95 = percentileMs(durations, 0.95);
  const double p99 = percentileMs(durations, 0.99);
  const double max = *std::max_element(durations.begin(), durations.end());
  RecordProperty("sample_count", samples);
  RecordProperty("p50_ms", p50);
  RecordProperty("p95_ms", p95);
  RecordProperty("p99_ms", p99);
  RecordProperty("max_ms", max);
  std::cout << "LOCAL_MINCO timing_ms p50=" << p50 << " p95=" << p95 << " p99=" << p99 << " max=" << max
            << " samples=" << samples << std::endl;
}

TEST(PlanningTimingBenchmark, GlobalAstarP95P99)
{
  nav2_costmap_2d::Costmap2D costmap(200, 200, 0.05, -5.0, -5.0);
  auto query = std::make_shared<Nav2CostmapQuery>(&costmap);
  auto clearance = std::make_shared<ClearanceQuery>(&costmap);
  std::vector<unsigned char> values;
  ASSERT_TRUE(query->copyValues(values));
  ASSERT_EQ(values.size(), 200U * 200U);

  constexpr int warmup = 5;
  constexpr int samples = 100;
  std::vector<double> durations;
  durations.reserve(samples);
  for (int sample = -warmup; sample < samples; ++sample) {
    Astar astar(200, 200);
    astar.setMap(query);
    astar.setESDFQuery(clearance);
    astar.setCollisionDistance(0.2);
    astar.setStart(20, 20);
    astar.setGoal(180, 180);
    astar.setCostmap(values.data(), true, false);
    const auto before = std::chrono::steady_clock::now();
    astar.setupNavFn(true);
    const int cycles_per_step = std::max(200 * 200 / 20, 200 + 200);
    int remaining = 200 * 200 * 9999;
    while (remaining > 0 && astar.propNavFnAstar(cycles_per_step)) {
      remaining -= cycles_per_step;
    }
    ASSERT_TRUE(astar.calcPath(200 * 200 / 2));
    if (sample >= 0) {
      durations.push_back(
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - before).count());
    }
  }

  const double p50 = percentileMs(durations, 0.50);
  const double p95 = percentileMs(durations, 0.95);
  const double p99 = percentileMs(durations, 0.99);
  const double max = *std::max_element(durations.begin(), durations.end());
  RecordProperty("sample_count", samples);
  RecordProperty("p50_ms", p50);
  RecordProperty("p95_ms", p95);
  RecordProperty("p99_ms", p99);
  RecordProperty("max_ms", max);
  std::cout << "GLOBAL_ASTAR timing_ms p50=" << p50 << " p95=" << p95 << " p99=" << p99 << " max=" << max
            << " samples=" << samples << std::endl;
}

TEST(SmacGoalApproach, PlansToGoalWhenStartAlreadyInsideSearchTolerance)
{
  // The tighter Nav2 goal check still needs a trackable approach path here.
  nav2_costmap_2d::Costmap2D costmap(200, 200, 0.05, 0.0, 0.0);
  auto query = std::make_shared<Nav2CostmapQuery>(&costmap);
  smac::SmacPlanner2DSimple smac;
  smac.setMap(query);
  smac.setParameters(false, 1000000, 0.3F);
  smac::SmacPlanner2DSimple::CoordinateVector path;
  ASSERT_TRUE(smac.createPath(92, 145, 89, 145, path));
  ASSERT_GE(path.size(), 2U);
  EXPECT_EQ(path.front().x, 89.0F);
  EXPECT_EQ(path.front().y, 145.0F);
  EXPECT_EQ(path.back().x, 92.0F);
  EXPECT_EQ(path.back().y, 145.0F);
}

TEST(PlanningTimingBenchmark, GlobalSmacP95P99)
{
  nav2_costmap_2d::Costmap2D costmap(200, 200, 0.05, -5.0, -5.0);
  for (unsigned int y = 0; y < 200; ++y) {
    if (y < 92 || y > 108) {
      costmap.setCost(100, y, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
  for (unsigned int x = 115; x < 185; ++x) {
    if (x < 148 || x > 162) {
      costmap.setCost(x, 100, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
  auto query = std::make_shared<Nav2CostmapQuery>(&costmap);
  auto clearance = std::make_shared<ClearanceQuery>(&costmap);
  smac::SmacPlanner2DSimple smac;
  smac.setMap(query);
  smac.setESDFQuery(clearance);
  smac.setCollisionDistance(0.2);
  smac.setParameters(false, 1000000, 0.3F);

  constexpr int warmup = 5;
  constexpr int samples = 100;
  std::vector<double> durations;
  durations.reserve(samples);
  for (int sample = -warmup; sample < samples; ++sample) {
    smac::SmacPlanner2DSimple::CoordinateVector path;
    const auto before = std::chrono::steady_clock::now();
    ASSERT_TRUE(smac.createPath(20, 20, 180, 180, path));
    if (sample >= 0) {
      durations.push_back(
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - before).count());
    }
  }

  const double p50 = percentileMs(durations, 0.50);
  const double p95 = percentileMs(durations, 0.95);
  const double p99 = percentileMs(durations, 0.99);
  const double max = *std::max_element(durations.begin(), durations.end());
  RecordProperty("sample_count", samples);
  RecordProperty("p50_ms", p50);
  RecordProperty("p95_ms", p95);
  RecordProperty("p99_ms", p99);
  RecordProperty("max_ms", max);
  std::cout << "GLOBAL_SMAC timing_ms p50=" << p50 << " p95=" << p95 << " p99=" << p99 << " max=" << max
            << " samples=" << samples << std::endl;
}

TEST_F(PlanningBudgetTest, SafetyCheckP95P99)
{
  nav2_costmap_2d::Costmap2D costmap(30, 30, 1.0, 0.0, 0.0);
  auto query = std::make_shared<CountingQuery>(&costmap);
  MincoPlanner p;
  MincoPlannerTestPeer::initialize(p, node);
  MincoPlannerTestPeer::readyCandidate(p);
  ASSERT_EQ(p.planAsync(geometry_msgs::msg::PoseStamped{}, geometry_msgs::msg::PoseStamped{}, false),
    std::optional<bool>(true));
  MincoPlannerTestPeer::installSafety(p, query);

  constexpr int warmup = 10;
  constexpr int samples = 100;
  for (int i = 0; i < warmup; ++i) {
    p.safetyTimerCallback();
  }
  std::vector<double> durations;
  durations.reserve(samples);
  for (int i = 0; i < samples; ++i) {
    const auto before = std::chrono::steady_clock::now();
    p.safetyTimerCallback();
    durations.push_back(
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - before).count());
  }

  const double p50 = percentileMs(durations, 0.50);
  const double p95 = percentileMs(durations, 0.95);
  const double p99 = percentileMs(durations, 0.99);
  const double max = *std::max_element(durations.begin(), durations.end());
  RecordProperty("sample_count", samples);
  RecordProperty("p50_ms", p50);
  RecordProperty("p95_ms", p95);
  RecordProperty("p99_ms", p99);
  RecordProperty("max_ms", max);
  std::cout << "SAFETY_CHECK timing_ms p50=" << p50 << " p95=" << p95 << " p99=" << p99 << " max=" << max
            << " samples=" << samples << std::endl;
}
}  // namespace
}  // namespace minco_planner
