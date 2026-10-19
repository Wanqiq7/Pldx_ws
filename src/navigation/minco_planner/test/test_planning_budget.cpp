#include "minco_core/minco_planner.hpp"
#include "minco_core/planning_deadline.hpp"

#include <algorithm>
#include <future>
#include <gtest/gtest.h>
#include <numeric>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

namespace minco_planner {

class MincoPlannerTestPeer
{
public:
  static void initialize(MincoPlanner & p, const nav2_util::LifecycleNode::SharedPtr & node)
  {
    p.node_ = node;
    p.output_frame_ = "map";
    p.active_ = p.execution_enabled_ = true;
    p.opt_path_pub_ = node->create_publisher<ros_interfaces::msg::MpcPositionCommand>("test_reference", 1);
    p.startPlanningWorker();
  }

  static void readyCandidate(MincoPlanner & p, bool old = false)
  {
    Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(3, 6);
    coefficients(0, 5) = 1.0;
    traj_opt::Trajectory traj;
    traj.emplace_back(1.0, coefficients);
    auto command = utils::sampleOptimizedTrajectory(traj, traj, 0, std_msgs::msg::Header{}, 21, 0.05);
    std::lock_guard<std::mutex> lock(p.planning_mutex_);
    p.candidate_ = MincoPlanner::Candidate{traj, traj, command};
    p.computing_version_ = p.planning_version_.load();
    p.computed_at_ = std::chrono::steady_clock::now() - (old ? 200ms : 0ms);
    p.planning_result_ = true;
  }

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
  nav2_util::LifecycleNode::SharedPtr node = std::make_shared<nav2_util::LifecycleNode>("budget_test");
};

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
