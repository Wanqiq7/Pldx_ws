#include "minco_controller/mpc_solver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>

#include <gtest/gtest.h>
#include <qpOASES.hpp>

namespace minco_controller {
namespace {

// Independent dense batch formulation, cold-started on every call. This also
// supplies the pre-optimization baseline for the opt-in benchmark below.
Eigen::VectorXd solveCold(const MPCConfig & config,
  const State & state,
  const std::vector<ReferencePoint> & refs,
  const Eigen::Vector3d & previous)
{
  const int n = 3 * config.horizon;
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(n, n);
  Eigen::MatrixXd Q = B;
  Eigen::MatrixXd R = B;
  Eigen::VectorXd error(n), feedforward(n), lower(n), upper(n);
  for (int k = 0; k < config.horizon; ++k) {
    const auto & ref = refs[k];
    const double c = std::cos(ref.yaw), s = std::sin(ref.yaw);
    Eigen::Matrix3d rotation;
    rotation << c, -s, 0, s, c, 0, 0, 0, 1;
    Q.block<3, 3>(3 * k, 3 * k) =
      rotation.transpose() * Eigen::Vector3d(config.q_along, config.q_cross, config.Q.z()).asDiagonal() *
      rotation;
    R.block<3, 3>(3 * k, 3 * k) = config.R.asDiagonal();
    error.segment<3>(3 * k) << state.x - ref.pos.x(), state.y - ref.pos.y(), state.yaw - ref.yaw;
    feedforward.segment<3>(3 * k) << ref.vel.x(), ref.vel.y(), ref.yaw_rate;
    lower.segment<3>(3 * k) << config.vx_min, config.vy_min, config.omega_min;
    upper.segment<3>(3 * k) << config.vx_max, config.vy_max, config.omega_max;
    for (int j = 0; j <= k; ++j) {
      B.block<3, 3>(3 * k, 3 * j) = config.dt * Eigen::Matrix3d::Identity();
    }
  }
  using RowMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
  RowMatrix H = 2.0 * (B.transpose() * Q * B + R);
  H.diagonal().array() += config.eps_regularization;
  Eigen::VectorXd g = 2.0 * (B.transpose() * Q * error - R * feedforward);
  const int nc = config.use_acc_constraints ? n : 0;
  RowMatrix A = RowMatrix::Zero(nc, n);
  Eigen::VectorXd lower_acc(nc), upper_acc(nc);
  for (int k = 0; k < nc / 3; ++k) {
    A.block<3, 3>(3 * k, 3 * k).setIdentity();
    if (k > 0) {
      A.block<3, 3>(3 * k, 3 * (k - 1)) = -Eigen::Matrix3d::Identity();
    }
    lower_acc.segment<3>(3 * k) =
      config.dt * Eigen::Vector3d(config.ax_min, config.ay_min, config.alpha_min);
    upper_acc.segment<3>(3 * k) =
      config.dt * Eigen::Vector3d(config.ax_max, config.ay_max, config.alpha_max);
  }
  if (nc > 0) {
    lower_acc.head<3>() += previous;
    upper_acc.head<3>() += previous;
  }
  qpOASES::QProblem qp(n, nc);
  qpOASES::Options options;
  options.setToMPC();
  options.printLevel = qpOASES::PL_NONE;
  qp.setOptions(options);
  qpOASES::int_t iterations = config.max_working_set_recalculations;
  if (qp.init(H.data(),
        g.data(),
        nc ? A.data() : nullptr,
        lower.data(),
        upper.data(),
        nc ? lower_acc.data() : nullptr,
        nc ? upper_acc.data() : nullptr,
        iterations) != qpOASES::SUCCESSFUL_RETURN) {
    return {};
  }
  Eigen::VectorXd solution(n);
  if (qp.getPrimalSolution(solution.data()) != qpOASES::SUCCESSFUL_RETURN) {
    return {};
  }
  return solution;
}

std::vector<ReferencePoint> makeReference(const MPCConfig & config, int cycle)
{
  std::vector<ReferencePoint> refs(config.horizon);
  const double phase = 0.025 * cycle;
  for (int k = 0; k < config.horizon; ++k) {
    const double t = phase + config.dt * (k + 1);
    refs[k].pos = Eigen::Vector2d(1.2 * std::sin(t), 0.8 * std::cos(1.3 * t));
    refs[k].vel = Eigen::Vector2d(1.2 * std::cos(t), -1.04 * std::sin(1.3 * t));
    refs[k].yaw = 1.4 * std::sin(0.7 * t);
    refs[k].yaw_rate = 0.98 * std::cos(0.7 * t);
  }
  return refs;
}

void compareSolution(MpcSolver & solver,
  const MPCConfig & config,
  const State & state,
  const std::vector<ReferencePoint> & refs)
{
  const Eigen::Vector3d previous = solver.getLastControl();
  const auto expected = solveCold(config, state, refs, previous);
  ASSERT_EQ(expected.size(), 3 * config.horizon);
  Control control;
  std::vector<State> predicted;
  ASSERT_TRUE(solver.solve(state, refs, control, &predicted));
  ASSERT_EQ(predicted.size(), config.horizon + 1u);
  EXPECT_NEAR(control.vx, expected(0), 2e-6);
  EXPECT_NEAR(control.vy, expected(1), 2e-6);
  EXPECT_NEAR(control.omega, expected(2), 2e-6);
  Eigen::Vector3d position(state.x, state.y, state.yaw);
  Eigen::Vector3d last = previous;
  for (int k = 0; k < config.horizon; ++k) {
    position += config.dt * expected.segment<3>(3 * k);
    EXPECT_NEAR(predicted[k + 1].x, position.x(), 2e-6);
    EXPECT_NEAR(predicted[k + 1].y, position.y(), 2e-6);
    EXPECT_NEAR(predicted[k + 1].yaw, position.z(), 2e-6);
    const Eigen::Vector3d actual((predicted[k + 1].x - predicted[k].x) / config.dt,
      (predicted[k + 1].y - predicted[k].y) / config.dt,
      (predicted[k + 1].yaw - predicted[k].yaw) / config.dt);
    EXPECT_GE(actual.x(), config.vx_min - 1e-6);
    EXPECT_LE(actual.x(), config.vx_max + 1e-6);
    EXPECT_GE(actual.y(), config.vy_min - 1e-6);
    EXPECT_LE(actual.y(), config.vy_max + 1e-6);
    EXPECT_GE(actual.z(), config.omega_min - 1e-6);
    EXPECT_LE(actual.z(), config.omega_max + 1e-6);
    if (config.use_acc_constraints) {
      const Eigen::Vector3d acceleration = (actual - last) / config.dt;
      EXPECT_GE(acceleration.x(), config.ax_min - 1e-5);
      EXPECT_LE(acceleration.x(), config.ax_max + 1e-5);
      EXPECT_GE(acceleration.y(), config.ay_min - 1e-5);
      EXPECT_LE(acceleration.y(), config.ay_max + 1e-5);
      EXPECT_GE(acceleration.z(), config.alpha_min - 1e-5);
      EXPECT_LE(acceleration.z(), config.alpha_max + 1e-5);
    }
    last = actual;
  }
}

TEST(MpcSolver, MatchesColdBatchWithChangingHessianAndActiveConstraints)
{
  for (bool acceleration : {false, true}) {
    MPCConfig config;
    config.use_acc_constraints = acceleration;
    config.q_along = 0.7;
    config.q_cross = 80.0;
    MpcSolver solver(config);
    for (int cycle = 0; cycle < 160; ++cycle) {
      SCOPED_TRACE(::testing::Message() << "acceleration=" << acceleration << " cycle=" << cycle);
      State state;
      state.x = 0.2 * std::sin(0.03 * cycle);
      state.yaw = 0.2 * std::cos(0.02 * cycle);
      compareSolution(solver, config, state, makeReference(config, cycle));
    }
  }
}

TEST(MpcSolver, RebuildsCachesOnConfigurationChanges)
{
  MPCConfig config;
  MpcSolver solver(config);
  for (int cycle = 0; cycle < 12; ++cycle) {
    config.horizon = cycle % 3 == 0 ? 1 : 5 + cycle;
    config.dt = 0.02 + cycle * 0.003;
    config.use_acc_constraints = cycle % 2 == 0;
    config.R = Eigen::Vector3d(0.6, 1.3, 0.7) * (cycle + 1);
    config.q_cross = 3.0 + cycle;
    config.eps_regularization = 1e-6 * (cycle + 1);
    config.vx_min = -1.0 - cycle * 0.1;
    config.vy_max = 1.0 + cycle * 0.1;
    config.ax_max = 2.0 + cycle;
    solver.setConfig(config);
    compareSolution(solver, config, {}, makeReference(config, cycle));
    compareSolution(solver, config, {}, makeReference(config, cycle + 1));
  }
}

TEST(MpcSolver, RejectsInvalidInputAndRecoversWithoutPublishingStaleSolution)
{
  MPCConfig config;
  config.use_acc_constraints = true;
  MpcSolver solver(config);
  auto refs = makeReference(config, 0);
  compareSolution(solver, config, {}, refs);
  const Eigen::Vector3d previous = solver.getLastControl();
  Control control{11.0, 12.0, 13.0};
  std::vector<State> predicted(1);
  predicted[0].x = 42.0;
  EXPECT_FALSE(solver.solve({}, {}, control, &predicted));
  refs[0].yaw = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(solver.solve({}, refs, control, &predicted));
  EXPECT_EQ(control.vx, 11.0);
  ASSERT_EQ(predicted.size(), 1u);
  EXPECT_EQ(predicted[0].x, 42.0);
  EXPECT_EQ(solver.getLastControl(), previous);
  compareSolution(solver, config, {}, makeReference(config, 3));
  config.horizon = 0;
  solver.setConfig(config);
  EXPECT_FALSE(solver.solve({}, {}, control));
}

TEST(MpcSolver, RecoversAfterInfeasibleQpAndIterationExhaustion)
{
  MPCConfig config;
  config.use_acc_constraints = true;
  config.vx_min = 2.0;
  config.vx_max = 3.0;  // Unreachable from zero within the first acceleration step.
  MpcSolver solver(config);
  Control control{11.0, 12.0, 13.0};
  for (int attempt = 0; attempt < 2; ++attempt) {
    EXPECT_FALSE(solver.solve({}, makeReference(config, attempt), control));
    EXPECT_EQ(control.vx, 11.0);
    EXPECT_EQ(solver.getLastControl(), Eigen::Vector3d::Zero());
  }
  config.vx_min = -1.0;
  solver.setConfig(config);
  compareSolution(solver, config, {}, makeReference(config, 2));
  config.max_working_set_recalculations = 0;
  solver.setConfig(config);
  EXPECT_FALSE(solver.solve({}, makeReference(config, 3), control));
  config.max_working_set_recalculations = 200;
  solver.setConfig(config);
  compareSolution(solver, config, {}, makeReference(config, 4));
}

TEST(MpcSolver, RecoversAfterFailedHotstartWithoutReconfiguration)
{
  MPCConfig config;
  config.max_working_set_recalculations = 1;
  MpcSolver solver(config);
  const std::vector<ReferencePoint> stationary(config.horizon);
  Control control;
  ASSERT_TRUE(solver.solve({}, stationary, control));
  const auto previous = solver.getLastControl();
  control = {11.0, 12.0, 13.0};
  EXPECT_FALSE(solver.solve({}, makeReference(config, 0), control));
  EXPECT_EQ(control.vx, 11.0);
  EXPECT_EQ(solver.getLastControl(), previous);
  ASSERT_TRUE(solver.solve({}, stationary, control));
  EXPECT_NEAR(control.vx, 0.0, 1e-8);
  EXPECT_NEAR(control.vy, 0.0, 1e-8);
  EXPECT_NEAR(control.omega, 0.0, 1e-8);
}

TEST(MpcSolver, DISABLED_Benchmark)
{
  using Clock = std::chrono::steady_clock;
  for (int horizon : {10, 20}) {
    MPCConfig config;
    config.horizon = horizon;
    config.use_acc_constraints = true;
    MpcSolver solver(config);
    Eigen::Vector3d previous = Eigen::Vector3d::Zero();
    std::vector<double> cold_times, warm_times;
    int cold_failures = 0, warm_failures = 0;
    for (int cycle = 0; cycle < 2200; ++cycle) {
      const auto refs = makeReference(config, cycle);
      auto start = Clock::now();
      const auto cold = solveCold(config, {}, refs, previous);
      const auto middle = Clock::now();
      Control control;
      const bool ok = solver.solve({}, refs, control);
      const auto end = Clock::now();
      if (ok) {
        previous = solver.getLastControl();
      }
      if (cycle >= 200) {
        cold_failures += cold.size() == 0;
        warm_failures += !ok;
        cold_times.push_back(std::chrono::duration<double, std::micro>(middle - start).count());
        warm_times.push_back(std::chrono::duration<double, std::micro>(end - middle).count());
      }
    }
    std::sort(cold_times.begin(), cold_times.end());
    std::sort(warm_times.begin(), warm_times.end());
    std::cout << "horizon=" << horizon << " samples=" << cold_times.size()
              << " cold_us(p50/p95/p99)=" << cold_times[1000] << "/" << cold_times[1900] << "/"
              << cold_times[1980] << " warm_us(p50/p95/p99)=" << warm_times[1000] << "/" << warm_times[1900]
              << "/" << warm_times[1980] << " failures(cold/warm)=" << cold_failures << "/" << warm_failures
              << std::endl;
    EXPECT_EQ(cold_failures, 0);
    EXPECT_EQ(warm_failures, 0);
  }
}

}  // namespace
}  // namespace minco_controller
