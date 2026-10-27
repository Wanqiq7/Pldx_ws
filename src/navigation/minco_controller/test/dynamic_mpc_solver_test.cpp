#include "minco_controller/dynamic_mpc_solver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>

#include <gtest/gtest.h>
#include <qpOASES.hpp>

namespace minco_controller {
namespace {

using RowMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

double accelerationLimit(const DynamicMpcConfig & c)
{
  return std::min(c.friction_coeff * 9.81, c.max_force / c.mass);
}

// Independent dense formulation over the full 6-state model, cold-started each call.
// Valid for references whose speed stays above the heading threshold and for
// initial velocities inside the bounds (no relaxation).
Eigen::VectorXd solveDense(const DynamicMpcConfig & c,
  const State & s,
  const std::vector<ReferencePoint> & refs,
  const Eigen::Vector3d & last_acc,
  const Eigen::Vector2d & drift = Eigen::Vector2d::Zero())
{
  const int n = c.horizon;
  const double dt = c.dt;
  Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Identity();
  A.topRightCorner<3, 3>() = dt * Eigen::Matrix3d::Identity();
  Eigen::Matrix<double, 6, 3> B;
  B << 0.5 * dt * dt * Eigen::Matrix3d::Identity(), dt * Eigen::Matrix3d::Identity();

  Eigen::MatrixXd Sx = Eigen::MatrixXd::Zero(6 * n, 6);
  Eigen::MatrixXd Su = Eigen::MatrixXd::Zero(6 * n, 3 * n);
  Eigen::Matrix<double, 6, 6> power = Eigen::Matrix<double, 6, 6>::Identity();
  for (int k = 0; k < n; ++k) {
    power = A * power;
    Sx.middleRows<6>(6 * k) = power;
    for (int j = 0; j <= k; ++j) {
      Eigen::Matrix<double, 6, 6> Ap = Eigen::Matrix<double, 6, 6>::Identity();
      for (int m = 0; m < k - j; ++m) {
        Ap = A * Ap;
      }
      Su.block<6, 3>(6 * k, 3 * j) = Ap * B;
    }
  }

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(6 * n, 6 * n);
  Eigen::VectorXd xref(6 * n), uref(3 * n);
  for (int k = 1; k <= n; ++k) {
    const auto & r = refs[k];
    const Eigen::Vector2d t = r.vel.normalized();
    const Eigen::Vector2d nrm(-t.y(), t.x());
    const Eigen::Matrix2d wp = c.q_pos_along * t * t.transpose() + c.q_pos_cross * nrm * nrm.transpose();
    const Eigen::Matrix2d wv = c.q_vel_along * t * t.transpose() + c.q_vel_cross * nrm * nrm.transpose();
    const int o = 6 * (k - 1);
    Q.block<2, 2>(o, o) = wp;
    Q(o + 2, o + 2) = c.q_yaw;
    Q.block<2, 2>(o + 3, o + 3) = wv;
    Q(o + 5, o + 5) = c.q_omega;
    xref.segment<6>(o) << r.pos, r.yaw, r.vel, r.yaw_rate;
  }
  for (int j = 0; j < n; ++j) {
    uref.segment<2>(3 * j) = (refs[j + 1].vel - refs[j].vel) / dt - drift;
    uref(3 * j + 2) = (refs[j + 1].yaw_rate - refs[j].yaw_rate) / dt;
  }
  const Eigen::Vector3d rdiag(c.r_acc, c.r_acc, c.r_alpha);
  const Eigen::Vector3d ddiag(c.r_jerk, c.r_jerk, c.r_dalpha);
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(3 * n, 3 * n);
  Eigen::MatrixXd D = Eigen::MatrixXd::Zero(3 * n, 3 * n);
  Eigen::MatrixXd Wd = Eigen::MatrixXd::Zero(3 * n, 3 * n);
  for (int j = 0; j < n; ++j) {
    R.block<3, 3>(3 * j, 3 * j) = rdiag.asDiagonal();
    Wd.block<3, 3>(3 * j, 3 * j) = ddiag.asDiagonal();
    D.block<3, 3>(3 * j, 3 * j).setIdentity();
    if (j > 0) {
      D.block<3, 3>(3 * j, 3 * (j - 1)) = -Eigen::Matrix3d::Identity();
    }
  }
  Eigen::VectorXd d0 = Eigen::VectorXd::Zero(3 * n);
  d0.head<3>() = last_acc;

  Eigen::Matrix<double, 6, 1> x0;
  x0 << s.x, s.y, s.yaw, s.vx, s.vy, s.omega;
  // x_{k+1} = A·x_k + B·(u_k + [d, 0]): the drift enters exactly like a constant input.
  Eigen::VectorXd drift_inputs = Eigen::VectorXd::Zero(3 * n);
  for (int j = 0; j < n; ++j) {
    drift_inputs.segment<2>(3 * j) = drift;
  }
  const Eigen::VectorXd drift_response = Su * drift_inputs;
  const Eigen::VectorXd e0 = Sx * x0 + drift_response - xref;
  RowMatrix H = 2.0 * (Su.transpose() * Q * Su + R + D.transpose() * Wd * D);
  H.diagonal().array() += c.eps_regularization;
  Eigen::VectorXd g = 2.0 * (Su.transpose() * Q * e0 - R * uref - D.transpose() * Wd * d0);

  const double a_lim = accelerationLimit(c);
  const double face = 0.92387953251128674 * a_lim;
  const double alpha_max = c.max_torque / c.inertia_z;
  Eigen::VectorXd lb(3 * n), ub(3 * n);
  RowMatrix G = RowMatrix::Zero(5 * n, 3 * n);
  Eigen::VectorXd lbA(5 * n), ubA(5 * n);
  for (int k = 0; k < n; ++k) {
    lb.segment<3>(3 * k) << -face, -face, -alpha_max;
    ub.segment<3>(3 * k) << face, face, alpha_max;
    for (int d = 0; d < 3; ++d) {
      G.row(5 * k + d) = Su.row(6 * k + 3 + d);
    }
    const Eigen::Vector3d v_free =
      Eigen::Vector3d(s.vx, s.vy, s.omega) + drift_response.segment<3>(6 * k + 3);
    lbA.segment<3>(5 * k) = Eigen::Vector3d(c.vx_min, c.vy_min, c.omega_min) - v_free;
    ubA.segment<3>(5 * k) = Eigen::Vector3d(c.vx_max, c.vy_max, c.omega_max) - v_free;
    G(5 * k + 3, 3 * k) = G(5 * k + 3, 3 * k + 1) = 1.0;
    G(5 * k + 4, 3 * k) = 1.0;
    G(5 * k + 4, 3 * k + 1) = -1.0;
    lbA.segment<2>(5 * k + 3).setConstant(-std::sqrt(2.0) * face);
    ubA.segment<2>(5 * k + 3).setConstant(std::sqrt(2.0) * face);
  }

  qpOASES::QProblem qp(3 * n, 5 * n);
  qpOASES::Options options;
  options.setToMPC();
  options.printLevel = qpOASES::PL_NONE;
  qp.setOptions(options);
  qpOASES::int_t nWSR = 1000;
  if (qp.init(H.data(), g.data(), G.data(), lb.data(), ub.data(), lbA.data(), ubA.data(), nWSR) !=
      qpOASES::SUCCESSFUL_RETURN) {
    return {};
  }
  Eigen::VectorXd u(3 * n);
  if (qp.getPrimalSolution(u.data()) != qpOASES::SUCCESSFUL_RETURN) {
    return {};
  }
  return u;
}

// Moving reference whose speed and acceleration exceed the bounds in places, so
// velocity, box and diagonal constraints all become active.
std::vector<ReferencePoint> makeReference(const DynamicMpcConfig & c, int cycle)
{
  std::vector<ReferencePoint> refs(c.horizon + 1);
  const double phase = 0.01 * cycle;
  for (int k = 0; k <= c.horizon; ++k) {
    const double t = phase + c.dt * k;
    refs[k].pos = Eigen::Vector2d(0.5 * std::sin(3.0 * t), 0.4 * std::cos(3.3 * t));
    refs[k].vel = Eigen::Vector2d(1.5 * std::cos(3.0 * t), -1.32 * std::sin(3.3 * t));
    refs[k].yaw = 1.2 * std::sin(1.5 * t);
    refs[k].yaw_rate = 1.8 * std::cos(1.5 * t);
  }
  return refs;
}

DynamicMpcConfig testConfig()
{
  DynamicMpcConfig c;
  c.horizon = 25;
  c.vx_min = c.vy_min = -1.2;
  c.vx_max = c.vy_max = 1.2;
  c.omega_min = -1.5;
  c.omega_max = 1.5;
  return c;
}

void expectPredictionFeasible(const DynamicMpcSolver & solver, const std::vector<State> & pred)
{
  const auto & c = solver.config();
  const double a_lim = solver.translationalAccelerationLimit();
  for (size_t k = 0; k + 1 < pred.size(); ++k) {
    const double ax = (pred[k + 1].vx - pred[k].vx) / c.dt;
    const double ay = (pred[k + 1].vy - pred[k].vy) / c.dt;
    const double alpha = (pred[k + 1].omega - pred[k].omega) / c.dt;
    // The inscribed octagon never exceeds the friction circle.
    EXPECT_LE(std::hypot(ax, ay), a_lim + 1e-6);
    EXPECT_LE(std::abs(alpha), solver.angularAccelerationLimit() + 1e-6);
  }
}

TEST(DynamicMpcSolver, MatchesDenseFormulationWithActiveConstraints)
{
  const auto c = testConfig();
  DynamicMpcSolver solver(c);
  int active = 0;
  for (int cycle = 0; cycle < 120; ++cycle) {
    SCOPED_TRACE(::testing::Message() << "cycle=" << cycle);
    State s;
    s.x = 0.1 * std::sin(0.05 * cycle);
    s.y = 0.35 + 0.1 * std::cos(0.04 * cycle);
    s.yaw = 0.2 * std::cos(0.03 * cycle);
    s.vx = 0.8 * std::cos(0.02 * cycle);
    s.vy = -0.5 * std::sin(0.03 * cycle);
    s.omega = 0.6 * std::sin(0.02 * cycle);
    const auto refs = makeReference(c, cycle);
    const Eigen::Vector3d last = solver.getLastAcceleration();
    const auto expected = solveDense(c, s, refs, last);
    ASSERT_EQ(expected.size(), 3 * c.horizon);

    DynamicControl u;
    std::vector<State> pred;
    ASSERT_TRUE(solver.solve(s, refs, u, &pred));
    ASSERT_EQ(pred.size(), c.horizon + 1u);
    EXPECT_NEAR(u.acceleration.x(), expected(0), 1e-5);
    EXPECT_NEAR(u.acceleration.y(), expected(1), 1e-5);
    EXPECT_NEAR(u.acceleration.z(), expected(2), 1e-5);
    EXPECT_EQ(solver.getLastAcceleration(), u.acceleration);
    State x = s;
    for (int k = 0; k < c.horizon; ++k) {
      const Eigen::Vector3d a = expected.segment<3>(3 * k);
      x.x += c.dt * x.vx + 0.5 * c.dt * c.dt * a.x();
      x.y += c.dt * x.vy + 0.5 * c.dt * c.dt * a.y();
      x.yaw += c.dt * x.omega + 0.5 * c.dt * c.dt * a.z();
      x.vx += c.dt * a.x();
      x.vy += c.dt * a.y();
      x.omega += c.dt * a.z();
      EXPECT_NEAR(pred[k + 1].x, x.x, 1e-6);
      EXPECT_NEAR(pred[k + 1].y, x.y, 1e-6);
      EXPECT_NEAR(pred[k + 1].yaw, x.yaw, 1e-6);
      EXPECT_NEAR(pred[k + 1].vx, x.vx, 1e-5);
      EXPECT_NEAR(pred[k + 1].vy, x.vy, 1e-5);
      EXPECT_NEAR(pred[k + 1].omega, x.omega, 1e-5);
      EXPECT_GE(x.vx, c.vx_min - 1e-6);
      EXPECT_LE(x.vx, c.vx_max + 1e-6);
      EXPECT_GE(x.vy, c.vy_min - 1e-6);
      EXPECT_LE(x.vy, c.vy_max + 1e-6);
      EXPECT_GE(x.omega, c.omega_min - 1e-6);
      EXPECT_LE(x.omega, c.omega_max + 1e-6);
      const double face = solver.octagonFaceLimit();
      if (std::abs(x.vx - c.vx_max) < 1e-6 || std::abs(a.x()) > face - 1e-6 ||
          std::abs(a.x() + a.y()) > std::sqrt(2.0) * face - 1e-6) {
        ++active;
      }
    }
    expectPredictionFeasible(solver, pred);
    EXPECT_NEAR(u.wrench.x(), c.mass * u.acceleration.x(), 1e-9);
    EXPECT_NEAR(u.wrench.z(), c.inertia_z * u.acceleration.z(), 1e-9);
  }
  EXPECT_GT(active, 0);
}

TEST(DynamicMpcSolver, ForceFeedforwardTracksConstantAccelerationWithoutLag)
{
  auto c = testConfig();
  c.r_jerk = c.r_dalpha = 0.0;
  DynamicMpcSolver solver(c);
  const Eigen::Vector2d a_ref(0.8, -0.4);
  const double alpha_ref = 0.6;
  State s;
  s.vx = 0.2;
  s.vy = 0.1;
  s.omega = -0.3;
  std::vector<ReferencePoint> refs(c.horizon + 1);
  for (int k = 0; k <= c.horizon; ++k) {
    const double t = c.dt * k;
    refs[k].pos = Eigen::Vector2d(s.vx, s.vy) * t + 0.5 * a_ref * t * t;
    refs[k].vel = Eigen::Vector2d(s.vx, s.vy) + a_ref * t;
    refs[k].yaw = s.omega * t + 0.5 * alpha_ref * t * t;
    refs[k].yaw_rate = s.omega + alpha_ref * t;
  }
  DynamicControl u;
  std::vector<State> pred;
  ASSERT_TRUE(solver.solve(s, refs, u, &pred));
  EXPECT_NEAR(u.acceleration.x(), a_ref.x(), 1e-6);
  EXPECT_NEAR(u.acceleration.y(), a_ref.y(), 1e-6);
  EXPECT_NEAR(u.acceleration.z(), alpha_ref, 1e-6);
  for (int k = 0; k <= c.horizon; ++k) {
    EXPECT_NEAR(pred[k].x, refs[k].pos.x(), 1e-6);
    EXPECT_NEAR(pred[k].y, refs[k].pos.y(), 1e-6);
    EXPECT_NEAR(pred[k].yaw, refs[k].yaw, 1e-6);
  }
}

TEST(DynamicMpcSolver, StationaryReferenceAtRestHoldsStill)
{
  const auto c = testConfig();
  DynamicMpcSolver solver(c);
  std::vector<ReferencePoint> refs(c.horizon + 1);
  for (auto & r : refs) {
    r.pos = Eigen::Vector2d(1.0, -2.0);
    r.yaw = 0.4;
  }
  State s;
  s.x = 1.0;
  s.y = -2.0;
  s.yaw = 0.4;
  DynamicControl u;
  ASSERT_TRUE(solver.solve(s, refs, u));
  EXPECT_NEAR(u.acceleration.norm(), 0.0, 1e-8);
  EXPECT_NEAR(u.wrench.norm(), 0.0, 1e-8);
}

TEST(DynamicMpcSolver, StartsFromRestTowardsMovingReference)
{
  auto c = testConfig();
  DynamicMpcSolver solver(c);
  std::vector<ReferencePoint> refs(c.horizon + 1);
  for (int k = 0; k <= c.horizon; ++k) {
    const double t = c.dt * k;
    refs[k].pos = Eigen::Vector2d(0.5 * t * t, 0.0);
    refs[k].vel = Eigen::Vector2d(1.0 * t, 0.0);
  }
  DynamicControl u;
  ASSERT_TRUE(solver.solve({}, refs, u));
  EXPECT_GT(u.acceleration.x(), 0.5);
  EXPECT_NEAR(u.wrench.x(), c.mass * u.acceleration.x(), 1e-9);
}

TEST(DynamicMpcSolver, StaysFeasibleWhenInitialVelocityExceedsBounds)
{
  const auto c = testConfig();
  DynamicMpcSolver solver(c);
  State s;
  // Both axes decelerate together at face/sqrt(2); 25 steps recover about 0.78 m/s.
  s.vx = c.vx_max + 0.5;
  s.vy = c.vy_min - 0.3;
  s.omega = c.omega_max + 1.0;
  const auto refs = makeReference(c, 0);
  DynamicControl u;
  std::vector<State> pred;
  ASSERT_TRUE(solver.solve(s, refs, u, &pred));
  expectPredictionFeasible(solver, pred);
  for (size_t k = 1; k < pred.size(); ++k) {
    // Never further outside than the start, and back inside by the end.
    EXPECT_LE(pred[k].vx, s.vx + 1e-9);
    EXPECT_GE(pred[k].vy, s.vy - 1e-9);
    EXPECT_LE(pred[k].omega, s.omega + 1e-9);
  }
  EXPECT_LE(pred.back().vx, c.vx_max + 1e-6);
  EXPECT_GE(pred.back().vy, c.vy_min - 1e-6);
  EXPECT_LE(pred.back().omega, c.omega_max + 1e-6);
}

TEST(DynamicMpcSolver, CommitControlsRecordedInput)
{
  const auto c = testConfig();
  DynamicMpcSolver solver(c);
  DynamicControl u;
  ASSERT_TRUE(solver.solve({}, makeReference(c, 0), u, nullptr, false));
  EXPECT_EQ(solver.getLastAcceleration(), Eigen::Vector3d::Zero());
  ASSERT_TRUE(solver.solve({}, makeReference(c, 0), u, nullptr, true));
  EXPECT_EQ(solver.getLastAcceleration(), u.acceleration);
  solver.resetLastAcceleration();
  EXPECT_EQ(solver.getLastAcceleration(), Eigen::Vector3d::Zero());
}

TEST(DynamicMpcSolver, RejectsInvalidInputWithoutChangingOutputs)
{
  auto c = testConfig();
  DynamicMpcSolver solver(c);
  auto refs = makeReference(c, 0);
  DynamicControl u;
  ASSERT_TRUE(solver.solve({}, refs, u));
  const Eigen::Vector3d last = solver.getLastAcceleration();

  DynamicControl untouched;
  untouched.acceleration = Eigen::Vector3d(11.0, 12.0, 13.0);
  std::vector<State> pred(1);
  pred[0].x = 42.0;
  std::vector<ReferencePoint> short_refs(refs.begin(), refs.end() - 1);
  EXPECT_FALSE(solver.solve({}, short_refs, untouched, &pred));
  State bad;
  bad.vx = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(solver.solve(bad, refs, untouched, &pred));
  refs[3].yaw_rate = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(solver.solve({}, refs, untouched, &pred));
  EXPECT_EQ(untouched.acceleration, Eigen::Vector3d(11.0, 12.0, 13.0));
  ASSERT_EQ(pred.size(), 1u);
  EXPECT_EQ(pred[0].x, 42.0);
  EXPECT_EQ(solver.getLastAcceleration(), last);

  c.mass = 0.0;
  solver.setConfig(c);
  EXPECT_FALSE(solver.solve({}, makeReference(c, 0), untouched));
  c = testConfig();
  c.horizon = 0;
  solver.setConfig(c);
  EXPECT_FALSE(solver.solve({}, makeReference(c, 0), untouched));
  c = testConfig();
  solver.setConfig(c);
  EXPECT_TRUE(solver.solve({}, makeReference(c, 0), untouched));
}

TEST(DynamicMpcSolver, RecoversAfterIterationExhaustionAndReconfiguration)
{
  auto c = testConfig();
  DynamicMpcSolver solver(c);
  DynamicControl u;
  ASSERT_TRUE(solver.solve({}, makeReference(c, 0), u));
  c.max_working_set_recalculations = 0;
  solver.setConfig(c);
  const Eigen::Vector3d last = solver.getLastAcceleration();
  EXPECT_FALSE(solver.solve({}, makeReference(c, 1), u));
  EXPECT_EQ(solver.getLastAcceleration(), last);
  c.max_working_set_recalculations = 200;
  for (int cycle = 0; cycle < 6; ++cycle) {
    c.horizon = 10 + 5 * cycle;
    c.dt = 0.015 + 0.002 * cycle;
    c.q_pos_cross = 20.0 + 10.0 * cycle;
    solver.setConfig(c);
    State s;
    s.vx = 0.4;
    const auto refs = makeReference(c, cycle);
    const auto expected = solveDense(c, s, refs, solver.getLastAcceleration());
    ASSERT_EQ(expected.size(), 3 * c.horizon);
    ASSERT_TRUE(solver.solve(s, refs, u));
    EXPECT_NEAR(u.acceleration.x(), expected(0), 1e-5);
    EXPECT_NEAR(u.acceleration.y(), expected(1), 1e-5);
    EXPECT_NEAR(u.acceleration.z(), expected(2), 1e-5);
  }
}

TEST(DynamicMpcSolver, MatchesDenseFormulationWithDrift)
{
  const auto c = testConfig();
  DynamicMpcSolver solver(c);
  for (int cycle = 0; cycle < 60; ++cycle) {
    SCOPED_TRACE(::testing::Message() << "cycle=" << cycle);
    const Eigen::Vector2d drift(0.6 * std::sin(0.1 * cycle), -0.5 * std::cos(0.07 * cycle));
    State s;
    s.y = 0.35;
    s.vx = 0.7 * std::cos(0.02 * cycle);
    s.vy = 0.3;
    const auto refs = makeReference(c, cycle);
    const auto expected = solveDense(c, s, refs, solver.getLastAcceleration(), drift);
    ASSERT_EQ(expected.size(), 3 * c.horizon);
    DynamicControl u;
    std::vector<State> pred;
    ASSERT_TRUE(solver.solve(s, refs, u, &pred, true, drift));
    EXPECT_NEAR(u.acceleration.x(), expected(0), 1e-5);
    EXPECT_NEAR(u.acceleration.y(), expected(1), 1e-5);
    EXPECT_NEAR(u.acceleration.z(), expected(2), 1e-5);
    // Predictions include the drift; the commanded input alone respects the friction octagon.
    State x = s;
    for (int k = 0; k < c.horizon; ++k) {
      const Eigen::Vector2d a = expected.segment<2>(3 * k) + drift;
      x.x += c.dt * x.vx + 0.5 * c.dt * c.dt * a.x();
      x.vx += c.dt * a.x();
      x.y += c.dt * x.vy + 0.5 * c.dt * c.dt * a.y();
      x.vy += c.dt * a.y();
      EXPECT_NEAR(pred[k + 1].x, x.x, 1e-6);
      EXPECT_NEAR(pred[k + 1].vy, x.vy, 1e-5);
      EXPECT_LE(
        std::hypot(expected(3 * k), expected(3 * k + 1)), solver.translationalAccelerationLimit() + 1e-6);
    }
  }
}

TEST(DynamicMpcSolver, HoldsStillAgainstKnownSlopeDrift)
{
  auto c = testConfig();
  c.r_jerk = c.r_dalpha = 0.0;  // otherwise the first input ramps from the previous one
  DynamicMpcSolver solver(c);
  std::vector<ReferencePoint> refs(c.horizon + 1);
  for (auto & r : refs) {
    r.pos = Eigen::Vector2d(2.0, 3.0);
  }
  State s;
  s.x = 2.0;
  s.y = 3.0;
  const Eigen::Vector2d drift(-0.6, 0.3);  // gravity pulling down a slope, below maxDisturbance()
  ASSERT_LT(drift.cwiseAbs().maxCoeff(), solver.maxDisturbance());
  DynamicControl u;
  std::vector<State> pred;
  ASSERT_TRUE(solver.solve(s, refs, u, &pred, true, drift));
  // The drive input cancels the drift exactly and the vehicle stays put.
  EXPECT_NEAR(u.acceleration.x(), -drift.x(), 1e-6);
  EXPECT_NEAR(u.acceleration.y(), -drift.y(), 1e-6);
  EXPECT_NEAR(u.wrench.x(), -c.mass * drift.x(), 1e-5);
  for (const auto & p : pred) {
    EXPECT_NEAR(p.x, 2.0, 1e-8);
    EXPECT_NEAR(p.vy, 0.0, 1e-8);
  }
}

TEST(DynamicMpcSolver, ClampsExcessiveDriftAndStaysFeasible)
{
  const auto c = testConfig();
  DynamicMpcSolver solver(c);
  State s;
  s.vx = c.vx_max + 0.3;  // also outside the bound
  DynamicControl u;
  std::vector<State> pred;
  ASSERT_TRUE(solver.solve(s, makeReference(c, 3), u, &pred, true, Eigen::Vector2d(50.0, -50.0)));
  const double d_max = solver.maxDisturbance();
  for (size_t k = 0; k + 1 < pred.size(); ++k) {
    const double ax = (pred[k + 1].vx - pred[k].vx) / c.dt - d_max;
    const double ay = (pred[k + 1].vy - pred[k].vy) / c.dt + d_max;
    EXPECT_LE(std::hypot(ax, ay), solver.translationalAccelerationLimit() + 1e-6);
    EXPECT_LE(pred[k + 1].vx, s.vx + 1e-9);
  }
  DynamicControl bad;
  EXPECT_FALSE(solver.solve(s,
    makeReference(c, 3),
    bad,
    nullptr,
    true,
    Eigen::Vector2d(std::numeric_limits<double>::quiet_NaN(), 0.0)));
}

TEST(DynamicMpcSolver, DISABLED_Benchmark)
{
  using Clock = std::chrono::steady_clock;
  for (int horizon : {20, 40}) {
    auto c = testConfig();
    c.horizon = horizon;
    DynamicMpcSolver solver(c);
    std::vector<double> times;
    int failures = 0;
    for (int cycle = 0; cycle < 2200; ++cycle) {
      State s;
      s.x = 0.05 * std::sin(0.05 * cycle);
      s.y = 0.4;
      s.vx = 0.8 * std::cos(0.02 * cycle);
      const auto refs = makeReference(c, cycle);
      DynamicControl u;
      const auto start = Clock::now();
      const bool ok = solver.solve(s, refs, u);
      const auto end = Clock::now();
      if (cycle >= 200) {
        failures += !ok;
        times.push_back(std::chrono::duration<double, std::micro>(end - start).count());
      }
    }
    std::sort(times.begin(), times.end());
    std::cout << "horizon=" << horizon << " samples=" << times.size()
              << " warm_us(p50/p95/p99)=" << times[1000] << "/" << times[1900] << "/" << times[1980]
              << " failures=" << failures << std::endl;
    EXPECT_EQ(failures, 0);
  }
}

}  // namespace
}  // namespace minco_controller
