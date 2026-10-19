#include "minco_controller/mpc_solver.hpp"

#include <array>
#include <cmath>
#include <iostream>

#include "color_text.hpp"
#include <qpOASES.hpp>

namespace minco_controller {

struct MpcSolver::Workspace
{
  using Matrix = Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
  using Vector = Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, 1>;

  explicit Workspace(const MPCConfig & config)
  : prediction(Eigen::MatrixXd::Zero(3 * config.horizon, 3 * config.horizon)),
    weighted_prediction(3 * config.horizon, 3 * config.horizon), weighted_error(3 * config.horizon),
    control_weights(3 * config.horizon), gradient(3 * config.horizon), lower(3 * config.horizon),
    upper(3 * config.horizon),
    constraints(Matrix::Zero(config.use_acc_constraints ? 3 * config.horizon : 0, 3 * config.horizon)),
    lower_acc(constraints.rows()), upper_acc(constraints.rows()), solution(3 * config.horizon),
    qp(3 * config.horizon, constraints.rows())
  {
    const int n = 3 * config.horizon;
    for (auto & hessian : hessians) {
      hessian.resize(n, n);
    }
    for (int k = 0; k < config.horizon; ++k) {
      // A = I, B = dt * I: all lower-triangular prediction blocks are dt * I.
      for (int j = 0; j <= k; ++j) {
        prediction.block<3, 3>(3 * k, 3 * j).diagonal().setConstant(config.dt);
      }
      control_weights.segment<3>(3 * k) = config.R;
      lower.segment<3>(3 * k) << config.vx_min, config.vy_min, config.omega_min;
      upper.segment<3>(3 * k) << config.vx_max, config.vy_max, config.omega_max;
      if (config.use_acc_constraints) {
        constraints.block<3, 3>(3 * k, 3 * k).setIdentity();
        if (k > 0) {
          constraints.block<3, 3>(3 * k, 3 * (k - 1)) = -Eigen::Matrix3d::Identity();
        }
        lower_acc.segment<3>(3 * k) << config.ax_min * config.dt, config.ay_min * config.dt,
          config.alpha_min * config.dt;
        upper_acc.segment<3>(3 * k) << config.ax_max * config.dt, config.ay_max * config.dt,
          config.alpha_max * config.dt;
      }
    }
    qpOASES::Options options;
    options.setToMPC();
    options.printLevel = qpOASES::PL_NONE;
    qp.setOptions(options);
  }

  Eigen::MatrixXd prediction;
  Eigen::MatrixXd weighted_prediction;
  Eigen::VectorXd weighted_error;
  Eigen::VectorXd control_weights;
  // qpOASES retains shallow matrix references. Build the next Hessian separately
  // so the previous QP's storage remains valid throughout the matrix hotstart.
  std::array<Matrix, 2> hessians;
  Vector gradient;
  Vector lower;
  Vector upper;
  Matrix constraints;
  Vector lower_acc;
  Vector upper_acc;
  Vector solution;
  qpOASES::SQProblem qp;
  int next_hessian{0};
  bool initialized{false};
};

MpcSolver::MpcSolver(const MPCConfig & config)
{
  setConfig(config);
}

MpcSolver::~MpcSolver() = default;

void MpcSolver::setConfig(const MPCConfig & config)
{
  config_ = config;
  // Configuration changes invalidate all cached dimensions, bounds and QP state.
  // Preserve the previous successful control used by the acceleration constraint.
  workspace_.reset();
  if (config_.horizon > 0) {
    workspace_ = std::make_unique<Workspace>(config_);
  }
}

void MpcSolver::buildCondensedQP(const State & curr, const std::vector<ReferencePoint> & ref_traj)
{
  auto & w = *workspace_;
  const Eigen::Vector3d x0(curr.x, curr.y, curr.yaw);
  // Only the per-step rotated Q, reference error and feedforward change each cycle.
  for (int k = 0; k < config_.horizon; ++k) {
    const auto & ref = ref_traj[k];
    const double c = std::cos(ref.yaw);
    const double s = std::sin(ref.yaw);
    Eigen::Matrix3d Q = Eigen::Matrix3d::Zero();
    Q(0, 0) = config_.q_along * c * c + config_.q_cross * s * s;
    Q(0, 1) = Q(1, 0) = (config_.q_cross - config_.q_along) * c * s;
    Q(1, 1) = config_.q_along * s * s + config_.q_cross * c * c;
    Q(2, 2) = config_.Q.z();
    w.weighted_prediction.middleRows<3>(3 * k).noalias() = Q * w.prediction.middleRows<3>(3 * k);
    w.weighted_error.segment<3>(3 * k) = Q * (x0 - Eigen::Vector3d(ref.pos.x(), ref.pos.y(), ref.yaw));
  }

  auto & H = w.hessians[w.next_hessian];
  H.noalias() = (2.0 * w.prediction.transpose() * w.weighted_prediction).cast<qpOASES::real_t>();
  H.diagonal().array() +=
    (2.0 * w.control_weights.array() + config_.eps_regularization).cast<qpOASES::real_t>();
  w.gradient.noalias() = (2.0 * w.prediction.transpose() * w.weighted_error).cast<qpOASES::real_t>();
  for (int k = 0; k < config_.horizon; ++k) {
    const auto & ref = ref_traj[k];
    w.gradient.segment<3>(3 * k) -=
      (2.0 * config_.R.cwiseProduct(Eigen::Vector3d(ref.vel.x(), ref.vel.y(), ref.yaw_rate)))
        .cast<qpOASES::real_t>();
  }
  if (config_.use_acc_constraints) {
    // Later differences have fixed bounds. Only v0 - last_u changes each solve.
    w.lower_acc.head<3>() =
      (last_u_global_ + config_.dt * Eigen::Vector3d(config_.ax_min, config_.ay_min, config_.alpha_min))
        .cast<qpOASES::real_t>();
    w.upper_acc.head<3>() =
      (last_u_global_ + config_.dt * Eigen::Vector3d(config_.ax_max, config_.ay_max, config_.alpha_max))
        .cast<qpOASES::real_t>();
  }
}

bool MpcSolver::solve(const State & curr,
  const std::vector<ReferencePoint> & ref_traj,
  Control & out_u,
  std::vector<State> * out_pred)
{
  if (!workspace_ || static_cast<int>(ref_traj.size()) < config_.horizon) {
    return false;
  }
  auto & w = *workspace_;
  buildCondensedQP(curr, ref_traj);
  auto & H = w.hessians[w.next_hessian];
  if (!H.allFinite() || !w.gradient.allFinite()) {
    w.qp.reset();
    w.initialized = false;
    return false;
  }
  const auto * A = config_.use_acc_constraints ? w.constraints.data() : nullptr;
  const auto * lbA = config_.use_acc_constraints ? w.lower_acc.data() : nullptr;
  const auto * ubA = config_.use_acc_constraints ? w.upper_acc.data() : nullptr;
  qpOASES::int_t nWSR = config_.max_working_set_recalculations;
  auto ret = qpOASES::SUCCESSFUL_RETURN;
  if (w.initialized) {
    // SQProblem updates H as well as vectors; QProblem::hotstart would reuse a stale H.
    ret = w.qp.hotstart(H.data(), w.gradient.data(), A, w.lower.data(), w.upper.data(), lbA, ubA, nWSR);
    if (ret != qpOASES::SUCCESSFUL_RETURN) {
      w.qp.reset();
      w.initialized = false;
      nWSR = config_.max_working_set_recalculations;
    }
  }
  if (!w.initialized) {
    ret = w.qp.init(H.data(), w.gradient.data(), A, w.lower.data(), w.upper.data(), lbA, ubA, nWSR);
  }
  if (ret != qpOASES::SUCCESSFUL_RETURN ||
      w.qp.getPrimalSolution(w.solution.data()) != qpOASES::SUCCESSFUL_RETURN || !w.solution.allFinite()) {
    std::cout << color_text::RED << "[MpcSolver] qpOASES Error Code: " << static_cast<int>(ret)
              << color_text::RESET << std::endl;
    w.qp.reset();
    w.initialized = false;
    return false;
  }
  w.initialized = true;
  w.next_hessian = 1 - w.next_hessian;

  if (out_pred) {
    out_pred->clear();
    out_pred->reserve(config_.horizon + 1);
    out_pred->push_back(curr);
    State state = curr;
    for (int k = 0; k < config_.horizon; ++k) {
      state.x += config_.dt * w.solution(3 * k);
      state.y += config_.dt * w.solution(3 * k + 1);
      state.yaw += config_.dt * w.solution(3 * k + 2);
      out_pred->push_back(state);
    }
  }
  last_u_global_ = w.solution.head<3>().cast<double>();
  out_u.vx = last_u_global_.x();
  out_u.vy = last_u_global_.y();
  out_u.omega = last_u_global_.z();
  return true;
}

}  // namespace minco_controller
