#include "minco_controller/dynamic_mpc_solver.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>

#include "color_text.hpp"
#include <qpOASES.hpp>

namespace minco_controller {
namespace {

constexpr double kGravity = 9.81;
// cos(pi/8): face distance of the regular octagon inscribed in the friction circle.
constexpr double kOctagonFace = 0.92387953251128674;
// Below this reference speed the velocity tangent is undefined for along/cross weights.
constexpr double kHeadingSpeed = 0.05;
// Keeps relaxed velocity bounds strictly inside what the input bounds can reach.
constexpr double kFeasibilityMargin = 0.999;

Eigen::Matrix2d rotatedWeight(double heading, double along, double cross)
{
  const double c = std::cos(heading);
  const double s = std::sin(heading);
  Eigen::Matrix2d w;
  w(0, 0) = along * c * c + cross * s * s;
  w(0, 1) = w(1, 0) = (along - cross) * c * s;
  w(1, 1) = along * s * s + cross * c * c;
  return w;
}

// A velocity that already violates its bound must stay feasible: widen the bound at
// step k by what a guaranteed per-axis net rate can recover within k + 1 steps. Inside
// the bounds the input can always cancel the drift, so the nominal bound is kept.
void relaxedBounds(double v0, double vmin, double vmax, double rate, int k, double & lb, double & ub)
{
  const double reach = rate * (k + 1);
  lb = std::min(vmin, v0 + reach);
  ub = std::max(vmax, v0 - reach);
}

// Adds 2·r·I + 2·r_diff·DᵀD + eps·I, where D differences consecutive inputs per axis.
template <typename Matrix>
void addInputRegularization(Matrix & H, int axes, double r, double r_diff, double eps)
{
  const int n = static_cast<int>(H.rows()) / axes;
  for (int j = 0; j < n; ++j) {
    const double diff_diag = j + 1 < n ? 2.0 : 1.0;
    for (int d = 0; d < axes; ++d) {
      const int idx = axes * j + d;
      H(idx, idx) += 2.0 * r + 2.0 * r_diff * diff_diag + eps;
      if (j + 1 < n) {
        H(idx, idx + axes) -= 2.0 * r_diff;
        H(idx + axes, idx) -= 2.0 * r_diff;
      }
    }
  }
}

bool finite(const State & s)
{
  return std::isfinite(s.x) && std::isfinite(s.y) && std::isfinite(s.yaw) && std::isfinite(s.vx) &&
         std::isfinite(s.vy) && std::isfinite(s.omega);
}

bool finite(const ReferencePoint & r)
{
  return r.pos.allFinite() && r.vel.allFinite() && std::isfinite(r.yaw) && std::isfinite(r.yaw_rate);
}

}  // namespace

struct DynamicMpcSolver::Workspace
{
  using Matrix = Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
  using Vector = Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, 1>;

  Workspace(const DynamicMpcConfig & c, double face, double alpha_max)
  : n(c.horizon), translation_qp(2 * c.horizon, 4 * c.horizon), yaw_qp(c.horizon, c.horizon)
  {
    const double dt = c.dt;
    for (auto & hessian : translation_hessians) {
      hessian.resize(2 * n, 2 * n);
    }
    // Rows [0, 2N): v_{k+1} = v0 + dt·Σ_{j<=k} a_j per axis. Rows [2N, 4N): |ax ± ay|.
    translation_constraints = Matrix::Zero(4 * n, 2 * n);
    yaw_constraints = Matrix::Zero(n, n);
    for (int k = 0; k < n; ++k) {
      for (int j = 0; j <= k; ++j) {
        translation_constraints(2 * k, 2 * j) = dt;
        translation_constraints(2 * k + 1, 2 * j + 1) = dt;
        yaw_constraints(k, j) = dt;
      }
      translation_constraints(2 * n + 2 * k, 2 * k) = 1.0;
      translation_constraints(2 * n + 2 * k, 2 * k + 1) = 1.0;
      translation_constraints(2 * n + 2 * k + 1, 2 * k) = 1.0;
      translation_constraints(2 * n + 2 * k + 1, 2 * k + 1) = -1.0;
    }
    translation_lower = Vector::Constant(2 * n, -face);
    translation_upper = Vector::Constant(2 * n, face);
    translation_lbA.resize(4 * n);
    translation_ubA.resize(4 * n);
    translation_lbA.tail(2 * n).setConstant(-std::sqrt(2.0) * face);
    translation_ubA.tail(2 * n).setConstant(std::sqrt(2.0) * face);
    translation_gradient.resize(2 * n);
    translation_solution.resize(2 * n);

    yaw_lower = Vector::Constant(n, -alpha_max);
    yaw_upper = Vector::Constant(n, alpha_max);
    yaw_lbA.resize(n);
    yaw_ubA.resize(n);
    yaw_gradient.resize(n);
    yaw_solution.resize(n);

    // Scalar yaw weights do not change, so the yaw Hessian is built once.
    yaw_hessian = Matrix::Zero(n, n);
    for (int j = 0; j < n; ++j) {
      for (int i = 0; i <= j; ++i) {
        double sum = 0.0;
        for (int k = j + 1; k <= n; ++k) {
          sum += c.q_yaw * dt * dt * (k - j - 0.5) * dt * dt * (k - i - 0.5) + c.q_omega * dt * dt;
        }
        yaw_hessian(j, i) = yaw_hessian(i, j) = 2.0 * sum;
      }
    }
    addInputRegularization(yaw_hessian, 1, c.r_alpha, c.r_dalpha, c.eps_regularization);

    headings.resize(n + 1);
    suffix_p0.resize(n + 1);
    suffix_p1.resize(n + 1);
    suffix_p2.resize(n + 1);
    suffix_v0.resize(n + 1);
    suffix_g0.resize(n + 1);
    suffix_g1.resize(n + 1);
    suffix_gv.resize(n + 1);
    yaw_s0.resize(n + 1);
    yaw_s1.resize(n + 1);
    yaw_sv.resize(n + 1);
    pred.resize(n + 1);
    sample_pred.resize(n + 1);
    sample_inputs.resize(n);

    qpOASES::Options options;
    options.setToMPC();
    options.printLevel = qpOASES::PL_NONE;
    translation_qp.setOptions(options);
    yaw_qp.setOptions(options);
  }

  int n;
  // qpOASES retains shallow matrix references. Build the next Hessian separately
  // so the previous QP's storage remains valid throughout the matrix hotstart.
  std::array<Matrix, 2> translation_hessians;
  int next_hessian{0};
  Matrix translation_constraints;
  Vector translation_lower, translation_upper, translation_lbA, translation_ubA;
  Vector translation_gradient, translation_solution;
  Matrix yaw_hessian;
  Matrix yaw_constraints;
  Vector yaw_lower, yaw_upper, yaw_lbA, yaw_ubA, yaw_gradient, yaw_solution;
  qpOASES::SQProblem translation_qp;
  qpOASES::QProblem yaw_qp;
  bool translation_initialized{false};
  bool yaw_initialized{false};

  // Suffix sums over k = m + 1 .. N, indexed by m; index N is zero.
  std::vector<double> headings;
  std::vector<Eigen::Matrix2d> suffix_p0, suffix_p1, suffix_p2, suffix_v0;
  std::vector<Eigen::Vector2d> suffix_g0, suffix_g1, suffix_gv;
  std::vector<double> yaw_s0, yaw_s1, yaw_sv;
  std::vector<State> pred;

  // Snapshot of the last successful solve, read by sample().
  bool has_sample{false};
  std::vector<State> sample_pred;
  std::vector<Eigen::Vector3d> sample_inputs;
  Eigen::Vector2d sample_drift{Eigen::Vector2d::Zero()};
};

namespace {

// Hotstart when possible; a failed hotstart resets and retries once from cold.
template <typename Hotstart, typename Init>
bool solveQp(
  qpOASES::QProblem & qp, bool & initialized, int max_wsr, const char * name, Hotstart hotstart, Init init)
{
  qpOASES::int_t nWSR = max_wsr;
  auto ret = qpOASES::SUCCESSFUL_RETURN;
  if (initialized) {
    ret = hotstart(nWSR);
    if (ret != qpOASES::SUCCESSFUL_RETURN) {
      qp.reset();
      initialized = false;
      nWSR = max_wsr;
    }
  }
  if (!initialized) {
    ret = init(nWSR);
  }
  if (ret != qpOASES::SUCCESSFUL_RETURN) {
    std::cout << color_text::RED << "[DynamicMpcSolver] " << name
              << " qpOASES Error Code: " << static_cast<int>(ret) << color_text::RESET << std::endl;
    qp.reset();
    initialized = false;
    return false;
  }
  initialized = true;
  return true;
}

}  // namespace

DynamicMpcSolver::DynamicMpcSolver(const DynamicMpcConfig & config)
{
  setConfig(config);
}

DynamicMpcSolver::~DynamicMpcSolver() = default;

double DynamicMpcSolver::translationalAccelerationLimit() const
{
  return std::min(config_.friction_coeff * kGravity, config_.max_force / config_.mass);
}

double DynamicMpcSolver::octagonFaceLimit() const
{
  return kOctagonFace * translationalAccelerationLimit();
}

bool DynamicMpcSolver::sample(double tau, DynamicSample & out) const
{
  if (!workspace_ || !workspace_->has_sample || !std::isfinite(tau)) {
    return false;
  }
  const auto & w = *workspace_;
  const int n = w.n;
  const double dt = config_.dt;
  // Inputs are piecewise constant, so velocity is linear inside each step.
  tau = std::clamp(tau, 0.0, n * dt);
  const int idx = std::min(static_cast<int>(tau / dt), n - 1);
  const double frac = tau / dt - idx;
  const State & a = w.sample_pred[idx];
  const State & b = w.sample_pred[idx + 1];
  // The chassis velocity loop does not see d: remove its drift so the command leads by
  // tau·(-d), i.e. the velocity that only the drive input would produce.
  out.velocity.vx = a.vx + (b.vx - a.vx) * frac - tau * w.sample_drift.x();
  out.velocity.vy = a.vy + (b.vy - a.vy) * frac - tau * w.sample_drift.y();
  out.velocity.omega = a.omega + (b.omega - a.omega) * frac;
  out.input = w.sample_inputs[idx];
  return true;
}

double DynamicMpcSolver::maxDisturbance() const
{
  return 0.5 * octagonFaceLimit() / std::sqrt(2.0);
}

void DynamicMpcSolver::setConfig(const DynamicMpcConfig & config)
{
  config_ = config;
  workspace_.reset();
  const bool valid = config_.horizon > 0 && config_.dt > 0.0 && config_.mass > 0.0 &&
                     config_.inertia_z > 0.0 && config_.friction_coeff > 0.0 && config_.max_force > 0.0 &&
                     config_.max_torque > 0.0 && config_.vx_min <= config_.vx_max &&
                     config_.vy_min <= config_.vy_max && config_.omega_min <= config_.omega_max;
  if (valid) {
    workspace_ = std::make_unique<Workspace>(config_, octagonFaceLimit(), angularAccelerationLimit());
  }
}

bool DynamicMpcSolver::solve(const State & curr,
  const std::vector<ReferencePoint> & ref_traj,
  DynamicControl & out_u,
  std::vector<State> * out_pred,
  bool commit,
  const Eigen::Vector2d & disturbance)
{
  if (!workspace_) {
    return false;
  }
  auto & w = *workspace_;
  // Any failed solve invalidates sample(); a success below sets it again.
  w.has_sample = false;
  if (static_cast<int>(ref_traj.size()) < config_.horizon + 1 || !finite(curr) ||
      !disturbance.allFinite()) {
    return false;
  }
  for (int k = 0; k <= config_.horizon; ++k) {
    if (!finite(ref_traj[k])) {
      return false;
    }
  }
  const int n = w.n;
  const double dt = config_.dt;
  const double dt2 = dt * dt;
  const double dt4 = dt2 * dt2;

  // --- Translation QP ---
  // Along/cross weights follow the reference velocity tangent. Before the first moving
  // sample use its tangent; afterwards hold the last one. A stationary reference falls
  // back to the reference yaw.
  int first_moving = -1;
  for (int k = 0; k <= n; ++k) {
    if (ref_traj[k].vel.norm() > kHeadingSpeed) {
      first_moving = k;
      break;
    }
  }
  double heading =
    first_moving < 0 ? 0.0 : std::atan2(ref_traj[first_moving].vel.y(), ref_traj[first_moving].vel.x());
  for (int k = 0; k <= n; ++k) {
    if (first_moving < 0) {
      heading = ref_traj[k].yaw;
    } else if (ref_traj[k].vel.norm() > kHeadingSpeed) {
      heading = std::atan2(ref_traj[k].vel.y(), ref_traj[k].vel.x());
    }
    w.headings[k] = heading;
  }

  const Eigen::Vector2d p0(curr.x, curr.y);
  const Eigen::Vector2d v0(curr.vx, curr.vy);
  const double d_max = maxDisturbance();
  const Eigen::Vector2d drift = disturbance.cwiseMax(-d_max).cwiseMin(d_max);
  w.suffix_p0[n].setZero();
  w.suffix_p1[n].setZero();
  w.suffix_p2[n].setZero();
  w.suffix_v0[n].setZero();
  w.suffix_g0[n].setZero();
  w.suffix_g1[n].setZero();
  w.suffix_gv[n].setZero();
  for (int k = n; k >= 1; --k) {
    const Eigen::Matrix2d wp = rotatedWeight(w.headings[k], config_.q_pos_along, config_.q_pos_cross);
    const Eigen::Matrix2d wv = rotatedWeight(w.headings[k], config_.q_vel_along, config_.q_vel_cross);
    // Free response with the input at zero: constant drift d integrates exactly.
    const Eigen::Vector2d ep = wp * (p0 + k * dt * v0 + 0.5 * k * k * dt2 * drift - ref_traj[k].pos);
    const Eigen::Vector2d ev = wv * (v0 + k * dt * drift - ref_traj[k].vel);
    const int m = k - 1;
    w.suffix_p0[m] = w.suffix_p0[k] + wp;
    w.suffix_p1[m] = w.suffix_p1[k] + k * wp;
    w.suffix_p2[m] = w.suffix_p2[k] + static_cast<double>(k) * k * wp;
    w.suffix_v0[m] = w.suffix_v0[k] + wv;
    w.suffix_g0[m] = w.suffix_g0[k] + ep;
    w.suffix_g1[m] = w.suffix_g1[k] + k * ep;
    w.suffix_gv[m] = w.suffix_gv[k] + ev;
  }

  // p_k depends on a_j (j < k) through dt²·(k - j - 1/2); v_k through dt. The block
  // Σ_{k>max(i,j)} Sp(k,j)·Sp(k,i)·Wp_k expands into the suffix moments above.
  auto & H = w.translation_hessians[w.next_hessian];
  for (int j = 0; j < n; ++j) {
    for (int i = 0; i <= j; ++i) {
      const Eigen::Matrix2d block =
        2.0 *
        (dt4 * (w.suffix_p2[j] - (i + j + 1.0) * w.suffix_p1[j] + (j + 0.5) * (i + 0.5) * w.suffix_p0[j]) +
          dt2 * w.suffix_v0[j]);
      H.block<2, 2>(2 * j, 2 * i) = block;
      if (i != j) {
        H.block<2, 2>(2 * i, 2 * j) = block;
      }
    }
  }
  addInputRegularization(H, 2, config_.r_acc, config_.r_jerk, config_.eps_regularization);

  for (int j = 0; j < n; ++j) {
    // Velocity difference quotient: the exact ZOH acceleration between reference samples.
    // The drive input that realises it under drift is a_ref - d (force feedforward m·(a_ref - d)).
    const Eigen::Vector2d acc_ref = (ref_traj[j + 1].vel - ref_traj[j].vel) / dt - drift;
    Eigen::Vector2d g = 2.0 * (dt2 * (w.suffix_g1[j] - (j + 0.5) * w.suffix_g0[j]) + dt * w.suffix_gv[j]) -
                        2.0 * config_.r_acc * acc_ref;
    if (j == 0) {
      g -= 2.0 * config_.r_jerk * last_acc_.head<2>();
    }
    w.translation_gradient.segment<2>(2 * j) = g;
  }

  // Recovery uses the diagonal input rate left after cancelling the drift on that axis.
  const double input_rate = kFeasibilityMargin * dt * octagonFaceLimit() / std::sqrt(2.0);
  for (int k = 0; k < n; ++k) {
    for (int axis = 0; axis < 2; ++axis) {
      const double vmin = axis == 0 ? config_.vx_min : config_.vy_min;
      const double vmax = axis == 0 ? config_.vx_max : config_.vy_max;
      const double rate = input_rate - dt * std::abs(drift(axis));
      double lb = 0.0;
      double ub = 0.0;
      relaxedBounds(v0(axis), vmin, vmax, rate, k, lb, ub);
      // Rows constrain dt·Σa_j; the free response v0 + (k + 1)·dt·d is moved to the bounds.
      const double v_free = v0(axis) + (k + 1) * dt * drift(axis);
      w.translation_lbA(2 * k + axis) = lb - v_free;
      w.translation_ubA(2 * k + axis) = ub - v_free;
    }
  }

  // --- Yaw QP ---
  w.yaw_s0[n] = w.yaw_s1[n] = w.yaw_sv[n] = 0.0;
  for (int k = n; k >= 1; --k) {
    const double e_yaw = config_.q_yaw * (curr.yaw + k * dt * curr.omega - ref_traj[k].yaw);
    const double e_omega = config_.q_omega * (curr.omega - ref_traj[k].yaw_rate);
    const int m = k - 1;
    w.yaw_s0[m] = w.yaw_s0[k] + e_yaw;
    w.yaw_s1[m] = w.yaw_s1[k] + k * e_yaw;
    w.yaw_sv[m] = w.yaw_sv[k] + e_omega;
  }
  for (int j = 0; j < n; ++j) {
    const double alpha_ref = (ref_traj[j + 1].yaw_rate - ref_traj[j].yaw_rate) / dt;
    double g = 2.0 * (dt2 * (w.yaw_s1[j] - (j + 0.5) * w.yaw_s0[j]) + dt * w.yaw_sv[j]) -
               2.0 * config_.r_alpha * alpha_ref;
    if (j == 0) {
      g -= 2.0 * config_.r_dalpha * last_acc_.z();
    }
    w.yaw_gradient(j) = g;
  }
  const double yaw_rate = kFeasibilityMargin * dt * angularAccelerationLimit();
  for (int k = 0; k < n; ++k) {
    double lb = 0.0;
    double ub = 0.0;
    relaxedBounds(curr.omega, config_.omega_min, config_.omega_max, yaw_rate, k, lb, ub);
    w.yaw_lbA(k) = lb - curr.omega;
    w.yaw_ubA(k) = ub - curr.omega;
  }

  if (!H.allFinite() || !w.translation_gradient.allFinite() || !w.yaw_gradient.allFinite()) {
    return false;
  }

  const int max_wsr = config_.max_working_set_recalculations;
  const bool translation_ok = solveQp(
    w.translation_qp,
    w.translation_initialized,
    max_wsr,
    "translation",
    [&](qpOASES::int_t & nWSR) {
      // SQProblem updates H as well as vectors; QProblem::hotstart would reuse a stale H.
      return w.translation_qp.hotstart(H.data(),
        w.translation_gradient.data(),
        w.translation_constraints.data(),
        w.translation_lower.data(),
        w.translation_upper.data(),
        w.translation_lbA.data(),
        w.translation_ubA.data(),
        nWSR);
    },
    [&](qpOASES::int_t & nWSR) {
      return w.translation_qp.init(H.data(),
        w.translation_gradient.data(),
        w.translation_constraints.data(),
        w.translation_lower.data(),
        w.translation_upper.data(),
        w.translation_lbA.data(),
        w.translation_ubA.data(),
        nWSR);
    });
  if (!translation_ok ||
      w.translation_qp.getPrimalSolution(w.translation_solution.data()) != qpOASES::SUCCESSFUL_RETURN ||
      !w.translation_solution.allFinite()) {
    w.translation_qp.reset();
    w.translation_initialized = false;
    return false;
  }
  // The solved QP now references this Hessian; the next cycle must build the other one.
  w.next_hessian = 1 - w.next_hessian;

  // The yaw Hessian and constraint matrix are constant, so only vectors are hotstarted.
  const bool yaw_ok = solveQp(
    w.yaw_qp,
    w.yaw_initialized,
    max_wsr,
    "yaw",
    [&](qpOASES::int_t & nWSR) {
      return w.yaw_qp.hotstart(w.yaw_gradient.data(),
        w.yaw_lower.data(),
        w.yaw_upper.data(),
        w.yaw_lbA.data(),
        w.yaw_ubA.data(),
        nWSR);
    },
    [&](qpOASES::int_t & nWSR) {
      return w.yaw_qp.init(w.yaw_hessian.data(),
        w.yaw_gradient.data(),
        w.yaw_constraints.data(),
        w.yaw_lower.data(),
        w.yaw_upper.data(),
        w.yaw_lbA.data(),
        w.yaw_ubA.data(),
        nWSR);
    });
  if (!yaw_ok || w.yaw_qp.getPrimalSolution(w.yaw_solution.data()) != qpOASES::SUCCESSFUL_RETURN ||
      !w.yaw_solution.allFinite()) {
    w.yaw_qp.reset();
    w.yaw_initialized = false;
    return false;
  }

  State state = curr;
  w.pred[0] = state;
  for (int k = 0; k < n; ++k) {
    const double ax = w.translation_solution(2 * k) + drift.x();
    const double ay = w.translation_solution(2 * k + 1) + drift.y();
    const double alpha = w.yaw_solution(k);
    state.x += dt * state.vx + 0.5 * dt2 * ax;
    state.y += dt * state.vy + 0.5 * dt2 * ay;
    state.yaw += dt * state.omega + 0.5 * dt2 * alpha;
    state.vx += dt * ax;
    state.vy += dt * ay;
    state.omega += dt * alpha;
    w.pred[k + 1] = state;
  }

  w.sample_pred = w.pred;
  for (int k = 0; k < n; ++k) {
    w.sample_inputs[k] =
      Eigen::Vector3d(w.translation_solution(2 * k), w.translation_solution(2 * k + 1), w.yaw_solution(k));
  }
  w.sample_drift = drift;
  w.has_sample = true;
  DynamicSample at_command;
  sample(config_.command_lookahead, at_command);
  out_u.velocity = at_command.velocity;
  out_u.acceleration =
    Eigen::Vector3d(w.translation_solution(0), w.translation_solution(1), w.yaw_solution(0));
  out_u.wrench = Eigen::Vector3d(config_.mass * out_u.acceleration.x(),
    config_.mass * out_u.acceleration.y(),
    config_.inertia_z * out_u.acceleration.z());
  if (out_pred) {
    *out_pred = w.pred;
  }
  if (commit) {
    last_acc_ = out_u.acceleration;
  }
  return true;
}

}  // namespace minco_controller
