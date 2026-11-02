#include "minco_controller/report_mpc_solver.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "acados_solver_report_mpc.h"

namespace minco_controller {
namespace {
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

struct ReportMpcSolver::Workspace
{
  explicit Workspace(const ReportMpcConfig & c) : n(c.motion.horizon)
  {
    capsule = report_mpc_acados_create_capsule();
    if (!capsule) {
      throw std::runtime_error("Failed to allocate acados solver capsule");
    }
    // The discrete dynamics use dt as a model parameter. Unit stage scaling
    // preserves the report's unscaled sum of tracking and input costs.
    std::vector<double> steps(n, 1.0);
    if (report_mpc_acados_create_with_discretization(capsule, n, steps.data()) != 0) {
      report_mpc_acados_free_capsule(capsule);
      capsule = nullptr;
      throw std::runtime_error("Failed to create acados report MPC solver");
    }
    auto * config = report_mpc_acados_get_nlp_config(capsule);
    auto * dims = report_mpc_acados_get_nlp_dims(capsule);
    auto * in = report_mpc_acados_get_nlp_in(capsule);
    auto * out = report_mpc_acados_get_nlp_out(capsule);
    const auto & m = c.motion;
    double params[] = {m.dt, m.mass, m.inertia_z};
    double lower_input[] = {-m.max_force, -m.max_force, -m.max_torque};
    double upper_input[] = {m.max_force, m.max_force, m.max_torque};
    double lower_state[] = {m.vx_min, m.vy_min, m.omega_min};
    double upper_state[] = {m.vx_max, m.vy_max, m.omega_max};
    Eigen::Matrix<double, 9, 9> W = Eigen::Matrix<double, 9, 9>::Zero();
    Eigen::Matrix<double, 6, 6> W_terminal = Eigen::Matrix<double, 6, 6>::Zero();
    for (int i = 0; i < 6; ++i) {
      W(i, i) = c.q[i];
      W_terminal(i, i) = c.q[i];
    }
    for (int i = 0; i < 3; ++i) {
      W(i + 6, i + 6) = c.r[i];
    }
    auto W_initial = W;
    W_initial.topLeftCorner<6, 6>().setZero();
    for (int k = 0; k < n; ++k) {
      report_mpc_acados_update_params(capsule, k, params, 3);
      ocp_nlp_cost_model_set(config, dims, in, k, "W", k == 0 ? W_initial.data() : W.data());
      ocp_nlp_constraints_model_set(config, dims, in, out, k, "lbu", lower_input);
      ocp_nlp_constraints_model_set(config, dims, in, out, k, "ubu", upper_input);
      if (k > 0) {
        ocp_nlp_constraints_model_set(config, dims, in, out, k, "lbx", lower_state);
        ocp_nlp_constraints_model_set(config, dims, in, out, k, "ubx", upper_state);
      }
    }
    ocp_nlp_cost_model_set(config, dims, in, n, "W", W_terminal.data());
    ocp_nlp_constraints_model_set(config, dims, in, out, n, "lbx", lower_state);
    ocp_nlp_constraints_model_set(config, dims, in, out, n, "ubx", upper_state);
  }

  ~Workspace()
  {
    if (capsule) {
      report_mpc_acados_free(capsule);
      report_mpc_acados_free_capsule(capsule);
    }
  }

  int n;
  report_mpc_solver_capsule * capsule{nullptr};
};

ReportMpcSolver::ReportMpcSolver(const ReportMpcConfig & config)
{
  setConfig(config);
}
ReportMpcSolver::~ReportMpcSolver() = default;

void ReportMpcSolver::setAppliedWrench(const Eigen::Vector3d & wrench)
{
  last_acceleration_ = Eigen::Vector3d(wrench.x() / config_.motion.mass,
    wrench.y() / config_.motion.mass,
    wrench.z() / config_.motion.inertia_z);
}

void ReportMpcSolver::setConfig(const ReportMpcConfig & c)
{
  const auto & m = c.motion;
  const bool valid = m.horizon > 0 && std::isfinite(m.dt) && m.dt > 0.0 && std::isfinite(m.mass) &&
                     m.mass > 0.0 && std::isfinite(m.inertia_z) && m.inertia_z > 0.0 &&
                     std::isfinite(m.max_force) && m.max_force > 0.0 && std::isfinite(m.max_torque) &&
                     m.max_torque > 0.0 && std::isfinite(m.vx_min) && std::isfinite(m.vx_max) &&
                     m.vx_min <= m.vx_max && std::isfinite(m.vy_min) && std::isfinite(m.vy_max) &&
                     m.vy_min <= m.vy_max && std::isfinite(m.omega_min) && std::isfinite(m.omega_max) &&
                     m.omega_min <= m.omega_max &&
                     std::all_of(c.q.begin(),
                       c.q.end(),
                       [](double x) {
                         return std::isfinite(x) && x >= 0.0;
                       }) &&
                     std::all_of(c.r.begin(), c.r.end(), [](double x) {
                       return std::isfinite(x) && x > 0.0;
                     });
  if (!valid) {
    throw std::invalid_argument("Invalid report MPC configuration");
  }
  auto workspace = std::make_unique<Workspace>(c);
  config_ = c;
  workspace_ = std::move(workspace);
}

bool ReportMpcSolver::solve(const State & current,
  const std::vector<ReferencePoint> & references,
  DynamicControl & control,
  std::vector<State> * predicted,
  bool commit)
{
  auto & w = *workspace_;
  if (!finite(current) || static_cast<int>(references.size()) < w.n ||
      !std::all_of(references.begin(), references.begin() + w.n, [](const auto & r) {
        return finite(r);
      })) {
    return false;
  }
  auto * config = report_mpc_acados_get_nlp_config(w.capsule);
  auto * dims = report_mpc_acados_get_nlp_dims(w.capsule);
  auto * in = report_mpc_acados_get_nlp_in(w.capsule);
  auto * out = report_mpc_acados_get_nlp_out(w.capsule);
  double x[] = {current.x, current.y, current.yaw, current.vx, current.vy, current.omega};
  ocp_nlp_constraints_model_set(config, dims, in, out, 0, "lbx", x);
  ocp_nlp_constraints_model_set(config, dims, in, out, 0, "ubx", x);
  for (int k = 1; k <= w.n; ++k) {
    const auto & r = references[k - 1];
    double ref[] = {r.pos.x(), r.pos.y(), r.yaw, r.vel.x(), r.vel.y(), r.yaw_rate, 0.0, 0.0, 0.0};
    ocp_nlp_cost_model_set(config, dims, in, k, "yref", ref);
  }
  if (report_mpc_acados_solve(w.capsule) != 0) {
    report_mpc_acados_reset(w.capsule, 1, 0, 0, 0);
    return false;
  }
  double u[3];
  ocp_nlp_out_get(config, dims, out, 0, "u", u);
  if (!std::all_of(std::begin(u), std::end(u), [](double value) {
        return std::isfinite(value);
      })) {
    report_mpc_acados_reset(w.capsule, 1, 0, 0, 0);
    return false;
  }
  control.wrench = Eigen::Vector3d(u[0], u[1], u[2]);
  const auto & m = config_.motion;
  control.acceleration = Eigen::Vector3d(u[0] / m.mass, u[1] / m.mass, u[2] / m.inertia_z);
  if (predicted) {
    predicted->resize(w.n + 1);
    (*predicted)[0] = current;
    for (int k = 1; k <= w.n; ++k) {
      ocp_nlp_out_get(config, dims, out, k, "x", x);
      State & state = (*predicted)[k];
      state = {x[0], x[1], x[2], x[3], x[4], x[5]};
      if (!finite(state)) {
        report_mpc_acados_reset(w.capsule, 1, 0, 0, 0);
        predicted->clear();
        return false;
      }
    }
  }
  if (commit) {
    last_acceleration_ = control.acceleration;
  }
  return true;
}
}  // namespace minco_controller
