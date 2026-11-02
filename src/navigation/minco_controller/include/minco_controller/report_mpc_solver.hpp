#pragma once

#include <array>
#include <memory>
#include <vector>

#include <Eigen/Core>

#include "minco_controller/mpc_types.hpp"

namespace minco_controller {

// Report 5.5.5.3: x[k+1] = A x[k] + B u[k], u = [Fx, Fy, Mz].
// The first-order discretization, tracking cost and component-wise bounds match
// the Python report model. The generated acados solver handles the full horizon.
struct ReportMpcConfig
{
  DynamicMpcConfig motion;
  std::array<double, 6> q{800.0, 800.0, 20.0, 100.0, 100.0, 5.0};
  std::array<double, 3> r{0.001, 0.001, 0.5};
};

class ReportMpcSolver
{
public:
  explicit ReportMpcSolver(const ReportMpcConfig & config);
  ~ReportMpcSolver();
  void setConfig(const ReportMpcConfig & config);
  const ReportMpcConfig & config() const { return config_; }

  // references[0..N-1] correspond to predicted x[1..N], as in the Python model.
  bool solve(const State & current,
    const std::vector<ReferencePoint> & references,
    DynamicControl & control,
    std::vector<State> * predicted = nullptr,
    bool commit = true);
  Eigen::Vector3d getLastAcceleration() const { return last_acceleration_; }
  void resetLastAcceleration() { last_acceleration_.setZero(); }
  void setAppliedWrench(const Eigen::Vector3d & wrench);

private:
  struct Workspace;
  ReportMpcConfig config_;
  std::unique_ptr<Workspace> workspace_;
  Eigen::Vector3d last_acceleration_{Eigen::Vector3d::Zero()};
};

}  // namespace minco_controller
