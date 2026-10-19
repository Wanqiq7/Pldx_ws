#pragma once

#include <memory>
#include <vector>

#include <Eigen/Core>

#include "minco_controller/mpc_types.hpp"

namespace minco_controller {

// 纯 C++ MPC 求解器：不依赖 ROS，仅依赖 Eigen 与 qpOASES。
class MpcSolver
{
public:
  explicit MpcSolver(const MPCConfig & config);
  ~MpcSolver();

  void setConfig(const MPCConfig & config);

  // @brief 求解一次 MPC。
  // @param curr 当前状态 (map)
  // @param ref_traj 参考轨迹 (horizon 个点，若不足由插件层补齐)
  // @param out_u 输出控制 (map)
  // @param out_pred (可选) 输出预测轨迹
  // @return 是否求解成功
  bool solve(const State & curr,
    const std::vector<ReferencePoint> & ref_traj,
    Control & out_u,
    std::vector<State> * out_pred = nullptr);

  Eigen::Vector3d getLastControl() const { return last_u_global_; }

private:
  struct Workspace;
  void buildCondensedQP(const State & curr, const std::vector<ReferencePoint> & ref_traj);

  MPCConfig config_;
  std::unique_ptr<Workspace> workspace_;

  // 用于加速度约束的上一时刻 global 控制量
  Eigen::Vector3d last_u_global_{0.0, 0.0, 0.0};
};

}  // namespace minco_controller
