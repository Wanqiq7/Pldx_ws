#pragma once

#include <memory>
#include <vector>

#include <Eigen/Core>

#include "minco_controller/mpc_types.hpp"

namespace minco_controller {

// 力控 MPC：双积分动力学模型，零阶保持精确离散。
//   p_{k+1} = p_k + dt·v_k + dt²/2·a_k,  v_{k+1} = v_k + dt·a_k,  a = F/m (yaw 为 Mz/Iz)
// 平移与 yaw 只通过代价耦合，分解为 2N 维平移 QP 与 N 维 yaw QP 求解。
class DynamicMpcSolver
{
public:
  explicit DynamicMpcSolver(const DynamicMpcConfig & config);
  ~DynamicMpcSolver();

  void setConfig(const DynamicMpcConfig & config);
  const DynamicMpcConfig & config() const { return config_; }

  // @param curr 当前状态（含速度）
  // @param ref_traj horizon + 1 个参考点，ref_traj[k] 对应 t0 + k·dt
  // @param out_pred (可选) horizon + 1 个预测状态（含扰动漂移），首个为 curr
  // @param commit 成功时是否把首步输入记为已下发（用于输入差分代价与延迟外推）
  // @param disturbance map 系平移扰动加速度 d（重力分量、观测器估计），模型为
  //        v_{k+1} = v_k + dt·(a_k + d)。按 maxDisturbance() 截断以保留可行余量。
  //        a_ref 前馈变为 a_ref - d，输出 wrench 已包含抵消 d 的驱动力。
  bool solve(const State & curr,
    const std::vector<ReferencePoint> & ref_traj,
    DynamicControl & out_u,
    std::vector<State> * out_pred = nullptr,
    bool commit = true,
    const Eigen::Vector2d & disturbance = Eigen::Vector2d::Zero());

  // 最近一次已下发的 [ax, ay, alpha]
  Eigen::Vector3d getLastAcceleration() const { return last_acc_; }
  void resetLastAcceleration() { last_acc_.setZero(); }

  // 平移合加速度上限 min(mu·g, max_force/m)，以及线性化后八边形的面距离。
  double translationalAccelerationLimit() const;
  double octagonFaceLimit() const;
  double angularAccelerationLimit() const { return config_.max_torque / config_.inertia_z; }
  // 每轴扰动上限：输入在抵消扰动后仍保留一半的对角恢复能力。
  double maxDisturbance() const;

private:
  struct Workspace;

  DynamicMpcConfig config_;
  std::unique_ptr<Workspace> workspace_;
  Eigen::Vector3d last_acc_{0.0, 0.0, 0.0};
};

}  // namespace minco_controller
