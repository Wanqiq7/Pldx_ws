#pragma once

#include <vector>

#include <Eigen/Core>

namespace minco_controller {

// 状态：全局系 (map) 下的位姿
struct State
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double vx{0.0};
  double vy{0.0};
  double omega{0.0};
};

// 参考点：来自 Minco 轨迹的高精度前馈信息
struct ReferencePoint
{
  Eigen::Vector2d pos{0.0, 0.0};  // map系位置
  Eigen::Vector2d vel{0.0, 0.0};  // map系速度(前馈)
  double yaw{0.0};                // 参考航向
  double yaw_rate{0.0};           // 参考角速度(前馈)
};

// 力控 MPC 配置：状态和力/力矩均采用全局坐标系。
struct DynamicMpcConfig
{
  double dt{0.02};
  int horizon{40};
  // 输出的平移合力受 min(mu·m·g, max_force) 限制。
  double mass{25.0};
  double inertia_z{1.0};
  double friction_coeff{0.6};
  double max_force{60.0};
  double max_torque{5.0};

  // 速度状态约束（map 系）
  double vx_min{-1.0};
  double vx_max{1.0};
  double vy_min{-1.0};
  double vy_max{1.0};
  double omega_min{-2.0};
  double omega_max{2.0};
};

// 力控 MPC 输出
struct DynamicControl
{
  Eigen::Vector3d acceleration{0.0, 0.0, 0.0};  // 首步 [ax, ay, alpha]，map 系
  Eigen::Vector3d wrench{0.0, 0.0, 0.0};        // 首步 [Fx, Fy, Mz] = [m·ax, m·ay, Iz·alpha]
};

}  // namespace minco_controller
