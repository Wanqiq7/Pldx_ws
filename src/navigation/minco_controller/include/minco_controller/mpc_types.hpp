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

// 力控（动力学）MPC 配置
// 状态 [px, py, yaw, vx, vy, omega]，输入为质量归一化的力/力矩 [Fx/m, Fy/m, Mz/Iz]（map 系）。
struct DynamicMpcConfig
{
  double dt{0.02};
  int horizon{40};
  // 物理参数：平移合力受 min(mu·m·g, max_force) 限制，按摩擦圆内接八边形线性化。
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

  // 位置/速度误差权重按参考速度切向旋转为沿轨迹与横向分量。
  double q_pos_along{5.0};
  double q_pos_cross{40.0};
  double q_vel_along{1.0};
  double q_vel_cross{2.0};
  double q_yaw{10.0};
  double q_omega{0.5};
  // 输入相对参考加速度（力前馈）的偏差与相邻输入差分权重。
  double r_acc{0.05};
  double r_jerk{0.002};
  double r_alpha{0.02};
  double r_dalpha{0.001};

  int max_working_set_recalculations{200};
  double eps_regularization{1e-9};
};

// 力控 MPC 输出
struct DynamicControl
{
  Eigen::Vector3d acceleration{0.0, 0.0, 0.0};  // 首步 [ax, ay, alpha]，map 系
  Eigen::Vector3d wrench{0.0, 0.0, 0.0};        // 首步 [Fx, Fy, Mz] = [m·ax, m·ay, Iz·alpha]
};

}  // namespace minco_controller
