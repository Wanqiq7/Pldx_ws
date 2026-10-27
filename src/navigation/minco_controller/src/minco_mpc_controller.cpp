#include "minco_controller/minco_mpc_controller.hpp"
#include "color_text.hpp"
#include "log.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include <iostream>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>

#include <Eigen/Geometry>

#include "nav2_util/node_utils.hpp"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace custom_log {
void log_info_line(std::string_view text)
{
  std::cout << text;
}
}  // namespace custom_log

namespace minco_controller {

MincoMpcController::~MincoMpcController()
{
  mpc_perf_monitor_.close();
}

double MincoMpcController::normalizeYaw(double yaw)
{
  return std::atan2(std::sin(yaw), std::cos(yaw));
}

void MincoMpcController::configureMpcPerfLogging(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
{
  const std::string default_mpc_csv_path = "/tmp/mpc_perf_detailed.csv";

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".performance.enable", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".performance.print_enable", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".performance.detailed_csv_enable", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".performance.odom_sub_debug_enable", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".performance.print_period_sec", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".performance.csv_flush_every_n", rclcpp::ParameterValue(30));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".performance.mpc_csv_path", rclcpp::ParameterValue(default_mpc_csv_path));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".performance.run_id", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".performance.scenario", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".performance.variant", rclcpp::ParameterValue(""));

  bool performance_enable = true;
  bool print_enable = true;
  bool detailed_csv_enable = false;
  bool odom_sub_debug_enable = true;
  double print_period_sec = 1.0;
  int csv_flush_every_n = 30;
  std::string mpc_csv_path = default_mpc_csv_path;
  std::string run_id;
  std::string scenario;
  std::string variant;
  node->get_parameter(name_ + ".performance.enable", performance_enable);
  node->get_parameter(name_ + ".performance.print_enable", print_enable);
  node->get_parameter(name_ + ".performance.detailed_csv_enable", detailed_csv_enable);
  node->get_parameter(name_ + ".performance.odom_sub_debug_enable", odom_sub_debug_enable);
  node->get_parameter(name_ + ".performance.print_period_sec", print_period_sec);
  node->get_parameter(name_ + ".performance.csv_flush_every_n", csv_flush_every_n);
  node->get_parameter(name_ + ".performance.mpc_csv_path", mpc_csv_path);
  node->get_parameter(name_ + ".performance.run_id", run_id);
  node->get_parameter(name_ + ".performance.scenario", scenario);
  node->get_parameter(name_ + ".performance.variant", variant);

  MpcPerformanceConfig perf_cfg;
  perf_cfg.enable = performance_enable;
  perf_cfg.print_enable = print_enable;
  perf_cfg.detailed_csv_enable = detailed_csv_enable;
  perf_cfg.odom_sub_debug_enable = odom_sub_debug_enable;
  perf_cfg.detailed_csv_path = mpc_csv_path;
  perf_cfg.run_id = run_id;
  perf_cfg.scenario = scenario;
  perf_cfg.variant = variant;
  perf_cfg.print_period_sec = print_period_sec;
  perf_cfg.csv_flush_every_n = csv_flush_every_n;

  mpc_perf_monitor_.configure(perf_cfg, logger_);
}

void MincoMpcController::configureDynamicModel(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
{
  const DynamicMpcConfig defaults;
  const auto declare = [&](const std::string & key, const rclcpp::ParameterValue & value) {
    nav2_util::declare_parameter_if_not_declared(node, name_ + "." + key, value);
  };
  declare("dynamic.secondary_reference", rclcpp::ParameterValue(false));
  declare("dynamic.dt", rclcpp::ParameterValue(defaults.dt));
  declare("dynamic.lookahead_time", rclcpp::ParameterValue(defaults.dt * defaults.horizon));
  declare("dynamic.mass", rclcpp::ParameterValue(defaults.mass));
  declare("dynamic.inertia_z", rclcpp::ParameterValue(defaults.inertia_z));
  declare("dynamic.friction_coeff", rclcpp::ParameterValue(defaults.friction_coeff));
  declare("dynamic.max_force", rclcpp::ParameterValue(defaults.max_force));
  declare("dynamic.max_torque", rclcpp::ParameterValue(defaults.max_torque));
  declare("dynamic.q_pos_along", rclcpp::ParameterValue(defaults.q_pos_along));
  declare("dynamic.q_pos_cross", rclcpp::ParameterValue(defaults.q_pos_cross));
  declare("dynamic.q_vel_along", rclcpp::ParameterValue(defaults.q_vel_along));
  declare("dynamic.q_vel_cross", rclcpp::ParameterValue(defaults.q_vel_cross));
  declare("dynamic.q_yaw", rclcpp::ParameterValue(defaults.q_yaw));
  declare("dynamic.q_omega", rclcpp::ParameterValue(defaults.q_omega));
  declare("dynamic.r_acc", rclcpp::ParameterValue(defaults.r_acc));
  declare("dynamic.r_jerk", rclcpp::ParameterValue(defaults.r_jerk));
  declare("dynamic.r_alpha", rclcpp::ParameterValue(defaults.r_alpha));
  declare("dynamic.r_dalpha", rclcpp::ParameterValue(defaults.r_dalpha));
  const DisturbanceObserverConfig observer_defaults;
  declare("dynamic.slope_gravity", rclcpp::ParameterValue(false));
  declare("dynamic.gravity_body_roll_offset", rclcpp::ParameterValue(0.0));
  declare("dynamic.disturbance_observer", rclcpp::ParameterValue(false));
  declare("dynamic.observer_bandwidth", rclcpp::ParameterValue(observer_defaults.bandwidth));
  declare("dynamic.observer_max_estimate", rclcpp::ParameterValue(observer_defaults.max_estimate));
  const WheelOdometryConfig wheel_defaults;
  declare("dynamic.wheel_odometry", rclcpp::ParameterValue(false));
  declare("dynamic.wheel_odometry_topic", rclcpp::ParameterValue(std::string("/sentry/chassis_velocity")));
  declare("dynamic.wheel_odometry_max_gap", rclcpp::ParameterValue(wheel_defaults.max_gap));
  declare("dynamic.wheel_odometry_timeout", rclcpp::ParameterValue(wheel_defaults.timeout));
  auto & c = dynamic_config_;
  double lookahead_time = defaults.dt * defaults.horizon;
  node->get_parameter(name_ + ".dynamic.secondary_reference", secondary_reference_);
  node->get_parameter(name_ + ".dynamic.dt", c.dt);
  node->get_parameter(name_ + ".dynamic.lookahead_time", lookahead_time);
  node->get_parameter(name_ + ".dynamic.mass", c.mass);
  node->get_parameter(name_ + ".dynamic.inertia_z", c.inertia_z);
  node->get_parameter(name_ + ".dynamic.friction_coeff", c.friction_coeff);
  node->get_parameter(name_ + ".dynamic.max_force", c.max_force);
  node->get_parameter(name_ + ".dynamic.max_torque", c.max_torque);
  node->get_parameter(name_ + ".dynamic.q_pos_along", c.q_pos_along);
  node->get_parameter(name_ + ".dynamic.q_pos_cross", c.q_pos_cross);
  node->get_parameter(name_ + ".dynamic.q_vel_along", c.q_vel_along);
  node->get_parameter(name_ + ".dynamic.q_vel_cross", c.q_vel_cross);
  node->get_parameter(name_ + ".dynamic.q_yaw", c.q_yaw);
  node->get_parameter(name_ + ".dynamic.q_omega", c.q_omega);
  node->get_parameter(name_ + ".dynamic.r_acc", c.r_acc);
  node->get_parameter(name_ + ".dynamic.r_jerk", c.r_jerk);
  node->get_parameter(name_ + ".dynamic.r_alpha", c.r_alpha);
  node->get_parameter(name_ + ".dynamic.r_dalpha", c.r_dalpha);
  const bool positive = c.dt > 0.0 && lookahead_time > 0.0 && c.mass > 0.0 && c.inertia_z > 0.0 &&
                        c.friction_coeff > 0.0 && c.max_force > 0.0 && c.max_torque > 0.0;
  const bool finite = std::isfinite(c.dt) && std::isfinite(lookahead_time) && std::isfinite(c.mass) &&
                      std::isfinite(c.inertia_z) && std::isfinite(c.friction_coeff) &&
                      std::isfinite(c.max_force) && std::isfinite(c.max_torque);
  if (!positive || !finite) {
    throw std::invalid_argument("Dynamic MPC physical parameters must be finite and positive");
  }
  c.horizon = static_cast<int>(std::ceil(lookahead_time / c.dt));
  dynamic_solver_ = std::make_unique<DynamicMpcSolver>(c);

  DisturbanceObserverConfig observer;
  node->get_parameter(name_ + ".dynamic.slope_gravity", slope_gravity_);
  node->get_parameter(name_ + ".dynamic.gravity_body_roll_offset", gravity_body_roll_offset_);
  node->get_parameter(name_ + ".dynamic.disturbance_observer", disturbance_observer_enabled_);
  node->get_parameter(name_ + ".dynamic.observer_bandwidth", observer.bandwidth);
  node->get_parameter(name_ + ".dynamic.observer_max_estimate", observer.max_estimate);
  if (disturbance_observer_enabled_ &&
      (!std::isfinite(observer.bandwidth) || observer.bandwidth <= 0.0 ||
        !std::isfinite(observer.max_estimate) || observer.max_estimate < 0.0)) {
    throw std::invalid_argument("Disturbance observer needs a positive bandwidth");
  }
  if (!std::isfinite(gravity_body_roll_offset_)) {
    throw std::invalid_argument("dynamic.gravity_body_roll_offset must be finite");
  }
  disturbance_observer_.setConfig(observer);

  WheelOdometryConfig wheel;
  std::string wheel_topic;
  node->get_parameter(name_ + ".dynamic.wheel_odometry", wheel_odometry_enabled_);
  node->get_parameter(name_ + ".dynamic.wheel_odometry_topic", wheel_topic);
  node->get_parameter(name_ + ".dynamic.wheel_odometry_max_gap", wheel.max_gap);
  node->get_parameter(name_ + ".dynamic.wheel_odometry_timeout", wheel.timeout);
  if (wheel_odometry_enabled_) {
    if (!std::isfinite(wheel.max_gap) || wheel.max_gap <= 0.0 || !std::isfinite(wheel.timeout) ||
        wheel.timeout <= 0.0 || wheel_topic.empty()) {
      throw std::invalid_argument("Wheel odometry needs a topic and positive max_gap/timeout");
    }
    {
      std::lock_guard<std::mutex> lk(data_mtx_);
      wheel_odometry_.setConfig(wheel);
    }
    // High-rate samples: keep a deep queue so executor jitter does not drop them.
    wheel_odom_sub_ = node->create_subscription<geometry_msgs::msg::TwistStamped>(wheel_topic,
      rclcpp::SensorDataQoS().keep_last(100),
      std::bind(&MincoMpcController::onWheelOdometry, this, std::placeholders::_1));
  } else {
    wheel_odom_sub_.reset();
  }

  RCLCPP_INFO(logger_,
    "%s: wheel odometry %s (topic=%s, max_gap=%.3f, timeout=%.3f)",
    name_.c_str(),
    wheel_odometry_enabled_ ? "enabled" : "disabled",
    wheel_topic.c_str(),
    wheel.max_gap,
    wheel.timeout);
  RCLCPP_INFO(logger_,
    "%s: force-control MPC (dt=%.3f, horizon=%d, mass=%.2f, Iz=%.3f, "
    "a_max=%.3f, alpha_max=%.3f, secondary_reference=%s, slope_gravity=%s, body_roll_offset=%.4f, "
    "disturbance_observer=%s, observer_bandwidth=%.2f, d_max=%.3f)",
    name_.c_str(),
    c.dt,
    c.horizon,
    c.mass,
    c.inertia_z,
    dynamic_solver_->translationalAccelerationLimit(),
    dynamic_solver_->angularAccelerationLimit(),
    secondary_reference_ ? "true" : "false",
    slope_gravity_ ? "true" : "false",
    gravity_body_roll_offset_,
    disturbance_observer_enabled_ ? "true" : "false",
    observer.bandwidth,
    dynamic_solver_->maxDisturbance());
}

void MincoMpcController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  auto node = parent.lock();
  logger_ = node->get_logger();

  global_frame_ = costmap_ros_->getGlobalFrameID();
  base_frame_ = costmap_ros_->getBaseFrameID();
  configureMpcPerfLogging(node);

  // 声明/读取参数
  nav2_util::declare_parameter_if_not_declared(node, name + ".vx_min", rclcpp::ParameterValue(-1.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".vx_max", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".vy_min", rclcpp::ParameterValue(-1.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".vy_max", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".omega_min", rclcpp::ParameterValue(-2.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".omega_max", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".control_delay_compensation", rclcpp::ParameterValue(0.25));

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".odom_frame", rclcpp::ParameterValue("camera_init"));
  nav2_util::declare_parameter_if_not_declared(node, name + ".map_frame", rclcpp::ParameterValue("map"));
  nav2_util::declare_parameter_if_not_declared(node, name + ".lidar_offset_x", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".lidar_offset_y", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".lidar_roll_offset", rclcpp::ParameterValue(0.0));

  // Velocity state bounds of the force-control MPC (map frame).
  node->get_parameter(name + ".vx_min", dynamic_config_.vx_min);
  node->get_parameter(name + ".vx_max", dynamic_config_.vx_max);
  node->get_parameter(name + ".vy_min", dynamic_config_.vy_min);
  node->get_parameter(name + ".vy_max", dynamic_config_.vy_max);
  node->get_parameter(name + ".omega_min", dynamic_config_.omega_min);
  node->get_parameter(name + ".omega_max", dynamic_config_.omega_max);
  node->get_parameter(name + ".control_delay_compensation", control_delay_compensation_);

  node->get_parameter(name + ".odom_frame", odom_frame_);
  node->get_parameter(name + ".map_frame", map_frame_);
  node->get_parameter(name + ".lidar_offset_x", lidar_offset_x_);
  node->get_parameter(name + ".lidar_offset_y", lidar_offset_y_);
  // Only logged: the slope term uses dynamic.gravity_body_roll_offset so the simulation's
  // level body frame is not tilted by the real robot's mounting value.
  double lidar_roll_offset = 0.0;
  node->get_parameter(name + ".lidar_roll_offset", lidar_roll_offset);

  nav2_util::declare_parameter_if_not_declared(
    node, name + ".reference_timeout_sec", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name + ".odom_timeout_sec", rclcpp::ParameterValue(0.3));
  node->get_parameter(name + ".reference_timeout_sec", reference_timeout_sec_);
  node->get_parameter(name + ".odom_timeout_sec", odom_timeout_sec_);
  if (!std::isfinite(reference_timeout_sec_) || reference_timeout_sec_ <= 0.0 ||
      !std::isfinite(odom_timeout_sec_) || odom_timeout_sec_ <= 0.0) {
    throw std::invalid_argument("Controller input timeouts must be finite and positive");
  }

  configureDynamicModel(node);

  // 订阅优化轨迹：/opt_path
  opt_path_sub_ = node->create_subscription<ros_interfaces::msg::MpcPositionCommand>("/opt_path",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&MincoMpcController::onOptPath, this, std::placeholders::_1));

  // 订阅里程计：用于延迟补偿和mpc输入状态
  auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    "/aft_mapped_to_init", odom_qos, std::bind(&MincoMpcController::onOdom, this, std::placeholders::_1));

  mpc_predict_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("/mpc_predict_path", 1);
  mpc_real_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("/mpc_real_path", 1);
  execution_pub_ =
    node->create_publisher<std_msgs::msg::Bool>("/minco/execution_enabled", rclcpp::QoS(10).reliable());
  chassis_cmd_pub_ = node->create_publisher<ros_interfaces::msg::ChassisCommand>(
    "/chassis_cmd_mpc", rclcpp::SensorDataQoS().keep_last(1));
  real_path_history_.clear();
  last_real_path_pub_time_ = node->now();

  RCLCPP_INFO(logger_,
    "%s: force MPC configured (delay_comp=%.3f, "
    "lidar_offset_x=%.3f, lidar_offset_y=%.3f, lidar_roll_offset=%.3f)",
    name_.c_str(),
    control_delay_compensation_,
    lidar_offset_x_,
    lidar_offset_y_,
    lidar_roll_offset);
}

void MincoMpcController::compensateLeverArm(double v_lidar_x,
  double v_lidar_y,
  double omega_z,
  double yaw,
  double & vx_global,
  double & vy_global,
  double & omega_global) const
{
  const double v_body_x = v_lidar_x + omega_z * lidar_offset_y_;
  const double v_body_y = v_lidar_y - omega_z * lidar_offset_x_;

  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  vx_global = v_body_x * cos_yaw - v_body_y * sin_yaw;
  vy_global = v_body_x * sin_yaw + v_body_y * cos_yaw;
  omega_global = omega_z;
  // std::cout << "Raw velocities: vx_lidar=" << v_lidar_x << ", vy_lidar=" << v_lidar_y
  //           << ", omega_z=" << omega_z << ", yaw=" << yaw << std::endl;
  // std::cout << "Compensated velocities: vx_global=" << vx_global << ", vy_global=" << vy_global
  //           << ", omega_global=" << omega_global << std::endl;
}

void MincoMpcController::extractGlobalVelocityAndYaw(const nav_msgs::msg::Odometry::SharedPtr & odom,
  double & vx_global,
  double & vy_global,
  double & omega_global,
  double & yaw_global) const
{
  // 1. 提取 3D 体轴速度 (IMU 物理中心的局部速度)
  Eigen::Vector3d v_body_imu(
    odom->twist.twist.linear.x, odom->twist.twist.linear.y, odom->twist.twist.linear.z);

  // 2. 提取全量 3D 姿态四元数
  Eigen::Quaterniond q(odom->pose.pose.orientation.w,
    odom->pose.pose.orientation.x,
    odom->pose.pose.orientation.y,
    odom->pose.pose.orientation.z);

  // 3. 3D 旋转：将倾斜机体系下的速度无损映射回与地面平行的全局系
  Eigen::Vector3d v_imu_global = q * v_body_imu;

  // 4. 提取角速度与 Yaw 角
  omega_global = odom->twist.twist.angular.z;
  yaw_global = tf2::getYaw(odom->pose.pose.orientation);

  // 5. 计算全局杆臂补偿 (将底盘水平安装偏置旋转到全局系)
  double offset_global_x = std::cos(yaw_global) * lidar_offset_x_ - std::sin(yaw_global) * lidar_offset_y_;
  double offset_global_y = std::sin(yaw_global) * lidar_offset_x_ + std::cos(yaw_global) * lidar_offset_y_;
  // 6. 计算底盘旋转中心的全局速度
  vx_global = v_imu_global.x() + omega_global * offset_global_y;
  vy_global = v_imu_global.y() - omega_global * offset_global_x;
}

void MincoMpcController::cleanup()
{
  reset();
  execution_pub_.reset();
  mpc_perf_monitor_.close();
  opt_path_sub_.reset();
  odom_sub_.reset();
  mpc_predict_path_pub_.reset();
  mpc_real_path_pub_.reset();
  chassis_cmd_pub_.reset();
  wheel_odom_sub_.reset();
  dynamic_solver_.reset();

  std::lock_guard<std::mutex> lk(data_mtx_);
  latest_opt_path_.reset();
  latest_odom_.reset();
}

void MincoMpcController::activate()
{
}

void MincoMpcController::deactivate()
{
  reset();
}

bool MincoMpcController::cancel()
{
  reset();
  return true;
}

void MincoMpcController::reset()
{
  std::lock_guard<std::mutex> lk(data_mtx_);
  execution_enabled_ = false;
  planning_failed_ = false;
  ++reference_version_;
  latest_opt_path_.reset();
  has_tracked_ref_ = false;
  if (execution_pub_) {
    std_msgs::msg::Bool enabled;
    enabled.data = false;
    execution_pub_->publish(enabled);
  }
  publishZeroCommand();
}

void MincoMpcController::publishZeroCommand()
{
  pending_wrench_.setZero();
  last_published_acceleration_.setZero();
  if (chassis_cmd_pub_) {
    ros_interfaces::msg::ChassisCommand chassis;
    if (auto node = node_.lock()) {
      chassis.header.stamp = node->now();
    }
    chassis.header.frame_id = global_frame_;
    chassis_cmd_pub_->publish(chassis);
  }
}

void MincoMpcController::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lk(plan_mtx_);
  global_plan_ = path;
  std::lock_guard<std::mutex> data_lock(data_mtx_);
  if (!execution_enabled_ && !path.poses.empty()) {
    latest_opt_path_.reset();
    planning_failed_ = false;
    has_tracked_ref_ = false;
    execution_start_ = node_.lock()->now();
    execution_enabled_ = true;
    ++reference_version_;
    std_msgs::msg::Bool enabled;
    enabled.data = true;
    execution_pub_->publish(enabled);
  }
}

void MincoMpcController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (!std::isfinite(speed_limit) || speed_limit < 0.0) {
    throw std::invalid_argument("Speed limit must be finite and nonnegative");
  }
  speed_limit_ = speed_limit;
  speed_limit_percentage_ = percentage;
}

void MincoMpcController::onOptPath(const ros_interfaces::msg::MpcPositionCommand::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(data_mtx_);

  // Ignore references queued before the new FollowPath execution began.
  if (!execution_enabled_ || !msg ||
      rclcpp::Time(msg->header.stamp, execution_start_.get_clock_type()) < execution_start_) {
    return;
  }
  ++reference_version_;
  planning_failed_ = msg->command_flag == ros_interfaces::msg::MpcPositionCommand::PLANNING_FAILED_COMMAND;
  if (planning_failed_ || msg->command_flag == ros_interfaces::msg::MpcPositionCommand::BLOCK_COMMAND ||
      msg->cmds.empty()) {
    latest_opt_path_.reset();
    has_tracked_ref_ = false;
    publishZeroCommand();
    return;
  }

  const uint32_t new_traj_id = (msg && !msg->cmds.empty()) ? msg->cmds.front().trajectory_id : 0u;
  const uint32_t old_traj_id = (latest_opt_path_ && !latest_opt_path_->cmds.empty())
                                 ? latest_opt_path_->cmds.front().trajectory_id
                                 : 0u;

  // 仅在轨迹 ID 切换时重置跟踪状态；同一轨迹重复发布（更新时间戳）不重置
  if (new_traj_id != old_traj_id) {
    has_tracked_ref_ = false;
  }

  latest_opt_path_ = msg;
  reference_received_at_ = std::chrono::steady_clock::now();
}

void MincoMpcController::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  auto node = node_.lock();
  if (node && msg) {
    mpc_perf_monitor_.recordOdomCallback(node->now(), msg->header.stamp);
  }

  std::lock_guard<std::mutex> lk(data_mtx_);
  latest_odom_ = msg;
  odom_received_at_ = std::chrono::steady_clock::now();
}

bool MincoMpcController::inputsFreshLocked() const
{
  const auto node = node_.lock();
  if (!node || !execution_enabled_ || !latest_opt_path_ || latest_opt_path_->cmds.empty() ||
      !latest_odom_) {
    return false;
  }
  const auto steady_now = std::chrono::steady_clock::now();
  const auto ros_now = node->now();
  const auto fresh = [&](const auto & stamp, const auto & received, double timeout) {
    const double age = (ros_now - rclcpp::Time(stamp, ros_now.get_clock_type())).seconds();
    const double receive_age = std::chrono::duration<double>(steady_now - received).count();
    return std::isfinite(age) && age >= 0.0 && age <= timeout && receive_age >= 0.0 &&
           receive_age <= timeout;
  };
  return fresh(latest_opt_path_->header.stamp, reference_received_at_, reference_timeout_sec_) &&
         fresh(latest_odom_->header.stamp, odom_received_at_, odom_timeout_sec_);
}

bool MincoMpcController::goalInPoseFrame(const geometry_msgs::msg::PoseStamped & goal,
  const std::string & pose_frame,
  geometry_msgs::msg::PoseStamped & out) const
{
  const std::string target = pose_frame.empty() ? global_frame_ : pose_frame;
  // An unstamped goal is taken to be in the pose frame already.
  if (goal.header.frame_id.empty() || goal.header.frame_id == target) {
    out = goal;
    out.header.frame_id = target;
    return true;
  }
  if (!tf_) {
    return false;
  }
  try {
    const auto transform = tf_->lookupTransform(target, goal.header.frame_id, tf2::TimePointZero);
    geometry_msgs::msg::PoseStamped transformed;
    tf2::doTransform(goal, transformed, transform);
    out = transformed;
    return true;
  } catch (const tf2::TransformException & ex) {
    if (const auto node = node_.lock()) {
      RCLCPP_WARN_THROTTLE(logger_,
        *node->get_clock(),
        5000,
        "Cannot transform goal from '%s' to '%s' for the goal check: %s",
        goal.header.frame_id.c_str(),
        target.c_str(),
        ex.what());
    }
    return false;
  }
}

bool MincoMpcController::transformPathToOdom(const ros_interfaces::msg::MpcPositionCommand::SharedPtr & opt,
  std::vector<ros_interfaces::msg::PositionCommand> & out_cmds) const
{
  std::string source_frame = opt->header.frame_id;
  if (source_frame.empty()) {
    source_frame = map_frame_;
    auto node_ptr = node_.lock();
    if (node_ptr) {
      RCLCPP_WARN_THROTTLE(logger_,
        *node_ptr->get_clock(),
        10000,
        "Received opt_path with empty frame_id, defaulting to '%s'",
        source_frame.c_str());
    }
  }

  // 从costmap获取全局坐标系，优先使用global_frame_，如果未设置则退回到odom_frame_
  std::string target_frame = global_frame_;
  if (target_frame.empty()) {
    target_frame = odom_frame_;
  }

  if (source_frame == target_frame) {
    out_cmds = opt->cmds;
    return true;
  }

  // 如果坐标系不同，尝试进行转换
  try {
    geometry_msgs::msg::TransformStamped transform =
      tf_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);

    out_cmds = opt->cmds;
    for (auto & cmd : out_cmds) {
      // Transform position
      geometry_msgs::msg::PointStamped p_in, p_out;
      p_in.header.frame_id = source_frame;
      p_in.point = cmd.position;
      tf2::doTransform(p_in, p_out, transform);
      cmd.position = p_out.point;

      // Transform velocity (vector)
      geometry_msgs::msg::Vector3Stamped v_in, v_out;
      v_in.header.frame_id = source_frame;
      v_in.vector = cmd.velocity;
      tf2::doTransform(v_in, v_out, transform);
      cmd.velocity = v_out.vector;

      // Transform acceleration (vector)
      geometry_msgs::msg::Vector3Stamped a_in, a_out;
      a_in.header.frame_id = source_frame;
      a_in.vector = cmd.acceleration;
      tf2::doTransform(a_in, a_out, transform);
      cmd.acceleration = a_out.vector;

      // Transform jerk (vector)
      geometry_msgs::msg::Vector3Stamped j_in, j_out;
      j_in.header.frame_id = source_frame;
      j_in.vector = cmd.jerk;
      tf2::doTransform(j_in, j_out, transform);
      cmd.jerk = j_out.vector;

      // Transform yaw
      double yaw_diff = tf2::getYaw(transform.transform.rotation);
      cmd.yaw = normalizeYaw(cmd.yaw + yaw_diff);
    }

  } catch (tf2::TransformException & ex) {
    return false;
  }
  return true;
}

bool MincoMpcController::prepareReference(const State & curr, ReferenceTrajectory & out) const
{
  auto node = node_.lock();
  if (!node) {
    return false;
  }
  const rclcpp::Time now = node->now();

  // 获取最新的优化轨迹
  ros_interfaces::msg::MpcPositionCommand::SharedPtr opt;
  {
    std::lock_guard<std::mutex> lk(data_mtx_);
    if (!inputsFreshLocked()) {
      return false;
    }
    opt = latest_opt_path_;
  }

  if (!opt || opt->cmds.empty()) {
    return false;
  }

  // 坐标系转换处理
  auto & cmds = out.cmds;
  if (!transformPathToOdom(opt, cmds)) {
    return false;
  }

  const size_t n_cmds = cmds.size();
  out.planner_dt = kPlannerDt;

  // 最近点搜索（基于最小欧式距离），并计算更精确的最近点投影索引
  size_t best_idx = 0;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < n_cmds; ++i) {
    const double dx = cmds[i].position.x - curr.x;
    const double dy = cmds[i].position.y - curr.y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best_idx = i;
    }
  }

  double nearest_idx_float = static_cast<double>(best_idx);

  if (best_idx < n_cmds - 1) {
    const auto & p_curr = cmds[best_idx];
    const auto & p_next = cmds[best_idx + 1];

    Eigen::Vector2d a_vec(p_next.position.x - p_curr.position.x, p_next.position.y - p_curr.position.y);
    Eigen::Vector2d b_vec(curr.x - p_curr.position.x, curr.y - p_curr.position.y);

    double len_sq = a_vec.squaredNorm();
    if (len_sq > 1e-6) {
      double projection = a_vec.dot(b_vec) / len_sq;
      if (projection > -0.5 && projection < 1.0) {
        nearest_idx_float += projection;
      }
    }
  }

  nearest_idx_float = std::max(0.0, nearest_idx_float);

  const uint32_t current_traj_id = (!opt->cmds.empty()) ? opt->cmds.front().trajectory_id : 0u;

  double current_idx_float = nearest_idx_float;
  current_idx_float = std::min(current_idx_float, static_cast<double>(n_cmds - 1));

  {
    std::lock_guard<std::mutex> lk(data_mtx_);
    tracked_ref_idx_ = current_idx_float;
    tracked_ref_time_ = now;
    tracked_opt_traj_id_ = current_traj_id;
    has_tracked_ref_ = true;
  }

  out.start_time = current_idx_float * kPlannerDt;
  return true;
}

ReferencePoint MincoMpcController::sampleReference(const ReferenceTrajectory & traj, double time) const
{
  const auto & cmds = traj.cmds;
  const size_t n_cmds = cmds.size();
  const double planner_dt = traj.planner_dt;
  double target_idx_float = time / planner_dt;

  if (target_idx_float < 0.0)
    target_idx_float = 0.0;
  if (target_idx_float > static_cast<double>(n_cmds - 1)) {
    target_idx_float = static_cast<double>(n_cmds - 1);
  }

  size_t target_idx = static_cast<size_t>(std::floor(target_idx_float));
  size_t next_idx = target_idx + 1;
  double alpha = target_idx_float - static_cast<double>(target_idx);

  ReferencePoint rp;
  if (next_idx < n_cmds) {
    // 二次前馈插值（泰勒展开），使用 P/V/A/J 进行插值，补偿控制延迟带来的误差。
    const double dt = std::max(0.0, std::min(planner_dt, alpha * planner_dt));
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;

    const Eigen::Vector2d p_i(cmds[target_idx].position.x, cmds[target_idx].position.y);
    const Eigen::Vector2d v_i(cmds[target_idx].velocity.x, cmds[target_idx].velocity.y);
    const Eigen::Vector2d a_i(cmds[target_idx].acceleration.x, cmds[target_idx].acceleration.y);
    const Eigen::Vector2d j_i(cmds[target_idx].jerk.x, cmds[target_idx].jerk.y);

    rp.pos = p_i + v_i * dt + 0.5 * a_i * dt2 + (1.0 / 6.0) * j_i * dt3;
    rp.vel = v_i + a_i * dt + 0.5 * j_i * dt2;

    // 线性角度插值
    rp.yaw = interpolateYaw(cmds[target_idx].yaw, cmds[next_idx].yaw, alpha);
    rp.yaw_rate = interpolate(cmds[target_idx].yaw_dot, cmds[next_idx].yaw_dot, alpha);
  } else {
    const auto & p_end = cmds.back();
    rp.pos = Eigen::Vector2d(p_end.position.x, p_end.position.y);
    rp.vel = Eigen::Vector2d(0.0, 0.0);
    rp.yaw = p_end.yaw;
    rp.yaw_rate = 0.0;
  }
  return rp;
}

void MincoMpcController::unwrapReferenceYaw(double curr_yaw, std::vector<ReferencePoint> & out_ref)
{
  // 对参考序列的 yaw 进行连续性处理
  if (!out_ref.empty()) {
    double diff = out_ref[0].yaw - curr_yaw;
    // 将 diff 限制在 [-PI, PI]
    diff = std::atan2(std::sin(diff), std::cos(diff));
    out_ref[0].yaw = curr_yaw + diff;

    for (size_t i = 1; i < out_ref.size(); ++i) {
      double d_yaw = out_ref[i].yaw - out_ref[i - 1].yaw;
      d_yaw = std::atan2(std::sin(d_yaw), std::cos(d_yaw));
      out_ref[i].yaw = out_ref[i - 1].yaw + d_yaw;
    }
  }
}

geometry_msgs::msg::TwistStamped MincoMpcController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  const bool record_perf = mpc_perf_monitor_.detailedCsvEnabled();
  const auto cycle_start =
    record_perf ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
  auto node = node_.lock();

  const rclcpp::Time now = node->now();
  std::optional<MpcPerfSample> perf;
  if (record_perf) {
    perf.emplace();
    perf->stamp_ros = now.seconds();
    perf->stamp_steady_ns = MpcPerformanceMonitor::steadyNowNs();
  }
  // Only a successful solve in this cycle supplies a drive wrench.
  Eigen::Vector3d wrench = Eigen::Vector3d::Zero();
  nav_msgs::msg::Odometry::SharedPtr latest_odom;
  uint64_t reference_version;
  std::chrono::steady_clock::time_point odom_received;
  bool inputs_fresh;
  bool planning_failed;
  {
    std::lock_guard<std::mutex> lk(data_mtx_);
    latest_odom = latest_odom_;
    reference_version = reference_version_;
    odom_received = odom_received_at_;
    inputs_fresh = inputsFreshLocked();
    planning_failed = planning_failed_;
  }
  // Stop the force channel explicitly before the BT recovers or aborts.
  if (planning_failed) {
    std::lock_guard<std::mutex> data_lock(data_mtx_);
    if (dynamic_solver_) {
      dynamic_solver_->resetLastAcceleration();
    }
    publishZeroCommand();
    throw nav2_core::NoValidControl("MincoPlanner gave up on the current goal");
  }
  auto finish = [&](geometry_msgs::msg::TwistStamped out_cmd, bool success, const std::string &) {
    // Serialize the final output with cancel()/BLOCK and reference replacement.
    // A cancellation during solve cannot be followed by an old nonzero command.
    {
      std::lock_guard<std::mutex> data_lock(data_mtx_);
      const auto finish_now = node->now();
      const double sampled_odom_age =
        latest_odom
          ? (finish_now - rclcpp::Time(latest_odom->header.stamp, finish_now.get_clock_type())).seconds()
          : std::numeric_limits<double>::infinity();
      if (!inputs_fresh || reference_version != reference_version_ || !inputsFreshLocked() ||
          sampled_odom_age < 0.0 || sampled_odom_age > odom_timeout_sec_ ||
          std::chrono::duration<double>(std::chrono::steady_clock::now() - odom_received).count() >
            odom_timeout_sec_) {
        out_cmd.twist = geometry_msgs::msg::Twist{};
        success = false;
      }
      // A suppressed command was never applied, so it must not seed the next solve.
      if (!success && dynamic_solver_) {
        dynamic_solver_->resetLastAcceleration();
      }
      pending_wrench_ = success ? wrench : Eigen::Vector3d::Zero();
      if (chassis_cmd_pub_) {
        ros_interfaces::msg::ChassisCommand chassis;
        chassis.header.stamp = finish_now;
        chassis.header.frame_id = global_frame_;
        chassis.wrench.force.x = pending_wrench_.x();
        chassis.wrench.force.y = pending_wrench_.y();
        chassis.wrench.torque.z = pending_wrench_.z();
        chassis_cmd_pub_->publish(chassis);
      }
      last_published_acceleration_ = pending_wrench_.head<2>() / dynamic_config_.mass;
    }
    if (perf) {
      perf->success = success;
      perf->cycle_time_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - cycle_start).count();
      if (last_mpc_perf_stamp_ns_ > 0 && perf->stamp_steady_ns > last_mpc_perf_stamp_ns_) {
        const double dt_sec = static_cast<double>(perf->stamp_steady_ns - last_mpc_perf_stamp_ns_) * 1.0e-9;
        if (dt_sec > 1.0e-9) {
          perf->controller_hz = 1.0 / dt_sec;
        }
      }
      last_mpc_perf_stamp_ns_ = perf->stamp_steady_ns;
      perf->cmd_fx = success ? wrench.x() : 0.0;
      perf->cmd_fy = success ? wrench.y() : 0.0;
      perf->cmd_mz = success ? wrench.z() : 0.0;
      mpc_perf_monitor_.recordMpcSample(*perf);
    }
    return out_cmd;
  };

  // 1) 构造当前状态：位置与航向来自 robot_base_frame（base_link_fake），
  // 世界系速度由 LIO 机体系速度按 body yaw 旋出。
  State curr;
  Eigen::Vector2d raw_velocity = Eigen::Vector2d::Zero();
  curr.x = pose.pose.position.x;
  curr.y = pose.pose.position.y;
  curr.yaw = normalizeYaw(tf2::getYaw(pose.pose.orientation));
  if (latest_odom) {
    double vx = latest_odom->twist.twist.linear.x;
    double vy = latest_odom->twist.twist.linear.y;
    double omega = latest_odom->twist.twist.angular.z;
    double yaw_lio = normalizeYaw(tf2::getYaw(latest_odom->pose.pose.orientation));
    curr.yaw = yaw_lio;
    double omega_global = 0.0;
    compensateLeverArm(vx, vy, omega, yaw_lio, curr.vx, curr.vy, omega_global);
    curr.omega = omega_global;
    raw_velocity = Eigen::Vector2d(curr.vx, curr.vy);

    const double noise_threshold = 0.03;
    if (std::abs(curr.vx) < noise_threshold) {
      curr.vx = 0.0;
    }
    if (std::abs(curr.vy) < noise_threshold) {
      curr.vy = 0.0;
    }
    if (std::abs(curr.omega) < noise_threshold) {
      curr.omega = 0.0;
    }
  } else {
    curr.vx = 0.0;
    curr.vy = 0.0;
    curr.omega = 0.0;
  }

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = now;
  cmd.header.frame_id = global_frame_;

  if (!inputs_fresh) {
    return finish(cmd, false, "STALE_INPUT");
  }

  // Sync Nav2 success semantics: if GoalChecker says reached, output zero command immediately.
  if (goal_checker != nullptr) {
    geometry_msgs::msg::PoseStamped goal_pose;
    bool has_goal_pose = false;
    {
      std::lock_guard<std::mutex> lk(plan_mtx_);
      if (!global_plan_.poses.empty()) {
        goal_pose = global_plan_.poses.back();
        if (goal_pose.header.frame_id.empty()) {
          goal_pose.header.frame_id = global_plan_.header.frame_id;
        }
        has_goal_pose = true;
      }
    }

    // The plan is in the planner frame (map) while the pose is in the local costmap frame
    // (camera_init); compare in the pose frame, as controller_server does.
    if (has_goal_pose && goalInPoseFrame(goal_pose, pose.header.frame_id, goal_pose) &&
        goal_checker->isGoalReached(pose.pose, goal_pose.pose, velocity)) {
      {
        std::lock_guard<std::mutex> lk(data_mtx_);
        has_tracked_ref_ = false;
        latest_opt_path_.reset();
        ++reference_version_;
      }
      if (dynamic_solver_) {
        dynamic_solver_->resetLastAcceleration();
      }
      cmd.twist.linear.x = 0.0;
      cmd.twist.linear.y = 0.0;
      cmd.twist.angular.z = 0.0;
      return finish(cmd, true, "NONE");
    }
  }

  std::vector<State> pred_states;
  const char * failure = solveDynamic(curr, raw_velocity, latest_odom, now, wrench, pred_states, perf);
  publishVisualization(pred_states, curr);
  if (failure) {
    cmd.twist = geometry_msgs::msg::Twist{};
    return finish(cmd, false, failure);
  }
  return finish(cmd, true, "NONE");
}

Eigen::Vector2d MincoMpcController::estimateDisturbance(
  const Eigen::Vector2d & raw_velocity, const nav_msgs::msg::Odometry::SharedPtr & odom)
{
  Eigen::Vector2d d = Eigen::Vector2d::Zero();
  if (!odom) {
    return d;
  }
  if (slope_gravity_) {
    const auto & o = odom->pose.pose.orientation;
    d += slopeGravity(Eigen::Quaterniond(o.w, o.x, o.y, o.z), gravity_body_roll_offset_);
  }
  if (disturbance_observer_enabled_) {
    // One observer step per odometry sample. The chassis has been executing the last
    // published command since the previous sample; the gravity term is part of the
    // known model, so only the remainder is estimated.
    const rclcpp::Time stamp(odom->header.stamp, observer_odom_stamp_.get_clock_type());
    if (stamp != observer_odom_stamp_) {
      observer_odom_stamp_ = stamp;
      Eigen::Vector2d applied;
      {
        std::lock_guard<std::mutex> lock(data_mtx_);
        applied = last_published_acceleration_;
      }
      disturbance_observer_.update(stamp.seconds(), raw_velocity, applied + d);
    }
    d += disturbance_observer_.estimate();
  }
  return d;
}

void MincoMpcController::onWheelOdometry(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  const double stamp = rclcpp::Time(msg->header.stamp, RCL_ROS_TIME).seconds();
  std::lock_guard<std::mutex> lk(data_mtx_);
  wheel_odometry_.push(
    stamp, Eigen::Vector3d(msg->twist.linear.x, msg->twist.linear.y, msg->twist.angular.z));
}

bool MincoMpcController::propagateWithWheelOdometry(
  const nav_msgs::msg::Odometry::SharedPtr & odom, const rclcpp::Time & now, State & curr)
{
  if (!odom) {
    return false;
  }
  // The pose (robot_base_frame via TF) and the LIO odom share the LIO stamp. Integrate the
  // body-frame wheel velocity from there to now. The heading for the rotation is the LIO
  // yaw: base_link_fake cancels yaw, so the pose yaw is not the chassis heading on the robot.
  const double t0 = rclcpp::Time(odom->header.stamp, RCL_ROS_TIME).seconds();
  const double t1 = now.seconds();
  const double yaw_lio = tf2::getYaw(odom->pose.pose.orientation);
  Eigen::Vector2d p1;
  double yaw1 = yaw_lio;
  Eigen::Vector3d v_body;
  bool ok = false;
  {
    std::lock_guard<std::mutex> lk(data_mtx_);
    ok = wheel_odometry_.propagate(t0, t1, Eigen::Vector2d(curr.x, curr.y), yaw_lio, p1, yaw1, v_body);
  }
  if (!ok) {
    return false;
  }
  const double c = std::cos(yaw1);
  const double s = std::sin(yaw1);
  curr.x = p1.x();
  curr.y = p1.y();
  curr.yaw = normalizeYaw(curr.yaw + (yaw1 - yaw_lio));
  curr.vx = c * v_body.x() - s * v_body.y();
  curr.vy = s * v_body.x() + c * v_body.y();
  curr.omega = v_body.z();
  return true;
}

const char * MincoMpcController::solveDynamic(const State & measured,
  const Eigen::Vector2d & raw_velocity,
  const nav_msgs::msg::Odometry::SharedPtr & odom,
  const rclcpp::Time & now,
  Eigen::Vector3d & out_wrench,
  std::vector<State> & pred_states,
  std::optional<MpcPerfSample> & perf)
{
  if (!dynamic_solver_) {
    return "SOLVER_UNAVAILABLE";
  }
  // Constrain the prediction itself; scaling a force would also weaken braking.
  auto limited = dynamic_config_;
  if (speed_limit_ > 0.0) {
    const double scale = speed_limit_percentage_ ? std::clamp(speed_limit_ / 100.0, 0.0, 1.0) : 1.0;
    const double axis_limit =
      speed_limit_percentage_ ? std::numeric_limits<double>::infinity() : speed_limit_ / std::sqrt(2.0);
    limited.vx_min = std::max(scale * limited.vx_min, -axis_limit);
    limited.vx_max = std::min(scale * limited.vx_max, axis_limit);
    limited.vy_min = std::max(scale * limited.vy_min, -axis_limit);
    limited.vy_max = std::min(scale * limited.vy_max, axis_limit);
  }
  const auto & previous = dynamic_solver_->config();
  if (limited.vx_min != previous.vx_min || limited.vx_max != previous.vx_max ||
      limited.vy_min != previous.vy_min || limited.vy_max != previous.vy_max) {
    dynamic_solver_->setConfig(limited);
  }
  const auto & config = dynamic_solver_->config();
  const double d_max = dynamic_solver_->maxDisturbance();
  const Eigen::Vector2d disturbance =
    estimateDisturbance(raw_velocity, odom).cwiseMax(-d_max).cwiseMin(d_max);
  if (perf) {
    perf->dist_x = disturbance.x();
    perf->dist_y = disturbance.y();
  }

  // control_delay_compensation is the lead from the odom stamp to when the command acts.
  // Wheel odometry measures the part up to now; only the remainder is predicted.
  State curr = measured;
  double delay = control_delay_compensation_;
  if (wheel_odometry_enabled_) {
    if (propagateWithWheelOdometry(odom, now, curr)) {
      delay -= (now - rclcpp::Time(odom->header.stamp, now.get_clock_type())).seconds();
      delay = std::max(0.0, delay);
    } else {
      ++wheel_odometry_fallbacks_;
      RCLCPP_WARN_THROTTLE(logger_,
        *node_.lock()->get_clock(),
        5000,
        "Wheel odometry unavailable or stale; using LIO-only state (%lu fallbacks)",
        static_cast<unsigned long>(wheel_odometry_fallbacks_));
    }
  }
  // Propagate with the last applied input plus the drift instead of constant velocity.
  if (delay > 1e-3) {
    Eigen::Vector3d a = dynamic_solver_->getLastAcceleration();
    a.head<2>() += disturbance;
    curr.x += curr.vx * delay + 0.5 * a.x() * delay * delay;
    curr.y += curr.vy * delay + 0.5 * a.y() * delay * delay;
    curr.yaw = normalizeYaw(curr.yaw + curr.omega * delay + 0.5 * a.z() * delay * delay);
    curr.vx += a.x() * delay;
    curr.vy += a.y() * delay;
    curr.omega += a.z() * delay;
  }

  // The projected position already accounts for the delay, so the reference starts there.
  ReferenceTrajectory traj;
  if (!prepareReference(curr, traj)) {
    return "NO_REFERENCE";
  }
  const int n = config.horizon;
  std::vector<ReferencePoint> ref(static_cast<size_t>(n + 1));
  for (int k = 0; k <= n; ++k) {
    ref[k] = sampleReference(traj, traj.start_time + k * config.dt);
  }
  unwrapReferenceYaw(curr.yaw, ref);
  if (perf) {
    perf->ref_vx = ref.front().vel.x();
    perf->ref_vy = ref.front().vel.y();
    perf->ref_wz = ref.front().yaw_rate;
  }

  const auto solve_start =
    perf ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
  DynamicControl u;
  bool success = dynamic_solver_->solve(curr, ref, u, &pred_states, !secondary_reference_, disturbance);
  if (success && secondary_reference_) {
    // Shrink each reference step by cos(angle(v_pred, v_ref)): when the first plan heads
    // away from the reference, along-track progress is traded for cross-track accuracy.
    constexpr double kMinSpeed = 0.05;
    double t = traj.start_time;
    std::vector<ReferencePoint> shrunk(ref.size());
    shrunk[0] = sampleReference(traj, t);
    for (int k = 1; k <= n; ++k) {
      const Eigen::Vector2d v_pred(pred_states[k].vx, pred_states[k].vy);
      const Eigen::Vector2d & v_ref = ref[k].vel;
      double alpha = 1.0;
      if (v_pred.norm() > kMinSpeed && v_ref.norm() > kMinSpeed) {
        alpha = std::max(0.0, v_pred.dot(v_ref) / (v_pred.norm() * v_ref.norm()));
      }
      t += alpha * config.dt;
      shrunk[k] = sampleReference(traj, t);
    }
    unwrapReferenceYaw(curr.yaw, shrunk);
    DynamicControl second;
    std::vector<State> second_pred;
    if (dynamic_solver_->solve(curr, shrunk, second, &second_pred, true, disturbance)) {
      u = second;
      pred_states = std::move(second_pred);
    } else {
      // Keep the valid first pass and record it as the applied input.
      success = dynamic_solver_->solve(curr, ref, u, &pred_states, true, disturbance);
    }
  }
  if (perf) {
    perf->solve_time_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - solve_start).count();
  }
  if (!success) {
    std::cout << color_text::RED << "[MincoMpc] Dynamic Solver Failed!" << color_text::RESET << std::endl;
    std::cout << "[MincoMpc] Curr: " << curr.x << ", " << curr.y << ", " << curr.yaw << " v: " << curr.vx
              << ", " << curr.vy << ", " << curr.omega << std::endl;
    std::cout << "[MincoMpc] Ref[0]: " << ref[0].pos.x() << ", " << ref[0].pos.y() << ", " << ref[0].yaw
              << std::endl;
    return "QP_FAILED";
  }

  // Apply the first optimized input of the receding horizon directly.
  out_wrench = u.wrench;
  return nullptr;
}

void MincoMpcController::publishVisualization(
  const std::vector<State> & pred_path, const State & curr_state)
{
  auto node = node_.lock();
  if (!node)
    return;

  rclcpp::Time now = node->now();

  // 1. 发布预测路径
  if (mpc_predict_path_pub_ && mpc_predict_path_pub_->get_subscription_count() > 0 && !pred_path.empty()) {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = now;
    path_msg.header.frame_id = global_frame_;

    for (const auto & s : pred_path) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path_msg.header;
      ps.pose.position.x = s.x;
      ps.pose.position.y = s.y;
      ps.pose.position.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0, 0, s.yaw);
      ps.pose.orientation = tf2::toMsg(q);
      path_msg.poses.push_back(ps);
    }
    mpc_predict_path_pub_->publish(path_msg);
  }

  // 2. 发布实际路径
  geometry_msgs::msg::PoseStamped ps;
  ps.header.stamp = now;
  ps.header.frame_id = global_frame_;
  ps.pose.position.x = curr_state.x;
  ps.pose.position.y = curr_state.y;
  ps.pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, curr_state.yaw);
  ps.pose.orientation = tf2::toMsg(q);

  if (real_path_history_.empty()) {
    real_path_history_.push_back(ps);
  } else {
    const auto & last = real_path_history_.back();
    double dist =
      std::hypot(last.pose.position.x - ps.pose.position.x, last.pose.position.y - ps.pose.position.y);
    // 简单的距离过滤，避免原地不动时数据堆积
    if (dist > 0.02) {
      real_path_history_.push_back(ps);
    }
  }

  // 保持历史长度
  if (real_path_history_.size() > 5000) {
    size_t remove_count = real_path_history_.size() - 5000;
    real_path_history_.erase(real_path_history_.begin(), real_path_history_.begin() + remove_count);
  }

  if (mpc_real_path_pub_ && mpc_real_path_pub_->get_subscription_count() > 0) {
    // 降频发布：1.0 Hz
    if ((now - last_real_path_pub_time_).seconds() > 1.0) {
      nav_msgs::msg::Path path_msg;
      path_msg.header.stamp = now;
      path_msg.header.frame_id = global_frame_;
      // 如果历史太长，可以只发布最近的一部分，或者对历史进行下采样（本例直接发布全部，但频率低）
      path_msg.poses = real_path_history_;
      mpc_real_path_pub_->publish(path_msg);
      last_real_path_pub_time_ = now;
    }
  }
}

}  // namespace minco_controller

PLUGINLIB_EXPORT_CLASS(minco_controller::MincoMpcController, nav2_core::Controller)
