#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "nav2_core/controller.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/bool.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

#include "ros_interfaces/msg/chassis_command.hpp"
#include "ros_interfaces/msg/mpc_position_command.hpp"

#include "log.hpp"
#include "minco_controller/disturbance.hpp"
#include "minco_controller/dynamic_mpc_solver.hpp"
#include "minco_controller/performance/mpc_performance_monitor.hpp"
#include "minco_controller/wheel_odometry.hpp"

namespace minco_controller {

class MincoMpcController : public nav2_core::Controller
{
public:
  // === Constructor & Lifecycle ===
  MincoMpcController() = default;
  ~MincoMpcController() override;

  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  // === Core Planning Interfaces ===
  geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
  bool cancel() override;
  void reset() override;

private:
  friend class MincoMpcControllerTestPeer;
  bool inputsFreshLocked() const;
  // === Callbacks ===
  void onOptPath(const ros_interfaces::msg::MpcPositionCommand::SharedPtr msg);
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void publishZeroCommand();

  // === Utility & Helper Functions ===
  void configureMpcPerfLogging(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);
  void configureDynamicModel(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);

  // --- Reference and Path Processing ---
  // Transformed /opt_path samples and the trajectory time projected from the current position.
  struct ReferenceTrajectory
  {
    std::vector<ros_interfaces::msg::PositionCommand> cmds;
    double planner_dt{0.05};
    double start_time{0.0};
  };
  bool prepareReference(const State & curr, ReferenceTrajectory & out) const;
  ReferencePoint sampleReference(const ReferenceTrajectory & traj, double time) const;
  static void unwrapReferenceYaw(double curr_yaw, std::vector<ReferencePoint> & ref);

  void onWheelOdometry(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  // Replaces the delayed odom state by wheel-integrated state at `now`; false keeps it unchanged.
  bool propagateWithWheelOdometry(
    const nav_msgs::msg::Odometry::SharedPtr & odom, const rclcpp::Time & now, State & curr);

  // Force-control MPC: fills the velocity command and prediction, or returns a failure reason.
  // raw_velocity is the global-frame odom velocity before the small-speed dead band.
  const char * solveDynamic(const State & measured,
    const Eigen::Vector2d & raw_velocity,
    const nav_msgs::msg::Odometry::SharedPtr & odom,
    const rclcpp::Time & now,
    geometry_msgs::msg::Twist & out_twist,
    std::vector<State> & pred_states,
    std::optional<MpcPerfSample> & perf);
  // Gravity along the ground plus the observer estimate, in the global frame.
  Eigen::Vector2d estimateDisturbance(
    const Eigen::Vector2d & raw_velocity, const nav_msgs::msg::Odometry::SharedPtr & odom);
  // Scales vx/vy to the Nav2 speed limit; returns the applied factor (1 when unlimited).
  double applySpeedLimit(double & vx, double & vy) const;

  // Plan end pose in the robot pose frame for the goal check; false if TF is unavailable.
  bool goalInPoseFrame(const geometry_msgs::msg::PoseStamped & goal,
    const std::string & pose_frame,
    geometry_msgs::msg::PoseStamped & out) const;

  bool transformPathToOdom(const ros_interfaces::msg::MpcPositionCommand::SharedPtr & opt,
    std::vector<ros_interfaces::msg::PositionCommand> & out_cmds) const;

  // --- Motion and Frame Utilities ---
  void compensateLeverArm(double v_lidar_x,
    double v_lidar_y,
    double omega_z,
    double yaw,
    double & vx_global,
    double & vy_global,
    double & omega_global) const;

  void extractGlobalVelocityAndYaw(const nav_msgs::msg::Odometry::SharedPtr & odom,
    double & vx_global,
    double & vy_global,
    double & omega_global,
    double & yaw_global) const;

  // --- Interpolation Utilities ---
  static double normalizeYaw(double yaw);
  inline static double interpolateYaw(double yaw1, double yaw2, double alpha)
  {
    double diff = std::atan2(std::sin(yaw2 - yaw1), std::cos(yaw2 - yaw1));
    return yaw1 + diff * alpha;
  }

  inline static double interpolate(double v1, double v2, double alpha) { return v1 + (v2 - v1) * alpha; }

  inline static Eigen::Vector2d interpolate(
    const Eigen::Vector2d & v1, const Eigen::Vector2d & v2, double alpha)
  {
    return v1 + (v2 - v1) * alpha;
  }

  // --- Visualization ---
  void publishVisualization(const std::vector<State> & pred_path, const State & curr_state);

  // === ROS 2 Interfaces (Publishers, Subscribers, Timers) ===
  rclcpp::Subscription<ros_interfaces::msg::MpcPositionCommand>::SharedPtr opt_path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mpc_predict_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mpc_real_path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_mpc_pub_;
  // Velocity plus planned drive wrench (force feedforward); same frame as /cmd_vel_mpc.
  rclcpp::Publisher<ros_interfaces::msg::ChassisCommand>::SharedPtr chassis_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr execution_pub_;
  bool execution_enabled_{false};
  uint64_t reference_version_{0};
  std::chrono::steady_clock::time_point reference_received_at_;
  std::chrono::steady_clock::time_point odom_received_at_;
  double reference_timeout_sec_{2.0};
  double odom_timeout_sec_{0.3};
  rclcpp::Time execution_start_;

  // === TF & Costmap & Frames ===
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("MincoMpcController")};
  std::string name_;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  std::string global_frame_;
  std::string base_frame_;
  std::string odom_frame_;
  std::string map_frame_;

  // === State Variables & Caches ===
  // --- Visualization Cache ---
  std::vector<geometry_msgs::msg::PoseStamped> real_path_history_;
  rclcpp::Time last_real_path_pub_time_;

  // --- Runtime Data Cache ---
  mutable std::mutex data_mtx_;
  ros_interfaces::msg::MpcPositionCommand::SharedPtr latest_opt_path_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  // Set by a PLANNING_FAILED_COMMAND of the current execution; fails FollowPath.
  bool planning_failed_{false};

  // --- Reference Tracking State ---
  // Track reference index over time when trajectory is not updated to avoid jumping back to start.
  mutable double tracked_ref_idx_{0.0};
  mutable rclcpp::Time tracked_ref_time_;
  mutable uint32_t tracked_opt_traj_id_{0};
  mutable bool has_tracked_ref_{false};

  // --- Nav2 Plan Cache ---
  nav_msgs::msg::Path global_plan_;
  mutable std::mutex plan_mtx_;

  // === Core Modules (Pointers to FSM, Optimizers, etc.) ===
  // Force-control MPC. The top-level vx/vy/omega bounds are read into dynamic_config_.
  DynamicMpcConfig dynamic_config_;
  std::unique_ptr<DynamicMpcSolver> dynamic_solver_;
  // Second pass shrinks reference time steps by cos(angle(v_pred, v_ref)) to favour cross-track accuracy.
  bool secondary_reference_{false};

  // === Configurations & Parameters ===
  // Dynamic speed limit from Nav2 setSpeedLimit().
  double speed_limit_{0.0};
  bool speed_limit_percentage_{false};

  // Other controller parameters.
  // Sample period of /opt_path (MincoPlanner publishes at 20 Hz).
  static constexpr double kPlannerDt = 0.05;
  double fixed_wz_{0.0};
  double deadzone_speed_threshold_{0.02};
  double control_delay_compensation_{0.25};
  double lidar_offset_x_{0.0};
  double lidar_offset_y_{0.0};
  bool use_small_gyro_mode_{true};
  MpcPerformanceMonitor mpc_perf_monitor_;
  long long last_mpc_perf_stamp_ns_{0};

  // Slope disturbance for the dynamic model (compute thread only).
  bool slope_gravity_{false};
  double gravity_body_roll_offset_{0.0};
  bool disturbance_observer_enabled_{false};
  DisturbanceObserver disturbance_observer_;
  rclcpp::Time observer_odom_stamp_{0, 0, RCL_ROS_TIME};
  // Last command actually published on /cmd_vel_mpc, i.e. what the chassis executes next.
  Eigen::Vector2d last_published_velocity_{0.0, 0.0};

  // Force feedforward (stage 3): F_ff = k·m·a_plan(t0 + ff_lookahead), sent with the velocity
  // setpoint. The velocity lead shrinks from command_lookahead to ff_lookahead as k grows, so the
  // firmware velocity PID and the feedforward do not both supply the planned acceleration.
  bool force_ff_enabled_{false};
  double force_ff_gain_{0.0};
  double force_ff_lookahead_{0.02};
  bool force_ff_yaw_{false};
  double force_ff_max_force_{100.0};
  double force_ff_max_torque_{10.0};
  // Wrench for the command being built; published with it, zeroed on any suppression.
  Eigen::Vector3d pending_wrench_ff_{0.0, 0.0, 0.0};

  // Wheel odometry (body-frame chassis velocity) integrated from the delayed LIO pose to now.
  bool wheel_odometry_enabled_{false};
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr wheel_odom_sub_;
  WheelOdometry wheel_odometry_;  // guarded by data_mtx_
  uint64_t wheel_odometry_fallbacks_{0};
};

}  // namespace minco_controller
