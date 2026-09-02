#include "ros2_libxr/ros2libxr.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "rclcpp_components/register_node_macro.hpp"

namespace rm_serial_driver {
namespace {
constexpr const char * kNavTopic = "nav_data";
constexpr const char * kBehaviorTopic = "behavior_data";
constexpr const char * kTeamTopic = "team_info";
constexpr const char * kGameTopic = "game_info";
constexpr const char * kOnlineTopic = "online_info";
constexpr const char * kOfflineTopic = "offline_info";
constexpr const char * kRadarTopic = "radar_info";
constexpr size_t kImuHistoryCapacity = 128;

template <typename T> bool valid_payload(const LibXR::RawData & data)
{
  return data.addr_ != nullptr && data.size_ == sizeof(T);
}
}  // namespace

void XRobotMain(LibXR::HardwareContainer & hw)
{
  static LibXR::ApplicationManager appmgr;
  static SharedTopic shared_topic_rx(hw,
    appmgr,
    "uart_client",
    256,
    {{kTeamTopic}, {kGameTopic}, {kOnlineTopic}, {kOfflineTopic}, {kRadarTopic}});
  static SharedTopicClient shared_topic_tx(hw, appmgr, "uart_client", 16, {{kNavTopic}, {kBehaviorTopic}});
  (void)shared_topic_rx;
  (void)shared_topic_tx;
}

RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options) : Node("rm_serial_driver", options)
{
  declare_parameter<int64_t>("communication.imu_yaw_window_ms", 40);
  imu_yaw_window_ms_ = get_parameter("communication.imu_yaw_window_ms").as_int();

  LibXR::PlatformInit();
  ramfs_ = std::make_unique<LibXR::RamFS>();
  uart_client_ = std::make_unique<LibXR::LinuxUART>(
    // USB HS composite CDC control-interface string.  VID/PID alone is
    // ambiguous when navigation and vision share one physical device.
    "16d0",
    "1492",
    "XRUSB Navigation Control",
    921600,
    LibXR::LinuxUART::Parity::NO_PARITY,
    8,
    1);
  terminal_ = std::make_unique<LibXR::Terminal<1024, 64, 16, 128>>(*ramfs_);
  term_thread_ = std::make_unique<LibXR::Thread>();
  term_thread_->Create(terminal_.get(),
    LibXR::Terminal<1024, 64, 16, 128>::ThreadFun,
    "terminal",
    81900,
    LibXR::Thread::Priority::MEDIUM);

  LibXR::HardwareContainer hw_container{LibXR::Entry<LibXR::RamFS>({*ramfs_, {"ramfs"}}),
    LibXR::Entry<LibXR::UART>({*uart_client_, {"uart_client"}})};

  nav_data_topic_ = LibXR::Topic::CreateTopic<ChassisTarget>(kNavTopic);
  behavior_data_topic_ = LibXR::Topic::CreateTopic<BehaviorData>(kBehaviorTopic);
  team_info_topic_ = LibXR::Topic::CreateTopic<TeamInfo>(kTeamTopic);
  game_info_topic_ = LibXR::Topic::CreateTopic<GameInfo>(kGameTopic);
  online_info_topic_ = LibXR::Topic::CreateTopic<SentryInfoOnline>(kOnlineTopic);
  offline_info_topic_ = LibXR::Topic::CreateTopic<SentryInfoOffline>(kOfflineTopic);
  radar_info_topic_ = LibXR::Topic::CreateTopic<RadarInfo>(kRadarTopic);
  XRobotMain(hw_container);

  team_pub_ = create_publisher<ros_interfaces::msg::TeamInformation>("/sentry/team_info", 10);
  game_pub_ = create_publisher<ros_interfaces::msg::GameInfo>("/sentry/game_info", 10);
  online_pub_ = create_publisher<ros_interfaces::msg::SentryInfoOnline>("/sentry/online_info", 10);
  offline_pub_ = create_publisher<ros_interfaces::msg::SentryInfoOffline>("/sentry/offline_info", 10);
  radar_pub_ = create_publisher<ros_interfaces::msg::RadarInfo>("/sentry/radar_info", 10);

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel_mpc", rclcpp::QoS(1), [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      cmd_vel_ = *msg;
    });
  cmd_force_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
    "/cmd_force_mpc", rclcpp::QoS(1), [this](geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      cmd_force_ = *msg;
    });
  behavior_sub_ = create_subscription<ros_interfaces::msg::Behavior>(
    "/sentry/behaivor_send", rclcpp::QoS(1), [this](ros_interfaces::msg::Behavior::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      behavior_ = *msg;
    });
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/aft_mapped_to_init",
    rclcpp::QoS(1).best_effort().durability_volatile(),
    [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      odom_ = *msg;
      update_delta_yaw_locked();
    });

  auto register_callback = [this](LibXR::Topic & topic, auto fn) {
    auto cb = LibXR::Topic::Callback::Create(fn, this);
    topic.RegisterCallback(cb);
  };
  register_callback(team_info_topic_, [](bool, RMSerialDriver * self, LibXR::RawData & raw) {
    if (valid_payload<TeamInfo>(raw))
      self->publish_team(*reinterpret_cast<const TeamInfo *>(raw.addr_));
  });
  register_callback(game_info_topic_, [](bool, RMSerialDriver * self, LibXR::RawData & raw) {
    if (valid_payload<GameInfo>(raw))
      self->publish_game(*reinterpret_cast<const GameInfo *>(raw.addr_));
  });
  register_callback(online_info_topic_, [](bool, RMSerialDriver * self, LibXR::RawData & raw) {
    if (valid_payload<SentryInfoOnline>(raw))
      self->publish_online(*reinterpret_cast<const SentryInfoOnline *>(raw.addr_));
  });
  register_callback(offline_info_topic_, [](bool, RMSerialDriver * self, LibXR::RawData & raw) {
    if (valid_payload<SentryInfoOffline>(raw))
      self->publish_offline(*reinterpret_cast<const SentryInfoOffline *>(raw.addr_), raw);
  });
  register_callback(radar_info_topic_, [](bool, RMSerialDriver * self, LibXR::RawData & raw) {
    if (valid_payload<RadarInfo>(raw))
      self->publish_radar(*reinterpret_cast<const RadarInfo *>(raw.addr_));
  });

  tx_timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() {
    publish_chassis();
  });
  RCLCPP_INFO(get_logger(), "ROS2_LIBXR contract topics initialized");
}

void RMSerialDriver::update_delta_yaw_locked()
{
  const auto stamp = rclcpp::Time(odom_.header.stamp);
  if (stamp.nanoseconds() == 0 || imu_yaw_history_.empty())
    return;
  auto best = imu_yaw_history_.end();
  int64_t best_abs = std::numeric_limits<int64_t>::max();
  for (auto it = imu_yaw_history_.begin(); it != imu_yaw_history_.end(); ++it) {
    const int64_t dt = (it->first - stamp).nanoseconds();
    if (std::llabs(dt) < best_abs) {
      best_abs = std::llabs(dt);
      best = it;
    }
  }
  if (best == imu_yaw_history_.end() || best_abs > imu_yaw_window_ms_ * 1000000LL)
    return;
  const auto & orientation = odom_.pose.pose.orientation;
  tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  delta_yaw_ = static_cast<float>(yaw - best->second);
  while (delta_yaw_ > static_cast<float>(M_PI))
    delta_yaw_ -= 2.0F * static_cast<float>(M_PI);
  while (delta_yaw_ < -static_cast<float>(M_PI))
    delta_yaw_ += 2.0F * static_cast<float>(M_PI);
  delta_yaw_initialized_ = std::isfinite(delta_yaw_);
}

void RMSerialDriver::publish_chassis()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  ChassisTarget nav{};
  nav.vx_mps = static_cast<float>(cmd_vel_.linear.x);
  nav.vy_mps = static_cast<float>(cmd_vel_.linear.y);
  nav.vw_rad_s = static_cast<float>(cmd_vel_.angular.z);
  nav.current_vx = static_cast<float>(odom_.twist.twist.linear.x);
  nav.current_vy = static_cast<float>(odom_.twist.twist.linear.y);
  nav.current_vw = static_cast<float>(odom_.twist.twist.angular.z);
  nav.fx_global = static_cast<float>(cmd_force_.wrench.force.x);
  nav.fy_global = static_cast<float>(cmd_force_.wrench.force.y);
  nav.fw_global = static_cast<float>(cmd_force_.wrench.torque.z);
  const auto & orientation = odom_.pose.pose.orientation;
  tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  nav.current_yaw = static_cast<float>(yaw);
  nav.use_speed_control = true;
  nav.delta_yaw = delta_yaw_;
  if (behavior_.tunnel_escape_active) {
    nav.vx_mps = behavior_.tunnel_escape_vx;
    nav.vy_mps = behavior_.tunnel_escape_vy;
    nav.vw_rad_s = behavior_.tunnel_escape_wz;
  }
  if (behavior_.tunnel_prepare_active && !behavior_.tunnel_ready) {
    nav.vx_mps *= 0.15F;
    nav.vy_mps *= 0.15F;
  }
  if (!delta_yaw_initialized_) {
    nav.vx_mps = nav.vy_mps = nav.fx_global = nav.fy_global = 0.0F;
  } else {
    const float theta = nav.current_yaw - delta_yaw_;
    const float cos_theta = std::cos(theta);
    const float sin_theta = std::sin(theta);
    const float vx_b = nav.vx_mps * cos_theta + nav.vy_mps * sin_theta;
    const float vy_b = -nav.vx_mps * sin_theta + nav.vy_mps * cos_theta;
    nav.vx_mps = vx_b;
    nav.vy_mps = vy_b;
  }
  BehaviorData behavior{};
  behavior.pitch_mode = behavior_.pitch_mode;
  behavior.desire_stance = behavior_.desired_stance;
  behavior.desire_lifter_pos = behavior_.desire_lifter_pos;
  behavior.scan_yaw_min_rad = static_cast<float>(behavior_.scan_yaw_min * M_PI / 180.0);
  behavior.scan_yaw_max_rad = static_cast<float>(behavior_.scan_yaw_max * M_PI / 180.0);
  behavior.ammo_purchase_request = behavior_.ammo_purchase_request;
  behavior.revive_request = behavior_.revive_request;
  behavior.remote_revive_request = behavior_.remote_revive_request;
  behavior.remote_ammo_request = behavior_.remote_ammo_request;
  behavior.remote_health_request = behavior_.remote_health_request;
  behavior.use_limited_scan = behavior_.use_limited_scan;
  behavior.not_aim_enemy = behavior_.not_aim_enemy;
  behavior.use_capacitor = behavior_.use_capacitor;
  behavior.tunnel_align_active = behavior_.tunnel_align_active;
  behavior.tunnel_align_angle_rad = static_cast<float>(behavior_.tunnel_align_angle_deg * M_PI / 180.0);
  behavior.use_gyro_mode = behavior_.use_gyro_mode;
  nav_data_topic_.Publish(nav);
  behavior_data_topic_.Publish(behavior);
}

void RMSerialDriver::publish_team(const TeamInfo & data)
{
  ros_interfaces::msg::TeamInformation msg;
  msg.header.stamp = now();
  for (int i = 0; i < 4; ++i) {
    msg.allies[i].robot_id = data.ally_status[i].robot_id;
    msg.allies[i].robot_hp = data.ally_status[i].robot_hp;
    msg.allies[i].position.position.x = data.ally_status[i].robot_pos_x;
    msg.allies[i].position.position.y = data.ally_status[i].robot_pos_y;
  }
  msg.outpost_hp = data.outpost_hp;
  msg.base_hp = data.base_hp;
  team_pub_->publish(msg);
}

void RMSerialDriver::publish_game(const GameInfo & data)
{
  ros_interfaces::msg::GameInfo msg;
  msg.header.stamp = now();
  msg.game_time_remaining = data.game_time_remaining;
  msg.coin_remaining = data.coin_remaining;
  msg.event_code = data.event_code;
  msg.game_status = data.game_status;
  msg.manual_point_x = data.manual_point_x;
  msg.manual_point_y = data.manual_point_y;
  msg.manual_key = data.manual_key;
  msg.enemy_outpost_hp = data.enemy_outpost_hp;
  msg.enemy_base_hp = data.enemy_base_hp;
  game_pub_->publish(msg);
}

void RMSerialDriver::publish_online(const SentryInfoOnline & data)
{
  ros_interfaces::msg::SentryInfoOnline msg;
  msg.header.stamp = now();
  msg.self_health = data.self_health;
  msg.bullets_remaining = data.bullets_remaining;
  msg.cooling_value = data.cooling_value;
  msg.heat_limit = data.heat_limit;
  msg.current_heat = data.current_heat;
  msg.sentry_pos.x = data.sentry_pos_x;
  msg.sentry_pos.y = data.sentry_pos_y;
  msg.speed_monitor_angle = data.speed_monitor_angle;
  msg.sentry_info_1 = data.sentry_info_1;
  msg.sentry_info_2 = data.sentry_info_2;
  msg.sentry_info_3 = data.sentry_info_3;
  msg.energy_ratio = data.energy_ratio;
  online_pub_->publish(msg);
}

void RMSerialDriver::publish_offline(const SentryInfoOffline & data, const LibXR::RawData &)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  ros_interfaces::msg::SentryInfoOffline msg;
  msg.header.stamp = now();
  msg.yaw_camerainit_to_gimbal = data.yaw_camerainit_to_gimbal;
  msg.lifter_current_pos = data.lifter_current_pos;
  msg.is_transformable = data.is_transformable;
  msg.transform_state = data.transform_state;
  msg.capacitor_capacity = data.capacitor_capacity;
  msg.tunnel_yaw_aligned = data.tunnel_yaw_aligned;
  if (std::isfinite(data.chassis_imu_yaw)) {
    imu_yaw_history_.push_back({msg.header.stamp, data.chassis_imu_yaw});
    if (imu_yaw_history_.size() > kImuHistoryCapacity)
      imu_yaw_history_.pop_front();
    update_delta_yaw_locked();
  }
  offline_pub_->publish(msg);
}

void RMSerialDriver::publish_radar(const RadarInfo & data)
{
  ros_interfaces::msg::RadarInfo msg;
  msg.header.stamp = now();
  for (int i = 0; i < 6; ++i) {
    msg.enemies[i].robot_id = data.enemy_status[i].robot_id;
    msg.enemies[i].robot_hp = data.enemy_status[i].robot_hp;
    msg.enemies[i].allowed_projectile = data.enemy_status[i].allowed_projectile;
    msg.enemies[i].position.position.x = static_cast<double>(data.enemy_status[i].robot_pos_x) / 100.0;
    msg.enemies[i].position.position.y = static_cast<double>(data.enemy_status[i].robot_pos_y) / 100.0;
  }
  msg.enemy_coin_left = data.enemy_coin_left;
  msg.enemy_coin_accumulated = data.enemy_coin_accumulated;
  msg.is_enemy_outpost_sensed = data.is_enemy_outpost_sensed;
  radar_pub_->publish(msg);
}

}  // namespace rm_serial_driver

RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
