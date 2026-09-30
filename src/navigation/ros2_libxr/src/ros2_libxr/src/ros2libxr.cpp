#include "ros2_libxr/ros2libxr.hpp"

#include <cmath>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "libxr_mem.hpp"
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
constexpr const char * kUsbVid = "16d0";
constexpr const char * kUsbPid = "1492";
constexpr const char * kUsbControlInterface = "XRUSB CDC Control";
constexpr unsigned int kUartBaudrate = 921600;
constexpr uint32_t kSharedTopicRxBufferSize = 256;
constexpr uint32_t kSharedTopicTxSlots = 32;

template <typename T> bool copy_payload(const LibXR::Topic::RawMessageView & message, T & value)
{
  if (message.payload.addr_ == nullptr || message.payload.size_ != sizeof(T))
    return false;
  LibXR::Memory::FastCopy(&value, message.payload.addr_, sizeof(value));
  return true;
}
}  // namespace

void XRobotMain(LibXR::HardwareContainer & hw)
{
  static LibXR::ApplicationManager appmgr;
  static SharedTopic shared_topic_rx(hw,
    appmgr,
    "uart_client",
    kSharedTopicRxBufferSize,
    {{kTeamTopic}, {kGameTopic}, {kOnlineTopic}, {kOfflineTopic}, {kRadarTopic}});
  static SharedTopicClient shared_topic_tx(
    hw, appmgr, "uart_client", kSharedTopicTxSlots, {{kNavTopic}, {kBehaviorTopic}});
  (void)shared_topic_rx;
  (void)shared_topic_tx;
}

RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options) : Node("rm_serial_driver", options)
{
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  LibXR::PlatformInit();
  ramfs_ = std::make_unique<LibXR::RamFS>();
  uart_client_ = std::make_unique<LibXR::LinuxUART>(
    // C-board CDC ACM control-interface string measured from the connected
    // device (VID 16d0, PID 1492, product MainCtrl).
    kUsbVid,
    kUsbPid,
    kUsbControlInterface,
    kUartBaudrate,
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
      cmd_vel_.update(*msg);
    });
  cmd_force_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
    "/cmd_force_mpc", rclcpp::QoS(1), [this](geometry_msgs::msg::WrenchStamped::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      cmd_force_.update(*msg);
    });
  behavior_sub_ = create_subscription<ros_interfaces::msg::Behavior>(
    "/sentry/behaivor_send", rclcpp::QoS(1), [this](ros_interfaces::msg::Behavior::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      behavior_.update(*msg);
    });
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/aft_mapped_to_init",
    rclcpp::QoS(1).best_effort().durability_volatile(),
    [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      odom_ = *msg;
    });

  auto register_callback = [this](LibXR::Topic & topic, auto fn) {
    auto cb = LibXR::Topic::Callback::Create(fn, this);
    topic.RegisterCallback(cb);
  };
  register_callback(
    team_info_topic_, [](bool, RMSerialDriver * self, const LibXR::Topic::RawMessageView & message) {
      TeamInfo payload{};
      if (copy_payload(message, payload))
        self->publish_team(payload);
    });
  register_callback(
    game_info_topic_, [](bool, RMSerialDriver * self, const LibXR::Topic::RawMessageView & message) {
      GameInfo payload{};
      if (copy_payload(message, payload))
        self->publish_game(payload);
    });
  register_callback(
    online_info_topic_, [](bool, RMSerialDriver * self, const LibXR::Topic::RawMessageView & message) {
      SentryInfoOnline payload{};
      if (copy_payload(message, payload))
        self->publish_online(payload);
    });
  register_callback(
    offline_info_topic_, [](bool, RMSerialDriver * self, const LibXR::Topic::RawMessageView & message) {
      SentryInfoOffline payload{};
      if (copy_payload(message, payload))
        self->publish_offline(payload);
    });
  register_callback(
    radar_info_topic_, [](bool, RMSerialDriver * self, const LibXR::Topic::RawMessageView & message) {
      RadarInfo payload{};
      if (copy_payload(message, payload))
        self->publish_radar(payload);
    });

  tx_timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() {
    publish_chassis();
  });
  RCLCPP_INFO(get_logger(), "ROS2_LIBXR contract topics initialized");
}

void RMSerialDriver::publish_chassis()
{
  double chassis_yaw = 0.0;
  bool have_chassis_yaw = false;
  try {
    const auto tf = tf_buffer_->lookupTransform("camera_init", "base_link", tf2::TimePointZero);
    chassis_yaw = tf2::getYaw(tf.transform.rotation);
    have_chassis_yaw = std::isfinite(chassis_yaw);
  } catch (const tf2::TransformException &) {
    have_chassis_yaw = false;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  const auto now = std::chrono::steady_clock::now();
  const auto cmd_vel = cmd_vel_.value(now);
  const auto cmd_force = cmd_force_.value(now);
  const auto behavior_command = behavior_.value(now);
  ChassisTarget nav{};
  nav.vx_mps = static_cast<float>(cmd_vel.linear.x);
  nav.vy_mps = static_cast<float>(cmd_vel.linear.y);
  nav.vw_rad_s = static_cast<float>(cmd_vel.angular.z);
  nav.current_vx = static_cast<float>(odom_.twist.twist.linear.x);
  nav.current_vy = static_cast<float>(odom_.twist.twist.linear.y);
  nav.current_vw = static_cast<float>(odom_.twist.twist.angular.z);
  nav.fx_global = static_cast<float>(cmd_force.wrench.force.x);
  nav.fy_global = static_cast<float>(cmd_force.wrench.force.y);
  nav.fw_global = static_cast<float>(cmd_force.wrench.torque.z);
  const auto & orientation = odom_.pose.pose.orientation;
  tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  nav.current_yaw = have_chassis_yaw ? static_cast<float>(chassis_yaw) : static_cast<float>(yaw);
  nav.use_speed_control = true;
  nav.delta_yaw = have_chassis_yaw ? static_cast<float>(chassis_yaw) : 0.0F;
  if (behavior_command.tunnel_escape_active) {
    nav.vx_mps = behavior_command.tunnel_escape_vx;
    nav.vy_mps = behavior_command.tunnel_escape_vy;
    nav.vw_rad_s = behavior_command.tunnel_escape_wz;
  }
  if (behavior_command.tunnel_prepare_active && !behavior_command.tunnel_ready) {
    nav.vx_mps *= 0.15F;
    nav.vy_mps *= 0.15F;
  }
  if (!have_chassis_yaw) {
    nav.vx_mps = nav.vy_mps = nav.fx_global = nav.fy_global = 0.0F;
  } else {
    const float theta = static_cast<float>(chassis_yaw);
    const float cos_theta = std::cos(theta);
    const float sin_theta = std::sin(theta);
    const float vx_world = nav.vx_mps;
    const float vy_world = nav.vy_mps;
    nav.vx_mps = vx_world * sin_theta - vy_world * cos_theta;
    nav.vy_mps = vx_world * cos_theta + vy_world * sin_theta;
  }
  BehaviorData behavior{};
  behavior.pitch_mode = behavior_command.pitch_mode;
  behavior.desire_stance = behavior_command.desired_stance;
  behavior.desire_lifter_pos = behavior_command.desire_lifter_pos;
  behavior.scan_yaw_min_rad = static_cast<float>(behavior_command.scan_yaw_min * M_PI / 180.0);
  behavior.scan_yaw_max_rad = static_cast<float>(behavior_command.scan_yaw_max * M_PI / 180.0);
  behavior.ammo_purchase_request = behavior_command.ammo_purchase_request;
  behavior.revive_request = behavior_command.revive_request;
  behavior.remote_revive_request = behavior_command.remote_revive_request;
  behavior.remote_ammo_request = behavior_command.remote_ammo_request;
  behavior.remote_health_request = behavior_command.remote_health_request;
  behavior.use_limited_scan = behavior_command.use_limited_scan;
  behavior.not_aim_enemy = behavior_command.not_aim_enemy;
  behavior.use_capacitor = behavior_command.use_capacitor;
  behavior.tunnel_align_active = behavior_command.tunnel_align_active;
  behavior.tunnel_align_angle_rad =
    static_cast<float>(behavior_command.tunnel_align_angle_deg * M_PI / 180.0);
  behavior.use_gyro_mode = behavior_command.use_gyro_mode;
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

void RMSerialDriver::publish_offline(const SentryInfoOffline & data)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  ros_interfaces::msg::SentryInfoOffline msg;
  msg.header.stamp = now();
  msg.yaw_camerainit_to_gimbal = data.yaw_camerainit_to_gimbal;
  msg.lifter_current_pos = data.lifter_current_pos;
  msg.is_transformable = data.is_transformable;
  msg.transform_state = data.transform_state;
  msg.capacitor_capacity = data.capacitor_capacity;
  msg.chassis_imu_yaw = data.chassis_imu_yaw;
  msg.tunnel_yaw_aligned = data.tunnel_yaw_aligned;
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
