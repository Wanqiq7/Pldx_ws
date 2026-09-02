#ifndef ROS2_LIBXR__ROS2LIBXR_HPP_
#define ROS2_LIBXR__ROS2LIBXR_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <type_traits>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ros_interfaces/msg/behavior.hpp>
#include <ros_interfaces/msg/game_info.hpp>
#include <ros_interfaces/msg/radar_info.hpp>
#include <ros_interfaces/msg/sentry_info_offline.hpp>
#include <ros_interfaces/msg/sentry_info_online.hpp>
#include <ros_interfaces/msg/team_information.hpp>

#include "SharedTopic/SharedTopic.hpp"
#include "SharedTopicClient/SharedTopicClient.hpp"
#include "app_framework.hpp"
#include "linux_uart.hpp"
#include "message.hpp"
#include "thread.hpp"
#include "uart.hpp"

namespace rm_serial_driver {

// Wire payloads frozen by communication_protocol_contract.md.
#pragma pack(push, 1)
struct ChassisTarget
{
  float vx_mps{}, vy_mps{}, vw_rad_s{}, current_yaw{}, current_vx{}, current_vy{}, current_vw{},
    fx_global{}, fy_global{}, fw_global{};
  bool use_speed_control{};
  float delta_yaw{};
};
struct BehaviorData
{
  uint8_t pitch_mode{}, desire_stance{}, desire_lifter_pos{};
  float scan_yaw_min_rad{}, scan_yaw_max_rad{};
  uint16_t ammo_purchase_request{};
  uint8_t revive_request{}, remote_revive_request{}, remote_ammo_request{}, remote_health_request{};
  bool use_limited_scan{}, not_aim_enemy{}, use_capacitor{}, tunnel_align_active{};
  float tunnel_align_angle_rad{};
  bool use_gyro_mode{};
};
struct AllyRobotStatus
{
  uint8_t robot_id{};
  uint16_t robot_hp{};
  float robot_pos_x{}, robot_pos_y{};
};
struct TeamInfo
{
  AllyRobotStatus ally_status[4]{};
  uint16_t outpost_hp{}, base_hp{};
};
struct GameInfo
{
  uint16_t game_time_remaining{}, coin_remaining{};
  uint32_t event_code{};
  uint8_t game_status{};
  float manual_point_x{}, manual_point_y{};
  uint8_t manual_key{};
  uint16_t enemy_outpost_hp{}, enemy_base_hp{};
};
struct SentryInfoOnline
{
  uint16_t self_health{}, bullets_remaining{}, cooling_value{}, heat_limit{}, current_heat{};
  float sentry_pos_x{}, sentry_pos_y{}, speed_monitor_angle{};
  uint32_t sentry_info_1{};
  uint16_t sentry_info_2{};
  uint64_t sentry_info_3{};
  uint8_t energy_ratio{};
};
struct SentryInfoOffline
{
  float yaw_camerainit_to_gimbal{};
  uint8_t lifter_current_pos{};
  bool is_transformable{};
  float transform_state{};
  uint8_t capacitor_capacity{};
  float chassis_imu_yaw{};
  bool tunnel_yaw_aligned{};
};
struct EnemyRobotStatus
{
  uint8_t robot_id{};
  uint16_t robot_hp{}, allowed_projectile{}, robot_pos_x{}, robot_pos_y{};
};
struct RadarInfo
{
  EnemyRobotStatus enemy_status[6]{};
  uint16_t enemy_coin_left{}, enemy_coin_accumulated{};
  bool is_enemy_outpost_sensed{};
};
#pragma pack(pop)

static_assert(sizeof(ChassisTarget) == 45);
static_assert(sizeof(BehaviorData) == 26);
static_assert(offsetof(BehaviorData, use_gyro_mode) == 25);
static_assert(sizeof(TeamInfo) == 48);
static_assert(sizeof(GameInfo) == 22);
static_assert(sizeof(SentryInfoOnline) == 37);
static_assert(sizeof(SentryInfoOffline) == 16);
static_assert(sizeof(RadarInfo) == 59);
static_assert(sizeof(bool) == 1);
static_assert(offsetof(SentryInfoOffline, yaw_camerainit_to_gimbal) == 0);
static_assert(offsetof(SentryInfoOffline, lifter_current_pos) == 4);
static_assert(offsetof(SentryInfoOffline, is_transformable) == 5);
static_assert(offsetof(SentryInfoOffline, transform_state) == 6);
static_assert(offsetof(SentryInfoOffline, capacitor_capacity) == 10);
static_assert(offsetof(SentryInfoOffline, chassis_imu_yaw) == 11);
static_assert(offsetof(SentryInfoOffline, tunnel_yaw_aligned) == 15);
static_assert(std::is_trivially_copyable_v<ChassisTarget>);
static_assert(std::is_trivially_copyable_v<BehaviorData>);
static_assert(std::is_trivially_copyable_v<TeamInfo>);
static_assert(std::is_trivially_copyable_v<GameInfo>);
static_assert(std::is_trivially_copyable_v<SentryInfoOnline>);
static_assert(std::is_trivially_copyable_v<SentryInfoOffline>);
static_assert(std::is_trivially_copyable_v<RadarInfo>);

static void XRobotMain(LibXR::HardwareContainer & hw);

class RMSerialDriver : public rclcpp::Node
{
public:
  explicit RMSerialDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~RMSerialDriver() override = default;

private:
  void publish_chassis();
  void update_delta_yaw_locked();
  void publish_team(const TeamInfo & data);
  void publish_game(const GameInfo & data);
  void publish_online(const SentryInfoOnline & data);
  void publish_offline(const SentryInfoOffline & data, const LibXR::RawData & raw);
  void publish_radar(const RadarInfo & data);

  std::unique_ptr<LibXR::RamFS> ramfs_;
  std::unique_ptr<LibXR::LinuxUART> uart_client_;
  std::unique_ptr<LibXR::Terminal<1024, 64, 16, 128>> terminal_;
  std::unique_ptr<LibXR::Thread> term_thread_;
  LibXR::Topic nav_data_topic_, behavior_data_topic_, team_info_topic_, game_info_topic_,
    online_info_topic_, offline_info_topic_, radar_info_topic_;

  geometry_msgs::msg::Twist cmd_vel_{};
  geometry_msgs::msg::WrenchStamped cmd_force_{};
  nav_msgs::msg::Odometry odom_{};
  ros_interfaces::msg::Behavior behavior_{};
  std::deque<std::pair<rclcpp::Time, float>> imu_yaw_history_;
  float delta_yaw_{0.0F};
  bool delta_yaw_initialized_{false};
  int64_t imu_yaw_window_ms_{40};
  std::mutex state_mutex_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr cmd_force_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<ros_interfaces::msg::Behavior>::SharedPtr behavior_sub_;
  rclcpp::Publisher<ros_interfaces::msg::TeamInformation>::SharedPtr team_pub_;
  rclcpp::Publisher<ros_interfaces::msg::GameInfo>::SharedPtr game_pub_;
  rclcpp::Publisher<ros_interfaces::msg::SentryInfoOnline>::SharedPtr online_pub_;
  rclcpp::Publisher<ros_interfaces::msg::SentryInfoOffline>::SharedPtr offline_pub_;
  rclcpp::Publisher<ros_interfaces::msg::RadarInfo>::SharedPtr radar_pub_;
  rclcpp::TimerBase::SharedPtr tx_timer_;
};

}  // namespace rm_serial_driver
#endif  // ROS2_LIBXR__ROS2LIBXR_HPP_
