#pragma once

#include <cstddef>
#include <cstdint>

// These definitions are the frozen application payloads from
// communication_protocol_contract.md. They stay local to the legacy package so
// the rollback implementation remains buildable after the standalone protocol
// package was removed.
enum PacketTypeEnum : uint8_t
{
  ENUM_PACKET_NAV_DATA = 0,
  ENUM_PACKET_ALLY_STATUS,
  ENUM_PACKET_GAMESTATUS_DATA,
  ENUM_PACKET_SENTRY_SERVER_DATA,
  ENUM_PACKET_SENTRY_SELF_DATA,
  ENUM_PACKET_RADAR,
  ENUM_PACKET_GLOBAL_PATH_X,
  ENUM_PACKET_GLOBAL_PATH_Y,
  ENUM_PACKET_BEHAVIOR_DATA
};

enum ArmEnum : uint8_t
{
  ENUM_ARM_SLAVE_COMPUTER = 0,
  ENUM_ARM_SENTRY = 7
};

#pragma pack(push, 1)

struct NavRes
{
  float x{};
  float y{};
  float yaw{};
  bool is_reach{};
};

struct GlobalPath
{
  uint16_t start_x{};
  uint16_t start_y{};
  int8_t delta_x[49]{};
  int8_t delta_y[49]{};
};

struct GlobalPathX
{
  uint16_t start_x{};
  int8_t delta_x[49]{};
};

struct GlobalPathY
{
  uint16_t start_y{};
  int8_t delta_y[49]{};
};

struct ChassisTarget
{
  float vx_mps{};
  float vy_mps{};
  float vw_rad_s{};
  float current_yaw{};
  float current_vx{};
  float current_vy{};
  float current_vw{};
  float fx_global{};
  float fy_global{};
  float fw_global{};
  bool use_speed_control{};
  float delta_yaw{};

  ChassisTarget(float vx,
    float vy,
    float vw,
    float yaw,
    float cvx,
    float cvy,
    float cvw,
    float fx,
    float fy,
    float fw,
    bool speed_control,
    float delta)
  : vx_mps(vx), vy_mps(vy), vw_rad_s(vw), current_yaw(yaw), current_vx(cvx), current_vy(cvy),
    current_vw(cvw), fx_global(fx), fy_global(fy), fw_global(fw), use_speed_control(speed_control),
    delta_yaw(delta)
  {
  }
};

struct BehaviorData
{
  uint8_t pitch_mode{};
  uint8_t desire_stance{};
  uint8_t desire_lifter_pos{};
  float scan_yaw_min_rad{};
  float scan_yaw_max_rad{};
  uint16_t ammo_purchase_request{};
  uint8_t revive_request{};
  uint8_t remote_revive_request{};
  uint8_t remote_ammo_request{};
  uint8_t remote_health_request{};
  bool use_limited_scan{};
  bool not_aim_enemy{};
  bool use_capacitor{};
  bool tunnel_align_active{};
  float tunnel_align_angle_rad{};
  bool use_gyro_mode{};

  BehaviorData(uint8_t pitch,
    uint8_t stance,
    uint8_t lifter,
    float min_yaw,
    float max_yaw,
    uint16_t ammo,
    uint8_t revive,
    uint8_t remote_revive,
    uint8_t remote_ammo,
    uint8_t remote_health,
    bool limited_scan,
    bool not_aim,
    bool capacitor,
    bool align,
    float align_angle,
    bool gyro_mode)
  : pitch_mode(pitch), desire_stance(stance), desire_lifter_pos(lifter), scan_yaw_min_rad(min_yaw),
    scan_yaw_max_rad(max_yaw), ammo_purchase_request(ammo), revive_request(revive),
    remote_revive_request(remote_revive), remote_ammo_request(remote_ammo),
    remote_health_request(remote_health), use_limited_scan(limited_scan), not_aim_enemy(not_aim),
    use_capacitor(capacitor), tunnel_align_active(align), tunnel_align_angle_rad(align_angle),
    use_gyro_mode(gyro_mode)
  {
  }
};

struct AllyRobotStatus
{
  uint8_t robot_id{};
  uint16_t robot_hp{};
  float robot_pos_x{};
  float robot_pos_y{};
};

struct EnemyRobotStatus
{
  uint8_t robot_id{};
  uint16_t robot_hp{};
  uint16_t allowed_projectile{};
  uint16_t robot_pos_x{};
  uint16_t robot_pos_y{};
};

struct TeamInfo
{
  AllyRobotStatus ally_status[4]{};
  uint16_t outpost_hp{};
  uint16_t base_hp{};

  TeamInfo(const AllyRobotStatus allies[4], uint16_t outpost, uint16_t base)
  : outpost_hp(outpost), base_hp(base)
  {
    for (int i = 0; i < 4; ++i)
      ally_status[i] = allies[i];
  }
};

struct GameInfo
{
  uint16_t game_time_remaining{};
  uint16_t coin_remaining{};
  uint32_t event_code{};
  uint8_t game_status{};
  float manual_point_x{};
  float manual_point_y{};
  uint8_t manual_key{};
  uint16_t enemy_outpost_hp{};
  uint16_t enemy_base_hp{};

  GameInfo(uint16_t time, uint16_t coins, uint32_t event, uint8_t status)
  : game_time_remaining(time), coin_remaining(coins), event_code(event), game_status(status)
  {
  }
};

struct SentryInfoOnline
{
  uint16_t self_health{};
  uint16_t bullets_remaining{};
  uint16_t cooling_value{};
  uint16_t heat_limit{};
  uint16_t current_heat{};
  float sentry_pos_x{};
  float sentry_pos_y{};
  float speed_monitor_angle{};
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
  uint8_t capacitor_capacity{};  // 0..100 percent; 255 means unavailable
  float chassis_imu_yaw{};
  bool tunnel_yaw_aligned{};
};

struct RadarInfo
{
  EnemyRobotStatus enemy_status[6]{};
  uint16_t enemy_coin_left{};
  uint16_t enemy_coin_accumulated{};
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
