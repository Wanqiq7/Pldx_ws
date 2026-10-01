#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO}/setup.bash}"
[[ -f "${ROS_SETUP}" ]] || { echo "ROS setup not found: ${ROS_SETUP}" >&2; exit 1; }
source "${ROS_SETUP}"

if [[ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  echo "Navigation workspace is not built: ${WORKSPACE_DIR}/install/setup.bash" >&2
  echo "Run ./build.bash first." >&2
  exit 1
fi
source "${WORKSPACE_DIR}/install/setup.bash"
cd "${WORKSPACE_DIR}"
set -u

# Real-vehicle single-Livox startup. Optional stages can be disabled with 0.
PTP_ENABLE="${PTP_ENABLE:-1}"
BT_MANAGER_ENABLE="${BT_MANAGER_ENABLE:-1}"
# /cloud_registered_full is published only after Point-LIO IMU gravity alignment.
POINT_LIO_CLOUD_WAIT_S="${POINT_LIO_CLOUD_WAIT_S:-40}"
POINT_LIO_SETTLE_S="${POINT_LIO_SETTLE_S:-3}"

if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "gnome-terminal is required to start the chain in separate terminals." >&2
  exit 1
fi

run_terminal() {
  local title="$1"
  shift
  gnome-terminal --title="${title}" -- bash -lc \
    "cd '${WORKSPACE_DIR}'; source '${ROS_SETUP}'; source '${WORKSPACE_DIR}/install/setup.bash'; $*; exec bash"
}

wait_for_topic_message() {
  local topic="$1"
  local timeout_s="$2"
  local field="${3:-header}"
  echo "[start] Waiting up to ${timeout_s}s for ${topic} (Point-LIO publishes this after IMU init)"
  if ros2 topic echo \
    --once \
    --timeout "${timeout_s}" \
    --qos-profile sensor_data \
    --field "${field}" \
    "${topic}" >/dev/null 2>&1; then
    echo "[start] Received ${topic}"
    return 0
  fi
  echo "[start] Timed out waiting for ${topic} after ${timeout_s}s; starting GICP anyway" >&2
  return 0
}

# PTP Sync
if [[ "${PTP_ENABLE}" == "1" ]]; then
  run_terminal "PTP Sync" "sudo ./scripts/ptp_sync.bash"
  sleep 8
fi

# Single Livox MID-360 + Point-LIO
run_terminal "Single Livox Point-LIO" \
  "ros2 launch point_lio single_livox_pointlio_intra_process.launch.py"
echo "[start] Keep the vehicle stationary until GICP finishes power-on alignment"
wait_for_topic_message "/cloud_registered_full" "${POINT_LIO_CLOUD_WAIT_S}"
echo "[start] Settling ${POINT_LIO_SETTLE_S}s so the first GICP window is after gravity alignment"
sleep "${POINT_LIO_SETTLE_S}"

# ICP
run_terminal "GICP Relocalization" \
  "ros2 launch icp_relocalization gicp_relocalization.launch.py"
sleep 1

# Navigation2
run_terminal "Navigation2 and RViz" \
  "ros2 launch navi2 navigation2.launch.py"
sleep 5

# Decision. The vision process owns the USB transport; navigation publishes
# behavior and chassis topics only.
if [[ "${BT_MANAGER_ENABLE}" == "1" ]]; then
  run_terminal "BT Manager" "ros2 launch bt_manager bt_manager.launch.py"
  sleep 3
else
  echo "[start] BT Manager disabled; publishing default DEFEND behavior for chassis enable"
  run_terminal "Default Behavior" \
    "ros2 topic pub -r 10 /sentry/behaivor_send ros_interfaces/msg/Behavior '{desired_stance: 2, desire_lifter_pos: 0, pitch_mode: 0, scan_yaw_min: -180.0, scan_yaw_max: 180.0}'"
fi

# Rosbag record
ROSBAG_TOPICS=(
  /tf
  /tf_static
  /aft_mapped_to_init
  /global_costmap/costmap_raw
  /opt_path
  /opt_path_vis
  /astar_path_vis
  /recover_goal
  /mpc_predict_path
  /cmd_vel_mpc
  /sentry/behaivor_send
  /sentry/area_markers
  /cloud_registered_full
  /rog_map/occupied
  /rog_map/layer_value
  /rog_map/field
  /minco_control_points_vis
  /rog_map/layer_height_delta
  # /livox/lidar
  # /livox/imu_192_168_1_122
  # /livox/imu_192_168_1_135
  # /livox/lidar_192_168_1_122
  # /livox/lidar_192_168_1_135

)
# gnome-terminal -- bash -c "mkdir -p ~/rosbag && ros2 bag record -o ~/rosbag/$(date +%Y%m%d_%H%M%S) ${ROSBAG_TOPICS[*]}; exec bash"

# Foxglove
# gnome-terminal -- bash -c "ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765; exec bash"
