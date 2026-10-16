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

# Real-vehicle single-Livox startup. PTP is intentionally disabled for this
# chain; set PTP_ENABLE=1 explicitly only when a synchronized-clock test needs it.
PTP_ENABLE="${PTP_ENABLE:-0}"
BT_MANAGER_ENABLE="${BT_MANAGER_ENABLE:-1}"
VISION_ENABLE="${VISION_ENABLE:-1}"
VISION_ROOT="${VISION_ROOT:-${WORKSPACE_DIR}/../PLDX_Sentry_Vision}"
VISION_STARTUP_WAIT_S="${VISION_STARTUP_WAIT_S:-30}"
# /cloud_registered_full is published only after Point-LIO IMU gravity alignment.
POINT_LIO_CLOUD_WAIT_S="${POINT_LIO_CLOUD_WAIT_S:-40}"
POINT_LIO_SETTLE_S="${POINT_LIO_SETTLE_S:-3}"

LOG_DIR="${LOG_DIR:-${WORKSPACE_DIR}/log/runtime-$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$LOG_DIR"
declare -a CHILD_PIDS=()
cleanup_children() {
  trap - EXIT INT TERM
  for pid in "${CHILD_PIDS[@]}"; do kill -- "-$pid" 2>/dev/null || kill "$pid" 2>/dev/null || true; done
}
trap cleanup_children EXIT INT TERM

run_terminal() {
  local title="$1"
  shift
  setsid bash -lc "cd '$WORKSPACE_DIR'; source '$ROS_SETUP'; source '$WORKSPACE_DIR/install/setup.bash'; exec $*" \
    >"$LOG_DIR/${title// /_}.log" 2>&1 &
  CHILD_PIDS+=("$!")
  echo "[start] $title pid=${CHILD_PIDS[-1]} log=$LOG_DIR/${title// /_}.log"
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
  echo "[start] Timed out waiting for ${topic} after ${timeout_s}s" >&2
  return 1
}

wait_for_node() {
  local node="$1"
  local timeout_s="$2"
  local deadline=$((SECONDS + timeout_s))
  echo "[start] Waiting up to ${timeout_s}s for ${node}"
  while (( SECONDS < deadline )); do
    if ros2 node list 2>/dev/null | grep -Fxq "${node}"; then
      echo "[start] Found ${node}"
      return 0
    fi
    sleep 1
  done
  echo "[start] Timed out waiting for ${node}" >&2
  return 1
}

# Start the only USB CDC owner before navigation begins publishing commands.
if [[ "${VISION_ENABLE}" == "1" ]]; then
  [[ -x "${VISION_ROOT}/build-jazzy/sentry" ]] || {
    echo "Vision executable not found: ${VISION_ROOT}/build-jazzy/sentry" >&2
    echo "Run PLDX_Sentry_Vision/tools/build_sentry.sh first, or set VISION_ENABLE=0." >&2
    exit 1
  }
  [[ -f "${VISION_ROOT}/configs/sentry.yaml" ]] || {
    echo "Vision config not found: ${VISION_ROOT}/configs/sentry.yaml" >&2
    exit 1
  }
  run_terminal "Vision Communication" \
    "cd '${VISION_ROOT}'; source tools/ros_env.sh; bash tools/run_sentry.sh configs/sentry.yaml"
  wait_for_node "/nav_subscriber" "${VISION_STARTUP_WAIT_S}"
else
  echo "[start] Vision communication disabled (VISION_ENABLE=0)"
fi

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
