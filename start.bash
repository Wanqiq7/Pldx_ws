#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"

PLDX_VISION_WS="${PLDX_VISION_WS:-}"
if [[ -n "${PLDX_VISION_WS}" ]]; then
  PLDX_VISION_SETUP="${PLDX_VISION_WS%/}/install/setup.bash"
  if [[ ! -f "${PLDX_VISION_SETUP}" ]]; then
    echo "PLDX_VISION_WS does not contain install/setup.bash: ${PLDX_VISION_WS}" >&2
    exit 1
  fi
  source "${PLDX_VISION_SETUP}"
fi
if ! ros2 pkg prefix pldx_vision_interfaces >/dev/null 2>&1; then
  echo "pldx_vision_interfaces is unavailable. Set PLDX_VISION_WS to a built vision workspace." >&2
  exit 1
fi
if [[ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  echo "Navigation workspace is not built: ${WORKSPACE_DIR}/install/setup.bash" >&2
  echo "Run PLDX_VISION_WS=/path/to/vision_ws ./build.bash first." >&2
  exit 1
fi
source "${WORKSPACE_DIR}/install/setup.bash"
cd "${WORKSPACE_DIR}"
set -u

# ros2_libxr is the default host-side comm bridge. Set ROS2_LIBXR_ENABLE=0 to skip it.
ROS2_LIBXR_WS="${ROS2_LIBXR_WS:-}"
ROS2_LIBXR_ENABLE="${ROS2_LIBXR_ENABLE:-1}"
if [[ -n "${ROS2_LIBXR_WS}" ]]; then
  ROS2_LIBXR_SETUP="${ROS2_LIBXR_WS%/}/install/setup.bash"
  if [[ ! -f "${ROS2_LIBXR_SETUP}" ]]; then
    echo "ROS2_LIBXR_WS does not contain install/setup.bash: ${ROS2_LIBXR_WS}" >&2
    exit 1
  fi
  source "${ROS2_LIBXR_SETUP}"
fi
if [[ "${ROS2_LIBXR_ENABLE}" == "1" ]]; then
  if ! ros2 pkg prefix ros2_libxr >/dev/null 2>&1; then
    echo "ros2_libxr is not available. Initialize the submodule and run ./build.bash first." >&2
    echo "Run: git submodule update --init --recursive" >&2
    exit 1
  fi

  gnome-terminal -- bash -c "ros2 run ros2_libxr ros2_libxr_node; exec bash"
  sleep 3
else
  echo "[start] ROS2_LIBXR disabled (ROS2_LIBXR_ENABLE=${ROS2_LIBXR_ENABLE})"
fi

# PTP Sync
gnome-terminal -- bash -c "sudo ./scripts/ptp_sync.bash; exec bash"
sleep 8

# Driver + SLAM
# gnome-terminal -- bash -c "ros2 launch point_lio single_livox_pointlio_intra_process.launch.py; exec bash"
gnome-terminal -- bash -c "ros2 launch point_lio mixed_livox_pointlio_intra_process.launch.py; exec bash"
sleep 5

# ICP
gnome-terminal -- bash -c "ros2 launch icp_relocalization gicp_relocalization.launch.py; exec bash"
sleep 1

# Navigation2
gnome-terminal -- bash -c "ros2 launch navi2 navigation2.launch.py; exec bash"
# gnome-terminal -- bash -c "ros2 launch navi2 slam.launch.py; exec bash"
sleep 5

# Decision
gnome-terminal -- bash -c "ros2 launch bt_manager bt_manager.launch.py; exec bash"
sleep 3

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
