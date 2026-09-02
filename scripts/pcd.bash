#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${WORKSPACE_DIR}/install/setup.bash"
cd "${WORKSPACE_DIR}"

gnome-terminal -- bash -c "ros2 launch livox_ros_driver2 msg_mixed_MID360.launch.py; exec bash"
# gnome-terminal -- bash -c "ros2 launch livox_ros_driver2 msg_MID360.launch.py; exec bash"
sleep 5s

gnome-terminal -- bash -c "ros2 launch point_lio point_lio.launch.py; exec bash"
