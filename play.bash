#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO}/setup.bash}"
[[ -f "${ROS_SETUP}" ]] || { echo "ROS setup not found: ${ROS_SETUP}" >&2; exit 1; }
source "${ROS_SETUP}"
source "${WORKSPACE_DIR}/install/setup.bash"

# Navigation2
gnome-terminal --title="Navigation2 and RViz" -- bash -lc \
  "cd '${WORKSPACE_DIR}'; source '${ROS_SETUP}'; source '${WORKSPACE_DIR}/install/setup.bash'; ros2 launch navi2 navigation2.launch.py; exec bash"
sleep 0.5s
