#!/usr/bin/env bash
# Simulation with navigation and RViz (default world: rmuc_2025). Examples:
#   ./start_sim.sh
#   ./start_sim.sh world:=rmuc_2024 headless:=true rviz:=false
set -eo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO}/setup.bash}"
if [[ -z "${NAV_INSTALL_DIR:-}" ]]; then
    NAV_INSTALL_DIR="${WORKSPACE_DIR}/install-link"
    if [[ ! -f "${NAV_INSTALL_DIR}/setup.bash" && -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
        NAV_INSTALL_DIR="${WORKSPACE_DIR}/install"
    fi
fi

if [[ ! -f "${ROS_SETUP}" ]]; then
    echo "ROS setup not found: ${ROS_SETUP}" >&2
    exit 1
fi
source "${ROS_SETUP}"

if [[ ! -f "${NAV_INSTALL_DIR}/setup.bash" ]]; then
    echo "Workspace is not built: ${NAV_INSTALL_DIR}/setup.bash" >&2
    echo "Run ./build.bash first." >&2
    exit 1
fi
source "${NAV_INSTALL_DIR}/setup.bash"

cd "${WORKSPACE_DIR}"
has_world=false
for arg in "$@"; do
    if [[ "$arg" == world:=* ]]; then
        has_world=true
        break
    fi
done
if [[ "$has_world" == false ]]; then
    set -- world:=rmuc_2025 "$@"
fi
exec ros2 launch sentry_simulation simulation.launch.py "$@"
