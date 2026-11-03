#!/usr/bin/env bash
# Simulation with navigation and RViz (default world: rmuc_2026). Examples:
#   ./start_sim.sh
#   ./start_sim.sh robot_model:=real
#   ./start_sim.sh world:=rmuc_2024 headless:=true rviz:=false
# It writes a model-specific copy of sentry1.yaml under log/sim_params. The omni
# model uses the same MPC velocity limits; the real model gets its CAD geometry
# from the launch profile. An explicit params_file:= takes precedence.
# Foxglove Bridge listens on port 8765 and stops when this script exits.
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
has_params=false
robot_model=omni
for arg in "$@"; do
    case "$arg" in
        world:=*) has_world=true ;;
        params_file:=*) has_params=true ;;
        robot_model:=*) robot_model="${arg#robot_model:=}" ;;
    esac
done
if [[ "$has_world" == false ]]; then
    set -- world:=rmuc_2026 "$@"
fi

if [[ "$has_params" == false ]]; then
    base_params="$(ros2 pkg prefix --share navi2)/params/sentry1.yaml"
    if [[ "${robot_model}" == real ]]; then
        sim_params="${WORKSPACE_DIR}/log/sim_params/sentry1_real_sim.yaml"
    else
        sim_params="${WORKSPACE_DIR}/log/sim_params/sentry1_sim.yaml"
    fi
    mkdir -p "$(dirname "${sim_params}")"
    cp "${base_params}" "${sim_params}"
    set -- "$@" "params_file:=${sim_params}"
fi

if ! ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
    echo "foxglove_bridge is not installed or not available in the sourced ROS environment." >&2
    exit 1
fi

bridge_pid=""
cleanup() {
    for pid in "${bridge_pid}"; do
        if [[ -n "${pid}" ]]; then
            kill -TERM "${pid}" 2>/dev/null || true
            wait "${pid}" 2>/dev/null || true
        fi
    done
}
trap cleanup EXIT

if ss -ltn 'sport = :8765' | grep -q LISTEN; then
    echo "Port 8765 is already in use; using the existing Foxglove connection if available."
else
    ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 &
    bridge_pid=$!
    echo "Starting Foxglove Bridge on ws://localhost:8765"
fi

ros2 launch sentry_simulation simulation.launch.py "$@"
