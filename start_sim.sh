#!/usr/bin/env bash
# Simulation with navigation and RViz (default world: rmuc_2026). Examples:
#   ./start_sim.sh
#   ./start_sim.sh robot_model:=real
#   ./start_sim.sh world:=rmuc_2024 headless:=true rviz:=false
#   SIM_WHEEL_ODOM=1 SIM_SLOPE_COMP=1 ./start_sim.sh
# It writes a copy of sentry1.yaml to log/sim_params/sentry1_sim.yaml with the mass of
# the simulated chassis, so the shared real-robot configuration stays unchanged. An
# explicit params_file:= argument takes precedence and is passed through unchanged.
# SIM_WHEEL_ODOM=1 also runs scripts/sim_wheel_odometry.py (noisy ground-truth body
# velocity on /sentry/chassis_velocity) and enables dynamic.wheel_odometry in the copy.
# SIM_SLOPE_COMP=1 enables dynamic.slope_gravity and dynamic.disturbance_observer.
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
for arg in "$@"; do
    case "$arg" in
        world:=*) has_world=true ;;
        params_file:=*) has_params=true ;;
    esac
done
if [[ "$has_world" == false ]]; then
    set -- world:=rmuc_2026 "$@"
fi

if [[ "$has_params" == false ]]; then
    base_params="$(ros2 pkg prefix --share navi2)/params/sentry1.yaml"
    sim_params="${WORKSPACE_DIR}/log/sim_params/sentry1_sim.yaml"
    mkdir -p "$(dirname "${sim_params}")"
    # Physical parameters of the simulated omni chassis (sentry_omni model mass;
    # max_force sets the 2.4 m/s^2 limit of this simulation model).
    sed -E -e "s/^(        mass:) [0-9.]+\$/\1 38.15/" \
        -e "s/^(        max_force:) [0-9.]+\$/\1 91.56/" "${base_params}" > "${sim_params}"
    if ! grep -qE "^        mass: 38.15\$" "${sim_params}"; then
        echo "FollowPath.dynamic.mass not found in ${base_params}" >&2
        exit 1
    fi
    if [[ "${SIM_WHEEL_ODOM:-0}" == 1 ]]; then
        sed -i -E "s/^(        wheel_odometry:) (true|false)\$/\1 true/" "${sim_params}"
        if ! grep -qE "^        wheel_odometry: true\$" "${sim_params}"; then
            echo "FollowPath.dynamic.wheel_odometry not found in ${base_params}" >&2
            exit 1
        fi
    fi
    if [[ "${SIM_SLOPE_COMP:-0}" == 1 ]]; then
        for key in slope_gravity disturbance_observer; do
            sed -i -E "s/^(        ${key}:) (true|false)\$/\1 true/" "${sim_params}"
            if ! grep -qE "^        ${key}: true\$" "${sim_params}"; then
                echo "FollowPath.dynamic.${key} not found in ${base_params}" >&2
                exit 1
            fi
        done
    fi
    echo "Force-control MPC: wheel odometry: ${SIM_WHEEL_ODOM:-0}," \
        "slope compensation: ${SIM_SLOPE_COMP:-0}, drive wrench output" \
        "(params: ${sim_params})"
    set -- "$@" "params_file:=${sim_params}"
fi

if [[ "${SIM_WHEEL_ODOM:-0}" != 1 ]]; then
    exec ros2 launch sentry_simulation simulation.launch.py "$@"
fi
# Emulated wheel odometry from ground truth; stopped together with the launch.
python3 "${WORKSPACE_DIR}/scripts/sim_wheel_odometry.py" --ros-args -p use_sim_time:=true &
wheel_pid=$!
trap 'kill "${wheel_pid}" 2>/dev/null || true' EXIT
ros2 launch sentry_simulation simulation.launch.py "$@"
