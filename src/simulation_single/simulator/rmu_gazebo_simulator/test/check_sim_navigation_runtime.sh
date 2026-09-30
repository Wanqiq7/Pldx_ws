#!/usr/bin/env bash
set -euo pipefail

launch_pid=""
active_log=""
disabled_params="$(mktemp /tmp/rmu_sim_nav2_no_rog_viz.XXXXXX.yaml)"

stop_launch() {
    if [[ -z "${launch_pid}" ]]; then
        return
    fi

    if pgrep -s "${launch_pid}" >/dev/null; then
        pkill -INT -s "${launch_pid}" || true
        for _ in {1..15}; do
            pgrep -s "${launch_pid}" >/dev/null || break
            sleep 1
        done
    fi
    if pgrep -s "${launch_pid}" >/dev/null; then
        pkill -TERM -s "${launch_pid}" || true
        for _ in {1..5}; do
            pgrep -s "${launch_pid}" >/dev/null || break
            sleep 1
        done
    fi
    if pgrep -s "${launch_pid}" >/dev/null; then
        pkill -KILL -s "${launch_pid}" || true
    fi
    wait "${launch_pid}" || true
    launch_pid=""
}

cleanup() {
    stop_launch
    rm -f "${disabled_params}"
}
trap cleanup EXIT

simulator_share="$(ros2 pkg prefix --share rmu_gazebo_simulator)"
default_params="${simulator_share}/config/simulation_nav2_params.yaml"
python3 - "${default_params}" "${disabled_params}" <<'PY'
from pathlib import Path
import sys

import yaml

source = Path(sys.argv[1])
target = Path(sys.argv[2])
parameters = yaml.safe_load(source.read_text())
parameters["planner_server"]["ros__parameters"]["GridBased"]["rog_map"][
    "visualization"
]["enable"] = False
target.write_text(yaml.safe_dump(parameters, sort_keys=False))
PY

run_trial() {
    local trial="$1"
    local params_file="$2"
    local visualization_state="$3"
    active_log="$(mktemp "/tmp/rmu_sim_navigation_trial_${trial}.XXXXXX.log")"

    setsid bash -c 'trap - INT TERM; exec "$@"' bash \
        ros2 launch rmu_gazebo_simulator simulation.launch.py use_rviz:=false headless:=true \
        params_file:="${params_file}" >"${active_log}" 2>&1 &
    launch_pid="$!"

    local deadline=$((SECONDS + 45))
    while (( SECONDS < deadline )); do
        if rg -q 'lifecycle_manager_navigation.*Managed nodes are active' "${active_log}"; then
            break
        fi
        sleep 1
    done

    if ! rg -q 'lifecycle_manager_navigation.*Managed nodes are active' "${active_log}"; then
        tail -n 120 "${active_log}"
        echo "Trial ${trial}: Nav2 lifecycle activation failed; full log: ${active_log}" >&2
        return 1
    fi

    ros2 run sim_nav_adapter check_sim_tf.py --ros-args -p timeout_sec:=15.0
    ros2 run sim_nav_adapter check_sim_costmap.py --ros-args -p timeout_sec:=20.0

    local navigation_result
    if ! navigation_result="$(timeout 120s ros2 action send_goal \
        /navigate_to_pose nav2_msgs/action/NavigateToPose \
        '{pose: {header: {frame_id: map}, pose: {position: {x: 8.41, y: 14.2}, orientation: {w: 1.0}}}}' \
        2>&1)"; then
        printf '%s\n' "${navigation_result}"
        echo "Trial ${trial}: navigation did not finish within 120 s; full log: ${active_log}" >&2
        return 1
    fi
    if ! rg -q 'Goal finished with status: SUCCEEDED' <<<"${navigation_result}"; then
        printf '%s\n' "${navigation_result}"
        echo "Trial ${trial}: NavigateToPose failed; full log: ${active_log}" >&2
        return 1
    fi

    ros2 run sim_nav_adapter check_sim_tf.py --ros-args -p timeout_sec:=15.0
    ros2 run sim_nav_adapter check_sim_costmap.py --ros-args -p timeout_sec:=20.0

    local failure_pattern
    failure_pattern='BT_RegisterNodesFromPlugin|libnav2_constrained_smoother.so|open set exhausted|Failed to create plan|Failed to find path|Dropping oldest cloud while waiting|Dropping cloud because'
    if rg -ni "${failure_pattern}" "${active_log}"; then
        echo "Trial ${trial}: navigation regression detected; full log: ${active_log}" >&2
        return 1
    fi

    local timestamp_drop_pattern
    timestamp_drop_pattern='Message Filter dropping message.*earlier than all the data in the transform cache'
    local timestamp_drop_count
    timestamp_drop_count="$(rg "${timestamp_drop_pattern}" "${active_log}" \
        | sed -E 's/.* at time ([0-9.]+) for reason.*/\1/' \
        | sort -u | wc -l || true)"
    timestamp_drop_count="${timestamp_drop_count:-0}"
    if (( timestamp_drop_count > 2 )); then
        rg -n "${timestamp_drop_pattern}" "${active_log}"
        echo "Trial ${trial}: persistent timestamp drops; full log: ${active_log}" >&2
        return 1
    fi

    echo "PASS trial ${trial}: ROG visualization ${visualization_state}, navigation succeeded"
    stop_launch
    rm -f "${active_log}"
    active_log=""
    sleep 2
}

run_trial 1 "${default_params}" enabled
run_trial 2 "${disabled_params}" disabled
run_trial 3 "${default_params}" enabled

echo "PASS: three cold-start navigation trials met TF, truth, costmap, and timing limits"
