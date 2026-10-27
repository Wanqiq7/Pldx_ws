#!/usr/bin/env bash
# Real-vehicle navigation bringup without bt_manager.
# Use this to verify RViz Nav2 Goal -> Nav2 -> MINCO -> /chassis_cmd_mpc -> MCU.
# A default DEFEND behavior heartbeat is published so the chassis accepts speed.
# Simulation: ./start_nav.sh --sim robot_model:=real world:=rmuc_2026
set -eo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == "--sim" ]]; then
    shift
    exec "${WORKSPACE_DIR}/start_sim.sh" "$@"
fi
export BT_MANAGER_ENABLE=0
exec "${WORKSPACE_DIR}/start.bash" "$@"
