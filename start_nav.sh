#!/usr/bin/env bash
# Real-vehicle navigation bringup without bt_manager.
# Use this to verify RViz Nav2 Goal -> Nav2 -> MINCO -> /cmd_vel_mpc -> MCU.
# A default DEFEND behavior heartbeat is published so the chassis accepts speed.
set -eo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export BT_MANAGER_ENABLE=0
exec "${WORKSPACE_DIR}/start.bash" "$@"
