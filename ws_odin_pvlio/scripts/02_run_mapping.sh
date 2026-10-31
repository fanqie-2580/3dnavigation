#!/usr/bin/env bash
set -euo pipefail
# shellcheck disable=SC1091
source /home/ll/ws_odin_pvlio/env.sh
mkdir -p /home/ll/ws_odin_pvlio/maps
exec ros2 launch odin_pv_lio_bringup odin1_pv_lio_mapping.launch.py "$@"
