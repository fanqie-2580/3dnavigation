#!/usr/bin/env bash
set -euo pipefail
# shellcheck disable=SC1091
source /home/ll/ws_odin_pvlio/env.sh
mkdir -p /home/ll/ws_odin_pvlio/maps

if ! lsusb | grep -q '2207:0019'; then
  echo "[warn] Jetson 上未看到 Odin USB (2207:0019)。请把 Odin 数据线插到本机 USB3，再重试。"
  echo "       若仍插在拓展坞，本机 ROS2 CT-LIO 收不到数据。"
fi

exec ros2 launch odin_pv_lio_bringup odin1_ct_lio_mapping.launch.py "$@"
