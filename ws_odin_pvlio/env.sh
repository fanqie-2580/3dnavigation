#!/usr/bin/env bash
# Clean ROS env for Odin1 + PV-LIO (avoid Autoware/hesai overlays chained into other workspaces)
# Usage: source /home/ll/ws_odin_pvlio/env.sh

_WS_ODIN_PVLIO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Drop polluted overlays first
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
# Keep system PATH; clear ROS python overlays
export PYTHONPATH=""

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

# Prepend only what we need (isolated install prefixes)
_LIVOX="/home/ll/ws_livox/install/livox_ros_driver2"
_ODIN="$_WS_ODIN_PVLIO/install/odin_ros_driver"
_PV="$_WS_ODIN_PVLIO/install/pv_lio"
_BR="$_WS_ODIN_PVLIO/install/odin_pv_lio_bringup"

for p in "$_BR" "$_PV" "$_ODIN" "$_LIVOX"; do
  if [[ -d "$p" ]]; then
    export AMENT_PREFIX_PATH="${p}${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
    export CMAKE_PREFIX_PATH="${p}${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
    if [[ -d "$p/lib" ]]; then
      export LD_LIBRARY_PATH="${p}/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    fi
  fi
done

# Python path for ament_index / launch
export PYTHONPATH="${_BR}/lib/python3.10/site-packages:${_PV}/lib/python3.10/site-packages:${_ODIN}/lib/python3.10/site-packages:${PYTHONPATH:-}"

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
unset CYCLONEDDS_URI
export DISPLAY="${DISPLAY:-:1}"

echo "[env] Odin+PV-LIO ready. pkgs:"
echo "  $_BR"
echo "  $_PV"
echo "  $_ODIN"
