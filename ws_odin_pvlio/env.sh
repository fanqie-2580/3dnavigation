#!/usr/bin/env bash
# Clean ROS env for Odin1 + CT-LIO (avoid Autoware overlays)
# Usage: source /home/ll/ws_odin_pvlio/env.sh

_WS_ODIN_PVLIO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
export PYTHONPATH=""

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

_LIVOX="/home/ll/ws_livox/install/livox_ros_driver2"
_ODIN="$_WS_ODIN_PVLIO/install/odin_ros_driver"
_BR="$_WS_ODIN_PVLIO/install/odin_pv_lio_bringup"
_CT="/home/ll/ct_lio_to_ros2-master/install/ct_lio"
# keep PV install on path in case old launches are used
_PV="$_WS_ODIN_PVLIO/install/pv_lio"

for p in "$_BR" "$_CT" "$_ODIN" "$_LIVOX" "$_PV"; do
  if [[ -d "$p" ]]; then
    export AMENT_PREFIX_PATH="${p}${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
    export CMAKE_PREFIX_PATH="${p}${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
    if [[ -d "$p/lib" ]]; then
      export LD_LIBRARY_PATH="${p}/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    fi
  fi
done

export PYTHONPATH="${_BR}/lib/python3.10/site-packages:${_CT}/lib/python3.10/site-packages:${_ODIN}/lib/python3.10/site-packages:${PYTHONPATH:-}"

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
unset CYCLONEDDS_URI
export DISPLAY="${DISPLAY:-:1}"

echo "[env] Odin+CT-LIO ready:"
echo "  $_BR"
echo "  $_CT"
echo "  $_ODIN"
