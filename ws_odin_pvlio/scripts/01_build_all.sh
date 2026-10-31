#!/usr/bin/env bash
set -euo pipefail
WS=/home/ll/ws_odin_pvlio
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH || true
source /opt/ros/humble/setup.bash
export AMENT_PREFIX_PATH="/home/ll/ws_livox/install/livox_ros_driver2:${AMENT_PREFIX_PATH}"
export CMAKE_PREFIX_PATH="/home/ll/ws_livox/install/livox_ros_driver2:${CMAKE_PREFIX_PATH:-}"
cd "$WS/src/odin_ros_driver"
bash ./script/build_ros2.sh || true
rm -f "$WS/install/COLCON_IGNORE" "$WS/build/COLCON_IGNORE" "$WS/log/COLCON_IGNORE"
cd "$WS"
colcon build --symlink-install --packages-select pv_lio odin_pv_lio_bringup
echo "OK: source $WS/env.sh"
