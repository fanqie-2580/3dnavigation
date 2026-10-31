#!/usr/bin/env bash
set -eo pipefail
WS=/home/ll/ws_odin_pvlio
CT=/home/ll/ct_lio_to_ros2-master
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH || true
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
export AMENT_PREFIX_PATH="/home/ll/ws_livox/install/livox_ros_driver2:${AMENT_PREFIX_PATH:-}"
export CMAKE_PREFIX_PATH="/home/ll/ws_livox/install/livox_ros_driver2:${CMAKE_PREFIX_PATH:-}"

# Odin driver (optional rebuild)
if [[ "${SKIP_ODIN_BUILD:-0}" != "1" ]]; then
  cd "$WS/src/odin_ros_driver"
  bash ./script/build_ros2.sh || true
  rm -f "$WS/install/COLCON_IGNORE" "$WS/build/COLCON_IGNORE" "$WS/log/COLCON_IGNORE"
fi

cd "$WS"
colcon build --symlink-install --packages-select odin_pv_lio_bringup

# CT-LIO (Odin1 handler)
cd "$CT"
rm -f install/COLCON_IGNORE build/COLCON_IGNORE
colcon build --symlink-install --packages-select ct_lio
rm -f install/COLCON_IGNORE

echo "OK: source $WS/env.sh"
echo "Run: bash $WS/scripts/02_run_mapping.sh"
