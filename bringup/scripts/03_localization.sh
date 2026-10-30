#!/usr/bin/env bash
# Phase 2a: FAST-LIO + open3d_loc (host).
# Prerequisite: Livox driver (or bag), and MAP_LOC_PCD from 02_save_map.sh.
set -eo pipefail
# ROS setup.bash references unset vars; keep -u off while sourcing.
set +u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/../env.sh"
source_fastlio

LOC_MAP="${MAP_LOC_PCD}"
if [[ ! -f "${LOC_MAP}" ]]; then
  if [[ -f "${FASTLIO_WS}/data/ct_lio_map.pcd" ]]; then
    LOC_MAP="${FASTLIO_WS}/data/ct_lio_map.pcd"
  else
    echo "[bringup] ERROR: localization map not found: ${MAP_LOC_PCD}"
    echo "  Run mapping + 02_save_map.sh first, or set MAP_LOC_PCD=/path/to/map.pcd"
    exit 1
  fi
fi

echo "[bringup] localization_3d_go2"
echo "  map_file=${LOC_MAP}"
echo "  In RViz: set 2D Pose Estimate (/initialpose) on the map."

exec ros2 launch open3d_loc localization_3d_go2.launch.py \
  use_sim_time:="${USE_SIM_TIME}" \
  map_file:="${LOC_MAP}" \
  fastlio_config_path:="${FASTLIO_CONFIG_PATH}" \
  fastlio_config_file:="${FASTLIO_CONFIG_NAV_FILE}" \
  fastlio_rviz:=false \
  localization_rviz:=true \
  publish_livox_pointcloud2:=true \
  livox_custom_topic:="${LIVOX_CUSTOM_TOPIC}" \
  livox_pointcloud2_topic:="${LIVOX_PC2_TOPIC}"
