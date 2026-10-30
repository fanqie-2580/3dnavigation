#!/usr/bin/env bash
# Phase 1: FAST-LIO online mapping (host).
# Prerequisite: Livox driver publishing /livox/lidar and /livox/imu (or bag with --clock + USE_SIM_TIME=true).
set -eo pipefail
# ROS setup.bash references unset vars; keep -u off while sourcing.
set +u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/../env.sh"
source_fastlio

echo "[bringup] FAST-LIO mapping"
echo "  config: ${FASTLIO_CONFIG_PATH}/${FASTLIO_CONFIG_FILE}"
echo "  map will save to: ${MAP_PCD} (via /map_save)"
echo "  lidar: ${LIVOX_CUSTOM_TOPIC}  -> also publishes ${LIVOX_PC2_TOPIC}"

exec ros2 launch fast_lio mapping.launch.py \
  use_sim_time:="${USE_SIM_TIME}" \
  config_path:="${FASTLIO_CONFIG_PATH}" \
  config_file:="${FASTLIO_CONFIG_FILE}" \
  rviz:=false \
  publish_livox_pointcloud2:=true \
  livox_custom_topic:="${LIVOX_CUSTOM_TOPIC}" \
  livox_pointcloud2_topic:="${LIVOX_PC2_TOPIC}"
