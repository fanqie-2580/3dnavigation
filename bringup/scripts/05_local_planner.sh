#!/usr/bin/env bash
# Phase 2c: EGO local planner + cmd_vel tracking (host).
# Follows /global_path using /baselink2map and /cloud_registered_1.
set -eo pipefail
# ROS setup.bash references unset vars; keep -u off while sourcing.
set +u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/../env.sh"
source_ego

echo "[bringup] EGO ct_lio_fastlio_reference_tracking"
echo "  odom=${ODOM_TOPIC} cloud=${CLOUD_TOPIC}"
echo "  reference=${GLOBAL_PATH_TOPIC} cmd=${CMD_VEL_TOPIC}"

exec ros2 launch ego_planner ct_lio_fastlio_reference_tracking.launch.py \
  odom_topic:="${ODOM_TOPIC}" \
  cloud_topic:="${CLOUD_TOPIC}" \
  reference_path_topic:="${GLOBAL_PATH_TOPIC}" \
  frame_id:="${FRAME_ID}" \
  base_frame:="${BASE_FRAME}" \
  cmd_topic:="${CMD_VEL_TOPIC}" \
  use_rviz:=true
