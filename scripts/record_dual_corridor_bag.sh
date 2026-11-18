#!/usr/bin/env bash
# Record Mid360 + Odin1 raw topics for offline dual-LIO verification.
# Usage:
#   bash /home/ll/3dnavigation/scripts/record_dual_corridor_bag.sh
# Stop: Ctrl-C  (or kill the ros2 bag process)
set -eo pipefail

OUT_ROOT="${BAG_OUT_ROOT:-/home/ll/pv_lio_dual_ws/bags}"
STAMP=$(date +%Y%m%d_%H%M%S)
OUT_DIR="${OUT_ROOT}/corridor_${STAMP}"
# Only create parent; ros2 bag record creates OUT_DIR itself.
mkdir -p "$OUT_ROOT"
rmdir "$OUT_DIR" 2>/dev/null || true

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

echo "[bag] out: $OUT_DIR"
echo "[bag] topics: Mid /livox/* + Odin cloud_raw/imu/odometry (no live map)"
echo "[bag] walk the corridor; tell agent to stop when done"

# Sensors + Odin onboard odom (for offline Z pose-factor). No live map topics.
exec ros2 bag record -o "$OUT_DIR" --compression-mode file --compression-format zstd \
  /livox/lidar \
  /livox/imu \
  /odin1/cloud_raw \
  /odin1/imu \
  /odin1/odometry
