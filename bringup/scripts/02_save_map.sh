#!/usr/bin/env bash
# Phase 1b: trigger FAST-LIO map save, then split ground/obstacle for DDDMR.
set -eo pipefail
# ROS setup.bash references unset vars; keep -u off while sourcing.
set +u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/../env.sh"
source_fastlio

echo "[bringup] calling /map_save ..."
ros2 service call /map_save std_srvs/srv/Trigger '{}'

# FAST-LIO also dumps ROOT_DIR/PCD/scans.pcd on clean shutdown; prefer configured path.
FALLBACK="${FASTLIO_WS}/FAST_LIO/PCD/scans.pcd"
SRC="${MAP_PCD}"
if [[ ! -f "${SRC}" ]]; then
  if [[ -f "${FALLBACK}" ]]; then
    echo "[bringup] ${SRC} missing, using fallback ${FALLBACK}"
    SRC="${FALLBACK}"
    cp -f "${SRC}" "${MAP_PCD}"
  else
    echo "[bringup] ERROR: map file not found at ${MAP_PCD} or ${FALLBACK}"
    echo "  Make sure mapping is running with pcd_save_en=true, then retry."
    exit 1
  fi
fi

echo "[bringup] splitting map for localization + DDDMR global planner"
python3 "${SCRIPT_DIR}/split_map.py" "${MAP_PCD}" \
  --ground-out "${MAP_GROUND_PCD}" \
  --obstacle-out "${MAP_OBSTACLE_PCD}" \
  --loc-out "${MAP_LOC_PCD}"

# Keep a copy where open3d_loc / dddmr defaults often look
mkdir -p "${FASTLIO_WS}/data" "${DDDMR_HOST}/src/dddmr_global_planner/data"
cp -f "${MAP_LOC_PCD}" "${FASTLIO_WS}/data/ct_lio_map.pcd"
cp -f "${MAP_OBSTACLE_PCD}" "${DDDMR_HOST}/src/dddmr_global_planner/data/segmentation_obstacle_map.pcd"
cp -f "${MAP_GROUND_PCD}" "${DDDMR_HOST}/src/dddmr_global_planner/data/segmentation_ground_map.pcd"

echo "[bringup] map ready:"
echo "  loc:       ${MAP_LOC_PCD}"
echo "  obstacle:  ${MAP_OBSTACLE_PCD}"
echo "  ground:    ${MAP_GROUND_PCD}"
