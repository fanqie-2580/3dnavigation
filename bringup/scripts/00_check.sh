#!/usr/bin/env bash
# Quick environment / topic sanity check.
set -eo pipefail
# ROS setup.bash references unset vars; keep -u off while sourcing.
set +u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/../env.sh"
source_host_ros

echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>}"
echo "MAP_LOC_PCD=${MAP_LOC_PCD} exists=$([[ -f ${MAP_LOC_PCD} ]] && echo yes || echo no)"
echo "MAP_OBSTACLE_PCD exists=$([[ -f ${MAP_OBSTACLE_PCD} ]] && echo yes || echo no)"
echo "MAP_GROUND_PCD exists=$([[ -f ${MAP_GROUND_PCD} ]] && echo yes || echo no)"
echo "FASTLIO install: $([[ -f ${FASTLIO_WS}/install/setup.bash ]] && echo ok || echo MISSING)"
echo "EGO install: $([[ -f ${EGO_WS}/install/setup.bash ]] && echo ok || echo MISSING)"
echo "DDDMR container: $(docker ps --format '{{.Names}}' | grep -qx ${DDDMR_CONTAINER} && echo running || echo not-running)"

echo
echo "--- ros2 topic list (sample) ---"
ros2 topic list 2>/dev/null | grep -E 'livox|Odometry|cloud_registered|baselink2map|global_path|cmd_vel|initialpose' || true
