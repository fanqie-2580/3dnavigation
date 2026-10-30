#!/usr/bin/env bash
# Phase 2b: DDDMR global planner only (inside dddmr Docker).
# Consumes /baselink2map + /livox/lidar_pointcloud2 from host localization.
set -eo pipefail
# ROS setup.bash references unset vars; keep -u off while sourcing.
set +u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/../env.sh"

ensure_dddmr_container || exit 1

# Paths inside the container (host DDDMR_HOST is mounted at DDDMR_DOCKER)
MAP_OBSTACLE_DOCKER="${DDDMR_DOCKER}/src/dddmr_global_planner/data/segmentation_obstacle_map.pcd"
MAP_GROUND_DOCKER="${DDDMR_DOCKER}/src/dddmr_global_planner/data/segmentation_ground_map.pcd"

echo "[bringup] DDDMR global planner in ${DDDMR_CONTAINER}"
echo "  odom=${ODOM_TOPIC} lidar=${LIVOX_PC2_TOPIC}"
echo "  obstacle=${MAP_OBSTACLE_DOCKER}"
echo "  ground=${MAP_GROUND_DOCKER}"
echo "  In RViz: Publish Point / 3D Goal -> /clicked_point -> /global_path"

docker exec -it "${DDDMR_CONTAINER}" bash -lc "
  source /opt/ros/humble/setup.bash
  export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}
  cd ${DDDMR_DOCKER}
  source install/setup.bash
  ros2 launch p2p_move_base dddmr_global_ego_docker.launch.py \
    use_sim_time:=${USE_SIM_TIME} \
    odom_topic:=${ODOM_TOPIC} \
    lidar_topic:=${LIVOX_PC2_TOPIC} \
    map_dir:=${MAP_OBSTACLE_DOCKER} \
    ground_dir:=${MAP_GROUND_DOCKER} \
    publish_internal_registered_cloud:=false \
    use_rviz:=true \
    enable_goal_pose_relay:=true
"
