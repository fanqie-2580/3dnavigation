#!/usr/bin/env bash
# Common paths/topics for Go2 + MID360 mapping/navigation bringup.
# Source this from other scripts; override any var before sourcing if needed.

: "${BRINGUP_ROOT:=/home/ll/3dnavigation/bringup}"
: "${FASTLIO_WS:=/home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble}"
: "${EGO_WS:=/home/ll/ego-planner-swarm-ros2_version}"
: "${DDDMR_HOST:=/home/ll/dddmr_navigation}"
: "${DDDMR_DOCKER:=/root/dddmr_ws}"
: "${DDDMR_IMAGE:=dddmr:l4t_r36}"
: "${DDDMR_CONTAINER:=dddmr_dev}"

: "${MAP_DIR:=${BRINGUP_ROOT}/maps}"
: "${MAP_PCD:=${MAP_DIR}/fastlio_map.pcd}"
: "${MAP_LOC_PCD:=${MAP_DIR}/loc_map.pcd}"
: "${MAP_OBSTACLE_PCD:=${MAP_DIR}/obstacle_map.pcd}"
: "${MAP_GROUND_PCD:=${MAP_DIR}/ground_map.pcd}"

: "${FASTLIO_CONFIG_PATH:=${BRINGUP_ROOT}/config}"
: "${FASTLIO_CONFIG_FILE:=mid360_mapping.yaml}"
: "${FASTLIO_CONFIG_NAV_FILE:=mid360_mapping.yaml}"

: "${LIVOX_CUSTOM_TOPIC:=/livox/lidar}"
: "${LIVOX_PC2_TOPIC:=/livox/lidar_pointcloud2}"
: "${ODOM_TOPIC:=/baselink2map}"
: "${CLOUD_TOPIC:=/cloud_registered_1}"
: "${GLOBAL_PATH_TOPIC:=/global_path}"
: "${CMD_VEL_TOPIC:=/cmd_vel}"
: "${FRAME_ID:=map}"
: "${BASE_FRAME:=base_link}"

: "${USE_SIM_TIME:=false}"
: "${RMW_IMPLEMENTATION:=rmw_cyclonedds_cpp}"

export BRINGUP_ROOT FASTLIO_WS EGO_WS DDDMR_HOST DDDMR_DOCKER DDDMR_IMAGE DDDMR_CONTAINER
export MAP_DIR MAP_PCD MAP_LOC_PCD MAP_OBSTACLE_PCD MAP_GROUND_PCD
export FASTLIO_CONFIG_PATH FASTLIO_CONFIG_FILE FASTLIO_CONFIG_NAV_FILE
export LIVOX_CUSTOM_TOPIC LIVOX_PC2_TOPIC ODOM_TOPIC CLOUD_TOPIC GLOBAL_PATH_TOPIC CMD_VEL_TOPIC
export FRAME_ID BASE_FRAME USE_SIM_TIME RMW_IMPLEMENTATION

mkdir -p "${MAP_DIR}"

source_host_ros() {
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  export RMW_IMPLEMENTATION
}

source_fastlio() {
  source_host_ros
  # shellcheck disable=SC1091
  source "${FASTLIO_WS}/install/setup.bash"
}

source_ego() {
  source_host_ros
  # shellcheck disable=SC1091
  source "${EGO_WS}/install/setup.bash"
}

ensure_dddmr_container() {
  if docker ps --format '{{.Names}}' | grep -qx "${DDDMR_CONTAINER}"; then
    return 0
  fi
  if docker ps -a --format '{{.Names}}' | grep -qx "${DDDMR_CONTAINER}"; then
    echo "[bringup] starting existing container ${DDDMR_CONTAINER}"
    docker start "${DDDMR_CONTAINER}" >/dev/null
    return 0
  fi
  echo "[bringup] no ${DDDMR_CONTAINER} running."
  echo "  Start one first, e.g.:"
  echo "  docker run -it --name ${DDDMR_CONTAINER} --network host --runtime nvidia --privileged \\"
  echo "    -v /dev:/dev -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=\$DISPLAY \\"
  echo "    -v ${DDDMR_HOST}:/root/dddmr_ws -v /home/ll/dddmr_bags:/root/dddmr_bags \\"
  echo "    ${DDDMR_IMAGE} /bin/bash"
  return 1
}

dddmr_exec() {
  ensure_dddmr_container || return 1
  docker exec -i "${DDDMR_CONTAINER}" bash -lc "
    source /opt/ros/humble/setup.bash
    export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}
    cd ${DDDMR_DOCKER}
    [ -f install/setup.bash ] && source install/setup.bash
    $*
  "
}
