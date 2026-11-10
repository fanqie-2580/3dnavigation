#!/usr/bin/env bash
# Mid360-only nav stack. Never touches Odin1 trees or dddmr_odin1.
set -eo pipefail
export DISPLAY="${DISPLAY:-:1}"
export QT_X11_NO_MITSHM=1
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
unset CYCLONEDDS_URI FASTRTPS_DEFAULT_PROFILES_FILE || true
xhost +local:root >/dev/null 2>&1 || true

MAP="${MAP:-/home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble/data/ct_lio_map.pcd}"

set +u
source /opt/ros/humble/setup.bash
source /home/ll/ws_livox/install/setup.bash
source /home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble/install/setup.bash

pkill -f 'msg_MID360_launch|livox_ros_driver2_node' 2>/dev/null || true
pkill -f 'localization_3d_go2|global_localization_node|fastlio_mapping' 2>/dev/null || true
pkill -f 'dddmr_reference_tracking.launch.py|ego_planner_node|traj_server' 2>/dev/null || true
sleep 1

# 1) Livox
nohup ros2 launch livox_ros_driver2 msg_MID360_launch.py \
  > /tmp/livox_mid360.log 2>&1 &
sleep 4

# 2) Localization (Mid360)
nohup ros2 launch open3d_loc localization_3d_go2.launch.py \
  map_file:="$MAP" \
  body_to_base_qy:=-0.1132032138 \
  body_to_base_qw:=0.9935718557 \
  localization_rviz:=true \
  publish_livox_pointcloud2:=true \
  > /tmp/localization_3d_go2.log 2>&1 &
sleep 8

# 3) DDDMR Mid360 docker only
if ! docker ps --format '{{.Names}}' | grep -qx dddmr_dev; then
  docker start dddmr_dev || docker run -d --name dddmr_dev \
    --network host --runtime nvidia --privileged \
    -v /dev:/dev \
    -v /etc/localtime:/etc/localtime:ro \
    -v /home/ll/dddmr_navigation:/root/dddmr_ws \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY="$DISPLAY" \
    dddmr:l4t_r36 sleep infinity
  sleep 2
fi

# Kill by PID only — pkill -f matches this bash -lc cmdline and self-kills.
docker exec dddmr_dev bash -lc '
  export DISPLAY=:1 QT_X11_NO_MITSHM=1
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  unset CYCLONEDDS_URI
  export ROS_LOCALHOST_ONLY=0
  source /opt/ros/humble/setup.bash
  cd /root/dddmr_ws
  [ -f install/setup.bash ] && source install/setup.bash
  self=$$
  for pid in $(pgrep -f "dddmr_global_ego_docker.launch.py|global_planner_node|mcl_feature|pcl_publisher|p2p_move_base_localization.rviz" || true); do
    if [ "$pid" != "$self" ]; then kill "$pid" 2>/dev/null || true; fi
  done
  sleep 1
  nohup ros2 launch p2p_move_base dddmr_global_ego_docker.launch.py \
    odom_topic:=/baselink2map \
    lidar_topic:=/livox/lidar_pointcloud2 \
    use_rviz:=false \
    publish_sensor_tf:=true \
    publish_base_footprint_tf:=true \
    >/tmp/dddmr_global_ego.log 2>&1 &
  sleep 8
  nohup rviz2 -d /root/dddmr_ws/src/dddmr_p2p_move_base/rviz/p2p_move_base_localization.rviz \
    >/tmp/dddmr_nav_rviz.log 2>&1 &
  echo DOCKER_MID_NAV_STARTED
'

# 4) EGO Mid360 only — must not inherit Odin1 (same .so names, ABI mismatch → SIGSEGV)
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH || true
export LD_LIBRARY_PATH=""
set +u
source /opt/ros/humble/setup.bash
source /home/ll/ego-planner-swarm-ros2_version/install/setup.bash
# Guard: refuse to start if Odin paths leaked back in
case ":${LD_LIBRARY_PATH}:" in
  *":/home/ll/ego-planner-swarm-odin1/"*) echo "ERROR: Odin1 libs in LD_LIBRARY_PATH; abort Mid EGO"; exit 1 ;;
esac
nohup ros2 launch ego_planner dddmr_reference_tracking.launch.py \
  use_rviz:=true \
  odom_topic:=/baselink2map \
  cloud_topic:=/cloud_registered_1 \
  reference_path_topic:=/ego_repaired_global_path \
  > /tmp/ego_local_planner.log 2>&1 &
sleep 5

# 5) Motion bridge (shared hardware)
source /home/ll/unitree_ros2/setup.sh
source /home/ll/ros_ws/install/setup.bash
nohup ros2 run go2_cmd_vel_handler go2_bridge_node > /tmp/go2_bridge.log 2>&1 &

echo
echo "Mid360 nav started."
echo "  Trees: /home/ll/dddmr_navigation , /home/ll/ego-planner-swarm-ros2_version"
echo "  Docker: dddmr_dev"
echo "  Loc: 2D Pose Estimate; DDDMR: Publish Point"
echo "Logs: /tmp/livox_mid360.log /tmp/localization_3d_go2.log /tmp/dddmr_global_ego.log /tmp/ego_local_planner.log"
