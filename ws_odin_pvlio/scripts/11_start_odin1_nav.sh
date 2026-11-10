#!/usr/bin/env bash
# Start Odin1 localization + DDDMR global nav (does NOT touch Mid360 launches).
set -eo pipefail
WS=/home/ll/ws_odin_pvlio
ODIN_NAV=$WS/odin1_nav
MAP=$ODIN_NAV/maps/odin1_odom_map.pcd
export DISPLAY="${DISPLAY:-:1}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
unset CYCLONEDDS_URI || true

# 1) Dock: driver + TCP bridge (cloud_slam in odom frame for open3d_loc)
sshpass -p '123' scp "$WS/bridge/odin_ros1_tcp_server.py" \
  unitree@192.168.123.18:/home/unitree/odin_bridge/odin_ros1_tcp_server.py
sshpass -p '123' ssh unitree@192.168.123.18 'bash -s' <<'EOF'
pkill -f odin_ros1_tcp_server.py 2>/dev/null || true
if ! pgrep -f host_sdk_sample >/dev/null; then
  bash /home/unitree/odin_bridge/start_dock_mapping.sh >/home/unitree/odin_bridge/logs/start_for_nav.log 2>&1 &
  sleep 10
fi
source /opt/ros/noetic/setup.bash
source /home/unitree/ws_odin/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=192.168.123.18
# cloud_slam is in odom frame — matches saved map / open3d_loc expectation
setsid python3 /home/unitree/odin_bridge/odin_ros1_tcp_server.py \
  _cloud_topic:=/odin1/cloud_slam \
  > /home/unitree/odin_bridge/logs/tcp_server_nav.log 2>&1 < /dev/null &
sleep 2
ss -ltn | grep 5601
pgrep -a host_sdk_sample | head -1
EOF

# 2) Jetson ROS2 env (do NOT source broken host dddmr install)
set +u
source /opt/ros/humble/setup.bash
source /home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble/install/setup.bash

pkill -f odin_ros2_tcp_client.py 2>/dev/null || true
pkill -f odin1_loc_topic_adapter.py 2>/dev/null || true
pkill -f 'localization_3d_odin1' 2>/dev/null || true
pkill -f global_localization_node 2>/dev/null || true
pkill -f 'dddmr_reference_tracking|ego_planner_node|traj_server' 2>/dev/null || true
sleep 1

nohup python3 "$WS/bridge/odin_ros2_tcp_client.py" \
  --ros-args \
  -p server_host:=192.168.123.18 \
  -p server_port:=5601 \
  -p cloud_topic:=/odin1/cloud_slam \
  > /tmp/odin1_nav_tcp_client.log 2>&1 &
sleep 3
nohup python3 "$ODIN_NAV/scripts/odin1_loc_topic_adapter.py" \
  > /tmp/odin1_loc_adapter.log 2>&1 &
sleep 2

SHARE_LAUNCH=/home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble/install/open3d_loc/share/open3d_loc/launch
mkdir -p "$SHARE_LAUNCH"
SRC_LAUNCH=/home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble/open3d_loc/launch/localization_3d_odin1.launch.py
if [ "$(readlink -f "$SRC_LAUNCH")" != "$(readlink -f "$SHARE_LAUNCH/localization_3d_odin1.launch.py" 2>/dev/null || true)" ]; then
  cp -f "$SRC_LAUNCH" "$SHARE_LAUNCH/"
fi

nohup ros2 launch open3d_loc localization_3d_odin1.launch.py \
  map_file:="$MAP" \
  localization_rviz:=true \
  > /tmp/odin1_localization.log 2>&1 &
sleep 8
# Same continuous Odin odom as mapping: force map≡odom (ICP corridor slide otherwise)
python3 "$ODIN_NAV/scripts/odin1_reset_odom2map_identity.py" \
  > /tmp/odin1_reset_o2m.log 2>&1 || true

# 3) DDDMR docker — Odin1 fork only (never dddmr_dev / Mid360 tree)
if ! docker ps --format '{{.Names}}' | grep -qx dddmr_odin1; then
  docker start dddmr_odin1 || docker run -d --name dddmr_odin1 \
    --network host --runtime nvidia --privileged \
    -v /dev:/dev \
    -v /etc/localtime:/etc/localtime:ro \
    -v /home/ll/dddmr_navigation_odin1:/root/dddmr_ws \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    dddmr:l4t_r36 sleep infinity
  sleep 2
fi

docker exec dddmr_odin1 bash -lc '
  export DISPLAY=:1 QT_X11_NO_MITSHM=1
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  unset CYCLONEDDS_URI
  export ROS_LOCALHOST_ONLY=0
  source /opt/ros/humble/setup.bash
  cd /root/dddmr_ws
  [ -f install/setup.bash ] && source install/setup.bash
  pkill -f dddmr_global_ego_odin1 2>/dev/null || true
  pkill -f "global_planner_node|mcl_feature|pcl_publisher" 2>/dev/null || true
  sleep 1
  nohup ros2 launch p2p_move_base dddmr_global_ego_odin1.launch.py \
    odom_topic:=/baselink2map \
    lidar_topic:=/cloud_registered_body_1 \
    use_rviz:=true \
    publish_sensor_tf:=true \
    sensor_frame:=body \
    >/tmp/dddmr_odin1_nav.log 2>&1 &
  sleep 5
  echo DOCKER_NAV_STARTED
'

# 4) EGO local planner — Odin1 fork only
set +u
source /home/ll/ego-planner-swarm-odin1/install/setup.bash
nohup ros2 launch ego_planner dddmr_reference_tracking_odin1.launch.py \
  use_rviz:=true \
  odom_topic:=/baselink2map \
  cloud_topic:=/cloud_registered_1 \
  reference_path_topic:=/ego_repaired_global_path \
  > /tmp/ego_odin1_local.log 2>&1 &
sleep 6

echo
echo "Odin1 nav started (loc + dddmr_odin1 + ego-odin1)."
echo "  Trees: /home/ll/dddmr_navigation_odin1 , /home/ll/ego-planner-swarm-odin1"
echo "  Loc RViz: give 2D Pose Estimate if reloc looks off"
echo "  DDDMR goal: Publish Point -> /clicked_point"
echo "Logs: /tmp/odin1_localization.log /tmp/ego_odin1_local.log /tmp/dddmr_odin1_nav.log"