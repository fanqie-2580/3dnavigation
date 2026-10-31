#!/usr/bin/env bash
# Odin on dock (TCP bridge) + Odin odometry mapper on Jetson
# (does NOT use PV-LIO / CT-LIO odometry)
set -eo pipefail
WS=/home/ll/ws_odin_pvlio
set +u
# shellcheck disable=SC1091
source "$WS/env.sh"
set -u
mkdir -p "$WS/maps"

MODE="${ODIN_SOURCE:-bridge}"  # bridge | usb
RVIZ="${RVIZ:-true}"
MAPPER="${MAPPER:-odin_odom}"  # odin_odom | ct_lio | pv_lio
DOCK_HOST="${DOCK_HOST:-192.168.123.18}"
DOCK_PORT="${DOCK_PORT:-5601}"

if [[ "$MODE" == "usb" ]]; then
  if ! lsusb | grep -q '2207:0019'; then
    echo "[warn] Jetson 上未看到 Odin USB。改用 bridge 模式，或把数据线插到本机。"
    exit 1
  fi
fi

# stop previous mappers
pkill -f odin_ros2_tcp_client.py 2>/dev/null || true
pkill -f odin_odom_mapper.py 2>/dev/null || true
pkill -f ct_lio_eskf 2>/dev/null || true
pkill -f pv_lio_node 2>/dev/null || true
pkill -f odin_cloud_to_pvlio 2>/dev/null || true
sleep 1

if [[ "$MODE" == "bridge" ]]; then
  echo "[map] starting dock ROS1 driver + TCP bridge..."
  bash "$WS/scripts/04_start_dock_odin_bridge.sh"
  echo "[map] starting Jetson TCP client..."
  nohup python3 "$WS/bridge/odin_ros2_tcp_client.py" \
    --ros-args \
    -p server_host:="$DOCK_HOST" \
    -p server_port:="$DOCK_PORT" \
    > /tmp/odin_ros2_tcp_client.log 2>&1 &
  echo $! > /tmp/odin_ros2_tcp_client.pid
  for i in $(seq 1 30); do
    if timeout 1 ros2 topic hz /odin1/odometry 2>/dev/null | grep -q 'average rate'; then
      break
    fi
    sleep 0.5
  done
else
  echo "[map] starting local Odin ROS2 driver..."
  ros2 launch odin_ros_driver odin1_ros2.launch.py &
  echo $! > /tmp/odin_ros2_driver.pid
  sleep 3
fi

case "$MAPPER" in
  odin_odom)
    echo "[map] using Odin1 odometry mapper (not PV/CT-LIO)"
    nohup python3 "$WS/bridge/odin_odom_mapper.py" \
      > /tmp/odin_odom_mapper.log 2>&1 &
    echo $! > /tmp/odin_odom_mapper.pid
    if [[ "$RVIZ" == "true" ]]; then
      CFG=/home/ll/ct_lio_to_ros2-master/rviz_cfg/odin1_ct_lio.rviz
      nohup rviz2 -d "$CFG" > /tmp/rviz_odin_odom.log 2>&1 &
      echo $! > /tmp/rviz_odin_odom.pid
    fi
    echo "[map] topics: /cloud_registered /Laser_map /Odometry /path"
    echo "[map] save:   ros2 service call /save_map std_srvs/srv/Trigger"
    echo "[map]        -> $WS/maps/odin1_odom_map.pcd"
    # keep script attached so Ctrl-C can stop
    wait "$(cat /tmp/odin_odom_mapper.pid)"
    ;;
  ct_lio)
    echo "[map] launching CT-LIO..."
    ros2 launch ct_lio run_eskf_odin1.launch.py rviz_use:="$RVIZ" "$@"
    ;;
  pv_lio)
    echo "[map] launching PV-LIO..."
    ros2 launch odin_pv_lio_bringup odin1_pv_lio_mapping.launch.py rviz:="$RVIZ" "$@"
    ;;
  *)
    echo "unknown MAPPER=$MAPPER (odin_odom|ct_lio|pv_lio)"
    exit 1
    ;;
esac
