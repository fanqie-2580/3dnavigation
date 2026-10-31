#!/usr/bin/env bash
# Odin on dock (TCP bridge) + CT-LIO on Jetson
set -eo pipefail
WS=/home/ll/ws_odin_pvlio
set +u
# shellcheck disable=SC1091
source "$WS/env.sh"
set -u
mkdir -p "$WS/maps"

MODE="${ODIN_SOURCE:-bridge}"  # bridge | usb
RVIZ="${RVIZ:-true}"

if [[ "$MODE" == "usb" ]]; then
  if ! lsusb | grep -q '2207:0019'; then
    echo "[warn] Jetson 上未看到 Odin USB。改用 bridge 模式，或把数据线插到本机。"
    exit 1
  fi
  exec ros2 launch odin_pv_lio_bringup odin1_ct_lio_mapping.launch.py \
    start_odin_driver:=true rviz:="$RVIZ" "$@"
fi

echo "[map] starting dock ROS1 driver + TCP bridge..."
bash "$WS/scripts/04_start_dock_odin_bridge.sh"

echo "[map] starting Jetson TCP client + CT-LIO..."
pkill -f odin_ros2_tcp_client.py 2>/dev/null || true
pkill -f ct_lio_eskf 2>/dev/null || true
sleep 1

python3 "$WS/bridge/odin_ros2_tcp_client.py" \
  --ros-args \
  -p server_host:=192.168.123.18 \
  -p server_port:=5601 &
echo $! > /tmp/odin_ros2_tcp_client.pid

# wait for bridged topics
for i in $(seq 1 20); do
  if timeout 1 ros2 topic hz /odin1/imu 2>/dev/null | grep -q 'average rate'; then
    break
  fi
  sleep 0.5
done

ros2 launch ct_lio run_eskf_odin1.launch.py rviz_use:="$RVIZ" "$@"
