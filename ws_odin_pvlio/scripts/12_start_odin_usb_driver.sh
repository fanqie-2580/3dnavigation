#!/usr/bin/env bash
# Start Odin1 host_sdk on Jetson USB (no dock TCP). Same fusion stack as 657a8e0.
set -eo pipefail
WS=/home/ll/ws_odin_pvlio
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

if ! lsusb | grep -q '2207:0019'; then
  echo "[odin-usb] Jetson 上未看到 Odin USB (2207:0019)。请确认线插在本机。"
  exit 1
fi

pkill -f 'odin_ros2_tcp_client.py' 2>/dev/null || true
pkill -f 'host_sdk_sample' 2>/dev/null || true
sleep 1

# Align Odin stamps to host clock so Mid /Odometry pairing works on one board.
CFG="$WS/install/odin_ros_driver/share/odin_ros_driver/config/control_command.yaml"
if [[ ! -f "$CFG" ]]; then
  CFG="$WS/src/odin_ros_driver/config/control_command.yaml"
fi

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
nohup ros2 run odin_ros_driver host_sdk_sample --ros-args \
  -r __node:=host_sdk_sample \
  -p config_file:="$CFG" \
  > /tmp/odin_usb_driver.log 2>&1 &
echo $! > /tmp/odin_usb_driver.pid
echo "[odin-usb] host_sdk_sample pid=$(cat /tmp/odin_usb_driver.pid) cfg=$CFG"
echo "[odin-usb] waiting for /odin1/odometry ..."
for _ in $(seq 1 60); do
  if timeout 2 ros2 topic echo /odin1/odometry --once >/dev/null 2>&1; then
    echo "[odin-usb] ok"
    exit 0
  fi
  sleep 0.5
done
echo "[odin-usb] timeout — see /tmp/odin_usb_driver.log"
exit 1
