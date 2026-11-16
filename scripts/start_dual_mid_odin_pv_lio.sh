#!/usr/bin/env bash
# Mid360 + Odin1 双雷达 PV-LIO 建图一键启动
# 用法: bash /home/ll/3dnavigation/scripts/start_dual_mid_odin_pv_lio.sh
set -euo pipefail

LOG_DIR=/tmp/pv_lio_dual_run
mkdir -p "$LOG_DIR"

export DISPLAY="${DISPLAY:-:1}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"

echo "[dual-pv] stop old processes (if any)"
bash /home/ll/3dnavigation/scripts/stop_dual_mid_odin_pv_lio.sh >/dev/null 2>&1 || true
sleep 1

echo "[dual-pv] start Mid360 driver"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source /home/ll/ws_livox/install/setup.bash
nohup ros2 launch livox_ros_driver2 msg_MID360_launch.py \
  >"$LOG_DIR/livox.log" 2>&1 &
sleep 4

echo "[dual-pv] start Odin1 driver"
# shellcheck disable=SC1091
source /home/ll/ws_odin_pvlio/install/setup.bash
CFG=/home/ll/ws_odin_pvlio/install/odin_ros_driver/share/odin_ros_driver/config/control_command.yaml
nohup ros2 run odin_ros_driver host_sdk_sample --ros-args \
  -r __node:=host_sdk_sample -p config_file:="$CFG" \
  >"$LOG_DIR/odin.log" 2>&1 &
sleep 6

echo "[dual-pv] start PV-LIO dual mapping"
env -i HOME="$HOME" USER="$USER" PATH="/usr/bin:/bin:/usr/local/bin" \
  DISPLAY="$DISPLAY" RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=0 \
  XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR" \
  bash -lc 'source /opt/ros/humble/setup.bash; \
            source /home/ll/ws_livox/install/setup.bash; \
            source /home/ll/pv_lio_dual_ws/install/setup.bash; \
            exec ros2 launch pv_lio mapping_dual_mid_odin.launch.py rviz:=false' \
  >"$LOG_DIR/mapping.log" 2>&1 &
sleep 2

echo "[dual-pv] start RViz"
env -i HOME="$HOME" USER="$USER" PATH="/usr/bin:/bin" \
  DISPLAY="$DISPLAY" RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR" \
  bash -lc 'source /opt/ros/humble/setup.bash; \
            source /home/ll/pv_lio_dual_ws/install/setup.bash; \
            exec rviz2 -d /home/ll/pv_lio_dual_ws/install/pv_lio/share/pv_lio/config/rviz_cfg/voxel_mapping.rviz' \
  >"$LOG_DIR/rviz.log" 2>&1 &
sleep 3

echo "[dual-pv] processes:"
pgrep -af 'livox_ros_driver2_node|host_sdk_sample|pv_lio_node|rviz2' | head -8 || true
echo "[dual-pv] logs: $LOG_DIR/{livox,odin,mapping,rviz}.log"
echo "[dual-pv] done"
