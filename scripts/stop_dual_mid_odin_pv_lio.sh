#!/usr/bin/env bash
# Mid360 + Odin1 双雷达 PV-LIO 建图一键关闭
# 用法: bash /home/ll/3dnavigation/scripts/stop_dual_mid_odin_pv_lio.sh
set -u

echo "[dual-pv] stopping mapping / RViz / drivers ..."
pkill -f 'pv_lio_node|mapping_dual_mid_odin' 2>/dev/null || true
pkill -f 'rviz2' 2>/dev/null || true
pkill -f 'host_sdk_sample|odin_ros_driver' 2>/dev/null || true
pkill -f 'livox_ros_driver2|msg_MID360_launch' 2>/dev/null || true
sleep 1

# hard kill leftovers
pkill -9 -f 'pv_lio_node' 2>/dev/null || true
pkill -9 -f 'host_sdk_sample' 2>/dev/null || true
pkill -9 -f 'livox_ros_driver2_node' 2>/dev/null || true
# only kill rviz that was started for this stack if still around
pkill -9 -f 'rviz2 -d /home/ll/pv_lio_dual_ws' 2>/dev/null || true
sleep 1

left=$(pgrep -af 'pv_lio_node|host_sdk_sample|livox_ros_driver2_node|mapping_dual_mid_odin' 2>/dev/null || true)
if [[ -z "${left}" ]]; then
  echo "[dual-pv] all cleared"
else
  echo "[dual-pv] still running:"
  echo "$left"
  exit 1
fi
