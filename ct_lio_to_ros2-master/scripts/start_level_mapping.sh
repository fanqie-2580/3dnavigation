#!/usr/bin/env bash
# Start CT-LIO (leveled) + linefit for Go2 Mid-360 remapping.
# Stops localization first so /cloud_registered is owned by CT-LIO.
set -eo pipefail

ROOT=/home/ll/ct_lio_to_ros2-master
LOG_DIR=/tmp
export DISPLAY="${DISPLAY:-:1}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
unset CYCLONEDDS_URI || true
# ROS setup.bash references optional unset vars; disable nounset while sourcing.
set +u

echo "[pipeline] Stopping localization / old CT-LIO / linefit..."
# Avoid matching this script path (contains "ct_lio").
pkill -f 'global_localization_node' || true
pkill -f 'open3d_loc_go2.launch' || true
pkill -f '/fast_lio/fastlio_mapping' || true
pkill -f 'fastlio_mapping --ros-args' || true
pkill -f 'ct_lio_eskf' || true
pkill -f 'run_eskf.launch' || true
pkill -f 'ground_segmentation_node' || true
sleep 2

if ! pgrep -f 'livox_ros_driver2_node' >/dev/null; then
  echo "[pipeline] Starting Livox MID360 driver..."
  source /opt/ros/humble/setup.bash
  source /home/ll/ws_livox/install/setup.bash
  nohup ros2 launch livox_ros_driver2 msg_MID360_launch.py \
    > "${LOG_DIR}/livox_mid360_mapping.log" 2>&1 &
  sleep 3
else
  echo "[pipeline] Livox driver already running."
fi

source /opt/ros/humble/setup.bash
source /home/ll/ws_livox/install/setup.bash
source "${ROOT}/install/setup.bash"

# Always pin source-tree mapping.yaml so we never pick up a stale install copy
# with extrinsic_R=I (that was the main cause of tilted remaps).
PARAMS="${ROOT}/config/mapping.yaml"
echo "[pipeline] CT-LIO params: ${PARAMS}"
grep -n 'extrinsic_R\|level_map_enable\|base_to_lidar_translation' "${PARAMS}" | head -20

# Stop dddmr TF publishers if container is up (host-network FastDDS clash).
if docker ps --format '{{.Names}}' 2>/dev/null | grep -qx dddmr_dev; then
  echo "[pipeline] Pausing dddmr nodes inside dddmr_dev (TF clash)..."
  docker exec dddmr_dev bash -lc \
    'pkill -f "ros2 launch|global_planner|mcl_feature|pcl_publisher|rviz2|static_transform" || true' \
    >/dev/null 2>&1 || true
fi

nohup ros2 launch ct_lio run_eskf.launch.py \
  params_file:="${PARAMS}" \
  rviz_use:=true \
  > "${LOG_DIR}/ct_lio_mapping.log" 2>&1 &

echo "[pipeline] Waiting for map->mid360_lidar TF before linefit..."
python3 - <<'PY'
import rclpy, time, sys
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
rclpy.init()
n = Node('wait_tf_linefit')
buf = Buffer()
TransformListener(buf, n)
ok = False
t0 = time.time()
while time.time() - t0 < 30:
    rclpy.spin_once(n, timeout_sec=0.1)
    try:
        buf.lookup_transform('map', 'mid360_lidar', rclpy.time.Time())
        ok = True
        break
    except Exception:
        pass
rclpy.shutdown()
sys.exit(0 if ok else 1)
PY
if [[ $? -ne 0 ]]; then
  echo "[pipeline] WARN: map->mid360_lidar TF not ready; linefit may drop early frames."
else
  echo "[pipeline] TF ready."
fi

source /home/ll/linefit_ground_segmentation_ros2/install/setup.bash
mkdir -p "${ROOT}/maps/segmentation"
# Pin source params so save_directory / gravity frames stay in sync.
LINEFIT_PARAMS=/home/ll/linefit_ground_segmentation_ros2/linefit_ground_segmentation_ros/launch/segmentation_params.yaml
pkill -f ground_segmentation_node || true
sleep 1
nohup ros2 run linefit_ground_segmentation_ros ground_segmentation_node \
  --ros-args --params-file "${LINEFIT_PARAMS}" \
  > "${LOG_DIR}/linefit_seg.log" 2>&1 &
sleep 2

echo "[pipeline] Running:"
pgrep -af 'ct_lio_eskf|ground_segmentation_node|livox_ros_driver2_node' | grep -v pgrep || true
echo
echo "Walk the area. Keep the robot still for a few seconds at start (IMU init)."
echo "After leveling locks, RViz /cloud_registered should look flat vs grid."
echo
echo "When done, run:  ${ROOT}/scripts/save_and_deploy_leveled_maps.sh"
