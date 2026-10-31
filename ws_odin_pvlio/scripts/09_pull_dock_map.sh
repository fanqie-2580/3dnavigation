#!/usr/bin/env bash
# Save map on dock (if mapper up) and copy PCD + segment on Jetson.
set -eo pipefail
WS=/home/ll/ws_odin_pvlio
DOCK=unitree@192.168.123.18
PASS=123
REMOTE_MAP=/home/unitree/odin_bridge/maps/odin1_4floor_map.pcd
LOCAL_DIR="$WS/maps"
mkdir -p "$LOCAL_DIR"

sshpass -p "$PASS" ssh -o StrictHostKeyChecking=no "$DOCK" 'bash -s' <<'EOF'
source /opt/ros/noetic/setup.bash
source /home/unitree/ws_odin/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
rosservice call /save_map
ls -lh /home/unitree/odin_bridge/maps/odin1_4floor_map.pcd
EOF

STAMP=$(date +%Y%m%d_%H%M%S)
LOCAL="$LOCAL_DIR/odin1_4floor_map_${STAMP}.pcd"
sshpass -p "$PASS" scp -o StrictHostKeyChecking=no \
  "$DOCK:$REMOTE_MAP" "$LOCAL"
cp -f "$LOCAL" "$LOCAL_DIR/odin1_4floor_map.pcd"
ls -lh "$LOCAL" "$LOCAL_DIR/odin1_4floor_map.pcd"

bash "$WS/scripts/07_segment_map.sh" "$LOCAL_DIR/odin1_4floor_map.pcd"
echo "[pull] done"
