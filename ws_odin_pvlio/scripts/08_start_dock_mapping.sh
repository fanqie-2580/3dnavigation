#!/usr/bin/env bash
# Run Odin driver + ROS1 map accumulator ON the Go2 dock (no TCP bridge).
set -eo pipefail
DOCK=unitree@192.168.123.18
PASS=123
REMOTE=/home/unitree/odin_bridge

sshpass -p "$PASS" ssh -o StrictHostKeyChecking=no "$DOCK" "mkdir -p $REMOTE/maps $REMOTE/logs"
sshpass -p "$PASS" scp -o StrictHostKeyChecking=no \
  /home/ll/ws_odin_pvlio/bridge/odin_odom_mapper_ros1.py \
  "$DOCK:$REMOTE/odin_odom_mapper_ros1.py"

# stop Jetson-side bridge mapping (not needed)
pkill -f odin_ros2_tcp_client.py 2>/dev/null || true
pkill -f odin_odom_mapper.py 2>/dev/null || true

sshpass -p "$PASS" ssh -o StrictHostKeyChecking=no "$DOCK" 'bash -s' <<'EOF'
set -eo pipefail
pkill -f odin_ros1_tcp_server.py 2>/dev/null || true
pkill -f odin_odom_mapper_ros1.py 2>/dev/null || true
pkill -f host_sdk_sample 2>/dev/null || true
pkill -f 'odin1_ros1.launch' 2>/dev/null || true
sleep 2

cat > /home/unitree/odin_bridge/start_dock_mapping.sh <<'INNER'
#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /home/unitree/ws_odin/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=192.168.123.18
LOGDIR=/home/unitree/odin_bridge/logs
MAPDIR=/home/unitree/odin_bridge/maps
mkdir -p "$LOGDIR" "$MAPDIR"
LAUNCH=/home/unitree/ws_odin/src/odin_ros_driver/launch_ROS1/odin1_ros1.launch

killall -q roscore rosmaster 2>/dev/null || true
sleep 1

# ensure SLAM mode in config
sed -i 's/custom_map_mode: 0/custom_map_mode: 1/' \
  /home/unitree/ws_odin/src/odin_ros_driver/config/control_command.yaml || true
sed -i 's/sendcloudslam: 0/sendcloudslam: 1/' \
  /home/unitree/ws_odin/src/odin_ros_driver/config/control_command.yaml || true

setsid roslaunch "$LAUNCH" > "$LOGDIR/odin_driver.log" 2>&1 < /dev/null &
echo $! > /home/unitree/odin_bridge/odin_driver.pid
sleep 8

setsid python3 /home/unitree/odin_bridge/odin_odom_mapper_ros1.py \
  _cloud_topic:=/odin1/cloud_slam \
  _cloud_in_odom_frame:=true \
  _save_map_path:=/home/unitree/odin_bridge/maps/odin1_4floor_map.pcd \
  > "$LOGDIR/mapper.log" 2>&1 < /dev/null &
echo $! > /home/unitree/odin_bridge/mapper.pid
INNER
chmod +x /home/unitree/odin_bridge/start_dock_mapping.sh
setsid bash /home/unitree/odin_bridge/start_dock_mapping.sh \
  > /home/unitree/odin_bridge/logs/start_dock_mapping.log 2>&1 < /dev/null &
sleep 12

source /opt/ros/noetic/setup.bash
source /home/unitree/ws_odin/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
echo '--- dock mapping status ---'
grep -E 'custom_map_mode|sendcloudslam' /home/unitree/ws_odin/src/odin_ros_driver/config/control_command.yaml | head -5
pgrep -a host_sdk_sample | head -1 || echo NO_SDK
pgrep -a odin_odom_mapper_ros1 | head -1 || echo NO_MAPPER
timeout 3 rostopic hz /odin1/cloud_slam 2>&1 | head -4 || true
timeout 3 rostopic hz /odin1/odometry 2>&1 | head -4 || true
timeout 3 rostopic hz /cloud_registered 2>&1 | head -4 || true
timeout 2 rostopic echo /Laser_map -n 1 2>&1 | grep -E 'width:|frame_id' | head -4 || true
tail -8 /home/unitree/odin_bridge/logs/mapper.log || true
echo DOCK_MAPPING_OK
EOF

echo
echo "Save on dock:  ssh unitree@192.168.123.18 \"source /opt/ros/noetic/setup.bash; source /home/unitree/ws_odin/devel/setup.bash; export ROS_MASTER_URI=http://127.0.0.1:11311; rosservice call /save_map\""
echo "Pull map:      bash /home/ll/ws_odin_pvlio/scripts/09_pull_dock_map.sh"
