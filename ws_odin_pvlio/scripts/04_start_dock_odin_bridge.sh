#!/usr/bin/env bash
# Start Odin ROS1 driver + TCP bridge on Go2 dock (survives SSH disconnect)
set -eo pipefail
DOCK=unitree@192.168.123.18
PASS=123
REMOTE_DIR=/home/unitree/odin_bridge

sshpass -p "$PASS" ssh -o StrictHostKeyChecking=no "$DOCK" "mkdir -p $REMOTE_DIR/logs"
sshpass -p "$PASS" scp -o StrictHostKeyChecking=no \
  /home/ll/ws_odin_pvlio/bridge/odin_ros1_tcp_server.py \
  "$DOCK:$REMOTE_DIR/odin_ros1_tcp_server.py"

sshpass -p "$PASS" ssh -o StrictHostKeyChecking=no "$DOCK" 'bash -s' <<'EOF'
set -eo pipefail
pkill -f odin_ros1_tcp_server.py 2>/dev/null || true
pkill -f host_sdk_sample 2>/dev/null || true
pkill -f 'roslaunch .*odin1_ros1' 2>/dev/null || true
pkill -f odin1_ros1.launch 2>/dev/null || true
sleep 1

cat > /home/unitree/odin_bridge/start_all.sh <<'INNER'
#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /home/unitree/ws_odin/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=192.168.123.18
LAUNCH=/home/unitree/ws_odin/src/odin_ros_driver/launch_ROS1/odin1_ros1.launch
LOGDIR=/home/unitree/odin_bridge/logs
mkdir -p "$LOGDIR"

# kill stale master if any
killall -q roscore rosmaster 2>/dev/null || true
sleep 1

setsid roslaunch "$LAUNCH" > "$LOGDIR/odin_driver.log" 2>&1 < /dev/null &
echo $! > /home/unitree/odin_bridge/odin_driver.pid
sleep 6

setsid python3 /home/unitree/odin_bridge/odin_ros1_tcp_server.py \
  > "$LOGDIR/tcp_server.log" 2>&1 < /dev/null &
echo $! > /home/unitree/odin_bridge/tcp_server.pid
INNER
chmod +x /home/unitree/odin_bridge/start_all.sh
setsid bash /home/unitree/odin_bridge/start_all.sh > /home/unitree/odin_bridge/logs/start_all.log 2>&1 < /dev/null &
sleep 8

source /opt/ros/noetic/setup.bash
source /home/unitree/ws_odin/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
echo '--- dock status ---'
pgrep -a host_sdk_sample | head -2 || echo 'NO host_sdk_sample'
pgrep -a odin_ros1_tcp_server | head -2 || echo 'NO tcp_server'
ss -ltn | grep 5601 || echo 'NO listen 5601'
timeout 3 rostopic hz /odin1/imu 2>&1 | head -4 || true
timeout 3 rostopic hz /odin1/cloud_raw 2>&1 | head -4 || true
tail -5 /home/unitree/odin_bridge/logs/tcp_server.log || true
echo DOCK_BRIDGE_OK
EOF
