#!/usr/bin/env bash
set -eo pipefail
DOCK=unitree@192.168.123.18
PASS=123
sshpass -p "$PASS" ssh -o StrictHostKeyChecking=no "$DOCK" 'bash -s' <<'EOF'
pkill -f odin_odom_mapper_ros1.py 2>/dev/null || true
pkill -f odin_ros1_tcp_server.py 2>/dev/null || true
pkill -f host_sdk_sample 2>/dev/null || true
pkill -f odin1_ros1.launch 2>/dev/null || true
sleep 1
echo stopped_dock_mapping
EOF
pkill -f odin_ros2_tcp_client.py 2>/dev/null || true
pkill -f 'odin_odom_mapper.py' 2>/dev/null || true
echo stopped_jetson_bridge
