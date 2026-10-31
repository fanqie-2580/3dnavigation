#!/usr/bin/env bash
sshpass -p '123' ssh -o StrictHostKeyChecking=no unitree@192.168.123.18 'bash -s' <<'EOF'
pkill -f odin_ros1_tcp_server.py 2>/dev/null || true
pkill -f host_sdk_sample 2>/dev/null || true
pkill -f odin1_ros1.launch 2>/dev/null || true
pkill -f 'roslaunch.*odin' 2>/dev/null || true
sleep 1
pkill -9 -f odin_ros1_tcp_server.py 2>/dev/null || true
echo dock_bridge_stopped
EOF
