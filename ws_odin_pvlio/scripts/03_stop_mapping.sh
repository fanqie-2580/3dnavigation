#!/usr/bin/env bash
pkill -f 'odin1_ct_lio_mapping.launch.py' || true
pkill -f 'odin1_pv_lio_mapping.launch.py' || true
pkill -f 'odin1_ros2.launch.py' || true
pkill -f 'run_eskf_odin1.launch.py' || true
pkill -f 'run_eskf.launch.py' || true
pkill -f 'odin_cloud_to_pvlio' || true
pkill -f 'pv_lio_node' || true
pkill -f 'ct_lio_eskf' || true
pkill -f 'host_sdk_sample' || true
pkill -f 'odin_ros_driver' || true
sleep 1
pkill -9 -f 'pv_lio_node|odin_cloud_to_pvlio|host_sdk_sample|ct_lio_eskf' || true
echo stopped
