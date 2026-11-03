#!/usr/bin/env bash
# Force Odin1 custom_map_mode=0 so fusion uses sensor-frame /odin1/cloud_raw
# (NOT /odin1/cloud_slam which is already in Odin odom frame).
set -euo pipefail

WS=/home/ll/ws_odin_pvlio
CFG_SRC="$WS/src/odin_ros_driver/config/control_command.yaml"
CFG_INSTALL="$WS/install/odin_ros_driver/share/odin_ros_driver/config/control_command.yaml"

set_mode0() {
  local f="$1"
  [[ -f "$f" ]] || return 0
  if grep -q 'custom_map_mode:' "$f"; then
    sed -i 's/custom_map_mode:[[:space:]]*[0-9]/custom_map_mode: 0/' "$f"
  else
    echo "  custom_map_mode: 0" >> "$f"
  fi
  echo "[odin-mode0] $f -> $(grep -E 'custom_map_mode' "$f" | head -1)"
}

set_mode0 "$CFG_SRC"
set_mode0 "$CFG_INSTALL"

# Dock ROS1 tree if present (common path used by bridge scripts)
DOCK_CFG=/home/unitree/ws_odin/src/odin_ros_driver/config/control_command.yaml
if [[ -f "$DOCK_CFG" ]]; then
  set_mode0 "$DOCK_CFG"
elif command -v ssh >/dev/null 2>&1; then
  echo "[odin-mode0] Tip: on dock, set custom_map_mode: 0 in odin control_command.yaml and restart driver"
fi

cat <<'EOF'
[odin-mode0] Fusion requirements:
  - custom_map_mode: 0
  - subscribe /odin1/cloud_raw (+ optional /odin1/imu)
  - do NOT feed /odin1/cloud_slam into FAST_LIO_MULTI
Restart Odin driver / dock bridge after this change.
EOF
