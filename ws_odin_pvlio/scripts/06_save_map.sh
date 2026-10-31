#!/usr/bin/env bash
# Save accumulated Odin-odom map PCD
set -eo pipefail
WS=/home/ll/ws_odin_pvlio
set +u
# shellcheck disable=SC1091
source "$WS/env.sh"
set -u
mkdir -p "$WS/maps"
OUT="${1:-}"
if [[ -n "$OUT" ]]; then
  # restart save path is fixed in node; copy after save if custom name wanted
  :
fi
echo "[save] calling /save_map ..."
RESP=$(ros2 service call /save_map std_srvs/srv/Trigger)
echo "$RESP"
SRC="$WS/maps/odin1_odom_map.pcd"
if [[ -f "$SRC" ]]; then
  STAMP=$(date +%Y%m%d_%H%M%S)
  CP="${OUT:-$WS/maps/odin1_odom_map_${STAMP}.pcd}"
  cp -f "$SRC" "$CP"
  ls -lh "$SRC" "$CP"
  echo "[save] ok: $CP"
else
  echo "[save] expected file missing: $SRC"
  exit 1
fi
