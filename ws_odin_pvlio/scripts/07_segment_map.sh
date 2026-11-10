#!/usr/bin/env bash
# Segment Odin global PCD -> dddmr ground/obstacle maps
set -eo pipefail
WS=/home/ll/ws_odin_pvlio
SCRIPT="$WS/scripts/07_segment_map.py"

INPUT="${1:-}"
if [[ -z "$INPUT" ]]; then
  if [[ -f "$WS/maps/odin1_odom_map.pcd" ]]; then
    INPUT="$WS/maps/odin1_odom_map.pcd"
  else
    # fallback: newest map_*.pcd
    INPUT=$(ls -1t "$WS/maps"/map_*.pcd "$WS/maps"/odin1_*.pcd 2>/dev/null | head -1 || true)
  fi
fi

if [[ -z "$INPUT" || ! -f "$INPUT" ]]; then
  echo "[seg] no input PCD. Usage: $0 [map.pcd]"
  echo "      or save first: bash $WS/scripts/06_save_map.sh"
  exit 1
fi

OUT_DIR="${OUT_DIR:-$WS/maps}"
VOXEL="${VOXEL:-0.05}"
DISTANCE="${DISTANCE:-0.08}"
MAX_ANGLE="${MAX_ANGLE:-25}"
MAX_HEIGHT="${MAX_HEIGHT:-3.0}"
MIN_OBS_H="${MIN_OBSTACLE_HEIGHT:-0.12}"

echo "[seg] input=$INPUT"
echo "[seg] out_dir=$OUT_DIR voxel=$VOXEL distance=$DISTANCE"

python3 "$SCRIPT" \
  --input "$INPUT" \
  --out-dir "$OUT_DIR" \
  --voxel "$VOXEL" \
  --distance "$DISTANCE" \
  --max-angle "$MAX_ANGLE" \
  --max-height "$MAX_HEIGHT" \
  --min-obstacle-height "$MIN_OBS_H"

echo "[seg] copy to dddmr (optional):"
echo "  cp $OUT_DIR/segmentation_ground_map.pcd $OUT_DIR/segmentation_obstacle_map.pcd \\"
echo "     /home/ll/dddmr_navigation/src/dddmr_global_planner/data/"
