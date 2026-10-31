#!/usr/bin/env bash
# Save CT-LIO leveled map + linefit ground/obstacle, verify tilt, deploy to
# open3d_loc and dddmr_navigation.
set -eo pipefail

ROOT=/home/ll/ct_lio_to_ros2-master
MAP_DIR="${ROOT}/maps"
SEG_DIR="${MAP_DIR}/segmentation"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_MAP="${MAP_DIR}/ct_lio_map_${STAMP}.pcd"
LOC_MAP=/home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble/data/ct_lio_map.pcd
DDDMR_DATA=/home/ll/dddmr_navigation/src/dddmr_global_planner/data

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
unset CYCLONEDDS_URI || true
set +u
source /opt/ros/humble/setup.bash
source "${ROOT}/install/setup.bash"
source /home/ll/linefit_ground_segmentation_ros2/install/setup.bash
set -u

mkdir -p "${MAP_DIR}" "${SEG_DIR}"

echo "[deploy] Saving CT-LIO map -> ${OUT_MAP}"
ros2 param set /ct_lio save_map_path "${OUT_MAP}"
ros2 service call /save_map std_srvs/srv/Trigger {}
sleep 1

if [[ ! -f "${OUT_MAP}" ]]; then
  echo "[deploy] ERROR: map file not created: ${OUT_MAP}" >&2
  exit 1
fi

echo "[deploy] Saving linefit segmentation maps..."
# Service name may be absolute or relative depending on node namespace.
if ros2 service list | grep -q '/save_segmented_clouds'; then
  ros2 service call /save_segmented_clouds std_srvs/srv/Trigger {}
elif ros2 service list | grep -q 'save_segmented_clouds'; then
  SVC="$(ros2 service list | grep 'save_segmented_clouds' | head -1)"
  ros2 service call "${SVC}" std_srvs/srv/Trigger {}
else
  echo "[deploy] WARN: save_segmented_clouds service not found; check linefit log."
fi
sleep 1

# Prefer newest segmentation_* files under SEG_DIR / linefit default dirs
pick_latest() {
  local pattern="$1"
  ls -t ${pattern} 2>/dev/null | head -1 || true
}

# Prefer accumulated *_ground_map / *_obstacle_map (not single *_frame dumps).
GROUND="$(pick_latest "${SEG_DIR}/segmentation*_ground_map.pcd")"
OBST="$(pick_latest "${SEG_DIR}/segmentation*_obstacle_map.pcd")"
if [[ -z "${GROUND}" ]]; then
  GROUND="$(pick_latest /home/ll/linefit_ground_segmentation_ros2/segmentation*_ground_map.pcd)"
fi
if [[ -z "${OBST}" ]]; then
  OBST="$(pick_latest /home/ll/linefit_ground_segmentation_ros2/segmentation*_obstacle_map.pcd)"
fi

python3 - << PY
import numpy as np, open3d as o3d, sys
path = "${OUT_MAP}"
pc = o3d.io.read_point_cloud(path)
pts = np.asarray(pc.points)
if len(pts) < 100:
    print(f"[verify] FAIL too few points in {path}")
    sys.exit(2)
if len(pts) > 200000:
    pts = pts[np.random.default_rng(0).choice(len(pts), 200000, replace=False)]
z = pts[:, 2]
g = pts[z <= np.quantile(z, 0.35)]
c = g.mean(0)
n = np.linalg.svd(g - c, full_matrices=False)[2][-1]
n = n / np.linalg.norm(n)
if n[2] < 0:
    n = -n
tilt = float(np.degrees(np.arccos(np.clip(n[2], -1, 1))))
pitch = float(np.degrees(np.arctan2(-n[0], n[2])))
print(f"[verify] {path}")
print(f"[verify] ground_tilt_deg={tilt:.2f} pitchY_deg={pitch:.2f} N={len(np.asarray(pc.points))}")
if tilt > 2.5:
    print("[verify] FAIL: map still tilted > 2.5 deg — do not deploy")
    sys.exit(3)
print("[verify] OK: map is level enough")
PY

echo "[deploy] Copy map -> localization + dddmr"
cp -f "${OUT_MAP}" "${LOC_MAP}"
cp -f "${OUT_MAP}" "${DDDMR_DATA}/ct_lio_map_${STAMP}.pcd"
cp -f "${OUT_MAP}" "${DDDMR_DATA}/ct_lio_map.pcd"
ln -sfn "${OUT_MAP}" "${MAP_DIR}/ct_lio_map_latest.pcd"

if [[ -n "${GROUND}" && -f "${GROUND}" ]]; then
  cp -f "${GROUND}" "${DDDMR_DATA}/segmentation_ground_map.pcd"
  echo "[deploy] ground: ${GROUND}"
else
  echo "[deploy] WARN: no ground map found"
fi
if [[ -n "${OBST}" && -f "${OBST}" ]]; then
  cp -f "${OBST}" "${DDDMR_DATA}/segmentation_obstacle_map.pcd"
  echo "[deploy] obstacle: ${OBST}"
else
  echo "[deploy] WARN: no obstacle map found"
fi

# Verify segmentation tilt if present
if [[ -n "${GROUND}" && -f "${GROUND}" ]]; then
python3 - << PY
import numpy as np, open3d as o3d
path = "${GROUND}"
pts = np.asarray(o3d.io.read_point_cloud(path).points)
if len(pts) < 50:
    print("[verify] ground map too small"); raise SystemExit(0)
z = pts[:, 2]
g = pts[z <= np.quantile(z, 0.5)]
n = np.linalg.svd(g - g.mean(0), full_matrices=False)[2][-1]
n = n / np.linalg.norm(n)
if n[2] < 0: n = -n
tilt = float(np.degrees(np.arccos(np.clip(n[2], -1, 1))))
print(f"[verify] ground_map_tilt_deg={tilt:.2f}")
if tilt > 3.0:
    print("[verify] WARN: ground map still tilted")
PY
fi

echo
echo "[deploy] Done."
echo "  map:   ${OUT_MAP}"
echo "  loc:   ${LOC_MAP}"
echo "  dddmr: ${DDDMR_DATA}/segmentation_{ground,obstacle}_map.pcd"
echo "Next: stop mapping, start localization with the new map."
