# ego-planner-swarm-odin1

Odin1-only fork of EGO local planning. Do **not** use this tree for Mid360.

- Mid360: `/home/ll/ego-planner-swarm-ros2_version`
- Odin1: `/home/ll/ego-planner-swarm-odin1`

Primary launch:

```bash
source /home/ll/ego-planner-swarm-odin1/install/setup.bash
ros2 launch ego_planner dddmr_reference_tracking_odin1.launch.py \
  use_rviz:=true \
  odom_topic:=/baselink2map \
  cloud_topic:=/cloud_registered_1 \
  reference_path_topic:=/ego_repaired_global_path
```

RViz config: `odin1_navigation.rviz`

Never source both Mid and Odin EGO `install/setup.bash` in the same shell.
