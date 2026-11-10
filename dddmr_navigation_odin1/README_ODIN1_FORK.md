# dddmr_navigation_odin1

Odin1-only fork of DDDMR global planning. Do **not** use this tree for Mid360.

- Mid360 tree: `/home/ll/dddmr_navigation` + Docker `dddmr_dev`
- Odin1 tree: `/home/ll/dddmr_navigation_odin1` + Docker `dddmr_odin1`

Primary launch: `ros2 launch p2p_move_base dddmr_global_ego_odin1.launch.py`

Odin-only configs:
- `config/p2p_move_base_localization_odin1.yaml`
- `config/dddmr_global_ego_frames_odin1.yaml`
- `rviz/p2p_move_base_localization_odin1.rviz`
- maps: `src/dddmr_global_planner/data/odin1/`

Never source both Mid and Odin `install/setup.bash` in the same shell.
