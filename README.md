# 3dnavigation

Robot 3D navigation / planning / localization workspace (source only).

## Push workflow

See **[`docs/push_to_github.md`](./docs/push_to_github.md)**（中文完整步骤）。

Summary: sync sources into this directory → commit →  
`git push https://fanqie-2580:<PAT>@github.com/fanqie-2580/3dnavigation.git main`  
（PAT 放 `/tmp/gh_token.txt`，不要写入 remote）。

## Projects

- `dddmr_navigation` — global planning (run/build in Docker)
- `ego-planner-swarm-ros2_version` — local planning
- `FAST_LIO_LOCALIZATION_HUMANOID-humble` — localization
- `Elevator-LIO-main`, `SCAN-Planner-main`, `go2_cmd_vel_handler`, `fastlio_reloc` — related packages

Build artifacts, bags, maps, and model weights are intentionally excluded.
