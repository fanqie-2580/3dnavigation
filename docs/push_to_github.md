# 推送到 GitHub

- 上传根目录：`/home/ll/3dnavigation_github_upload`
- 远端：`https://github.com/fanqie-2580/3dnavigation`（HTTPS + PAT，不用 SSH）
- 分支：`main`

## 要同步的项目

- `dddmr_navigation` ← `/home/ll/dddmr_navigation`（Mid360）
- `dddmr_navigation_odin1` ← `/home/ll/dddmr_navigation_odin1`（Odin1）
- `ego-planner-swarm-ros2_version` ← `/home/ll/ego-planner-swarm-ros2_version`（Mid360）
- `ego-planner-swarm-odin1` ← `/home/ll/ego-planner-swarm-odin1`（Odin1）
- `FAST_LIO_LOCALIZATION_HUMANOID-humble` ← `/home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble`
- 以及工作区内其它已改动的文档/脚本

## 排除

`build/`、`install/`、`log/`、bags、maps（`*.pcd`/`*.ply`）、模型权重、大媒体。

## 流程

1. `rsync -a --copy-links`（带上述排除）到 upload 根下对应目录  
2. `git status` / `git diff` → commit（写清 why）  
3. HTTPS + PAT 推 `origin main`  
4. 不要把 PAT 写进 remote URL 并提交  
