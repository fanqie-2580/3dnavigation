# 推送到 GitHub（固定流程）

私有仓：**https://github.com/fanqie-2580/3dnavigation**（账号 `fanqie-2580`，分支 `main`）

本地上传用 git 根目录（不是工作区里各项目各自的 `.git`）：

```text
/home/ll/3dnavigation_github_upload
```

**不要用** G7-3208 的 SSH 推这个仓；一律 **HTTPS + Personal Access Token**。

---

## 1. 准备 PAT（只放临时文件，不写进 remote）

```bash
# 把 token 写入临时文件（权限 600），不要 commit，不要写进 git remote URL 持久配置
printf '%s' 'ghp_xxxxxxxx' > /tmp/gh_token.txt
chmod 600 /tmp/gh_token.txt
```

校验：

```bash
TOKEN=$(tr -d '\n' < /tmp/gh_token.txt)
curl -sS -o /tmp/gh_user.json -w '%{http_code}\n' \
  -H "Authorization: token ${TOKEN}" \
  -H "Accept: application/vnd.github+json" \
  https://api.github.com/user
python3 -c 'import json; print(json.load(open("/tmp/gh_user.json")).get("login"))'
# 应输出: fanqie-2580
```

申请/更换 token：https://github.com/settings/tokens （勾选 `repo`）

---

## 2. 同步源码到上传目录（只拷改过的项目）

例：改了 ego / docs / ws_odin 脚本时：

```bash
UPLOAD=/home/ll/3dnavigation_github_upload

# ego（示例）
rsync -a --delete --copy-links \
  --exclude 'build/' --exclude 'install/' --exclude 'log/' \
  --exclude '.git/' \
  --exclude '*.bag' --exclude '*.db3' --exclude '*.mcap' \
  --exclude '*.pcd' --exclude '*.ply' \
  --exclude '*.onnx' --exclude '*.pt' --exclude '*.pth' --exclude '*.engine' \
  /home/ll/ego-planner-swarm-ros2_version/ \
  "$UPLOAD/ego-planner-swarm-ros2_version/"

# 文档（本工作区）
mkdir -p "$UPLOAD/docs"
cp -a /home/ll/3dnavigation/docs/*.md "$UPLOAD/docs/"
```

**必须排除**：`build/` `install/` `log/`、bag、地图 PCD/PLY、模型权重、大媒体。

常见源 → 上传子目录：

| 源路径 | 上传子目录 |
|--------|------------|
| `/home/ll/dddmr_navigation` | `dddmr_navigation/` |
| `/home/ll/ego-planner-swarm-ros2_version` | `ego-planner-swarm-ros2_version/` |
| `/home/ll/FAST_LIO_LOCALIZATION_HUMANOID-humble` | `FAST_LIO_LOCALIZATION_HUMANOID-humble/` |
| `/home/ll/3dnavigation/docs` | `docs/` |
| `/home/ll/ws_odin_pvlio`（脚本/文档） | `ws_odin_pvlio/` |

---

## 3. 提交

```bash
cd /home/ll/3dnavigation_github_upload
git status
git add <改动的路径...>
GIT_AUTHOR_NAME='fanqie-2580' GIT_AUTHOR_EMAIL='fanqie-2580@users.noreply.github.com' \
GIT_COMMITTER_NAME='fanqie-2580' GIT_COMMITTER_EMAIL='fanqie-2580@users.noreply.github.com' \
git commit -m "$(cat <<'EOF'
一句话说明为什么改（why）。

EOF
)"
```

---

## 4. 推送（HTTPS + PAT，一次性 URL，不改 origin）

```bash
cd /home/ll/3dnavigation_github_upload
TOKEN=$(tr -d '\n' < /tmp/gh_token.txt)

git push "https://fanqie-2580:${TOKEN}@github.com/fanqie-2580/3dnavigation.git" main \
  2>&1 | sed -E 's/(ghp_[A-Za-z0-9]+|github_pat_[A-Za-z0-9_]+|:[^/@]+@)/***@***/g'

# 同步本地 tracking（可选）
git fetch "https://fanqie-2580:${TOKEN}@github.com/fanqie-2580/3dnavigation.git" main \
  2>&1 | sed -E 's/(ghp_[A-Za-z0-9]+|github_pat_[A-Za-z0-9_]+|:[^/@]+@)/***@***/g'
git update-ref refs/remotes/origin/main "$(git rev-parse HEAD)"
git status -sb
git log -1 --oneline
```

成功后应看到类似：`xxxx..yyyy  main -> main`，仓库地址：

https://github.com/fanqie-2580/3dnavigation

---

## 5. 注意

- **不要**把 PAT 写进 `git remote set-url` 或任何会进仓库的文件。
- **不要** force-push `main`，除非明确要求。
- 只分析、未改文件时不必推送。
- 推送日志用 `sed` 遮住 token，避免终端/日志泄漏。
