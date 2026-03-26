# 本地工作区相对原作者 `upstream/main` 的修改凭证

> **用途**：与 GitHub 原作者仓库（远程名 **`upstream`**，如 `git@github.com:asure1008/camera03.git`）的 `main` 做 `fetch` / `merge` 出现冲突时，用本文判断「应保留本 fork 行为」还是「采纳上游」。你自己的 fork 应为远程 **`origin`**（如 `git@github.com:Jianghe-2025/camera03.git`）；**不要**再用 `origin` 指代原作者。  
> **一键同步**：见 **§6** `scripts/sync_upstream.sh`（自动对 §6 所列文件在冲突时 `checkout --ours`，完成后推送到 `origin`）。  
> **生成基准**：与 **`upstream/main`** 的差异请用 §1 命令随时刷新；若需记录快照哈希可写在本段。

---

## 1. 快速对照命令

在仓库根目录执行，可随时刷新「本 fork 相对原作者」的差异：

```bash
cd /home/uniubi/Desktop/camera03-main
git fetch upstream
git status
git diff upstream/main --stat
git diff upstream/main > /tmp/camera03-vs-upstream.patch   # 完整差异备份
```

与**自己 GitHub fork**（`origin`）的差异：

```bash
git fetch origin
git diff origin/main --stat
```

冲突文件中若需**整文件**二选一（慎用，会丢掉另一边的全部改动）：

```bash
# 保留「合并进来的远程版本」
git checkout --theirs -- path/to/file

# 保留「合并前本地的版本」
git checkout --ours -- path/to/file
```

> `ours` / `theirs` 在 **rebase** 与 **merge** 下含义相反；合并时一般 **`--ours`=当前分支（本地 main）**，**`--theirs`=被合并进来的远程**。若不确定，先用 `git status` 看提示，或用 `git show :2:file` / `git show :3:file` 对比两个版本。

---

## 2. 按文件的「本地意图」一览

| 路径 | 相对远程 | 本地意图摘要 |
|------|----------|----------------|
| `ptz_config.yaml` | 已改 | **端口**：`6670`/`6671`/`6674`；**Isaac**：`python_sh` → `/home/uniubi/isaacsim/python.sh`；**去掉** `presets` 持久化段；**RTSP URL** 与 mediamtx 端口一致。 |
| `mediamtx.yml` | 已改 | RTSP 监听 **`:6674`**（原为 `8554`）。 |
| `ptz_launcher.py` | 已改 | **`ISAAC_SIM_PYTHON` 环境变量**优先于 yaml；**FLV 拉流**使用配置项 **`_RTSP_PORT`**，不再写死 8554；**Tilt 换算**改为 `×90` + clamp；**移除** 预置位 CRUD、`presets` 写回 yaml、`/presets*` 等一大块逻辑；默认 PTZ 回退改为 `pan=0, tilt=-45`；启动时打印实际 `python_sh`。 |
| `ptz_stream.py` | 已合并远程 | 含上游 `preview_enabled`、OSD、`readBufferCount` 等；端口仍以本机 yaml 为准。 |
| `ptz_web_control.html` | 已改 | **API/WS/快照** 使用 **`location.origin` / `location.host`**，随 `launcher_port` 变化，**不再写死 8080**；ONVIF 提示文案随端口更新；与远程「预置位面板」等 UI 若有分歧，以本地为准或需手工合并。 |
| `test_onvif.py` | 已改 | 端口/默认连接与本地配置对齐（见 diff）。 |
| `README.md` | 已改 | 文档与当前 API 描述对齐（预置位说明简化、tilt 公式等）。 |
| `docs/API.md` | 已改 | ONVIF 预置位能力描述与 **4 个固定 token**、tilt 换算等与本地 launcher 一致。 |
| `.gitignore` | 已改 | 增加 **`nohup.out`**。 |
| `test_ws_flv.html` | **本地删除** | 工作区中不存在；远程仍有。Pull 时可能被**恢复**；若不需要可再次删除或加入 `.gitignore`。 |
| `test_wss_flv.html` | **本地删除** | 同上。 |

### 仅本地存在、远程无跟踪的文件（pull 一般不会冲突，除非远程后来新增了同名路径）

| 路径 | 说明 |
|------|------|
| `README_local.md` | 本机部署说明（路径、端口、Python 等）。 |
| `API.md`（仓库根目录） | 若与 `docs/API.md` 重复，合并远程时注意只保留一份权威说明。 |
| `docs/TRAINING.md` | 本地文档。 |
| `docs/服务地址与端口.md` | 地址/端口清单。 |
| `scripts/play_rtsp.sh` | ffplay 播放 RTSP。 |

---

## 3. 关键配置「远程默认 vs 本机当前」

便于冲突时一眼决策（**本机**指当前工作区意图）：

| 项 | 上游常见值（`upstream/main`） | 本 fork 意图 |
|----|-----------------------------|----------------|
| Launcher HTTP | `8080` | **`6670`** |
| Isaac 控制 HTTP | `8081` | **`6671`** |
| MediaMTX RTSP | `8554` | **`6674`** |
| `python_sh` | `…/env_isaaclab/...` 等 | **`/home/uniubi/isaacsim/python.sh`**（可用 **`ISAAC_SIM_PYTHON`** 覆盖） |
| 预置位 | `ptz_config.yaml` 中 `presets` + launcher 持久化 + `/presets*` API | **内置固定 token（1~4/home），不写回 yaml**（以当前 `ptz_launcher.py` 为准） |
| Tilt ONVIF ↔ 度 | 仿射公式 `-60*norm-30` 等 | **`-norm*90` clamp 到 [-90,30]**（与 `docs/API.md` 一致） |

---

## 4. 与远程合并时的推荐策略

1. **先备份**：`git diff upstream/main > ~/camera03-local-$(date +%F).patch`  
2. **推荐**：运行 **`./scripts/sync_upstream.sh`**（见 §6）；或手动 `git fetch upstream && git merge upstream/main`。  
3. **配置类**（`ptz_config.yaml`、`mediamtx.yml`）：冲突时通常应**保留本 fork 端口与 `python_sh`**，除非你有意改为与上游一致（脚本对整文件冲突会默认 `--ours`）。  
4. **逻辑类**（`ptz_launcher.py`、`ptz_stream.py` 等）：若上游新增功能与本 fork 简化冲突，需**手工合并**；脚本**不会**自动覆盖这两文件。  
5. **逻辑类（网页）**（`ptz_web_control.html`）：脚本在整文件冲突时默认保留本 fork；若需同时保留上游大块新 UI，应手工合并。  
6. 合并完成后在本机测：**`http://127.0.0.1:<launcher_port>/`**、**RTSP `6674`**、**Isaac 启动**。

---

## 5. 修订记录

| 日期 | 说明 |
|------|------|
| 2026-03-25 | 首版：基于 `origin/main` @ `8a2ee37` 之上的工作区差异整理。 |
| 2026-03-26 | 已 `pull` 同步至 `224da6e`；冲突按本文在 `ptz_launcher.py`（保留 tilt×90 + `_TILT_SCALE`）、`ptz_web_control.html`（`location.origin`）解决；`preview_enabled` 置 `true` 以保留网页预览。 |
| 2026-03-26 | 约定 **`upstream`=原作者**、**`origin`=自己的 fork**；新增 §6 `scripts/sync_upstream.sh` 与 Cursor 规则「拉取原作者代码」流程。 |

---

## 6. 一键拉取原作者并推送本 fork（`scripts/sync_upstream.sh`）

在仓库根目录执行：

```bash
./scripts/sync_upstream.sh
```

行为概要：

1. `git fetch upstream`，再 `git merge upstream/main` 到**当前分支**（需工作区干净）。  
2. 若发生冲突，对下列路径**整文件保留本 fork**（`git checkout --ours` + `git add`），与 §2 中「配置/文档/网页端口策略」一致：  
   `ptz_config.yaml`、`mediamtx.yml`、`ptz_web_control.html`、`test_onvif.py`、`README.md`、`docs/API.md`、`.gitignore`  
3. **`ptz_launcher.py`、`ptz_stream.py`** 若仍冲突：**不会**自动改，脚本退出并提示你按 §2 / §4 **手工合并**；完成后自行 `git add` + `git commit`，再 `git push origin <分支>`。  
4. 若所有冲突已解决，脚本会 `git commit --no-edit` 并完成 **`git push origin <当前分支>`**。

可选环境变量：

| 变量 | 含义 |
|------|------|
| `SYNC_UPSTREAM_FETCH_ONLY=1` | 只 `fetch upstream`，不合并、不推送 |
| `SYNC_UPSTREAM_NO_PUSH=1` | 合并并提交（若成功）但不 `push` |
| `UPSTREAM_BRANCH=main` | 上游分支名（默认 `main`） |

**限制**：脚本只做「列在 §6 的整文件保留本 fork」；**删除/重命名冲突**、`test_ws_flv.html` 等未列入路径仍可能需手工处理。调整策略时请同时改 **`scripts/sync_upstream.sh` 内数组** 与本文 §6，保持一致。

---

*本文档由本地维护；上游更新后请用 §1 命令重新核对 `git diff upstream/main`，必要时更新修订记录。*
