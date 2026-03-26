# 本地工作区相对 `origin/main` 的修改凭证

> **用途**：与 GitHub 远程 `asure1008/camera03` 的 `main` 分支做 `git pull` / 合并出现冲突时，用本文判断「应保留本地行为」还是「采纳远程」。  
> **生成基准**：已自 GitHub 同步至 **`origin/main` @ `224da6e`**；本机在之上另有提交 **`9d67c7d`**（按本文策略合并冲突后的工作区快照）。之后若有新 pull，请重新执行 §1 并更新本段哈希。

---

## 1. 快速对照命令

在仓库根目录执行，可随时刷新「与远程的差异」：

```bash
cd /home/uniubi/Desktop/camera03-main
git fetch origin
git status
git diff origin/main --stat
git diff origin/main > /tmp/camera03-vs-origin.patch   # 完整差异备份
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

| 项 | 远程常见值（`origin/main`） | 本机当前意图 |
|----|-----------------------------|----------------|
| Launcher HTTP | `8080` | **`6670`** |
| Isaac 控制 HTTP | `8081` | **`6671`** |
| MediaMTX RTSP | `8554` | **`6674`** |
| `python_sh` | `…/env_isaaclab/...` 等 | **`/home/uniubi/isaacsim/python.sh`**（可用 **`ISAAC_SIM_PYTHON`** 覆盖） |
| 预置位 | `ptz_config.yaml` 中 `presets` + launcher 持久化 + `/presets*` API | **内置固定 token（1~4/home），不写回 yaml**（以当前 `ptz_launcher.py` 为准） |
| Tilt ONVIF ↔ 度 | 仿射公式 `-60*norm-30` 等 | **`-norm*90` clamp 到 [-90,30]**（与 `docs/API.md` 一致） |

---

## 4. 与远程合并时的推荐策略

1. **先备份**：`git diff origin/main > ~/camera03-local-$(date +%F).patch`  
2. **先拉再合**：`git pull origin main`（有冲突则按文件处理）。  
3. **配置类**（`ptz_config.yaml`、`mediamtx.yml`）：冲突时通常应**保留本机端口与 `python_sh`**，除非你有意改为与远程一致。  
4. **逻辑类**（`ptz_launcher.py`、`ptz_web_control.html`）：若远程新增功能（例如预置位持久化），需**手工合并**两段逻辑，不能只选一侧。  
5. 合并完成后在本机测：**`http://127.0.0.1:<launcher_port>/`**、**RTSP `6674`**、**Isaac 启动**。

---

## 5. 修订记录

| 日期 | 说明 |
|------|------|
| 2026-03-25 | 首版：基于 `origin/main` @ `8a2ee37` 之上的工作区差异整理。 |
| 2026-03-26 | 已 `pull` 同步至 `224da6e`；冲突按本文在 `ptz_launcher.py`（保留 tilt×90 + `_TILT_SCALE`）、`ptz_web_control.html`（`location.origin`）解决；`preview_enabled` 置 `true` 以保留网页预览。 |

---

*本文档由本地维护；远程更新后请用第 1 节命令重新核对 `git diff origin/main`，必要时更新本节「生成基准」提交哈希。*
