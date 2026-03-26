# Camera03 — Isaac Sim PTZ 虚拟相机推流工程

## 工程概述

本工程基于 **NVIDIA Isaac Sim** 实现了一套完整的 PTZ（Pan-Tilt-Zoom）虚拟安防相机系统，将 Isaac Sim 仿真场景的相机画面通过 **RTSP** 和 **MJPEG** 协议实时推流输出，并提供 Web 控制台实现相机角度、焦距及场景对象的实时调控。

**第三方平台对接**（服务启停、拉流、PTZ 控制等完整 HTTP/RTSP 说明）见：**[docs/API.md](./docs/API.md)**。

> **本机约定**：工程根目录为 `/home/uniubi/Desktop/camera03-main`。下文端口统一为 **667x**（6670 启动器、6671 Isaac 内置 HTTP、6672 TLS 代理、6674 RTSP；6673 预留 HLS）。若你尚未把 `ptz_config.yaml`、`mediamtx.yml` 改成相同端口，请先改配置再按本文命令操作，或临时把命令里的端口改成配置文件中的实际值。

---

## 系统架构

```
┌─────────────────────────────────────────────────────────┐
│  用户浏览器  http://localhost:6670                       │
└──────────────────────┬──────────────────────────────────┘
                       │ HTTP
┌──────────────────────▼──────────────────────────────────┐
│  ptz_launcher.py  :6670  (始终运行，GPU ≈ 0%)           │
│  • 服务 Web UI                                          │
│  • 管理 Isaac Sim 子进程的启动 / 停止                    │
│  • 代理 /control /scene/* /stream.mjpeg 到 :6671        │
└──────────────────────┬──────────────────────────────────┘
                       │ 按需启动（start_new_session）
┌──────────────────────▼──────────────────────────────────┐
│  ptz_stream.py  :6671  (Isaac Sim headless)             │
│  • 加载 V4.0.usd 场景                                   │
│  • Replicator RenderProduct 捕获相机画面                 │
│  • MJPEG 推流（浏览器低延迟预览）                        │
│  • RTSP 推流（VLC / NVR 接入）                          │
│  • PTZ 控制 API + 场景控制 API                          │
└────────────┬────────────────────┬───────────────────────┘
             │ stdin pipe         │ RTSP ANNOUNCE
┌────────────▼────────┐  ┌────────▼────────────────────────┐
│  ffmpeg（H.264编码）│  │  mediamtx  :6674  RTSP Server   │
└─────────────────────┘  └─────────────────────────────────┘
```

**关键设计**：`ptz_launcher.py` 用 `start_new_session=True` 为 Isaac Sim 子进程创建独立进程组，停止时通过 `os.killpg(pgid, SIGKILL)` 确保 bash 壳、Python 进程、mediamtx、ffmpeg **全部干净退出**，彻底释放 GPU 资源。

---

## 文件说明

| 文件 | 类型 | 说明 |
|------|------|------|
| `ptz_launcher.py` | Python | 轻量启动器（系统入口，始终运行，无 GPU 占用） |
| `ptz_stream.py` | Python | Isaac Sim 推流主体（按需启动，含所有 PTZ / 场景控制逻辑） |
| `ptz_config.yaml` | 配置 | 端口、场景路径、分辨率、码率等全局配置 |
| `ptz_web_control.html` | HTML | Web 控制台页面（由 launcher 提供服务） |
| `mediamtx.yml` | 配置 | MediaMTX 配置（示例 RTSP **:6674**；当前仓库默认关闭 HLS，若开启可配 **:6673**） |
| `mediamtx` | 二进制 | MediaMTX v1.17.0 可执行文件（.gitignore 忽略） |
| `V4.0.usd` | USD 场景 | 当前使用的仿真场景（工地环境，Z-up，metersPerUnit=1） |
| `setup_camera_v4.py` | Python | 一次性工具：为裸 Camera prim 创建 Pan/Tilt 控制层级 |
| `fix_ptz_orientation.py` | Python | 一次性工具：修复 Y-up 球机模型放入 Z-up 场景时的方向/缩放 |
| `test_onvif.py` | Python | 验证脚本：用 python-onvif-zeep 测试 ONVIF PTZ 控制接口 |
| `PTZ_Camera_Rig.usda` | USDA | PTZ 球机外观参考资产（立柱+云台+镜头+Camera，Z-up，米） |
| `DiaoLan_ChangJing_2026.03.18.usdz` | USDZ | 原始工地场景资产（大文件，.gitignore 忽略） |

---

## USD 场景结构（V4.0.usd）

```
/World
├── DiaoLanChangJing_2026  [Xform]  工地建筑主体
├── Architectur_2026_03    [Xform]  建筑构件
├── DiaoLan                [Xform]  吊篮系统（Y-up cm，父级有坐标系转换）
│   └── Model
│       ├── Group1         [Xform]  吊篮本体（Y 方向控制高度，范围 0~3300 cm）
│       ├── node______1    [Xform]  工人 A（可见性由 API 控制）
│       └── node______2    [Xform]  工人 B（可见性由 API 控制）
├── Camera                 [Camera] 原始相机（已 deactivate，由 setup_camera_v4.py 禁用）
└── CameraRig              [Xform]  PTZ 相机控制根节点
    │   xformOp:translate  = 相机在场景中的安装位置
    │   xformOp:rotateZ    = Pan 水平旋转（-170° ~ +170°）
    └── CamTilt            [Xform]
        │   xformOp:rotateY = Tilt 俯仰旋转（-90° ~ +30°）
        └── Camera         [Camera] 实际渲染相机
                focalLength = 1× 基准 × zoom（1× ~ 32×）
```

> **坐标系**：场景为 Z-up，metersPerUnit=1.0（米）。
> DiaoLan 内部模型坐标为 Y-up cm（父级 xformOp 做了 rotateX=90°、scale=0.01 转换）。

---

## 快速开始

### 1. 启动轻量服务（始终运行）

**解释器说明**：`ptz_launcher.py` 只需标准 **Python 3** 与 **PyYAML**（`pip install pyyaml`），不依赖 Isaac Sim。请勿使用不存在的虚拟环境路径（例如本机若无 `env_isaaclab`，则 `/home/uniubi/miniconda3/envs/env_isaaclab/bin/python3` 会报 *No such file*）。

检测本机可用的 `python3` 与 PyYAML：

```bash
command -v python3
python3 -c "import yaml; print('PyYAML OK')"
```

本仓库在当前机器上已验证：Miniconda **base** 可用 **`/home/uniubi/miniconda3/bin/python3`**（与 `(base)` 下默认 `python3` 一般为同一路径）。启动命令在工程目录内使用**相对路径**，避免写死重复绝对路径：

```bash
cd /home/uniubi/Desktop/camera03-main

# 1) 清理旧进程（避免 6670 / 6671 / 6674 被占用）
pkill -f 'ptz_launcher.py|ptz_stream.py|mediamtx' || true
sleep 2

# 2) 启动本工程的 ONVIF / Web 入口（任选其一）
#    A) 已激活 conda base 或系统 PATH 中 python3 可用时：
nohup python3 ptz_launcher.py --config ./ptz_config.yaml \
  > /tmp/onvif_debug.log 2>&1 &
#    B) 显式指定本机已存在的解释器（当前机器推荐，不依赖 env_isaaclab）：
# nohup /home/uniubi/miniconda3/bin/python3 ptz_launcher.py --config ./ptz_config.yaml \
#   > /tmp/onvif_debug.log 2>&1 &

# 3) 检查 launcher 是否启动成功（端口以 ptz_config.yaml 的 launcher_port 为准，常见为 8080 或 6670）
ss -ltnp | grep -E ':6670|:8080'
ps -ef | grep ptz_launcher.py | grep -v grep
tail -n 50 /tmp/onvif_debug.log
```

**Isaac Sim 推流**：点击「开始推流」时，子进程由 `ptz_config.yaml` 里的 **`python_sh`** 启动；必须指向你本机 **Isaac Sim 的 `python.sh`**（或官方文档要求的解释器）。仅有 Miniconda 的 `python3` 无法加载仿真，但与启动器能否先跑起来无关。

打开浏览器访问：**`http://127.0.0.1:<launcher_port>/`**，其中 `<launcher_port>` 与 `ptz_config.yaml` 里 `launcher_port` 一致（仓库默认常为 **8080**）。

### 2. 在 Web UI 中启动 Isaac Sim 推流

点击右上角 **"⏻ 开始推流"** 按钮。

也可以直接用命令启动（端口从配置读取，避免与 `ptz_config.yaml` 不一致）：

```bash
cd /home/uniubi/Desktop/camera03-main
LP=$(awk '/^launcher_port:/{print $2;exit}' ptz_config.yaml)
curl -X POST "http://127.0.0.1:${LP}/start"

# 检查 Isaac Sim / 推流侧是否已起来（ctrl_port / mediamtx 常见为 8081+8554 或文档约定的 6671+6674）
ss -ltnp | grep -E ':8081|:8554|:6671|:6674'
ps -ef | grep -E 'ptz_stream.py|mediamtx' | grep -v grep
tail -n 100 ./isaac_stream.log
```

Isaac Sim 首次冷启动约需 **60~120 秒**，加载完成后 Web UI 自动切换为视频画面。

### 2.1 为 `ws-flv` 启动 HTTPS / WSS 代理

当上层页面是 `https://` 时，浏览器通常不能直接连接 `ws://<host>:<launcher_port>/ws-flv`（`<launcher_port>` 见 `ptz_config.yaml`）。  
可在服务器上额外启动一个 TLS 代理，将 `https://` / `wss://` 转发到本机同一端口：

```bash
cd /home/uniubi/Desktop/camera03-main

# 1) 生成自签名证书（仅首次需要）
mkdir -p /home/uniubi/Desktop/camera03-main/certs
openssl req -x509 -newkey rsa:2048 -nodes -days 3650 \
  -keyout /home/uniubi/Desktop/camera03-main/certs/camera03.key \
  -out /home/uniubi/Desktop/camera03-main/certs/camera03.crt \
  -subj "/CN=192.168.61.171" \
  -addext "subjectAltName=IP:192.168.61.171"

# 2) 启动 HTTPS / WSS 代理（6672 -> launcher_port，仅需标准库，任意 python3 即可）
UP=$(awk '/^launcher_port:/{print $2;exit}' ptz_config.yaml)
nohup python3 tls_tcp_proxy.py \
  --listen-host 0.0.0.0 \
  --listen-port 6672 \
  --upstream-host 127.0.0.1 \
  --upstream-port "$UP" \
  --cert ./certs/camera03.crt \
  --key ./certs/camera03.key \
  > /tmp/camera03_tls_proxy.log 2>&1 &
# 若 PATH 中无 python3，可改为：nohup /home/uniubi/miniconda3/bin/python3 tls_tcp_proxy.py ...

# 3) 检查端口与日志
ss -ltnp | grep 6672
tail -n 50 /tmp/camera03_tls_proxy.log
```

启动后可使用：

```text
https://192.168.61.171:6672/
wss://192.168.61.171:6672/ws-flv
```

说明：
- 自签名证书默认不会被浏览器信任，首次访问需要手动接受证书风险。
- 当前用户无 `sudo`，因此默认使用 `6672` 等非特权端口，不是 `443`。

### 3. 停止推流释放 GPU

点击 **"⏹ 停止推流"**，launcher 向整个进程组发送 SIGTERM + SIGKILL，所有资源完全释放。

### 4. 外部客户端接入 RTSP

```bash
# VLC
vlc rtsp://localhost:6674/ptz_cam

# ffplay
ffplay rtsp://localhost:6674/ptz_cam -rtsp_transport tcp
```

---

## Web 控制台功能

### PTZ 相机控制

| 控件 | 功能 | 范围 |
|------|------|------|
| 2D 摇杆 | 同时控制水平角（Pan）和俯仰角（Tilt） | Pan: -170°~+170°，Tilt: -90°~+30° |
| 变焦滑条 | 光学变焦倍数 | 1× ~ 32× |
| 预设按钮 | 快捷跳转到预设角度 | — |
| 键盘 | ← → 调 Pan，↑ ↓ 调 Tilt，+/- 调 Zoom | — |

### 场景控制（吊篮 + 工人）

| 控件 | 功能 | 范围 |
|------|------|------|
| 吊篮高度滑条 | 控制 `/World/DiaoLan/Model/Group1` Y 轴位移 | 0 ~ 3300 cm |
| 工人数量按钮 | 0：全隐；1：随机显示一人；2：全显 | 0 / 1 / 2 |

工人显示时，其 Y 轴位置**自动跟随吊篮高度**。

---

## HTTP API 参考

所有接口均在 **`http://127.0.0.1:<launcher_port>`**（launcher 代理，`<launcher_port>` 同 `ptz_config.yaml`）。

### 启动器管理

| 方法 | 路径 | 说明 |
|------|------|------|
| GET | `/status` | 返回 `isaac_state`（stopped/starting/running/stopping）及 PTZ 状态 |
| POST | `/start` | 启动 Isaac Sim 推流进程 |
| POST | `/stop` | 停止并完全清理 Isaac Sim 进程组 |
| GET | `/log` | 返回 Isaac Sim 启动日志末尾 8 KB |

### PTZ 控制（Isaac Sim 运行时有效）

```http
POST /control
Content-Type: application/json

{"pan": 45.0, "tilt": -30.0, "zoom": 8.0}
```

### 场景控制（Isaac Sim 运行时有效）

```http
# 吊篮高度
POST /scene/gondola
{"y": 1650}

# 工人数量（0/1/2）
POST /scene/workers
{"count": 1}

# 查询场景当前状态
GET /scene/state
```

### 视频流

| 路径 | 说明 |
|------|------|
| `GET /stream.mjpeg` | MJPEG 连续流（浏览器 canvas 低延迟播放，延迟 ~100-300ms） |
| `GET /snapshot.jpg` | 单帧 JPEG 快照 |

### ONVIF 服务（供 VMS / NVR 对接）

| 方法 | 路径 | 说明 |
|------|------|------|
| POST | `/onvif/device_service` | ONVIF 设备服务（GetCapabilities / GetSystemDateAndTime / GetServices） |
| POST | `/onvif/media_service` | ONVIF 媒体服务（GetProfiles / GetSnapshotUri / GetServiceCapabilities） |
| POST | `/onvif/ptz_service` | ONVIF PTZ 控制服务（AbsoluteMove / RelativeMove / GotoPreset / GetStatus / GetNodes / GetConfigurations） |
| GET | `/onvif-snap.jpg` | 单帧快照（ONVIF GetSnapshotUri 返回的 URL，代理到 Isaac Sim 内部快照） |

坐标单位转换（ONVIF 归一化 → Camera03）：

| ONVIF 参数 | 范围 | Camera03 参数 | 换算 |
|-----------|------|--------------|------|
| pan（x） | \[-1, 1\] | pan_deg | `× 170.0` → \[-170°, 170°\] |
| tilt（y） | \[-1, 1\] | tilt_deg | `- × 90.0` → \[-90°, 30°\]（clamp；向上更小，向下更大） |
| zoom（x） | \[0, 1\] | zoom_x | `1.0 + × 31.0` → \[1×, 32×\] |

预置位（GotoPreset token）：`"1"` = 正前方 -45°、`"2"` = 水平正视、`"3"` = 右转 90°、`"4"` = 左转 90°、`"home"` = 默认位。

---

## 配置文件（ptz_config.yaml）

```yaml
launcher_port: 6670          # launcher Web UI 端口
ctrl_port:     6671          # Isaac Sim 内部 HTTP 端口
python_sh: /path/to/python.sh  # Isaac Sim python 解释器路径

scene_path:   ./V4.0.usd    # 要加载的 USD 场景
camera_prim:  /World/CameraRig/CamTilt/Camera  # 推流相机路径

resolution:   [1920, 1080]
fps:          25
bitrate:      "4M"
rtsp_enabled: true           # false 时仅提供 /snapshot.jpg 快照，不启动 ffmpeg/mediamtx
focal_length_1x: 18.14756    # 1× 基准焦距（mm），32× 时自动 ×32
rtsp_url: rtsp://localhost:6674/ptz_cam  # 与 mediamtx 监听端口一致

mediamtx:
  port: 6674                 # 与 mediamtx.yml 中 rtspAddress 一致（例如 :6674）
```

另需在 `mediamtx.yml` 中将 `rtspAddress` 设为 `:6674`（若启用 HLS，再在配置里使用 **6673** 等与本文约定一致）。

---

## 移植到新工程

只需将以下文件复制到目标工程目录：

```
ptz_launcher.py
ptz_stream.py
ptz_config.yaml
ptz_web_control.html
mediamtx.yml
mediamtx（二进制，或设 auto_download: true 自动下载）
```

然后修改 `ptz_config.yaml` 中：
1. `python_sh` → Isaac Sim 安装路径
2. `scene_path` → 目标场景文件
3. `camera_prim` → 目标场景中的相机 Prim 路径

如果目标场景的相机没有 Pan/Tilt 控制层级，运行一次：
```bash
python3 setup_camera_v4.py --scene ./YourScene.usd --cam /World/Camera
```

---

## 开发工具脚本

### setup_camera_v4.py

为场景中已有的裸 Camera prim 添加 PTZ 控制层级（一次性执行）：

```bash
python3 setup_camera_v4.py [--scene ./V4.0.usd] [--cam /World/Camera]
```

执行后自动创建 `CameraRig / CamTilt / Camera` 三层结构，原 Camera 被 deactivate。

### fix_ptz_orientation.py

修复将 Y-up / cm 坐标的球机模型（如 PTZ_SecurityDome.usda）放入 Z-up / 米坐标场景时出现的"倒地"和"体积异常"问题：

```bash
python3 fix_ptz_orientation.py [--scene ./V3.0.usd] [--x 5.0] [--y -8.0] [--z 0.0]
```

自动应用 `rotateXYZ=(-90, 0, 0)` 和 `scale=(0.01, 0.01, 0.01)`。

---

## 技术栈

| 组件 | 版本 / 说明 |
|------|------------|
| NVIDIA Isaac Sim | headless 模式，Raytraced Lighting 渲染 |
| USD (pxr) | 场景描述，Z-up / metersPerUnit=1 |
| omni.replicator.core | RenderProduct + RGB Annotator 图像捕获 |
| MediaMTX | v1.17.0，RTSP Server |
| ffmpeg | H.264 编码，yuv420p，零延迟推流 |
| Pillow / OpenCV | MJPEG JPEG 帧编码 |
| Python 3.11 | Isaac Sim 内置 Python 环境 |

---

## 端口占用一览

| 端口 | 进程 | 用途 |
|------|------|------|
| 6670 | ptz_launcher.py | Web UI + API（始终监听） |
| 6671 | ptz_stream.py | Isaac Sim 内部 API（按需） |
| 6672 | tls_tcp_proxy.py（可选） | HTTPS / WSS 代理至启动器 |
| 6673 | mediamtx | HLS（可选；当前默认关闭） |
| 6674 | mediamtx | RTSP 流（供 VLC/NVR）（按需） |
