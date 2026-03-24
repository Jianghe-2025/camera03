# PTZ 仿真 ONVIF / WebSocket 服务 — 第三方对接接口文档

> 版本：与当前仓库 `ptz_launcher.py` / `ptz_stream.py` 一致  
> 默认端口以 `ptz_config.yaml` 为准，部署时可修改。

---

## 1. 概述

### 1.1 服务角色

| 服务 | 默认端口 | 说明 |
|------|----------|------|
| **启动器（Launcher）** | `8080` | 常驻进程，提供 HTTP API、Web 控制台、**ONVIF SOAP 服务**，并按需拉起/停止 Isaac Sim |
| **Isaac Sim 推流进程** | `8081`（内部） | 由 Launcher 启动，第三方**无需直连**；控制与抓图均经 `:8080` 代理 |
|| **WebSocket 视频流** | `8765` | 由 Isaac Sim 进程直接监听；每帧推送一条 binary JPEG 消息；Isaac 运行时生效 |

### 1.2 对接前提

1. 目标机器已安装 Isaac Sim，且 `ptz_config.yaml` 中 `python_sh` 指向正确的 `python.sh`。  
2. 先启动 Launcher（常驻）：  
   ```bash
   python3 ptz_launcher.py [--config ./ptz_config.yaml]
   ```  
3. 第三方需先调用 **`POST /start`** 启动仿真；ONVIF PTZ 指令在仿真就绪后才能实际生效。  
4. 跨域：所有 JSON 接口响应头含 `Access-Control-Allow-Origin: *`，浏览器页面可跨域调用。

### 1.3 Base URL 约定

下文 `{BASE}` 表示 Launcher 根地址，例如：

- 本机：`http://127.0.0.1:8080`
- 局域网：`http://<服务器IP>:8080`

---

## 2. 服务生命周期（启动 / 停止 / 状态）

### 2.1 启动仿真

**POST** `{BASE}/start`

- **Content-Type**：无要求（无 Body）
- **说明**：异步启动 Isaac Sim；首次冷启动约 **60～120 秒**。

**响应 200**（JSON）：

| 字段 | 类型 | 说明 |
|------|------|------|
| `ok` | bool | 是否接受启动请求 |
| `pid` | int | 子进程组首进程 PID（成功时） |
| `log` | string | Isaac 日志文件绝对路径（成功时） |
| `error` | string | 失败原因（`ok=false` 时） |

**示例**

```http
POST /start HTTP/1.1
Host: 192.168.1.100:8080
```

```json
{"ok": true, "pid": 12345, "log": "/path/to/isaac_stream.log"}
```

重复启动且已在运行：

```json
{"ok": false, "error": "Isaac Sim 已在运行"}
```

---

### 2.2 停止仿真

**POST** `{BASE}/stop`

- **说明**：向 Isaac Sim 进程组发送终止信号，释放 GPU。

**响应 200**（JSON）：

| 字段 | 类型 | 说明 |
|------|------|------|
| `ok` | bool | 是否执行了停止流程 |
| `stopping_pid` | int | 被停止的进程组 PID（成功时） |
| `error` | string | 未在运行时 |

```json
{"ok": true, "stopping_pid": 12345}
```

```json
{"ok": false, "error": "Isaac Sim 未在运行"}
```

---

### 2.3 查询运行状态与 PTZ 快照

**GET** `{BASE}/status`

**响应 200**（JSON）：

| 字段 | 类型 | 说明 |
|------|------|------|
| `isaac_state` | string | `stopped` \| `starting` \| `running` \| `stopping` |
| `isaac_port` | int | Isaac 内部 HTTP 端口（默认 8081，仅作信息展示） |
| `uptime_s` | int | 运行中时为已运行秒数；未运行为 `0` |
| `ptz` | object \| null | 运行中且 HTTP 就绪时为当前 PTZ；否则 `null` |

**`ptz` 对象字段**：

| 字段 | 类型 | 说明 |
|------|------|------|
| `pan` | number | 水平角（度） |
| `tilt` | number | 俯仰角（度） |
| `zoom` | number | 变焦倍数 |

**示例**

```json
{
  "isaac_state": "running",
  "isaac_port": 8081,
  "uptime_s": 3600,
  "ptz": {"pan": 0.0, "tilt": -45.0, "zoom": 1.0}
}
```

**轮询建议**：启动后每 **2～5 秒** 轮询 `/status`，直到 `isaac_state === "running"` 且 `ptz` 非空（或超时告警）。

---

### 2.4 查询启动日志（排错）

**GET** `{BASE}/log`

- **说明**：返回 `isaac_stream.log` **末尾最多 8KB** 文本。
- **响应**：`200` 为 `text/plain`；无日志文件时 `404`。

---

## 3. ONVIF 服务接口（视频取图与 PTZ 控制）

Launcher（`:8080`）内置轻量 ONVIF SOAP 服务，供 VMS / NVR / `python-onvif-zeep` 等标准 ONVIF 客户端直接对接，**无需修改外部系统**。

> **视频取图**：使用 ONVIF `GetSnapshotUri` → HTTP GET 单帧快照  
> **PTZ 控制**：使用 ONVIF `AbsoluteMove` / `RelativeMove` / `GotoPreset`

### 3.1 ONVIF 端点一览

| 方法 | 路径 | ONVIF 服务 |
|------|------|-----------|
| POST | `{BASE}/onvif/device_service` | Device Service（GetCapabilities / GetSystemDateAndTime / GetServices） |
| POST | `{BASE}/onvif/media_service` | Media Service（GetProfiles / GetSnapshotUri / GetServiceCapabilities） |
| POST | `{BASE}/onvif/ptz_service` | PTZ Service（AbsoluteMove / RelativeMove / GotoPreset / GetStatus / GetNodes / GetPresets） |
| GET  | `{BASE}/onvif-snap.jpg` | 单帧快照（GetSnapshotUri 返回的 URL，代理到 Isaac Sim 内部快照） |

### 3.2 ONVIF 客户端连接参数

| 参数 | 值 |
|------|-----|
| `host` | 服务器 IP（如 `192.168.1.100`） |
| `port` | `8080` |
| `user` | 任意（如 `admin`）——当前版本不验证 |
| `password` | 任意（如 `admin`）——当前版本不验证 |

### 3.3 坐标单位转换

ONVIF 使用归一化坐标，Launcher 自动换算为 Camera03 内部单位：

| ONVIF 参数 | ONVIF 范围 | Camera03 参数 | 换算公式 |
|-----------|-----------|--------------|---------|
| PanTilt.x（Pan） | [-1, 1] | pan（°） | `pan_deg = onvif_pan × 170.0` |
| PanTilt.y（Tilt） | [-1, 1] | tilt（°） | `tilt_deg = clamp(onvif_tilt × 90.0, -90, 30)` |
| Zoom.x | [0, 1] | zoom（倍） | `zoom_x = 1.0 + onvif_zoom × 31.0` |

### 3.4 预置位（GotoPreset）

| PresetToken | pan（°） | tilt（°） | zoom（×） | 说明 |
|------------|---------|---------|---------|------|
| `"1"` | 0.0 | -45.0 | 1.0 | 正前方（45° 俯角） |
| `"2"` | 90.0 | -45.0 | 1.0 | 右转 90° |
| `"3"` | -90.0 | -45.0 | 1.0 | 左转 90° |
| `"home"` | 0.0 | -45.0 | 1.0 | 默认位（同 token 1） |

### 3.5 典型集成示例（python-onvif-zeep）

```python
from onvif import ONVIFCamera

cam    = ONVIFCamera("192.168.1.100", 8080, "admin", "admin")
media  = cam.create_media_service()
ptz    = cam.create_ptz_service()

profile_token = media.GetProfiles()[0].token

# 获取快照 URI（通常为 http://192.168.1.100:8080/onvif-snap.jpg）
snap_uri = media.GetSnapshotUri({"ProfileToken": profile_token}).Uri
# 直接 HTTP GET snap_uri 即可取图

# AbsoluteMove（ONVIF 归一化坐标）
ptz.AbsoluteMove({
    "ProfileToken": profile_token,
    "Position": {
        "PanTilt": {"x": 0.3, "y": -0.2},  # → pan=51°, tilt=-18°
        "Zoom":    {"x": 0.1},              # → zoom=4.1×
    },
})

# RelativeMove
ptz.RelativeMove({
    "ProfileToken": profile_token,
    "Translation": {
        "PanTilt": {"x": 0.05, "y": 0.05},
    },
})

# GotoPreset
req = ptz.create_type("GotoPreset")
req.ProfileToken = profile_token
req.PresetToken  = "home"
ptz.GotoPreset(req)
```

> **注意**：ONVIF 接口仅在 Isaac Sim 进程运行时（`isaac_state == "running"`）才能实际下发 PTZ 指令；
> 未运行时 SOAP 响应仍返回 200，但内部控制调用将静默失败，快照返回 503。

---

## 4. 相机 PTZ 直接控制（可选，内部调试）

**POST** `{BASE}/control`

- **Content-Type**：`application/json`
- **前提**：`isaac_state === "running"`；否则 **503**
- **说明**：字段均可选，仅发送需要修改的项。生产环境建议优先通过 ONVIF 控制。

**请求体**：

| 字段 | 类型 | 范围 | 说明 |
|------|------|------|------|
| `pan` | number | -170.0 ~ 170.0 | 水平角（度） |
| `tilt` | number | -90.0 ~ 30.0 | 俯仰角（度） |
| `zoom` | number | 1.0 ~ 32.0 | 变焦倍数 |

**响应 200**：

```json
{"ok": true, "state": {"pan": 45.0, "tilt": -30.0, "zoom": 8.0}}
```

**示例**

```bash
curl -s -X POST "http://192.168.1.100:8080/control" \
  -H "Content-Type: application/json" \
  -d '{"pan":10,"tilt":-20,"zoom":4}'
```

---

## 5. 场景控制（吊篮 / 工人）

以下接口均经 Launcher 代理；**仅仿真运行中**有效，否则 **503**。

### 5.1 吊篮高度

**POST** `{BASE}/scene/gondola`

| 字段 | 类型 | 范围 | 说明 |
|------|------|------|------|
| `y` | number | 0 ~ 3300 | 吊篮模型局部 Y 高度（cm） |

```json
{"ok": true, "state": {"gondola_y": 1650.0, "workers": 2}}
```

### 5.2 工人显示数量

**POST** `{BASE}/scene/workers`

| 字段 | 类型 | 取值 | 说明 |
|------|------|------|------|
| `count` | int | 0 / 1 / 2 | 0：全隐藏；1：随机显示一名；2：两名均显示 |

```json
{"ok": true, "state": {"gondola_y": 0.0, "workers": 1}}
```

### 5.3 查询场景状态

**GET** `{BASE}/scene/state`

```json
{"gondola_y": 1650.0, "workers": 2}
```

---

## 6. CORS 与 OPTIONS

- 所有接口响应含：`Access-Control-Allow-Origin: *`
- **OPTIONS** 任意路径：返回 **204**

---

## 7. HTTP 状态码约定

| 状态码 | 场景 |
|--------|------|
| 200 | 成功（含 ONVIF SOAP 200） |
| 204 | OPTIONS 预检 |
| 404 | 路径不存在 |
| 502 | Launcher 代理到 Isaac 失败 |
| 503 | 仿真未运行，但调用了需 Isaac 的接口 |

---

## 8. 第三方集成流程建议

```
1. 部署并常驻启动：python3 ptz_launcher.py
2. POST /start
3. 循环 GET /status，直到 isaac_state == "running" 且 ptz 非空（或超时告警，建议 ≥ 180s）
4. 用 ONVIF 客户端连接：ONVIFCamera(host, 8080, "admin", "admin")
   ├─ 取图：GetSnapshotUri → HTTP GET /onvif-snap.jpg
   └─ 控制：AbsoluteMove / RelativeMove / GotoPreset
5. 结束业务：POST /stop 释放 GPU
```

---

## 9. WebSocket 视频流

### 9.1 连接方式

| 字段 | 值 |
|------|----|
| 协议 | `ws://` |
| 地址 | `ws://<服务器IP>:<ws_port>/` |
| 默认端口 | `8765`（由 `ptz_config.yaml` 的 `ws_port` 配置） |
| 消息类型 | **binary**（每条消息为一帧完整 JPEG 图像字节） |
| 帧率 | 与 `fps` 配置一致（默认 15 fps）；只有在线客户端 > 0 时才编码 JPEG，节省 CPU |
| 生效条件 | Isaac Sim 进程运行后自动监听；Isaac 停止后端口关闭 |

### 9.2 端口获取

WebSocket 端口可从 Launcher 动态读取，无需硬编码：

```http
GET {BASE}/status
```

响应中包含 `ws_port` 字段：

```json
{
  "isaac_state": "running",
  "ws_port": 8765
}
```

### 9.3 浏览器接入示例

```javascript
const ws = new WebSocket(`ws://${host}:8765/`);
ws.binaryType = 'blob';

ws.onmessage = (ev) => {
  const url = URL.createObjectURL(ev.data);
  const img = new Image();
  img.onload = () => { ctx.drawImage(img, 0, 0); URL.revokeObjectURL(url); };
  img.src = url;
};
```

### 9.4 Python 接入示例

```python
import asyncio, websockets, cv2, numpy as np

async def recv_frames():
    async with websockets.connect("ws://192.168.1.100:8765/") as ws:
        while True:
            jpeg_bytes = await ws.recv()          # bytes
            arr   = np.frombuffer(jpeg_bytes, np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            cv2.imshow("PTZ", frame)
            cv2.waitKey(1)

asyncio.run(recv_frames())
```

### 9.5 与 ONVIF/RTSP 的关系

| 通道 | 协议 | 典型客户端 | 延迟 |
|------|------|-----------|------|
| ONVIF `GetSnapshotUri` | HTTP | NVR、ONVIF 客户端 | ~100ms/帧（按需拉取） |
| RTSP `:8554/ptz_cam` | RTSP/H.264 | VLC、NVR、FFmpeg | ~200ms |
| WebSocket `:8765/` | WS/JPEG | 浏览器、Python | < 100ms |

三个通道**同时可用**，互不干扰。

---

## 10. 配置项（`ptz_config.yaml`）

| 配置项 | 默认值 | 说明 |
|--------|--------|------|
| `launcher_port` | `8080` | Launcher HTTP 端口（文档中 `{BASE}` 端口） |
| `ctrl_port` | `8081` | Isaac 内部端口，第三方无需感知 |
| `ws_port` | `8765` | WebSocket 视频流端口（Isaac Sim 进程监听） |
| `python_sh` | — | Isaac Sim Python 解释器路径 |
| `fps` | `15` | 推流帧率；同时影响 RTSP 和 WebSocket 帧率 |

---

## 11. 变更说明

接口行为以仓库内 `ptz_launcher.py`、`ptz_stream.py` 为准。

- **WebSocket 视频流**（新增）：Isaac Sim 进程内嵌 WebSocket 服务器（纯 stdlib，无需额外依赖），端口 `8765`，每帧推送 binary JPEG；Launcher 的 `/status` 接口透传 `ws_port`。
- **视频接入**：同时支持 RTSP（`:8554`）、ONVIF `GetSnapshotUri`（`:8080/onvif-snap.jpg`）、WebSocket（`:8765`）三路并行，互不干扰。
- **PTZ 控制**：统一使用 ONVIF `AbsoluteMove` / `RelativeMove`；直接 HTTP `POST /control` 仅用于内部调试。
- 如需**鉴权**，建议在 Launcher 前加反向代理（Nginx）或 API 网关统一实现。
