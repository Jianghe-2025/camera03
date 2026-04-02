#!/usr/bin/env python3
"""
PTZ 仿真 ONVIF 服务  ptz_launcher.py
======================================
始终运行的轻量 ONVIF 服务（默认端口 8080），GPU 占用接近 0%。
通过 Web UI 按需启动 / 停止 Isaac Sim 仿真进程（ptz_stream.py）。

使用方法：
    python3 ptz_launcher.py [--config ./ptz_config.yaml]

访问地址：
    http://localhost:8080/                       Web 控制面板（始终可用）
    POST http://localhost:8080/onvif/device_service  ONVIF 设备服务
    POST http://localhost:8080/onvif/media_service   ONVIF 媒体服务
    POST http://localhost:8080/onvif/ptz_service     ONVIF PTZ 服务
    GET  http://localhost:8080/onvif-snap.jpg         ONVIF 快照端点
    GET  ws://localhost:8080/ws                       WebSocket 视频流（JPEG 帧推送）
    POST http://localhost:8080/start              启动 Isaac Sim
    POST http://localhost:8080/stop               停止 Isaac Sim

架构说明：
    launcher（8080）← 代理 → Isaac Sim（8081）
    外部 ONVIF 客户端只需连接 8080，无需感知 8081。

坐标换算：
    pan_deg  = onvif_pan  × 170.0        ONVIF[-1,1] → Isaac[-170°,170°]
    tilt_deg = clamp(-60.0 × onvif_tilt - 30.0, -90°, 30°)
    zoom_x   = 1.0 + onvif_zoom × 31.0  ONVIF[0,1]  → Isaac[1×,32×]
"""

import argparse
import base64
import datetime
import io
import json
import os
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
import urllib.request
import urllib.error
import uuid as _uuid_mod
import xml.etree.ElementTree as ET
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import yaml

# ── 配置 ─────────────────────────────────────────────────────────────
script_dir = os.path.dirname(os.path.abspath(__file__))

ap = argparse.ArgumentParser(description="PTZ Launcher - ONVIF simulation server")
ap.add_argument("--config", default=os.path.join(script_dir, "ptz_config.yaml"))
args, _ = ap.parse_known_args()

with open(args.config, "r", encoding="utf-8") as f:
    cfg = yaml.safe_load(f)

LAUNCHER_PORT = cfg.get("launcher_port", 8080)
ISAAC_PORT    = cfg.get("ctrl_port",     8081)
PYTHON_SH     = cfg.get("python_sh",
                         "/home/uniubi/projects/issac/.isaac_sim_unzip/python.sh")
STREAM_SCRIPT = os.path.join(script_dir, "ptz_stream.py")
ISAAC_LOG     = os.path.join(script_dir, "isaac_stream.log")
CONFIG_PATH   = args.config

_PRESET_LIMIT = 5
_DEFAULT_PRESETS = {
    "1": {"name": "默认位",   "pan": -56.0, "tilt": 9.0,   "zoom": 1.0},
    "2": {"name": "水平正视", "pan": 0.0,   "tilt": 0.0,   "zoom": 1.0},
    "3": {"name": "左90°",   "pan": -90.0, "tilt": -45.0, "zoom": 1.0},
    "4": {"name": "右90°",   "pan": 90.0,  "tilt": -45.0, "zoom": 1.0},
    "5": {"name": "俯视",    "pan": 0.0,   "tilt": 30.0,  "zoom": 1.0},
}
_preset_lock = threading.Lock()


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _preset_token_order() -> list[str]:
    return [str(i) for i in range(1, _PRESET_LIMIT + 1)]


def _normalize_preset(token: str, item: dict | None) -> dict | None:
    if token not in _preset_token_order() or not isinstance(item, dict):
        return None
    return {
        "token": token,
        "name": str(item.get("name") or f"预置位{token}"),
        "pan": round(_clamp(float(item.get("pan", 0.0)), -170.0, 170.0), 3),
        "tilt": round(_clamp(float(item.get("tilt", -45.0)), -90.0, 30.0), 3),
        "zoom": round(_clamp(float(item.get("zoom", 1.0)), 1.0, 32.0), 3),
    }


def _load_presets_from_cfg() -> dict[str, dict]:
    raw = cfg.get("presets") or {}
    presets: dict[str, dict] = {}
    if isinstance(raw, dict):
        for token in _preset_token_order():
            norm = _normalize_preset(token, raw.get(token))
            if norm is not None:
                presets[token] = norm
    if not presets:
        for token, item in _DEFAULT_PRESETS.items():
            presets[token] = _normalize_preset(token, item)
    return presets


_presets = _load_presets_from_cfg()


def _persist_presets() -> None:
    serializable = {
        token: {
            "name": item["name"],
            "pan": item["pan"],
            "tilt": item["tilt"],
            "zoom": item["zoom"],
        }
        for token, item in sorted(_presets.items(), key=lambda pair: int(pair[0]))
    }
    cfg["presets"] = serializable
    with open(CONFIG_PATH, "w", encoding="utf-8") as f:
        yaml.safe_dump(cfg, f, allow_unicode=True, sort_keys=False)


def _list_presets() -> list[dict]:
    with _preset_lock:
        return [dict(_presets[token]) for token in _preset_token_order() if token in _presets]


def _get_preset(token: str) -> dict | None:
    with _preset_lock:
        item = _presets.get(token)
        return dict(item) if item else None


def _save_preset(token: str | None, name: str | None, ptz: dict) -> dict:
    with _preset_lock:
        use_token = token
        if use_token is None:
            for candidate in _preset_token_order():
                if candidate not in _presets:
                    use_token = candidate
                    break
            if use_token is None:
                use_token = "1"
        current = _presets.get(use_token, {})
        preset = _normalize_preset(use_token, {
            "name": name or current.get("name") or f"预置位{use_token}",
            "pan": ptz["pan"],
            "tilt": ptz["tilt"],
            "zoom": ptz["zoom"],
        })
        _presets[use_token] = preset
        _persist_presets()
        return dict(preset)


def _delete_preset(token: str) -> bool:
    with _preset_lock:
        existed = token in _presets
        if existed:
            _presets.pop(token, None)
            _persist_presets()
        return existed


def _preset_to_onvif_xml(preset: dict) -> str:
    return f"""  <tptz:Preset token="{preset['token']}">
    <tt:Name>{preset['name']}</tt:Name>
    <tt:PTZPosition>
      <tt:PanTilt x="{round(_pan_to_norm(preset['pan']), 4)}" y="{round(_tilt_to_norm(preset['tilt']), 4)}" space="http://www.onvif.org/ver10/tptz/PanTiltSpaces/PositionGenericSpace"/>
      <tt:Zoom x="{round(_zoom_to_norm(preset['zoom']), 4)}" space="http://www.onvif.org/ver10/tptz/ZoomSpaces/PositionGenericSpace"/>
    </tt:PTZPosition>
  </tptz:Preset>"""


def _soap_fault(reason: str, code: str = "ter:InvalidArgVal") -> bytes:
    return _soap_wrap(f"""<s:Fault>
  <s:Code>
    <s:Value>s:Sender</s:Value>
    <s:Subcode><s:Value>{code}</s:Value></s:Subcode>
  </s:Code>
  <s:Reason><s:Text xml:lang="zh-CN">{reason}</s:Text></s:Reason>
</s:Fault>""")

# ── Isaac Sim 子进程管理 ─────────────────────────────────────────────
_proc_lock  = threading.Lock()
_isaac_proc: subprocess.Popen | None = None
_start_time: float | None = None
_isaac_state = "stopped"   # "stopped" | "starting" | "running" | "stopping"


def _is_isaac_http_ready() -> bool:
    """检测 Isaac Sim 内部 HTTP（8081）是否已就绪。"""
    try:
        with urllib.request.urlopen(
            f"http://localhost:{ISAAC_PORT}/status", timeout=1
        ) as r:
            return r.status == 200
    except Exception:
        return False


def _watch_isaac(proc: subprocess.Popen) -> None:
    """监控 Isaac Sim 启动 / 运行状态（后台线程）。"""
    global _isaac_state
    for _ in range(300):
        if proc.poll() is not None:
            _isaac_state = "stopped"
            return
        if _is_isaac_http_ready():
            _isaac_state = "running"
            break
        time.sleep(1)
    else:
        if proc.poll() is None:
            _isaac_state = "running"
        else:
            _isaac_state = "stopped"
            return

    while True:
        if proc.poll() is not None:
            _isaac_state = "stopped"
            return
        time.sleep(2)


def start_isaac() -> dict:
    global _isaac_proc, _start_time, _isaac_state
    with _proc_lock:
        if _isaac_proc is not None and _isaac_proc.poll() is None:
            return {"ok": False, "error": "Isaac Sim 已在运行"}

        # 端口已被占用说明进程来自上一次 Launcher 会话，直接同步状态
        if _port_in_use(ISAAC_PORT):
            _isaac_state = "running"
            _start_time  = time.time()
            _isaac_proc  = None  # 无法持有旧进程句柄，置 None
            return {"ok": True, "pid": -1, "note": "Isaac Sim 已在运行（外部进程），已同步状态"}

        log_file = open(ISAAC_LOG, "w", encoding="utf-8", buffering=1)
        cmd = [
            PYTHON_SH, "-u", STREAM_SCRIPT,
            "--config", args.config,
            "--ctrl-port", str(ISAAC_PORT),
        ]
        _isaac_proc = subprocess.Popen(
            cmd,
            stdout=log_file,
            stderr=log_file,
            cwd=script_dir,
            start_new_session=True,
        )
        _start_time  = time.time()
        _isaac_state = "starting"

    threading.Thread(target=_watch_isaac, args=(_isaac_proc,), daemon=True).start()
    return {"ok": True, "pid": _isaac_proc.pid, "log": ISAAC_LOG}


def _port_free(port: int) -> bool:
    """检测 TCP 端口是否已释放。"""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.3)
        return s.connect_ex(("127.0.0.1", port)) != 0


def stop_isaac() -> dict:
    global _isaac_proc, _isaac_state
    with _proc_lock:
        if _isaac_proc is None or _isaac_proc.poll() is not None:
            _isaac_proc  = None
            _isaac_state = "stopped"
            return {"ok": False, "error": "Isaac Sim 未在运行"}

        _isaac_state = "stopping"
        pid  = _isaac_proc.pid
        proc = _isaac_proc
        try:
            pgid = os.getpgid(pid)
        except OSError:
            pgid = pid

    def _wait():
        global _isaac_proc, _isaac_state
        try:
            os.killpg(pgid, signal.SIGTERM)
        except (ProcessLookupError, OSError):
            pass

        deadline = time.time() + 15
        while time.time() < deadline:
            if _port_free(ISAAC_PORT):
                break
            time.sleep(0.5)

        try:
            os.killpg(pgid, signal.SIGKILL)
        except (ProcessLookupError, OSError):
            pass

        try:
            proc.wait(timeout=5)
        except (subprocess.TimeoutExpired, ChildProcessError):
            pass

        for port in (ISAAC_PORT,):
            for _ in range(20):
                if _port_free(port):
                    break
                time.sleep(0.5)

        global _isaac_proc, _isaac_state
        with _proc_lock:
            _isaac_proc = None
        _isaac_state = "stopped"
        print("[PTZ-Launcher] ✓ Isaac Sim 及所有子进程已完全退出，GPU 资源已释放。")

    threading.Thread(target=_wait, daemon=True, name="isaac-stopper").start()
    return {"ok": True, "stopping_pid": pid}


def get_status() -> dict:
    """返回启动器状态 + 可选的 Isaac Sim PTZ 状态。"""
    global _isaac_state
    with _proc_lock:
        proc   = _isaac_proc
        state  = _isaac_state
        up_sec = int(time.time() - _start_time) if _start_time else 0
        dead   = (proc is not None and proc.poll() is not None)

    if dead and state not in ("stopped", "stopping"):
        _isaac_state = "stopped"
        state = "stopped"

    result = {
        "isaac_state": state,
        "isaac_port":  ISAAC_PORT,
        "uptime_s":    up_sec if state == "running" else 0,
        "ptz":         None,
    }
    if state == "running":
        try:
            # 抓图/重负载时 Isaac 内 HTTP 可能 >1s 才响应；超时勿把对外状态改成 starting，
            # 否则 Web 会误以为仿真在重启（实际进程未停）。
            with urllib.request.urlopen(
                f"http://localhost:{ISAAC_PORT}/status", timeout=3
            ) as r:
                result["ptz"] = json.loads(r.read())
        except Exception:
            result["ptz"] = None
    return result




# ══════════════════════════════════════════════════════════════════════
# ONVIF 轻量 SOAP 服务器
# ══════════════════════════════════════════════════════════════════════

_ONVIF_PROFILE_TOKEN = "Profile_1"
_ONVIF_PTZ_TOKEN     = "PTZConfig_1"
_ONVIF_PTZ_NODE      = "PTZNode_1"

# 线程局部变量：记录当前请求的 SOAP 版本，让 _soap_wrap 自动匹配
# True=SOAP 1.2 (application/soap+xml), False=SOAP 1.1 (text/xml)
_soap_tls = threading.local()

# 坐标换算常量
_PAN_MIN   = -170.0
_PAN_MAX   = 170.0
_PAN_SCALE = 170.0

_TILT_MIN = -90.0
_TILT_MAX = 30.0

_ZOOM_MIN   = 1.0
_ZOOM_MAX   = 32.0
_ZOOM_SCALE = 31.0


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _norm_to_pan(norm: float) -> float:
    return _clamp(norm, -1.0, 1.0) * _PAN_SCALE


def _pan_to_norm(pan_deg: float) -> float:
    return _clamp(pan_deg, _PAN_MIN, _PAN_MAX) / _PAN_SCALE


def _norm_to_tilt(norm: float) -> float:
    norm = _clamp(norm, -1.0, 1.0)
    return _clamp(
        _TILT_MIN + (1.0 - norm) * (_TILT_MAX - _TILT_MIN) / 2.0,
        _TILT_MIN,
        _TILT_MAX,
    )


def _tilt_to_norm(tilt_deg: float) -> float:
    tilt_deg = _clamp(tilt_deg, _TILT_MIN, _TILT_MAX)
    return _clamp(
        -(2.0 * (tilt_deg - _TILT_MIN) / (_TILT_MAX - _TILT_MIN) - 1.0),
        -1.0,
        1.0,
    )


def _norm_to_zoom(norm: float) -> float:
    return _ZOOM_MIN + _clamp(norm, 0.0, 1.0) * _ZOOM_SCALE


def _zoom_to_norm(zoom_x: float) -> float:
    return (_clamp(zoom_x, _ZOOM_MIN, _ZOOM_MAX) - _ZOOM_MIN) / _ZOOM_SCALE

# RTSP 端口（来自 mediamtx 配置，默认 8554）
_RTSP_PORT = cfg.get("mediamtx", {}).get("port", 8554)


def _make_offline_jpeg() -> bytes:
    """
    生成"仿真未启动"占位 JPEG。
    优先用 Pillow；不可用时回退内嵌最小 JPEG（8×8 灰色）。
    确保快照端点永远返回 200 而不是 503，VMS/ODM 才能正常显示 NVT 模块。
    """
    try:
        from PIL import Image as _Img, ImageDraw as _Draw, ImageFont as _Font
        img  = _Img.new("RGB", (640, 360), color=(20, 20, 20))
        draw = _Draw.Draw(img)
        draw.rectangle([0, 0, 640, 360], fill=(20, 20, 40))
        draw.text((200, 155), "Simulation Offline", fill=(180, 180, 180))
        draw.text((250, 185), "请在 Web UI 启动仿真", fill=(120, 120, 120))
        buf = io.BytesIO()
        img.save(buf, "JPEG", quality=50)
        return buf.getvalue()
    except Exception:
        pass
    # 内嵌最小合法 JPEG（8×8 灰色像素，无外部依赖）
    return base64.b64decode(
        "/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAAgGBgcGBQgHBwcJCQgKDBQNDAsLDBkSEw8U"
        "HRofHh0aHBwgJC4nICIsIxwcKDcpLDAxNDQ0Hyc5PTgyPC4zNDL/wAAR"
        "CAAIAAgDASIAAhEBAxEB/8QAFgABAQEAAAAAAAAAAAAAAAAABgUE/8QAIhAA"
        "AgIDAQADAQAAAAAAAAAAAQIDBAUREiExQf/EABQBAQAAAAAAAAAAAAAAAAAA"
        "AAD/xAAUEQEAAAAAAAAAAAAAAAAAAAAA/9oADAMBAAIRAxEAPwCw1jiuNZv3"
        "fVc5bU2rxuTq9NGlfCb9ZVJfGkRQA7tezA7G+rEfFmAAAAAAAAAAAAAAAAAA"
        "AAAAAAAAAAB//9k="
    )


# 模块启动时预生成离线占位图（非阻塞，尽早缓存）
_OFFLINE_JPEG: bytes = _make_offline_jpeg()

# ══════════════════════════════════════════════════════════════════════
# WS-Discovery：局域网 ONVIF 设备发现（UDP 多播 239.255.255.250:3702）
# ══════════════════════════════════════════════════════════════════════

_WSD_MCAST_ADDR = "239.255.255.250"
_WSD_PORT       = 3702

# 稳定 UUID：基于主机名 + 端口，重启后不变，客户端可识别同一设备
_DEVICE_UUID = str(_uuid_mod.uuid5(
    _uuid_mod.NAMESPACE_DNS,
    f"onvif-ptz-cam-{socket.gethostname()}-{LAUNCHER_PORT}"
))

_WSD_SCOPES = (
    "onvif://www.onvif.org/location/country/china "
    "onvif://www.onvif.org/name/Isaac-Sim-PTZ-Camera "
    "onvif://www.onvif.org/hardware/PTZ-Camera-V4 "
    "onvif://www.onvif.org/type/video_encoder "
    "onvif://www.onvif.org/Profile/Streaming"
)


def _get_local_ips() -> list:
    """获取本机所有非 loopback IPv4 地址列表。"""
    ips: list = []
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ip = info[4][0]
            if not ip.startswith("127.") and ip not in ips:
                ips.append(ip)
    except Exception:
        pass
    if not ips:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as _s:
                _s.connect(("8.8.8.8", 80))
                ips.append(_s.getsockname()[0])
        except Exception:
            ips.append("127.0.0.1")
    return ips


def _get_sender_local_ip(remote_ip: str) -> str:
    """获取通往 remote_ip 所在网段的本地出口 IP。"""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as _s:
            _s.connect((remote_ip, 1))
            return _s.getsockname()[0]
    except Exception:
        ips = _get_local_ips()
        return ips[0] if ips else "127.0.0.1"


def _wsd_probe_match(msg_id: str, local_ip: str) -> bytes:
    """构建 WS-Discovery ProbeMatch 单播响应报文。"""
    xaddr    = f"http://{local_ip}:{LAUNCHER_PORT}/onvif/device_service"
    reply_id = str(_uuid_mod.uuid4())
    return (
        '<?xml version="1.0" encoding="UTF-8"?>'
        '<s:Envelope '
        'xmlns:s="http://www.w3.org/2003/05/soap-envelope" '
        'xmlns:wsa="http://schemas.xmlsoap.org/ws/2004/08/addressing" '
        'xmlns:wsd="http://schemas.xmlsoap.org/ws/2005/04/discovery" '
        'xmlns:dn="http://www.onvif.org/ver10/network/wsdl">'
        '<s:Header>'
        '<wsa:Action>'
        'http://schemas.xmlsoap.org/ws/2005/04/discovery/ProbeMatches'
        '</wsa:Action>'
        f'<wsa:MessageID>urn:uuid:{reply_id}</wsa:MessageID>'
        f'<wsa:RelatesTo>{msg_id}</wsa:RelatesTo>'
        '<wsa:To>'
        'http://schemas.xmlsoap.org/ws/2004/08/addressing/role/anonymous'
        '</wsa:To>'
        '</s:Header>'
        '<s:Body>'
        '<wsd:ProbeMatches>'
        '<wsd:ProbeMatch>'
        '<wsa:EndpointReference>'
        f'<wsa:Address>urn:uuid:{_DEVICE_UUID}</wsa:Address>'
        '</wsa:EndpointReference>'
        '<wsd:Types>dn:NetworkVideoTransmitter</wsd:Types>'
        f'<wsd:Scopes>{_WSD_SCOPES}</wsd:Scopes>'
        f'<wsd:XAddrs>{xaddr}</wsd:XAddrs>'
        '<wsd:MetadataVersion>1</wsd:MetadataVersion>'
        '</wsd:ProbeMatch>'
        '</wsd:ProbeMatches>'
        '</s:Body>'
        '</s:Envelope>'
    ).encode()


def _wsd_hello(sock: socket.socket) -> None:
    """服务启动时向多播组广播 Hello，主动宣告设备上线。"""
    local_ips = _get_local_ips()
    xaddrs = " ".join(
        f"http://{ip}:{LAUNCHER_PORT}/onvif/device_service" for ip in local_ips
    )
    reply_id = str(_uuid_mod.uuid4())
    msg = (
        '<?xml version="1.0" encoding="UTF-8"?>'
        '<s:Envelope '
        'xmlns:s="http://www.w3.org/2003/05/soap-envelope" '
        'xmlns:wsa="http://schemas.xmlsoap.org/ws/2004/08/addressing" '
        'xmlns:wsd="http://schemas.xmlsoap.org/ws/2005/04/discovery" '
        'xmlns:dn="http://www.onvif.org/ver10/network/wsdl">'
        '<s:Header>'
        '<wsa:Action>'
        'http://schemas.xmlsoap.org/ws/2005/04/discovery/Hello'
        '</wsa:Action>'
        f'<wsa:MessageID>urn:uuid:{reply_id}</wsa:MessageID>'
        '<wsa:To>urn:schemas-xmlsoap-org:ws:2005:04:discovery</wsa:To>'
        '</s:Header>'
        '<s:Body>'
        '<wsd:Hello>'
        '<wsa:EndpointReference>'
        f'<wsa:Address>urn:uuid:{_DEVICE_UUID}</wsa:Address>'
        '</wsa:EndpointReference>'
        '<wsd:Types>dn:NetworkVideoTransmitter</wsd:Types>'
        f'<wsd:Scopes>{_WSD_SCOPES}</wsd:Scopes>'
        f'<wsd:XAddrs>{xaddrs}</wsd:XAddrs>'
        '<wsd:MetadataVersion>1</wsd:MetadataVersion>'
        '</wsd:Hello>'
        '</s:Body>'
        '</s:Envelope>'
    ).encode()
    try:
        sock.sendto(msg, (_WSD_MCAST_ADDR, _WSD_PORT))
    except Exception:
        pass


def _wsd_listener() -> None:
    """
    WS-Discovery UDP 多播监听线程。
    监听 239.255.255.250:3702，响应局域网 ONVIF 客户端的 Probe 请求。
    失败时仅打印警告，不影响 ONVIF HTTP 服务正常运行。
    """
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except (AttributeError, OSError):
            pass  # 部分内核不支持 SO_REUSEPORT，忽略
        sock.bind(("", _WSD_PORT))

        # 加入 IPv4 多播组（所有网卡）
        mreq = struct.pack("4sL", socket.inet_aton(_WSD_MCAST_ADDR),
                           socket.INADDR_ANY)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        sock.settimeout(5.0)

        print(f"[PTZ-Launcher] WS-Discovery 已启动（{_WSD_MCAST_ADDR}:{_WSD_PORT}），"
              f"设备 UUID={_DEVICE_UUID}")

        # 启动时广播 Hello，主动宣告上线
        _wsd_hello(sock)

        while True:
            try:
                data, addr = sock.recvfrom(65536)
            except socket.timeout:
                continue
            except OSError:
                break

            try:
                text = data.decode("utf-8", errors="replace")
                # 只处理 Probe 请求
                if "Probe" not in text or "ProbeMatch" in text:
                    continue

                # 解析 MessageID 和 Types
                root   = ET.fromstring(data)
                msg_id = ""
                for el in root.iter():
                    lname = el.tag.split("}")[-1] if "}" in el.tag else el.tag
                    if lname == "MessageID" and el.text:
                        msg_id = el.text.strip()
                        break

                # 检查 Types 过滤：若指定类型中不含 NVT 则跳过
                accept = True
                for el in root.iter():
                    lname = el.tag.split("}")[-1] if "}" in el.tag else el.tag
                    if lname == "Types" and el.text and el.text.strip():
                        accept = "NetworkVideoTransmitter" in el.text
                        break

                if not accept:
                    continue

                local_ip = _get_sender_local_ip(addr[0])
                response = _wsd_probe_match(msg_id, local_ip)
                sock.sendto(response, addr)

            except Exception:
                pass

    except OSError as exc:
        print(f"[PTZ-Launcher] ⚠ WS-Discovery 绑定失败（端口 {_WSD_PORT} 可能被占用）："
              f"{exc}  局域网自动发现不可用，仍可手动填写 IP 连接。")
    except Exception as exc:
        print(f"[PTZ-Launcher] ⚠ WS-Discovery 异常退出：{exc}")


def _port_in_use(port: int) -> bool:
    """检测 TCP 端口是否已被监听。"""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.5)
        return s.connect_ex(("127.0.0.1", port)) == 0




def _soap_wrap(body_content: str) -> bytes:
    """
    将内容包裹在 SOAP Envelope 中，自动匹配当前请求的 SOAP 版本：
      - 请求用 application/soap+xml → SOAP 1.2（_soap_tls.soap12 = True）
      - 请求用 text/xml 或未指定   → SOAP 1.1（_soap_tls.soap12 = False）
    这样可以同时兼容：
      · 中文 ONVIF 工具（SOAP 1.1，text/xml）
      · .NET WCF / 现代 ONVIF 客户端（SOAP 1.2，application/soap+xml）
    """
    if getattr(_soap_tls, "soap12", False):
        s_ns = "http://www.w3.org/2003/05/soap-envelope"
    else:
        s_ns = "http://schemas.xmlsoap.org/soap/envelope/"
    return (
        f'<?xml version="1.0" encoding="UTF-8"?>'
        f'<s:Envelope '
        f'xmlns:s="{s_ns}" '
        f'xmlns:tt="http://www.onvif.org/ver10/schema" '
        f'xmlns:tds="http://www.onvif.org/ver10/device/wsdl" '
        f'xmlns:trt="http://www.onvif.org/ver10/media/wsdl" '
        f'xmlns:tptz="http://www.onvif.org/ver20/ptz/wsdl" '
        f'xmlns:timg="http://www.onvif.org/ver20/imaging/wsdl">'
        f'<s:Body>{body_content}</s:Body>'
        f'</s:Envelope>'
    ).encode()


def _parse_soap_action(data: bytes) -> tuple[str, ET.Element | None]:
    """解析 SOAP Body，返回 (action_name, body_child_element)。"""
    try:
        root = ET.fromstring(data)
        body = (
            root.find("{http://www.w3.org/2003/05/soap-envelope}Body") or
            root.find("{http://schemas.xmlsoap.org/soap/envelope/}Body")
        )
        if body is not None and len(body):
            el  = body[0]
            tag = el.tag
            action = tag.split("}")[1] if "}" in tag else tag
            return action, el
    except Exception:
        pass
    return "", None


def _find_attr(el: ET.Element, local_name: str, attr: str) -> str | None:
    """忽略命名空间，按本地名查找元素属性值。"""
    if el is None:
        return None
    for child in el.iter():
        lname = child.tag.split("}")[-1] if "}" in child.tag else child.tag
        if lname == local_name and attr in child.attrib:
            return child.attrib[attr]
    return None


def _find_text(el: ET.Element, *local_names: str) -> str | None:
    """忽略命名空间，按本地名查找元素文本内容。"""
    if el is None:
        return None
    for child in el.iter():
        lname = child.tag.split("}")[-1] if "}" in child.tag else child.tag
        if lname in local_names and child.text:
            return child.text.strip()
    return None


def _get_ptz_from_isaac() -> dict:
    """从 Isaac Sim 获取当前 PTZ 状态，失败时返回默认值。"""
    try:
        with urllib.request.urlopen(
            f"http://localhost:{ISAAC_PORT}/status", timeout=1
        ) as r:
            return json.loads(r.read())
    except Exception:
        return {"pan": -56.0, "tilt": 9.0, "zoom": 1.0}


def _set_ptz_to_isaac(pan_deg: float, tilt_deg: float, zoom_x: float) -> None:
    """向 Isaac Sim 发送 PTZ 绝对位置命令。"""
    payload = json.dumps({
        "pan":  round(_clamp(pan_deg, -170.0, 170.0), 3),
        "tilt": round(_clamp(tilt_deg, -90.0, 30.0), 3),
        "zoom": round(_clamp(zoom_x, 1.0, 32.0), 3),
    }).encode()
    try:
        req = urllib.request.Request(
            f"http://localhost:{ISAAC_PORT}/control",
            data=payload,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        urllib.request.urlopen(req, timeout=2)
    except Exception:
        pass


def _onvif_host(handler: "BaseHTTPRequestHandler") -> str:
    """从 Host 请求头提取地址（含端口），用于 ONVIF XAddr 拼接。"""
    return handler.headers.get("Host", f"localhost:{LAUNCHER_PORT}")


# ── Device Service ────────────────────────────────────────────────────

def _onvif_device(action: str, el: ET.Element | None, host: str) -> bytes:
    base = f"http://{host}"

    if action == "GetSystemDateAndTime":
        now = datetime.datetime.utcnow()
        return _soap_wrap(f"""<tds:GetSystemDateAndTimeResponse>
  <tds:SystemDateAndTime>
    <tt:DateTimeType>Manual</tt:DateTimeType>
    <tt:DaylightSavings>false</tt:DaylightSavings>
    <tt:TimeZone><tt:TZ>UTC</tt:TZ></tt:TimeZone>
    <tt:UTCDateTime>
      <tt:Time>
        <tt:Hour>{now.hour}</tt:Hour>
        <tt:Minute>{now.minute}</tt:Minute>
        <tt:Second>{now.second}</tt:Second>
      </tt:Time>
      <tt:Date>
        <tt:Year>{now.year}</tt:Year>
        <tt:Month>{now.month}</tt:Month>
        <tt:Day>{now.day}</tt:Day>
      </tt:Date>
    </tt:UTCDateTime>
  </tds:SystemDateAndTime>
</tds:GetSystemDateAndTimeResponse>""")

    if action == "GetCapabilities":
        return _soap_wrap(f"""<tds:GetCapabilitiesResponse>
  <tds:Capabilities>
    <tt:Device>
      <tt:XAddr>{base}/onvif/device_service</tt:XAddr>
      <tt:Network>
        <tt:IPFilter>false</tt:IPFilter>
        <tt:ZeroConfiguration>false</tt:ZeroConfiguration>
        <tt:IPVersion6>false</tt:IPVersion6>
        <tt:DynDNS>false</tt:DynDNS>
      </tt:Network>
      <tt:System>
        <tt:DiscoveryResolve>true</tt:DiscoveryResolve>
        <tt:DiscoveryBye>false</tt:DiscoveryBye>
        <tt:RemoteDiscovery>false</tt:RemoteDiscovery>
        <tt:SystemBackup>false</tt:SystemBackup>
        <tt:SystemLogging>false</tt:SystemLogging>
        <tt:FirmwareUpgrade>false</tt:FirmwareUpgrade>
      </tt:System>
      <tt:IO>
        <tt:InputConnectors>0</tt:InputConnectors>
        <tt:RelayOutputs>0</tt:RelayOutputs>
      </tt:IO>
      <tt:Security>
        <tt:TLS1.2>false</tt:TLS1.2>
        <tt:OnboardKeyGeneration>false</tt:OnboardKeyGeneration>
        <tt:AccessPolicyConfig>false</tt:AccessPolicyConfig>
        <tt:X.509Token>false</tt:X.509Token>
        <tt:SAMLToken>false</tt:SAMLToken>
        <tt:KerberosToken>false</tt:KerberosToken>
        <tt:RELToken>false</tt:RELToken>
      </tt:Security>
    </tt:Device>
    <tt:Media>
      <tt:XAddr>{base}/onvif/media_service</tt:XAddr>
      <tt:StreamingCapabilities>
        <tt:RTPMulticast>false</tt:RTPMulticast>
        <tt:RTP_TCP>true</tt:RTP_TCP>
        <tt:RTP_RTSP_TCP>true</tt:RTP_RTSP_TCP>
      </tt:StreamingCapabilities>
    </tt:Media>
    <tt:PTZ>
      <tt:XAddr>{base}/onvif/ptz_service</tt:XAddr>
    </tt:PTZ>
    <tt:Imaging>
      <tt:XAddr>{base}/onvif/imaging_service</tt:XAddr>
    </tt:Imaging>
  </tds:Capabilities>
</tds:GetCapabilitiesResponse>""")

    if action == "GetServices":
        return _soap_wrap(f"""<tds:GetServicesResponse>
  <tds:Service>
    <tds:Namespace>http://www.onvif.org/ver10/device/wsdl</tds:Namespace>
    <tds:XAddr>{base}/onvif/device_service</tds:XAddr>
    <tds:Version><tt:Major>2</tt:Major><tt:Minor>6</tt:Minor></tds:Version>
  </tds:Service>
  <tds:Service>
    <tds:Namespace>http://www.onvif.org/ver10/media/wsdl</tds:Namespace>
    <tds:XAddr>{base}/onvif/media_service</tds:XAddr>
    <tds:Version><tt:Major>2</tt:Major><tt:Minor>6</tt:Minor></tds:Version>
  </tds:Service>
  <tds:Service>
    <tds:Namespace>http://www.onvif.org/ver20/ptz/wsdl</tds:Namespace>
    <tds:XAddr>{base}/onvif/ptz_service</tds:XAddr>
    <tds:Version><tt:Major>2</tt:Major><tt:Minor>6</tt:Minor></tds:Version>
  </tds:Service>
  <tds:Service>
    <tds:Namespace>http://www.onvif.org/ver20/imaging/wsdl</tds:Namespace>
    <tds:XAddr>{base}/onvif/imaging_service</tds:XAddr>
    <tds:Version><tt:Major>2</tt:Major><tt:Minor>0</tt:Minor></tds:Version>
  </tds:Service>
</tds:GetServicesResponse>""")

    if action == "GetDeviceInformation":
        return _soap_wrap("""<tds:GetDeviceInformationResponse>
  <tds:Manufacturer>Isaac-Sim</tds:Manufacturer>
  <tds:Model>PTZ-Camera-V4</tds:Model>
  <tds:FirmwareVersion>4.0.0</tds:FirmwareVersion>
  <tds:SerialNumber>CAM-03-SIM</tds:SerialNumber>
  <tds:HardwareId>1.0</tds:HardwareId>
</tds:GetDeviceInformationResponse>""")

    if action == "GetScopes":
        return _soap_wrap("""<tds:GetScopesResponse>
  <tds:Scopes>
    <tt:ScopeDef>Fixed</tt:ScopeDef>
    <tt:ScopeItem>onvif://www.onvif.org/location/country/china</tt:ScopeItem>
  </tds:Scopes>
  <tds:Scopes>
    <tt:ScopeDef>Fixed</tt:ScopeDef>
    <tt:ScopeItem>onvif://www.onvif.org/name/Isaac-Sim-PTZ-Camera</tt:ScopeItem>
  </tds:Scopes>
  <tds:Scopes>
    <tt:ScopeDef>Fixed</tt:ScopeDef>
    <tt:ScopeItem>onvif://www.onvif.org/hardware/PTZ-Camera-V4</tt:ScopeItem>
  </tds:Scopes>
  <tds:Scopes>
    <tt:ScopeDef>Fixed</tt:ScopeDef>
    <tt:ScopeItem>onvif://www.onvif.org/type/video_encoder</tt:ScopeItem>
  </tds:Scopes>
  <tds:Scopes>
    <tt:ScopeDef>Fixed</tt:ScopeDef>
    <tt:ScopeItem>onvif://www.onvif.org/Profile/Streaming</tt:ScopeItem>
  </tds:Scopes>
</tds:GetScopesResponse>""")

    if action == "GetNetworkInterfaces":
        local_ips = _get_local_ips()
        ifaces_xml = ""
        for i, ip in enumerate(local_ips):
            ifaces_xml += f"""  <tds:NetworkInterfaces token="eth{i}">
    <tt:Enabled>true</tt:Enabled>
    <tt:Info>
      <tt:Name>eth{i}</tt:Name>
      <tt:HwAddress>00:00:00:00:00:0{i}</tt:HwAddress>
      <tt:MTU>1500</tt:MTU>
    </tt:Info>
    <tt:IPv4>
      <tt:Enabled>true</tt:Enabled>
      <tt:Config>
        <tt:Manual>
          <tt:Address>{ip}</tt:Address>
          <tt:PrefixLength>24</tt:PrefixLength>
        </tt:Manual>
        <tt:DHCP>true</tt:DHCP>
      </tt:Config>
    </tt:IPv4>
  </tds:NetworkInterfaces>
"""
        return _soap_wrap(
            f"<tds:GetNetworkInterfacesResponse>\n{ifaces_xml}"
            "</tds:GetNetworkInterfacesResponse>"
        )

    return _soap_wrap(f"<tds:{action}Response/>")


# ── Media Service ─────────────────────────────────────────────────────

def _onvif_media(action: str, el: ET.Element | None, host: str) -> bytes:
    base     = f"http://{host}"
    snap_url = f"{base}/onvif-snap.jpg"

    if action in ("GetProfiles", "GetProfile"):
        # GetProfile（单数）返回 GetProfileResponse；GetProfiles（复数）返回 GetProfilesResponse
        resp_tag  = "GetProfileResponse"  if action == "GetProfile"  else "GetProfilesResponse"
        item_tag  = "Profile"             if action == "GetProfile"  else "Profiles"
        profile_xml = f"""<trt:{item_tag} token="{_ONVIF_PROFILE_TOKEN}" fixed="true">
    <tt:Name>PTZ-Sim-H264-Profile</tt:Name>
    <tt:VideoSourceConfiguration token="VSC_1">
      <tt:Name>VideoSource_1</tt:Name>
      <tt:UseCount>1</tt:UseCount>
      <tt:SourceToken>VST_1</tt:SourceToken>
      <tt:Bounds x="0" y="0" width="1920" height="1080"/>
    </tt:VideoSourceConfiguration>
    <tt:VideoEncoderConfiguration token="VEC_1">
      <tt:Name>H264-1080p-25fps</tt:Name>
      <tt:UseCount>1</tt:UseCount>
      <tt:Encoding>H264</tt:Encoding>
      <tt:Resolution>
        <tt:Width>1920</tt:Width>
        <tt:Height>1080</tt:Height>
      </tt:Resolution>
      <tt:Quality>80</tt:Quality>
      <tt:RateControl>
        <tt:FrameRateLimit>25</tt:FrameRateLimit>
        <tt:EncodingInterval>1</tt:EncodingInterval>
        <tt:BitrateLimit>4000</tt:BitrateLimit>
      </tt:RateControl>
      <tt:H264>
        <tt:GovLength>50</tt:GovLength>
        <tt:H264Profile>High</tt:H264Profile>
      </tt:H264>
      <tt:Multicast>
        <tt:Address>
          <tt:Type>IPv4</tt:Type>
          <tt:IPv4Address>0.0.0.0</tt:IPv4Address>
        </tt:Address>
        <tt:Port>0</tt:Port>
        <tt:TTL>0</tt:TTL>
        <tt:AutoStart>false</tt:AutoStart>
      </tt:Multicast>
      <tt:SessionTimeout>PT60S</tt:SessionTimeout>
    </tt:VideoEncoderConfiguration>
    <tt:PTZConfiguration token="{_ONVIF_PTZ_TOKEN}">
      <tt:Name>PTZConfig</tt:Name>
      <tt:UseCount>1</tt:UseCount>
      <tt:NodeToken>{_ONVIF_PTZ_NODE}</tt:NodeToken>
      <tt:DefaultAbsolutePantTiltPositionSpace>
        http://www.onvif.org/ver10/tptz/PanTiltSpaces/PositionGenericSpace
      </tt:DefaultAbsolutePantTiltPositionSpace>
      <tt:DefaultAbsoluteZoomPositionSpace>
        http://www.onvif.org/ver10/tptz/ZoomSpaces/PositionGenericSpace
      </tt:DefaultAbsoluteZoomPositionSpace>
      <tt:DefaultRelativePanTiltTranslationSpace>
        http://www.onvif.org/ver10/tptz/PanTiltSpaces/TranslationGenericSpace
      </tt:DefaultRelativePanTiltTranslationSpace>
      <tt:PanTiltLimits>
        <tt:Range>
          <tt:URI>http://www.onvif.org/ver10/tptz/PanTiltSpaces/PositionGenericSpace</tt:URI>
          <tt:XRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:XRange>
          <tt:YRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:YRange>
        </tt:Range>
      </tt:PanTiltLimits>
      <tt:ZoomLimits>
        <tt:Range>
          <tt:URI>http://www.onvif.org/ver10/tptz/ZoomSpaces/PositionGenericSpace</tt:URI>
          <tt:XRange><tt:Min>0</tt:Min><tt:Max>1</tt:Max></tt:XRange>
        </tt:Range>
      </tt:ZoomLimits>
    </tt:PTZConfiguration>
  </trt:{item_tag}>"""
        return _soap_wrap(f"<trt:{resp_tag}>{profile_xml}</trt:{resp_tag}>")

    if action == "GetStreamUri":
        rtsp_host = host.split(":")[0]
        rtsp_url  = f"rtsp://{rtsp_host}:{_RTSP_PORT}/ptz_cam"
        return _soap_wrap(f"""<trt:GetStreamUriResponse>
  <trt:MediaUri>
    <tt:Uri>{rtsp_url}</tt:Uri>
    <tt:InvalidAfterConnect>false</tt:InvalidAfterConnect>
    <tt:InvalidAfterReboot>false</tt:InvalidAfterReboot>
    <tt:Timeout>PT0S</tt:Timeout>
  </trt:MediaUri>
</trt:GetStreamUriResponse>""")

    if action == "GetSnapshotUri":
        return _soap_wrap(f"""<trt:GetSnapshotUriResponse>
  <trt:MediaUri>
    <tt:Uri>{snap_url}</tt:Uri>
    <tt:InvalidAfterConnect>false</tt:InvalidAfterConnect>
    <tt:InvalidAfterReboot>false</tt:InvalidAfterReboot>
    <tt:Timeout>PT30S</tt:Timeout>
  </trt:MediaUri>
</trt:GetSnapshotUriResponse>""")

    if action == "GetServiceCapabilities":
        return _soap_wrap("""<trt:GetServiceCapabilitiesResponse>
  <trt:Capabilities SnapshotUri="true" Rotation="false"
    VideoSourceMode="false" OSD="false"/>
</trt:GetServiceCapabilitiesResponse>""")

    if action == "GetVideoSources":
        return _soap_wrap("""<trt:GetVideoSourcesResponse>
  <trt:VideoSources token="VST_1">
    <tt:Framerate>25</tt:Framerate>
    <tt:Resolution>
      <tt:Width>1920</tt:Width>
      <tt:Height>1080</tt:Height>
    </tt:Resolution>
    <tt:Imaging/>
  </trt:VideoSources>
</trt:GetVideoSourcesResponse>""")

    if action in ("GetVideoSourceConfigurations", "GetVideoSourceConfiguration"):
        # VideoSourceConfiguration（软件绑定层）与 VideoSources（物理源）是不同类型
        resp_tag = ("GetVideoSourceConfigurationResponse"
                    if action == "GetVideoSourceConfiguration"
                    else "GetVideoSourceConfigurationsResponse")
        item_tag = ("Configuration"
                    if action == "GetVideoSourceConfiguration"
                    else "Configurations")
        return _soap_wrap(f"""<trt:{resp_tag}>
  <trt:{item_tag} token="VSC_1">
    <tt:Name>VideoSource_1</tt:Name>
    <tt:UseCount>1</tt:UseCount>
    <tt:SourceToken>VST_1</tt:SourceToken>
    <tt:Bounds x="0" y="0" width="1920" height="1080"/>
  </trt:{item_tag}>
</trt:{resp_tag}>""")

    if action in ("GetVideoEncoderConfigurations", "GetVideoEncoderConfiguration"):
        return _soap_wrap("""<trt:GetVideoEncoderConfigurationsResponse>
  <trt:Configurations token="VEC_1">
    <tt:Name>H264-1080p</tt:Name>
    <tt:UseCount>1</tt:UseCount>
    <tt:Encoding>H264</tt:Encoding>
    <tt:Resolution>
      <tt:Width>1920</tt:Width>
      <tt:Height>1080</tt:Height>
    </tt:Resolution>
    <tt:Quality>80</tt:Quality>
    <tt:RateControl>
      <tt:FrameRateLimit>25</tt:FrameRateLimit>
      <tt:EncodingInterval>1</tt:EncodingInterval>
      <tt:BitrateLimit>4000</tt:BitrateLimit>
    </tt:RateControl>
    <tt:H264>
      <tt:GovLength>50</tt:GovLength>
      <tt:H264Profile>High</tt:H264Profile>
    </tt:H264>
    <tt:Multicast>
      <tt:Address>
        <tt:Type>IPv4</tt:Type>
        <tt:IPv4Address>0.0.0.0</tt:IPv4Address>
      </tt:Address>
      <tt:Port>0</tt:Port>
      <tt:TTL>0</tt:TTL>
      <tt:AutoStart>false</tt:AutoStart>
    </tt:Multicast>
    <tt:SessionTimeout>PT60S</tt:SessionTimeout>
  </trt:Configurations>
</trt:GetVideoEncoderConfigurationsResponse>""")

    if action in ("GetVideoEncoderConfigurationOptions",
                  "GetCompatibleVideoEncoderConfigurationOptions"):
        return _soap_wrap("""<trt:GetVideoEncoderConfigurationOptionsResponse>
  <trt:Options>
    <tt:QualityRange>
      <tt:Min>1</tt:Min>
      <tt:Max>100</tt:Max>
    </tt:QualityRange>
    <tt:H264>
      <tt:ResolutionsAvailable>
        <tt:Width>1920</tt:Width>
        <tt:Height>1080</tt:Height>
      </tt:ResolutionsAvailable>
      <tt:ResolutionsAvailable>
        <tt:Width>1280</tt:Width>
        <tt:Height>720</tt:Height>
      </tt:ResolutionsAvailable>
      <tt:GovLengthRange>
        <tt:Min>1</tt:Min>
        <tt:Max>300</tt:Max>
      </tt:GovLengthRange>
      <tt:FrameRateRange>
        <tt:Min>1</tt:Min>
        <tt:Max>30</tt:Max>
      </tt:FrameRateRange>
      <tt:EncodingIntervalRange>
        <tt:Min>1</tt:Min>
        <tt:Max>10</tt:Max>
      </tt:EncodingIntervalRange>
      <tt:H264ProfilesSupported>Baseline</tt:H264ProfilesSupported>
      <tt:H264ProfilesSupported>Main</tt:H264ProfilesSupported>
      <tt:H264ProfilesSupported>High</tt:H264ProfilesSupported>
    </tt:H264>
    <tt:Extension/>
  </trt:Options>
</trt:GetVideoEncoderConfigurationOptionsResponse>""")

    if action == "GetCompatibleVideoEncoderConfigurations":
        return _soap_wrap("""<trt:GetCompatibleVideoEncoderConfigurationsResponse>
  <trt:Configurations token="VEC_1">
    <tt:Name>H264-1080p</tt:Name>
    <tt:UseCount>1</tt:UseCount>
    <tt:Encoding>H264</tt:Encoding>
    <tt:Resolution>
      <tt:Width>1920</tt:Width>
      <tt:Height>1080</tt:Height>
    </tt:Resolution>
    <tt:Quality>80</tt:Quality>
    <tt:RateControl>
      <tt:FrameRateLimit>25</tt:FrameRateLimit>
      <tt:EncodingInterval>1</tt:EncodingInterval>
      <tt:BitrateLimit>4000</tt:BitrateLimit>
    </tt:RateControl>
    <tt:H264>
      <tt:GovLength>50</tt:GovLength>
      <tt:H264Profile>High</tt:H264Profile>
    </tt:H264>
  </trt:Configurations>
</trt:GetCompatibleVideoEncoderConfigurationsResponse>""")

    return _soap_wrap(f"<trt:{action}Response/>")


# ── PTZ Service ───────────────────────────────────────────────────────

def _onvif_ptz(action: str, el: ET.Element | None) -> bytes:

    if action == "AbsoluteMove":
        pan_x  = float(_find_attr(el, "PanTilt", "x") or 0.0)
        pan_y  = float(_find_attr(el, "PanTilt", "y") or 0.0)
        zoom_z = float(_find_attr(el, "Zoom",    "x") or 0.0)
        _set_ptz_to_isaac(
            _norm_to_pan(pan_x),
            _norm_to_tilt(pan_y),
            _norm_to_zoom(zoom_z),
        )
        return _soap_wrap("<tptz:AbsoluteMoveResponse/>")

    if action == "RelativeMove":
        pan_dx  = float(_find_attr(el, "PanTilt", "x") or 0.0)
        pan_dy  = float(_find_attr(el, "PanTilt", "y") or 0.0)
        zoom_dz = float(_find_attr(el, "Zoom",    "x") or 0.0)
        cur = _get_ptz_from_isaac()
        _set_ptz_to_isaac(
            cur["pan"]  + _clamp(pan_dx, -1.0, 1.0) * _PAN_SCALE,
            cur["tilt"] - _clamp(pan_dy, -1.0, 1.0) * 60.0,
            cur["zoom"] + _clamp(zoom_dz, -1.0, 1.0) * _ZOOM_SCALE,
        )
        return _soap_wrap("<tptz:RelativeMoveResponse/>")

    if action == "ContinuousMove":
        # 简化实现：按速度比例做单步位移（无持续运动定时器）
        pan_v  = float(_find_attr(el, "PanTilt", "x") or 0.0)
        tilt_v = float(_find_attr(el, "PanTilt", "y") or 0.0)
        zoom_v = float(_find_attr(el, "Zoom",    "x") or 0.0)
        cur = _get_ptz_from_isaac()
        _set_ptz_to_isaac(
            cur["pan"]  + pan_v  * 5.0,
            cur["tilt"] + tilt_v * 5.0,
            cur["zoom"] + zoom_v * 1.0,
        )
        return _soap_wrap("<tptz:ContinuousMoveResponse/>")

    if action == "Stop":
        return _soap_wrap("<tptz:StopResponse/>")

    if action == "GotoPreset":
        token = _find_text(el, "PresetToken") or "1"
        preset = _get_preset(token)
        if preset is None:
            return _soap_fault(f"PresetToken 不存在: {token}")
        _set_ptz_to_isaac(preset["pan"], preset["tilt"], preset["zoom"])
        return _soap_wrap("<tptz:GotoPresetResponse/>")

    if action == "SetPreset":
        token = _find_text(el, "PresetToken")
        name = _find_text(el, "PresetName")
        if token and token not in _preset_token_order():
            return _soap_fault(f"PresetToken 超出范围: {token}")
        preset = _save_preset(token, name, _get_ptz_from_isaac())
        return _soap_wrap(f"""<tptz:SetPresetResponse>
  <tptz:PresetToken>{preset["token"]}</tptz:PresetToken>
</tptz:SetPresetResponse>""")

    if action == "RemovePreset":
        token = _find_text(el, "PresetToken") or ""
        if token not in _preset_token_order():
            return _soap_fault(f"PresetToken 超出范围: {token}")
        if not _delete_preset(token):
            return _soap_fault(f"PresetToken 不存在: {token}")
        return _soap_wrap("<tptz:RemovePresetResponse/>")

    if action == "GetPresets":
        preset_xml = "\n".join(_preset_to_onvif_xml(item) for item in _list_presets())
        return _soap_wrap(f"""<tptz:GetPresetsResponse>
{preset_xml}
</tptz:GetPresetsResponse>""")

    if action == "GetStatus":
        st         = _get_ptz_from_isaac()
        onvif_pan  = round(_pan_to_norm(st["pan"]), 4)
        onvif_tilt = round(_tilt_to_norm(st["tilt"]), 4)
        onvif_zoom = round(_zoom_to_norm(st["zoom"]), 4)
        return _soap_wrap(f"""<tptz:GetStatusResponse>
  <tptz:PTZStatus>
    <tt:Position>
      <tt:PanTilt x="{onvif_pan}" y="{onvif_tilt}"
        space="http://www.onvif.org/ver10/tptz/PanTiltSpaces/PositionGenericSpace"/>
      <tt:Zoom x="{onvif_zoom}"
        space="http://www.onvif.org/ver10/tptz/ZoomSpaces/PositionGenericSpace"/>
    </tt:Position>
    <tt:MoveStatus>
      <tt:PanTilt>IDLE</tt:PanTilt>
      <tt:Zoom>IDLE</tt:Zoom>
    </tt:MoveStatus>
  </tptz:PTZStatus>
</tptz:GetStatusResponse>""")

    if action == "GetNodes":
        return _soap_wrap(f"""<tptz:GetNodesResponse>
  <tptz:PTZNode token="{_ONVIF_PTZ_NODE}" FixedHomePosition="false">
    <tt:Name>PTZ Node</tt:Name>
    <tt:SupportedPTZSpaces>
      <tt:AbsolutePanTiltPositionSpace>
        <tt:URI>http://www.onvif.org/ver10/tptz/PanTiltSpaces/PositionGenericSpace</tt:URI>
        <tt:XRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:XRange>
        <tt:YRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:YRange>
      </tt:AbsolutePanTiltPositionSpace>
      <tt:AbsoluteZoomPositionSpace>
        <tt:URI>http://www.onvif.org/ver10/tptz/ZoomSpaces/PositionGenericSpace</tt:URI>
        <tt:XRange><tt:Min>0</tt:Min><tt:Max>1</tt:Max></tt:XRange>
      </tt:AbsoluteZoomPositionSpace>
      <tt:RelativePanTiltTranslationSpace>
        <tt:URI>http://www.onvif.org/ver10/tptz/PanTiltSpaces/TranslationGenericSpace</tt:URI>
        <tt:XRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:XRange>
        <tt:YRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:YRange>
      </tt:RelativePanTiltTranslationSpace>
    </tt:SupportedPTZSpaces>
    <tt:MaximumNumberOfPresets>5</tt:MaximumNumberOfPresets>
    <tt:HomeSupported>false</tt:HomeSupported>
  </tptz:PTZNode>
</tptz:GetNodesResponse>""")

    if action in ("GetConfigurations", "GetConfiguration"):
        return _soap_wrap(f"""<tptz:GetConfigurationsResponse>
  <tptz:PTZConfiguration token="{_ONVIF_PTZ_TOKEN}">
    <tt:Name>PTZConfig</tt:Name>
    <tt:UseCount>1</tt:UseCount>
    <tt:NodeToken>{_ONVIF_PTZ_NODE}</tt:NodeToken>
    <tt:DefaultPTZSpeed>
      <tt:PanTilt x="0.5" y="0.5"
        space="http://www.onvif.org/ver10/tptz/PanTiltSpaces/GenericSpeedSpace"/>
      <tt:Zoom x="0.5"
        space="http://www.onvif.org/ver10/tptz/ZoomSpaces/ZoomGenericSpeedSpace"/>
    </tt:DefaultPTZSpeed>
    <tt:DefaultPTZTimeout>PT5S</tt:DefaultPTZTimeout>
    <tt:PanTiltLimits>
      <tt:Range>
        <tt:URI>http://www.onvif.org/ver10/tptz/PanTiltSpaces/PositionGenericSpace</tt:URI>
        <tt:XRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:XRange>
        <tt:YRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:YRange>
      </tt:Range>
    </tt:PanTiltLimits>
    <tt:ZoomLimits>
      <tt:Range>
        <tt:URI>http://www.onvif.org/ver10/tptz/ZoomSpaces/PositionGenericSpace</tt:URI>
        <tt:XRange><tt:Min>0</tt:Min><tt:Max>1</tt:Max></tt:XRange>
      </tt:Range>
    </tt:ZoomLimits>
  </tptz:PTZConfiguration>
</tptz:GetConfigurationsResponse>""")

    return _soap_wrap(f"<tptz:{action}Response/>")


# ── Imaging Service ───────────────────────────────────────────────────
# NVT 设备的必要服务：ONVIF 客户端通过它确认设备是完整的 NVT 摄像头，
# 并据此显示 "Live Video" 模块。即使参数是只读存根也必须能正常响应。

def _onvif_imaging(action: str, el: ET.Element | None) -> bytes:

    if action == "GetServiceCapabilities":
        return _soap_wrap("""<timg:GetServiceCapabilitiesResponse>
  <timg:Capabilities ImageStabilization="false" Presets="false" AdaptablePreset="false"/>
</timg:GetServiceCapabilitiesResponse>""")

    if action == "GetImagingSettings":
        return _soap_wrap("""<timg:GetImagingSettingsResponse>
  <timg:ImagingSettings>
    <tt:BacklightCompensation>
      <tt:Mode>OFF</tt:Mode>
      <tt:Level>0</tt:Level>
    </tt:BacklightCompensation>
    <tt:Brightness>50</tt:Brightness>
    <tt:ColorSaturation>50</tt:ColorSaturation>
    <tt:Contrast>50</tt:Contrast>
    <tt:Exposure>
      <tt:Mode>AUTO</tt:Mode>
      <tt:MinExposureTime>1</tt:MinExposureTime>
      <tt:MaxExposureTime>100000</tt:MaxExposureTime>
      <tt:MinGain>0</tt:MinGain>
      <tt:MaxGain>100</tt:MaxGain>
    </tt:Exposure>
    <tt:Focus>
      <tt:AutoFocusMode>AUTO</tt:AutoFocusMode>
    </tt:Focus>
    <tt:IrCutFilter>AUTO</tt:IrCutFilter>
    <tt:Sharpness>50</tt:Sharpness>
    <tt:WideDynamicRange>
      <tt:Mode>OFF</tt:Mode>
      <tt:Level>0</tt:Level>
    </tt:WideDynamicRange>
    <tt:WhiteBalance>
      <tt:Mode>AUTO</tt:Mode>
    </tt:WhiteBalance>
  </timg:ImagingSettings>
</timg:GetImagingSettingsResponse>""")

    if action == "GetOptions":
        return _soap_wrap("""<timg:GetOptionsResponse>
  <timg:ImagingOptions>
    <tt:BacklightCompensation>
      <tt:Mode>OFF</tt:Mode>
      <tt:Mode>ON</tt:Mode>
    </tt:BacklightCompensation>
    <tt:Brightness>
      <tt:Min>0</tt:Min>
      <tt:Max>100</tt:Max>
    </tt:Brightness>
    <tt:ColorSaturation>
      <tt:Min>0</tt:Min>
      <tt:Max>100</tt:Max>
    </tt:ColorSaturation>
    <tt:Contrast>
      <tt:Min>0</tt:Min>
      <tt:Max>100</tt:Max>
    </tt:Contrast>
    <tt:Sharpness>
      <tt:Min>0</tt:Min>
      <tt:Max>100</tt:Max>
    </tt:Sharpness>
  </timg:ImagingOptions>
</timg:GetOptionsResponse>""")

    if action == "GetMoveOptions":
        return _soap_wrap("""<timg:GetMoveOptionsResponse>
  <timg:MoveOptions/>
</timg:GetMoveOptionsResponse>""")

    if action == "GetStatus":
        return _soap_wrap("""<timg:GetStatusResponse>
  <timg:Status>
    <tt:FocusStatus20>
      <tt:Position>0</tt:Position>
      <tt:MoveStatus>IDLE</tt:MoveStatus>
    </tt:FocusStatus20>
  </timg:Status>
</timg:GetStatusResponse>""")

    return _soap_wrap(f"<timg:{action}Response/>")


# ══════════════════════════════════════════════════════════════════════
# HTTP Handler
# ══════════════════════════════════════════════════════════════════════

class _Handler(BaseHTTPRequestHandler):

    def log_message(self, *_):
        pass

    def _cors(self):
        self.send_header("Access-Control-Allow-Origin",  "*")
        self.send_header("Access-Control-Allow-Methods", "GET,POST,DELETE,OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type,SOAPAction")

    def _json(self, data: dict, code: int = 200):
        body = json.dumps(data, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self._cors()
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        self.send_response(204)
        self._cors()
        self.end_headers()

    # ── GET ──────────────────────────────────────────────────────────
    def do_GET(self):
        path = self.path.split("?")[0]

        if path in ("/", "/index.html"):
            html_path = os.path.join(script_dir, "ptz_web_control.html")
            if not os.path.isfile(html_path):
                self.send_response(404); self.end_headers(); return
            data = open(html_path, "rb").read()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self._cors()
            self.end_headers()
            self.wfile.write(data)
            return

        if path == "/status":
            self._json(get_status())
            return

        if path == "/presets":
            self._json({"items": _list_presets(), "capacity": _PRESET_LIMIT})
            return

        if path == "/log":
            if os.path.isfile(ISAAC_LOG):
                with open(ISAAC_LOG, "rb") as f:
                    size = os.path.getsize(ISAAC_LOG)
                    f.seek(max(0, size - 8192))
                    data = f.read()
                self.send_response(200)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self._cors()
                self.end_headers()
                self.wfile.write(data)
            else:
                self.send_response(404); self.end_headers()
            return

        # ONVIF 快照端点（优先实时快照，失败时回退缓存帧）
        if path == "/onvif-snap.jpg":
            self._snap_handle()
            return

        # 场景状态代理（Web UI 用）
        if path == "/scene/state":
            self._proxy_once(f"http://localhost:{ISAAC_PORT}/scene/state")
            return

        if path == "/roam/state":
            self._proxy_once(f"http://localhost:{ISAAC_PORT}/roam/state")
            return

        self.send_response(404); self.end_headers()

    # ── POST ─────────────────────────────────────────────────────────
    def do_POST(self):
        path = self.path.split("?")[0]

        if path == "/start":
            self._json(start_isaac())
            return

        if path == "/stop":
            self._json(stop_isaac())
            return

        if path.startswith("/presets/"):
            self._preset_post(path)
            return

        # 透明代理 POST 到 Isaac Sim 的 /control、/scene/*、漫游与成像（Web UI 用）
        if path in (
            "/control",
            "/scene/gondola",
            "/scene/workers",
            "/scene/qiping",
            "/roam/position",
            "/imaging/settings",
        ):
            length = int(self.headers.get("Content-Length", 0))
            body   = self.rfile.read(length)
            if _isaac_state != "running" and not _port_in_use(ISAAC_PORT):
                self._json({"ok": False, "error": "Isaac Sim 未就绪"}, 503)
                return
            self._proxy_post(path, body)
            return

        # ONVIF SOAP 服务端点
        if path == "/onvif/device_service":
            self._onvif_handle("device")
            return

        if path == "/onvif/media_service":
            self._onvif_handle("media")
            return

        if path == "/onvif/ptz_service":
            self._onvif_handle("ptz")
            return

        if path == "/onvif/imaging_service":
            self._onvif_handle("imaging")
            return

        # 外部 ONVIF 设备 SOAP/快照代理端点（Web UI 外部设备模式使用）
        if path == "/onvif-proxy":
            self._onvif_proxy_handle()
            return

        self.send_response(404); self.end_headers()

    def do_DELETE(self):
        path = self.path.split("?")[0]
        if path.startswith("/presets/"):
            parts = [p for p in path.split("/") if p]
            if len(parts) == 2 and parts[0] == "presets" and parts[1] in _preset_token_order():
                if not _delete_preset(parts[1]):
                    self._json({"ok": False, "error": "预置位不存在"}, 404)
                    return
                self._json({"ok": True, "token": parts[1], "items": _list_presets()})
                return
        self.send_response(404); self.end_headers()

    # ── ONVIF 代理（外部设备 SOAP/快照转发）────────────────────────────
    def _onvif_proxy_handle(self) -> None:
        """
        POST /onvif-proxy
        Body JSON:
          {
            "mode":     "soap" | "snapshot",
            "url":      "http://target/onvif/...",
            "soap":     "<SOAP XML>",          # mode=soap
            "action":   "SOAPAction",           # mode=soap, optional
            "username": "...",                  # optional
            "password": "..."                   # optional
          }
        安全约束：
          - 仅允许 http / https scheme
          - 不允许转发到本机 127.0.0.1/localhost，避免 SSRF 反射攻击
          - 超时 10s
        """
        import urllib.request as _urllib_req
        import urllib.error  as _urllib_err

        try:
            length = int(self.headers.get("Content-Length", 0))
            raw    = self.rfile.read(length)
            req    = json.loads(raw)
        except Exception as e:
            self._json({"ok": False, "error": f"请求解析失败: {e}"}, 400)
            return

        mode = req.get("mode", "soap")
        url  = req.get("url", "").strip()

        # ── 安全校验 ────────────────────────────────────────────────
        try:
            parsed = urllib.parse.urlparse(url)
        except Exception:
            self._json({"ok": False, "error": "无效 URL"}, 400)
            return

        if parsed.scheme not in ("http", "https"):
            self._json({"ok": False, "error": "仅支持 http/https"}, 400)
            return

        # 禁止 SSRF 到本机
        blocked_hosts = {"localhost", "127.0.0.1", "0.0.0.0", "::1"}
        if parsed.hostname in blocked_hosts:
            self._json({"ok": False, "error": "不允许代理到本机地址"}, 403)
            return

        username = req.get("username", "")
        password = req.get("password", "")

        headers = {}
        if username:
            import base64 as _b64
            cred = _b64.b64encode(f"{username}:{password}".encode()).decode()
            headers["Authorization"] = f"Basic {cred}"

        try:
            if mode == "soap":
                soap_body = req.get("soap", "").encode()
                action    = req.get("action", "")
                headers["Content-Type"] = "application/soap+xml; charset=utf-8"
                if action:
                    headers["SOAPAction"] = action

                http_req = _urllib_req.Request(url, data=soap_body, headers=headers, method="POST")
                with _urllib_req.urlopen(http_req, timeout=10) as r:
                    resp_data = r.read()
                    resp_ct   = r.headers.get("Content-Type", "text/xml")

                try:
                    self.send_response(200)
                    self.send_header("Content-Type",   resp_ct)
                    self.send_header("Content-Length", str(len(resp_data)))
                    self._cors()
                    self.end_headers()
                    self.wfile.write(resp_data)
                except Exception:
                    pass

            elif mode == "snapshot":
                http_req = _urllib_req.Request(url, headers=headers, method="GET")
                with _urllib_req.urlopen(http_req, timeout=10) as r:
                    resp_data = r.read()
                    resp_ct   = r.headers.get("Content-Type", "image/jpeg")

                try:
                    self.send_response(200)
                    self.send_header("Content-Type",   resp_ct)
                    self.send_header("Content-Length", str(len(resp_data)))
                    self.send_header("Cache-Control",  "no-cache, no-store")
                    self._cors()
                    self.end_headers()
                    self.wfile.write(resp_data)
                except Exception:
                    pass

            else:
                self._json({"ok": False, "error": f"不支持的模式: {mode}"}, 400)

        except _urllib_err.HTTPError as e:
            err_body = e.read()
            try:
                self.send_response(e.code)
                self.send_header("Content-Type",   "text/plain")
                self.send_header("Content-Length", str(len(err_body)))
                self._cors()
                self.end_headers()
                self.wfile.write(err_body)
            except Exception:
                pass
        except Exception as e:
            self._json({"ok": False, "error": f"代理请求失败: {e}"}, 502)

    # ── ONVIF 分发 ───────────────────────────────────────────────────
    def _onvif_handle(self, svc: str) -> None:
        length = int(self.headers.get("Content-Length", 0))
        body   = self.rfile.read(length) if length else b""

        # 检测请求 SOAP 版本，同步到线程局部变量供 _soap_wrap 使用
        req_ct = self.headers.get("Content-Type", "")
        _soap_tls.soap12 = "application/soap+xml" in req_ct

        action, el = _parse_soap_action(body)
        host = _onvif_host(self)

        # 调试日志：打印每一次 ONVIF 请求，方便排查 NVT 空白问题
        soap_ver = "SOAP1.2" if _soap_tls.soap12 else "SOAP1.1"
        client_ip = self.client_address[0]
        print(f"[ONVIF] {client_ip} → {svc}.{action}  [{soap_ver}]  CT={req_ct[:40] if req_ct else '(none)'}", flush=True)

        if svc == "device":
            resp = _onvif_device(action, el, host)
        elif svc == "media":
            resp = _onvif_media(action, el, host)
        elif svc == "imaging":
            resp = _onvif_imaging(action, el)
        else:
            resp = _onvif_ptz(action, el)

        # Content-Type 与请求 SOAP 版本保持一致：
        #   SOAP 1.2 客户端（.NET WCF 等） → application/soap+xml
        #   SOAP 1.1 客户端（中文 ONVIF 工具等） → text/xml
        resp_ct = ("application/soap+xml; charset=utf-8"
                   if _soap_tls.soap12 else "text/xml; charset=utf-8")
        print(f"[ONVIF] → 响应 {svc}.{action}  {len(resp)}B  CT={resp_ct}", flush=True)
        self.send_response(200)
        self.send_header("Content-Type",   resp_ct)
        self.send_header("Content-Length", str(len(resp)))
        self._cors()
        self.end_headers()
        self.wfile.write(resp)

    # ── 代理工具 ─────────────────────────────────────────────────────
    def _preset_post(self, path: str) -> None:
        parts = [p for p in path.split("/") if p]
        if len(parts) < 2 or parts[0] != "presets":
            self.send_response(404); self.end_headers(); return
        token = parts[1]
        if token not in _preset_token_order():
            self._json({"ok": False, "error": "token 超出范围"}, 400)
            return

        if len(parts) == 3 and parts[2] == "goto":
            preset = _get_preset(token)
            if preset is None:
                self._json({"ok": False, "error": "预置位不存在"}, 404)
                return
            _set_ptz_to_isaac(preset["pan"], preset["tilt"], preset["zoom"])
            self._json({"ok": True, "item": preset})
            return

        if len(parts) != 2:
            self.send_response(404); self.end_headers(); return

        length = int(self.headers.get("Content-Length", 0))
        req = {}
        if length:
            try:
                req = json.loads(self.rfile.read(length))
            except Exception:
                self._json({"ok": False, "error": "JSON 无效"}, 400)
                return
        ptz = req.get("ptz") if isinstance(req.get("ptz"), dict) else _get_ptz_from_isaac()
        try:
            ptz_norm = {
                "pan": float(ptz["pan"]),
                "tilt": float(ptz["tilt"]),
                "zoom": float(ptz["zoom"]),
            }
        except Exception:
            self._json({"ok": False, "error": "ptz 参数无效"}, 400)
            return
        preset = _save_preset(token, req.get("name"), ptz_norm)
        self._json({"ok": True, "item": preset, "items": _list_presets()})

    def _proxy_post(self, path: str, body: bytes) -> None:
        try:
            req = urllib.request.Request(
                f"http://localhost:{ISAAC_PORT}{path}",
                data=body,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with urllib.request.urlopen(req, timeout=30) as r:
                resp = r.read()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self._cors()
            self.end_headers()
            self.wfile.write(resp)
        except Exception as e:
            self._json({"ok": False, "error": str(e)}, 502)

    def _snap_handle(self) -> None:
        """
        GET /onvif-snap.jpg —— ONVIF 标准快照端点（GetSnapshotUri 返回此 URL）。

        实现：从 Isaac Sim 内部 HTTP（8081/snapshot.jpg）拉取最新渲染帧 JPEG。
        回退：仿真未运行或拉取失败时返回 _OFFLINE_JPEG 离线占位图，确保永远 200。
        """
        jpeg = None

        # ── 从 ptz_stream 内部快照端点拉取实时帧 ────────────────────
        if _port_in_use(ISAAC_PORT):
            try:
                with urllib.request.urlopen(
                    f"http://localhost:{ISAAC_PORT}/snapshot.jpg", timeout=3
                ) as r:
                    data = r.read()
                if data and len(data) > 100:
                    jpeg = data
                else:
                    print(f"[PTZ-Snap] /snapshot.jpg 返回空数据（len={len(data) if data else 0}）")
            except urllib.error.HTTPError as e:
                print(f"[PTZ-Snap] /snapshot.jpg HTTP {e.code}：仿真帧尚未就绪")
            except Exception as e:
                print(f"[PTZ-Snap] /snapshot.jpg 拉取失败：{e}")
        else:
            print("[PTZ-Snap] Isaac Sim 未运行，返回离线占位图")

        # ── 回退：离线占位图（确保 ONVIF 端点永远 200）──────────────
        if jpeg is None:
            jpeg = _OFFLINE_JPEG

        try:
            self.send_response(200)
            self.send_header("Content-Type",   "image/jpeg")
            self.send_header("Content-Length", str(len(jpeg)))
            self.send_header("Cache-Control",  "no-cache, no-store")
            self._cors()
            self.end_headers()
            self.wfile.write(jpeg)
        except Exception:
            pass

    def _proxy_once(self, url: str) -> None:
        try:
            with urllib.request.urlopen(url, timeout=2) as r:
                data = r.read()
                ct   = r.headers.get("Content-Type", "application/octet-stream")
            self.send_response(200)
            self.send_header("Content-Type",   ct)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control",  "no-cache, no-store")
            self._cors()
            self.end_headers()
            self.wfile.write(data)
        except Exception:
            try:
                self.send_response(503); self.end_headers()
            except Exception:
                pass



# ── 主函数 ───────────────────────────────────────────────────────────
def _sync_isaac_state_on_startup() -> None:
    """Launcher 启动时探测 Isaac Sim 是否已在运行，同步内部状态。"""
    global _isaac_state, _start_time
    if _port_in_use(ISAAC_PORT):
        if _is_isaac_http_ready():
            _isaac_state = "running"
            _start_time  = time.time()
            print(f"[PTZ-Launcher] 检测到 Isaac Sim 已运行（端口 {ISAAC_PORT}），状态同步为 running")
        else:
            # 端口占用但 HTTP 还没就绪，标记为 starting
            _isaac_state = "starting"
            _start_time  = time.time()
            print(f"[PTZ-Launcher] 检测到 Isaac Sim 正在启动（端口 {ISAAC_PORT} 占用中）")


def main():
    # Launcher 重启后同步 Isaac Sim 实际运行状态
    _sync_isaac_state_on_startup()

    # 启动 WS-Discovery UDP 多播监听线程（局域网 ONVIF 自动发现）
    threading.Thread(target=_wsd_listener, daemon=True, name="wsd-discovery").start()

    srv = ThreadingHTTPServer(("0.0.0.0", LAUNCHER_PORT), _Handler)
    local_ips = _get_local_ips()
    lan_ip    = local_ips[0] if local_ips else "localhost"
    print(f"[PTZ-Launcher] ONVIF 仿真服务运行中 → http://localhost:{LAUNCHER_PORT}/")
    print(f"[PTZ-Launcher] ONVIF 设备服务（局域网）：http://{lan_ip}:{LAUNCHER_PORT}/onvif/device_service")
    print(f"[PTZ-Launcher] ONVIF 快照端点：http://{lan_ip}:{LAUNCHER_PORT}/onvif-snap.jpg")
    print(f"[PTZ-Launcher] RTSP 视频流（VLC 拉流）：rtsp://{lan_ip}:{_RTSP_PORT}/ptz_cam")
    print(f"[PTZ-Launcher] WS-Discovery：{_WSD_MCAST_ADDR}:{_WSD_PORT}（设备 UUID={_DEVICE_UUID}）")
    print(f"[PTZ-Launcher] Isaac Sim 内部端口：{ISAAC_PORT}（按需启动）")
    print(f"[PTZ-Launcher] 在 Web 界面点击 '启动仿真' 启动 Isaac Sim")
    print(f"[PTZ-Launcher] 按 Ctrl-C 停止")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n[PTZ-Launcher] 正在停止...")
        stop_isaac()
        time.sleep(2)


if __name__ == "__main__":
    main()
