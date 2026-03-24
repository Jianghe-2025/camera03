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
    tilt_deg = clamp(onvif_tilt × 90.0, -90°, 30°)
    zoom_x   = 1.0 + onvif_zoom × 31.0  ONVIF[0,1]  → Isaac[1×,32×]
"""

import argparse
import base64
import datetime
import hashlib
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
import xml.etree.ElementTree as ET
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

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

        log_file = open(ISAAC_LOG, "w", encoding="utf-8", buffering=1)
        cmd = [
            PYTHON_SH, STREAM_SCRIPT,
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
            with urllib.request.urlopen(
                f"http://localhost:{ISAAC_PORT}/status", timeout=1
            ) as r:
                result["ptz"] = json.loads(r.read())
        except Exception:
            result["isaac_state"] = "starting"
    return result


# ══════════════════════════════════════════════════════════════════════
# WebSocket 视频流
# ══════════════════════════════════════════════════════════════════════

# 帧缓存：后台线程写，WS handler 线程读
_ws_cache      = {"jpeg": None, "frame_id": 0}
_ws_cache_lock = threading.Lock()

# 目标推流帧率（从 config 读取，默认 25）
_WS_FPS = 25


def _ws_frame_fetcher() -> None:
    """后台线程：以 _WS_FPS 速率轮询 Isaac Sim 快照并缓存最新帧。"""
    interval = 1.0 / max(1, _WS_FPS)
    while True:
        t0 = time.monotonic()
        # 只要 Isaac Sim 端口可达就尝试拉帧（不依赖 _isaac_state 是否同步）
        if _isaac_state == "running" or _port_in_use(ISAAC_PORT):
            try:
                with urllib.request.urlopen(
                    f"http://localhost:{ISAAC_PORT}/snapshot.jpg", timeout=2
                ) as r:
                    data = r.read()
                if data and len(data) > 1000:
                    with _ws_cache_lock:
                        _ws_cache["jpeg"]     = data
                        _ws_cache["frame_id"] += 1
            except Exception:
                pass
        elapsed = time.monotonic() - t0
        time.sleep(max(0.0, interval - elapsed))


def _ws_send(sock_file, data: bytes) -> None:
    """向 WebSocket 客户端发送一帧 binary frame（RFC 6455，服务端无掩码）。"""
    length = len(data)
    if length <= 125:
        header = bytes([0x82, length])
    elif length <= 65535:
        header = struct.pack("!BBH", 0x82, 126, length)
    else:
        header = struct.pack("!BBQ", 0x82, 127, length)
    sock_file.write(header + data)
    sock_file.flush()


# ══════════════════════════════════════════════════════════════════════
# ONVIF 轻量 SOAP 服务器
# ══════════════════════════════════════════════════════════════════════

_ONVIF_PROFILE_TOKEN = "Profile_1"
_ONVIF_PTZ_TOKEN     = "PTZConfig_1"
_ONVIF_PTZ_NODE      = "PTZNode_1"

# 坐标换算常量
_PAN_SCALE  = 170.0   # ONVIF [-1,1] → Isaac [-170°, 170°]
_TILT_SCALE = 90.0    # ONVIF [-1,1] → Isaac [-90°,  +30°] (clamp 在 _set_ptz_to_isaac 内)
_ZOOM_SCALE = 31.0    # ONVIF [0,1]  → Isaac [1×,    32×]


# ══════════════════════════════════════════════════════════════════════
# WebSocket FLV+H.264 广播器
# ══════════════════════════════════════════════════════════════════════

def _port_in_use(port: int) -> bool:
    """检测 TCP 端口是否已被监听。"""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.5)
        return s.connect_ex(("127.0.0.1", port)) == 0


_flv_init_buf: bytes | None = None       # FLV 文件头 + metadata + AVC seq header
_flv_init_ready = threading.Event()      # init_buf 就绪信号
_flv_clients: dict = {}                  # cid → {sock_file, lock, state}
_flv_clients_lock = threading.Lock()
_flv_cid_counter = 0


def _flv_tag_size(data: bytes, pos: int) -> int:
    """返回 pos 处 FLV tag 的总字节数（含末尾 PreviousTagSize 4 字节），不足则返回 0。"""
    if pos + 11 > len(data):
        return 0
    data_size = (data[pos + 1] << 16) | (data[pos + 2] << 8) | data[pos + 3]
    total = 11 + data_size + 4
    return total if pos + total <= len(data) else 0


def _flv_is_keyframe(tag: bytes) -> bool:
    """判断是否为 H.264 视频关键帧 tag（FrameType=1, CodecID=7）。"""
    return len(tag) >= 12 and tag[0] == 0x09 and (tag[11] >> 4) == 1


def _flv_is_avc_seqhdr(tag: bytes) -> bool:
    """判断是否为 AVC Sequence Header（解码必须先收到此包）。"""
    return (len(tag) >= 13 and tag[0] == 0x09
            and tag[11] == 0x17 and tag[12] == 0x00)


def _flv_broadcast(tag: bytes, is_kf: bool) -> None:
    """将一个 FLV tag 广播给所有已连接客户端；新客户端等待首个关键帧后才开始接收。"""
    dead = []
    with _flv_clients_lock:
        items = list(_flv_clients.items())
    for cid, c in items:
        if c["state"] == "waiting_kf":
            if not is_kf:
                continue
            c["state"] = "live"
        with c["lock"]:
            try:
                _ws_send(c["sock_file"], tag)
            except Exception:
                dead.append(cid)
    if dead:
        with _flv_clients_lock:
            for cid in dead:
                _flv_clients.pop(cid, None)


def _flv_broadcaster() -> None:
    """后台线程：Isaac Sim 运行时启动 ffmpeg 读 RTSP，解析 FLV tag 并广播给客户端。"""
    global _flv_init_buf, _flv_cid_counter

    FLV_FILE_HDR_LEN = 13   # 9 字节 FLV 文件头 + 4 字节 PreviousTagSize0

    while True:
        if _isaac_state != "running" or not _port_in_use(8554):
            time.sleep(2)
            continue

        time.sleep(2)   # 等待 MediaMTX 稳定

        cmd = [
            "ffmpeg", "-loglevel", "quiet",
            "-rtsp_transport", "tcp",
            "-i", "rtsp://localhost:8554/ptz_cam",
            "-c:v", "copy",
            "-an",
            "-f", "flv", "pipe:1",
        ]
        proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0
        )

        buf        = bytearray()
        init_done  = False
        local_init = bytearray()

        try:
            while True:
                chunk = proc.stdout.read(65536)
                if not chunk:
                    break
                buf.extend(chunk)

                # 收集 FLV 文件头（固定 13 字节）
                if not local_init and len(buf) >= FLV_FILE_HDR_LEN:
                    if buf[:3] != b'FLV':
                        break
                    local_init.extend(buf[:FLV_FILE_HDR_LEN])
                    buf = buf[FLV_FILE_HDR_LEN:]

                if not local_init:
                    continue

                # 逐 tag 解析
                pos = 0
                while True:
                    sz = _flv_tag_size(bytes(buf), pos)
                    if sz == 0:
                        break
                    tag = bytes(buf[pos: pos + sz])
                    pos += sz

                    if not init_done:
                        local_init.extend(tag)
                        if _flv_is_avc_seqhdr(tag):
                            # 收到 AVC 序列头，初始化包完整
                            _flv_init_buf = bytes(local_init)
                            _flv_init_ready.set()
                            init_done = True
                    else:
                        _flv_broadcast(tag, _flv_is_keyframe(tag))

                buf = buf[pos:]

        except Exception:
            pass
        finally:
            try:
                proc.terminate()
                proc.wait(timeout=3)
            except Exception:
                pass

        # 流中断：清空状态，断开所有客户端
        _flv_init_ready.clear()
        _flv_init_buf = None
        with _flv_clients_lock:
            _flv_clients.clear()
        time.sleep(5)


def _soap_wrap(body_content: str) -> bytes:
    """将内容包裹在 SOAP Envelope 中。"""
    return (
        '<?xml version="1.0" encoding="UTF-8"?>'
        '<s:Envelope '
        'xmlns:s="http://www.w3.org/2003/05/soap-envelope" '
        'xmlns:tt="http://www.onvif.org/ver10/schema" '
        'xmlns:tds="http://www.onvif.org/ver10/device/wsdl" '
        'xmlns:trt="http://www.onvif.org/ver10/media/wsdl" '
        'xmlns:tptz="http://www.onvif.org/ver20/ptz/wsdl">'
        '<s:Body>' + body_content + '</s:Body>'
        '</s:Envelope>'
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
        return {"pan": 0.0, "tilt": -45.0, "zoom": 1.0}


def _set_ptz_to_isaac(pan_deg: float, tilt_deg: float, zoom_x: float) -> None:
    """向 Isaac Sim 发送 PTZ 绝对位置命令。"""
    payload = json.dumps({
        "pan":  round(max(-170.0, min(170.0, pan_deg)),  3),
        "tilt": round(max(-90.0,  min(30.0,  tilt_deg)), 3),
        "zoom": round(max(1.0,    min(32.0,  zoom_x)),   3),
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
    <tt:Media xaddr="{base}/onvif/media_service">
      <tt:StreamingCapabilities>
        <tt:RTPMulticast>false</tt:RTPMulticast>
        <tt:RTP_TCP>false</tt:RTP_TCP>
        <tt:RTP_RTSP_TCP>false</tt:RTP_RTSP_TCP>
      </tt:StreamingCapabilities>
    </tt:Media>
    <tt:PTZ xaddr="{base}/onvif/ptz_service"/>
  </tds:Capabilities>
</tds:GetCapabilitiesResponse>""")

    if action == "GetServices":
        return _soap_wrap(f"""<tds:GetServicesResponse>
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
</tds:GetServicesResponse>""")

    if action == "GetDeviceInformation":
        return _soap_wrap("""<tds:GetDeviceInformationResponse>
  <tds:Manufacturer>Isaac-Sim</tds:Manufacturer>
  <tds:Model>PTZ-Camera-V4</tds:Model>
  <tds:FirmwareVersion>4.0.0</tds:FirmwareVersion>
  <tds:SerialNumber>CAM-03-SIM</tds:SerialNumber>
  <tds:HardwareId>1.0</tds:HardwareId>
</tds:GetDeviceInformationResponse>""")

    return _soap_wrap(f"<tds:{action}Response/>")


# ── Media Service ─────────────────────────────────────────────────────

def _onvif_media(action: str, el: ET.Element | None, host: str) -> bytes:
    base     = f"http://{host}"
    snap_url = f"{base}/onvif-snap.jpg"

    if action in ("GetProfiles", "GetProfile"):
        return _soap_wrap(f"""<trt:GetProfilesResponse>
  <trt:Profiles token="{_ONVIF_PROFILE_TOKEN}" fixed="true">
    <tt:Name>PTZ Simulation Profile</tt:Name>
    <tt:VideoSourceConfiguration token="VSC_1">
      <tt:Name>VideoSource</tt:Name>
      <tt:UseCount>1</tt:UseCount>
      <tt:SourceToken>VST_1</tt:SourceToken>
      <tt:Bounds x="0" y="0" width="1920" height="1080"/>
    </tt:VideoSourceConfiguration>
    <tt:VideoEncoderConfiguration token="VEC_1">
      <tt:Name>JPEG</tt:Name>
      <tt:UseCount>1</tt:UseCount>
      <tt:Encoding>H264</tt:Encoding>
      <tt:Resolution><tt:Width>1920</tt:Width><tt:Height>1080</tt:Height></tt:Resolution>
      <tt:Quality>80</tt:Quality>
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
  </trt:Profiles>
</trt:GetProfilesResponse>""")

    if action == "GetStreamUri":
        rtsp_host = host.split(":")[0]
        rtsp_url  = f"rtsp://{rtsp_host}:8554/ptz_cam"
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

    return _soap_wrap(f"<trt:{action}Response/>")


# ── PTZ Service ───────────────────────────────────────────────────────

def _onvif_ptz(action: str, el: ET.Element | None) -> bytes:

    if action == "AbsoluteMove":
        pan_x  = float(_find_attr(el, "PanTilt", "x") or 0.0)
        pan_y  = float(_find_attr(el, "PanTilt", "y") or 0.0)
        zoom_z = float(_find_attr(el, "Zoom",    "x") or 0.0)
        _set_ptz_to_isaac(
            pan_x  * _PAN_SCALE,
            pan_y  * _TILT_SCALE,
            1.0 + zoom_z * _ZOOM_SCALE,
        )
        return _soap_wrap("<tptz:AbsoluteMoveResponse/>")

    if action == "RelativeMove":
        pan_dx  = float(_find_attr(el, "PanTilt", "x") or 0.0)
        pan_dy  = float(_find_attr(el, "PanTilt", "y") or 0.0)
        zoom_dz = float(_find_attr(el, "Zoom",    "x") or 0.0)
        cur = _get_ptz_from_isaac()
        _set_ptz_to_isaac(
            cur["pan"]  + pan_dx  * _PAN_SCALE,
            cur["tilt"] + pan_dy  * _TILT_SCALE,
            cur["zoom"] + zoom_dz * _ZOOM_SCALE,
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
        presets = {
            "1": (0.0,    -45.0, 1.0),
            "2": (0.0,      0.0, 1.0),
            "3": (90.0,  -45.0, 1.0),
            "4": (-90.0, -45.0, 1.0),
        }
        if token in presets:
            _set_ptz_to_isaac(*presets[token])
        return _soap_wrap("<tptz:GotoPresetResponse/>")

    if action == "GetPresets":
        return _soap_wrap(f"""<tptz:GetPresetsResponse>
  <tptz:Preset token="1">
    <tt:Name>正前方</tt:Name>
    <tt:PTZPosition>
      <tt:PanTilt x="0" y="-0.5" space="http://www.onvif.org/ver10/tptz/PanTiltSpaces/PositionGenericSpace"/>
      <tt:Zoom x="0" space="http://www.onvif.org/ver10/tptz/ZoomSpaces/PositionGenericSpace"/>
    </tt:PTZPosition>
  </tptz:Preset>
  <tptz:Preset token="2">
    <tt:Name>水平正视</tt:Name>
    <tt:PTZPosition>
      <tt:PanTilt x="0" y="0" space="http://www.onvif.org/ver10/tptz/PanTiltSpaces/PositionGenericSpace"/>
      <tt:Zoom x="0" space="http://www.onvif.org/ver10/tptz/ZoomSpaces/PositionGenericSpace"/>
    </tt:PTZPosition>
  </tptz:Preset>
  <tptz:Preset token="3">
    <tt:Name>右侧90°</tt:Name>
    <tt:PTZPosition>
      <tt:PanTilt x="0.529" y="-0.5" space="http://www.onvif.org/ver10/tptz/PanTiltSpaces/PositionGenericSpace"/>
      <tt:Zoom x="0" space="http://www.onvif.org/ver10/tptz/ZoomSpaces/PositionGenericSpace"/>
    </tt:PTZPosition>
  </tptz:Preset>
  <tptz:Preset token="4">
    <tt:Name>左侧90°</tt:Name>
    <tt:PTZPosition>
      <tt:PanTilt x="-0.529" y="-0.5" space="http://www.onvif.org/ver10/tptz/PanTiltSpaces/PositionGenericSpace"/>
      <tt:Zoom x="0" space="http://www.onvif.org/ver10/tptz/ZoomSpaces/PositionGenericSpace"/>
    </tt:PTZPosition>
  </tptz:Preset>
</tptz:GetPresetsResponse>""")

    if action == "GetStatus":
        st         = _get_ptz_from_isaac()
        onvif_pan  = round(st["pan"]  / _PAN_SCALE,  4)
        onvif_tilt = round(st["tilt"] / _TILT_SCALE, 4)
        onvif_zoom = round((st["zoom"] - 1.0) / _ZOOM_SCALE, 4)
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
    <tt:MaximumNumberOfPresets>16</tt:MaximumNumberOfPresets>
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


# ══════════════════════════════════════════════════════════════════════
# HTTP Handler
# ══════════════════════════════════════════════════════════════════════

class _Handler(BaseHTTPRequestHandler):

    def log_message(self, *_):
        pass

    def _cors(self):
        self.send_header("Access-Control-Allow-Origin",  "*")
        self.send_header("Access-Control-Allow-Methods", "GET,POST,OPTIONS")
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

        # WebSocket JPEG 帧流（轻量，浏览器自定义解码）
        if path == "/ws":
            self._ws_handle()
            return

        # WebSocket FLV+H.264 流（兼容 flv.js / jessibuca 等标准播放器）
        if path == "/ws-flv":
            self._ws_flv_handle()
            return

        # 场景状态代理（Web UI 用）
        if path == "/scene/state":
            self._proxy_once(f"http://localhost:{ISAAC_PORT}/scene/state")
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

        # 透明代理 POST 到 Isaac Sim 的 /control 和 /scene/* 端点（Web UI 用）
        if path in ("/control", "/scene/gondola", "/scene/workers"):
            length = int(self.headers.get("Content-Length", 0))
            body   = self.rfile.read(length)
            if _isaac_state != "running":
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

        self.send_response(404); self.end_headers()

    # ── ONVIF 分发 ───────────────────────────────────────────────────
    def _onvif_handle(self, svc: str) -> None:
        length = int(self.headers.get("Content-Length", 0))
        body   = self.rfile.read(length) if length else b""
        action, el = _parse_soap_action(body)
        host = _onvif_host(self)

        if svc == "device":
            resp = _onvif_device(action, el, host)
        elif svc == "media":
            resp = _onvif_media(action, el, host)
        else:
            resp = _onvif_ptz(action, el)

        self.send_response(200)
        self.send_header("Content-Type",   "application/soap+xml; charset=utf-8")
        self.send_header("Content-Length", str(len(resp)))
        self._cors()
        self.end_headers()
        self.wfile.write(resp)

    # ── 代理工具 ─────────────────────────────────────────────────────
    def _proxy_post(self, path: str, body: bytes) -> None:
        try:
            req = urllib.request.Request(
                f"http://localhost:{ISAAC_PORT}{path}",
                data=body,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with urllib.request.urlopen(req, timeout=2) as r:
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
        返回最新 JPEG 快照：
          1. 优先向 Isaac Sim 实时拉取（timeout=3s）
          2. 失败时回退到 _ws_cache 里的最近缓存帧
          3. 两者均无时返回 503
        """
        jpeg = None

        # ① 实时拉取
        try:
            with urllib.request.urlopen(
                f"http://localhost:{ISAAC_PORT}/snapshot.jpg", timeout=3
            ) as r:
                data = r.read()
            if data and len(data) > 1000:   # 非空且合理大小
                jpeg = data
        except Exception:
            pass

        # ② 回退缓存帧
        if jpeg is None:
            with _ws_cache_lock:
                jpeg = _ws_cache.get("jpeg")

        if jpeg is None:
            try:
                self.send_response(503)
                self.send_header("Content-Type", "text/plain")
                self._cors()
                self.end_headers()
                self.wfile.write(b"Isaac Sim not ready")
            except Exception:
                pass
            return

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

    # ── WebSocket 升级 & 推流 ────────────────────────────────────────
    def _ws_handle(self) -> None:
        """完成 WebSocket 握手，持续推送 JPEG 帧直到客户端断开。"""
        ws_key = self.headers.get("Sec-WebSocket-Key", "")
        if not ws_key:
            self.send_response(400); self.end_headers()
            return

        # RFC 6455 握手
        magic   = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
        accept  = base64.b64encode(
            hashlib.sha1((ws_key + magic).encode()).digest()
        ).decode()

        self.send_response(101, "Switching Protocols")
        self.send_header("Upgrade",              "websocket")
        self.send_header("Connection",           "Upgrade")
        self.send_header("Sec-WebSocket-Accept", accept)
        self.end_headers()

        last_fid = -1
        sock_file = self.connection.makefile("wb", buffering=0)
        try:
            while True:
                with _ws_cache_lock:
                    fid  = _ws_cache["frame_id"]
                    jpeg = _ws_cache["jpeg"]
                if fid == last_fid or jpeg is None:
                    time.sleep(0.015)
                    continue
                last_fid = fid
                _ws_send(sock_file, jpeg)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            try:
                sock_file.close()
            except Exception:
                pass

    # ── WebSocket FLV+H.264 推流 ─────────────────────────────────────
    def _ws_flv_handle(self) -> None:
        """完成 WebSocket 握手，向客户端推送 FLV+H.264 流（兼容 flv.js / jessibuca）。"""
        global _flv_cid_counter

        ws_key = self.headers.get("Sec-WebSocket-Key", "")
        if not ws_key:
            self.send_response(400); self.end_headers()
            return

        magic  = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
        accept = base64.b64encode(
            hashlib.sha1((ws_key + magic).encode()).digest()
        ).decode()

        self.send_response(101, "Switching Protocols")
        self.send_header("Upgrade",              "websocket")
        self.send_header("Connection",           "Upgrade")
        self.send_header("Sec-WebSocket-Accept", accept)
        self.end_headers()

        sock_file = self.connection.makefile("wb", buffering=0)
        lock = threading.Lock()

        # 等待 FLV 初始化包就绪（最多 20 秒）
        if not _flv_init_ready.wait(timeout=20) or _flv_init_buf is None:
            try: sock_file.close()
            except Exception: pass
            return

        # 先发送初始化包（FLV 文件头 + metadata + AVC 序列头）
        try:
            _ws_send(sock_file, _flv_init_buf)
        except Exception:
            try: sock_file.close()
            except Exception: pass
            return

        # 注册为活跃客户端，等下一个关键帧后开始接收实时数据
        with _flv_clients_lock:
            _flv_cid_counter += 1
            cid = _flv_cid_counter
            _flv_clients[cid] = {"sock_file": sock_file, "lock": lock, "state": "waiting_kf"}

        try:
            while True:
                with _flv_clients_lock:
                    if cid not in _flv_clients:
                        break
                time.sleep(1)
        finally:
            with _flv_clients_lock:
                _flv_clients.pop(cid, None)
            try: sock_file.close()
            except Exception: pass


# ── 主函数 ───────────────────────────────────────────────────────────
def main():
    global _WS_FPS
    _WS_FPS = cfg.get("fps", 25)

    # 启动 WebSocket JPEG 帧拉取后台线程
    threading.Thread(target=_ws_frame_fetcher, daemon=True, name="ws-fetcher").start()
    # 启动 WebSocket FLV+H.264 广播器线程
    threading.Thread(target=_flv_broadcaster, daemon=True, name="flv-broadcaster").start()

    srv = ThreadingHTTPServer(("0.0.0.0", LAUNCHER_PORT), _Handler)
    print(f"[PTZ-Launcher] ONVIF 仿真服务运行中 → http://localhost:{LAUNCHER_PORT}/")
    print(f"[PTZ-Launcher] ONVIF 设备服务：http://localhost:{LAUNCHER_PORT}/onvif/device_service")
    print(f"[PTZ-Launcher] ONVIF 快照端点：http://localhost:{LAUNCHER_PORT}/onvif-snap.jpg")
    print(f"[PTZ-Launcher] RTSP 视频流：rtsp://localhost:8554/ptz_cam  (VLC / NVR)")
    print(f"[PTZ-Launcher] HLS  视频流：http://localhost:8888/ptz_cam/index.m3u8  (浏览器 hls.js)")
    print(f"[PTZ-Launcher] WS-FLV 流：ws://localhost:{LAUNCHER_PORT}/ws-flv  (flv.js / jessibuca)")
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
