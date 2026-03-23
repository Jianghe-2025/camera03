#!/usr/bin/env python3
"""
PTZ 轻量启动器  ptz_launcher.py
================================
始终运行的轻量 Web 服务（默认端口 8080），GPU 占用接近 0%。
通过 Web UI 按需启动 / 停止 Isaac Sim 推流进程（ptz_rtsp_stream.py）。

使用方法：
    python3 ptz_launcher.py [--config ./ptz_rtsp_config.yaml]

访问地址：
    http://localhost:8080/          Web 控制面板（始终可用）
    POST http://localhost:8080/start  启动 Isaac Sim 推流
    POST http://localhost:8080/stop   停止 Isaac Sim 推流

架构说明：
    launcher（8080）← 代理 → Isaac Sim（8081）
    用户浏览器只需访问 8080，无需感知 8081。

ONVIF 服务（新增）：
    POST http://localhost:8080/onvif/device_service  ONVIF 设备服务
    POST http://localhost:8080/onvif/media_service   ONVIF 媒体服务
    POST http://localhost:8080/onvif/ptz_service     ONVIF PTZ 控制服务
    GET  http://localhost:8080/onvif-snap.jpg        单帧快照（代理 Isaac 内部）
"""

import argparse
import json
import os
import signal
import socket
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

ap = argparse.ArgumentParser(description="PTZ Launcher - lightweight always-on controller")
ap.add_argument("--config", default=os.path.join(script_dir, "ptz_rtsp_config.yaml"))
args, _ = ap.parse_known_args()

with open(args.config, "r", encoding="utf-8") as f:
    cfg = yaml.safe_load(f)

LAUNCHER_PORT = cfg.get("launcher_port", 8080)
ISAAC_PORT    = cfg.get("ctrl_port",     8081)
PYTHON_SH     = cfg.get("python_sh",
                         "/home/uniubi/projects/issac/.isaac_sim_unzip/python.sh")
STREAM_SCRIPT = os.path.join(script_dir, "ptz_rtsp_stream.py")
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
    # ── 阶段一：等待 HTTP 就绪（最多 5 分钟）──────────────────
    for _ in range(300):
        if proc.poll() is not None:
            _isaac_state = "stopped"
            return
        if _is_isaac_http_ready():
            _isaac_state = "running"
            break
        time.sleep(1)
    else:
        # 超时仍未就绪，进程还在则当运行处理
        if proc.poll() is None:
            _isaac_state = "running"
        else:
            _isaac_state = "stopped"
            return

    # ── 阶段二：持续监控，进程退出后标记 stopped ──────────────
    while True:
        if proc.poll() is not None:
            # 进程已退出（正常 or 崩溃），做兜底清理
            for name in ("mediamtx", "ffmpeg"):
                try:
                    subprocess.run(["pkill", "-9", "-f", name],
                                   capture_output=True, timeout=3)
                except Exception:
                    pass
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
            start_new_session=True,   # 创建独立进程组，确保停止时能整组杀掉
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
        # 在进程还活着时提前记录 PGID（进程退出后 getpgid 会 ESRCH）
        try:
            pgid = os.getpgid(pid)
        except OSError:
            pgid = pid

    def _wait():
        global _isaac_proc, _isaac_state

        # ① 向整个进程组发 SIGTERM（bash + Python + mediamtx + ffmpeg 同时收到）
        try:
            os.killpg(pgid, signal.SIGTERM)
        except (ProcessLookupError, OSError):
            pass

        # ② 给 Python 最多 15 秒做优雅清理（关闭 ffmpeg pipe、释放 GPU）
        deadline = time.time() + 15
        while time.time() < deadline:
            # 用端口检测 Isaac Sim Python 是否还在（比 poll() 更准确，因为 bash 可能先退）
            if _port_free(ISAAC_PORT):
                break
            time.sleep(0.5)

        # ③ 无论 bash 是否已退，都对整个进程组补发 SIGKILL
        #    确保 Python、mediamtx、ffmpeg 不再占用资源
        try:
            os.killpg(pgid, signal.SIGKILL)
        except (ProcessLookupError, OSError):
            pass

        # ④ 等 bash 进程完全回收（避免 zombie）
        try:
            proc.wait(timeout=5)
        except (subprocess.TimeoutExpired, ChildProcessError):
            pass

        # ⑤ 名称兜底：杀掉可能已脱离进程组的 mediamtx / ffmpeg
        for name in ("mediamtx", "ffmpeg"):
            try:
                subprocess.run(["pkill", "-9", "-f", name],
                               capture_output=True, timeout=3)
            except Exception:
                pass

        # ⑥ 等待 GPU 驱动侧资源释放（端口释放后 CUDA context 通常也很快回收）
        for port in (ISAAC_PORT, 8554, 8888):
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
        proc     = _isaac_proc
        state    = _isaac_state
        up_sec   = int(time.time() - _start_time) if _start_time else 0
        dead     = (proc is not None and proc.poll() is not None)

    if dead and state not in ("stopped", "stopping"):
        # 进程意外退出
        _isaac_state = "stopped"
        state = "stopped"

    result = {
        "isaac_state":  state,          # stopped | starting | running | stopping
        "isaac_port":   ISAAC_PORT,
        "uptime_s":     up_sec if state == "running" else 0,
        "ptz":          None,
    }
    if state == "running":
        try:
            with urllib.request.urlopen(
                f"http://localhost:{ISAAC_PORT}/status", timeout=1
            ) as r:
                result["ptz"] = json.loads(r.read())
        except Exception:
            result["isaac_state"] = "starting"   # 进程在但 HTTP 还没好
    return result


# ── ONVIF 轻量 SOAP 服务器 ───────────────────────────────────────────
#
# 实现当前项目 onvif_client.py 调用的最小 ONVIF 接口子集：
#   Device  : GetCapabilities, GetSystemDateAndTime
#   Media   : GetProfiles, GetSnapshotUri
#   PTZ     : GetNodes, AbsoluteMove, RelativeMove, GotoPreset
#
# 坐标单位转换（ONVIF 归一化 → Camera03 度/倍数）：
#   pan_deg  = onvif_pan  * 170.0        [-1,1] → [-170,170]
#   tilt_deg = onvif_tilt * 90.0         [-1,1] → [-90, 90]，clamp 到 -90~30
#   zoom_x   = 1.0 + onvif_zoom * 31.0  [0,1]  → [1, 32]
# ────────────────────────────────────────────────────────────────────

_SOAP12_ENV = "http://www.w3.org/2003/05/soap-envelope"
_SOAP11_ENV = "http://schemas.xmlsoap.org/soap/envelope/"
_TT  = "http://www.onvif.org/ver10/schema"
_TDS = "http://www.onvif.org/ver10/device/wsdl"
_TRT = "http://www.onvif.org/ver10/media/wsdl"
_TPZ = "http://www.onvif.org/ver10/ptz/wsdl"

# 预置位映射：token → (pan_deg, tilt_deg, zoom_x)
_PTZ_PRESETS = {
    "1": (0.0,    -45.0, 1.0),
    "2": (90.0,   -45.0, 1.0),
    "3": (-90.0,  -45.0, 1.0),
    "home": (0.0, -45.0, 1.0),
}


def _soap_wrap(body_xml: str) -> bytes:
    """将 body XML 包裹为 SOAP 1.2 Envelope。"""
    return (
        '<?xml version="1.0" encoding="utf-8"?>'
        '<s:Envelope xmlns:s="http://www.w3.org/2003/05/soap-envelope">'
        '<s:Body>' + body_xml + '</s:Body>'
        '</s:Envelope>'
    ).encode("utf-8")


def _parse_soap_action(raw: bytes):
    """解析 SOAP XML，返回 (action_local_name, action_element)。"""
    try:
        root = ET.fromstring(raw)
        for ns in (_SOAP12_ENV, _SOAP11_ENV):
            body = root.find(f"{{{ns}}}Body")
            if body is not None:
                for child in body:
                    tag = child.tag
                    local = tag.split("}")[-1] if "}" in tag else tag
                    return local, child
    except Exception as e:
        print(f"[ONVIF] XML 解析失败: {e}")
    return None, None


def _find_elem(elem, local_name):
    """递归查找 local name 匹配的第一个子元素。"""
    for child in elem.iter():
        tag = child.tag.split("}")[-1] if "}" in child.tag else child.tag
        if tag == local_name:
            return child
    return None


def _find_text(elem, local_name) -> str:
    """递归查找 local name 匹配的第一个子元素的文本。"""
    e = _find_elem(elem, local_name)
    return (e.text or "").strip() if e is not None else ""


def _onvif_call_control(pan=None, tilt=None, zoom=None) -> None:
    """调用 Isaac Sim 内部 /control 接口。"""
    payload = {}
    if pan  is not None: payload["pan"]  = round(float(pan),  4)
    if tilt is not None: payload["tilt"] = round(float(tilt), 4)
    if zoom is not None: payload["zoom"] = round(float(zoom), 4)
    if not payload:
        return
    try:
        data = json.dumps(payload).encode()
        req  = urllib.request.Request(
            f"http://localhost:{ISAAC_PORT}/control",
            data=data,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(req, timeout=3) as r:
            r.read()
        print(f"[ONVIF] control OK: {payload}")
    except Exception as e:
        print(f"[ONVIF] control 失败 ({payload}): {e}")


def _onvif_get_current_ptz() -> tuple:
    """从 Isaac Sim 内部 /status 取当前 PTZ 状态，返回 (pan, tilt, zoom)。"""
    try:
        with urllib.request.urlopen(
            f"http://localhost:{ISAAC_PORT}/status", timeout=1
        ) as r:
            data = json.loads(r.read())
            return float(data.get("pan", 0.0)), float(data.get("tilt", -45.0)), float(data.get("zoom", 1.0))
    except Exception:
        return 0.0, -45.0, 1.0


def _handle_device_service(xml_body: bytes, host_port: str) -> bytes:
    action, elem = _parse_soap_action(xml_body)
    print(f"[ONVIF] device_service action={action}")

    if action == "GetCapabilities":
        body = f"""
<tds:GetCapabilitiesResponse xmlns:tds="{_TDS}">
  <tds:Capabilities>
    <tt:Device xmlns:tt="{_TT}">
      <tt:XAddr>http://{host_port}/onvif/device_service</tt:XAddr>
    </tt:Device>
    <tt:Media xmlns:tt="{_TT}">
      <tt:XAddr>http://{host_port}/onvif/media_service</tt:XAddr>
      <tt:StreamingCapabilities>
        <tt:RTPMulticast>false</tt:RTPMulticast>
        <tt:RTP_TCP>false</tt:RTP_TCP>
        <tt:RTP_RTSP_TCP>false</tt:RTP_RTSP_TCP>
      </tt:StreamingCapabilities>
    </tt:Media>
    <tt:PTZ xmlns:tt="{_TT}">
      <tt:XAddr>http://{host_port}/onvif/ptz_service</tt:XAddr>
    </tt:PTZ>
  </tds:Capabilities>
</tds:GetCapabilitiesResponse>"""
        return _soap_wrap(body)

    if action == "GetSystemDateAndTime":
        import datetime
        now = datetime.datetime.utcnow()
        body = f"""
<tds:GetSystemDateAndTimeResponse xmlns:tds="{_TDS}">
  <tds:SystemDateAndTime>
    <tt:DateTimeType xmlns:tt="{_TT}">NTP</tt:DateTimeType>
    <tt:DaylightSavings xmlns:tt="{_TT}">false</tt:DaylightSavings>
    <tt:UTCDateTime xmlns:tt="{_TT}">
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
</tds:GetSystemDateAndTimeResponse>"""
        return _soap_wrap(body)

    if action == "GetServices":
        body = f"""
<tds:GetServicesResponse xmlns:tds="{_TDS}">
  <tds:Service>
    <tds:Namespace>http://www.onvif.org/ver10/media/wsdl</tds:Namespace>
    <tds:XAddr>http://{host_port}/onvif/media_service</tds:XAddr>
    <tds:Version><tt:Major xmlns:tt="{_TT}">2</tt:Major><tt:Minor xmlns:tt="{_TT}">0</tt:Minor></tds:Version>
  </tds:Service>
  <tds:Service>
    <tds:Namespace>http://www.onvif.org/ver10/ptz/wsdl</tds:Namespace>
    <tds:XAddr>http://{host_port}/onvif/ptz_service</tds:XAddr>
    <tds:Version><tt:Major xmlns:tt="{_TT}">2</tt:Major><tt:Minor xmlns:tt="{_TT}">0</tt:Minor></tds:Version>
  </tds:Service>
</tds:GetServicesResponse>"""
        return _soap_wrap(body)

    # 未知操作：返回空 OK，不报错
    print(f"[ONVIF] device_service 未知 action={action}，返回空响应")
    return _soap_wrap(f'<tds:UnknownResponse xmlns:tds="{_TDS}"/>')


def _handle_media_service(xml_body: bytes, host_port: str) -> bytes:
    action, elem = _parse_soap_action(xml_body)
    print(f"[ONVIF] media_service action={action}")

    if action == "GetProfiles":
        body = f"""
<trt:GetProfilesResponse xmlns:trt="{_TRT}">
  <trt:Profiles token="MainProfileToken" fixed="true" xmlns:tt="{_TT}">
    <tt:Name>MainStream</tt:Name>
    <tt:PTZConfiguration token="PTZConfigToken">
      <tt:Name>PTZConfig</tt:Name>
      <tt:UseCount>1</tt:UseCount>
      <tt:NodeToken>PTZNodeToken</tt:NodeToken>
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
</trt:GetProfilesResponse>"""
        return _soap_wrap(body)

    if action == "GetSnapshotUri":
        body = f"""
<trt:GetSnapshotUriResponse xmlns:trt="{_TRT}">
  <trt:MediaUri>
    <tt:Uri xmlns:tt="{_TT}">http://{host_port}/onvif-snap.jpg</tt:Uri>
    <tt:InvalidAfterConnect xmlns:tt="{_TT}">false</tt:InvalidAfterConnect>
    <tt:InvalidAfterReboot xmlns:tt="{_TT}">false</tt:InvalidAfterReboot>
    <tt:Timeout xmlns:tt="{_TT}">PT30S</tt:Timeout>
  </trt:MediaUri>
</trt:GetSnapshotUriResponse>"""
        return _soap_wrap(body)

    if action == "GetServiceCapabilities":
        body = f"""
<trt:GetServiceCapabilitiesResponse xmlns:trt="{_TRT}">
  <trt:Capabilities SnapshotUri="true" Rotation="false" VideoSourceMode="false"
                    OSD="false" TemporaryOSDText="false" EXICompression="false"
                    xmlns:trt="{_TRT}"/>
</trt:GetServiceCapabilitiesResponse>"""
        return _soap_wrap(body)

    print(f"[ONVIF] media_service 未知 action={action}，返回空响应")
    return _soap_wrap(f'<trt:UnknownResponse xmlns:trt="{_TRT}"/>')


def _handle_ptz_service(xml_body: bytes) -> bytes:
    action, elem = _parse_soap_action(xml_body)
    print(f"[ONVIF] ptz_service action={action}")

    if action == "GetNodes":
        body = f"""
<tptz:GetNodesResponse xmlns:tptz="{_TPZ}">
  <tptz:PTZNode token="PTZNodeToken" FixedHomePosition="false" xmlns:tt="{_TT}">
    <tt:Name>PTZNode</tt:Name>
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
    <tt:MaximumNumberOfPresets>10</tt:MaximumNumberOfPresets>
    <tt:HomeSupported>false</tt:HomeSupported>
  </tptz:PTZNode>
</tptz:GetNodesResponse>"""
        return _soap_wrap(body)

    if action == "GetConfigurations":
        body = f"""
<tptz:GetConfigurationsResponse xmlns:tptz="{_TPZ}">
  <tptz:PTZConfiguration token="PTZConfigToken" xmlns:tt="{_TT}">
    <tt:Name>PTZConfig</tt:Name>
    <tt:UseCount>1</tt:UseCount>
    <tt:NodeToken>PTZNodeToken</tt:NodeToken>
    <tt:DefaultAbsolutePantTiltPositionSpace>http://www.onvif.org/ver10/tptz/PanTiltSpaces/PositionGenericSpace</tt:DefaultAbsolutePantTiltPositionSpace>
    <tt:DefaultAbsoluteZoomPositionSpace>http://www.onvif.org/ver10/tptz/ZoomSpaces/PositionGenericSpace</tt:DefaultAbsoluteZoomPositionSpace>
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
</tptz:GetConfigurationsResponse>"""
        return _soap_wrap(body)

    if action == "GetServiceCapabilities":
        body = f"""
<tptz:GetServiceCapabilitiesResponse xmlns:tptz="{_TPZ}">
  <tptz:Capabilities EFlip="false" Reverse="false" GetCompatibleConfigurations="false"
                     MoveStatus="false" StatusPosition="false"/>
</tptz:GetServiceCapabilitiesResponse>"""
        return _soap_wrap(body)

    if action == "GetPresets":
        presets_xml = ""
        for token, (p, t, z) in _PTZ_PRESETS.items():
            presets_xml += f"""
  <tptz:Preset token="{token}" xmlns:tt="{_TT}">
    <tt:Name>Preset{token}</tt:Name>
    <tt:PTZPosition>
      <tt:PanTilt x="{round(p/170.0, 4)}" y="{round(t/90.0, 4)}" space="http://www.onvif.org/ver10/tptz/PanTiltSpaces/PositionGenericSpace"/>
      <tt:Zoom x="{round((z-1)/31.0, 4)}" space="http://www.onvif.org/ver10/tptz/ZoomSpaces/PositionGenericSpace"/>
    </tt:PTZPosition>
  </tptz:Preset>"""
        return _soap_wrap(f'<tptz:GetPresetsResponse xmlns:tptz="{_TPZ}">{presets_xml}</tptz:GetPresetsResponse>')

    if action == "AbsoluteMove":
        try:
            pt_elem   = _find_elem(elem, "PanTilt")
            zoom_elem = _find_elem(elem, "Zoom")
            pan_onvif  = float(pt_elem.get("x", 0))   if pt_elem   is not None else 0.0
            tilt_onvif = float(pt_elem.get("y", 0))   if pt_elem   is not None else 0.0
            zoom_onvif = float(zoom_elem.get("x", 0)) if zoom_elem is not None else 0.0

            # ONVIF 归一化 → Camera03 度/倍数
            pan_deg  = pan_onvif  * 170.0
            tilt_deg = max(-90.0, min(30.0, tilt_onvif * 90.0))
            zoom_x   = 1.0 + zoom_onvif * 31.0

            print(f"[ONVIF] AbsoluteMove onvif=({pan_onvif:.3f},{tilt_onvif:.3f},{zoom_onvif:.3f}) "
                  f"→ cam03=({pan_deg:.1f}°,{tilt_deg:.1f}°,{zoom_x:.2f}x)")
            _onvif_call_control(pan=pan_deg, tilt=tilt_deg, zoom=zoom_x)
        except Exception as e:
            print(f"[ONVIF] AbsoluteMove 解析失败: {e}")
        return _soap_wrap(f'<tptz:AbsoluteMoveResponse xmlns:tptz="{_TPZ}"/>')

    if action == "RelativeMove":
        try:
            pt_elem    = _find_elem(elem, "PanTilt")
            pan_onvif  = float(pt_elem.get("x", 0)) if pt_elem is not None else 0.0
            tilt_onvif = float(pt_elem.get("y", 0)) if pt_elem is not None else 0.0

            cur_pan, cur_tilt, _ = _onvif_get_current_ptz()
            pan_new  = max(-170.0, min(170.0, cur_pan  + pan_onvif  * 170.0))
            tilt_new = max(-90.0,  min(30.0,  cur_tilt + tilt_onvif * 90.0))

            print(f"[ONVIF] RelativeMove Δ=({pan_onvif:.3f},{tilt_onvif:.3f}) "
                  f"→ new=({pan_new:.1f}°,{tilt_new:.1f}°)")
            _onvif_call_control(pan=pan_new, tilt=tilt_new)
        except Exception as e:
            print(f"[ONVIF] RelativeMove 解析失败: {e}")
        return _soap_wrap(f'<tptz:RelativeMoveResponse xmlns:tptz="{_TPZ}"/>')

    if action == "GotoPreset":
        try:
            preset_token = _find_text(elem, "PresetToken")
            if preset_token in _PTZ_PRESETS:
                pan_deg, tilt_deg, zoom_x = _PTZ_PRESETS[preset_token]
                print(f"[ONVIF] GotoPreset token={preset_token} → ({pan_deg}°,{tilt_deg}°,{zoom_x}x)")
                _onvif_call_control(pan=pan_deg, tilt=tilt_deg, zoom=zoom_x)
            else:
                print(f"[ONVIF] GotoPreset 未知 token={preset_token}")
        except Exception as e:
            print(f"[ONVIF] GotoPreset 解析失败: {e}")
        return _soap_wrap(f'<tptz:GotoPresetResponse xmlns:tptz="{_TPZ}"/>')

    if action == "GetStatus":
        cur_pan, cur_tilt, cur_zoom = _onvif_get_current_ptz()
        pan_n  = round(cur_pan  / 170.0, 4)
        tilt_n = round(cur_tilt / 90.0,  4)
        zoom_n = round((cur_zoom - 1.0) / 31.0, 4)
        body = f"""
<tptz:GetStatusResponse xmlns:tptz="{_TPZ}">
  <tptz:PTZStatus xmlns:tt="{_TT}">
    <tt:Position>
      <tt:PanTilt x="{pan_n}" y="{tilt_n}" space="http://www.onvif.org/ver10/tptz/PanTiltSpaces/PositionGenericSpace"/>
      <tt:Zoom x="{zoom_n}" space="http://www.onvif.org/ver10/tptz/ZoomSpaces/PositionGenericSpace"/>
    </tt:Position>
    <tt:MoveStatus>
      <tt:PanTilt>IDLE</tt:PanTilt>
      <tt:Zoom>IDLE</tt:Zoom>
    </tt:MoveStatus>
  </tptz:PTZStatus>
</tptz:GetStatusResponse>"""
        return _soap_wrap(body)

    print(f"[ONVIF] ptz_service 未知 action={action}，返回空响应")
    return _soap_wrap(f'<tptz:UnknownResponse xmlns:tptz="{_TPZ}"/>')


# ── HTTP Handler ─────────────────────────────────────────────────────
class _Handler(BaseHTTPRequestHandler):

    def log_message(self, *_):
        pass  # 静默访问日志

    def _cors(self):
        self.send_header("Access-Control-Allow-Origin",  "*")
        self.send_header("Access-Control-Allow-Methods", "GET,POST,OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")

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

    # ── GET ──────────────────────────────────────────────────
    def do_GET(self):
        path = self.path.split("?")[0]

        # Web UI
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

        # 启动器状态
        if path == "/status":
            self._json(get_status())
            return

        # Isaac Sim 启动日志（末尾 8 KB）
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

        # MJPEG 代理（持续流）
        if path == "/stream.mjpeg":
            self._proxy_stream(f"http://localhost:{ISAAC_PORT}/stream.mjpeg")
            return

        # 快照代理
        if path == "/snapshot.jpg":
            self._proxy_once(f"http://localhost:{ISAAC_PORT}/snapshot.jpg")
            return

        # ONVIF 单帧快照（代理 Isaac 内部快照，供 ONVIF GetSnapshotUri 结果使用）
        if path == "/onvif-snap.jpg":
            self._proxy_once(f"http://localhost:{ISAAC_PORT}/snapshot.jpg")
            return

        # 场景状态代理
        if path == "/scene/state":
            self._proxy_once(f"http://localhost:{ISAAC_PORT}/scene/state")
            return

        self.send_response(404); self.end_headers()

    # ── POST ─────────────────────────────────────────────────
    def do_POST(self):
        path = self.path.split("?")[0]

        if path == "/start":
            self._json(start_isaac())
            return

        if path == "/stop":
            self._json(stop_isaac())
            return

        # ONVIF SOAP 服务（新增）
        if path.startswith("/onvif/"):
            length   = int(self.headers.get("Content-Length", 0))
            xml_body = self.rfile.read(length)
            host_port = self.headers.get("Host", f"localhost:{LAUNCHER_PORT}")

            if path == "/onvif/device_service":
                resp = _handle_device_service(xml_body, host_port)
            elif path == "/onvif/media_service":
                resp = _handle_media_service(xml_body, host_port)
            elif path == "/onvif/ptz_service":
                resp = _handle_ptz_service(xml_body)
            else:
                self.send_response(404); self.end_headers(); return

            self.send_response(200)
            self.send_header("Content-Type", "application/soap+xml; charset=utf-8")
            self.send_header("Content-Length", str(len(resp)))
            self._cors()
            self.end_headers()
            self.wfile.write(resp)
            return

        # 透明代理 POST 到 Isaac Sim 的 /control 和 /scene/* 端点
        if path in ("/control", "/scene/gondola", "/scene/workers"):
            length = int(self.headers.get("Content-Length", 0))
            body   = self.rfile.read(length)
            if _isaac_state != "running":
                self._json({"ok": False, "error": "Isaac Sim 未就绪"}, 503)
                return
            self._proxy_post(path, body)
            return

        self.send_response(404); self.end_headers()

    # ── 代理工具 ─────────────────────────────────────────────
    def _proxy_stream(self, url: str) -> None:
        """代理持续 MJPEG 流。"""
        try:
            req = urllib.request.Request(url)
            with urllib.request.urlopen(req, timeout=5) as upstream:
                ct = upstream.headers.get(
                    "Content-Type",
                    "multipart/x-mixed-replace; boundary=ptzframe"
                )
                self.send_response(200)
                self.send_header("Content-Type", ct)
                self.send_header("Cache-Control", "no-cache, no-store")
                self.send_header("Connection",    "close")
                self._cors()
                self.end_headers()
                while True:
                    chunk = upstream.read(65536)
                    if not chunk:
                        break
                    self.wfile.write(chunk)
                    self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        except Exception:
            # Isaac Sim 未就绪，返回 503
            try:
                self.send_response(503); self.end_headers()
            except Exception:
                pass

    def _proxy_post(self, path: str, body: bytes) -> None:
        """将 POST 请求代理到 Isaac Sim 并返回响应。"""
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

    def _proxy_once(self, url: str) -> None:
        """代理单次 HTTP 请求（快照等）。"""
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


# ── 主函数 ──────────────────────────────────────────────────────────
def main():
    srv = ThreadingHTTPServer(("0.0.0.0", LAUNCHER_PORT), _Handler)
    print(f"[PTZ-Launcher] 轻量启动器运行中 → http://localhost:{LAUNCHER_PORT}/")
    print(f"[PTZ-Launcher] Isaac Sim 内部端口：{ISAAC_PORT}（按需启动）")
    print(f"[PTZ-Launcher] ONVIF 服务端点：")
    print(f"[PTZ-Launcher]   POST http://localhost:{LAUNCHER_PORT}/onvif/device_service")
    print(f"[PTZ-Launcher]   POST http://localhost:{LAUNCHER_PORT}/onvif/media_service")
    print(f"[PTZ-Launcher]   POST http://localhost:{LAUNCHER_PORT}/onvif/ptz_service")
    print(f"[PTZ-Launcher]   GET  http://localhost:{LAUNCHER_PORT}/onvif-snap.jpg")
    print(f"[PTZ-Launcher] 在 Web 界面点击 '开始推流' 启动 Isaac Sim")
    print(f"[PTZ-Launcher] 按 Ctrl-C 停止（同时停止 Isaac Sim）")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n[PTZ-Launcher] 正在停止...")
        stop_isaac()
        time.sleep(2)


if __name__ == "__main__":
    main()
