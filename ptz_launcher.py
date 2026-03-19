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
