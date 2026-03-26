#!/usr/bin/env bash
# 用 ffplay 播放本工程 RTSP（需：Isaac 已启动且 ffmpeg 正在向 mediamtx 发布 ptz_cam）
# 若报 404：多为 ffmpeg 已崩溃而 mediamtx 仍空跑 —— 在 Web 停止再启动仿真，或：
#   pkill -f 'ffmpeg.*6674|mediamtx.*camera03' ; 再启动仿真
# 用法：在图形桌面的终端里执行 ./scripts/play_rtsp.sh
# 续行时反斜杠须在行尾；勿写成 low_delay \␠rtsp（行中 \␠ 会破坏参数）

set -euo pipefail
URL="${1:-rtsp://127.0.0.1:6674/ptz_cam}"
export DISPLAY="${DISPLAY:-:0}"

exec ffplay \
  -window_title "camera03 RTSP" \
  -rtsp_transport tcp \
  -fflags nobuffer \
  -flags low_delay \
  "$URL"
