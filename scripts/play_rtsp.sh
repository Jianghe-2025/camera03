#!/usr/bin/env bash
# 用 ffplay 播放本工程 RTSP（需：仿真已运行，且 6674 上 mediamtx 在推流）
# 用法：在图形桌面的终端里执行 ./scripts/play_rtsp.sh
# 勿在命令中间换行；URL 必须完整为 ...6674/ptz_cam

set -euo pipefail
URL="${1:-rtsp://127.0.0.1:6674/ptz_cam}"
export DISPLAY="${DISPLAY:-:0}"

exec ffplay \
  -window_title "camera03 RTSP" \
  -rtsp_transport tcp \
  -fflags nobuffer \
  -flags low_delay \
  "$URL"
