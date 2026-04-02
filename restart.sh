#!/usr/bin/env bash
# 重启 PTZ 启动器（结束旧 launcher / Isaac 推流 / mediamtx，再后台拉起 ptz_launcher）
# 用法：
#   ./restart.sh
# 环境变量（可选）：
#   CAMERA03_PYTHON         默认 python3，可改为 /path/to/python3
#   CAMERA03_LAUNCHER_LOG   默认 /tmp/ptz_launcher_restart.log

set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT"
CFG="$ROOT/ptz_config.yaml"

PYTHON="${CAMERA03_PYTHON:-python3}"
LOG="${CAMERA03_LAUNCHER_LOG:-/tmp/ptz_launcher_restart.log}"

# 与推流脚本一致：优先用 conda 里的 ffmpeg/git 等
export PATH="${HOME}/miniconda3/bin:/usr/bin:/bin:${PATH}"

LAUNCHER_PORT="$(grep -m1 '^launcher_port:' "$CFG" 2>/dev/null | sed 's/^launcher_port:[[:space:]]*//;s/[[:space:]]*$//' || true)"
[[ -z "${LAUNCHER_PORT:-}" || ! "$LAUNCHER_PORT" =~ ^[0-9]+$ ]] && LAUNCHER_PORT=8080

echo "[restart] 目录: $ROOT"
echo "[restart] 结束旧进程…"
pkill -f "ptz_launcher.py" 2>/dev/null || true
pkill -f "ptz_stream.py" 2>/dev/null || true
pkill -f "[m]ediamtx" 2>/dev/null || true
sleep 2

: > "$LOG"
echo "[restart] 启动 ptz_launcher（日志: $LOG）…"
nohup "$PYTHON" "$ROOT/ptz_launcher.py" --config "$CFG" >>"$LOG" 2>&1 &

for i in $(seq 1 20); do
  if command -v ss >/dev/null 2>&1; then
    if ss -ltn 2>/dev/null | grep -qE ":${LAUNCHER_PORT}[[:space:]]"; then
      echo "[restart] 已监听 :${LAUNCHER_PORT}  →  http://127.0.0.1:${LAUNCHER_PORT}/"
      exit 0
    fi
  elif command -v nc >/dev/null 2>&1; then
    if nc -z 127.0.0.1 "$LAUNCHER_PORT" 2>/dev/null; then
      echo "[restart] 已监听 :${LAUNCHER_PORT}  →  http://127.0.0.1:${LAUNCHER_PORT}/"
      exit 0
    fi
  fi
  sleep 1
done

echo "[restart] 警告: 约 20s 内未检测到 :${LAUNCHER_PORT} 监听，请查看 $LOG"
exit 1
