#!/usr/bin/env bash
# 安全退出视觉检测服务（优雅停止，超时则强制）。
# 用法: bash stop_vision_service.sh
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PORT=48051

cd "$PROJECT_DIR"

# ---- systemd 管理模式 ----
# 若服务由 systemd 单元 gdy_vison_detect.service 守护（Restart=always），
# 直接 pkill 会被 systemd 3 秒后自动拉起，因此必须通过 systemctl stop。
SERVICE_UNIT="gdy_vison_detect.service"
if systemctl is-active --quiet "${SERVICE_UNIT}" 2>/dev/null; then
  echo "检测到 systemd 单元 ${SERVICE_UNIT} 正在运行，通过 systemctl 停止..."
  sudo systemctl stop "${SERVICE_UNIT}"
  for i in $(seq 1 15); do
    if ! systemctl is-active --quiet "${SERVICE_UNIT}" 2>/dev/null; then
      echo "     systemd 单元已停止 (${i}秒)"
      break
    fi
    sleep 1
  done
  if systemctl is-active --quiet "${SERVICE_UNIT}" 2>/dev/null; then
    echo "[错误] systemd 单元未能停止，请检查: systemctl status ${SERVICE_UNIT}"
    exit 1
  fi
else
  echo "[提示] 未检测到 systemd 单元管理，按手动进程模式停止。"
fi

# ---- 手动/兜底模式 ----
PIDS="$(pgrep -f "run_service.py" || true)"
if [[ -z "${PIDS}" ]]; then
  echo "[提示] 未发现残留 run_service.py 进程。"
else
  echo "发现残留视觉服务进程: ${PIDS}"
  echo "[1/2] 发送SIGTERM优雅退出..."
  pkill -TERM -f "run_service.py" || true

  for i in $(seq 1 10); do
    if ! pgrep -f "run_service.py" >/dev/null 2>&1; then
      echo "     已优雅退出 (${i}秒)"
      break
    fi
    sleep 1
  done

  if pgrep -f "run_service.py" >/dev/null 2>&1; then
    echo "[2/2] 优雅退出超时，发送SIGKILL强制结束..."
    pkill -KILL -f "run_service.py" || true
    sleep 1
  fi
fi

if ss -tln | grep -q ":${PORT} "; then
  echo "[错误] 端口 ${PORT} 仍被占用:"
  ss -tlnp | grep ":${PORT} " || true
  exit 1
fi
echo "[完成] 服务已停止，端口 ${PORT} 已释放。"
