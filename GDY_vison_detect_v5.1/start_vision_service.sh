#!/usr/bin/env bash
# 一键启动视觉检测服务并挂后台。
# 用法: bash start_vision_service.sh
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PORT=48051
LOG_FILE="${PROJECT_DIR}/service.log"

cd "$PROJECT_DIR"

# 已在运行则直接提示
if pgrep -f "run_service.py" >/dev/null 2>&1; then
  echo "[提示] 视觉服务已在运行:"
  pgrep -af "run_service.py"
  exit 0
fi

echo "[1/3] 启动视觉服务(后台)..."
nohup bash run_service.sh > "${LOG_FILE}" 2>&1 &
SERVICE_PID=$!
echo "     启动PID=$SERVICE_PID，日志: ${LOG_FILE}"

echo "[2/3] 等待端口 ${PORT} 就绪(最多60秒)..."
for i in $(seq 1 60); do
  if ss -tln | grep -q ":${PORT} "; then
    echo "     端口 ${PORT} 已就绪 (${i}秒)"
    break
  fi
  if ! kill -0 "$SERVICE_PID" 2>/dev/null; then
    echo "[错误] 服务进程已退出，最近日志:"
    tail -20 "${LOG_FILE}"
    exit 1
  fi
  sleep 1
done

echo "[3/3] 最终状态:"
ss -tlnp | grep ":${PORT} " || {
  echo "[错误] 端口 ${PORT} 未监听，最近日志:"
  tail -20 "${LOG_FILE}"
  exit 1
}
echo "视觉服务已启动并监听 ${PORT}。"
