#!/usr/bin/env bash
# 一次启动完成一个工件的交互式标定；只读JAKA位姿，不控制机械臂运动。
set -euo pipefail

TOOL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${TOOL_DIR}/.." && pwd)"
CONFIG_FILE="${1:-${TOOL_DIR}/interactive_tool_calibration.yaml}"

if [[ -f "${PROJECT_DIR}/linux_sdk_paths.env" ]]; then
  # shellcheck disable=SC1091
  source "${PROJECT_DIR}/load_sdk_env.sh"
else
  PYTHON_BIN="${PYTHON_BIN:-python3}"
  echo "[提示] 未找到linux_sdk_paths.env，使用 ${PYTHON_BIN}。"
fi

exec "${PYTHON_BIN}" "${TOOL_DIR}/interactive_tool_calibration.py" \
  --config "${CONFIG_FILE}"
