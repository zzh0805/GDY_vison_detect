#!/usr/bin/env bash
# 先成功采集5组 -> 空闲5分钟 -> 持续软触发采集10分钟 -> 空闲5分钟。
# 空闲期间不调用触发/读取/丢帧；不会用后台读帧掩盖SDK缓存增长。
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
IDLE_BEFORE="${IDLE_BEFORE:-300}" # 第一次空闲秒数
IDLE_AFTER="${IDLE_AFTER:-300}"   # 采集后的空闲秒数
WARMUP_FRAMES="${WARMUP_FRAMES:-5}" # 空闲前必须成功采集的组数；失败则不进入空闲
export DURATION="${DURATION:-600}" # 中间采集秒数
export MODE=soft                  # 持续发送软触发，不切换成连续曝光模式
export STREAMS=rgbd
export INTERVAL="${INTERVAL:-1}" # 每组后检查无额外帧的等待时间
bash "$HERE/run_test.sh" --warmup-frames "$WARMUP_FRAMES" --idle-before "$IDLE_BEFORE" --idle-after "$IDLE_AFTER" "$@"
