#!/usr/bin/env bash
# 独立测试，不启动服务、不连接机械臂、不修改workflow。
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# 必填：完整厂家3DCameraSDK目录，内含inc/3dcamera和lib/3dcamera/linux。
# 可直接修改下面的默认值，或运行前 export SDK_ROOT=/实际路径。
SDK_ROOT="${SDK_ROOT:-/home/nvidia/3DCameraSDK-v3.2.229.20251110}"
PYTHON_BIN="${PYTHON_BIN:-python}"
MODE="${MODE:-soft}"             # soft=真实深度软触发；continuous=原生连续流对照
DURATION="${DURATION:-1200}"     # 采集阶段20分钟；初始化、关闭时间另计
STREAMS="${STREAMS:-rgbd}"       # rgbd=SDK配对彩色+深度；depth=原来的深度单流
RGB_PROFILE="${RGB_PROFILE:-0}" # 使用输出列出的RGB8规格索引
INTERVAL="${INTERVAL:-1}"        # soft:触发间无帧检查时间；continuous:两次读取间等待秒数
CAMERA_IP="${CAMERA_IP:-192.168.16.122}"
PROFILE="${PROFILE:-0}"          # 使用输出列出的Z16规格索引；对照时保持一致
MAX_MEMORY_MB="${MAX_MEMORY_MB:-2048}" # RSS+Swap保护上限，不等系统OOM
CALL_TIMEOUT="${CALL_TIMEOUT:-15}" # 独立进程监督单次触发/取帧/关闭；超时后再留10秒退出
OPEN_TIMEOUT="${OPEN_TIMEOUT:-90}" # SDK加载/打开允许较长初始化时间

if pgrep -f '(^|[ /])run_service\.py([[:space:]]|$)' >/dev/null; then
    echo '检测到run_service.py相关进程，请先停止视觉服务（包括systemd自动重启），关闭厂家相机软件。'
    exit 1
fi
case "$(uname -m)" in
    aarch64|arm64) ARCH=aarch64 ;;
    x86_64) ARCH=x64 ;;
    *) echo '不支持的CPU架构'; exit 1 ;;
esac
SDK_LIB_DIR="${SDK_LIB_DIR:-$SDK_ROOT/lib/3dcamera/linux/$ARCH}"
if [[ ! -f "$SDK_ROOT/inc/3dcamera/3DCamera.hpp" || ! -f "$SDK_LIB_DIR/lib3DCamera.so" ]]; then
    echo "找不到完整原生SDK。请设置SDK_ROOT（当前：$SDK_ROOT）和必要时SDK_LIB_DIR。"
    echo '只有libOpenNI2.so不能运行此软触发测试；必须用配套架构的lib3DCamera.so和头文件。'
    exit 1
fi
command -v g++ >/dev/null || { echo '缺少g++，请安装build-essential'; exit 1; }
# 不source生产load_sdk_env.sh，不导入torch/YOLO/OpenNI。避免预加载项污染基线。
unset LD_PRELOAD
export LD_LIBRARY_PATH="$SDK_LIB_DIR${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
g++ -std=c++14 -O2 -fPIC -shared "$HERE/bridge.cpp" \
    -I"$SDK_ROOT/inc/3dcamera" -L"$SDK_LIB_DIR" \
    -Wl,-rpath,"$SDK_LIB_DIR" -l3DCamera -o "$HERE/libsoft_trigger_test.so"
echo "Python=$PYTHON_BIN SDK=$SDK_LIB_DIR mode=$MODE streams=$STREAMS interval=$INTERVAL duration=$DURATION"
exec "$PYTHON_BIN" -u "$HERE/test_soft_trigger.py" \
    --ip "$CAMERA_IP" --mode "$MODE" --duration "$DURATION" --interval "$INTERVAL" \
    --profile "$PROFILE" --streams "$STREAMS" --rgb-profile "$RGB_PROFILE" \
    --call-timeout "$CALL_TIMEOUT" --open-timeout "$OPEN_TIMEOUT" \
    --max-memory-mb "$MAX_MEMORY_MB" "$@"
