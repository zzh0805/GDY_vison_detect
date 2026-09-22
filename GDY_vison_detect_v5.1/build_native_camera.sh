#!/usr/bin/env bash
# Run once on the target Ubuntu before starting v5.1; never opens the camera.
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# The SDK normally sits next to the project (deployment layout); a copy placed inside the
# project is also accepted so a local test copy does not need SDK_ROOT to be exported.
SDK_ROOT="${SDK_ROOT:-$(dirname "$PROJECT_DIR")/3DCameraSDK-v3.2.229.20251110}"
if [[ ! -f "$SDK_ROOT/inc/3dcamera/3DCamera.hpp" && -f "$PROJECT_DIR/3DCameraSDK-v3.2.229.20251110/inc/3dcamera/3DCamera.hpp" ]]; then
  SDK_ROOT="$PROJECT_DIR/3DCameraSDK-v3.2.229.20251110"
fi
case "$(uname -m)" in
  aarch64|arm64) ARCH=aarch64 ;;
  x86_64) ARCH=x64 ;;
  *) echo 'Unsupported architecture'; exit 1 ;;
esac
SDK_LIB_DIR="${SDK_LIB_DIR:-$SDK_ROOT/lib/3dcamera/linux/$ARCH}"
NATIVE_DIR="$PROJECT_DIR/handeye_calib/native_camera"
if [[ ! -f "$SDK_ROOT/inc/3dcamera/3DCamera.hpp" || ! -f "$SDK_LIB_DIR/lib3DCamera.so" ]]; then
  echo "Set SDK_ROOT and SDK_LIB_DIR to the matching native SDK headers/library. Current root: $SDK_ROOT"
  exit 1
fi
command -v g++ >/dev/null || { echo 'Install build-essential first'; exit 1; }
# Keep the service stopped during a rebuild; use a temporary output then rename.
g++ -std=c++14 -O2 -Wall -Wextra "$NATIVE_DIR/worker.cpp" \
  -I"$SDK_ROOT/inc/3dcamera" -L"$SDK_LIB_DIR" \
  -Wl,-rpath,"$SDK_LIB_DIR" -Wl,-z,defs -l3DCamera -pthread \
  -o "$NATIVE_DIR/gdy_camera_worker.new"
mv -- "$NATIVE_DIR/gdy_camera_worker.new" "$NATIVE_DIR/gdy_camera_worker"
"${PYTHON_BIN:-python}" - "$NATIVE_DIR/sdk_location.json" "$SDK_LIB_DIR" <<'PY'
import json, sys
from pathlib import Path
Path(sys.argv[1]).write_text(json.dumps({"sdk_lib_dir": str(Path(sys.argv[2]).resolve())}), encoding="utf-8")
PY
echo "v5.1 C++ camera worker built. Configure camera.native and preview in config/workflow.yaml."
