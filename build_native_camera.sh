#!/usr/bin/env bash
# Run once on the target Ubuntu before starting v5; never opens the camera.
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SDK_ROOT="${SDK_ROOT:-$(dirname "$PROJECT_DIR")/3DCameraSDK-v3.2.229.20251110}"
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
g++ -std=c++14 -O2 -fPIC -shared "$NATIVE_DIR/bridge.cpp" \
  -I"$SDK_ROOT/inc/3dcamera" -L"$SDK_LIB_DIR" \
  -Wl,-rpath,"$SDK_LIB_DIR" -Wl,-z,defs -l3DCamera -pthread \
  -o "$NATIVE_DIR/libgdy_native_camera.so.new"
mv -- "$NATIVE_DIR/libgdy_native_camera.so.new" "$NATIVE_DIR/libgdy_native_camera.so"
"${PYTHON_BIN:-python}" - "$NATIVE_DIR/sdk_location.json" "$SDK_LIB_DIR" <<'PY'
import json, sys
from pathlib import Path
Path(sys.argv[1]).write_text(json.dumps({"sdk_lib_dir": str(Path(sys.argv[2]).resolve())}), encoding="utf-8")
PY
echo "v5 native bridge built. Business YAML and calibration files were not modified."
