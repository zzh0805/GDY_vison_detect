#!/usr/bin/env bash
# 本文件必须被 source，不能单独执行。
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SDK_ENV="${PROJECT_DIR}/linux_sdk_paths.env"

if [[ ! -f "${SDK_ENV}" ]]; then
  echo "[ERROR] 缺少 ${SDK_ENV}" >&2
  echo "请复制 linux_sdk_paths.env.example 为 linux_sdk_paths.env 并填写ARM64 SDK路径。" >&2
  return 1
fi

# shellcheck disable=SC1090
source "${SDK_ENV}"

LIB_DIRS=(
  "${SURFACEPRO50_OPENNI2_REDIST:-}"
  "${SURFACEPRO50_OPENNI2_REDIST:-}/OpenNI2/Drivers"
  "${SURFACEPRO50_LIBRARY_PATH:-}"
  "${JAKA_SDK_PATH:-}"
  "${JAKA_LIBRARY_PATH:-}"
)
JOINED_LIB_DIRS="$(IFS=:; echo "${LIB_DIRS[*]}")"
export LD_LIBRARY_PATH="${JOINED_LIB_DIRS}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

if [[ -n "${SURFACEPRO50_PYTHON_PATH:-}" ]]; then
  export PYTHONPATH="${SURFACEPRO50_PYTHON_PATH}${PYTHONPATH:+:${PYTHONPATH}}"
fi
if [[ -n "${JAKA_SDK_PATH:-}" ]]; then
  export PYTHONPATH="${JAKA_SDK_PATH}${PYTHONPATH:+:${PYTHONPATH}}"
fi

# 由相机适配器使用 SURFACEPRO50_OPENNI2_REDIST 选择正确运行库。
unset OPENNI2_REDIST || true

if command -v conda >/dev/null 2>&1; then
  eval "$(conda shell.bash hook)"
  conda activate "${VISION_CONDA_ENV:-ur_odcam}"
fi

# 避免OpenCV/PyTorch在ARM64 Conda环境中出现
# "cannot allocate memory in static TLS block"。
# ① glibc>=2.34：增大可选static TLS空间；
# ② glibc 2.31等旧版：预加载libgomp与torch核心库，把static TLS分配提前到进程启动。
export GLIBC_TUNABLES="${GLIBC_TUNABLES:+${GLIBC_TUNABLES}:}glibc.rtld.optional_static_tls=1M"
if [[ -n "${CONDA_PREFIX:-}" ]]; then
  PRELOAD_LIBS=()
  GOMP_LIBRARY="${CONDA_PREFIX}/lib/libgomp.so.1"
  if [[ -f "${GOMP_LIBRARY}" ]]; then
    PRELOAD_LIBS+=("${GOMP_LIBRARY}")
  fi
  TORCH_LIB_DIR="${CONDA_PREFIX}/lib/python3.10/site-packages/torch/lib"
  for lib in libc10.so libtorch_cpu.so; do
    if [[ -f "${TORCH_LIB_DIR}/${lib}" ]]; then
      PRELOAD_LIBS+=("${TORCH_LIB_DIR}/${lib}")
    fi
  done
  if (( ${#PRELOAD_LIBS[@]} > 0 )); then
    JOINED_PRELOAD="$(IFS=:; echo "${PRELOAD_LIBS[*]}")"
    export LD_PRELOAD="${JOINED_PRELOAD}${LD_PRELOAD:+:${LD_PRELOAD}}"
  fi
fi

export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-offscreen}"
export MPLBACKEND="${MPLBACKEND:-Agg}"
export PYTHON_BIN="${PYTHON_BIN:-python}"

