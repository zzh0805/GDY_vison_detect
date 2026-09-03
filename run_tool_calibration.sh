#!/usr/bin/env bash
# GDY视觉v3工具标定/验证/微调脚本。
#
# 使用方法：
#   1. 只修改下面“用户参数区”；
#   2. 在Ubuntu项目根目录执行：bash run_tool_calibration.sh
#   3. 将输出的类别段复制到 config/tool_offsets.yaml 的 tools: 下；
#   4. 保存后下一次/get_tcp_pose自动生效，不需要重启视觉服务。
#
# 本脚本只计算并打印结果，不连接相机、不控制机械臂，也不会自动改YAML。
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ======================== 用户参数区（从这里修改） ========================

# 操作模式，只能填写以下一个值：
#   calibrate：标定新工具，或用最新示教TCP完整重标定旧工具；
#   verify：   验证当前standard_to_tool会计算出什么最终TCP；
#   adjust：   在当前standard_to_tool基础上做基座系位置/RPY小幅微调。
MODE="calibrate"

# 标准TCP来源，只能填写以下一个值：
#   result_json：从v3完整解算报告的cameraReference.TBaseTcpStandard读取；
#   standard_reference：直接使用下面STANDARD_REFERENCE填写的6维位姿。
REFERENCE_SOURCE="result_json"

# REFERENCE_SOURCE=result_json时使用。
# 必须指向服务output/solve-...目录中的完整result.json，不能使用仅含HTTP
# code/pos的field_test/output/test_result.json。
RESULT_JSON="output/solve-请替换为实际目录/result.json"

# REFERENCE_SOURCE=standard_reference时使用。
# 顺序为X,Y,Z,Rx,Ry,Rz；位置单位mm，JAKA RPY角度单位deg。
# 示例：STANDARD_REFERENCE="959.9279,-170.8918,430.8637,90.0636,-45.0013,91.0221"
STANDARD_REFERENCE=""

# YOLO输出的类别名称，必须与config/tool_offsets.yaml中的键完全一致。
# 注意konb1不要误写成kobn1。
CLASS_NAME="new_tool"

# 返回报告中用于识别物理工具的名称，可自行命名。
TOOL_ID="tool_new_tool"

# MODE=calibrate时使用：工具人工示教到最终正确工作位时读取的活动TCP。
# 顺序为X,Y,Z,Rx,Ry,Rz；位置单位mm，JAKA RPY角度单位deg。
TAUGHT_TCP=""

# MODE=verify或adjust时使用：当前工具偏移，共6个数字。
# 顺序为xyz_mm.X,Y,Z,rpy_deg.Rx,Ry,Rz。
# 示例：CURRENT_STANDARD_TO_TOOL="144.6178,-15.8401,-16.9575,-0.0006,-0.0020,0.0004"
CURRENT_STANDARD_TO_TOOL=""

# MODE=adjust时使用：希望“最终TCP”在JAKA基座坐标系移动多少mm。
# 顺序为基座X,Y,Z；留空表示不调整位置。
# 示例：向基座+Z移动1mm：ADJUST_XYZ_MM="0,0,1"
#       向基座-Y移动1mm：ADJUST_XYZ_MM="0,-1,0"
ADJUST_XYZ_MM=""

# MODE=adjust时使用：希望最终JAKA RPY分量分别增加多少度。
# 顺序为Rx,Ry,Rz；留空表示不调整角度。只建议用于小角度微调；
# 姿态偏差较大时应改用MODE=calibrate重新示教。
# 示例：Rz增加0.2度：ADJUST_RPY_DEG="0,0,0.2"
ADJUST_RPY_DEG=""

# ========================== 用户参数区结束 ================================

die() {
  echo "[错误] $*" >&2
  exit 2
}

require_value() {
  local value="$1"
  local name="$2"
  [[ -n "${value}" ]] || die "${name}不能为空，请修改脚本顶部用户参数区。"
}

# 部署机已有linux_sdk_paths.env时复用其中的Conda环境/PYTHON_BIN；工具标定
# 本身只需要Python和项目依赖，因此没有该文件时直接使用python3。
if [[ -f "${PROJECT_DIR}/linux_sdk_paths.env" ]]; then
  # shellcheck disable=SC1091
  source "${PROJECT_DIR}/load_sdk_env.sh"
else
  PYTHON_BIN="${PYTHON_BIN:-python3}"
  echo "[提示] 未找到linux_sdk_paths.env，使用 ${PYTHON_BIN}。"
fi

ARGS=()
case "${REFERENCE_SOURCE}" in
  result_json)
    require_value "${RESULT_JSON}" "RESULT_JSON"
    RESULT_PATH="${RESULT_JSON}"
    if [[ "${RESULT_PATH}" != /* ]]; then
      RESULT_PATH="${PROJECT_DIR}/${RESULT_PATH}"
    fi
    [[ -f "${RESULT_PATH}" ]] || die "完整result.json不存在：${RESULT_PATH}"
    ARGS+=(--result-json "${RESULT_PATH}")
    ;;
  standard_reference)
    require_value "${STANDARD_REFERENCE}" "STANDARD_REFERENCE"
    ARGS+=("--standard-reference=${STANDARD_REFERENCE}")
    ;;
  *)
    die "REFERENCE_SOURCE只能是result_json或standard_reference，当前为：${REFERENCE_SOURCE}"
    ;;
esac

require_value "${CLASS_NAME}" "CLASS_NAME"
require_value "${TOOL_ID}" "TOOL_ID"
ARGS+=(--class-name "${CLASS_NAME}" --tool-id "${TOOL_ID}")

case "${MODE}" in
  calibrate)
    require_value "${TAUGHT_TCP}" "TAUGHT_TCP"
    ARGS+=("--taught-tcp=${TAUGHT_TCP}")
    ;;
  verify)
    require_value "${CURRENT_STANDARD_TO_TOOL}" "CURRENT_STANDARD_TO_TOOL"
    ARGS+=("--standard-to-tool=${CURRENT_STANDARD_TO_TOOL}")
    ;;
  adjust)
    require_value "${CURRENT_STANDARD_TO_TOOL}" "CURRENT_STANDARD_TO_TOOL"
    if [[ -z "${ADJUST_XYZ_MM}" && -z "${ADJUST_RPY_DEG}" ]]; then
      die "MODE=adjust时ADJUST_XYZ_MM和ADJUST_RPY_DEG至少填写一个。"
    fi
    ARGS+=("--standard-to-tool=${CURRENT_STANDARD_TO_TOOL}")
    if [[ -n "${ADJUST_XYZ_MM}" ]]; then
      ARGS+=("--adjust-xyz-mm=${ADJUST_XYZ_MM}")
    fi
    if [[ -n "${ADJUST_RPY_DEG}" ]]; then
      ARGS+=("--adjust-rpy-deg=${ADJUST_RPY_DEG}")
    fi
    ;;
  *)
    die "MODE只能是calibrate、verify或adjust，当前为：${MODE}"
    ;;
esac

echo "==== 工具标定参数 ===="
echo "模式: ${MODE}"
echo "标准TCP来源: ${REFERENCE_SOURCE}"
echo "目标类别: ${CLASS_NAME}"
echo "工具ID: ${TOOL_ID}"
printf '执行命令:'
printf ' %q' "${PYTHON_BIN}" "${PROJECT_DIR}/calibrate_tool_offset.py" "${ARGS[@]}"
printf '\n\n'

"${PYTHON_BIN}" "${PROJECT_DIR}/calibrate_tool_offset.py" "${ARGS[@]}"

echo
echo "[完成] 请将输出结果复制到config/tool_offsets.yaml的tools:下。"
echo "保存后下一次/get_tcp_pose会热加载新值，无需重启服务。"
