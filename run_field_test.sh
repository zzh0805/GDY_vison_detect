#!/usr/bin/env bash
# 用法示例：
#   bash run_field_test.sh --live --base-label 5 --code 9-8-1
#   bash run_field_test.sh --live --base-label 5 --code 9-8-1 --approach-x-mm -200
# --code在use_yolo=false时指定本次工件；也可写入test_case.yaml的case.code。
# --approach-x-mm临时覆盖test_case.yaml中的基座X预备位偏移。
set -euo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${PROJECT_DIR}/load_sdk_env.sh"
cd "$PROJECT_DIR"
exec "$PYTHON_BIN" field_test/02_run_labelme_test.py "$@"
