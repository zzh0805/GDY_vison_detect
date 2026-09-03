#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${PROJECT_DIR}/load_sdk_env.sh"
cd "$PROJECT_DIR"
exec "$PYTHON_BIN" field_test/01_capture_test_case.py "$@"

