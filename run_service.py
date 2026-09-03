# -*- coding: utf-8 -*-
"""正式持续视觉HTTP解算服务入口。"""
from __future__ import annotations

import argparse
import signal
import sys
from pathlib import Path

from vision_tool_tcp_api import VisionToolTcpSolver
from vision_solver.config import load_config
from vision_solver.http_protocol import VisionHttpProtocol
from vision_solver.http_server import VisionHttpServer
from vision_solver.logging_utils import get_logger, setup_logging


PROJECT_ROOT = Path(__file__).resolve().parent
DEFAULT_CONFIG = PROJECT_ROOT / "config" / "workflow.yaml"

log = get_logger()


def main() -> int:
    parser = argparse.ArgumentParser(description="SurfacePro50目标TCP解算服务")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    args = parser.parse_args()
    config = load_config(args.config)
    log_cfg = dict(config.system.get("log") or {})
    log_cfg.setdefault("file", str(PROJECT_ROOT / "service.log"))
    setup_logging(PROJECT_ROOT, log_cfg)
    http = config.http
    log.info("启动视觉解算服务: config=%s 端口=%s",
             config.source_path, http.get("listen_port"))
    with VisionToolTcpSolver(args.config) as solver:
        protocol = VisionHttpProtocol(solver)
        server = VisionHttpServer(
            protocol,
            host=str(http.get("listen_host", "0.0.0.0")),
            port=int(http.get("listen_port", 48051)))
        for signal_name in (signal.SIGINT, signal.SIGTERM):
            signal.signal(signal_name, lambda *_args: server.stop_async())
        try:
            server.serve_forever()
        finally:
            server.stop()
    log.info("服务退出")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        log.error("服务启动失败: %s", exc, exc_info=True)
        raise SystemExit(1) from exc
