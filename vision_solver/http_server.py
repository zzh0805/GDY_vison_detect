# -*- coding: utf-8 -*-
"""机械臂末端视觉HTTP服务，仅提供/snapshot和/get_tcp_pose。"""
from __future__ import annotations

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
from urllib.parse import urlsplit

from .logging_utils import get_logger


MAX_REQUEST_BODY = 1024 * 1024

log = get_logger()


def _round_pos(pos) -> str:
    """把返回的 pos 列表格式化为简洁字符串（容错 None/非列表）。"""
    if not isinstance(pos, (list, tuple)):
        return str(pos)
    try:
        return "[" + ", ".join(f"{float(v):.3f}" for v in pos) + "]"
    except (TypeError, ValueError):
        return str(pos)


class _ReusableThreadingHttpServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True


class VisionHttpServer:
    def __init__(self, protocol: Any, host: str, port: int):
        self.protocol = protocol
        self.host = str(host)
        self.port = int(port)
        self._stop_lock = threading.Lock()
        self._stopped = False
        self._stop_complete = threading.Event()
        self._server = _ReusableThreadingHttpServer(
            (self.host, self.port), self._handler_class())

    @property
    def server_address(self):
        return self._server.server_address

    def _handler_class(self):
        protocol = self.protocol

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def log_message(self, format_string, *args):
                log.debug("HTTP %s - %s", self.address_string(),
                           format_string % args)

            def _send_json(self, payload: dict, http_status: int = 200):
                body = json.dumps(
                    payload, ensure_ascii=False,
                    separators=(",", ":")).encode("utf-8")
                self.send_response(http_status)
                self.send_header(
                    "Content-Type", "application/json;charset=UTF-8")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Connection", "close")
                self.end_headers()
                self.wfile.write(body)

            def _json_body(self):
                raw_length = self.headers.get("Content-Length", "0")
                try:
                    length = int(raw_length)
                except ValueError as exc:
                    raise ValueError("Content-Length无效") from exc
                if length <= 0:
                    raise ValueError("请求体不能为空")
                if length > MAX_REQUEST_BODY:
                    raise ValueError("请求体过大")
                raw = self.rfile.read(length)
                try:
                    return json.loads(raw.decode("utf-8"))
                except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                    raise ValueError(f"JSON解析失败: {exc}") from exc

            def do_POST(self):
                path = urlsplit(self.path).path.rstrip("/") or "/"
                started = time.perf_counter()
                client = self.address_string()
                try:
                    if path == "/snapshot":
                        result = protocol.snapshot()
                        self._send_json(result)
                        log.info("HTTP /snapshot %s 耗时=%.1fms code=%s",
                                 client, (time.perf_counter() - started) * 1000.0,
                                 result.get("code"))
                        return
                    if path == "/get_tcp_pose":
                        result = protocol.get_tcp_pose(self._json_body())
                        self._send_json(result)
                        code = result.get("code")
                        if code == 200:
                            log.info("HTTP /get_tcp_pose %s 耗时=%.1fms code=200 "
                                     "pos=%s", client,
                                     (time.perf_counter() - started) * 1000.0,
                                     _round_pos(result.get("pos")))
                        else:
                            log.error("HTTP /get_tcp_pose %s 耗时=%.1fms code=%s "
                                      "status=%s", client,
                                      (time.perf_counter() - started) * 1000.0,
                                      code, result.get("status"))
                        return
                    self._send_json(
                        {"code": 404, "status": f"未知接口{path}"}, 404)
                except Exception as exc:
                    # 单个HTTP处理器异常不能终止长期视觉服务。
                    log.error("HTTP %s 处理异常 %s: %s",
                              client, path, exc, exc_info=True)
                    self._send_json({"code": 500, "status": str(exc)})

        return Handler

    def serve_forever(self) -> None:
        address = self.server_address
        log.info("视觉HTTP服务已监听 %s:%s", address[0], address[1])
        self._server.serve_forever(poll_interval=0.5)

    def stop(self) -> None:
        with self._stop_lock:
            if self._stopped:
                owner = False
            else:
                self._stopped = True
                owner = True
        if not owner:
            self._stop_complete.wait(timeout=5.0)
            return
        try:
            self._server.shutdown()
            self._server.server_close()
        finally:
            self._stop_complete.set()

    def stop_async(self) -> None:
        threading.Thread(target=self.stop, daemon=True).start()
