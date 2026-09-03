# -*- coding: utf-8 -*-
"""现场测试使用的极简HTTP客户端；调用方式与后台一致。"""
from __future__ import annotations

import json
from typing import Any, Sequence
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


class VisionHttpClient:
    def __init__(self, base_url: str, timeout_s: float = 15.0):
        self.base_url = str(base_url).rstrip("/")
        self.timeout_s = float(timeout_s)

    def _post(self, path: str, payload: Any = None) -> dict:
        data = (None if payload is None else json.dumps(
            payload, ensure_ascii=False).encode("utf-8"))
        headers = ({"Content-Type": "application/json;charset=UTF-8"}
                   if data is not None else {})
        request = Request(
            self.base_url + path, data=(data if data is not None else b""),
            headers=headers, method="POST")
        try:
            with urlopen(request, timeout=self.timeout_s) as response:
                raw = response.read()
        except HTTPError as exc:
            raw = exc.read()
        except URLError as exc:
            raise ConnectionError(f"无法连接视觉服务{self.base_url}: {exc}") from exc
        result = json.loads(raw.decode("utf-8"))
        if not isinstance(result, dict):
            raise RuntimeError("视觉服务响应不是JSON对象")
        return result

    def snapshot(self) -> dict:
        return self._post("/snapshot")

    def get_tcp_pose(self, pos_mm_rpy_rad: Sequence[float],
                     x1: float, y1: float, x2: float, y2: float,
                     *, base: Any = None, live: bool = False) -> dict:
        """解算目标TCP。
        - target 矩形由 x1,y1,x2,y2 给出；
        - base：基座面板矩形 dict {"x1","y1","x2","y2"}（可选）。
          提供时目标中心深度以 base 面板平面为准（每次检测以基座深度为准）；
        - live=True时实时采集当前帧（拍照+检测+解算一体）。
        """
        payload = {
            "pos": list(pos_mm_rpy_rad),
            "target": {
                "x1": float(x1), "y1": float(y1),
                "x2": float(x2), "y2": float(y2),
            },
        }
        if base is not None:
            payload["base"] = {
                "x1": float(base["x1"]), "y1": float(base["y1"]),
                "x2": float(base["x2"]), "y2": float(base["y2"]),
            }
        if live:
            payload["live"] = True
        return self._post("/get_tcp_pose", payload)
