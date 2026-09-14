# -*- coding: utf-8 -*-
"""最新机械臂末端视觉HTTP协议到现有视觉解算核心的薄适配层。"""
from __future__ import annotations

import math
import time
import uuid
from typing import Any, Mapping

import numpy as np


ERROR_CODE_MAP = {
    "INVALID_REQUEST": 400,
    "INVALID_JSON": 400,
    "UNSUPPORTED_TASK": 400,
    "SNAPSHOT_REQUIRED": 409,
    "SNAPSHOT_EXPIRED": 409,
    "NO_YOLO_TARGET": 404,
    "NO_CIRCLE_TARGET": 404,
    "TARGET_TOO_FAR": 404,
    "TARGET_GEOMETRY_FAILED": 422,
    "PLANE_RMS_TOO_HIGH": 422,
    "TOOL_MAPPING_MISSING": 422,
    "TOOL_CODE_UNKNOWN": 422,
    "TOOL_DISABLED": 422,
    "TOOL_CONFIG_RELOAD_FAILED": 500,
    "CAMERA_3D_MISSING": 500,
    "CAMERA_COLOR_MISSING": 500,
    "TASK_TIMEOUT": 504,
}


def _task_id(prefix: str) -> str:
    stamp = time.strftime("%Y%m%d%H%M%S")
    return f"{prefix}-{stamp}-{uuid.uuid4().hex[:8]}"


def _finite_number(value: Any, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name}必须是数字") from exc
    if not math.isfinite(result):
        raise ValueError(f"{name}不能是NaN/Inf")
    return result


class VisionHttpProtocol:
    """公开拍照和TCP解算所需的最小HTTP边界。"""

    def __init__(self, solver: Any):
        self.solver = solver
        self.timeout_s = float(
            solver.config.http.get("request_timeout_s", 15.0))

    @staticmethod
    def _failure(result: Mapping[str, Any]) -> dict:
        internal = str(result.get("errorCode") or "INTERNAL_ERROR")
        return {
            "code": int(ERROR_CODE_MAP.get(internal, 500)),
            "status": str(result.get("message") or internal),
        }

    def snapshot(self) -> dict:
        result = self.solver.handle_task({
            "taskType": "http_snapshot",
            "taskId": _task_id("snapshot"),
        }, timeout_s=self.timeout_s)
        if not result.get("ok"):
            return self._failure(result)
        return {"code": 200, "path": str(result["imagePath"])}

    def capture_color(self, payload: Mapping[str, Any]) -> dict:
        """只采集彩色图；不创建或修改HTTP三维快照缓存。"""
        if not isinstance(payload, Mapping):
            return {"code": 400, "status": "请求体必须是JSON对象"}
        dataset = payload.get("dataset", "datas_get")
        file_name = payload.get("fileName", "")
        if not isinstance(dataset, str) or not dataset.strip():
            return {"code": 400, "status": "dataset必须是非空字符串"}
        if not isinstance(file_name, str):
            return {"code": 400, "status": "fileName必须是字符串"}
        try:
            discard_s = _finite_number(
                payload.get("discardStaleFramesS", 0.0),
                "discardStaleFramesS")
            if discard_s < 0.0 or discard_s > 10.0:
                raise ValueError("discardStaleFramesS必须在0到10秒之间")
        except ValueError as exc:
            return {"code": 400, "status": str(exc)}
        result = self.solver.handle_task({
            "taskType": "capture_annotation_image",
            "taskId": _task_id("color"),
            "datasetName": dataset.strip(),
            "fileName": file_name.strip(),
            "discardStaleFramesS": discard_s,
        }, timeout_s=self.timeout_s)
        if not result.get("ok"):
            return self._failure(result)
        return {"code": 200, "path": str(result["imagePath"])}

    def capture_dataset_reference(self, payload: Mapping[str, Any]) -> dict:
        """数据采集专用三维参考入口；现有生产接口及响应保持不变。"""
        if not isinstance(payload, Mapping):
            return {"code": 400, "status": "请求体必须是JSON对象"}
        try:
            dataset = payload.get("dataset", "datas_get")
            file_name = payload.get("fileName", "reference.png")
            if not isinstance(dataset, str) or not dataset.strip():
                raise ValueError("dataset必须是非空字符串")
            if not isinstance(file_name, str):
                raise ValueError("fileName必须是字符串")
            matching = self.solver.config.matching
            width = int(matching.get("source_image_width", 1920))
            height = int(matching.get("source_image_height", 1080))
            roi_corners = self._parse_rect(
                payload.get("roi"), "roi", width, height)
            discard_s = _finite_number(
                payload.get("discardStaleFramesS", 0.0),
                "discardStaleFramesS")
            if discard_s < 0.0 or discard_s > 10.0:
                raise ValueError("discardStaleFramesS必须在0到10秒之间")
            geometry = payload.get("geometry", {})
            if not isinstance(geometry, Mapping):
                raise ValueError("geometry必须是JSON对象")
            allowed = {
                "plane_expand_ratio", "exclude_roi_margin_ratio",
                "min_plane_points", "max_plane_sample_points",
                "ransac_threshold_mm", "ransac_iterations",
                "min_inlier_ratio", "max_plane_rms_mm",
            }
            unknown = set(geometry) - allowed
            if unknown:
                raise ValueError(
                    f"geometry包含未知参数{sorted(unknown)!r}")
            geometry = {
                str(key): _finite_number(value, f"geometry.{key}")
                for key, value in geometry.items()
            }
        except (TypeError, ValueError) as exc:
            return {"code": 400, "status": str(exc)}
        result = self.solver.handle_task({
            "taskType": "capture_dataset_reference",
            "taskId": _task_id("dataset-reference"),
            "datasetName": dataset.strip(),
            "fileName": file_name.strip(),
            "roiCornersPx": roi_corners,
            "discardStaleFramesS": discard_s,
            "geometry": geometry,
        }, timeout_s=self.timeout_s)
        if not result.get("ok"):
            return self._failure(result)
        return {
            "code": 200,
            "path": str(result["imagePath"]),
            "targetCameraMm": result["targetCameraMm"],
            "roiCornersCameraMm": result["roiCornersCameraMm"],
            "planeNormalCamera": result["planeNormalCamera"],
            "quality": result["quality"],
        }

    @staticmethod
    def _parse_rect(rect: Any, name: str, width: int, height: int) -> list:
        if not isinstance(rect, Mapping):
            raise ValueError(f"{name}必须是包含x1/y1/x2/y2的对象")
        x1 = _finite_number(rect.get("x1"), f"{name}.x1")
        y1 = _finite_number(rect.get("y1"), f"{name}.y1")
        x2 = _finite_number(rect.get("x2"), f"{name}.x2")
        y2 = _finite_number(rect.get("y2"), f"{name}.y2")
        left, right = sorted((x1, x2))
        top, bottom = sorted((y1, y2))
        if right <= left or bottom <= top:
            raise ValueError(f"{name}矩形宽和高必须大于0")
        if left < 0 or top < 0 or right >= width or bottom >= height:
            raise ValueError(
                f"{name}框选坐标必须位于{width}x{height}原图范围内")
        return [[left, top], [right, top], [right, bottom], [left, bottom]]

    def _solve_tcp_pose(self, payload: Mapping[str, Any]) -> tuple[Any, dict]:
        """执行一次TCP解算，同时保留正式HTTP响应和内部完整结果。"""
        if not isinstance(payload, Mapping):
            return None, {"code": 400, "status": "请求体必须是JSON对象"}
        try:
            pos = np.asarray(payload.get("pos"), dtype=np.float64)
            if pos.size != 6:
                raise ValueError("pos必须包含6个数")
            pos = pos.reshape(6)
            if not np.isfinite(pos).all():
                raise ValueError("pos不能包含NaN/Inf")
            matching = self.solver.config.matching
            width = int(matching.get("source_image_width", 1920))
            height = int(matching.get("source_image_height", 1080))
            # 目标矩形：优先 target 键值对；兼容旧 x1/y1/x2/y2 字段。
            target_rect = payload.get("target")
            if target_rect is None:
                target_rect = {
                    "x1": payload.get("x1"), "y1": payload.get("y1"),
                    "x2": payload.get("x2"), "y2": payload.get("y2"),
                }
            target_corners = self._parse_rect(
                target_rect, "target", width, height)
            # 基座面板矩形：可选。提供时目标中心深度以面板平面为准。
            base_corners = None
            if payload.get("base") is not None:
                base_corners = self._parse_rect(
                    payload["base"], "base", width, height)
            workpiece_code = None
            raw_code = payload.get("code")
            if raw_code is not None:
                if not isinstance(raw_code, str) or not raw_code.strip():
                    raise ValueError("code必须是非空字符串")
                workpiece_code = raw_code.strip()
        except (TypeError, ValueError) as exc:
            return None, {"code": 400, "status": str(exc)}

        # HTTP边界是mm+RPY(rad)；现有且已现场标定的核心保持mm+RPY(deg)。
        internal_pos = pos.copy()
        internal_pos[3:] = np.degrees(internal_pos[3:])
        # live=true时实时采集当前帧拍照+检测+解算一体（不依赖/snapshot缓存）；
        # 默认false保持原两步流程（先用/snapshot拍照，再复用缓存帧解算）。
        use_live = bool(payload.get("live", False))
        task = {
            "taskType": "solve_target_tcp",
            "taskId": _task_id("solve"),
            "captureTcpMmRpyDeg": internal_pos.tolist(),
            "targetCornersPx": target_corners,
            "useCachedHttpSnapshot": not use_live,
        }
        if workpiece_code is not None:
            task["workpieceCode"] = workpiece_code
        if base_corners is not None:
            task["baseCornersPx"] = base_corners
        result = self.solver.handle_task(task, timeout_s=self.timeout_s)
        if not result.get("ok"):
            return None, self._failure(result)
        target = np.asarray(
            result["targetTcpMmRpyDeg"], dtype=np.float64).reshape(6)
        target[3:] = np.radians(target[3:])
        # 成功响应严格只有code和pos。
        return result, {"code": 200, "pos": target.tolist()}

    def get_tcp_pose(self, payload: Mapping[str, Any]) -> dict:
        """正式后台接口；成功响应继续严格保持code和pos两个字段。"""
        _, response = self._solve_tcp_pose(payload)
        return response

    def get_tcp_pose_with_approach(self, payload: Mapping[str, Any]) -> dict:
        """运动脚本专用入口，额外返回当次柜体法向进入方向。

        该入口与正式/get_tcp_pose分离，避免改变既有后台协议。方向采用
        pose_solver已经转换到JAKA基座系的approachDirectionBase。
        """
        result, response = self._solve_tcp_pose(payload)
        if result is None:
            return response
        try:
            geometry = result.get("geometry")
            if not isinstance(geometry, Mapping):
                raise ValueError("解算结果缺少geometry")
            direction = np.asarray(
                geometry.get("approachDirectionBase"),
                dtype=np.float64).reshape(3)
            norm = float(np.linalg.norm(direction))
            if not np.isfinite(direction).all() or not math.isfinite(norm) or \
                    norm < 1e-9:
                raise ValueError("approachDirectionBase不是有效方向")
            direction /= norm
        except (TypeError, ValueError) as exc:
            return {
                "code": 422,
                "status": f"无法生成安全进入方向: {exc}",
            }
        return {
            "code": 200,
            "pos": response["pos"],
            "approachDirectionBase": direction.tolist(),
            "approachDirectionSource": "fitted_panel_normal",
        }
