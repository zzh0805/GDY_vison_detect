# -*- coding: utf-8 -*-
"""新工作流的轻量数据结构。"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping, Tuple

import numpy as np


class TaskError(RuntimeError):
    """可以安全返回给上位机的单次任务错误。"""

    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = str(code)
        self.message = str(message)


def finite_vector(value: Any, length: int, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=np.float64)
    if array.size != length:
        raise TaskError("INVALID_REQUEST", f"{name}必须包含{length}个数")
    array = array.reshape(length)
    if not np.isfinite(array).all():
        raise TaskError("INVALID_REQUEST", f"{name}包含NaN/Inf")
    return array.copy()


@dataclass(frozen=True)
class SolveTargetRequest:
    task_id: str
    capture_tcp_mm_rpy_deg: np.ndarray
    target_corners_px: np.ndarray
    # 基座面板矩形（4x2像素，可选）。提供时目标中心深度以面板平面为准。
    base_corners_px: np.ndarray | None = None
    # 无YOLO模式由上位机传入，用于动态选择tools中的工件。
    workpiece_code: str | None = None

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> "SolveTargetRequest":
        task_id = str(data.get("taskId") or "").strip()
        if not task_id:
            raise TaskError("INVALID_REQUEST", "solve_target_tcp缺少taskId")
        tcp = finite_vector(
            data.get("captureTcpMmRpyDeg"), 6, "captureTcpMmRpyDeg")
        corners = np.asarray(data.get("targetCornersPx"), dtype=np.float64)
        if corners.shape != (4, 2) or not np.isfinite(corners).all():
            raise TaskError(
                "INVALID_REQUEST", "targetCornersPx必须是4x2有限像素坐标")
        base_corners = None
        base_raw = data.get("baseCornersPx")
        if base_raw is not None:
            base_corners = np.asarray(base_raw, dtype=np.float64)
            if base_corners.shape != (4, 2) or not np.isfinite(
                    base_corners).all():
                raise TaskError(
                    "INVALID_REQUEST", "baseCornersPx必须是4x2有限像素坐标")
            base_corners = base_corners.copy()
        workpiece_code = None
        raw_code = data.get("workpieceCode")
        if raw_code is not None:
            if not isinstance(raw_code, str) or not raw_code.strip():
                raise TaskError(
                    "INVALID_REQUEST", "workpieceCode必须是非空字符串")
            workpiece_code = raw_code.strip()
        return cls(
            task_id, tcp, corners.copy(), base_corners, workpiece_code)


@dataclass(frozen=True)
class AnnotationCaptureRequest:
    task_id: str
    dataset_name: str
    file_name: str

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> "AnnotationCaptureRequest":
        task_id = str(data.get("taskId") or "").strip()
        if not task_id:
            raise TaskError(
                "INVALID_REQUEST", "capture_annotation_image缺少taskId")
        return cls(
            task_id=task_id,
            dataset_name=str(data.get("datasetName") or "").strip(),
            file_name=str(data.get("fileName") or "").strip(),
        )


@dataclass(frozen=True)
class MatchResult:
    observation: Any
    request_center_px: np.ndarray
    detection_center_px: np.ndarray
    distance_px: float
    center_inside_polygon: bool
    ordered_corners_px: np.ndarray


def json_ready(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Mapping):
        return {str(key): json_ready(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_ready(item) for item in value]
    return value
