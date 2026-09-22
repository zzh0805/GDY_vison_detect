# -*- coding: utf-8 -*-
"""原始彩色图、调试图和JSON报告保存。"""
from __future__ import annotations

import json
import os
import re
import threading
import time
import uuid
from datetime import datetime
from pathlib import Path
from typing import Any, Mapping

import cv2
import numpy as np

from .config import AppConfig
from .models import TaskError, json_ready


def safe_name(value: Any, fallback: str = "task") -> str:
    text = re.sub(r"[^0-9A-Za-z_.-]+", "_", str(value or "").strip())
    text = text.strip("._")
    return text or fallback


def write_bgr(path: Path, image: np.ndarray,
              jpeg_quality: int = 95) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    suffix = path.suffix.lower()
    if suffix in (".jpg", ".jpeg"):
        parameters = [cv2.IMWRITE_JPEG_QUALITY, int(jpeg_quality)]
        extension = ".jpg"
    elif suffix == ".png":
        parameters = [cv2.IMWRITE_PNG_COMPRESSION, 3]
        extension = ".png"
    else:
        raise TaskError("INVALID_IMAGE_FORMAT", f"不支持图片格式{suffix!r}")
    ok, encoded = cv2.imencode(extension, np.asarray(image), parameters)
    if not ok:
        raise TaskError("IMAGE_SAVE_FAILED", f"彩色图编码失败: {path}")
    path.write_bytes(encoded.tobytes())


def write_bgr_atomic(path: Path, image: np.ndarray,
                     jpeg_quality: int = 95) -> None:
    """完整编码后原子替换，确保接口返回路径时图片已经写完。"""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    suffix = path.suffix.lower()
    if suffix in (".jpg", ".jpeg"):
        parameters = [cv2.IMWRITE_JPEG_QUALITY, int(jpeg_quality)]
        extension = ".jpg"
    elif suffix == ".png":
        parameters = [cv2.IMWRITE_PNG_COMPRESSION, 3]
        extension = ".png"
    else:
        raise TaskError("INVALID_IMAGE_FORMAT", f"不支持图片格式{suffix!r}")
    ok, encoded = cv2.imencode(extension, np.asarray(image), parameters)
    if not ok:
        raise TaskError("IMAGE_SAVE_FAILED", f"彩色图编码失败: {path}")
    temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    try:
        temporary.write_bytes(encoded.tobytes())
        os.replace(temporary, path)
    finally:
        try:
            temporary.unlink(missing_ok=True)
        except OSError:
            pass


class ArtifactWriter:
    def __init__(self, config: AppConfig):
        self.config = config
        self._snapshot_name_lock = threading.Lock()

    @staticmethod
    def _relative_component(value: str, name: str) -> Path:
        path = Path(value or ".")
        if path.is_absolute() or ".." in path.parts:
            raise TaskError("INVALID_SAVE_PATH", f"{name}必须是不含..的相对路径")
        return path

    def save_annotation(self, frame: Any, request: Any) -> dict:
        cfg = self.config.annotation
        root = self.config.resolve_path(cfg.get("root_directory"))
        dataset = self._relative_component(request.dataset_name, "datasetName")
        target_dir = (root / dataset).resolve()
        if root != target_dir and root not in target_dir.parents:
            raise TaskError("INVALID_SAVE_PATH", "标注图片路径越界")
        configured_format = str(cfg.get("image_format", "png")).lower().lstrip(".")
        extension = "jpg" if configured_format in ("jpg", "jpeg") else "png"
        if request.file_name:
            requested = self._relative_component(request.file_name, "fileName")
            if len(requested.parts) != 1:
                raise TaskError("INVALID_SAVE_PATH", "fileName只能是文件名")
            file_name = safe_name(requested.stem, "capture") + requested.suffix.lower()
            if Path(file_name).suffix.lower() not in (".png", ".jpg", ".jpeg"):
                file_name = safe_name(requested.stem, "capture") + "." + extension
        else:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            file_name = f"{timestamp}_{safe_name(request.task_id)}.{extension}"
        image_path = target_dir / file_name
        write_bgr(
            image_path, frame.color,
            jpeg_quality=int(cfg.get("jpeg_quality", 95)))
        result = {
            "imagePath": str(image_path),
            "imageWidth": int(np.asarray(frame.color).shape[1]),
            "imageHeight": int(np.asarray(frame.color).shape[0]),
            "capturedAtUnixS": float(frame.timestamp),
            "frameId": int(frame.frame_id),
        }
        if bool(cfg.get("save_metadata_json", True)):
            metadata_path = image_path.with_suffix(".metadata.json")
            metadata_path.write_text(json.dumps(
                json_ready({
                    "taskId": request.task_id,
                    **result,
                    "cameraMetadata": dict(frame.metadata or {}),
                }), ensure_ascii=False, indent=2), encoding="utf-8")
            result["metadataPath"] = str(metadata_path)
        return result

    def save_http_snapshot(self, frame: Any) -> dict:
        """按yyyyMMddHHmmss.jpg保存HTTP快照并返回绝对路径。
        http.save_snapshot=false 时快照不落盘，imagePath 返回空。"""
        cfg = self.config.http
        image_path = ""
        if bool(cfg.get("save_snapshot", True)):
            root = self.config.resolve_path(
                cfg.get("snapshot_directory", "../shared_images"))
            root.mkdir(parents=True, exist_ok=True)
            quality = int(cfg.get("jpeg_quality", 95))
            with self._snapshot_name_lock:
                while True:
                    stamp = datetime.now().strftime("%Y%m%d%H%M%S")
                    image_path = root / f"{stamp}.jpg"
                    if not image_path.exists():
                        break
                    # 协议要求文件名只到秒；同秒请求等待下一秒，避免覆盖。
                    time.sleep(0.05)
                write_bgr_atomic(image_path, frame.color,
                                 jpeg_quality=quality)
        return {
            "imagePath": (str(image_path.resolve()) if image_path else ""),
            "imageWidth": int(np.asarray(frame.color).shape[1]),
            "imageHeight": int(np.asarray(frame.color).shape[0]),
            "capturedAtUnixS": float(frame.timestamp),
            "frameId": int(frame.frame_id),
        }

    def task_output_dir(self, task_id: str) -> Path:
        cfg = self.config.system
        root = self.config.resolve_path(
            cfg.get("output_directory", "../output"))
        path = root / safe_name(task_id)
        path.mkdir(parents=True, exist_ok=True)
        return path

    def save_result(self, task_id: str, result: Mapping[str, Any],
                    overlay: Any = None) -> dict:
        if not bool(self.config.system.get("save_report", True)):
            return {}
        output_dir = self.task_output_dir(task_id)
        report_path = output_dir / "result.json"
        report_path.write_text(
            json.dumps(json_ready(result), ensure_ascii=False, indent=2),
            encoding="utf-8")
        saved = {"reportPath": str(report_path)}
        if overlay is not None and bool(
                self.config.system.get("save_debug_image", True)):
            overlay_path = output_dir / "detection_overlay.png"
            write_bgr(overlay_path, overlay)
            saved["debugImagePath"] = str(overlay_path)
        return saved
