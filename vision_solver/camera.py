# -*- coding: utf-8 -*-
"""复用旧项目取流方式的 SurfacePro50 长生命周期包装。"""
from __future__ import annotations

import os
import threading
import time
from typing import Any, Optional

import numpy as np

from handeye_calib.approach.models import CameraModel
from handeye_calib.hardware_interfaces import CameraFrame

from .config import AppConfig
from .models import TaskError


class CameraManager:
    def __init__(self, config: AppConfig, adapter: Optional[Any] = None):
        self.config = config
        self.camera_config = config.camera
        self._lock = threading.RLock()
        self.adapter = adapter
        self._owns_adapter = adapter is None

    def connect(self) -> None:
        with self._lock:
            if self.adapter is None:
                calibration_path = self.config.resolve_path(
                    self.camera_config.get("calibration_file"), must_exist=True)
                os.environ["SURFACEPRO50_CALIBRATION_YAML"] = str(calibration_path)
                python_path = str(
                    self.camera_config.get("python_path") or "").strip()
                openni_redist = str(
                    self.camera_config.get("openni_redist") or "").strip()
                if python_path:
                    os.environ["SURFACEPRO50_PYTHON_PATH"] = python_path
                if openni_redist:
                    os.environ["OPENNI2_REDIST"] = openni_redist
                os.environ["SURFACEPRO50_SHOW_FRAME_OUTPUT"] = (
                    "1" if bool(self.camera_config.get(
                        "show_driver_frame_output", False)) else "0")
                from handeye_calib.surfacepro50_adapter import (
                    SurfacePro50SyncAdapter)
                self.adapter = SurfacePro50SyncAdapter(
                    endpoint=str(self.camera_config.get("ip", "auto")),
                    capture_3d=True,
                    openni_redist=openni_redist or None,
                    unload_openni_on_disconnect=bool(
                        self.camera_config.get(
                            "unload_openni_on_disconnect", False)),
                )
            if not self.adapter.is_connected():
                self.adapter.connect(str(self.camera_config.get("ip", "auto")))

    def disconnect(self) -> None:
        with self._lock:
            if self.adapter is not None and self.adapter.is_connected():
                self.adapter.disconnect()

    def capture_3d(self) -> CameraFrame:
        with self._lock:
            frame = self.adapter.capture()
        if frame.color is None or frame.point_cloud is None:
            raise TaskError(
                "CAMERA_3D_MISSING", "相机未返回彩色图或对齐点云")
        return frame

    def discard_stale_frames(self, duration_s: float) -> int:
        """在稳定等待期主动消费旧帧；不支持该能力的适配器回退为等待。"""
        duration = float(duration_s)
        if duration <= 0.0:
            return 0
        with self._lock:
            discard = getattr(self.adapter, "discard_frames", None)
            if callable(discard):
                return int(discard(duration))
            # 兼容测试相机或其他旧适配器；SurfacePro50正式后端不会走这里。
            time.sleep(duration)
            return 0

    def capture_color(self) -> CameraFrame:
        """只采集并返回原始彩色图，不保存深度/点云。"""
        with self._lock:
            capture_color = getattr(self.adapter, "capture_color", None)
            if callable(capture_color):
                frame = capture_color()
            else:
                backend = getattr(self.adapter, "backend", None)
                if backend is not None and callable(getattr(backend, "capture", None)):
                    # 原相机后端已支持 save_images=False；不改动其取流逻辑。
                    frame = backend.capture(save_images=False)
                else:
                    frame = self.adapter.capture()
        if frame.color is None:
            raise TaskError("CAMERA_COLOR_MISSING", "相机未返回彩色图")
        return frame

    def camera_model(self, frame: CameraFrame) -> CameraModel:
        intrinsics = self.adapter.get_intrinsics()
        if intrinsics is None:
            raise TaskError("CAMERA_INTRINSICS_MISSING", "无法读取彩色相机内参")
        if frame.color is None:
            raise TaskError("CAMERA_COLOR_MISSING", "当前帧没有彩色图")
        K, distortion = intrinsics
        height, width = np.asarray(frame.color).shape[:2]
        metadata = dict(frame.metadata or {})
        return CameraModel(
            K=K, distortion=distortion,
            width=width, height=height,
            image_frame=str(metadata.get(
                "calibration_camera_frame", frame.camera_frame)),
            profile=str(metadata.get("image_profile", "")),
            intrinsics_source=str(metadata.get("intrinsics_source", "camera_adapter")),
        )
