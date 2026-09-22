# -*- coding: utf-8 -*-
"""真实/模拟机器人和纯彩色拍照客户端。"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from handeye_calib.jaka_adapter import unified_pose_to_jaka
from vision_solver.http_client import VisionHttpClient


def robot_pose_mm_rpy_deg(robot: Any) -> np.ndarray:
    pose = unified_pose_to_jaka(robot.get_tcp_pose())
    pose[3:] = np.degrees(pose[3:])
    return pose


@dataclass(frozen=True)
class ColorCaptureResult:
    path: Path


@dataclass(frozen=True)
class ReferenceCaptureResult:
    path: Path
    target_camera_mm: np.ndarray
    roi_corners_camera_mm: np.ndarray
    plane_normal_camera: np.ndarray
    quality: dict


class HttpColorCaptureClient:
    def __init__(self, base_url: str, timeout_s: float,
                 discard_stale_frames_s: float = 0.0):
        self.client = VisionHttpClient(base_url, timeout_s=timeout_s)
        self.discard_stale_frames_s = float(discard_stale_frames_s)

    def capture(self, dataset: str, file_name: str) -> ColorCaptureResult:
        result = self.client.capture_color(
            dataset, file_name,
            discard_stale_frames_s=self.discard_stale_frames_s)
        if int(result.get("code", 0)) != 200:
            raise RuntimeError(f"/capture_color失败: {result}")
        path = Path(str(result.get("path", ""))).expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(f"服务已返回成功，但本机无法读取图片: {path}")
        return ColorCaptureResult(path)

    def capture_reference(self, dataset: str, file_name: str,
                          roi_xyxy_px: Any,
                          geometry_settings: dict,
                          **_unused: Any) -> ReferenceCaptureResult:
        result = self.client.capture_dataset_reference(
            dataset, file_name, roi_xyxy_px,
            discard_stale_frames_s=self.discard_stale_frames_s,
            geometry=geometry_settings)
        if int(result.get("code", 0)) != 200:
            raise RuntimeError(f"/datas_get/reference失败: {result}")
        path = Path(str(result.get("path", ""))).expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(f"服务已返回成功，但本机无法读取图片: {path}")
        return ReferenceCaptureResult(
            path=path,
            target_camera_mm=np.asarray(
                result.get("targetCameraMm"), dtype=np.float64).reshape(3),
            roi_corners_camera_mm=np.asarray(
                result.get("roiCornersCameraMm"),
                dtype=np.float64).reshape(4, 3),
            plane_normal_camera=np.asarray(
                result.get("planeNormalCamera"),
                dtype=np.float64).reshape(3),
            quality=dict(result.get("quality") or {}))


class SimulatedRobot:
    adapter_name = "simulated_jaka"

    def __init__(self, initial_pose_mm_rpy_deg: Any):
        self.pose = np.asarray(initial_pose_mm_rpy_deg, dtype=np.float64).reshape(6)
        self.connected = False
        self.move_history: list[np.ndarray] = []

    def connect(self, _endpoint: str | None = None) -> None:
        self.connected = True

    def disconnect(self) -> None:
        self.connected = False

    def is_connected(self) -> bool:
        return self.connected

    def get_pose_mm_rpy_deg(self) -> np.ndarray:
        if not self.connected:
            raise RuntimeError("模拟机器人未连接")
        return self.pose.copy()

    def move_to_mm_rpy_deg(self, pose: Any, name: str = "") -> None:
        del name
        if not self.connected:
            raise RuntimeError("模拟机器人未连接")
        self.pose = np.asarray(pose, dtype=np.float64).reshape(6).copy()
        self.move_history.append(self.pose.copy())


class SimulatedColorCaptureClient:
    def __init__(self, root: Path, width: int, height: int,
                 roi_xyxy_px: Any):
        self.root = Path(root)
        self.width = int(width)
        self.height = int(height)
        self.roi = np.asarray(roi_xyxy_px, dtype=np.int32).reshape(4)
        self.counter = 0

    def capture(self, dataset: str, file_name: str) -> ColorCaptureResult:
        target = self.root / dataset / file_name
        target.parent.mkdir(parents=True, exist_ok=True)
        image = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        image[:] = (32, 40, 48)
        x1, y1, x2, y2 = (int(value) for value in self.roi)
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 220, 0), 5)
        cv2.circle(image, ((x1 + x2) // 2, (y1 + y2) // 2),
                   max(8, min(x2 - x1, y2 - y1) // 5), (0, 0, 255), -1)
        cv2.putText(image, f"SIM {self.counter:04d}", (30, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 2)
        if not cv2.imwrite(str(target), image):
            raise RuntimeError(f"模拟图片保存失败: {target}")
        self.counter += 1
        return ColorCaptureResult(target.resolve())

    def capture_reference(self, dataset: str, file_name: str,
                          roi_xyxy_px: Any,
                          geometry_settings: dict,
                          *, camera_matrix: Any,
                          distortion: Any,
                          fallback_depth_mm: float) -> ReferenceCaptureResult:
        del geometry_settings
        captured = self.capture(dataset, file_name)
        x1, y1, x2, y2 = np.asarray(
            roi_xyxy_px, dtype=np.float64).reshape(4)
        pixels = np.array([
            [(x1 + x2) * 0.5, (y1 + y2) * 0.5],
            [x1, y1], [x2, y1], [x2, y2], [x1, y2],
        ], dtype=np.float64)
        normalized = cv2.undistortPoints(
            pixels.reshape(-1, 1, 2),
            np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3),
            np.asarray(distortion, dtype=np.float64).reshape(1, -1))
        rays = np.c_[normalized.reshape(-1, 2), np.ones(5)]
        intersections = rays * float(fallback_depth_mm)
        center = intersections[0]
        normal = -center / np.linalg.norm(center)
        return ReferenceCaptureResult(
            path=captured.path,
            target_camera_mm=center,
            roi_corners_camera_mm=intersections[1:],
            plane_normal_camera=normal,
            quality={
                "source": "simulation_configured_depth",
                "targetOpticalDepthMm": float(fallback_depth_mm),
            })
