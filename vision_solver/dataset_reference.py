# -*- coding: utf-8 -*-
"""数据采集专用ROI三维参考点估计；不参与正常检测/TCP解算。"""
from __future__ import annotations

from typing import Any, Mapping

import cv2
import numpy as np

from feature_geometry_fit_v1_0 import FeatureGeometryFitter


def _number(settings: Mapping[str, Any], key: str, default: float,
            *, minimum: float | None = None) -> float:
    value = float(settings.get(key, default))
    if not np.isfinite(value) or (minimum is not None and value < minimum):
        raise ValueError(f"geometry.{key}参数无效")
    return value


def _pixel_rays(pixels: np.ndarray, K: np.ndarray,
                distortion: np.ndarray) -> np.ndarray:
    normalized = cv2.undistortPoints(
        np.asarray(pixels, dtype=np.float64).reshape(-1, 1, 2),
        np.asarray(K, dtype=np.float64).reshape(3, 3),
        np.asarray(distortion, dtype=np.float64).reshape(1, -1))
    return np.c_[normalized.reshape(-1, 2), np.ones(len(normalized))]


def estimate_roi_reference(frame: Any, camera_model: Any,
                           roi_corners_px: Any,
                           settings: Mapping[str, Any] | None = None) -> dict:
    """用ROI外围安装面点云确定中心射线和平面的真实交点。

    返回坐标均位于彩色相机光学坐标系，长度单位为mm。该函数只接受已经
    对齐到彩色图像、且与手眼矩阵兼容的有组织点云，避免静默混用坐标系。
    """
    settings = dict(settings or {})
    color = np.asarray(frame.color)
    cloud = np.asarray(frame.point_cloud)
    if color.ndim != 3 or color.shape[2] < 3:
        raise ValueError("三维参考帧没有有效彩色图")
    if cloud.ndim != 3 or cloud.shape[2] < 3:
        raise ValueError(f"点云必须为HxWx3，实际{cloud.shape}")
    if cloud.shape[:2] != color.shape[:2]:
        raise ValueError(
            f"点云{cloud.shape[1]}x{cloud.shape[0]}没有逐像素对齐彩色图"
            f"{color.shape[1]}x{color.shape[0]}")
    metadata = dict(frame.metadata or {})
    if metadata.get("point_cloud_pixel_aligned_to_image") is False:
        raise ValueError("点云元数据声明未对齐彩色图")
    if metadata.get("point_cloud_handeye_compatible") is False:
        raise ValueError("点云坐标系与当前手眼矩阵不兼容")
    unit = str(metadata.get("point_cloud_unit", "mm")).strip().lower()
    if unit in ("mm", "millimeter", "millimeters"):
        scale_to_mm = 1.0
    elif unit in ("m", "meter", "meters"):
        scale_to_mm = 1000.0
    else:
        raise ValueError(f"不支持的点云单位{unit!r}")

    corners = np.asarray(roi_corners_px, dtype=np.float64).reshape(4, 2)
    if not np.isfinite(corners).all():
        raise ValueError("ROI角点包含NaN/Inf")
    height, width = color.shape[:2]
    left, top = np.min(corners, axis=0)
    right, bottom = np.max(corners, axis=0)
    if not (0.0 <= left < right < width and 0.0 <= top < bottom < height):
        raise ValueError("ROI必须完整位于彩色图范围内")

    expand = _number(settings, "plane_expand_ratio", 0.7, minimum=0.05)
    exclude = _number(
        settings, "exclude_roi_margin_ratio", 0.08, minimum=0.0)
    box_width = right - left
    box_height = bottom - top
    ox1 = max(0, int(np.floor(left - expand * box_width)))
    oy1 = max(0, int(np.floor(top - expand * box_height)))
    ox2 = min(width, int(np.ceil(right + expand * box_width)) + 1)
    oy2 = min(height, int(np.ceil(bottom + expand * box_height)) + 1)
    ex1 = left - exclude * box_width
    ey1 = top - exclude * box_height
    ex2 = right + exclude * box_width
    ey2 = bottom + exclude * box_height

    crop = np.asarray(cloud[oy1:oy2, ox1:ox2, :3], dtype=np.float64)
    yy, xx = np.mgrid[oy1:oy2, ox1:ox2]
    outside_roi = ~(
        (xx >= ex1) & (xx <= ex2) & (yy >= ey1) & (yy <= ey2))
    valid = (
        outside_roi & np.isfinite(crop).all(axis=2) & (crop[:, :, 2] > 0.0))
    points_mm = crop[valid] * scale_to_mm
    min_points = int(_number(settings, "min_plane_points", 300, minimum=3))
    if len(points_mm) < min_points:
        raise ValueError(
            f"ROI外围有效安装面点云不足: {len(points_mm)} < {min_points}")

    max_points = int(_number(
        settings, "max_plane_sample_points", 20000, minimum=min_points))
    fit_points = points_mm
    if len(fit_points) > max_points:
        indices = np.random.default_rng(0).choice(
            len(fit_points), max_points, replace=False)
        fit_points = fit_points[indices]
    threshold = _number(
        settings, "ransac_threshold_mm", 3.0, minimum=0.05)
    min_ratio = _number(settings, "min_inlier_ratio", 0.35, minimum=0.01)
    if min_ratio > 1.0:
        raise ValueError("geometry.min_inlier_ratio不能大于1")
    iterations = int(_number(
        settings, "ransac_iterations", 600, minimum=10))
    fit = FeatureGeometryFitter.fit_plane_ransac(
        fit_points, distance_threshold_mm=threshold,
        max_iterations=iterations, min_inlier_ratio=min_ratio,
        random_seed=0)
    rms = float(fit["rms_mm"])
    max_rms = _number(settings, "max_plane_rms_mm", 3.0, minimum=0.05)
    if rms > max_rms:
        raise ValueError(f"ROI安装面RMS={rms:.3f}mm超过{max_rms:.3f}mm")

    plane_center = np.asarray(fit["center_camera_mm"], dtype=np.float64)
    normal = np.asarray(fit["normal_camera"], dtype=np.float64)
    normal /= np.linalg.norm(normal)
    # FeatureGeometryFitter已朝向相机；这里再次防御第三方实现差异。
    if float(np.dot(normal, -plane_center)) < 0.0:
        normal = -normal

    center_px = np.mean(corners, axis=0, keepdims=True)
    rays = _pixel_rays(
        np.vstack([center_px, corners]), camera_model.K,
        camera_model.distortion)
    denominator = rays @ normal
    if np.any(np.abs(denominator) < 1e-8):
        raise ValueError("ROI像素射线与安装面近似平行")
    distances = float(np.dot(normal, plane_center)) / denominator
    if not np.isfinite(distances).all() or np.any(distances <= 0.0):
        raise ValueError("ROI射线与安装面的交点不在相机前方")
    intersections = rays * distances[:, None]

    return {
        "targetCameraMm": intersections[0],
        "roiCornersCameraMm": intersections[1:],
        "planeNormalCamera": normal,
        "quality": {
            "source": "aligned_point_cloud_surrounding_plane",
            "pointCloudUnit": unit,
            "surroundingPointCount": int(len(points_mm)),
            "planeSampleCount": int(len(fit_points)),
            "planeInlierCount": int(fit["inlier_count"]),
            "planeInlierRatio": float(fit["inlier_ratio"]),
            "planeRmsMm": rms,
            "planeDistanceThresholdMm": threshold,
            "targetOpticalDepthMm": float(intersections[0, 2]),
            "outerRegionPx": [ox1, oy1, ox2 - 1, oy2 - 1],
        },
    }
