# -*- coding: utf-8 -*-
"""后台四角点与 YOLO 检测框的二维匹配。"""
from __future__ import annotations

from typing import Iterable

import cv2
import numpy as np

from .models import MatchResult, TaskError


def order_corners(corners: np.ndarray) -> np.ndarray:
    points = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    center = np.mean(points, axis=0)
    angles = np.arctan2(points[:, 1] - center[1], points[:, 0] - center[0])
    ordered = points[np.argsort(angles)]
    area = 0.5 * abs(float(
        np.dot(ordered[:, 0], np.roll(ordered[:, 1], -1)) -
        np.dot(ordered[:, 1], np.roll(ordered[:, 0], -1))))
    if area < 1.0:
        raise TaskError("INVALID_TARGET_REGION", "目标四角形面积过小")
    return ordered


def scale_request_corners(
        corners: np.ndarray,
        source_width: int,
        source_height: int,
        actual_width: int,
        actual_height: int) -> np.ndarray:
    if min(source_width, source_height, actual_width, actual_height) <= 0:
        raise TaskError("INVALID_IMAGE_SIZE", "图像尺寸必须大于0")
    scaled = np.asarray(corners, dtype=np.float64).reshape(4, 2).copy()
    scaled[:, 0] *= float(actual_width) / float(source_width)
    scaled[:, 1] *= float(actual_height) / float(source_height)
    return order_corners(scaled)


def match_observation(
        observations: Iterable,
        ordered_corners_px: np.ndarray,
        prefer_inside: bool,
        max_distance_px: float) -> MatchResult:
    polygon = np.asarray(ordered_corners_px, dtype=np.float32).reshape(-1, 1, 2)
    request_center = np.mean(ordered_corners_px, axis=0)
    candidates = []
    for observation in observations:
        if observation.bbox_2d is None:
            continue
        x1, y1, x2, y2 = (float(x) for x in observation.bbox_2d)
        center = np.array([(x1 + x2) * 0.5, (y1 + y2) * 0.5],
                          dtype=np.float64)
        inside = cv2.pointPolygonTest(
            polygon, (float(center[0]), float(center[1])), False) >= 0.0
        distance = float(np.linalg.norm(center - request_center))
        candidates.append((observation, center, distance, inside))
    if not candidates:
        raise TaskError("NO_YOLO_TARGET", "YOLO没有返回可匹配的检测框")
    pool = candidates
    if prefer_inside:
        inside_items = [item for item in candidates if item[3]]
        if inside_items:
            pool = inside_items
    selected = min(pool, key=lambda item: item[2])
    if max_distance_px > 0.0 and selected[2] > max_distance_px:
        raise TaskError(
            "TARGET_TOO_FAR",
            f"最近YOLO中心距指定区域中心{selected[2]:.1f}px，"
            f"超过阈值{max_distance_px:.1f}px")
    return MatchResult(
        observation=selected[0],
        request_center_px=request_center,
        detection_center_px=selected[1],
        distance_px=selected[2],
        center_inside_polygon=selected[3],
        ordered_corners_px=np.asarray(ordered_corners_px, dtype=np.float64),
    )

