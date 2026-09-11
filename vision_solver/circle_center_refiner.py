# -*- coding: utf-8 -*-
"""无YOLO模式下，在请求粗框附近用灰度边缘寻找最近圆心。"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Mapping, Optional

import cv2
import numpy as np


@dataclass(frozen=True)
class CircleCenterMatch:
    center_px: np.ndarray
    radius_px: float
    score: float
    edge_support: float
    distance_to_request_center_px: float
    search_roi_xyxy_px: tuple[int, int, int, int]
    candidate_count: int
    accepted_candidate_count: int


def _setting_float(settings: Mapping[str, Any], name: str,
                   default: float) -> float:
    value = float(settings.get(name, default))
    if not math.isfinite(value):
        raise ValueError(f"圆心拟合参数{name}必须是有限数字")
    return value


def validate_circle_refinement_settings(settings: Mapping[str, Any]) -> None:
    """校验workflow.yaml中的圆心拟合配置。"""
    if not isinstance(settings.get("enabled", False), bool):
        raise ValueError("target_matching.circle_refinement.enabled必须是true或false")
    if not isinstance(settings.get("fallback_to_box_center", False), bool):
        raise ValueError(
            "target_matching.circle_refinement.fallback_to_box_center"
            "必须是true或false")
    expand_ratio = _setting_float(settings, "expand_ratio", 0.45)
    min_score = _setting_float(settings, "min_score", 0.35)
    min_radius_ratio = _setting_float(settings, "min_radius_ratio", 0.14)
    max_radius_ratio = _setting_float(settings, "max_radius_ratio", 0.46)
    hough_param2 = _setting_float(settings, "hough_param2", 28.0)
    max_distance = _setting_float(
        settings, "max_center_distance_px", 250.0)
    if not 0.0 <= expand_ratio <= 2.0:
        raise ValueError("圆心拟合expand_ratio必须在[0,2]内")
    if not 0.0 <= min_score <= 1.0:
        raise ValueError("圆心拟合min_score必须在[0,1]内")
    if not 0.02 <= min_radius_ratio < max_radius_ratio <= 0.49:
        raise ValueError(
            "圆心拟合半径比例必须满足"
            "0.02<=min_radius_ratio<max_radius_ratio<=0.49")
    if hough_param2 <= 0.0:
        raise ValueError("圆心拟合hough_param2必须大于0")
    if max_distance <= 0.0:
        raise ValueError("圆心拟合max_center_distance_px必须大于0")


def _expanded_roi(corners: np.ndarray, image_shape: tuple[int, ...],
                  expand_ratio: float) -> tuple[int, int, int, int]:
    x1, y1 = np.min(corners, axis=0)
    x2, y2 = np.max(corners, axis=0)
    width = float(x2 - x1)
    height = float(y2 - y1)
    image_height, image_width = image_shape[:2]
    return (
        max(0, int(math.floor(x1 - width * expand_ratio))),
        max(0, int(math.floor(y1 - height * expand_ratio))),
        min(image_width, int(math.ceil(x2 + width * expand_ratio))),
        min(image_height, int(math.ceil(y2 + height * expand_ratio))),
    )


def _edge_inputs(gray_roi: np.ndarray) -> tuple[np.ndarray, np.ndarray,
                                                        np.ndarray, float]:
    enhanced = cv2.createCLAHE(
        clipLimit=2.0, tileGridSize=(8, 8)).apply(gray_roi)
    blurred = cv2.GaussianBlur(enhanced, (7, 7), 1.5)
    median = float(np.median(blurred))
    lower = int(max(20.0, 0.55 * median))
    upper = int(min(255.0, max(lower + 30.0, 1.45 * median)))
    edges = cv2.Canny(blurred, lower, upper, L2gradient=True)
    distance = cv2.distanceTransform(255 - edges, cv2.DIST_L2, 3)
    grad_x = cv2.Sobel(blurred, cv2.CV_32F, 1, 0, ksize=3)
    grad_y = cv2.Sobel(blurred, cv2.CV_32F, 0, 1, ksize=3)
    magnitude = cv2.magnitude(grad_x, grad_y)
    gradient_threshold = max(
        8.0, float(np.percentile(magnitude, 82.0)))
    return blurred, distance, magnitude, gradient_threshold


def _hard_edge_support(distance: np.ndarray, cx: float, cy: float,
                       radius: float, tolerance_px: float = 3.0) -> float:
    angles = np.linspace(0.0, 2.0 * np.pi, 360, endpoint=False)
    xs = np.rint(cx + radius * np.cos(angles)).astype(np.int32)
    ys = np.rint(cy + radius * np.sin(angles)).astype(np.int32)
    valid = ((xs >= 0) & (ys >= 0) &
             (xs < distance.shape[1]) & (ys < distance.shape[0]))
    if not np.any(valid):
        return 0.0
    return float(np.mean(distance[ys[valid], xs[valid]] <= tolerance_px))


def _soft_edge_support(magnitude: np.ndarray, gradient_threshold: float,
                       cx: float, cy: float, radius: float) -> float:
    angles = np.linspace(0.0, 2.0 * np.pi, 360, endpoint=False)
    band = max(2.0, radius * 0.045)
    samples = []
    for offset in np.linspace(-band, band, 7):
        sample_radius = radius + offset
        xs = np.rint(
            cx + sample_radius * np.cos(angles)).astype(np.int32)
        ys = np.rint(
            cy + sample_radius * np.sin(angles)).astype(np.int32)
        valid = ((xs >= 0) & (ys >= 0) &
                 (xs < magnitude.shape[1]) & (ys < magnitude.shape[0]))
        values = np.zeros(len(angles), dtype=np.float32)
        values[valid] = magnitude[ys[valid], xs[valid]]
        samples.append(values)
    radial_maximum = np.max(np.asarray(samples), axis=0)
    coverage = float(np.mean(radial_maximum >= gradient_threshold))
    strength = float(np.mean(np.clip(
        radial_maximum / max(gradient_threshold * 2.0, 1.0),
        0.0, 1.0)))
    return 0.70 * coverage + 0.30 * strength


def _best_radius(distance: np.ndarray, magnitude: np.ndarray,
                 gradient_threshold: float, cx: float, cy: float,
                 hough_radius: float) -> tuple[float, float]:
    choices = []
    for scale in np.linspace(0.78, 1.18, 17):
        radius = float(hough_radius * scale)
        hard = _hard_edge_support(distance, cx, cy, radius)
        soft = _soft_edge_support(
            magnitude, gradient_threshold, cx, cy, radius)
        choices.append((0.20 * hard + 0.80 * soft, radius))
    support, radius = max(
        choices,
        key=lambda item: (item[0], -abs(item[1] - hough_radius)))
    return radius, support


def find_nearest_circle_center(
        color_image: np.ndarray, target_corners_px: np.ndarray,
        settings: Mapping[str, Any], *,
        max_center_distance_px: Optional[float] = None,
) -> Optional[CircleCenterMatch]:
    """返回粗框中心附近最近的合格圆；没有可靠候选时返回None。"""
    validate_circle_refinement_settings(settings)
    image = np.asarray(color_image)
    if image.ndim == 3 and image.shape[2] >= 3:
        gray = cv2.cvtColor(image[:, :, :3], cv2.COLOR_BGR2GRAY)
    elif image.ndim == 2:
        gray = image.astype(np.uint8, copy=False)
    else:
        raise ValueError(f"圆心拟合要求灰度或BGR图像，实际{image.shape}")
    corners = np.asarray(
        target_corners_px, dtype=np.float64).reshape(-1, 2)
    if corners.shape != (4, 2) or not np.isfinite(corners).all():
        raise ValueError("圆心拟合目标框必须是4个有限像素角点")
    x1, y1 = np.min(corners, axis=0)
    x2, y2 = np.max(corners, axis=0)
    if x2 <= x1 or y2 <= y1:
        raise ValueError("圆心拟合目标框为空")
    request_center = np.mean(corners, axis=0)
    expand_ratio = _setting_float(settings, "expand_ratio", 0.45)
    rx1, ry1, rx2, ry2 = _expanded_roi(
        corners, gray.shape, expand_ratio)
    gray_roi = gray[ry1:ry2, rx1:rx2]
    if min(gray_roi.shape[:2]) < 24:
        return None
    expected_roi_center = request_center - np.array([rx1, ry1])
    blurred, distance, magnitude, gradient_threshold = _edge_inputs(gray_roi)
    minimum_dimension = min(gray_roi.shape[:2])
    min_radius_ratio = _setting_float(
        settings, "min_radius_ratio", 0.14)
    max_radius_ratio = _setting_float(
        settings, "max_radius_ratio", 0.46)
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=max(20.0, minimum_dimension * 0.30),
        param1=100,
        param2=_setting_float(settings, "hough_param2", 28.0),
        minRadius=max(8, int(minimum_dimension * min_radius_ratio)),
        maxRadius=max(10, int(minimum_dimension * max_radius_ratio)),
    )
    if circles is None:
        return None

    min_score = _setting_float(settings, "min_score", 0.35)
    allowed_distance = (
        _setting_float(settings, "max_center_distance_px", 250.0)
        if max_center_distance_px is None else float(max_center_distance_px))
    accepted = []
    all_circles = np.asarray(circles[0], dtype=np.float64)
    for cx, cy, hough_radius in all_circles:
        radius, support = _best_radius(
            distance, magnitude, gradient_threshold,
            float(cx), float(cy), float(hough_radius))
        displacement = float(np.linalg.norm(
            np.array([cx, cy]) - expected_roi_center))
        center_score = max(
            0.0,
            1.0 - displacement / max(0.75 * minimum_dimension, 1.0))
        score = 0.25 + 0.55 * support + 0.20 * center_score
        if score < min_score or displacement > allowed_distance:
            continue
        accepted.append((
            displacement, -score, float(cx), float(cy),
            float(radius), float(score), float(support)))
    if not accepted:
        return None

    # 用户要求：一次请求只保留离原始目标框中心最近的圆。
    displacement, _negative_score, cx, cy, radius, score, support = min(
        accepted, key=lambda item: (item[0], item[1]))
    return CircleCenterMatch(
        center_px=np.array([cx + rx1, cy + ry1], dtype=np.float64),
        radius_px=radius,
        score=score,
        edge_support=support,
        distance_to_request_center_px=displacement,
        search_roi_xyxy_px=(rx1, ry1, rx2, ry2),
        candidate_count=len(all_circles),
        accepted_candidate_count=len(accepted),
    )
