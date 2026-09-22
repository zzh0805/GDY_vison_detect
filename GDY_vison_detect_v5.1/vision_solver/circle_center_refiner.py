# -*- coding: utf-8 -*-
"""无YOLO模式下，在请求粗框附近寻找目标的最大有效外圆。"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Mapping, Optional

import cv2
import numpy as np


@dataclass(frozen=True)
class CircleCenterMatch:
    center_px: np.ndarray
    edge_center_px: np.ndarray
    radius_px: float
    score: float
    edge_support: float
    distance_to_request_center_px: float
    search_roi_xyxy_px: tuple[int, int, int, int]
    candidate_count: int
    accepted_candidate_count: int
    selected_cluster_candidate_count: int
    color_name: Optional[str]
    color_score: float
    color_coverage: float
    color_center_px: Optional[np.ndarray]
    color_fusion_applied: bool


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
    min_radius_ratio = _setting_float(settings, "min_radius_ratio", 0.08)
    max_radius_ratio = _setting_float(settings, "max_radius_ratio", 0.75)
    min_edge_support = _setting_float(
        settings, "min_edge_support", 0.55)
    cluster_tolerance_ratio = _setting_float(
        settings, "center_cluster_tolerance_ratio", 0.18)
    outer_support_ratio = _setting_float(
        settings, "outer_circle_min_support_ratio", 0.65)
    hough_param2 = _setting_float(settings, "hough_param2", 28.0)
    max_distance = _setting_float(
        settings, "max_center_distance_px", 250.0)
    raw_color_settings = settings.get("color_fusion") or {}
    if not isinstance(raw_color_settings, Mapping):
        raise ValueError(
            "target_matching.circle_refinement.color_fusion必须是字典")
    color_settings = dict(raw_color_settings)
    if not isinstance(color_settings.get("enabled", False), bool):
        raise ValueError("圆心拟合color_fusion.enabled必须是true或false")
    color_center_mode = str(
        color_settings.get("center_mode", "edge")).strip().lower()
    if color_center_mode not in ("edge", "weighted_centroid"):
        raise ValueError(
            "圆心拟合color_fusion.center_mode必须是"
            "edge或weighted_centroid")
    color_min_coverage = _setting_float(
        color_settings, "min_coverage", 0.06)
    color_score_weight = _setting_float(
        color_settings, "score_weight", 0.20)
    color_center_blend = _setting_float(
        color_settings, "center_blend", 0.35)
    color_max_shift_ratio = _setting_float(
        color_settings, "max_center_shift_ratio", 0.20)
    color_selection_weight = _setting_float(
        color_settings, "selection_weight", 0.20)
    if not 0.0 <= expand_ratio <= 2.0:
        raise ValueError("圆心拟合expand_ratio必须在[0,2]内")
    if not 0.0 <= min_score <= 1.0:
        raise ValueError("圆心拟合min_score必须在[0,1]内")
    try:
        radius_band_count = int(settings.get("radius_band_count", 7))
    except (TypeError, ValueError) as exc:
        raise ValueError("圆心拟合radius_band_count必须是整数") from exc
    if not 0.02 <= min_radius_ratio < max_radius_ratio <= 1.5:
        raise ValueError(
            "圆心拟合半径比例必须满足"
            "0.02<=min_radius_ratio<max_radius_ratio<=1.5")
    if not 0.0 <= min_edge_support <= 1.0:
        raise ValueError("圆心拟合min_edge_support必须在[0,1]内")
    if not 0.01 <= cluster_tolerance_ratio <= 0.5:
        raise ValueError(
            "圆心拟合center_cluster_tolerance_ratio必须在[0.01,0.5]内")
    if not 0.0 <= outer_support_ratio <= 1.0:
        raise ValueError(
            "圆心拟合outer_circle_min_support_ratio必须在[0,1]内")
    if not 2 <= radius_band_count <= 12:
        raise ValueError("圆心拟合radius_band_count必须在[2,12]内")
    if hough_param2 <= 0.0:
        raise ValueError("圆心拟合hough_param2必须大于0")
    if max_distance <= 0.0:
        raise ValueError("圆心拟合max_center_distance_px必须大于0")
    if not 0.0 <= color_min_coverage <= 1.0:
        raise ValueError("圆心拟合color_fusion.min_coverage必须在[0,1]内")
    if not 0.0 <= color_score_weight <= 1.0:
        raise ValueError("圆心拟合color_fusion.score_weight必须在[0,1]内")
    if not 0.0 <= color_center_blend <= 1.0:
        raise ValueError("圆心拟合color_fusion.center_blend必须在[0,1]内")
    if not 0.0 <= color_max_shift_ratio <= 0.5:
        raise ValueError(
            "圆心拟合color_fusion.max_center_shift_ratio必须在[0,0.5]内")
    if not 0.0 <= color_selection_weight <= 1.0:
        raise ValueError("圆心拟合color_fusion.selection_weight必须在[0,1]内")


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


def _color_masks(bgr_roi: np.ndarray) -> dict[str, np.ndarray]:
    """生成较宽松的红、绿、黑掩膜，适应现场轻微曝光变化。"""
    smoothed = cv2.GaussianBlur(bgr_roi, (5, 5), 0.8)
    hsv = cv2.cvtColor(smoothed, cv2.COLOR_BGR2HSV)
    hue, saturation, value = cv2.split(hsv)
    red = (((hue <= 13) | (hue >= 167)) &
           (saturation >= 45) & (value >= 30))
    green = ((hue >= 32) & (hue <= 100) &
             (saturation >= 35) & (value >= 25))
    black = value <= 100
    kernel = np.ones((3, 3), dtype=np.uint8)
    result = {}
    for name, raw_mask in (
            ("red", red), ("green", green), ("black", black)):
        mask = raw_mask.astype(np.uint8) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        result[name] = mask
    return result


def _single_color_evidence(mask: np.ndarray, color_name: str,
                           cx: float, cy: float, radius: float,
                           min_coverage: float) -> Optional[dict[str, Any]]:
    """计算圆内部某种颜色的连通区域、质心和可靠度。"""
    height, width = mask.shape[:2]
    inner_radius = max(5.0, radius * 0.86)
    disk = np.zeros((height, width), dtype=np.uint8)
    cv2.circle(
        disk, (int(round(cx)), int(round(cy))),
        int(round(inner_radius)), 255, -1, cv2.LINE_8)
    inside = cv2.bitwise_and(mask, disk)
    disk_area = max(1, int(np.count_nonzero(disk)))
    component_count, labels, statistics, centroids = (
        cv2.connectedComponentsWithStats(inside, connectivity=8))
    choices = []
    minimum_area = max(6, int(disk_area * 0.012))
    for component in range(1, component_count):
        area = int(statistics[component, cv2.CC_STAT_AREA])
        if area < minimum_area:
            continue
        center = np.asarray(centroids[component], dtype=np.float64)
        displacement = float(np.linalg.norm(center - np.array([cx, cy])))
        proximity = max(
            0.0, 1.0 - displacement / max(inner_radius, 1.0))
        choices.append((area * (0.65 + 0.35 * proximity),
                        area, center, component, proximity))
    if not choices:
        return None
    _rank, area, center, component, proximity = max(
        choices, key=lambda item: item[0])
    coverage = float(area / disk_area)
    if coverage < min_coverage:
        return None
    component_mask = np.where(labels == component, 255, 0).astype(np.uint8)
    contours, _hierarchy = cv2.findContours(
        component_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    perimeter = sum(float(cv2.arcLength(contour, True))
                    for contour in contours)
    circularity = float(np.clip(
        4.0 * math.pi * area / max(perimeter * perimeter, 1.0),
        0.0, 1.0))
    coverage_score = min(
        1.0, coverage / max(0.24, min_coverage * 3.0))
    score = float(np.clip(
        0.55 * coverage_score + 0.30 * proximity +
        0.15 * circularity,
        0.0, 1.0))
    return {
        "name": color_name,
        "center_roi_px": center,
        "coverage": coverage,
        "circularity": circularity,
        "score": score,
    }


def _candidate_color_evidence(
        masks: Mapping[str, np.ndarray], expected_color: Optional[str],
        cx: float, cy: float, radius: float,
        min_coverage: float) -> Optional[dict[str, Any]]:
    names = ([expected_color] if expected_color is not None
             else ["red", "green", "black"])
    evidence = [
        item for item in (
            _single_color_evidence(
                masks[name], name, cx, cy, radius, min_coverage)
            for name in names)
        if item is not None
    ]
    if not evidence:
        return None
    return max(
        evidence,
        key=lambda item: (
            item["score"], item["coverage"], item["circularity"]))


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
    # 只在当前霍夫半径附近精修，避免外圆候选重新缩到更清晰的内圈。
    for scale in np.linspace(0.94, 1.06, 7):
        radius = float(hough_radius * scale)
        hard = _hard_edge_support(distance, cx, cy, radius)
        soft = _soft_edge_support(
            magnitude, gradient_threshold, cx, cy, radius)
        choices.append((0.20 * hard + 0.80 * soft, radius))
    support, radius = max(
        choices,
        key=lambda item: (item[0], -abs(item[1] - hough_radius)))
    return radius, support


def _radius_banded_hough(
        blurred: np.ndarray, minimum_radius: int, maximum_radius: int,
        radius_band_count: int, minimum_center_distance: float,
        hough_param2: float) -> np.ndarray:
    """分半径段调用霍夫圆，使同一中心的内外圆都能成为候选。"""
    boundaries = np.linspace(
        minimum_radius, maximum_radius, radius_band_count + 1)
    parts = []
    for index in range(radius_band_count):
        band_width = boundaries[index + 1] - boundaries[index]
        overlap = max(2, int(round(band_width * 0.15)))
        band_minimum = max(
            8, int(math.floor(boundaries[index] - overlap)))
        band_maximum = max(
            band_minimum + 2,
            int(math.ceil(boundaries[index + 1] + overlap)))
        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=max(12.0, minimum_center_distance),
            param1=100,
            param2=hough_param2,
            minRadius=band_minimum,
            maxRadius=band_maximum,
        )
        if circles is not None:
            parts.append(np.asarray(circles[0], dtype=np.float64))
    if not parts:
        return np.empty((0, 3), dtype=np.float64)
    return np.concatenate(parts, axis=0)


def _center_clusters(candidates: list[dict[str, Any]],
                     tolerance_px: float) -> list[list[dict[str, Any]]]:
    """按圆心距离聚合同一物理工件的多层同心圆。"""
    count = len(candidates)
    parents = list(range(count))

    def find(index: int) -> int:
        while parents[index] != index:
            parents[index] = parents[parents[index]]
            index = parents[index]
        return index

    def union(first: int, second: int) -> None:
        first_root = find(first)
        second_root = find(second)
        if first_root != second_root:
            parents[second_root] = first_root

    for first in range(count):
        for second in range(first):
            distance = float(np.linalg.norm(
                candidates[first]["center_roi_px"] -
                candidates[second]["center_roi_px"]))
            if distance <= tolerance_px:
                union(first, second)
    grouped: dict[int, list[dict[str, Any]]] = {}
    for index, candidate in enumerate(candidates):
        grouped.setdefault(find(index), []).append(candidate)
    return list(grouped.values())


def find_nearest_circle_center(
        color_image: np.ndarray, target_corners_px: np.ndarray,
        settings: Mapping[str, Any], *,
        max_center_distance_px: Optional[float] = None,
        expected_color: Optional[str] = None,
) -> Optional[CircleCenterMatch]:
    """选择离粗框中心最近的工件组，并返回该组最大有效外圆。"""
    validate_circle_refinement_settings(settings)
    image = np.asarray(color_image)
    if image.ndim == 3 and image.shape[2] >= 3:
        bgr = image[:, :, :3]
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    elif image.ndim == 2:
        bgr = None
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
    color_settings = dict(settings.get("color_fusion") or {})
    color_enabled = bool(color_settings.get("enabled", False)) and bgr is not None
    normalized_expected_color = (
        None if expected_color is None
        else str(expected_color).strip().lower())
    if normalized_expected_color in ("", "auto"):
        normalized_expected_color = None
    if normalized_expected_color not in (None, "red", "green", "black"):
        raise ValueError(
            "expected_color必须是red、green、black或auto")
    color_masks = (
        _color_masks(bgr[ry1:ry2, rx1:rx2]) if color_enabled else {})
    color_min_coverage = _setting_float(
        color_settings, "min_coverage", 0.06)
    color_score_weight = _setting_float(
        color_settings, "score_weight", 0.20)
    minimum_dimension = min(gray_roi.shape[:2])
    request_box_minimum_dimension = min(float(x2 - x1), float(y2 - y1))
    min_radius_ratio = _setting_float(
        settings, "min_radius_ratio", 0.08)
    max_radius_ratio = _setting_float(
        settings, "max_radius_ratio", 0.75)
    minimum_radius = max(
        8, int(request_box_minimum_dimension * min_radius_ratio))
    maximum_radius = max(
        minimum_radius + 4,
        int(request_box_minimum_dimension * max_radius_ratio))
    radius_band_count = int(settings.get("radius_band_count", 7))
    circles = _radius_banded_hough(
        blurred,
        minimum_radius,
        maximum_radius,
        radius_band_count,
        request_box_minimum_dimension * 0.10,
        _setting_float(settings, "hough_param2", 28.0),
    )
    if not len(circles):
        return None

    min_score = _setting_float(settings, "min_score", 0.35)
    min_edge_support = _setting_float(
        settings, "min_edge_support", 0.55)
    allowed_distance = (
        _setting_float(settings, "max_center_distance_px", 250.0)
        if max_center_distance_px is None else float(max_center_distance_px))
    accepted = []
    all_circles = np.asarray(circles, dtype=np.float64)
    for cx, cy, hough_radius in all_circles:
        radius, support = _best_radius(
            distance, magnitude, gradient_threshold,
            float(cx), float(cy), float(hough_radius))
        color_evidence = (
            _candidate_color_evidence(
                color_masks, normalized_expected_color,
                float(cx), float(cy), float(radius),
                color_min_coverage)
            if color_enabled else None)
        displacement = float(np.linalg.norm(
            np.array([cx, cy]) - expected_roi_center))
        center_score = max(
            0.0,
            1.0 - displacement / max(0.75 * minimum_dimension, 1.0))
        edge_score = 0.25 + 0.55 * support + 0.20 * center_score
        score = min(
            1.0,
            edge_score + color_score_weight * float(
                color_evidence["score"] if color_evidence else 0.0))
        if (score < min_score or support < min_edge_support or
                displacement > allowed_distance):
            continue
        accepted.append({
            "distance_px": displacement,
            "center_roi_px": np.array([cx, cy], dtype=np.float64),
            "radius_px": float(radius),
            "score": float(score),
            "edge_support": float(support),
            "color_evidence": color_evidence,
        })
    if not accepted:
        return None

    cluster_tolerance = max(
        7.0,
        request_box_minimum_dimension * _setting_float(
            settings, "center_cluster_tolerance_ratio", 0.18))
    clusters = _center_clusters(accepted, cluster_tolerance)
    # 第一级：选择包含“离原始框中心最近圆”的物理工件组。
    selectable_clusters = clusters
    if color_enabled and normalized_expected_color is not None:
        color_clusters = [
            cluster for cluster in clusters
            if any(item["color_evidence"] is not None for item in cluster)
        ]
        # 期望颜色确实存在时优先匹配；不存在则安全回退纯边缘策略。
        if color_clusters:
            selectable_clusters = color_clusters
    selection_weight_px = (
        request_box_minimum_dimension * _setting_float(
            color_settings, "selection_weight", 0.20)
        if color_enabled else 0.0)
    selected_cluster = min(
        selectable_clusters,
        key=lambda cluster: (
            min(item["distance_px"] for item in cluster) -
            selection_weight_px * max(
                float(item["color_evidence"]["score"])
                if item["color_evidence"] is not None else 0.0
                for item in cluster),
            -max(item["score"] for item in cluster)))
    # 第二级：在该工件的同心圆组中，先剔除相对组内最佳支持度过低的
    # 虚假大圆，再选择最大有效外圆。
    best_cluster_support = max(
        item["edge_support"] for item in selected_cluster)
    minimum_outer_support = max(
        min_edge_support,
        best_cluster_support * _setting_float(
            settings, "outer_circle_min_support_ratio", 0.65))
    outer_candidates = [
        item for item in selected_cluster
        if item["edge_support"] >= minimum_outer_support
    ]
    if color_enabled and normalized_expected_color is not None:
        colored_outer_candidates = [
            item for item in outer_candidates
            if item["color_evidence"] is not None
        ]
        if colored_outer_candidates:
            outer_candidates = colored_outer_candidates
    selected = max(
        outer_candidates,
        key=lambda item: (
            item["radius_px"],
            float(item["color_evidence"]["score"])
            if item["color_evidence"] is not None else 0.0,
            item["score"]))
    edge_center_roi = selected["center_roi_px"]
    center_roi = edge_center_roi.copy()
    selected_color = selected["color_evidence"]
    color_fusion_applied = False
    color_center_mode = str(
        color_settings.get("center_mode", "edge")).strip().lower()
    if (selected_color is not None and
            color_center_mode == "weighted_centroid"):
        color_delta = (
            selected_color["center_roi_px"] - edge_center_roi)
        color_delta_length = float(np.linalg.norm(color_delta))
        maximum_color_shift = (
            selected["radius_px"] * _setting_float(
                color_settings, "max_center_shift_ratio", 0.20))
        if color_delta_length > maximum_color_shift > 0.0:
            color_delta *= maximum_color_shift / color_delta_length
        # 非圆形的开关手柄降低颜色质心权重，圆形按钮使用完整配置权重。
        shape_weight = 0.35 + 0.65 * float(
            selected_color["circularity"])
        blend = (_setting_float(
            color_settings, "center_blend", 0.35) * shape_weight)
        center_roi += color_delta * blend
        color_fusion_applied = bool(
            blend > 0.0 and np.linalg.norm(color_delta) > 0.0)
    center_px = center_roi + np.array([rx1, ry1], dtype=np.float64)
    edge_center_px = edge_center_roi + np.array(
        [rx1, ry1], dtype=np.float64)
    color_center_px = (
        None if selected_color is None else
        selected_color["center_roi_px"] +
        np.array([rx1, ry1], dtype=np.float64))
    return CircleCenterMatch(
        center_px=center_px,
        edge_center_px=edge_center_px,
        radius_px=selected["radius_px"],
        score=selected["score"],
        edge_support=selected["edge_support"],
        distance_to_request_center_px=float(np.linalg.norm(
            center_px - request_center)),
        search_roi_xyxy_px=(rx1, ry1, rx2, ry2),
        candidate_count=len(all_circles),
        accepted_candidate_count=len(accepted),
        selected_cluster_candidate_count=len(selected_cluster),
        color_name=(
            None if selected_color is None else str(selected_color["name"])),
        color_score=(
            0.0 if selected_color is None
            else float(selected_color["score"])),
        color_coverage=(
            0.0 if selected_color is None
            else float(selected_color["coverage"])),
        color_center_px=color_center_px,
        color_fusion_applied=color_fusion_applied,
    )
