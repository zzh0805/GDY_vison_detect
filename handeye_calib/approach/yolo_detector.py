# -*- coding: utf-8 -*-
"""Ultralytics YOLO 二维检测及基于有组织点云的三维中心/法向恢复。"""
from __future__ import annotations

import time
from typing import Any, Optional, Sequence, Tuple

import cv2
import numpy as np

from feature_geometry_fit_v1_0 import FeatureGeometryFitter
from ..hardware_interfaces import CameraFrame
from .models import (CameraModel, FrameChannelRequirements,
                     TargetDetectionBatch, TargetObservation,
                     Yolo2DDetection)
from .perception import validate_frame_channels


def _cpu_numpy(value: Any) -> np.ndarray:
    if value is None:
        return np.empty((0,), dtype=np.float64)
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        value = value.numpy()
    return np.asarray(value)


class YoloTargetDetector:
    """YOLO 负责分类/框；有组织点云负责三维中心和表面法向。

    仅 ``boxes.xywh`` 只能得到像素中心，不能直接用于机器人三维接近。本类将
    YOLO 框与点云对齐。``surrounding_panel`` 模式排除所有 YOLO 框后拟合
    外围安装平面，并以目标框中心像素射线与该平面的交点作为三维中心；
    因此不依赖目标本身的深度。
    """

    detector_name = "ultralytics_yolo_pointcloud"
    detector_version = "1.1.0"

    def __init__(
            self,
            model_path: str,
            confidence: float = 0.5,
            iou: float = 0.7,
            device: Optional[str] = None,
            classes: Optional[Sequence[int]] = None,
            class_names: Optional[Sequence[str]] = None,
            max_detections: int = 100,
            bbox_shrink_ratio: float = 0.15,
            center_patch_radius_px: int = 5,
            min_plane_points: int = 100,
            max_plane_sample_points: int = 20000,
            plane_distance_threshold_mm: float = 3.0,
            plane_min_inlier_ratio: float = 0.35,
            plane_region_mode: str = "bbox_inner",
            surrounding_expand_ratio: float = 1.0,
            surrounding_exclude_margin_ratio: float = 0.15,
            require_handeye_compatible_cloud: bool = True,
            model: Any = None,
    ):
        self.model_path = str(model_path)
        self.confidence = float(confidence)
        self.iou = float(iou)
        self.device = device
        self.classes = None if classes is None else [int(x) for x in classes]
        self.class_names = None if not class_names else [
            str(x).strip() for x in class_names if str(x).strip()]
        self.max_detections = int(max_detections)
        self.bbox_shrink_ratio = float(bbox_shrink_ratio)
        self.center_patch_radius_px = int(center_patch_radius_px)
        self.min_plane_points = int(min_plane_points)
        self.max_plane_sample_points = int(max_plane_sample_points)
        self.plane_distance_threshold_mm = float(plane_distance_threshold_mm)
        self.plane_min_inlier_ratio = float(plane_min_inlier_ratio)
        self.plane_region_mode = str(plane_region_mode).strip().lower()
        self.surrounding_expand_ratio = float(surrounding_expand_ratio)
        self.surrounding_exclude_margin_ratio = float(
            surrounding_exclude_margin_ratio)
        self.require_handeye_compatible_cloud = bool(
            require_handeye_compatible_cloud)
        if not 0.0 < self.confidence <= 1.0:
            raise ValueError("confidence 必须在 (0,1] 内")
        if not 0.0 <= self.bbox_shrink_ratio < 0.5:
            raise ValueError("bbox_shrink_ratio 必须在 [0,0.5) 内")
        if (self.center_patch_radius_px < 0 or self.min_plane_points < 3 or
                self.max_plane_sample_points < self.min_plane_points):
            raise ValueError("点云几何参数无效")
        if self.plane_region_mode not in ("bbox_inner", "surrounding_panel"):
            raise ValueError(
                "plane_region_mode 仅支持 bbox_inner 或 surrounding_panel")
        if self.surrounding_expand_ratio <= 0.0:
            raise ValueError("surrounding_expand_ratio 必须大于 0")
        if self.surrounding_exclude_margin_ratio < 0.0:
            raise ValueError(
                "surrounding_exclude_margin_ratio 不能小于 0")
        self._model = model

    def _ensure_model(self):
        if self._model is None:
            try:
                from ultralytics import YOLO
            except ImportError as exc:
                raise ImportError(
                    "未安装 ultralytics；请在运行环境执行 pip install ultralytics") from exc
            self._model = YOLO(self.model_path)
        return self._model

    def required_channels(self) -> FrameChannelRequirements:
        return FrameChannelRequirements(require_color=True, require_point_cloud=True)

    def predict_2d(self, source: Any, *, show: bool = False,
                   save: bool = False) -> Tuple[Yolo2DDetection, ...]:
        """与用户现有 YOLO 调用等价，但返回全部框的类型化结果。"""
        model = self._ensure_model()
        kwargs = {
            "source": source,
            "save": bool(save),
            "show": bool(show),
            "conf": self.confidence,
            "iou": self.iou,
            "max_det": self.max_detections,
            "verbose": False,
        }
        if self.device:
            kwargs["device"] = self.device
        if self.classes is not None:
            kwargs["classes"] = self.classes
        results = model.predict(**kwargs)
        if not results:
            return ()
        result = results[0]
        boxes = getattr(result, "boxes", None)
        if boxes is None:
            return ()
        xywh = _cpu_numpy(getattr(boxes, "xywh", None)).reshape(-1, 4)
        xyxy = _cpu_numpy(getattr(boxes, "xyxy", None)).reshape(-1, 4)
        confidences = _cpu_numpy(getattr(boxes, "conf", None)).reshape(-1)
        class_ids = _cpu_numpy(getattr(boxes, "cls", None)).reshape(-1)
        names = getattr(result, "names", None) or getattr(model, "names", {}) or {}
        output = []
        for index in range(len(xywh)):
            class_id = int(class_ids[index])
            if isinstance(names, dict):
                class_name = str(names.get(class_id, class_id))
            else:
                class_name = str(names[class_id]) if class_id < len(names) else str(class_id)
            output.append(Yolo2DDetection(
                instance_id=f"{class_name}_{index:03d}",
                class_id=class_id,
                class_name=class_name,
                confidence=float(confidences[index]),
                xywh_px=xywh[index],
                xyxy_px=xyxy[index],
            ))
        return tuple(output)

    @staticmethod
    def _point_scale_to_m(metadata: dict) -> float:
        unit = str(metadata.get("point_cloud_unit", "")).lower().strip()
        if unit in ("m", "meter", "metre"):
            return 1.0
        if unit in ("mm", "millimeter", "millimetre"):
            return 0.001
        raise ValueError(
            f"未知点云单位 {metadata.get('point_cloud_unit')!r}，必须声明 m 或 mm")

    @staticmethod
    def _pixel_ray_camera(
            pixel_uv: np.ndarray, camera_model: CameraModel) -> np.ndarray:
        """将彩色图像素去畸变后转为从相机光心出发的单位射线。"""
        pixel = np.asarray(pixel_uv, dtype=np.float64).reshape(1, 1, 2)
        normalized = cv2.undistortPoints(
            pixel, camera_model.K, camera_model.distortion).reshape(2)
        ray = np.array([normalized[0], normalized[1], 1.0], dtype=np.float64)
        length = float(np.linalg.norm(ray))
        if not np.isfinite(ray).all() or length < 1e-12:
            raise ValueError("YOLO 中心像素无法生成有效相机射线")
        return ray / length

    def _geometry_from_box(
            self, point_cloud: np.ndarray, detection: Yolo2DDetection,
            scale_to_m: float, camera_model: CameraModel,
            projected_cloud: Optional[Tuple[np.ndarray, np.ndarray]] = None,
            all_detections: Optional[Sequence[Yolo2DDetection]] = None,
            panel_plane: Optional[Tuple[np.ndarray, np.ndarray]] = None,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, dict]:
        # 面板平面模式：目标中心深度以基座面板平面为准（不读目标周边平面）。
        if panel_plane is not None:
            plane_center_m, plane_normal = panel_plane
            u, v = detection.xywh_px[:2]
            center_ray = self._pixel_ray_camera(
                np.asarray([u, v], dtype=np.float64), camera_model)
            denominator = float(np.dot(plane_normal, center_ray))
            if abs(denominator) < 1e-8:
                raise ValueError("YOLO 中心射线与面板平面近似平行")
            ray_plane_distance_m = float(
                np.dot(plane_normal, plane_center_m) / denominator)
            if (not np.isfinite(ray_plane_distance_m) or
                    ray_plane_distance_m <= 0.0):
                raise ValueError(
                    "YOLO 中心射线与面板平面交点不在相机前方")
            center_m = center_ray * ray_plane_distance_m
            normal = plane_normal / np.linalg.norm(plane_normal)
            tangent = np.array([1.0, 0.0, 0.0])
            tangent -= float(np.dot(tangent, normal)) * normal
            if np.linalg.norm(tangent) < 1e-8:
                tangent = np.array([0.0, 1.0, 0.0])
                tangent -= float(np.dot(tangent, normal)) * normal
            tangent /= np.linalg.norm(tangent)
            metrics = {
                "point_count": 0,
                "plane_rms_mm": float("nan"),
                "plane_inlier_ratio": float("nan"),
                "center_depth_source": (
                    "yolo_center_ray_panel_plane_intersection"),
                "center_ray_camera": center_ray.tolist(),
                "center_ray_plane_distance_m": ray_plane_distance_m,
                "camera_to_plane_perpendicular_distance_m": abs(float(
                    np.dot(plane_normal, plane_center_m))),
                "panel_plane": True,
                "plane_region_mode": self.plane_region_mode,
            }
            return center_m, normal, tangent, metrics
        height, width = point_cloud.shape[:2]
        x1f, y1f, x2f, y2f = detection.xyxy_px
        box_width = max(float(x2f - x1f), 1.0)
        box_height = max(float(y2f - y1f), 1.0)
        if self.plane_region_mode == "surrounding_panel":
            x1 = float(x1f - self.surrounding_expand_ratio * box_width)
            x2 = float(x2f + self.surrounding_expand_ratio * box_width)
            y1 = float(y1f - self.surrounding_expand_ratio * box_height)
            y2 = float(y2f + self.surrounding_expand_ratio * box_height)
        else:
            x1 = float(x1f + self.bbox_shrink_ratio * box_width)
            x2 = float(x2f - self.bbox_shrink_ratio * box_width)
            y1 = float(y1f + self.bbox_shrink_ratio * box_height)
            y2 = float(y2f - self.bbox_shrink_ratio * box_height)
        if x2 <= x1 or y2 <= y1:
            raise ValueError("YOLO 框裁剪后为空")

        exclusion_boxes = []
        if self.plane_region_mode == "surrounding_panel":
            margin = self.surrounding_exclude_margin_ratio
            for item in tuple(all_detections or (detection,)):
                ex1, ey1, ex2, ey2 = (
                    float(value) for value in item.xyxy_px)
                ew = max(ex2 - ex1, 1.0)
                eh = max(ey2 - ey1, 1.0)
                exclusion_boxes.append((
                    ex1 - margin * ew, ey1 - margin * eh,
                    ex2 + margin * ew, ey2 + margin * eh))

        def outside_exclusions(uu: np.ndarray, vv: np.ndarray) -> np.ndarray:
            keep = np.ones(np.asarray(uu).shape, dtype=bool)
            for ex1, ey1, ex2, ey2 in exclusion_boxes:
                keep &= ~(
                    (uu >= ex1) & (uu <= ex2) &
                    (vv >= ey1) & (vv <= ey2))
            return keep

        u, v = detection.xywh_px[:2]
        radius = self.center_patch_radius_px
        if projected_cloud is None:
            ix1, ix2 = max(0, int(np.floor(x1))), min(width, int(np.ceil(x2)))
            iy1, iy2 = max(0, int(np.floor(y1))), min(height, int(np.ceil(y2)))
            crop = np.asarray(
                point_cloud[iy1:iy2, ix1:ix2, :3], dtype=np.float64)
            valid = np.isfinite(crop).all(axis=2) & (crop[:, :, 2] > 0.0)
            if exclusion_boxes:
                crop_u, crop_v = np.meshgrid(
                    np.arange(ix1, ix2, dtype=np.float64),
                    np.arange(iy1, iy2, dtype=np.float64))
                valid &= outside_exclusions(crop_u, crop_v)
            points_source = crop[valid]
            if self.plane_region_mode == "bbox_inner":
                cx1 = max(0, int(round(u)) - radius)
                cx2 = min(width, int(round(u)) + radius + 1)
                cy1 = max(0, int(round(v)) - radius)
                cy2 = min(height, int(round(v)) + radius + 1)
                patch = np.asarray(
                    point_cloud[cy1:cy2, cx1:cx2, :3], dtype=np.float64)
                patch_valid = (
                    np.isfinite(patch).all(axis=2) & (patch[:, :, 2] > 0.0))
                center_points_source = patch[patch_valid]
            else:
                # 外围平面模式不读取目标本身深度。
                center_points_source = np.empty((0, 3), dtype=np.float64)
            geometry_mode = "pixel_aligned_crop"
        else:
            all_points, all_uv = projected_cloud
            all_points = np.asarray(all_points, dtype=np.float64).reshape(-1, 3)
            all_uv = np.asarray(all_uv, dtype=np.float64).reshape(-1, 2)
            inside = (
                (all_uv[:, 0] >= x1) & (all_uv[:, 0] <= x2) &
                (all_uv[:, 1] >= y1) & (all_uv[:, 1] <= y2))
            if exclusion_boxes:
                inside &= outside_exclusions(all_uv[:, 0], all_uv[:, 1])
            points_source = all_points[inside]
            if self.plane_region_mode == "bbox_inner":
                center_inside = (
                    (np.abs(all_uv[:, 0] - float(u)) <= radius) &
                    (np.abs(all_uv[:, 1] - float(v)) <= radius))
                center_points_source = all_points[center_inside]
            else:
                center_points_source = np.empty((0, 3), dtype=np.float64)
            geometry_mode = "projected_to_texture_image"
        if len(points_source) < self.min_plane_points:
            raise ValueError(
                f"目标框有效点云不足: {len(points_source)} < {self.min_plane_points}")

        # 控制 RANSAC 计算量；完整点数仍写入质量指标。
        fit_points = points_source
        if len(fit_points) > self.max_plane_sample_points:
            rng = np.random.default_rng(0)
            indices = rng.choice(
                len(fit_points), self.max_plane_sample_points, replace=False)
            fit_points = fit_points[indices]
        # 复用现有平面拟合器，其单位约定为 mm。
        source_to_mm = scale_to_m * 1000.0
        fit = FeatureGeometryFitter.fit_plane_ransac(
            fit_points * source_to_mm,
            distance_threshold_mm=self.plane_distance_threshold_mm,
            max_iterations=800,
            min_inlier_ratio=self.plane_min_inlier_ratio,
            random_seed=0,
        )
        plane_center_m = np.asarray(fit["center_camera_mm"], dtype=np.float64) * 0.001
        normal = np.asarray(fit["normal_camera"], dtype=np.float64)
        normal /= np.linalg.norm(normal)

        # 法向始终指向相机，后续悬停点使用 center + standoff * normal。
        if float(np.dot(normal, -plane_center_m)) < 0.0:
            normal = -normal

        center_ray = None
        ray_plane_distance_m = None
        if self.plane_region_mode == "surrounding_panel":
            # YOLO 只给出二维中心；三维中心由该像素射线与外围平面求交。
            # 不读取按钮/工件中心深度，因而中心深度缺失也不影响解算。
            center_ray = self._pixel_ray_camera(
                np.asarray([u, v], dtype=np.float64), camera_model)
            denominator = float(np.dot(normal, center_ray))
            if abs(denominator) < 1e-8:
                raise ValueError("YOLO 中心射线与拟合平面近似平行")
            ray_plane_distance_m = float(
                np.dot(normal, plane_center_m) / denominator)
            if (not np.isfinite(ray_plane_distance_m) or
                    ray_plane_distance_m <= 0.0):
                raise ValueError("YOLO 中心射线与拟合平面交点不在相机前方")
            center_m = center_ray * ray_plane_distance_m
            center_depth_source = "yolo_center_ray_plane_intersection"
        elif len(center_points_source):
            center_m = np.median(center_points_source, axis=0) * scale_to_m
            # 旧模式中中心点和平面来自同一目标，将局部毛刺投影回该平面。
            center_m = center_m - float(np.dot(
                center_m - plane_center_m, normal)) * normal
            center_depth_source = "center_patch_median_projected_to_plane"
        else:
            center_m = plane_center_m.copy()
            center_depth_source = "plane_inlier_centroid_fallback"
        plane_d_mm = -float(np.dot(normal, plane_center_m * 1000.0))

        tangent = np.array([1.0, 0.0, 0.0])
        tangent -= float(np.dot(tangent, normal)) * normal
        if np.linalg.norm(tangent) < 1e-8:
            tangent = np.array([0.0, 1.0, 0.0])
            tangent -= float(np.dot(tangent, normal)) * normal
        tangent /= np.linalg.norm(tangent)
        metrics = {
            "point_count": int(len(points_source)),
            "plane_sample_count": int(len(fit_points)),
            "plane_rms_mm": float(fit["rms_mm"]),
            "plane_inlier_ratio": float(fit["inlier_ratio"]),
            "plane_inlier_count": int(fit["inlier_count"]),
            "plane_center_camera_m": plane_center_m.tolist(),
            "plane_normal_camera": normal.tolist(),
            "plane_equation_camera_mm": [
                float(normal[0]), float(normal[1]), float(normal[2]),
                plane_d_mm],
            "plane_distance_threshold_mm": (
                self.plane_distance_threshold_mm),
            "center_patch_valid_count": int(len(center_points_source)),
            "center_depth_source": center_depth_source,
            "center_ray_camera": (
                center_ray.tolist() if center_ray is not None else None),
            "center_ray_plane_distance_m": ray_plane_distance_m,
            "camera_to_plane_perpendicular_distance_m": abs(float(
                np.dot(normal, plane_center_m))),
            "bbox_shrink_ratio": self.bbox_shrink_ratio,
            "plane_region_mode": self.plane_region_mode,
            "plane_outer_bbox_xyxy_px": [x1, y1, x2, y2],
            "plane_exclusion_boxes_xyxy_px": [
                list(item) for item in exclusion_boxes],
            "surrounding_expand_ratio": self.surrounding_expand_ratio,
            "surrounding_exclude_margin_ratio": (
                self.surrounding_exclude_margin_ratio),
            "point_cloud_geometry_mode": geometry_mode,
        }
        return center_m, normal, tangent, metrics

    def _fit_panel_plane(
            self, point_cloud: np.ndarray, panel_region_px: np.ndarray,
            scale_to_m: float, camera_model: CameraModel,
            projected_cloud: Optional[Tuple[np.ndarray, np.ndarray]],
            all_detections: Optional[Sequence[Yolo2DDetection]],
    ) -> Tuple[np.ndarray, np.ndarray]:
        """用基座面板矩形区域拟合统一平面（排除所有YOLO框）。
        返回 (plane_center_camera_m, normal_camera)。法向指向相机。
        panel_region_px 为 4x2 角点数组或 [x1,y1,x2,y2]。"""
        raw = np.asarray(panel_region_px, dtype=np.float64).reshape(-1, 2)
        if raw.shape[0] == 4 and raw.shape[1] == 2:
            px1, py1 = float(np.min(raw[:, 0])), float(np.min(raw[:, 1]))
            px2, py2 = float(np.max(raw[:, 0])), float(np.max(raw[:, 1]))
        else:
            values = np.asarray(panel_region_px, dtype=np.float64).reshape(4)
            px1, py1, px2, py2 = (float(v) for v in values)
        if px2 <= px1 or py2 <= py1:
            raise ValueError("基座面板区域为空")
        height, width = point_cloud.shape[:2]
        exclusion_boxes = []
        margin = self.surrounding_exclude_margin_ratio
        for item in tuple(all_detections or ()):
            ex1, ey1, ex2, ey2 = (float(v) for v in item.xyxy_px)
            ew = max(ex2 - ex1, 1.0)
            eh = max(ey2 - ey1, 1.0)
            exclusion_boxes.append((
                ex1 - margin * ew, ey1 - margin * eh,
                ex2 + margin * ew, ey2 + margin * eh))

        def outside_exclusions(uu: np.ndarray, vv: np.ndarray) -> np.ndarray:
            keep = np.ones(np.asarray(uu).shape, dtype=bool)
            for ex1, ey1, ex2, ey2 in exclusion_boxes:
                keep &= ~(
                    (uu >= ex1) & (uu <= ex2) &
                    (vv >= ey1) & (vv <= ey2))
            return keep

        if projected_cloud is None:
            ix1, ix2 = max(0, int(np.floor(px1))), min(width, int(np.ceil(px2)))
            iy1, iy2 = max(0, int(np.floor(py1))), min(height, int(np.ceil(py2)))
            crop = np.asarray(
                point_cloud[iy1:iy2, ix1:ix2, :3], dtype=np.float64)
            valid = np.isfinite(crop).all(axis=2) & (crop[:, :, 2] > 0.0)
            if exclusion_boxes:
                crop_u, crop_v = np.meshgrid(
                    np.arange(ix1, ix2, dtype=np.float64),
                    np.arange(iy1, iy2, dtype=np.float64))
                valid &= outside_exclusions(crop_u, crop_v)
            points_source = crop[valid]
        else:
            all_points, all_uv = projected_cloud
            all_points = np.asarray(all_points, dtype=np.float64).reshape(-1, 3)
            all_uv = np.asarray(all_uv, dtype=np.float64).reshape(-1, 2)
            inside = (
                (all_uv[:, 0] >= px1) & (all_uv[:, 0] <= px2) &
                (all_uv[:, 1] >= py1) & (all_uv[:, 1] <= py2))
            if exclusion_boxes:
                inside &= outside_exclusions(all_uv[:, 0], all_uv[:, 1])
            points_source = all_points[inside]
        if len(points_source) < self.min_plane_points:
            raise ValueError(
                f"基座面板有效点云不足: {len(points_source)} < "
                f"{self.min_plane_points}")
        fit_points = points_source
        if len(fit_points) > self.max_plane_sample_points:
            rng = np.random.default_rng(0)
            indices = rng.choice(
                len(fit_points), self.max_plane_sample_points, replace=False)
            fit_points = fit_points[indices]
        source_to_mm = scale_to_m * 1000.0
        fit = FeatureGeometryFitter.fit_plane_ransac(
            fit_points * source_to_mm,
            distance_threshold_mm=self.plane_distance_threshold_mm,
            max_iterations=800,
            min_inlier_ratio=self.plane_min_inlier_ratio,
            random_seed=0,
        )
        plane_center_m = np.asarray(
            fit["center_camera_mm"], dtype=np.float64) * 0.001
        normal = np.asarray(fit["normal_camera"], dtype=np.float64)
        normal /= np.linalg.norm(normal)
        if float(np.dot(normal, -plane_center_m)) < 0.0:
            normal = -normal
        return plane_center_m, normal

    @staticmethod
    def _project_cloud_to_image(
            point_cloud: np.ndarray,
            camera_model: CameraModel) -> Tuple[np.ndarray, np.ndarray]:
        """把纹理相机坐标系点云投影到纹理图，不依赖深度像素排列。"""
        cloud = np.asarray(point_cloud, dtype=np.float64)[..., :3].reshape(-1, 3)
        valid = np.isfinite(cloud).all(axis=1) & (cloud[:, 2] > 0.0)
        points = cloud[valid]
        if not len(points):
            raise ValueError("点云没有可投影的有限正深度点")
        projected_parts = []
        # 限制单次OpenCV临时数组大小，兼容百万级Mech-Eye点云。
        for start in range(0, len(points), 250000):
            image_points, _ = cv2.projectPoints(
                points[start:start + 250000],
                np.zeros(3), np.zeros(3),
                camera_model.K, camera_model.distortion)
            projected_parts.append(image_points.reshape(-1, 2))
        uv = np.concatenate(projected_parts, axis=0)
        finite = np.isfinite(uv).all(axis=1)
        return points[finite], uv[finite]

    @staticmethod
    def _target_transform(center: np.ndarray, normal: np.ndarray,
                          tangent: np.ndarray) -> np.ndarray:
        z_axis = normal / np.linalg.norm(normal)
        x_axis = tangent - float(np.dot(tangent, z_axis)) * z_axis
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        y_axis /= np.linalg.norm(y_axis)
        x_axis = np.cross(y_axis, z_axis)
        transform = np.eye(4, dtype=np.float64)
        transform[:3, :3] = np.column_stack((x_axis, y_axis, z_axis))
        transform[:3, 3] = center
        return transform

    @staticmethod
    def _draw_normal(overlay: np.ndarray, center: np.ndarray, normal: np.ndarray,
                     camera_model: CameraModel) -> None:
        try:
            points = np.stack([center, center + normal * 0.05], axis=0)
            image_points, _ = cv2.projectPoints(
                points, np.zeros(3), np.zeros(3),
                camera_model.K, camera_model.distortion)
            p0, p1 = np.rint(image_points.reshape(-1, 2)).astype(int)
            cv2.arrowedLine(overlay, tuple(p0), tuple(p1),
                            (255, 0, 255), 2, cv2.LINE_AA, tipLength=0.2)
        except Exception:
            pass

    def process(self, frame: CameraFrame, camera_model: CameraModel,
                request_id: str,
                panel_region_px: Optional[np.ndarray] = None
                ) -> TargetDetectionBatch:
        started = time.perf_counter()
        validate_frame_channels(frame, self.required_channels())
        color = np.asarray(frame.color)
        cloud = np.asarray(frame.point_cloud)
        if color.shape[:2] != (camera_model.height, camera_model.width):
            raise ValueError("彩色图尺寸与 CameraModel 不一致")
        if cloud.ndim != 3 or cloud.shape[2] < 3:
            raise ValueError(f"point_cloud 必须为 HxWx3，实际 {cloud.shape}")
        metadata = dict(frame.metadata or {})
        if self.require_handeye_compatible_cloud and not bool(
                metadata.get("point_cloud_handeye_compatible", False)):
            raise ValueError(
                "点云未确认与标定彩色相机坐标系兼容，禁止计算机器人目标")
        scale_to_m = self._point_scale_to_m(metadata)
        pixel_aligned = bool(metadata.get(
            "point_cloud_pixel_aligned_to_image",
            cloud.shape[:2] == color.shape[:2]))
        projected_cloud = None
        if pixel_aligned:
            if cloud.shape[:2] != color.shape[:2]:
                raise ValueError(
                    "点云声明为像素对齐，但尺寸 "
                    f"{cloud.shape[:2]} 与彩色图 {color.shape[:2]} 不一致")
        else:
            projected_cloud = self._project_cloud_to_image(cloud, camera_model)
        all_detections = self.predict_2d(color, show=False, save=False)
        detections = all_detections
        observations = []
        errors = []
        # 基座面板平面（可选）：用 base 面板矩形拟合统一平面，
        # 所有目标中心深度以该面板平面为准。
        panel_plane = None
        if panel_region_px is not None:
            panel_plane = self._fit_panel_plane(
                cloud, panel_region_px, scale_to_m, camera_model,
                projected_cloud, all_detections)
        if self.class_names:
            allowed_names = set(self.class_names)
            filtered = tuple(
                d for d in detections if d.class_name in allowed_names)
            dropped = len(detections) - len(filtered)
            if dropped:
                errors.append(
                    f"按 class_names 白名单过滤了 {dropped} 个非白名单框")
            detections = filtered
        overlay = color.copy()
        for index, detection in enumerate(detections):
            instance_id = (
                f"{detection.class_name}_{frame.frame_id:06d}_{index:03d}")
            x1, y1, x2, y2 = np.rint(detection.xyxy_px).astype(int)
            try:
                center, normal, tangent, metrics = self._geometry_from_box(
                    cloud, detection, scale_to_m, camera_model, projected_cloud,
                    all_detections=all_detections, panel_plane=panel_plane)
                observation = TargetObservation(
                    request_id=request_id, frame_id=frame.frame_id,
                    instance_id=instance_id,
                    class_id=detection.class_id,
                    class_name=detection.class_name,
                    confidence=detection.confidence,
                    valid=True,
                    center_camera_m=center,
                    surface_normal_camera=normal,
                    approach_direction_camera=-normal,
                    tangent_x_camera=tangent,
                    T_camera_target=self._target_transform(center, normal, tangent),
                    bbox_2d=tuple(float(x) for x in detection.xyxy_px),
                    quality_metrics={
                        "yolo_xywh_px": detection.xywh_px.tolist(),
                        "yolo_confidence": detection.confidence,
                        **metrics,
                    },
                    detector_name=self.detector_name,
                    detector_version=self.detector_version,
                )
                color_box = (0, 220, 0)
                self._draw_normal(overlay, center, normal, camera_model)
                if self.plane_region_mode == "surrounding_panel":
                    outer_val = metrics.get("plane_outer_bbox_xyxy_px")
                    if outer_val is not None:
                        outer = np.rint(outer_val).astype(int)
                        cv2.rectangle(
                            overlay, (outer[0], outer[1]),
                            (outer[2], outer[3]), (255, 255, 0), 1)
            except Exception as exc:
                message = f"{instance_id} 三维恢复失败: {exc}"
                errors.append(message)
                observation = TargetObservation(
                    request_id=request_id, frame_id=frame.frame_id,
                    instance_id=instance_id,
                    class_id=detection.class_id,
                    class_name=detection.class_name,
                    confidence=detection.confidence,
                    valid=False,
                    bbox_2d=tuple(float(x) for x in detection.xyxy_px),
                    quality_metrics={
                        "yolo_xywh_px": detection.xywh_px.tolist(),
                        "yolo_confidence": detection.confidence,
                    },
                    detector_name=self.detector_name,
                    detector_version=self.detector_version,
                    error_code="GEOMETRY_RECOVERY_FAILED",
                    error_message=message,
                )
                color_box = (0, 0, 255)
            observations.append(observation)
            cv2.rectangle(overlay, (x1, y1), (x2, y2), color_box, 2)
            cv2.circle(
                overlay,
                tuple(np.rint(detection.xywh_px[:2]).astype(int)),
                5, (0, 255, 255), -1)
            cv2.putText(
                overlay,
                f"{index}:{detection.class_name} {detection.confidence:.2f}",
                (x1, max(20, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, color_box, 2, cv2.LINE_AA)
        if not detections:
            errors.append("YOLO 未检测到置信度达到阈值的目标")
        return TargetDetectionBatch(
            request_id=request_id,
            frame_id=frame.frame_id,
            timestamp=frame.timestamp,
            detector_name=self.detector_name,
            detector_version=self.detector_version,
            observations=tuple(observations),
            processing_time_ms=(time.perf_counter() - started) * 1000.0,
            errors=tuple(errors),
            debug_overlay=overlay,
        )
