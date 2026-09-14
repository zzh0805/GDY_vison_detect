# -*- coding: utf-8 -*-
"""多视角采集YAML读取和严格校验。"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import yaml


def _section(data: Mapping[str, Any], name: str) -> dict:
    value = data.get(name)
    if not isinstance(value, Mapping):
        raise ValueError(f"配置{name}必须是字典")
    return dict(value)


def _finite(value: Any, name: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name}必须是数字") from exc
    if not np.isfinite(number):
        raise ValueError(f"{name}不能是NaN/Inf")
    return number


@dataclass(frozen=True)
class CollectionConfig:
    source_path: Path
    data: dict

    @property
    def root(self) -> Path:
        return self.source_path.parent

    def section(self, name: str) -> dict:
        return _section(self.data, name)

    def resolve_path(self, value: Any, *, must_exist: bool = False) -> Path:
        text = str(value or "").strip()
        if not text:
            raise ValueError("配置路径不能为空")
        path = Path(text).expanduser()
        if not path.is_absolute():
            path = self.root / path
        path = path.resolve()
        if must_exist and not path.is_file():
            raise FileNotFoundError(path)
        return path

    @property
    def runtime(self) -> dict:
        return self.section("runtime")

    @property
    def robot(self) -> dict:
        return self.section("robot")

    @property
    def service(self) -> dict:
        return self.section("service")

    @property
    def dataset(self) -> dict:
        return self.section("dataset")

    @property
    def reference(self) -> dict:
        return self.section("reference")

    @property
    def trajectory(self) -> dict:
        return self.section("trajectory")

    def validate(self) -> None:
        mode = str(self.runtime.get("mode", "simulation")).strip().lower()
        if mode not in ("simulation", "real"):
            raise ValueError("runtime.mode只支持simulation或real")

        robot = self.robot
        if not str(robot.get("ip", "")).strip():
            raise ValueError("robot.ip不能为空")
        for key in ("speed_mm_s", "accel_mm_s2", "settle_s", "timeout_s"):
            if _finite(robot.get(key), f"robot.{key}") < 0.0:
                raise ValueError(f"robot.{key}不能小于0")

        service = self.service
        if not str(service.get("base_url", "")).startswith(("http://", "https://")):
            raise ValueError("service.base_url必须是HTTP地址")
        if _finite(service.get("timeout_s"), "service.timeout_s") <= 0.0:
            raise ValueError("service.timeout_s必须大于0")
        discard_s = _finite(
            service.get("discard_stale_frames_s", 0.0),
            "service.discard_stale_frames_s")
        if discard_s < 0.0 or discard_s > 10.0:
            raise ValueError("service.discard_stale_frames_s必须在0到10秒之间")
        dataset_name = str(service.get("dataset_name", "")).strip()
        if not dataset_name or ".." in Path(dataset_name).parts or Path(dataset_name).is_absolute():
            raise ValueError("service.dataset_name必须是不含..的相对名称")

        dataset = self.dataset
        self.resolve_path(dataset.get("output_directory"))
        if str(dataset.get("image_format", "png")).lower() not in ("png", "jpg", "jpeg"):
            raise ValueError("dataset.image_format只支持png/jpg/jpeg")

        reference = self.reference
        width = int(reference.get("image_width", 0))
        height = int(reference.get("image_height", 0))
        if width <= 0 or height <= 0:
            raise ValueError("reference图像宽高必须大于0")
        roi = np.asarray(reference.get("roi_xyxy_px"), dtype=np.float64)
        if roi.shape != (4,) or not np.isfinite(roi).all():
            raise ValueError("reference.roi_xyxy_px必须是4个有限数字")
        x1, y1, x2, y2 = roi
        if not (0 <= x1 < x2 < width and 0 <= y1 < y2 < height):
            raise ValueError("reference.roi_xyxy_px必须完全位于图像内")
        if _finite(reference.get("target_depth_mm"),
                   "reference.target_depth_mm") <= 0.0:
            raise ValueError("reference.target_depth_mm必须大于0")
        target_source = str(reference.get(
            "target_source", "configured_depth")).strip().lower()
        if target_source not in ("point_cloud_plane", "configured_depth"):
            raise ValueError(
                "reference.target_source只支持point_cloud_plane或configured_depth")
        geometry = reference.get("geometry", {})
        if not isinstance(geometry, Mapping):
            raise ValueError("reference.geometry必须是字典")
        for key in (
                "plane_expand_ratio", "ransac_threshold_mm",
                "min_inlier_ratio", "max_plane_rms_mm"):
            if _finite(geometry.get(key), f"reference.geometry.{key}") <= 0.0:
                raise ValueError(f"reference.geometry.{key}必须大于0")
        if float(geometry["min_inlier_ratio"]) > 1.0:
            raise ValueError("reference.geometry.min_inlier_ratio不能大于1")
        if _finite(geometry.get("exclude_roi_margin_ratio"),
                   "reference.geometry.exclude_roi_margin_ratio") < 0.0:
            raise ValueError(
                "reference.geometry.exclude_roi_margin_ratio不能小于0")
        for key in (
                "min_plane_points", "max_plane_sample_points",
                "ransac_iterations"):
            if int(geometry.get(key, 0)) < 3:
                raise ValueError(f"reference.geometry.{key}必须至少为3")
        if int(geometry["max_plane_sample_points"]) < int(
                geometry["min_plane_points"]):
            raise ValueError(
                "reference.geometry.max_plane_sample_points不能小于min_plane_points")
        self.resolve_path(reference.get("camera_calibration_file"), must_exist=True)
        self.resolve_path(reference.get("handeye_file"), must_exist=True)
        simulation_pose = np.asarray(
            reference.get("simulation_tcp_mm_rpy_deg"), dtype=np.float64)
        if simulation_pose.shape != (6,) or not np.isfinite(simulation_pose).all():
            raise ValueError("reference.simulation_tcp_mm_rpy_deg必须是6个有限数字")

        trajectory = self.trajectory
        samples = int(trajectory.get("samples_per_path", 0))
        if samples < 2:
            raise ValueError("trajectory.samples_per_path至少为2")
        margin = int(trajectory.get("image_margin_px", 0))
        if margin < 0 or margin * 2 >= min(width, height):
            raise ValueError("trajectory.image_margin_px无效")
        for key in ("max_tcp_translation_mm", "max_tcp_rotation_deg"):
            if _finite(trajectory.get(key), f"trajectory.{key}") <= 0.0:
                raise ValueError(f"trajectory.{key}必须大于0")
        for key in ("max_position_step_mm", "max_look_angle_step_deg"):
            if _finite(trajectory.get(key), f"trajectory.{key}") <= 0.0:
                raise ValueError(f"trajectory.{key}必须大于0")
        max_samples = int(trajectory.get("max_samples_per_path", 0))
        if max_samples < samples:
            raise ValueError(
                "trajectory.max_samples_per_path不能小于samples_per_path")
        incidence = _finite(
            trajectory.get("max_view_incidence_deg"),
            "trajectory.max_view_incidence_deg")
        if not 0.0 < incidence < 90.0:
            raise ValueError(
                "trajectory.max_view_incidence_deg必须在0到90度之间")
        verification = trajectory.get("verification")
        if not isinstance(verification, Mapping):
            raise ValueError("trajectory.verification必须是字典")
        for key in (
                "max_position_error_mm", "max_rotation_error_deg",
                "max_look_at_error_deg"):
            if _finite(verification.get(key),
                       f"trajectory.verification.{key}") <= 0.0:
                raise ValueError(f"trajectory.verification.{key}必须大于0")
        if int(verification.get("retry_count", -1)) < 0:
            raise ValueError("trajectory.verification.retry_count不能小于0")
        views = trajectory.get("views")
        if not isinstance(views, list) or not views:
            raise ValueError("trajectory.views必须是非空列表")
        names = set()
        for index, item in enumerate(views):
            if not isinstance(item, Mapping):
                raise ValueError(f"trajectory.views[{index}]必须是字典")
            name = str(item.get("name", "")).strip()
            if not name or name in names:
                raise ValueError("trajectory.views中的name必须非空且唯一")
            names.add(name)
            for key in ("azimuth_deg", "elevation_deg", "distance_offset_mm"):
                _finite(item.get(key, 0.0), f"trajectory.views[{index}].{key}")


def load_collection_config(path: Any) -> CollectionConfig:
    source = Path(path).expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(source)
    data = yaml.safe_load(source.read_text(encoding="utf-8-sig")) or {}
    if not isinstance(data, Mapping):
        raise ValueError("采集配置顶层必须是字典")
    config = CollectionConfig(source, dict(data))
    config.validate()
    return config
