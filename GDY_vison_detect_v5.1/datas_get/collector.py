# -*- coding: utf-8 -*-
"""多视角采集编排；不包含相机SDK和检测算法。"""
from __future__ import annotations

import json
import shutil
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from handeye_calib.jaka_adapter import jaka_pose_to_unified

from .clients import robot_pose_mm_rpy_deg
from .config import CollectionConfig
from .geometry import (
    build_routes, evaluate_actual_waypoint, load_handeye_matrix,
    load_scaled_rgb_intrinsics, make_reference_geometry,
    make_reference_geometry_from_camera_points)


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


def _actual_pose(robot: Any) -> np.ndarray:
    getter = getattr(robot, "get_pose_mm_rpy_deg", None)
    if callable(getter):
        return np.asarray(getter(), dtype=np.float64).reshape(6)
    return robot_pose_mm_rpy_deg(robot)


def _move(robot: Any, pose_mm_rpy_deg: np.ndarray, name: str) -> None:
    direct = getattr(robot, "move_to_mm_rpy_deg", None)
    if callable(direct):
        direct(pose_mm_rpy_deg, name=name)
        return
    pose = np.asarray(pose_mm_rpy_deg, dtype=np.float64).reshape(6).copy()
    pose[3:] = np.radians(pose[3:])
    robot.move_linear(jaka_pose_to_unified(pose), name=name)


class DatasetCollector:
    def __init__(self, config: CollectionConfig, robot: Any,
                 color_client: Any):
        self.config = config
        self.robot = robot
        self.color_client = color_client
        self.reference_pose: np.ndarray | None = None
        self.session_dir: Path | None = None
        self.records: list[dict] = []

    def _new_session_dir(self) -> Path:
        dataset = self.config.dataset
        root = self.config.resolve_path(dataset["output_directory"])
        name = str(dataset.get("name", "multiview")).strip() or "multiview"
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        path = root / name / stamp
        path.mkdir(parents=True, exist_ok=False)
        (path / "images").mkdir()
        return path

    def _capture_to_session(self, file_stem: str) -> Path:
        assert self.session_dir is not None
        dataset = self.config.dataset
        extension = str(dataset.get("image_format", "png")).lower()
        if extension == "jpeg":
            extension = "jpg"
        file_name = f"{file_stem}.{extension}"
        service_dataset = (
            f"{self.config.service['dataset_name']}/{self.session_dir.name}")
        result = self.color_client.capture(service_dataset, file_name)
        return self._copy_capture_to_session(result.path, file_name)

    def _copy_capture_to_session(self, source_path: Path,
                                 file_name: str) -> Path:
        """把服务端已原子保存完成的图片复制到本次数据集并校验。"""
        assert self.session_dir is not None
        destination = self.session_dir / "images" / file_name
        source = Path(source_path).resolve()
        if source != destination.resolve():
            shutil.copy2(source, destination)
        image = cv2.imread(str(destination), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"保存后的彩色图无法读取: {destination}")
        expected = (int(self.config.reference["image_height"]),
                    int(self.config.reference["image_width"]))
        if image.shape[:2] != expected:
            raise RuntimeError(
                f"彩色图尺寸{image.shape[1]}x{image.shape[0]}与配置"
                f"{expected[1]}x{expected[0]}不一致")
        return destination

    def _capture_reference_to_session(self, intrinsics: Any) -> tuple:
        assert self.session_dir is not None
        reference = self.config.reference
        dataset = self.config.dataset
        extension = str(dataset.get("image_format", "png")).lower()
        if extension == "jpeg":
            extension = "jpg"
        file_name = f"reference.{extension}"
        service_dataset = (
            f"{self.config.service['dataset_name']}/{self.session_dir.name}")
        result = self.color_client.capture_reference(
            service_dataset, file_name, reference["roi_xyxy_px"],
            dict(reference.get("geometry") or {}),
            camera_matrix=intrinsics.K,
            distortion=intrinsics.distortion,
            fallback_depth_mm=float(reference["target_depth_mm"]))
        local_path = self._copy_capture_to_session(result.path, file_name)
        return local_path, result

    def _move_verified(self, geometry: Any, waypoint: Any,
                       name: str) -> tuple[np.ndarray, dict]:
        verification = dict(
            self.config.trajectory.get("verification") or {})
        retries = int(verification.get("retry_count", 0))
        last_pose = None
        last_metrics = None
        for attempt in range(retries + 1):
            suffix = "" if attempt == 0 else f"_retry_{attempt}"
            _move(self.robot, waypoint.tcp_mm_rpy_deg, name + suffix)
            last_pose = _actual_pose(self.robot)
            last_metrics = evaluate_actual_waypoint(
                geometry, waypoint, last_pose,
                int(self.config.trajectory["image_margin_px"]))
            if not bool(verification.get("enabled", True)):
                return last_pose, last_metrics
            valid = (
                float(last_metrics["positionErrorMm"]) <=
                float(verification["max_position_error_mm"]) and
                float(last_metrics["rotationErrorDeg"]) <=
                float(verification["max_rotation_error_deg"]) and
                float(last_metrics["lookAtErrorDeg"]) <=
                float(verification["max_look_at_error_deg"]) and
                bool(last_metrics["roiVisible"]))
            if valid:
                return last_pose, last_metrics
        assert last_pose is not None and last_metrics is not None
        raise RuntimeError(
            f"{name}到位校验失败: 位置误差="
            f"{last_metrics['positionErrorMm']:.2f}mm 姿态误差="
            f"{last_metrics['rotationErrorDeg']:.2f}° 光轴误差="
            f"{last_metrics['lookAtErrorDeg']:.2f}° "
            f"ROI可见={last_metrics['roiVisible']}")

    def _append_record(self, record: dict) -> None:
        assert self.session_dir is not None
        ready = _jsonable(record)
        self.records.append(ready)
        with (self.session_dir / "manifest.jsonl").open(
                "a", encoding="utf-8") as stream:
            stream.write(json.dumps(ready, ensure_ascii=False) + "\n")

    def run(self) -> Path:
        self.session_dir = self._new_session_dir()
        self.records = []
        moved = False
        try:
            self.robot.connect()
            self.reference_pose = _actual_pose(self.robot)
            reference = self.config.reference
            intrinsics = load_scaled_rgb_intrinsics(
                self.config.resolve_path(
                    reference["camera_calibration_file"], must_exist=True),
                int(reference["image_width"]), int(reference["image_height"]))
            handeye = load_handeye_matrix(self.config.resolve_path(
                reference["handeye_file"], must_exist=True))
            target_source = str(reference.get(
                "target_source", "configured_depth")).strip().lower()
            reference_quality = {
                "source": "configured_depth",
                "targetOpticalDepthMm": float(reference["target_depth_mm"]),
            }
            if target_source == "point_cloud_plane":
                reference_image, captured_reference = (
                    self._capture_reference_to_session(intrinsics))
                geometry = make_reference_geometry_from_camera_points(
                    self.reference_pose, handeye, intrinsics,
                    captured_reference.target_camera_mm,
                    captured_reference.roi_corners_camera_mm,
                    captured_reference.plane_normal_camera)
                reference_quality = dict(captured_reference.quality)
            else:
                reference_image = self._capture_to_session("reference")
                geometry = make_reference_geometry(
                    self.reference_pose, handeye, intrinsics,
                    reference["roi_xyxy_px"],
                    float(reference["target_depth_mm"]))
            trajectory = self.config.trajectory
            routes = build_routes(
                geometry, trajectory["views"],
                int(trajectory["samples_per_path"]),
                int(trajectory["image_margin_px"]),
                float(trajectory["max_tcp_translation_mm"]),
                float(trajectory["max_tcp_rotation_deg"]),
                bool(trajectory.get("preserve_reference_pose", True)),
                float(trajectory["max_position_step_mm"]),
                float(trajectory["max_look_angle_step_deg"]),
                int(trajectory["max_samples_per_path"]),
                float(trajectory["max_view_incidence_deg"]))
            accepted = [route for route in routes if route.accepted]
            plan_report = {
                "referenceTcpMmRpyDeg": self.reference_pose,
                "referenceImage": reference_image,
                "referenceGeometryQuality": reference_quality,
                "targetBaseMm": geometry.target_base_m * 1000.0,
                "targetNormalBase": geometry.target_normal_base,
                "routes": [{
                    "name": route.name,
                    "azimuthDeg": route.azimuth_deg,
                    "elevationDeg": route.elevation_deg,
                    "distanceOffsetMm": route.distance_offset_mm,
                    "accepted": route.accepted,
                    "rejectionReason": route.rejection_reason,
                    "waypoints": [{
                        "tcpMmRpyDeg": point.tcp_mm_rpy_deg,
                        "roiPx": point.projected_roi_px,
                        "roiVisible": point.roi_visible,
                        "cameraDistanceMm": point.camera_distance_mm,
                    } for point in route.waypoints],
                } for route in routes],
            }
            (self.session_dir / "plan.json").write_text(
                json.dumps(_jsonable(plan_report), ensure_ascii=False, indent=2),
                encoding="utf-8")
            if not accepted:
                raise RuntimeError("所有采集路线都因ROI或运动范围约束被拒绝")
            if bool(trajectory.get("fail_if_any_route_rejected", True)) and len(accepted) != len(routes):
                rejected = ", ".join(
                    f"{route.name}: {route.rejection_reason}"
                    for route in routes if not route.accepted)
                raise RuntimeError(f"存在不可执行路线，机械臂尚未运动: {rejected}")

            for route in accepted:
                # 从参考位沿同一路径逐点走到侧向起点，不拍照；避免跨方向直连。
                for point in reversed(route.waypoints[:-1]):
                    moved = True
                    self._move_verified(
                        geometry, point,
                        f"datas_get_{route.name}_out_{point.index:03d}")
                # 从侧向/远近起点逐点回到参考拍照位并采集。
                for point in route.waypoints:
                    moved = True
                    before, before_verification = self._move_verified(
                        geometry, point,
                        f"datas_get_{route.name}_in_{point.index:03d}")
                    file_stem = f"{route.name}_{point.index:03d}"
                    image_path = self._capture_to_session(file_stem)
                    after = _actual_pose(self.robot)
                    after_verification = evaluate_actual_waypoint(
                        geometry, point, after,
                        int(trajectory["image_margin_px"]))
                    self._append_record({
                        "image": str(image_path.relative_to(self.session_dir)),
                        "route": route.name,
                        "sampleIndex": point.index,
                        "azimuthDeg": route.azimuth_deg,
                        "elevationDeg": route.elevation_deg,
                        "distanceOffsetMm": route.distance_offset_mm,
                        "cameraDistanceMm": point.camera_distance_mm,
                        "plannedTcpMmRpyDeg": point.tcp_mm_rpy_deg,
                        "actualTcpBeforeMmRpyDeg": before,
                        "actualTcpAfterMmRpyDeg": after,
                        "projectedRoiPx": point.projected_roi_px,
                        "roiVisible": point.roi_visible,
                        "verificationBeforeCapture": before_verification,
                        "verificationAfterCapture": after_verification,
                    })
            summary = {
                "status": "completed",
                "imageCount": len(self.records) + 1,
                "sampleCount": len(self.records),
                "routeCount": len(accepted),
                "referenceTcpMmRpyDeg": self.reference_pose,
                "targetBaseMm": geometry.target_base_m * 1000.0,
                "referenceGeometryQuality": reference_quality,
            }
            (self.session_dir / "session.json").write_text(
                json.dumps(_jsonable(summary), ensure_ascii=False, indent=2),
                encoding="utf-8")
            return self.session_dir
        finally:
            if (moved and self.reference_pose is not None and
                    bool(self.config.trajectory.get("return_to_reference", True))):
                try:
                    _move(self.robot, self.reference_pose,
                          "datas_get_return_reference")
                except Exception as exc:
                    print(f"警告：返回初始拍照位失败：{exc}")
            try:
                self.robot.disconnect()
            except Exception as exc:
                print(f"警告：断开机械臂失败：{exc}")
