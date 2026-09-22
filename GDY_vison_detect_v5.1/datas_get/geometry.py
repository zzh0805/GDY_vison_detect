# -*- coding: utf-8 -*-
"""目标中心锁定、ROI投影和多方向轨迹规划的纯计算模块。"""
from __future__ import annotations

import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import cv2
import numpy as np

from handeye_calib import handeye_math as hm
from handeye_calib.jaka_adapter import (
    jaka_pose_to_unified, unified_pose_to_jaka)


@dataclass(frozen=True)
class CameraIntrinsics:
    K: np.ndarray
    distortion: np.ndarray
    width: int
    height: int


@dataclass(frozen=True)
class ReferenceGeometry:
    reference_tcp_mm_rpy_deg: np.ndarray
    T_base_tcp: np.ndarray
    T_tcp_camera: np.ndarray
    T_base_camera: np.ndarray
    target_base_m: np.ndarray
    roi_corners_base_m: np.ndarray
    target_normal_base: np.ndarray
    intrinsics: CameraIntrinsics


@dataclass(frozen=True)
class Waypoint:
    route_name: str
    index: int
    T_base_camera: np.ndarray
    T_base_tcp: np.ndarray
    tcp_mm_rpy_deg: np.ndarray
    projected_roi_px: np.ndarray
    roi_visible: bool
    camera_distance_mm: float


@dataclass(frozen=True)
class RoutePlan:
    name: str
    azimuth_deg: float
    elevation_deg: float
    distance_offset_mm: float
    waypoints: tuple[Waypoint, ...]
    accepted: bool
    rejection_reason: str = ""


def _matrix_from_jaka_deg(pose: Any) -> np.ndarray:
    values = np.asarray(pose, dtype=np.float64).reshape(6).copy()
    values[3:] = np.radians(values[3:])
    unified = jaka_pose_to_unified(values)
    return hm.homogeneous(hm._rotvec_to_mat(unified[3:]), unified[:3])


def _jaka_deg_from_matrix(transform: np.ndarray) -> np.ndarray:
    transform = np.asarray(transform, dtype=np.float64).reshape(4, 4)
    unified = np.r_[transform[:3, 3], hm._mat_to_rotvec(transform[:3, :3])]
    pose = unified_pose_to_jaka(unified)
    pose[3:] = np.degrees(pose[3:])
    return pose


def load_handeye_matrix(path: Path) -> np.ndarray:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    matrix = data.get("T_tcp_camera")
    if matrix is None:
        transform = data.get("transform") or {}
        matrix = transform.get("matrix") if transform.get("name") == "T_tcp_camera" else None
    result = np.asarray(matrix, dtype=np.float64)
    if result.shape != (4, 4) or not np.isfinite(result).all():
        raise ValueError("手眼文件没有有效的T_tcp_camera")
    if not np.allclose(result[3], [0, 0, 0, 1], atol=1e-9):
        raise ValueError("T_tcp_camera末行无效")
    return result


def load_scaled_rgb_intrinsics(path: Path, output_width: int,
                               output_height: int) -> CameraIntrinsics:
    """读取厂家OpenCV YAML中的RGB内参并缩放到实际保存图尺寸。"""
    text = Path(path).read_text(encoding="utf-8-sig")
    width_match = re.search(r"^rgb_width:\s*(\d+)\s*$", text, re.MULTILINE)
    height_match = re.search(r"^rgb_height:\s*(\d+)\s*$", text, re.MULTILINE)
    block_match = re.search(
        r"^cameraMatrix_rgb:\s*!!opencv-matrix\s*(.*?)(?=^\S|\Z)",
        text, re.MULTILINE | re.DOTALL)
    if not (width_match and height_match and block_match):
        raise ValueError("相机YAML缺少rgb_width/rgb_height/cameraMatrix_rgb")
    data_match = re.search(r"data:\s*\[([^\]]+)\]", block_match.group(1), re.DOTALL)
    if not data_match:
        raise ValueError("cameraMatrix_rgb缺少data")
    values = np.fromstring(data_match.group(1).replace("\n", " "), sep=",")
    if values.size != 9 or not np.isfinite(values).all():
        raise ValueError("cameraMatrix_rgb必须包含9个有效数字")
    source_width, source_height = int(width_match.group(1)), int(height_match.group(1))
    K = values.reshape(3, 3)
    K[0, :] *= float(output_width) / source_width
    K[1, :] *= float(output_height) / source_height
    K[2, :] = [0.0, 0.0, 1.0]
    distortion_match = re.search(
        r"^distCoeffs_rgb_sdk_order:\s*!!opencv-matrix\s*(.*?)(?=^\S|\Z)",
        text, re.MULTILINE | re.DOTALL)
    distortion = np.zeros(5, dtype=np.float64)
    if distortion_match:
        data_match = re.search(
            r"data:\s*\[([^\]]+)\]", distortion_match.group(1), re.DOTALL)
        if data_match:
            parsed = np.fromstring(
                data_match.group(1).replace("\n", " "), sep=",")
            if parsed.size and np.isfinite(parsed).all():
                distortion = parsed
    return CameraIntrinsics(
        K=K, distortion=distortion,
        width=int(output_width), height=int(output_height))


def _pixels_at_z(pixels: np.ndarray, intrinsics: CameraIntrinsics,
                 z_m: float) -> np.ndarray:
    pixels = np.asarray(pixels, dtype=np.float64).reshape(-1, 2)
    normalized = cv2.undistortPoints(
        pixels.reshape(-1, 1, 2), intrinsics.K,
        intrinsics.distortion.reshape(1, -1)).reshape(-1, 2)
    rays = np.c_[normalized, np.ones(len(normalized))]
    scale = z_m / rays[:, 2]
    return rays * scale[:, None]


def make_reference_geometry_from_camera_points(
        reference_tcp_mm_rpy_deg: Any,
        T_tcp_camera: np.ndarray,
        intrinsics: CameraIntrinsics,
        target_camera_mm: Any,
        roi_corners_camera_mm: Any,
        target_normal_camera: Any) -> ReferenceGeometry:
    """把专用三维参考接口返回的相机系几何锁定到JAKA基座系。"""
    T_base_tcp = _matrix_from_jaka_deg(reference_tcp_mm_rpy_deg)
    T_tcp_camera = np.asarray(T_tcp_camera, dtype=np.float64).reshape(4, 4)
    T_base_camera = T_base_tcp @ T_tcp_camera
    target_camera = np.asarray(
        target_camera_mm, dtype=np.float64).reshape(3) * 0.001
    corners_camera = np.asarray(
        roi_corners_camera_mm, dtype=np.float64).reshape(4, 3) * 0.001
    normal_camera = np.asarray(
        target_normal_camera, dtype=np.float64).reshape(3)
    if not (np.isfinite(target_camera).all() and
            np.isfinite(corners_camera).all() and
            np.isfinite(normal_camera).all()):
        raise ValueError("三维参考几何包含NaN/Inf")
    normal_camera = _unit(normal_camera, "目标平面法向")
    corners_base = (
        T_base_camera @ np.c_[corners_camera, np.ones(4)].T).T[:, :3]
    target_base = (T_base_camera @ np.r_[target_camera, 1.0])[:3]
    normal_base = T_base_camera[:3, :3] @ normal_camera
    return ReferenceGeometry(
        reference_tcp_mm_rpy_deg=np.asarray(
            reference_tcp_mm_rpy_deg, dtype=np.float64).reshape(6).copy(),
        T_base_tcp=T_base_tcp, T_tcp_camera=T_tcp_camera,
        T_base_camera=T_base_camera, target_base_m=target_base,
        roi_corners_base_m=corners_base,
        target_normal_base=_unit(normal_base, "基座系目标平面法向"),
        intrinsics=intrinsics)


def make_reference_geometry(reference_tcp_mm_rpy_deg: Any,
                            T_tcp_camera: np.ndarray,
                            intrinsics: CameraIntrinsics,
                            roi_xyxy_px: Any,
                            target_depth_mm: float) -> ReferenceGeometry:
    x1, y1, x2, y2 = np.asarray(roi_xyxy_px, dtype=np.float64).reshape(4)
    corners_px = np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]])
    center_px = np.array([[(x1 + x2) * 0.5, (y1 + y2) * 0.5]])
    z_m = float(target_depth_mm) * 0.001
    corners_camera = _pixels_at_z(corners_px, intrinsics, z_m)
    center_camera = _pixels_at_z(center_px, intrinsics, z_m)[0]
    normal_camera = _unit(-center_camera, "配置深度目标朝向相机的法向")
    return make_reference_geometry_from_camera_points(
        reference_tcp_mm_rpy_deg, T_tcp_camera, intrinsics,
        center_camera * 1000.0, corners_camera * 1000.0, normal_camera)


def _unit(vector: np.ndarray, name: str) -> np.ndarray:
    vector = np.asarray(vector, dtype=np.float64).reshape(3)
    norm = float(np.linalg.norm(vector))
    if norm < 1e-10:
        raise ValueError(f"{name}长度过小")
    return vector / norm


def _look_at(camera_position: np.ndarray, target: np.ndarray,
             x_hint: np.ndarray) -> np.ndarray:
    z_axis = _unit(target - camera_position, "相机到目标方向")
    x_axis = np.asarray(x_hint, dtype=np.float64) - z_axis * float(np.dot(x_hint, z_axis))
    if np.linalg.norm(x_axis) < 1e-8:
        fallback = np.array([1.0, 0.0, 0.0])
        if abs(float(np.dot(fallback, z_axis))) > 0.9:
            fallback = np.array([0.0, 1.0, 0.0])
        x_axis = fallback - z_axis * float(np.dot(fallback, z_axis))
    x_axis = _unit(x_axis, "相机横轴")
    y_axis = _unit(np.cross(z_axis, x_axis), "相机纵轴")
    return np.column_stack([x_axis, y_axis, z_axis])


def _project(points_base: np.ndarray, T_base_camera: np.ndarray,
             intrinsics: CameraIntrinsics, margin_px: int) -> tuple[np.ndarray, bool]:
    points = np.asarray(points_base, dtype=np.float64).reshape(-1, 3)
    T_camera_base = np.linalg.inv(T_base_camera)
    camera = (T_camera_base @ np.c_[points, np.ones(len(points))].T).T[:, :3]
    positive = camera[:, 2] > 1e-6
    pixels, _ = cv2.projectPoints(
        camera, np.zeros(3), np.zeros(3), intrinsics.K,
        intrinsics.distortion.reshape(1, -1))
    pixels = pixels.reshape(-1, 2)
    m = float(margin_px)
    visible = bool(
        positive.all() and np.isfinite(pixels).all() and
        (pixels[:, 0] >= m).all() and
        (pixels[:, 0] < intrinsics.width - m).all() and
        (pixels[:, 1] >= m).all() and
        (pixels[:, 1] < intrinsics.height - m).all())
    return pixels, visible


def _rotation_angle_deg(first: np.ndarray, second: np.ndarray) -> float:
    relative = first.T @ second
    cosine = np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0)
    return float(np.degrees(np.arccos(cosine)))


def _unwrap_rpy_deg(current: np.ndarray, previous: np.ndarray) -> np.ndarray:
    result = np.asarray(current, dtype=np.float64).copy()
    previous = np.asarray(previous, dtype=np.float64).reshape(3)
    result += 360.0 * np.round((previous - result) / 360.0)
    return result


def _angle_between_deg(first: np.ndarray, second: np.ndarray) -> float:
    first = _unit(first, "第一方向")
    second = _unit(second, "第二方向")
    return float(np.degrees(np.arccos(np.clip(
        float(np.dot(first, second)), -1.0, 1.0))))


def evaluate_actual_waypoint(reference: ReferenceGeometry,
                             waypoint: Waypoint,
                             actual_tcp_mm_rpy_deg: Any,
                             image_margin_px: int) -> dict:
    """根据机器人实测TCP验证到位、光轴对准和ROI可见性。"""
    T_base_tcp = _matrix_from_jaka_deg(actual_tcp_mm_rpy_deg)
    T_base_camera = T_base_tcp @ reference.T_tcp_camera
    target_direction = reference.target_base_m - T_base_camera[:3, 3]
    look_at_error = _angle_between_deg(
        T_base_camera[:3, 2], target_direction)
    roi_px, roi_visible = _project(
        reference.roi_corners_base_m, T_base_camera,
        reference.intrinsics, image_margin_px)
    target_px, target_visible = _project(
        reference.target_base_m.reshape(1, 3), T_base_camera,
        reference.intrinsics, image_margin_px)
    return {
        "positionErrorMm": float(np.linalg.norm(
            T_base_tcp[:3, 3] - waypoint.T_base_tcp[:3, 3]) * 1000.0),
        "rotationErrorDeg": _rotation_angle_deg(
            waypoint.T_base_tcp[:3, :3], T_base_tcp[:3, :3]),
        "lookAtErrorDeg": look_at_error,
        "actualProjectedRoiPx": roi_px,
        "actualProjectedTargetPx": target_px[0],
        "roiVisible": bool(roi_visible and target_visible),
    }


def build_routes(reference: ReferenceGeometry, views: Iterable[dict],
                 samples_per_path: int, image_margin_px: int,
                 max_tcp_translation_mm: float,
                 max_tcp_rotation_deg: float,
                 preserve_reference_pose: bool = True,
                 max_position_step_mm: float | None = None,
                 max_look_angle_step_deg: float | None = None,
                 max_samples_per_path: int = 100,
                 max_view_incidence_deg: float = 89.0) -> tuple[RoutePlan, ...]:
    target = reference.target_base_m
    reference_position = reference.T_base_camera[:3, 3]
    radial = _unit(reference_position - target, "目标到参考相机方向")
    x_hint = reference.T_base_camera[:3, 0]
    right = _unit(x_hint - radial * float(np.dot(x_hint, radial)), "参考相机横向")
    up = _unit(np.cross(radial, right), "参考相机上向")
    reference_distance = float(np.linalg.norm(reference_position - target))
    plans = []
    for raw_view in views:
        name = str(raw_view["name"])
        azimuth = float(raw_view.get("azimuth_deg", 0.0))
        elevation = float(raw_view.get("elevation_deg", 0.0))
        distance_offset = float(raw_view.get("distance_offset_mm", 0.0))
        az = np.radians(azimuth)
        el = np.radians(elevation)
        radius = reference_distance + distance_offset * 0.001
        if radius <= 0.01:
            plans.append(RoutePlan(name, azimuth, elevation, distance_offset,
                                   (), False, "相机到目标距离小于10mm"))
            continue
        side_direction = _unit(
            np.cos(el) * (np.cos(az) * radial + np.sin(az) * right) +
            np.sin(el) * up,
            "侧向观察方向")
        side_position = target + radius * side_direction
        rejection = ""
        incidence = _angle_between_deg(side_direction, reference.target_normal_base)
        if incidence > float(max_view_incidence_deg):
            rejection = (
                f"侧向观察入射角{incidence:.1f}°超过限制"
                f"{float(max_view_incidence_deg):.1f}°")
        translation_path_mm = float(np.linalg.norm(
            side_position - reference_position) * 1000.0)
        view_angle_deg = _angle_between_deg(side_direction, radial)
        sample_count = int(samples_per_path)
        if max_position_step_mm is not None:
            sample_count = max(sample_count, int(np.ceil(
                translation_path_mm / float(max_position_step_mm))) + 1)
        if max_look_angle_step_deg is not None:
            sample_count = max(sample_count, int(np.ceil(
                view_angle_deg / float(max_look_angle_step_deg))) + 1)
        if sample_count > int(max_samples_per_path):
            rejection = rejection or (
                f"路线需要{sample_count}点，超过max_samples_per_path="
                f"{int(max_samples_per_path)}")
            sample_count = int(max_samples_per_path)
        positions = []
        for index in range(sample_count):
            alpha = index / float(sample_count - 1)
            positions.append(
                (1.0 - alpha) * side_position + alpha * reference_position)

        # 从参考位向侧向点做平行传输，保持相机横轴/滚转连续；采集顺序再反转。
        rotations_outward = []
        x_transport = x_hint.copy()
        for outward_index, position in enumerate(reversed(positions)):
            rotation = _look_at(position, target, x_transport)
            if preserve_reference_pose and outward_index == 0:
                rotation = reference.T_base_camera[:3, :3].copy()
            x_transport = rotation[:, 0].copy()
            rotations_outward.append(rotation)
        rotations = list(reversed(rotations_outward))

        transforms = []
        for position, rotation in zip(positions, rotations):
            T_base_camera = np.eye(4)
            T_base_camera[:3, :3] = rotation
            T_base_camera[:3, 3] = position
            T_base_tcp = T_base_camera @ np.linalg.inv(reference.T_tcp_camera)
            transforms.append((T_base_camera, T_base_tcp))

        # JAKA欧拉角选择靠近前一点的±360°等价表达，避免无意义绕转。
        poses_outward = []
        previous_rpy = reference.reference_tcp_mm_rpy_deg[3:].copy()
        for _, T_base_tcp in reversed(transforms):
            pose = _jaka_deg_from_matrix(T_base_tcp)
            pose[3:] = _unwrap_rpy_deg(pose[3:], previous_rpy)
            previous_rpy = pose[3:].copy()
            poses_outward.append(pose)
        poses = list(reversed(poses_outward))

        waypoints = []
        for index, ((T_base_camera, T_base_tcp), pose) in enumerate(
                zip(transforms, poses)):
            roi_px, visible = _project(
                reference.roi_corners_base_m, T_base_camera,
                reference.intrinsics, int(image_margin_px))
            translation_mm = float(np.linalg.norm(
                T_base_tcp[:3, 3] - reference.T_base_tcp[:3, 3]) * 1000.0)
            rotation_deg = _rotation_angle_deg(
                reference.T_base_tcp[:3, :3], T_base_tcp[:3, :3])
            if not visible and not rejection:
                rejection = f"第{index + 1}点ROI预计越出图像边界"
            if translation_mm > float(max_tcp_translation_mm) and not rejection:
                rejection = f"第{index + 1}点TCP平移{translation_mm:.1f}mm超过限制"
            if rotation_deg > float(max_tcp_rotation_deg) and not rejection:
                rejection = f"第{index + 1}点TCP旋转{rotation_deg:.1f}°超过限制"
            waypoints.append(Waypoint(
                route_name=name, index=index,
                T_base_camera=T_base_camera, T_base_tcp=T_base_tcp,
                tcp_mm_rpy_deg=pose,
                projected_roi_px=roi_px, roi_visible=visible,
                camera_distance_mm=float(np.linalg.norm(
                    T_base_camera[:3, 3] - target) * 1000.0)))
        plans.append(RoutePlan(
            name=name, azimuth_deg=azimuth, elevation_deg=elevation,
            distance_offset_mm=distance_offset, waypoints=tuple(waypoints),
            accepted=not rejection, rejection_reason=rejection))
    return tuple(plans)
