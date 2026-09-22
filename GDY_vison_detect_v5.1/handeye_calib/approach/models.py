# -*- coding: utf-8 -*-
"""接近流程的类型化数据模型。

核心模块之间传 dataclass，不传裸 dict。只有 JSON/配置边界调用 ``to_dict``、
``from_dict``。除 CameraFrame 厂商原始点云外，所有三维位置统一为米，角度统一为
弧度，齐次矩阵采用 ``T_A_B``（把 B 系转换到 A 系）。
"""
from __future__ import annotations

import json
import time
from dataclasses import dataclass, field, fields, is_dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Optional, Sequence, Tuple

import numpy as np

from ..hardware_interfaces import CameraFrame


_EPS = 1e-12


def json_ready(value: Any) -> Any:
    """递归转换为可由 json.dumps 写出的对象。"""
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    if is_dataclass(value):
        return {item.name: json_ready(getattr(value, item.name)) for item in fields(value)}
    if isinstance(value, Mapping):
        return {str(key): json_ready(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_ready(item) for item in value]
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return str(value)


def _vector(value: Any, length: int, name: str, *, allow_none: bool = False):
    if value is None and allow_none:
        return None
    array = np.asarray(value, dtype=np.float64)
    if array.size != length:
        raise ValueError(f"{name} 必须包含 {length} 个数，当前 shape={array.shape}")
    array = array.reshape(length).copy()
    if not np.isfinite(array).all():
        raise ValueError(f"{name} 包含 NaN/Inf")
    return array


def _unit_vector(value: Any, name: str, *, allow_none: bool = False):
    array = _vector(value, 3, name, allow_none=allow_none)
    if array is None:
        return None
    length = float(np.linalg.norm(array))
    if length < _EPS:
        raise ValueError(f"{name} 不能是零向量")
    return array / length


def _rotation(value: Any, name: str) -> np.ndarray:
    matrix = np.asarray(value, dtype=np.float64)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError(f"{name} 必须是有限的 3x3 矩阵")
    error = float(np.linalg.norm(matrix.T @ matrix - np.eye(3)))
    determinant = float(np.linalg.det(matrix))
    if error > 1e-5 or abs(determinant - 1.0) > 1e-5:
        raise ValueError(
            f"{name} 不是有效旋转矩阵：正交误差={error:.3g}, det={determinant:.9g}")
    return matrix.copy()


def _transform(value: Any, name: str) -> np.ndarray:
    matrix = np.asarray(value, dtype=np.float64)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError(f"{name} 必须是有限的 4x4 矩阵")
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9):
        raise ValueError(f"{name} 最后一行必须是 [0,0,0,1]")
    _rotation(matrix[:3, :3], f"{name}[:3,:3]")
    return matrix.copy()


def _positive(value: Any, name: str, *, allow_zero: bool = False) -> float:
    number = float(value)
    valid = number >= 0.0 if allow_zero else number > 0.0
    if not np.isfinite(number) or not valid:
        relation = "大于等于 0" if allow_zero else "大于 0"
        raise ValueError(f"{name} 必须是{relation}的有限数")
    return number


@dataclass(frozen=True)
class CameraModel:
    """与当前图像严格对应的相机模型。"""

    K: np.ndarray
    distortion: np.ndarray
    width: int
    height: int
    image_frame: str
    profile: str = ""
    intrinsics_source: str = ""

    def __post_init__(self) -> None:
        K = np.asarray(self.K, dtype=np.float64)
        if K.shape != (3, 3) or not np.isfinite(K).all():
            raise ValueError("CameraModel.K 必须是有限的 3x3 矩阵")
        distortion = np.asarray(self.distortion, dtype=np.float64).reshape(1, -1)
        if not np.isfinite(distortion).all():
            raise ValueError("CameraModel.distortion 包含 NaN/Inf")
        if int(self.width) <= 0 or int(self.height) <= 0:
            raise ValueError("CameraModel 图像宽高必须大于 0")
        object.__setattr__(self, "K", K.copy())
        object.__setattr__(self, "distortion", distortion.copy())
        object.__setattr__(self, "width", int(self.width))
        object.__setattr__(self, "height", int(self.height))

    def to_dict(self) -> dict:
        return json_ready(self)


@dataclass(frozen=True)
class CaptureContext:
    """相机帧与曝光前后机器人 TCP 的绑定结果。"""

    request_id: str
    frame: CameraFrame
    camera_model: CameraModel
    tcp_pose_before_m_rotvec_rad: np.ndarray
    tcp_pose_after_m_rotvec_rad: np.ndarray
    tcp_pose_capture_m_rotvec_rad: np.ndarray
    T_base_tcp_capture: np.ndarray
    pose_before_timestamp: float
    camera_timestamp: float
    pose_after_timestamp: float
    motion_during_capture_mm: float
    motion_during_capture_deg: float
    pose_stable: bool
    validation: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "tcp_pose_before_m_rotvec_rad", _vector(
            self.tcp_pose_before_m_rotvec_rad, 6, "tcp_pose_before_m_rotvec_rad"))
        object.__setattr__(self, "tcp_pose_after_m_rotvec_rad", _vector(
            self.tcp_pose_after_m_rotvec_rad, 6, "tcp_pose_after_m_rotvec_rad"))
        object.__setattr__(self, "tcp_pose_capture_m_rotvec_rad", _vector(
            self.tcp_pose_capture_m_rotvec_rad, 6, "tcp_pose_capture_m_rotvec_rad"))
        object.__setattr__(self, "T_base_tcp_capture", _transform(
            self.T_base_tcp_capture, "T_base_tcp_capture"))
        object.__setattr__(self, "validation", dict(self.validation))


@dataclass(frozen=True)
class FrameChannelRequirements:
    require_gray: bool = False
    require_color: bool = False
    require_depth: bool = False
    require_point_cloud: bool = False


@dataclass(frozen=True)
class Yolo2DDetection:
    """Ultralytics 检测框的类型化结果。"""

    instance_id: str
    class_id: int
    class_name: str
    confidence: float
    xywh_px: np.ndarray
    xyxy_px: np.ndarray

    def __post_init__(self) -> None:
        object.__setattr__(self, "xywh_px", _vector(self.xywh_px, 4, "xywh_px"))
        object.__setattr__(self, "xyxy_px", _vector(self.xyxy_px, 4, "xyxy_px"))

    def to_dict(self) -> dict:
        return json_ready(self)


@dataclass(frozen=True)
class TargetObservation:
    """一个目标在拍照相机坐标系中的三维几何观测。"""

    request_id: str
    frame_id: int
    instance_id: str
    class_id: int
    class_name: str
    confidence: float
    valid: bool
    center_camera_m: Optional[np.ndarray] = None
    surface_normal_camera: Optional[np.ndarray] = None
    approach_direction_camera: Optional[np.ndarray] = None
    tangent_x_camera: Optional[np.ndarray] = None
    T_camera_target: Optional[np.ndarray] = None
    bbox_2d: Optional[Tuple[float, float, float, float]] = None
    mask_2d: Optional[np.ndarray] = None
    quality_metrics: Dict[str, Any] = field(default_factory=dict)
    detector_name: str = ""
    detector_version: str = ""
    error_code: Optional[str] = None
    error_message: Optional[str] = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "confidence", float(self.confidence))
        object.__setattr__(self, "quality_metrics", dict(self.quality_metrics))
        if self.bbox_2d is not None:
            object.__setattr__(self, "bbox_2d", tuple(float(x) for x in self.bbox_2d))
        if self.valid:
            center = _vector(self.center_camera_m, 3, "center_camera_m")
            normal = _unit_vector(self.surface_normal_camera, "surface_normal_camera")
            approach = _unit_vector(
                self.approach_direction_camera, "approach_direction_camera")
            if float(np.dot(normal, approach)) > -0.99:
                raise ValueError("approach_direction_camera 必须与 surface_normal_camera 反向")
            object.__setattr__(self, "center_camera_m", center)
            object.__setattr__(self, "surface_normal_camera", normal)
            object.__setattr__(self, "approach_direction_camera", approach)
            object.__setattr__(self, "tangent_x_camera", _unit_vector(
                self.tangent_x_camera, "tangent_x_camera", allow_none=True))
            if self.T_camera_target is not None:
                object.__setattr__(self, "T_camera_target", _transform(
                    self.T_camera_target, "T_camera_target"))

    @classmethod
    def failed(cls, request_id: str, frame_id: int, detector_name: str,
               error_code: str, error_message: str) -> "TargetObservation":
        return cls(
            request_id=request_id, frame_id=frame_id,
            instance_id="", class_id=-1, class_name="", confidence=0.0,
            valid=False, detector_name=detector_name,
            error_code=error_code, error_message=error_message)

    def to_dict(self, include_mask: bool = False) -> dict:
        result = json_ready(self)
        if not include_mask:
            result.pop("mask_2d", None)
        return result


@dataclass(frozen=True)
class TargetDetectionBatch:
    request_id: str
    frame_id: int
    timestamp: float
    detector_name: str
    detector_version: str
    observations: Tuple[TargetObservation, ...]
    processing_time_ms: float
    errors: Tuple[str, ...] = ()
    debug_overlay: Optional[np.ndarray] = None

    def valid_observations(self) -> Tuple[TargetObservation, ...]:
        return tuple(item for item in self.observations if item.valid)

    def to_dict(self, include_overlay: bool = False) -> dict:
        result = {
            "request_id": self.request_id,
            "frame_id": self.frame_id,
            "timestamp": self.timestamp,
            "detector_name": self.detector_name,
            "detector_version": self.detector_version,
            "processing_time_ms": self.processing_time_ms,
            "errors": list(self.errors),
            "observations": [item.to_dict() for item in self.observations],
        }
        if include_overlay and self.debug_overlay is not None:
            result["debug_overlay"] = self.debug_overlay.tolist()
        return result


@dataclass(frozen=True)
class HandEyeCalibration:
    mode: str
    T_tcp_camera: np.ndarray
    accepted: bool
    quality_status: str
    source_path: Path
    metadata: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.mode not in ("eye-in-hand", "eye_in_hand"):
            raise ValueError(f"当前接近流程只支持 eye-in-hand，实际为 {self.mode!r}")
        object.__setattr__(self, "mode", "eye-in-hand")
        object.__setattr__(self, "T_tcp_camera", _transform(
            self.T_tcp_camera, "T_tcp_camera"))
        object.__setattr__(self, "source_path", Path(self.source_path))
        object.__setattr__(self, "metadata", dict(self.metadata))

    @classmethod
    def load(cls, path: Any, require_accepted: bool = True) -> "HandEyeCalibration":
        source = Path(path).expanduser().resolve()
        if not source.is_file():
            raise FileNotFoundError(f"手眼标定结果不存在: {source}")
        data = json.loads(source.read_text(encoding="utf-8"))
        mode = str(data.get("mode") or data.get("calibration_mode") or "")
        matrix = data.get("T_tcp_camera")
        if matrix is None:
            transform = data.get("transform") or {}
            if transform.get("name") == "T_tcp_camera":
                matrix = transform.get("matrix")
        if matrix is None:
            raise ValueError("手眼结果中未找到 T_tcp_camera")
        quality = data.get("quality") or {}
        accepted = bool(quality.get("accepted", False))
        status = str(quality.get("status", "UNKNOWN"))
        if require_accepted and not accepted:
            raise ValueError(
                f"手眼标定质量未通过：status={status}, accepted={accepted}")
        return cls(
            mode=mode, T_tcp_camera=np.asarray(matrix, dtype=np.float64),
            accepted=accepted, quality_status=status, source_path=source,
            metadata={
                "schema_version": data.get("schema_version"),
                "algorithm_version": data.get("algorithm_version"),
                "target": data.get("target"),
                "quality": quality,
            })

    def to_dict(self) -> dict:
        return json_ready(self)


@dataclass(frozen=True)
class ApproachSpec:
    """内部单位固定为 m、m/s、m/s²。"""

    preapproach_standoff_m: float = 0.25
    final_standoff_m: float = 0.10
    reference_point: str = "camera_origin"
    roll_policy: str = "target_x_axis"
    preapproach_speed_m_s: float = 0.05
    preapproach_accel_m_s2: float = 0.10
    normal_entry_speed_m_s: float = 0.01
    normal_entry_accel_m_s2: float = 0.03
    retreat_speed_m_s: float = 0.03
    retreat_accel_m_s2: float = 0.06
    execute_final_entry: bool = False

    def __post_init__(self) -> None:
        pre = _positive(self.preapproach_standoff_m, "preapproach_standoff_m")
        final = _positive(self.final_standoff_m, "final_standoff_m")
        if pre <= final:
            raise ValueError("preapproach_standoff_m 必须大于 final_standoff_m")
        if self.reference_point != "camera_origin":
            raise ValueError("当前版本 reference_point 仅支持 camera_origin")
        if self.roll_policy not in ("target_x_axis", "keep_capture_roll", "detected_angle"):
            raise ValueError(f"不支持的 roll_policy: {self.roll_policy}")
        for name in (
                "preapproach_speed_m_s", "preapproach_accel_m_s2",
                "normal_entry_speed_m_s", "normal_entry_accel_m_s2",
                "retreat_speed_m_s", "retreat_accel_m_s2"):
            _positive(getattr(self, name), name)

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "ApproachSpec":
        def mm(name: str, default_mm: float) -> float:
            return float(data.get(name, default_mm)) * 0.001

        return cls(
            preapproach_standoff_m=mm("preapproach_standoff_mm", 250.0),
            final_standoff_m=mm("final_standoff_mm", 100.0),
            reference_point=str(data.get("reference_point", "camera_origin")),
            roll_policy=str(data.get("roll_policy", "target_x_axis")),
            preapproach_speed_m_s=mm("preapproach_speed_mm_s", 50.0),
            preapproach_accel_m_s2=mm("preapproach_accel_mm_s2", 100.0),
            normal_entry_speed_m_s=mm("normal_entry_speed_mm_s", 10.0),
            normal_entry_accel_m_s2=mm("normal_entry_accel_mm_s2", 30.0),
            retreat_speed_m_s=mm("retreat_speed_mm_s", 30.0),
            retreat_accel_m_s2=mm("retreat_accel_mm_s2", 60.0),
            execute_final_entry=bool(data.get("execute_final_entry", False)),
        )

    @classmethod
    def load(cls, path: Any) -> "ApproachSpec":
        return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))

    def to_dict(self) -> dict:
        return {
            "preapproach_standoff_mm": self.preapproach_standoff_m * 1000.0,
            "final_standoff_mm": self.final_standoff_m * 1000.0,
            "reference_point": self.reference_point,
            "roll_policy": self.roll_policy,
            "preapproach_speed_mm_s": self.preapproach_speed_m_s * 1000.0,
            "preapproach_accel_mm_s2": self.preapproach_accel_m_s2 * 1000.0,
            "normal_entry_speed_mm_s": self.normal_entry_speed_m_s * 1000.0,
            "normal_entry_accel_mm_s2": self.normal_entry_accel_m_s2 * 1000.0,
            "retreat_speed_mm_s": self.retreat_speed_m_s * 1000.0,
            "retreat_accel_mm_s2": self.retreat_accel_m_s2 * 1000.0,
            "execute_final_entry": self.execute_final_entry,
        }


@dataclass(frozen=True)
class LocalizedTarget:
    """一个观测经拍照 TCP 和手眼矩阵变换后的机器人基座系目标。"""

    request_id: str
    instance_id: str
    class_id: int
    class_name: str
    confidence: float
    center_base_m: np.ndarray
    surface_normal_base: np.ndarray
    approach_direction_base: np.ndarray
    tangent_x_base: Optional[np.ndarray]

    def __post_init__(self) -> None:
        object.__setattr__(self, "center_base_m", _vector(
            self.center_base_m, 3, "center_base_m"))
        object.__setattr__(self, "surface_normal_base", _unit_vector(
            self.surface_normal_base, "surface_normal_base"))
        object.__setattr__(self, "approach_direction_base", _unit_vector(
            self.approach_direction_base, "approach_direction_base"))
        object.__setattr__(self, "tangent_x_base", _unit_vector(
            self.tangent_x_base, "tangent_x_base", allow_none=True))


@dataclass(frozen=True)
class MotionPlan:
    request_id: str
    target_instance_id: str
    target_class_name: str
    target_confidence: float
    target_center_base_m: np.ndarray
    surface_normal_base: np.ndarray
    approach_direction_base: np.ndarray
    T_base_camera_capture: np.ndarray
    T_base_camera_preapproach: np.ndarray
    T_base_camera_approach: np.ndarray
    T_base_tcp_preapproach: np.ndarray
    T_base_tcp_approach: np.ndarray
    preapproach_tcp_pose_m_rotvec_rad: np.ndarray
    approach_tcp_pose_m_rotvec_rad: np.ndarray
    normal_entry_distance_m: float
    current_to_preapproach_distance_m: float
    rotation_delta_deg: float
    spec: ApproachSpec
    planning_metrics: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        for name in ("target_center_base_m", "surface_normal_base",
                     "approach_direction_base"):
            value = (_unit_vector(getattr(self, name), name)
                     if name != "target_center_base_m"
                     else _vector(getattr(self, name), 3, name))
            object.__setattr__(self, name, value)
        for name in (
                "T_base_camera_capture", "T_base_camera_preapproach",
                "T_base_camera_approach", "T_base_tcp_preapproach",
                "T_base_tcp_approach"):
            object.__setattr__(self, name, _transform(getattr(self, name), name))
        object.__setattr__(self, "preapproach_tcp_pose_m_rotvec_rad", _vector(
            self.preapproach_tcp_pose_m_rotvec_rad, 6,
            "preapproach_tcp_pose_m_rotvec_rad"))
        object.__setattr__(self, "approach_tcp_pose_m_rotvec_rad", _vector(
            self.approach_tcp_pose_m_rotvec_rad, 6,
            "approach_tcp_pose_m_rotvec_rad"))
        object.__setattr__(self, "planning_metrics", dict(self.planning_metrics))

    def to_dict(self) -> dict:
        return json_ready(self)


@dataclass(frozen=True)
class SafetyConfig:
    min_confidence: float = 0.5
    max_capture_motion_mm: float = 0.5
    max_capture_motion_deg: float = 0.1
    max_target_age_s: float = 30.0
    max_current_to_preapproach_m: float = 1.0
    max_rotation_delta_deg: float = 120.0
    min_final_standoff_m: float = 0.05
    workspace_bounds_m: Dict[str, Tuple[float, float]] = field(default_factory=dict)
    forbidden_zones: Tuple[Dict[str, Any], ...] = ()

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "SafetyConfig":
        bounds = {}
        for axis, value in (data.get("workspace_bounds_m") or {}).items():
            bounds[str(axis)] = (float(value[0]), float(value[1]))
        return cls(
            min_confidence=float(data.get("min_confidence", 0.5)),
            max_capture_motion_mm=float(data.get("max_capture_motion_mm", 0.5)),
            max_capture_motion_deg=float(data.get("max_capture_motion_deg", 0.1)),
            max_target_age_s=float(data.get("max_target_age_s", 30.0)),
            max_current_to_preapproach_m=float(
                data.get("max_current_to_preapproach_mm", 1000.0)) * 0.001,
            max_rotation_delta_deg=float(data.get("max_rotation_delta_deg", 120.0)),
            min_final_standoff_m=float(
                data.get("min_final_standoff_mm", 50.0)) * 0.001,
            workspace_bounds_m=bounds,
            forbidden_zones=tuple(data.get("forbidden_zones") or ()),
        )


@dataclass(frozen=True)
class SafetyReport:
    accepted: bool
    errors: Tuple[str, ...]
    warnings: Tuple[str, ...]
    checked_at: float
    request_id: str
    metrics: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict:
        return json_ready(self)


@dataclass(frozen=True)
class MotionOptions:
    dry_run: bool = True
    execute_preapproach: bool = True
    execute_final_entry: bool = False
    retreat_after_final: bool = True
    confirmation_token: Optional[str] = None
    position_tolerance_m: float = 0.002
    orientation_tolerance_rad: float = np.deg2rad(1.0)


@dataclass(frozen=True)
class MotionResult:
    request_id: str
    target_instance_id: str
    success: bool
    completed_stage: str
    elapsed_s: float
    actual_tcp_pose_m_rotvec_rad: Optional[np.ndarray] = None
    position_error_m: Optional[float] = None
    orientation_error_rad: Optional[float] = None
    error_code: Optional[str] = None
    error_message: Optional[str] = None

    def to_dict(self) -> dict:
        return json_ready(self)


@dataclass(frozen=True)
class ScanResult:
    capture: CaptureContext
    detections: TargetDetectionBatch


@dataclass(frozen=True)
class HeldApproach:
    """到达最终工作位但尚未撤退的一次分阶段任务。"""

    scan: ScanResult
    observation: TargetObservation
    plan: MotionPlan
    safety: SafetyReport
    approach_motion: MotionResult


@dataclass(frozen=True)
class WorkflowTargetResult:
    instance_id: str
    class_name: str
    success: bool
    plan: Optional[MotionPlan]
    safety: Optional[SafetyReport]
    motion: Optional[MotionResult]
    error_message: Optional[str] = None

    def to_dict(self) -> dict:
        return json_ready(self)


@dataclass(frozen=True)
class WorkflowReport:
    job_id: str
    success: bool
    started_at: float
    completed_at: float
    initial_scan_request_id: str
    target_results: Tuple[WorkflowTargetResult, ...]
    errors: Tuple[str, ...] = ()
    initial_capture_tcp_pose_m_rotvec_rad: Optional[np.ndarray] = None
    return_to_capture_pose_enabled: bool = False

    def __post_init__(self) -> None:
        if self.initial_capture_tcp_pose_m_rotvec_rad is not None:
            object.__setattr__(
                self, "initial_capture_tcp_pose_m_rotvec_rad",
                _vector(self.initial_capture_tcp_pose_m_rotvec_rad, 6,
                        "initial_capture_tcp_pose_m_rotvec_rad"))
        object.__setattr__(
            self, "return_to_capture_pose_enabled",
            bool(self.return_to_capture_pose_enabled))

    def to_dict(self) -> dict:
        return json_ready(self)

    def save(self, path: Any) -> Path:
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(
            json.dumps(self.to_dict(), ensure_ascii=False, indent=2),
            encoding="utf-8")
        return destination
