# -*- coding: utf-8 -*-
"""从相机观测解算唯一的 JAKA 目标 TCP，不执行机器人运动。"""
from __future__ import annotations

from typing import Any, Mapping, Optional

import numpy as np

from handeye_calib import handeye_math as hm
from handeye_calib.approach.models import HandEyeCalibration
from handeye_calib.jaka_adapter import (
    jaka_pose_to_unified, jaka_rpy_to_matrix, unified_pose_to_jaka)

from .models import TaskError, finite_vector, json_ready


def _transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    result[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return result


def _jaka_deg_to_matrix(pose_mm_rpy_deg: np.ndarray) -> np.ndarray:
    pose = finite_vector(pose_mm_rpy_deg, 6, "captureTcpMmRpyDeg")
    pose_rad = pose.copy()
    pose_rad[3:] = np.radians(pose_rad[3:])
    unified = jaka_pose_to_unified(pose_rad)
    return hm.homogeneous(hm._rotvec_to_mat(unified[3:]), unified[:3])


def _matrix_to_jaka_deg(transform: np.ndarray) -> np.ndarray:
    unified = np.concatenate([
        transform[:3, 3], hm._mat_to_rotvec(transform[:3, :3])])
    jaka = unified_pose_to_jaka(unified)
    jaka[3:] = np.degrees(jaka[3:])
    return jaka


def _desired_rotation(approach_direction: np.ndarray,
                      x_hint: np.ndarray) -> np.ndarray:
    z_axis = np.asarray(approach_direction, dtype=np.float64).reshape(3)
    z_axis /= np.linalg.norm(z_axis)
    x_axis = np.asarray(x_hint, dtype=np.float64).reshape(3)
    x_axis -= float(np.dot(x_axis, z_axis)) * z_axis
    if np.linalg.norm(x_axis) < 1e-8:
        basis = min(np.eye(3), key=lambda axis: abs(float(np.dot(axis, z_axis))))
        x_axis = basis - float(np.dot(basis, z_axis)) * z_axis
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    y_axis /= np.linalg.norm(y_axis)
    x_axis = np.cross(y_axis, z_axis)
    x_axis /= np.linalg.norm(x_axis)
    return np.column_stack((x_axis, y_axis, z_axis))


def _standard_to_tool(tool_config: Mapping[str, Any]) -> np.ndarray:
    """读取“修正后标准TCP -> 工具工作TCP”的固定局部变换。"""
    if "camera_to_tool" in tool_config:
        raise TaskError(
            "LEGACY_TOOL_OFFSET_NOT_SUPPORTED",
            "v2不再接受camera_to_tool，请先迁移为standard_to_tool")
    transform = dict(tool_config.get("standard_to_tool") or {})
    xyz_mm = finite_vector(transform.get("xyz_mm", [0, 0, 0]), 3,
                           "standard_to_tool.xyz_mm")
    rpy_deg = finite_vector(transform.get("rpy_deg", [0, 0, 0]), 3,
                            "standard_to_tool.rpy_deg")
    return _transform(
        jaka_rpy_to_matrix(np.radians(rpy_deg)), xyz_mm * 0.001)


class TargetPoseSolver:
    def __init__(self, calibration: HandEyeCalibration,
                 pose_config: Mapping[str, Any],
                 tools_config: Mapping[str, Any]):
        self.calibration = calibration
        self.pose_config = dict(pose_config)
        self.tools_config = dict(tools_config)
        # v2统一基准：先用逆手眼把50mm光心参考转换为活动TCP，再对该
        # 活动TCP施加一次现场全局修正。工具偏移只在修正后的标准TCP上叠加。
        # 全局修正沿用现场验证过的 JAKA 基座系 [mm, RPY°] 分量加法语义。
        self._tcp_correction = np.zeros(6, dtype=np.float64)
        tcp_corr = self.pose_config.get("tcp_correction")
        if tcp_corr:
            t_mm = finite_vector(
                tcp_corr.get("position_mm", [0, 0, 0]), 3,
                "tcp_correction.position_mm")
            rpy_deg = finite_vector(
                tcp_corr.get("rpy_deg", [0, 0, 0]), 3,
                "tcp_correction.rpy_deg")
            self._tcp_correction = np.concatenate([
                np.asarray(t_mm, dtype=np.float64),
                np.asarray(rpy_deg, dtype=np.float64)])

    def solve(self, observation: Any,
              capture_tcp_mm_rpy_deg: np.ndarray,
              tools_config: Optional[Mapping[str, Any]] = None) -> dict:
        if not observation.valid:
            raise TaskError(
                observation.error_code or "TARGET_GEOMETRY_FAILED",
                observation.error_message or "选中目标三维几何解算失败")
        T_base_tcp_capture = _jaka_deg_to_matrix(capture_tcp_mm_rpy_deg)
        T_base_camera_capture = (
            T_base_tcp_capture @ self.calibration.T_tcp_camera)
        rotation_capture = T_base_camera_capture[:3, :3]
        center_base = (
            rotation_capture @ observation.center_camera_m +
            T_base_camera_capture[:3, 3])
        normal_base = rotation_capture @ observation.surface_normal_camera
        normal_base /= np.linalg.norm(normal_base)
        approach_base = -normal_base
        desired_rotation = _desired_rotation(
            approach_base, rotation_capture[:3, 0])

        mode = str(self.pose_config.get("alignment_mode", "camera_center"))
        default_standoff_mm = float(self.pose_config.get("standoff_mm", 50.0))
        tool_id = None
        tool_config = None
        tool_offset_xyz_mm = np.zeros(3, dtype=np.float64)
        tool_offset_rpy_deg = np.zeros(3, dtype=np.float64)
        if mode == "tool":
            # 服务端每次解算都会传入刚从tool_offsets.yaml读取的快照。
            # 保留初始化配置仅用于直接调用TargetPoseSolver的兼容场景。
            effective_tools = (
                self.tools_config if tools_config is None
                else dict(tools_config))
            raw_tool = effective_tools.get(observation.class_name)
            if not isinstance(raw_tool, Mapping):
                raise TaskError(
                    "TOOL_MAPPING_MISSING",
                    f"类别{observation.class_name!r}没有配置工具偏移")
            tool_config = dict(raw_tool)
            if not bool(tool_config.get("enabled", True)):
                raise TaskError(
                    "TOOL_DISABLED",
                    f"类别{observation.class_name!r}对应工具已禁用")
            tool_id = str(tool_config.get("tool_id") or observation.class_name)
            standoff_mm = float(
                tool_config.get("standoff_mm", default_standoff_mm))
            raw_offset = dict(tool_config.get("standard_to_tool") or {})
            tool_offset_xyz_mm = finite_vector(
                raw_offset.get("xyz_mm", [0, 0, 0]), 3,
                "standard_to_tool.xyz_mm")
            tool_offset_rpy_deg = finite_vector(
                raw_offset.get("rpy_deg", [0, 0, 0]), 3,
                "standard_to_tool.rpy_deg")
        elif mode == "camera_center":
            standoff_mm = default_standoff_mm
        else:
            raise TaskError("INVALID_ALIGNMENT_MODE", f"不支持对齐模式{mode!r}")
        if standoff_mm < 0.0:
            raise TaskError("INVALID_STANDOFF", "standoff_mm不能小于0")

        aligned_origin = center_base + standoff_mm * 0.001 * normal_base

        # ===== v2统一位姿链 =====
        # ① T_base_camera_reference：光心位于目标法向外 standoff 处，
        #    相机光轴正对目标。
        # ② T_base_tcp_inverse_handeye = 光心参考 @ inv(T_tcp_camera)：
        #    将期望相机位姿反算成机器人活动TCP位姿。
        # ③ T_base_tcp_standard：对②的 JAKA 位姿施加一次 tcp_correction。
        #    该位姿是所有工具共同的、现场验证过的标准零位。
        # ④ T_base_tcp_target = 标准零位 @ standard_to_tool。零工具偏移为
        #    单位矩阵，因此自然得到“逆手眼 + 全局修正”的光心对齐结果。
        T_base_camera_reference = _transform(
            desired_rotation, aligned_origin)
        T_camera_tcp = np.linalg.inv(self.calibration.T_tcp_camera)
        T_base_tcp_inverse_handeye = (
            T_base_camera_reference @ T_camera_tcp)
        inverse_handeye_tcp = _matrix_to_jaka_deg(
            T_base_tcp_inverse_handeye)
        standard_tcp = inverse_handeye_tcp + self._tcp_correction
        if not np.isfinite(standard_tcp).all():
            raise TaskError("POSE_SOLVE_FAILED", "标准TCP包含NaN/Inf")
        T_base_tcp_standard = _jaka_deg_to_matrix(standard_tcp)

        T_standard_tool = _standard_to_tool(tool_config) if (
            mode == "tool" and tool_config is not None) else np.eye(4)
        T_base_tcp_target = T_base_tcp_standard @ T_standard_tool
        target_tcp = _matrix_to_jaka_deg(T_base_tcp_target)
        if not np.isfinite(target_tcp).all():
            raise TaskError("POSE_SOLVE_FAILED", "解算出的目标TCP包含NaN/Inf")

        # 这是按手眼矩阵预测的最终真实相机位姿，用于报告审计。工具偏移
        # 非零时，相机随机器人离开50mm参考位是预期行为。
        T_base_camera_target = (
            T_base_tcp_target @ self.calibration.T_tcp_camera)

        return json_ready({
            "alignmentMode": mode,
            "toolId": tool_id,
            "standoffMm": standoff_mm,
            "tcpCorrectionMmRpyDeg": self._tcp_correction,
            "toolOffsetDefinition": "standard_tcp_to_final_active_tcp",
            "standardToToolXyzMm": tool_offset_xyz_mm,
            "standardToToolRpyDeg": tool_offset_rpy_deg,
            "targetTcpMmRpyDeg": target_tcp,
            "referenceFrame": str(self.pose_config.get(
                "output_reference_frame", "jaka_base")),
            "poseReference": str(self.pose_config.get(
                "output_pose_reference", "active_tcp")),
            "targetCenterBaseMm": center_base * 1000.0,
            "surfaceNormalBase": normal_base,
            "approachDirectionBase": approach_base,
            "TBaseCameraCapture": T_base_camera_capture,
            "inverseHandeyeTcpMmRpyDeg": inverse_handeye_tcp,
            "standardTcpMmRpyDeg": standard_tcp,
            "toolTargetTcpMmRpyDeg": target_tcp,
            "TBaseCameraReference": T_base_camera_reference,
            "TCameraTcpInverseHandeye": T_camera_tcp,
            "TBaseTcpInverseHandeye": T_base_tcp_inverse_handeye,
            "TBaseTcpStandard": T_base_tcp_standard,
            "TStandardToolEffective": T_standard_tool,
            "TBaseCameraTarget": T_base_camera_target,
            "TBaseTcpTarget": T_base_tcp_target,
        })
