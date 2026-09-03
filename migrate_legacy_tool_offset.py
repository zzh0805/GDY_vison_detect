# -*- coding: utf-8 -*-
"""把GDY视觉v1 camera_to_tool迁移为v2 standard_to_tool。"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

from calibrate_tool_offset import (
    _fmt, _matrix_to_pose, _parse_values, _pose_to_matrix,
    _validate_transform, _yaml_block)


def migrate(
        T_base_camera_reference: np.ndarray,
        T_tcp_camera: np.ndarray,
        tcp_correction: np.ndarray,
        *,
        taught_tcp: Optional[np.ndarray] = None,
        legacy_camera_to_tool: Optional[np.ndarray] = None,
) -> dict:
    """返回标准TCP、旧最终TCP和迁移后的固定工具偏移。"""
    T_camera_reference = _validate_transform(
        T_base_camera_reference, "TBaseCameraReference")
    T_handeye = _validate_transform(T_tcp_camera, "T_tcp_camera")
    correction = np.asarray(tcp_correction, dtype=np.float64).reshape(6)

    inverse_handeye_pose = _matrix_to_pose(
        T_camera_reference @ np.linalg.inv(T_handeye))
    standard_pose = inverse_handeye_pose + correction
    T_standard = _pose_to_matrix(standard_pose)

    if taught_tcp is not None:
        final_pose = np.asarray(taught_tcp, dtype=np.float64).reshape(6)
        final_source = "saved_taught_tcp"
    elif legacy_camera_to_tool is not None:
        T_legacy_offset = _pose_to_matrix(legacy_camera_to_tool)
        old_raw_pose = _matrix_to_pose(T_camera_reference @ T_legacy_offset)
        final_pose = old_raw_pose + correction
        final_source = "reconstructed_v1_runtime_target"
    else:
        raise ValueError("必须提供taught_tcp或legacy_camera_to_tool")

    T_final = _pose_to_matrix(final_pose)
    T_standard_tool = _validate_transform(
        np.linalg.inv(T_standard) @ T_final, "T_standard_tool")
    offset_pose = _matrix_to_pose(T_standard_tool)
    reconstructed = T_standard @ T_standard_tool
    return {
        "inverse_handeye_tcp_mm_rpy_deg": inverse_handeye_pose,
        "standard_tcp_mm_rpy_deg": standard_pose,
        "legacy_final_source": final_source,
        "legacy_final_tcp_mm_rpy_deg": final_pose,
        "standard_to_tool_mm_rpy_deg": offset_pose,
        "position_residual_mm": float(np.linalg.norm(
            reconstructed[:3, 3] - T_final[:3, 3]) * 1000.0),
    }


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--legacy-result-json", type=Path, required=True)
    parser.add_argument("--handeye-result", type=Path, required=True)
    parser.add_argument(
        "--tcp-correction", required=True,
        help="position_mm+rpy_deg，共6个逗号分隔数字")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--taught-tcp", help="保存的最终正确示教TCP")
    source.add_argument(
        "--legacy-camera-to-tool", help="v1 camera_to_tool的xyz+rpy")
    parser.add_argument("--class-name", default="<new_class>")
    parser.add_argument("--tool-id", default="tool_<new_class>")
    args = parser.parse_args(argv)

    result = json.loads(args.legacy_result_json.read_text(encoding="utf-8"))
    T_reference = np.asarray(
        result["cameraReference"]["TBaseCameraReference"], dtype=np.float64)
    handeye = json.loads(args.handeye_result.read_text(encoding="utf-8"))
    T_handeye = np.asarray(handeye["T_tcp_camera"], dtype=np.float64)
    correction = _parse_values(
        args.tcp_correction, "--tcp-correction", 6)
    taught = (_parse_values(args.taught_tcp, "--taught-tcp", 6)
              if args.taught_tcp else None)
    legacy = (_parse_values(
        args.legacy_camera_to_tool, "--legacy-camera-to-tool", 6)
              if args.legacy_camera_to_tool else None)

    migrated = migrate(
        T_reference, T_handeye, correction,
        taught_tcp=taught, legacy_camera_to_tool=legacy)
    offset = migrated["standard_to_tool_mm_rpy_deg"]
    print("标准TCP： " + _fmt(migrated[
        "standard_tcp_mm_rpy_deg"], 6))
    print("旧最终TCP：" + _fmt(migrated[
        "legacy_final_tcp_mm_rpy_deg"], 6))
    print("迁移来源： " + migrated["legacy_final_source"])
    print("\n" + _yaml_block(
        args.class_name, args.tool_id, offset[:3], offset[3:]))
    print("位置复现残差："
          f"{migrated['position_residual_mm']:.9f} mm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
