# -*- coding: utf-8 -*-
"""GDY视觉v3工具工作位标定。

v2先统一得到“逆手眼 + 全局修正”的标准活动TCP，再标定工具相对于该
标准TCP的固定局部变换：

    T_standard_tool = inv(T_base_tcp_standard) @ T_base_tcp_taught
    T_base_tcp_target = T_base_tcp_standard @ T_standard_tool

输入/输出位姿均为 mm + JAKA RPY(度)，4x4矩阵平移单位为m。
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

from handeye_calib import handeye_math as hm
from handeye_calib.jaka_adapter import (
    jaka_rpy_to_matrix, matrix_to_jaka_rpy)


def _pose_to_matrix(pose: Sequence[float]) -> np.ndarray:
    values = np.asarray(pose, dtype=np.float64).reshape(6)
    if not np.isfinite(values).all():
        raise ValueError("位姿包含NaN/Inf")
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = jaka_rpy_to_matrix(np.radians(values[3:]))
    result[:3, 3] = values[:3] * 0.001
    return result


def _matrix_to_pose(transform: np.ndarray) -> np.ndarray:
    matrix = _validate_transform(transform, "transform")
    return np.concatenate([
        matrix[:3, 3] * 1000.0,
        np.degrees(matrix_to_jaka_rpy(matrix[:3, :3])),
    ])


def _validate_transform(transform: np.ndarray, name: str) -> np.ndarray:
    matrix = np.asarray(transform, dtype=np.float64)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError(f"{name}必须是有限的4x4矩阵")
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9):
        raise ValueError(f"{name}最后一行必须是[0,0,0,1]")
    orthogonal_error = float(np.linalg.norm(
        matrix[:3, :3].T @ matrix[:3, :3] - np.eye(3)))
    if orthogonal_error > 1e-5:
        raise ValueError(
            f"{name}旋转部分不是正交矩阵（误差={orthogonal_error:.3g}）")
    return matrix


def _parse_values(text: str, name: str, count: int) -> np.ndarray:
    try:
        values = np.asarray(
            [float(item) for item in str(text).replace(" ", "").split(",")],
            dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name}必须是{count}个逗号分隔数字") from exc
    if values.size != count or not np.isfinite(values).all():
        raise ValueError(f"{name}必须是{count}个有限数字")
    return values


def _load_standard_reference(result_json: Path) -> np.ndarray:
    data = json.loads(result_json.read_text(encoding="utf-8"))
    camera_reference = data.get("cameraReference") or {}
    value = camera_reference.get("TBaseTcpStandard")
    if value is None:
        raise KeyError(
            f"{result_json}没有cameraReference.TBaseTcpStandard；"
            "该文件可能来自v1，请使用v3解算结果")
    return _validate_transform(np.asarray(value, dtype=np.float64),
                               "TBaseTcpStandard")


def _fmt(values: np.ndarray, digits: int = 4) -> str:
    return ", ".join(f"{float(value):.{digits}f}" for value in values)


def _yaml_block(class_name: str, tool_id: str,
                xyz_mm: np.ndarray, rpy_deg: np.ndarray) -> str:
    return (
        f"  {class_name}:\n"
        f"    tool_id: {tool_id}\n"
        f"    enabled: true\n"
        f"    standard_to_tool:\n"
        f"      xyz_mm: [{_fmt(xyz_mm)}]\n"
        f"      rpy_deg: [{_fmt(rpy_deg)}]\n")


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="基于v2标准TCP反推或验证standard_to_tool")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument(
        "--result-json", type=Path,
        help="v2 /get_tcp_pose保存的result.json")
    source.add_argument(
        "--standard-reference",
        help="直接输入标准TCP位姿：mm+RPY度，6个逗号分隔数字")
    parser.add_argument(
        "--taught-tcp",
        help="工具人工示教到最终工作位的活动TCP：mm+RPY度")
    parser.add_argument(
        "--standard-to-tool",
        help="已有standard_to_tool：xyz_mm+rpy_deg，共6个数字")
    parser.add_argument(
        "--adjust-xyz-mm",
        help="对当前预测最终TCP施加的基座系位置微调，3个数字")
    parser.add_argument(
        "--adjust-rpy-deg",
        help="对当前预测最终TCP施加的基座系RPY微调，3个数字")
    parser.add_argument("--class-name", default="<new_class>")
    parser.add_argument("--tool-id", default="tool_<new_class>")
    args = parser.parse_args(argv)

    if args.result_json is not None:
        if not args.result_json.is_file():
            print(f"错误：result.json不存在：{args.result_json}",
                  file=sys.stderr)
            return 2
        T_standard = _load_standard_reference(args.result_json)
        print(f"标准TCP：从{args.result_json}读取TBaseTcpStandard")
    else:
        T_standard = _pose_to_matrix(_parse_values(
            args.standard_reference, "--standard-reference", 6))
        print("标准TCP：由--standard-reference转换")
    print("  " + _fmt(_matrix_to_pose(T_standard), 6))

    if args.standard_to_tool is not None:
        offset_pose = _parse_values(
            args.standard_to_tool, "--standard-to-tool", 6)
        T_offset = _pose_to_matrix(offset_pose)
        predicted_pose = _matrix_to_pose(T_standard @ T_offset)
        if args.adjust_xyz_mm is None and args.adjust_rpy_deg is None:
            print("\n[验证] 当前standard_to_tool对应最终TCP：")
            print("  " + _fmt(predicted_pose, 6))
            return 0

        adjust_xyz = (_parse_values(
            args.adjust_xyz_mm, "--adjust-xyz-mm", 3)
            if args.adjust_xyz_mm else np.zeros(3))
        adjust_rpy = (_parse_values(
            args.adjust_rpy_deg, "--adjust-rpy-deg", 3)
            if args.adjust_rpy_deg else np.zeros(3))
        taught_pose = predicted_pose + np.concatenate([adjust_xyz, adjust_rpy])
    else:
        if args.taught_tcp is None:
            print("错误：请提供--taught-tcp，或提供--standard-to-tool进行验证",
                  file=sys.stderr)
            return 2
        if args.adjust_xyz_mm is not None or args.adjust_rpy_deg is not None:
            print("错误：微调参数必须与--standard-to-tool一起使用",
                  file=sys.stderr)
            return 2
        taught_pose = _parse_values(args.taught_tcp, "--taught-tcp", 6)

    T_taught = _pose_to_matrix(taught_pose)
    T_offset = _validate_transform(
        np.linalg.inv(T_standard) @ T_taught, "T_standard_tool")
    offset_pose = _matrix_to_pose(T_offset)
    predicted = T_standard @ T_offset
    position_error_mm = float(np.linalg.norm(
        predicted[:3, 3] - T_taught[:3, 3]) * 1000.0)
    rotation_error_deg = hm.angle_deg(
        predicted[:3, :3], T_taught[:3, :3])

    print("\n==== v3工具偏移 ====")
    print(f"  xyz_mm:  [{_fmt(offset_pose[:3])}]")
    print(f"  rpy_deg: [{_fmt(offset_pose[3:])}]")
    print("\n==== 可粘贴到config/tool_offsets.yaml的tools下 ====")
    print(_yaml_block(
        args.class_name, args.tool_id, offset_pose[:3], offset_pose[3:]))
    print("==== 正向校验 ====")
    print(f"  位置残差: {position_error_mm:.9f} mm")
    print(f"  旋转残差: {rotation_error_deg:.9f} 度")
    return 0


if __name__ == "__main__":
    sys.exit(main())
