# -*- coding: utf-8 -*-
from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any, Mapping, Tuple

import numpy as np
import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


def load_test_case(path: Any) -> Tuple[Path, dict]:
    source = Path(path).expanduser().resolve()
    data = yaml.safe_load(source.read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        raise ValueError("测试配置必须是字典")
    return source, data


def save_test_case(path: Path, data: Mapping[str, Any]) -> None:
    path.write_text(
        yaml.safe_dump(dict(data), allow_unicode=True, sort_keys=False),
        encoding="utf-8")


def resolve_from(path: Path, value: Any) -> Path:
    result = Path(str(value)).expanduser()
    if not result.is_absolute():
        result = path.parent / result
    return result.resolve()


def create_http_client(test_data: Mapping[str, Any]):
    from vision_solver.http_client import VisionHttpClient
    cfg = dict(test_data.get("http_test") or {})
    return VisionHttpClient(
        str(cfg.get("service_url", "http://127.0.0.1:48051")),
        timeout_s=float(cfg.get("timeout_s", 15.0)))


def create_jaka(test_data: Mapping[str, Any], enable_motion: bool):
    from handeye_calib.jaka_adapter import JAKARobotAdapter
    cfg = dict(test_data.get("jaka_test") or {})
    return JAKARobotAdapter(
        endpoint=str(cfg.get("ip", "192.168.16.158")),
        enable_motion=bool(enable_motion),
        speed=float(cfg.get("move_speed_mm_s", 300.0)) * 0.001,
        accel=float(cfg.get("move_accel_mm_s2", 800.0)) * 0.001,
        settle=float(cfg.get("settle_s", 0.5)),
        timeout=float(cfg.get("timeout_s", 30.0)),
        sdk_path=(str(cfg.get("sdk_path") or "").strip() or None),
    )


def read_jaka_pose_mm_rpy_deg(robot) -> np.ndarray:
    from handeye_calib.jaka_adapter import unified_pose_to_jaka
    pose = unified_pose_to_jaka(robot.get_tcp_pose())
    pose[3:] = np.degrees(pose[3:])
    return pose


def jaka_deg_to_unified(pose_mm_rpy_deg: Any) -> np.ndarray:
    from handeye_calib.jaka_adapter import jaka_pose_to_unified
    pose = np.asarray(pose_mm_rpy_deg, dtype=np.float64).reshape(6)
    pose_rad = pose.copy()
    pose_rad[3:] = np.radians(pose_rad[3:])
    return jaka_pose_to_unified(pose_rad)


def jaka_rad_to_unified(pose_mm_rpy_rad: Any) -> np.ndarray:
    from handeye_calib.jaka_adapter import jaka_pose_to_unified
    return jaka_pose_to_unified(
        np.asarray(pose_mm_rpy_rad, dtype=np.float64).reshape(6))


def base_x_approach_pose_mm_rpy_deg(
        target_pose_mm_rpy_deg: Any,
        approach_offset_base_x_mm: float) -> np.ndarray:
    """生成只与最终工作位相差基座X的预备位。

    偏移为负表示预备位在最终位的基座-X侧，随后沿基座+X进入；
    偏移为正则方向相反。姿态及Y/Z始终与最终工作位完全一致。
    """
    target = np.asarray(
        target_pose_mm_rpy_deg, dtype=np.float64).reshape(6)
    offset = float(approach_offset_base_x_mm)
    if not np.isfinite(target).all() or not np.isfinite(offset):
        raise ValueError("工作TCP和基座X预备位偏移必须是有限数字")
    approach = target.copy()
    approach[0] += offset
    return approach


def read_labelme_corners(path: Path, label: str,
                         shape_index: int = 0) -> np.ndarray:
    # 兼容 Windows 工具保存的 UTF-16/UTF-8 BOM 标注文件：
    # UTF-16 带 BOM（\xff\xfe / \xfe\xff），UTF-8 带 BOM（\xef\xbb\xbf）。
    raw = path.read_bytes()
    if raw.startswith((b"\xff\xfe", b"\xfe\xff")):
        text = raw.decode("utf-16")
    elif raw.startswith(b"\xef\xbb\xbf"):
        text = raw.decode("utf-8-sig")
    else:
        text = raw.decode("utf-8")
    data = json.loads(text)
    shapes = [shape for shape in data.get("shapes", [])
              if str(shape.get("label")) == str(label)]
    if not shapes:
        raise ValueError(f"LabelMe中没有标签{label!r}")
    if shape_index < 0 or shape_index >= len(shapes):
        raise IndexError(
            f"标签{label!r}只有{len(shapes)}个，无法选择索引{shape_index}")
    points = np.asarray(shapes[shape_index].get("points"), dtype=np.float64)
    if points.shape == (4, 2):
        return points
    if points.shape == (2, 2):
        x1, y1 = np.minimum(points[0], points[1])
        x2, y2 = np.maximum(points[0], points[1])
        return np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]])
    raise ValueError(
        f"LabelMe目标必须是4点polygon或2点rectangle，实际{points.shape}")
