# -*- coding: utf-8 -*-
from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any, Mapping, Tuple
from urllib.parse import unquote, urlsplit

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


def snapshot_directory(case_path: Path, test_data: Mapping[str, Any]) -> Path:
    """定位视觉服务保存快照的本地目录（要求脚本与视觉服务同机）。

    目录取自 test_case.yaml 的 application_config 指向的 workflow.yaml
    中 http.snapshot_directory，相对该 workflow.yaml 所在目录解析。
    """
    raw_config = test_data.get("application_config")
    if not raw_config:
        raise ValueError("test_case.yaml缺少application_config路径")
    config_path = resolve_from(case_path, raw_config)
    if not config_path.is_file():
        raise FileNotFoundError(f"应用配置不存在: {config_path}")
    content = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    http_config = content.get("http") or {}
    directory = Path(
        str(http_config.get("snapshot_directory") or "../shared_images")
    ).expanduser()
    if not directory.is_absolute():
        directory = config_path.parent / directory
    return directory.resolve()


def resolve_local_snapshot(
        image_ref: Any, case_path: Path,
        test_data: Mapping[str, Any]) -> Path:
    """/snapshot返回的path解析为本地快照文件（只读本地，不下载URL）。

    path是HTTP(S) URL时，按URL最后一段在snapshot_directory下找同名文件；
    path本身是本地路径时直接使用。文件缺失时提示检查删除开关。
    """
    value = str(image_ref or "").strip()
    if not value:
        raise ValueError("/snapshot响应缺少path")
    parsed = urlsplit(value)
    if parsed.scheme in ("http", "https"):
        file_name = Path(unquote(parsed.path)).name
        if not file_name:
            raise ValueError(f"无法从path解析快照文件名: {value}")
        local_path = snapshot_directory(case_path, test_data) / file_name
    else:
        local_path = Path(value).expanduser().resolve()
    if not local_path.is_file():
        raise FileNotFoundError(
            f"本地快照不存在: {local_path}\n"
            "该脚本要求与视觉服务同机运行；MinIO发布模式下若"
            "snapshot_minio.delete_local_after_upload=true，"
            "JPG会在上传后立即删除，请改为false并重启视觉服务。")
    return local_path


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


def directional_approach_pose_mm_rpy_deg(
        target_pose_mm_rpy_deg: Any,
        approach_direction_base: Any,
        approach_distance_mm: float) -> np.ndarray:
    """沿当次柜体法向的反方向生成预备TCP。

    approach_direction_base是在JAKA基座系中从相机/预备位指向柜体的
    单位方向。预备位与最终位姿态完全相同，只把位置沿该方向反退指定距离。
    """
    target = np.asarray(
        target_pose_mm_rpy_deg, dtype=np.float64).reshape(6)
    direction = np.asarray(
        approach_direction_base, dtype=np.float64).reshape(3)
    distance = float(approach_distance_mm)
    norm = float(np.linalg.norm(direction))
    if not np.isfinite(target).all():
        raise ValueError("工作TCP必须是6个有限数字")
    if not np.isfinite(direction).all() or not np.isfinite(norm) or norm < 1e-9:
        raise ValueError("进入方向必须是3个有限数字且长度不能为0")
    if not np.isfinite(distance) or distance < 0.0:
        raise ValueError("预备距离必须是非负有限数字")
    direction /= norm
    approach = target.copy()
    approach[:3] -= distance * direction
    return approach


def resolve_return_to_capture_pose(
        jaka_config: Mapping[str, Any],
        command_line_override: Any = None) -> bool:
    """解析到达工作位后是否执行完整退出并返回拍照位。"""
    if command_line_override is not None:
        if not isinstance(command_line_override, bool):
            raise ValueError("命令行返回拍照位开关必须是布尔值")
        return command_line_override
    value = jaka_config.get("return_to_capture_pose", True)
    if not isinstance(value, bool):
        raise ValueError(
            "jaka_test.return_to_capture_pose必须填写true或false")
    return value


def resolve_live_capture(
        case_config: Mapping[str, Any],
        command_line_override: Any = None) -> bool:
    """解析现场测试是否实时采集；命令行显式值优先于YAML。"""
    value = (command_line_override if command_line_override is not None
             else case_config.get("live", False))
    if not isinstance(value, bool):
        raise ValueError("case.live必须填写true或false")
    return value


def resolve_base_label(
        case_config: Mapping[str, Any],
        command_line_override: Any = None) -> Any:
    """解析安装面板标签；空值或旧式0表示不传base矩形。"""
    value = (command_line_override if command_line_override is not None
             else case_config.get("base_label"))
    if value is None:
        return None
    if isinstance(value, bool):
        raise ValueError("case.base_label必须是LabelMe标签或null")
    label = str(value).strip()
    if not label or label == "0":
        return None
    return label


def field_test_motion_waypoints(
        approach_pose_mm_rpy_deg: Any,
        work_pose_mm_rpy_deg: Any,
        capture_pose_mm_rpy_deg: Any,
        return_to_capture_pose: bool
        ) -> Tuple[Tuple[str, np.ndarray], ...]:
    """生成现场测试的实际运动序列，便于在不连接JAKA时完整验证。"""
    if not isinstance(return_to_capture_pose, bool):
        raise ValueError("return_to_capture_pose必须是布尔值")
    approach = np.asarray(
        approach_pose_mm_rpy_deg, dtype=np.float64).reshape(6)
    work = np.asarray(work_pose_mm_rpy_deg, dtype=np.float64).reshape(6)
    capture = np.asarray(
        capture_pose_mm_rpy_deg, dtype=np.float64).reshape(6)
    if not all(np.isfinite(item).all() for item in (
            approach, work, capture)):
        raise ValueError("现场运动路径必须全部为有限TCP")
    waypoints = [
        ("field_test_work_approach_pose", approach.copy()),
        ("field_test_work_pose", work.copy()),
    ]
    if return_to_capture_pose:
        waypoints.extend([
            ("field_test_work_retreat_pose", approach.copy()),
            ("field_test_return_capture_pose", capture.copy()),
        ])
    return tuple(waypoints)


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
