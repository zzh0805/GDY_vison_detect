# -*- coding: utf-8 -*-
"""由YAML驱动的三模式、分步骤工具工作位标定。"""
from __future__ import annotations

import argparse
import re
import shutil
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping, Optional, Sequence

import numpy as np
import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from calibrate_tool_offset import _matrix_to_pose, _pose_to_matrix  # noqa: E402
from handeye_calib import handeye_math as hm  # noqa: E402


DEFAULT_CONFIG = Path(__file__).resolve().parent / "tool_calibration.yaml"
SUPPORTED_COLORS = ("auto", "red", "green", "black")
CLASS_NAME_PATTERN = re.compile(r"^[A-Za-z0-9_.-]+$")


class CalibrationWorkflowError(ValueError):
    """可直接展示给现场操作员的配置或流程错误。"""


def _mapping(value: Any, name: str) -> dict:
    if value is None:
        return {}
    if not isinstance(value, Mapping):
        raise CalibrationWorkflowError(f"{name}必须是YAML字典")
    return dict(value)


def _finite_pose(value: Any, name: str, count: int = 6) -> np.ndarray:
    if isinstance(value, str):
        value = [item.strip() for item in value.split(",")]
    try:
        pose = np.asarray(value, dtype=np.float64).reshape(-1)
    except (TypeError, ValueError) as exc:
        raise CalibrationWorkflowError(
            f"{name}必须是{count}个有限数字") from exc
    if pose.size != count or not np.isfinite(pose).all():
        raise CalibrationWorkflowError(f"{name}必须是{count}个有限数字")
    return pose


def _load_yaml(path: Path, *, missing_ok: bool = False) -> dict:
    if not path.is_file():
        if missing_ok:
            return {}
        raise CalibrationWorkflowError(f"YAML文件不存在：{path}")
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8-sig")) or {}
    except (UnicodeDecodeError, yaml.YAMLError) as exc:
        raise CalibrationWorkflowError(f"YAML格式错误：{path}: {exc}") from exc
    if not isinstance(data, Mapping):
        raise CalibrationWorkflowError(f"YAML顶层必须是字典：{path}")
    return dict(data)


def _write_yaml_atomic(path: Path, data: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(
        yaml.safe_dump(dict(data), allow_unicode=True, sort_keys=False),
        encoding="utf-8")
    temporary.replace(path)


def _resolve_from(config_path: Path, value: Any, name: str) -> Path:
    text = str(value or "").strip()
    if not text:
        raise CalibrationWorkflowError(f"{name}不能为空")
    path = Path(text).expanduser()
    if not path.is_absolute():
        path = config_path.parent / path
    return path.resolve()


def _mode_step(config: Mapping[str, Any]) -> tuple[int, int]:
    operation = _mapping(config.get("operation"), "operation")
    raw_mode = operation.get("mode", operation.get("model"))
    try:
        mode = int(raw_mode)
        step = int(operation.get("step"))
    except (TypeError, ValueError) as exc:
        raise CalibrationWorkflowError(
            "operation.mode和operation.step必须是整数") from exc
    if mode not in (1, 2, 3):
        raise CalibrationWorkflowError("operation.mode只能填写1、2或3")
    allowed_steps = (1, 2, 3) if mode in (1, 2) else (1, 2)
    if step not in allowed_steps:
        raise CalibrationWorkflowError(
            f"模式{mode}的step只能填写{allowed_steps}")
    return mode, step


def _workpiece(config: Mapping[str, Any]) -> dict:
    value = _mapping(config.get("workpiece"), "workpiece")
    class_name = str(value.get("class_name") or "").strip()
    if not CLASS_NAME_PATTERN.fullmatch(class_name):
        raise CalibrationWorkflowError(
            "workpiece.class_name只能包含字母、数字、点、横线或下划线")
    tool_id = str(value.get("tool_id") or f"tool_{class_name}").strip()
    if not tool_id:
        raise CalibrationWorkflowError("workpiece.tool_id不能为空")
    code = str(value.get("code") or "").strip()
    color = str(value.get("target_color") or "auto").strip().lower()
    if color not in SUPPORTED_COLORS:
        raise CalibrationWorkflowError(
            "workpiece.target_color只能是auto、red、green或black")
    return {
        "class_name": class_name,
        "tool_id": tool_id,
        "code": code,
        "target_color": color,
    }


def _record_current_tcp(config: Mapping[str, Any],
                        pose_override: Optional[Sequence[float]]) -> np.ndarray:
    if pose_override is not None:
        return _finite_pose(pose_override, "--pose")
    robot_cfg = _mapping(config.get("robot"), "robot")
    from handeye_calib.jaka_adapter import (
        JAKARobotAdapter, unified_pose_to_jaka)

    robot = JAKARobotAdapter(
        endpoint=str(robot_cfg.get("ip", "192.168.16.158")),
        enable_motion=False,
        speed=0.1,
        accel=0.3,
        settle=0.0,
        timeout=float(robot_cfg.get("timeout_s", 30.0)),
        sdk_path=(str(robot_cfg.get("sdk_path") or "").strip() or None),
    )
    try:
        robot.connect()
        pose = unified_pose_to_jaka(robot.get_tcp_pose())
        pose[3:] = np.degrees(pose[3:])
        return _finite_pose(pose, "JAKA当前TCP")
    finally:
        robot.disconnect()


def _current_offset(tool_offsets_path: Path,
                    class_name: str) -> tuple[np.ndarray, dict]:
    data = _load_yaml(tool_offsets_path)
    tools = _mapping(data.get("tools"), "tool_offsets.tools")
    if class_name not in tools:
        raise CalibrationWorkflowError(
            f"{tool_offsets_path}中不存在工件{class_name!r}")
    tool = _mapping(tools[class_name], f"tools.{class_name}")
    transform = _mapping(
        tool.get("standard_to_tool"),
        f"tools.{class_name}.standard_to_tool")
    offset = np.concatenate([
        _finite_pose(transform.get("xyz_mm"), "xyz_mm", 3),
        _finite_pose(transform.get("rpy_deg"), "rpy_deg", 3),
    ])
    return offset, tool


def calculate_from_zero(standard_tcp: Sequence[float],
                        taught_tcp: Sequence[float]) -> np.ndarray:
    """模式1：由50mm标准TCP与最终示教TCP反求局部工具偏移。"""
    T_standard = _pose_to_matrix(_finite_pose(standard_tcp, "标准TCP"))
    T_taught = _pose_to_matrix(_finite_pose(taught_tcp, "示教TCP"))
    return _matrix_to_pose(np.linalg.inv(T_standard) @ T_taught)


def calculate_correction(current_offset: Sequence[float],
                         old_work_tcp: Sequence[float],
                         new_work_tcp: Sequence[float]) -> np.ndarray:
    """模式2：用旧/新工作TCP对已有工具偏移做刚体纠正。"""
    T_offset = _pose_to_matrix(_finite_pose(current_offset, "当前工具偏移"))
    T_old = _pose_to_matrix(_finite_pose(old_work_tcp, "旧工作TCP"))
    T_new = _pose_to_matrix(_finite_pose(new_work_tcp, "新工作TCP"))
    return _matrix_to_pose(T_offset @ np.linalg.inv(T_old) @ T_new)


def calculate_adjustment(current_offset: Sequence[float],
                         current_work_tcp: Sequence[float],
                         adjust_xyz_mm: Sequence[float],
                         adjust_rpy_deg: Sequence[float]) -> tuple[np.ndarray, np.ndarray]:
    """模式3：在JAKA基座系对当前工作TCP做分量增量并更新偏移。"""
    old_pose = _finite_pose(current_work_tcp, "当前工作TCP")
    delta = np.concatenate([
        _finite_pose(adjust_xyz_mm, "adjustment.xyz_mm", 3),
        _finite_pose(adjust_rpy_deg, "adjustment.rpy_deg", 3),
    ])
    new_pose = old_pose + delta
    return calculate_correction(current_offset, old_pose, new_pose), new_pose


def _result_entry(workpiece: Mapping[str, Any], offset: np.ndarray) -> dict:
    entry: dict[str, Any] = {}
    if workpiece["code"]:
        entry["code"] = workpiece["code"]
    entry["target_color"] = workpiece["target_color"]
    entry["tool_id"] = workpiece["tool_id"]
    entry["enabled"] = True
    entry["standard_to_tool"] = {
        "xyz_mm": [round(float(value), 6) for value in offset[:3]],
        "rpy_deg": [round(float(value), 6) for value in offset[3:]],
    }
    return entry


def _apply_result(tool_offsets_path: Path, workpiece: Mapping[str, Any],
                  entry: Mapping[str, Any], backup_dir: Path) -> Path:
    data = _load_yaml(tool_offsets_path)
    tools = _mapping(data.get("tools"), "tool_offsets.tools")
    existing = _mapping(
        tools.get(workpiece["class_name"]),
        f"tools.{workpiece['class_name']}")
    # 已有工件保留其code、颜色、tool_id和enabled，只更新标定结果；
    # 新工件则使用workflow.yaml中的完整信息创建。
    if existing:
        existing["standard_to_tool"] = dict(entry["standard_to_tool"])
        tools[workpiece["class_name"]] = existing
    else:
        tools[workpiece["class_name"]] = dict(entry)
    data["tools"] = tools
    backup_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    backup = backup_dir / f"tool_offsets-{stamp}.yaml"
    shutil.copy2(tool_offsets_path, backup)
    try:
        _write_yaml_atomic(tool_offsets_path, data)
        # 用正式加载器做同级严格校验；失败时立即恢复备份。
        from vision_solver.config import load_tool_offsets
        load_tool_offsets(tool_offsets_path)
    except Exception:
        shutil.copy2(backup, tool_offsets_path)
        raise
    return backup


def _verify_result(mode: int, records: Mapping[str, Any],
                   old_offset: Optional[np.ndarray],
                   new_offset: np.ndarray,
                   expected_pose: np.ndarray) -> tuple[float, float]:
    if mode == 1:
        T_standard = _pose_to_matrix(records["standard_tcp_mm_rpy_deg"])
    else:
        if old_offset is None:
            raise RuntimeError("缺少原工具偏移")
        old_key = ("old_work_tcp_mm_rpy_deg" if mode == 2
                   else "current_work_tcp_mm_rpy_deg")
        T_old = _pose_to_matrix(records[old_key])
        T_standard = T_old @ np.linalg.inv(_pose_to_matrix(old_offset))
    predicted = T_standard @ _pose_to_matrix(new_offset)
    expected = _pose_to_matrix(expected_pose)
    position_error = float(np.linalg.norm(
        predicted[:3, 3] - expected[:3, 3]) * 1000.0)
    rotation_error = float(hm.angle_deg(
        predicted[:3, :3], expected[:3, :3]))
    return position_error, rotation_error


def execute_workflow(config_path: Path,
                     pose_override: Optional[Sequence[float]] = None) -> dict:
    config_path = Path(config_path).expanduser().resolve()
    config = _load_yaml(config_path)
    if int(config.get("version", 1)) != 1:
        raise CalibrationWorkflowError("tool_calibration.yaml的version只支持1")
    mode, step = _mode_step(config)
    workpiece = _workpiece(config)
    files = _mapping(config.get("files"), "files")
    state_path = _resolve_from(
        config_path, files.get("record_file", "./tool_calibration_record.yaml"),
        "files.record_file")
    offsets_path = _resolve_from(
        config_path, files.get("tool_offsets_file", "../config/tool_offsets.yaml"),
        "files.tool_offsets_file")
    state = _load_yaml(state_path, missing_ok=True)
    state.setdefault("version", 1)
    sessions = _mapping(state.get("sessions"), "record.sessions")
    class_sessions = _mapping(
        sessions.get(workpiece["class_name"]),
        f"record.sessions.{workpiece['class_name']}")
    session_key = f"mode_{mode}"
    records = _mapping(class_sessions.get(session_key), session_key)

    record_keys = {
        (1, 1): "standard_tcp_mm_rpy_deg",
        (1, 2): "taught_tcp_mm_rpy_deg",
        (2, 1): "old_work_tcp_mm_rpy_deg",
        (2, 2): "new_work_tcp_mm_rpy_deg",
        (3, 1): "current_work_tcp_mm_rpy_deg",
    }
    record_key = record_keys.get((mode, step))
    now = datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")
    if record_key is not None:
        if step == 1:
            stale_fields = {
                1: ("taught_tcp_mm_rpy_deg",
                    "taught_tcp_mm_rpy_deg_recorded_at"),
                2: ("new_work_tcp_mm_rpy_deg",
                    "new_work_tcp_mm_rpy_deg_recorded_at"),
                3: (),
            }
            for field in (*stale_fields[mode], "last_result"):
                records.pop(field, None)
        if mode in (2, 3) and step == 1:
            source_offset, _source_tool = _current_offset(
                offsets_path, workpiece["class_name"])
            records["source_standard_to_tool_mm_rpy_deg"] = [
                round(float(value), 9) for value in source_offset]
        pose = _record_current_tcp(config, pose_override)
        records[record_key] = [round(float(value), 9) for value in pose]
        records[f"{record_key}_recorded_at"] = now
        class_sessions[session_key] = records
        sessions[workpiece["class_name"]] = class_sessions
        state["sessions"] = sessions
        state["last_action"] = {
            "mode": mode, "step": step,
            "class_name": workpiece["class_name"],
            "recorded_field": record_key,
            "recorded_tcp_mm_rpy_deg": records[record_key],
            "at": now,
        }
        _write_yaml_atomic(state_path, state)
        return {
            "action": "recorded", "mode": mode, "step": step,
            "field": record_key, "pose": pose, "record_file": state_path,
        }

    old_offset: Optional[np.ndarray] = None
    if mode == 1:
        required = ("standard_tcp_mm_rpy_deg", "taught_tcp_mm_rpy_deg")
        missing = [name for name in required if name not in records]
        if missing:
            raise CalibrationWorkflowError(
                f"模式1缺少记录{missing}，请先依次执行step 1和step 2")
        new_offset = calculate_from_zero(
            records[required[0]], records[required[1]])
        expected_pose = _finite_pose(records[required[1]], "示教TCP")
    elif mode == 2:
        required = ("old_work_tcp_mm_rpy_deg", "new_work_tcp_mm_rpy_deg")
        missing = [name for name in required if name not in records]
        if missing:
            raise CalibrationWorkflowError(
                f"模式2缺少记录{missing}，请先依次执行step 1和step 2")
        if "source_standard_to_tool_mm_rpy_deg" not in records:
            raise CalibrationWorkflowError(
                "模式2缺少step 1记录的原工具偏移，请重新执行step 1")
        old_offset = _finite_pose(
            records["source_standard_to_tool_mm_rpy_deg"], "原工具偏移")
        new_offset = calculate_correction(
            old_offset, records[required[0]], records[required[1]])
        expected_pose = _finite_pose(records[required[1]], "新工作TCP")
    else:
        if "current_work_tcp_mm_rpy_deg" not in records:
            raise CalibrationWorkflowError("模式3缺少step 1记录的当前工作TCP")
        if "source_standard_to_tool_mm_rpy_deg" not in records:
            raise CalibrationWorkflowError(
                "模式3缺少step 1记录的原工具偏移，请重新执行step 1")
        old_offset = _finite_pose(
            records["source_standard_to_tool_mm_rpy_deg"], "原工具偏移")
        adjustment = _mapping(config.get("adjustment"), "adjustment")
        new_offset, expected_pose = calculate_adjustment(
            old_offset,
            records["current_work_tcp_mm_rpy_deg"],
            adjustment.get("xyz_mm", [0, 0, 0]),
            adjustment.get("rpy_deg", [0, 0, 0]),
        )

    position_error, rotation_error = _verify_result(
        mode, records, old_offset, new_offset, expected_pose)
    entry = _result_entry(workpiece, new_offset)
    result = {
        "mode": mode,
        "class_name": workpiece["class_name"],
        "calculated_at": now,
        "standard_to_tool": entry["standard_to_tool"],
        "expected_work_tcp_mm_rpy_deg": [
            round(float(value), 9) for value in expected_pose],
        "verification": {
            "position_error_mm": position_error,
            "rotation_error_deg": rotation_error,
        },
        "ready_to_copy": {
            workpiece["class_name"]: entry,
        },
        "applied_to_tool_offsets": False,
    }
    output = _mapping(config.get("output"), "output")
    apply_to_offsets = output.get("apply_to_tool_offsets", False)
    if not isinstance(apply_to_offsets, bool):
        raise CalibrationWorkflowError(
            "output.apply_to_tool_offsets必须是true或false")
    if apply_to_offsets:
        backup_dir = _resolve_from(
            config_path,
            output.get("backup_directory", "./backups"),
            "output.backup_directory")
        backup = _apply_result(offsets_path, workpiece, entry, backup_dir)
        result["applied_to_tool_offsets"] = True
        result["tool_offsets_file"] = str(offsets_path)
        result["backup_file"] = str(backup)

    records["last_result"] = result
    class_sessions[session_key] = records
    sessions[workpiece["class_name"]] = class_sessions
    state["sessions"] = sessions
    state["last_result"] = result
    state["last_action"] = {
        "mode": mode, "step": step,
        "class_name": workpiece["class_name"],
        "action": "calculated", "at": now,
    }
    _write_yaml_atomic(state_path, state)
    return {
        "action": "calculated", "mode": mode, "step": step,
        "result": result, "record_file": state_path,
    }


def _fmt(values: Sequence[float], digits: int = 6) -> str:
    return ", ".join(f"{float(value):.{digits}f}" for value in values)


def _print_result(outcome: Mapping[str, Any]) -> None:
    print(f"模式={outcome['mode']} step={outcome['step']}")
    if outcome["action"] == "recorded":
        print(f"已记录：{outcome['field']}")
        print(f"TCP [mm + JAKA RPY deg]: [{_fmt(outcome['pose'])}]")
        print(f"记录文件：{outcome['record_file']}")
        return
    result = outcome["result"]
    offset = result["standard_to_tool"]
    print("\n==== standard_to_tool ====")
    print(f"xyz_mm:  [{_fmt(offset['xyz_mm'])}]")
    print(f"rpy_deg: [{_fmt(offset['rpy_deg'])}]")
    print("\n==== 可粘贴到config/tool_offsets.yaml的tools下 ====")
    print(yaml.safe_dump(
        result["ready_to_copy"], allow_unicode=True,
        sort_keys=False).rstrip())
    verify = result["verification"]
    print("\n==== 正向校验 ====")
    print(f"位置残差: {verify['position_error_mm']:.9f} mm")
    print(f"旋转残差: {verify['rotation_error_deg']:.9f} 度")
    print(f"记录文件：{outcome['record_file']}")
    if result["applied_to_tool_offsets"]:
        print(f"已更新：{result['tool_offsets_file']}")
        print(f"更新前备份：{result['backup_file']}")
        print("视觉服务将在下一次解算时热加载新偏移，无需重启。")
    else:
        print("未自动修改正式工具配置；确认后复制上述YAML，或将")
        print("output.apply_to_tool_offsets改为true后重新执行计算步骤。")


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="YAML分步骤工具标定")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--pose",
        help="离线调试时覆盖JAKA当前TCP，6个逗号分隔数字；现场不要填写")
    args = parser.parse_args(argv)
    pose_override = (
        _finite_pose(args.pose, "--pose") if args.pose is not None else None)
    try:
        outcome = execute_workflow(args.config, pose_override)
    except (CalibrationWorkflowError, FileNotFoundError, KeyError,
            RuntimeError, ValueError) as exc:
        print(f"[错误] {exc}", file=sys.stderr)
        return 2
    _print_result(outcome)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
