# -*- coding: utf-8 -*-
"""一次启动即可完成一个工件标定的现场交互入口。"""
from __future__ import annotations

import argparse
import re
import shutil
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Mapping, Optional, Sequence

import numpy as np
import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from calibrate_tool_offset import _matrix_to_pose, _pose_to_matrix  # noqa: E402
from handeye_calib import handeye_math as hm  # noqa: E402
from tool.tool_calibration_workflow import (  # noqa: E402
    CalibrationWorkflowError, _finite_pose, _load_yaml, _mapping,
    _resolve_from, _write_yaml_atomic, calculate_adjustment,
    calculate_correction, calculate_from_zero)
from vision_solver.config import load_tool_offsets  # noqa: E402


DEFAULT_CONFIG = Path(__file__).resolve().parent / (
    "interactive_tool_calibration.yaml")
InputFunction = Callable[[str], str]
PrintFunction = Callable[[str], None]
PoseReader = Callable[[], Sequence[float]]


class InteractiveCalibrationCancelled(RuntimeError):
    """操作员主动取消本次标定。"""


class JakaPoseReader:
    """懒连接且在整个交互会话内复用同一个JAKA只读连接。"""

    def __init__(self, robot_config: Mapping[str, Any]):
        self.config = dict(robot_config)
        self.robot = None

    def __call__(self) -> np.ndarray:
        if self.robot is None:
            from handeye_calib.jaka_adapter import (
                JAKARobotAdapter, unified_pose_to_jaka)
            self._unified_pose_to_jaka = unified_pose_to_jaka
            self.robot = JAKARobotAdapter(
                endpoint=str(self.config.get("ip", "192.168.16.158")),
                enable_motion=False,
                speed=0.1,
                accel=0.3,
                settle=0.0,
                timeout=float(self.config.get("timeout_s", 30.0)),
                sdk_path=(
                    str(self.config.get("sdk_path") or "").strip() or None),
            )
            self.robot.connect()
        pose = self._unified_pose_to_jaka(self.robot.get_tcp_pose())
        pose[3:] = np.degrees(pose[3:])
        return _finite_pose(pose, "JAKA当前TCP")

    def close(self) -> None:
        if self.robot is not None:
            try:
                self.robot.disconnect()
            finally:
                self.robot = None


def calculate_reference_adjustment(
        current_offset: Sequence[float],
        current_work_tcp: Sequence[float],
        adjust_xyz_mm: Sequence[float],
        adjust_rpy_deg: Sequence[float]) -> tuple[np.ndarray, np.ndarray]:
    """在标准/相机参考系中微调，不随JAKA基座相对柜体朝向改变。"""
    offset = _finite_pose(current_offset, "当前工具偏移")
    current_work = _finite_pose(current_work_tcp, "当前工作TCP")
    xyz = _finite_pose(adjust_xyz_mm, "微调XYZ", 3)
    rpy = _finite_pose(adjust_rpy_deg, "微调RPY", 3)

    T_offset = _pose_to_matrix(offset)
    T_new_offset = T_offset.copy()
    # 平移直接在标准TCP坐标系表达；旋转也绕标准TCP坐标轴左乘。
    T_new_offset[:3, 3] += xyz / 1000.0
    T_delta_rotation = _pose_to_matrix(
        np.concatenate([np.zeros(3), rpy]))
    T_new_offset[:3, :3] = (
        T_delta_rotation[:3, :3] @ T_offset[:3, :3])

    T_current_work = _pose_to_matrix(current_work)
    T_standard = T_current_work @ np.linalg.inv(T_offset)
    T_expected_work = T_standard @ T_new_offset
    return _matrix_to_pose(T_new_offset), _matrix_to_pose(T_expected_work)


def _single_backup(source: Path, backup: Path) -> None:
    """覆盖式保存一份固定名称备份，不累积历史副本。"""
    backup.parent.mkdir(parents=True, exist_ok=True)
    temporary = backup.with_name(backup.name + ".tmp")
    try:
        shutil.copy2(source, temporary)
        temporary.replace(backup)
    finally:
        if temporary.exists():
            temporary.unlink()


def _ask_choice(input_fn: InputFunction, print_fn: PrintFunction,
                prompt: str, valid: Sequence[str], default: str = "") -> str:
    allowed = set(valid)
    while True:
        answer = input_fn(prompt).strip().lower()
        if not answer and default:
            return default
        if answer in allowed:
            return answer
        if answer in {"q", "quit", "exit", "取消"}:
            raise InteractiveCalibrationCancelled("操作员取消")
        print_fn(f"输入无效，可选值：{', '.join(valid)}；输入q取消。")


def _wait_to_read(input_fn: InputFunction, print_fn: PrintFunction,
                  message: str, pose_reader: PoseReader) -> np.ndarray:
    while True:
        answer = input_fn(
            f"\n{message}\n按Enter或输入yes读取当前机械臂位姿；输入q取消："
        ).strip().lower()
        if answer in {"", "y", "yes", "是", "确认"}:
            pose = _finite_pose(pose_reader(), "JAKA当前TCP")
            print_fn(
                "已读取 TCP [mm + JAKA RPY°]：\n  [" +
                ", ".join(f"{value:.6f}" for value in pose) + "]")
            retry = input_fn(
                "按Enter确认此位姿；输入r重新读取；输入q取消："
            ).strip().lower()
            if retry in {"", "y", "yes", "是", "确认"}:
                return pose
            if retry in {"r", "retry", "重读"}:
                continue
            if retry in {"q", "quit", "exit", "取消"}:
                raise InteractiveCalibrationCancelled("操作员取消")
            print_fn("未确认，重新读取当前位姿。")
            continue
        if answer in {"q", "quit", "exit", "取消"}:
            raise InteractiveCalibrationCancelled("操作员取消")
        print_fn("请输入yes、直接按Enter，或输入q取消。")


def _read_vector(input_fn: InputFunction, print_fn: PrintFunction,
                 prompt: str, *, default_zero: bool = False) -> np.ndarray:
    while True:
        raw = input_fn(prompt).strip()
        if not raw and default_zero:
            return np.zeros(3, dtype=np.float64)
        if raw.lower() in {"q", "quit", "exit", "取消"}:
            raise InteractiveCalibrationCancelled("操作员取消")
        parts = [item for item in re.split(r"[,，\s]+", raw) if item]
        try:
            return _finite_pose(parts, "输入", 3)
        except CalibrationWorkflowError:
            print_fn("请输入3个有限数字，例如：0,-1,0；输入q取消。")


def _select_workpiece(snapshot: Any, input_fn: InputFunction,
                      print_fn: PrintFunction) -> tuple[str, dict]:
    print_fn("\n可标定工件：")
    for class_name, tool in snapshot.tools.items():
        code = str(tool.get("code") or "-")
        state = "启用" if bool(tool.get("enabled", True)) else "禁用"
        print_fn(
            f"  code={code:<10} 工件={class_name:<20} "
            f"tool_id={tool.get('tool_id', '-')} [{state}]")
    while True:
        selected = input_fn(
            "\n请输入工件code（例如9-8-1，也可输入工件名称；q取消）："
        ).strip()
        if selected.lower() in {"q", "quit", "exit", "取消"}:
            raise InteractiveCalibrationCancelled("操作员取消")
        class_name = snapshot.code_to_tool.get(selected)
        if class_name is None and selected in snapshot.tools:
            class_name = selected
        if class_name is not None:
            return class_name, dict(snapshot.tools[class_name])
        print_fn("没有找到该code或工件名称，请重新输入。")


def _current_offset(tool: Mapping[str, Any]) -> np.ndarray:
    transform = _mapping(tool.get("standard_to_tool"), "standard_to_tool")
    return np.concatenate([
        _finite_pose(transform.get("xyz_mm"), "xyz_mm", 3),
        _finite_pose(transform.get("rpy_deg"), "rpy_deg", 3),
    ])


def _verification_error(T_predicted: np.ndarray,
                        T_expected: np.ndarray) -> tuple[float, float]:
    position_error = float(np.linalg.norm(
        T_predicted[:3, 3] - T_expected[:3, 3]) * 1000.0)
    rotation_error = float(hm.angle_deg(
        T_predicted[:3, :3], T_expected[:3, :3]))
    return position_error, rotation_error


def _apply_offset(offsets_path: Path, backup_path: Path,
                  class_name: str, new_offset: np.ndarray) -> None:
    data = _load_yaml(offsets_path)
    tools = _mapping(data.get("tools"), "tool_offsets.tools")
    if class_name not in tools:
        raise CalibrationWorkflowError(
            f"写入时工件{class_name!r}已从tool_offsets.yaml删除")
    tool = _mapping(tools[class_name], f"tools.{class_name}")
    tool["standard_to_tool"] = {
        "xyz_mm": [round(float(value), 6) for value in new_offset[:3]],
        "rpy_deg": [round(float(value), 6) for value in new_offset[3:]],
    }
    tools[class_name] = tool
    data["tools"] = tools
    try:
        _write_yaml_atomic(offsets_path, data)
        load_tool_offsets(offsets_path)
    except Exception:
        shutil.copy2(backup_path, offsets_path)
        raise


def run_interactive_calibration(
        config_path: Path = DEFAULT_CONFIG,
        *, input_fn: InputFunction = input,
        print_fn: PrintFunction = print,
        pose_reader: Optional[PoseReader] = None) -> dict:
    config_path = Path(config_path).expanduser().resolve()
    settings = _load_yaml(config_path)
    if int(settings.get("version", 1)) != 1:
        raise CalibrationWorkflowError("交互标定配置version只支持1")
    files = _mapping(settings.get("files"), "files")
    offsets_path = _resolve_from(
        config_path,
        files.get("tool_offsets_file", "../config/tool_offsets.yaml"),
        "files.tool_offsets_file")
    backup_path = _resolve_from(
        config_path,
        files.get("backup_file", "./tool_offsets.backup.yaml"),
        "files.backup_file")
    result_path = _resolve_from(
        config_path,
        files.get(
            "result_file", "./interactive_tool_calibration_result.yaml"),
        "files.result_file")

    # 启动后、任何现场操作之前立即校验并覆盖唯一备份。
    snapshot = load_tool_offsets(offsets_path)
    _single_backup(offsets_path, backup_path)
    print_fn("=" * 66)
    print_fn("交互式末端工件标定（只读取机械臂，不主动运动）")
    print_fn(f"正式工具配置：{offsets_path}")
    print_fn(f"本次启动备份：{backup_path}（下次运行会覆盖此备份）")
    print_fn("任何步骤均可输入q取消；取消不会修改正式工具配置。")
    print_fn("=" * 66)

    class_name, tool = _select_workpiece(snapshot, input_fn, print_fn)
    current_offset = _current_offset(tool)
    print_fn("\n选择模式：")
    print_fn("  1 = 从零标定：先记录50mm标准位，再记录工具工作位")
    print_fn("  2 = 纠正已有工件：先记录旧工作位，再记录调整后工作位")
    print_fn("  3 = XYZ/RPY微调：记录当前工作位后输入增量")
    mode = int(_ask_choice(
        input_fn, print_fn, "请输入模式1/2/3（q取消）：", ("1", "2", "3")))

    if pose_reader is None:
        raise RuntimeError("内部错误：未提供机械臂位姿读取器")

    if mode == 1:
        standard_tcp = _wait_to_read(
            input_fn, print_fn,
            "请先将相机移动到：光心正对目标安装平面、距离50mm的标准位。",
            pose_reader)
        taught_tcp = _wait_to_read(
            input_fn, print_fn,
            "请手动调整机械臂，使工具准确到达最终工作位。调整完成后再继续。",
            pose_reader)
        new_offset = calculate_from_zero(standard_tcp, taught_tcp)
        T_standard = _pose_to_matrix(standard_tcp)
        expected_work = taught_tcp
        detail = {
            "standard_tcp_mm_rpy_deg": standard_tcp.tolist(),
            "taught_tcp_mm_rpy_deg": taught_tcp.tolist(),
        }
    elif mode == 2:
        old_work_tcp = _wait_to_read(
            input_fn, print_fn,
            "请先让机械臂使用当前工具偏移到达旧工作位。",
            pose_reader)
        new_work_tcp = _wait_to_read(
            input_fn, print_fn,
            "请手动调整到新的正确工作位。调整完成后再继续。",
            pose_reader)
        new_offset = calculate_correction(
            current_offset, old_work_tcp, new_work_tcp)
        T_standard = (
            _pose_to_matrix(old_work_tcp) @
            np.linalg.inv(_pose_to_matrix(current_offset)))
        expected_work = new_work_tcp
        detail = {
            "old_work_tcp_mm_rpy_deg": old_work_tcp.tolist(),
            "new_work_tcp_mm_rpy_deg": new_work_tcp.tolist(),
        }
    else:
        current_work_tcp = _wait_to_read(
            input_fn, print_fn,
            "请让机械臂到达当前工件工作位。程序将以此位姿作为微调起点。",
            pose_reader)
        print_fn("\n选择微调坐标系：")
        print_fn("  1 = 标准/相机参考坐标系（推荐，跟随柜体朝向）")
        print_fn("  2 = JAKA基座坐标系（固定按基座X/Y/Z）")
        frame_choice = _ask_choice(
            input_fn, print_fn,
            "请输入1或2，直接按Enter默认选择1：", ("1", "2"), "1")
        xyz = _read_vector(
            input_fn, print_fn,
            "输入XYZ位移mm，例如0,-1,0表示所选坐标系Y-1mm：")
        rpy = _read_vector(
            input_fn, print_fn,
            "输入RPY角度增量°，不调整可直接按Enter：",
            default_zero=True)
        if frame_choice == "1":
            adjustment_frame = "standard_camera_reference"
            new_offset, expected_work = calculate_reference_adjustment(
                current_offset, current_work_tcp, xyz, rpy)
        else:
            adjustment_frame = "jaka_base"
            new_offset, expected_work = calculate_adjustment(
                current_offset, current_work_tcp, xyz, rpy)
        T_standard = (
            _pose_to_matrix(current_work_tcp) @
            np.linalg.inv(_pose_to_matrix(current_offset)))
        detail = {
            "current_work_tcp_mm_rpy_deg": current_work_tcp.tolist(),
            "adjustment_frame": adjustment_frame,
            "adjust_xyz_mm": xyz.tolist(),
            "adjust_rpy_deg": rpy.tolist(),
        }

    T_predicted = T_standard @ _pose_to_matrix(new_offset)
    T_expected = _pose_to_matrix(expected_work)
    position_error, rotation_error = _verification_error(
        T_predicted, T_expected)
    _apply_offset(offsets_path, backup_path, class_name, new_offset)

    now = datetime.now(timezone.utc).astimezone().isoformat(
        timespec="seconds")
    result = {
        "version": 1,
        "calibrated_at": now,
        "mode": mode,
        "workpiece": {
            "class_name": class_name,
            "code": str(tool.get("code") or ""),
            "tool_id": str(tool.get("tool_id") or ""),
        },
        "previous_standard_to_tool": {
            "xyz_mm": current_offset[:3].tolist(),
            "rpy_deg": current_offset[3:].tolist(),
        },
        "standard_to_tool": {
            "xyz_mm": [round(float(value), 6) for value in new_offset[:3]],
            "rpy_deg": [round(float(value), 6) for value in new_offset[3:]],
        },
        "expected_work_tcp_mm_rpy_deg": [
            round(float(value), 9) for value in expected_work],
        "verification": {
            "position_error_mm": position_error,
            "rotation_error_deg": rotation_error,
        },
        "detail": detail,
        "tool_offsets_file": str(offsets_path),
        "backup_file": str(backup_path),
    }
    _write_yaml_atomic(result_path, result)

    print_fn("\n" + "=" * 66)
    print_fn("标定完成，已自动更新tool_offsets.yaml")
    print_fn(f"工件：{class_name}  code={tool.get('code', '-')}")
    print_fn(
        "xyz_mm:  [" +
        ", ".join(f"{value:.6f}" for value in new_offset[:3]) + "]")
    print_fn(
        "rpy_deg: [" +
        ", ".join(f"{value:.6f}" for value in new_offset[3:]) + "]")
    print_fn(
        f"正向校验：位置残差={position_error:.9f}mm，"
        f"旋转残差={rotation_error:.9f}°")
    print_fn(f"唯一备份：{backup_path}")
    print_fn(f"结果记录：{result_path}")
    print_fn("视觉服务下一次解算会热加载新偏移，无需重启。")
    print_fn("=" * 66)
    return result


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="一次运行完成工件交互标定")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    args = parser.parse_args(argv)
    reader: Optional[JakaPoseReader] = None
    try:
        settings = _load_yaml(args.config.expanduser().resolve())
        reader = JakaPoseReader(_mapping(settings.get("robot"), "robot"))
        run_interactive_calibration(args.config, pose_reader=reader)
    except InteractiveCalibrationCancelled:
        print("\n[已取消] 未修改正式工具配置。")
        return 1
    except (CalibrationWorkflowError, FileNotFoundError, KeyError,
            RuntimeError, ValueError, OSError) as exc:
        print(f"\n[错误] {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("\n[已取消] 未修改正式工具配置。")
        return 130
    finally:
        if reader is not None:
            reader.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
