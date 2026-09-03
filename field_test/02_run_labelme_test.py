# -*- coding: utf-8 -*-
"""像真实后台一样通过POST /get_tcp_pose完成现场联动测试。"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

from common import (create_http_client, create_jaka,
                    jaka_deg_to_unified, jaka_rad_to_unified, load_test_case,
                    read_jaka_pose_mm_rpy_deg, read_labelme_corners,
                    resolve_from)


DEFAULT_CASE = Path(__file__).resolve().parent / "test_case.yaml"


def main() -> int:
    parser = argparse.ArgumentParser(description="LabelMe现场TCP解算测试")
    parser.add_argument("--case", default=str(DEFAULT_CASE))
    parser.add_argument(
        "--live", action="store_true",
        help="一步到位：实时采集当前帧拍照+检测+解算，无需先调用/snapshot")
    parser.add_argument(
        "--base-label", type=int, default=0,
        help="基座面板标签：用标注中该标签的矩形作为base（面板解算）")
    args = parser.parse_args()
    case_path, data = load_test_case(args.case)
    case = dict(data.get("case") or {})
    jaka = dict(data.get("jaka_test") or {})
    capture_tcp_value = case.get("capture_tcp_mm_rpy_deg")
    if capture_tcp_value is None:
        raise ValueError("请先运行01_capture_test_case.py记录拍照TCP")
    capture_tcp = np.asarray(capture_tcp_value, dtype=np.float64).reshape(6)
    labelme_path = resolve_from(case_path, case.get("labelme_file"))
    if not labelme_path.is_file():
        raise FileNotFoundError(f"LabelMe标注不存在: {labelme_path}")
    corners = read_labelme_corners(
        labelme_path,
        str(case.get("labelme_target_label", "request_target")),
        int(case.get("labelme_shape_index", 0)))
    # 基座面板矩形（可选）：从标注中 base-label 读，传给服务端做面板解算。
    base_rect = None
    if args.base_label:
        base_corners = read_labelme_corners(
            labelme_path, str(args.base_label), 0)
        bx1, by1 = np.min(base_corners, axis=0)
        bx2, by2 = np.max(base_corners, axis=0)
        base_rect = {"x1": float(bx1), "y1": float(by1),
                     "x2": float(bx2), "y2": float(by2)}

    start_jaka = bool(jaka.get("start_jaka", False))
    work_jaka = bool(jaka.get("work_jaka", False))
    robot = None
    try:
        if start_jaka or work_jaka:
            robot = create_jaka(data, enable_motion=True)
            robot.connect()
        if start_jaka:
            print("start_jaka=true：正在移动到记录的拍照TCP……")
            robot.move_linear(
                jaka_deg_to_unified(capture_tcp), name="field_test_capture_pose")
            # 机械臂到位后等待2秒稳定，再请求拍照检测（相机帧率低，避免运动未完全静止）。
            print("机械臂已到位，等待2秒稳定后再拍照检测……")
            time.sleep(2.0)
            # 使用运动完成后的实际TCP与实时图像绑定。
            capture_tcp = read_jaka_pose_mm_rpy_deg(robot)

        capture_tcp_rad = capture_tcp.copy()
        capture_tcp_rad[3:] = np.radians(capture_tcp_rad[3:])
        x1, y1 = np.min(corners, axis=0)
        x2, y2 = np.max(corners, axis=0)
        result = create_http_client(data).get_tcp_pose(
            capture_tcp_rad.tolist(), x1, y1, x2, y2,
            base=base_rect, live=bool(args.live))

        output_dir = case_path.parent / "output"
        output_dir.mkdir(parents=True, exist_ok=True)
        output_path = output_dir / "test_result.json"
        output_path.write_text(
            json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        print("测试结果(原始响应,mm+rad):", output_path)
        if int(result.get("code", 0)) != 200:
            raise RuntimeError(f"POST /get_tcp_pose失败: {result}")
        if set(result) != {"code", "pos"}:
            raise RuntimeError(f"成功响应字段不符合最新协议: {sorted(result)}")

        # 服务返回 pos 为 mm + RPY 弧度；JAKA 示教器用 mm + RPY 度。
        # 打印可直接输入示教器的坐标（拷贝这行即可）。
        pos_rad = np.asarray(result["pos"], dtype=np.float64).reshape(6)
        pos_deg = pos_rad.copy()
        pos_deg[3:] = np.degrees(pos_deg[3:])
        print("\n==== 目标TCP（可直接输入示教器，单位 mm + RPY 度） ====")
        print("  [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]"
              % tuple(pos_deg))
        print("   X=%.2f  Y=%.2f  Z=%.2f  Rx=%.3f°  Ry=%.3f°  Rz=%.3f°"
              % tuple(pos_deg))
        print("====================================================")

        if work_jaka:
            print("work_jaka=true：正在移动到解算工作TCP……")
            robot.move_linear(
                jaka_rad_to_unified(result["pos"]),
                name="field_test_work_pose")
            print("机器人已到达工作TCP")
        else:
            print("work_jaka=false：只输出TCP，未执行工作位运动")
    finally:
        if robot is not None:
            robot.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
