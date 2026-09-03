# -*- coding: utf-8 -*-
"""像真实后台一样通过POST /snapshot采集图片，并记录当前JAKA TCP。"""
from __future__ import annotations

import argparse
from pathlib import Path

import cv2

from common import (create_http_client, create_jaka, load_test_case,
                    read_jaka_pose_mm_rpy_deg, save_test_case)


DEFAULT_CASE = Path(__file__).resolve().parent / "test_case.yaml"


def main() -> int:
    parser = argparse.ArgumentParser(description="采集LabelMe现场测试样本")
    parser.add_argument("--case", default=str(DEFAULT_CASE))
    args = parser.parse_args()
    case_path, data = load_test_case(args.case)
    case = dict(data.get("case") or {})

    robot = create_jaka(data, enable_motion=False)
    try:
        robot.connect()
        capture_tcp = read_jaka_pose_mm_rpy_deg(robot)
    finally:
        robot.disconnect()

    result = create_http_client(data).snapshot()
    if int(result.get("code", 0)) != 200:
        raise RuntimeError(f"POST /snapshot失败: {result}")

    image_path = Path(result["path"]).resolve()
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"快照已返回但无法读取: {image_path}")
    try:
        image_value = str(image_path.relative_to(case_path.parent))
    except ValueError:
        image_value = str(image_path)
    case["image_file"] = image_value
    case["capture_tcp_mm_rpy_deg"] = [float(x) for x in capture_tcp]
    case["captured_image_width"] = int(image.shape[1])
    case["captured_image_height"] = int(image.shape[0])
    case["captured_at_unix_s"] = float(image_path.stat().st_mtime)
    data["case"] = case
    save_test_case(case_path, data)
    print("彩色图:", image_path)
    print("拍照TCP[mm, RPY deg]:", [round(float(x), 6) for x in capture_tcp])
    print("测试配置已更新:", case_path)
    print("下一步请用LabelMe标注目标矩形，然后保持视觉服务运行并执行第二步。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
