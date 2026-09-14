# -*- coding: utf-8 -*-
"""唯一运行入口：读取config.yaml并执行完整多视角彩色图采集。"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path


TOOL_ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = TOOL_ROOT.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from datas_get.clients import (HttpColorCaptureClient,
                               SimulatedColorCaptureClient,
                               SimulatedRobot)
from datas_get.collector import DatasetCollector
from datas_get.config import load_collection_config


DEFAULT_CONFIG = TOOL_ROOT / "config.yaml"


def main() -> int:
    parser = argparse.ArgumentParser(description="GDY多视角彩色图自动采集")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    args = parser.parse_args()
    config = load_collection_config(args.config)
    mode = str(config.runtime.get("mode", "simulation")).lower()
    if mode == "simulation":
        reference = config.reference
        robot = SimulatedRobot(reference["simulation_tcp_mm_rpy_deg"])
        client = SimulatedColorCaptureClient(
            TOOL_ROOT / ".simulation_service_images",
            int(reference["image_width"]), int(reference["image_height"]),
            reference["roi_xyxy_px"])
        print("运行模式：simulation（不会连接或移动真实机械臂）")
    else:
        robot_cfg = config.robot
        if not bool(robot_cfg.get("enable_motion", False)):
            raise RuntimeError("真实模式要求robot.enable_motion=true")
        if bool(config.runtime.get("require_confirmation", True)):
            answer = input("即将读取当前位置为参考位并自动移动JAKA，输入 START 继续：")
            if answer.strip() != "START":
                print("已取消，机械臂未运动")
                return 2
        from handeye_calib.jaka_adapter import JAKARobotAdapter
        robot = JAKARobotAdapter(
            endpoint=str(robot_cfg["ip"]), enable_motion=True,
            speed=float(robot_cfg["speed_mm_s"]) * 0.001,
            accel=float(robot_cfg["accel_mm_s2"]) * 0.001,
            settle=float(robot_cfg["settle_s"]),
            timeout=float(robot_cfg["timeout_s"]),
            sdk_path=(str(robot_cfg.get("sdk_path", "")).strip() or None))
        service = config.service
        client = HttpColorCaptureClient(
            str(service["base_url"]), float(service["timeout_s"]),
            float(service.get("discard_stale_frames_s", 0.0)))
        print("运行模式：real；当前JAKA TCP将作为参考拍照位")

    result = DatasetCollector(config, robot, client).run()
    print(f"采集完成：{result}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
