# -*- coding: utf-8 -*-
from __future__ import annotations

import copy
import json
import tempfile
import unittest
from pathlib import Path

from datas_get.clients import SimulatedColorCaptureClient, SimulatedRobot
from datas_get.collector import DatasetCollector
from datas_get.config import CollectionConfig, load_collection_config


ROOT = Path(__file__).resolve().parents[1]


def temporary_config(output: Path, *, views=None) -> CollectionConfig:
    original = load_collection_config(ROOT / "config.yaml")
    data = copy.deepcopy(original.data)
    data["dataset"]["output_directory"] = str(output)
    if views is not None:
        data["trajectory"]["views"] = views
    config = CollectionConfig(original.source_path, data)
    config.validate()
    return config


class CollectorTests(unittest.TestCase):
    def test_full_simulation_saves_only_color_and_returns_reference(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            config = temporary_config(root / "dataset")
            reference = config.reference
            robot = SimulatedRobot(reference["simulation_tcp_mm_rpy_deg"])
            client = SimulatedColorCaptureClient(
                root / "service", reference["image_width"],
                reference["image_height"], reference["roi_xyxy_px"])
            session = DatasetCollector(config, robot, client).run()

            images = sorted((session / "images").glob("*"))
            plan = json.loads((session / "plan.json").read_text(
                encoding="utf-8"))
            planned_samples = sum(
                len(route["waypoints"]) for route in plan["routes"])
            self.assertEqual(len(images), planned_samples + 1)
            self.assertTrue(all(path.suffix == ".png" for path in images))
            self.assertFalse(list(session.rglob("*.ply")))
            self.assertFalse(list(session.rglob("*.tif")))
            records = [json.loads(line) for line in
                       (session / "manifest.jsonl").read_text(
                           encoding="utf-8").splitlines()]
            self.assertEqual(len(records), planned_samples)
            self.assertTrue(all(item["roiVisible"] for item in records))
            self.assertTrue(all(
                item["actualTcpBeforeMmRpyDeg"] == item["plannedTcpMmRpyDeg"] ==
                item["actualTcpAfterMmRpyDeg"] for item in records))
            summary = json.loads((session / "session.json").read_text(
                encoding="utf-8"))
            self.assertEqual(summary["imageCount"], planned_samples + 1)
            self.assertEqual(summary["routeCount"], 10)
            self.assertFalse(robot.is_connected())
            self.assertEqual(robot.pose.tolist(),
                             reference["simulation_tcp_mm_rpy_deg"])

    def test_rejected_route_causes_no_robot_motion(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            config = temporary_config(root / "dataset", views=[{
                "name": "invalid", "azimuth_deg": 80,
                "elevation_deg": 60, "distance_offset_mm": -480,
            }])
            reference = config.reference
            robot = SimulatedRobot(reference["simulation_tcp_mm_rpy_deg"])
            client = SimulatedColorCaptureClient(
                root / "service", reference["image_width"],
                reference["image_height"], reference["roi_xyxy_px"])
            with self.assertRaisesRegex(RuntimeError, "机械臂尚未运动|所有采集路线"):
                DatasetCollector(config, robot, client).run()
            self.assertEqual(robot.move_history, [])
            self.assertFalse(robot.is_connected())


if __name__ == "__main__":
    unittest.main()
