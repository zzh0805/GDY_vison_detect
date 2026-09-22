# -*- coding: utf-8 -*-
from __future__ import annotations

import unittest

import numpy as np

from vision_solver.http_protocol import VisionHttpProtocol


class _Config:
    http = {
        "request_timeout_s": 5,
        "snapshot_public_base_url": "http://vision-device:48051",
    }
    matching = {"source_image_width": 1920, "source_image_height": 1080}


class _RecordingSolver:
    config = _Config()

    def __init__(self):
        self.requests = []

    def handle_task(self, payload, timeout_s=None):
        self.requests.append((payload, timeout_s))
        if payload["taskType"] == "http_snapshot":
            return {"ok": True, "imagePath": "/tmp/20260828120000.jpg"}
        if payload["taskType"] == "capture_annotation_image":
            return {"ok": True, "imagePath": "/tmp/color.png"}
        if payload["taskType"] == "capture_dataset_reference":
            return {
                "ok": True, "imagePath": "/tmp/reference.png",
                "targetCameraMm": [1, 2, 500],
                "roiCornersCameraMm": [[0, 0, 500]] * 4,
                "planeNormalCamera": [0, 0, -1],
                "quality": {"planeRmsMm": 0.2},
            }
        return {
            "ok": True,
            "targetTcpMmRpyDeg": [1, 2, 3, 90, -45, 180],
            "geometry": {"approachDirectionBase": [0, 2, 0]},
        }


class HttpProtocolTests(unittest.TestCase):
    def test_boundary_converts_rad_to_deg_and_back(self):
        solver = _RecordingSolver()
        protocol = VisionHttpProtocol(solver)
        result = protocol.get_tcp_pose({
            "pos": [10, 20, 30, np.pi / 2, -np.pi / 4, np.pi],
            "x1": 100, "y1": 200, "x2": 300, "y2": 400,
            "code": "9-8-1",
        })
        internal = solver.requests[0][0]
        self.assertTrue(np.allclose(
            internal["captureTcpMmRpyDeg"],
            [10, 20, 30, 90, -45, 180]))
        self.assertEqual(internal["targetCornersPx"], [
            [100, 200], [300, 200], [300, 400], [100, 400]])
        self.assertTrue(internal["useCachedHttpSnapshot"])
        self.assertEqual(internal["workpieceCode"], "9-8-1")
        self.assertEqual(set(result), {"code", "pos"})
        self.assertTrue(np.allclose(
            result["pos"], [1, 2, 3, np.pi / 2, -np.pi / 4, np.pi]))

    def test_snapshot_response_contains_only_code_and_path(self):
        result = VisionHttpProtocol(_RecordingSolver()).snapshot()
        self.assertEqual(result, {
            "code": 200,
            "path": (
                "http://vision-device:48051/snapshots/"
                "20260828120000.jpg"),
        })

    def test_snapshot_uses_request_host_when_public_base_is_empty(self):
        solver = _RecordingSolver()
        solver.config = type("AutoUrlConfig", (), {
            "http": {"request_timeout_s": 5},
            "matching": {
                "source_image_width": 1920,
                "source_image_height": 1080,
            },
        })()
        result = VisionHttpProtocol(solver).snapshot(
            request_base_url="http://10.10.20.30:48051")
        self.assertEqual(result, {
            "code": 200,
            "path": (
                "http://10.10.20.30:48051/snapshots/"
                "20260828120000.jpg"),
        })

    def test_motion_result_adds_normalized_approach_without_changing_public(self):
        solver = _RecordingSolver()
        protocol = VisionHttpProtocol(solver)
        payload = {
            "pos": [10, 20, 30, 0, 0, 0],
            "target": {"x1": 100, "y1": 200, "x2": 300, "y2": 400},
        }
        public = protocol.get_tcp_pose(payload)
        self.assertEqual(set(public), {"code", "pos"})

        motion = protocol.get_tcp_pose_with_approach(payload)
        self.assertEqual(set(motion), {
            "code", "pos", "approachDirectionBase",
            "approachDirectionSource",
        })
        self.assertEqual(motion["pos"], public["pos"])
        self.assertTrue(np.allclose(
            motion["approachDirectionBase"], [0.0, 1.0, 0.0]))
        self.assertEqual(
            motion["approachDirectionSource"], "fitted_panel_normal")

    def test_motion_result_rejects_missing_direction(self):
        class _MissingDirectionSolver(_RecordingSolver):
            def handle_task(self, payload, timeout_s=None):
                result = super().handle_task(payload, timeout_s)
                if payload["taskType"] == "solve_target_tcp":
                    result.pop("geometry", None)
                return result

        result = VisionHttpProtocol(
            _MissingDirectionSolver()).get_tcp_pose_with_approach({
                "pos": [10, 20, 30, 0, 0, 0],
                "target": {
                    "x1": 100, "y1": 200, "x2": 300, "y2": 400,
                },
            })
        self.assertEqual(result["code"], 422)
        self.assertIn("安全进入方向", result["status"])

    def test_capture_color_is_additive_and_uses_annotation_task(self):
        solver = _RecordingSolver()
        result = VisionHttpProtocol(solver).capture_color({
            "dataset": "dataset_a", "fileName": "right_01.png"})
        self.assertEqual(result, {"code": 200, "path": "/tmp/color.png"})
        task = solver.requests[0][0]
        self.assertEqual(task["taskType"], "capture_annotation_image")
        self.assertEqual(task["datasetName"], "dataset_a")
        self.assertEqual(task["fileName"], "right_01.png")
        self.assertEqual(task["discardStaleFramesS"], 0.0)

    def test_dataset_reference_is_additive_and_does_not_change_old_responses(self):
        solver = _RecordingSolver()
        result = VisionHttpProtocol(solver).capture_dataset_reference({
            "dataset": "set_a", "fileName": "reference.png",
            "roi": {"x1": 100, "y1": 200, "x2": 300, "y2": 400},
            "geometry": {"ransac_threshold_mm": 3.0},
        })
        self.assertEqual(result["code"], 200)
        self.assertEqual(result["targetCameraMm"], [1, 2, 500])
        task = solver.requests[0][0]
        self.assertEqual(task["taskType"], "capture_dataset_reference")
        self.assertEqual(task["roiCornersPx"], [
            [100, 200], [300, 200], [300, 400], [100, 400]])
        self.assertEqual(VisionHttpProtocol(_RecordingSolver()).snapshot(), {
            "code": 200,
            "path": (
                "http://vision-device:48051/snapshots/"
                "20260828120000.jpg"),
        })

    def test_empty_workpiece_code_is_rejected(self):
        result = VisionHttpProtocol(_RecordingSolver()).get_tcp_pose({
            "pos": [10, 20, 30, 0, 0, 0],
            "target": {"x1": 100, "y1": 200, "x2": 300, "y2": 400},
            "code": "   ",
        })
        self.assertEqual(result["code"], 400)
        self.assertIn("code", result["status"])


if __name__ == "__main__":
    unittest.main()
