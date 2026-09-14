# -*- coding: utf-8 -*-
from __future__ import annotations

import threading
import unittest

import numpy as np

from handeye_calib.hardware_interfaces import CameraFrame
from vision_solver.api import VisionToolTcpSolver
from vision_solver.http_client import VisionHttpClient
from vision_solver.http_protocol import VisionHttpProtocol
from vision_solver.http_server import VisionHttpServer


class _Config:
    http = {"request_timeout_s": 5}
    matching = {"source_image_width": 1920, "source_image_height": 1080}


class _HttpFakeSolver:
    config = _Config()

    def __init__(self):
        self.tasks = []

    def handle_task(self, payload, timeout_s=None):
        self.tasks.append((payload, timeout_s))
        if payload["taskType"] == "capture_dataset_reference":
            return {
                "ok": True, "imagePath": "/tmp/reference.png",
                "targetCameraMm": [0, 0, 500],
                "roiCornersCameraMm": [[0, 0, 500]] * 4,
                "planeNormalCamera": [0, 0, -1],
                "quality": {"planeRmsMm": 0.1},
            }
        return {"ok": True, "imagePath": "/tmp/pure_color.png"}


class _Camera:
    def __init__(self):
        self.discarded = []

    def discard_stale_frames(self, duration_s):
        self.discarded.append(float(duration_s))
        return 1

    def capture_color(self):
        return CameraFrame(frame_id=3, color=np.zeros((20, 30, 3), np.uint8))


class _Artifacts:
    def save_annotation(self, frame, request):
        return {"imagePath": "/tmp/direct_color.png"}


class HttpColorEndpointTests(unittest.TestCase):
    def test_http_route_reaches_only_color_task(self):
        solver = _HttpFakeSolver()
        server = VisionHttpServer(VisionHttpProtocol(solver), "127.0.0.1", 0)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            host, port = server.server_address
            result = VisionHttpClient(
                f"http://{host}:{port}", timeout_s=5).capture_color(
                    "set_a", "left_001.png")
            self.assertEqual(result, {
                "code": 200, "path": "/tmp/pure_color.png"})
            task = solver.tasks[0][0]
            self.assertEqual(task["taskType"], "capture_annotation_image")
            self.assertEqual(task["datasetName"], "set_a")
            self.assertEqual(task["fileName"], "left_001.png")
            self.assertEqual(task["discardStaleFramesS"], 0.0)
        finally:
            server.stop()
            thread.join(timeout=5)

    def test_direct_color_capture_does_not_touch_http_snapshot_cache(self):
        solver = VisionToolTcpSolver.__new__(VisionToolTcpSolver)
        solver.camera = _Camera()
        solver.artifacts = _Artifacts()
        sentinel = object()
        solver._http_snapshot_frame = sentinel
        result = solver._handle_annotation({
            "taskId": "color-1", "datasetName": "set_a",
            "fileName": "a.png", "discardStaleFramesS": 0.25})
        self.assertTrue(result["ok"])
        self.assertIs(solver._http_snapshot_frame, sentinel)
        self.assertEqual(solver.camera.discarded, [0.25])

    def test_dataset_reference_http_route_is_separate_from_production_routes(self):
        solver = _HttpFakeSolver()
        server = VisionHttpServer(VisionHttpProtocol(solver), "127.0.0.1", 0)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            host, port = server.server_address
            result = VisionHttpClient(
                f"http://{host}:{port}", timeout_s=5
            ).capture_dataset_reference(
                "set_a", "reference.png", [100, 100, 500, 500],
                geometry={"ransac_threshold_mm": 3.0})
            self.assertEqual(result["code"], 200)
            self.assertEqual(
                solver.tasks[0][0]["taskType"],
                "capture_dataset_reference")
        finally:
            server.stop()
            thread.join(timeout=5)


if __name__ == "__main__":
    unittest.main()
