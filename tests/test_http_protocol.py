# -*- coding: utf-8 -*-
from __future__ import annotations

import unittest

import numpy as np

from vision_solver.http_protocol import VisionHttpProtocol


class _Config:
    http = {"request_timeout_s": 5}
    matching = {"source_image_width": 1920, "source_image_height": 1080}


class _RecordingSolver:
    config = _Config()

    def __init__(self):
        self.requests = []

    def handle_task(self, payload, timeout_s=None):
        self.requests.append((payload, timeout_s))
        if payload["taskType"] == "http_snapshot":
            return {"ok": True, "imagePath": "/tmp/20260828120000.jpg"}
        return {
            "ok": True,
            "targetTcpMmRpyDeg": [1, 2, 3, 90, -45, 180],
        }


class HttpProtocolTests(unittest.TestCase):
    def test_boundary_converts_rad_to_deg_and_back(self):
        solver = _RecordingSolver()
        protocol = VisionHttpProtocol(solver)
        result = protocol.get_tcp_pose({
            "pos": [10, 20, 30, np.pi / 2, -np.pi / 4, np.pi],
            "x1": 100, "y1": 200, "x2": 300, "y2": 400,
        })
        internal = solver.requests[0][0]
        self.assertTrue(np.allclose(
            internal["captureTcpMmRpyDeg"],
            [10, 20, 30, 90, -45, 180]))
        self.assertEqual(internal["targetCornersPx"], [
            [100, 200], [300, 200], [300, 400], [100, 400]])
        self.assertTrue(internal["useCachedHttpSnapshot"])
        self.assertEqual(set(result), {"code", "pos"})
        self.assertTrue(np.allclose(
            result["pos"], [1, 2, 3, np.pi / 2, -np.pi / 4, np.pi]))

    def test_snapshot_response_contains_only_code_and_path(self):
        result = VisionHttpProtocol(_RecordingSolver()).snapshot()
        self.assertEqual(result, {
            "code": 200, "path": "/tmp/20260828120000.jpg"})


if __name__ == "__main__":
    unittest.main()
