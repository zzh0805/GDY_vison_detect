import importlib.util
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from urllib.request import Request, urlopen

import cv2
import numpy as np
import yaml

from handeye_calib.native_settings import native_settings, wire_settings
from vision_solver.preview import PreviewService, preview_settings
from vision_solver.http_server import VisionHttpServer

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("recorder", ROOT / "field_test/14_record_rgb_preview.py")
recorder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(recorder)


class FakeClient:
    def __init__(self):
        self.running = False
        self.sequence = 0
        self.starts = self.stops = 0
        self.fail = False

    def is_connected(self):
        return True

    def preview_start(self):
        self.running = True
        self.starts += 1

    def preview_stop(self):
        self.running = False
        self.stops += 1

    def preview_frame(self):
        if self.fail:
            raise RuntimeError("simulated preview failure")
        self.sequence += 1
        rgb = np.full((48, 64, 3), [self.sequence % 255, 40, 180], np.uint8)
        return rgb, {"sequence": self.sequence, "epoch": 1, "acquisition_fps": 20, "age_ms": 0}


class SettingsTests(unittest.TestCase):
    def test_wire_is_numeric_and_stable(self):
        values = wire_settings(native_settings()).split()
        # Order and length must match Settings::parse() in native_camera/worker.cpp.
        self.assertEqual(len(values), 20)
        self.assertEqual(values[0], "3000")
        self.assertTrue(all(np.isfinite(float(v)) for v in values))

    def test_default_wire_line_is_pinned(self):
        # Golden cross-language check. tests/native_state_test.cpp parses this same literal and
        # asserts each field, so a reordering on either side is caught offline.
        self.assertEqual(wire_settings(native_settings()),
                         "3000 100 3 60 -1 -1 -1 -1 -1 0 60 0 0 0 0 500 100 10 60 300")

    def test_reject_bad_parameters(self):
        for cfg in ({"capture_timeout_ms": 0}, {"rgb_gain": float("nan")},
                    {"rgb_exposure_us": 1.2}, {"match_enabled": "false"},
                    {"depth_exposure": 100, "depth_frame_time": 50},
                    {"unknown": 1}, {"timestamp_policy": "ignore"},
                    {"drain_max_frames": 0}, {"drain_timeout_ms": 0},
                    {"rgb_read_timeout_ms": 10}, {"periodic_clear_s": -1},
                    {"memory_report_s": 4000}, {"startup_max_reads": 2}):
            with self.subTest(cfg=cfg), self.assertRaises(ValueError):
                native_settings(cfg)
        for cfg in ({"publish_fps": 0}, {"max_clients": 1.5}, {"enabled": "false"}):
            with self.assertRaises(ValueError):
                preview_settings(cfg)


class PreviewTests(unittest.TestCase):
    def setUp(self):
        self.client = FakeClient()
        self.preview = PreviewService(SimpleNamespace(preview_client=lambda: self.client),
                                      {"publish_fps": 30})

    def tearDown(self):
        self.preview.stop()

    def test_idempotent_start_and_stop_restore_trigger_idle(self):
        result = self.preview.start()
        self.assertTrue(result["started_new"])
        self.assertFalse(self.preview.start()["started_new"])
        # Frames are only encoded while a client is reading the stream.
        self.assertIsNone(self.preview.next_frame(None, timeout=.05))
        self.preview.subscriber_added()
        frame = self.preview.next_frame(None, timeout=2)
        self.assertIsNotNone(frame)
        self.assertEqual(cv2.imdecode(np.frombuffer(frame[1], np.uint8), 1).shape, (48, 64, 3))
        self.preview.subscriber_removed()
        self.preview.stop(result["session"])
        self.assertEqual(self.client.starts, 1)
        self.assertEqual(self.client.stops, 1)
        self.assertIsNone(self.preview._jpeg)

    def test_foreign_session_cannot_stop(self):
        self.preview.start()
        with self.assertRaises(ValueError):
            self.preview.stop("not-the-owner")
        self.assertTrue(self.preview.status()["active"])

    def test_auto_stop_and_failure_release(self):
        self.client.fail = True
        self.preview.start()
        deadline = time.monotonic()+2
        while self.preview.status()["active"] and time.monotonic()<deadline:
            time.sleep(.01)
        self.preview.stop()
        self.assertIn("simulated", self.preview.status()["error"])
        self.assertEqual(self.client.stops, 1)

    def test_no_frame_wait_is_not_busy_loop(self):
        start = time.monotonic()
        self.assertIsNone(self.preview.next_frame(None, timeout=.05))
        self.assertGreater(time.monotonic()-start, .04)

    def test_session_timeout(self):
        self.preview.config["max_session_s"] = .05
        self.preview.start()
        time.sleep(.15)
        self.assertFalse(self.preview.status()["active"])
        self.assertEqual(self.client.stops, 1)


class RecorderTests(unittest.TestCase):
    def test_reject_unbounded_frame(self):
        data = io.BytesIO(b"--frame\r\nContent-Length: 999999999\r\n\r\n")
        with self.assertRaises(RuntimeError):
            recorder.read_frame(data)

    def test_timeline_and_mp4_decode(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "video.mp4"
            writer = recorder.TimelineWriter(path, 10, "mp4v")
            image = np.zeros((48, 64, 3), np.uint8)
            writer.add(image, 100)
            writer.add(image, 100.5)
            writer.close(101)
            self.assertEqual(writer.written, 11)
            self.assertGreater(writer.duplicates, 0)
            video = cv2.VideoCapture(str(path))
            try:
                self.assertTrue(video.isOpened())
                self.assertEqual(int(video.get(cv2.CAP_PROP_FRAME_COUNT)), 11)
                self.assertTrue(video.read()[0])
            finally:
                video.release()

    def test_full_http_to_mp4_script(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            client = FakeClient()
            config = SimpleNamespace(data={"preview": {"publish_fps": 15}}, http={},
                                     resolve_path=lambda value: root)
            solver = SimpleNamespace(config=config, camera=SimpleNamespace(preview_client=lambda: client))
            server = VisionHttpServer(SimpleNamespace(solver=solver), "127.0.0.1", 0)
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            host, port = server.server_address
            workflow = root / "workflow.yaml"
            workflow.write_text(yaml.safe_dump({"preview": {"record_seconds": 1}}), encoding="utf-8")
            output = root / "record.mp4"
            try:
                run = subprocess.run([sys.executable, str(ROOT / "field_test/14_record_rgb_preview.py"),
                                      "--config", str(workflow), "--base-url", f"http://{host}:{port}",
                                      "--output", str(output)], capture_output=True, timeout=20)
                self.assertEqual(run.returncode, 0, run.stdout.decode(errors="replace")+run.stderr.decode(errors="replace"))
                stats = json.loads(output.with_suffix(".json").read_text(encoding="utf-8"))
                self.assertGreater(stats["received_frames"], 2)
                self.assertGreater(stats["receive_fps"], 0)
                self.assertEqual(client.stops, 1)
                video = cv2.VideoCapture(str(output))
                try:
                    self.assertTrue(video.read()[0])
                finally:
                    video.release()
            finally:
                server.stop()
                thread.join(3)


if __name__ == "__main__":
    unittest.main()
