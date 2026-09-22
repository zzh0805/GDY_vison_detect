"""No camera / SDK import, no robot/network access."""
import unittest
from unittest.mock import patch, MagicMock
from tempfile import TemporaryDirectory
from pathlib import Path
from types import SimpleNamespace
from test_soft_trigger import memory_kib, verify_quiet, worker, CallTrace, watchdog_reason, ContentChanges, idle_phase, warmup_capture


class FakeCamera:
    def __init__(self, codes):
        self.codes = iter(codes)

    def read(self, timeout):
        return (next(self.codes), 0, 0, 0, 0, 0)


class OfflineTests(unittest.TestCase):
    def test_warmup_requires_five_successes(self):
        cam = MagicMock()
        cam.last_rgb = (0, 300, 10, 10)
        cam.read.side_effect = [(0, 0, 100, 10, 5, 1), (1, 0, 0, 0, 0, 0), (1, 0, 0, 0, 0, 0)] * 5
        with TemporaryDirectory() as folder:
            args = SimpleNamespace(output=Path(folder), warmup_frames=5, mode="soft", timeout_ms=3000)
            context = {}
            warmup_capture(cam, CallTrace(args.output), args, True, context)
            self.assertEqual(context["accepted_frames"], 5)
            self.assertEqual(cam.lib.st_trigger.call_count, 5)
            self.assertEqual(len((args.output / "warmup.csv").read_text().splitlines()), 6)

    def test_warmup_failure_stops_before_idle(self):
        cam = MagicMock()
        cam.read.return_value = (1, 0, 0, 0, 0, 0)
        with TemporaryDirectory() as folder:
            args = SimpleNamespace(output=Path(folder), warmup_frames=5, mode="soft", timeout_ms=3000)
            with self.assertRaisesRegex(RuntimeError, "未进入空闲"):
                warmup_capture(cam, CallTrace(args.output), args, True, {})
            self.assertEqual((args.output / "phase.txt").read_text(), "warmup")
            self.assertEqual(cam.read.call_count, 1)

    def test_idle_waits_without_camera_calls_and_has_own_deadline(self):
        import json
        with TemporaryDirectory() as folder:
            output = Path(folder)
            trace = CallTrace(output)
            with patch("test_soft_trigger.time.sleep") as sleep:
                idle_phase(output, trace, "idle_before", 300)
            sleep.assert_called_once_with(300)
            self.assertEqual((output / "phase.txt").read_text(), "idle_before")
            events = [json.loads(s) for s in (output / "calls.jsonl").read_text().splitlines()]
            self.assertEqual(len(events), 2)
            self.assertEqual(events[0]["operation"], "idle_before")
            self.assertAlmostEqual(events[0]["deadline"]-events[0]["started"], 310)
            self.assertIsNone(watchdog_reason(events[0], events[0]["started"]+200, 30))

    def test_content_change_intervals_and_static_tail(self):
        c = ContentChanges()
        self.assertEqual(c.update("a", 10), (False, None))
        self.assertEqual(c.update("a", 14), (False, None))
        self.assertEqual(c.update("b", 17), (True, 7))
        c.update("b", 27)
        self.assertEqual(c.summary()["content_changes"], 1)
        self.assertEqual(c.summary()["max_observed_unchanged_span_s"], 10)
        self.assertEqual(c.summary()["trailing_unchanged_span_s"], 10)

    def test_repeated_rgb_timestamp_only_warns(self):
        import json
        cam = MagicMock()
        cam.read.return_value = (0, 0, 100, 10, 5, 1)
        cam.last_rgb = (55807, 300, 10, 10)
        cam.last_rgb_hash = "abcdef"
        with TemporaryDirectory() as folder:
            args = SimpleNamespace(lib="unused", ip="test", mode="continuous", profile=0,
                                   streams="rgbd", rgb_profile=0, output=Path(folder),
                                   duration=0.45, timeout_ms=10, interval=0)
            with patch("test_soft_trigger.Camera", return_value=cam), \
                 patch("test_soft_trigger.time.monotonic", side_effect=[0, .1, .2, .3, .4, .5, .6]), \
                 patch("test_soft_trigger.time.sleep"):
                self.assertEqual(worker(args), 0)
            result = json.loads((Path(folder)/"capture_summary.json").read_text(encoding="utf-8"))
            self.assertEqual(result["frames_or_pairs"], 2)
            self.assertEqual(result["rgb_timestamp_warnings"], 1)
            self.assertEqual(result["rgb_content"]["content_changes"], 0)

    def test_original_error_saved_before_failing_cleanup(self):
        import json
        cam = MagicMock()
        cam.read.return_value = (0, 10, 100, 10, 5, 1)
        cam.last_rgb = (20, 300, 10, 10)
        with TemporaryDirectory() as folder:
            output = Path(folder)
            def fail_close():
                report = json.loads((output / "worker_error.json").read_text(encoding="utf-8"))
                self.assertIn("时间戳未递增", report["message"])
                self.assertEqual(report["context"]["attempt"], 2)
                self.assertEqual(report["context"]["last_read"]["rgb"], [20, 300, 10, 10])
                raise RuntimeError("cleanup also failed")
            cam.lib.st_close.side_effect = fail_close
            args = SimpleNamespace(lib="unused", ip="test", mode="continuous", profile=0,
                                   streams="rgbd", rgb_profile=0, output=output,
                                   duration=10, timeout_ms=10, interval=0)
            with patch("test_soft_trigger.Camera", return_value=cam):
                with self.assertRaisesRegex(RuntimeError, "时间戳未递增"):
                    worker(args)
            self.assertTrue((output / "cleanup_error.json").exists())

    def test_watchdog_begin_deadline_and_open_grace(self):
        state = {"phase": "begin", "operation": "open", "sequence": 1,
                 "started": 0, "updated": 0, "deadline": 90}
        self.assertIsNone(watchdog_reason(state, 45, 30))
        self.assertIn("open", watchdog_reason(state, 91, 30))

    def test_watchdog_idle(self):
        self.assertIsNone(watchdog_reason({"phase": "end", "updated": 10}, 20, 30))
        self.assertIsNotNone(watchdog_reason({"phase": "end", "updated": 10}, 50, 30))

    def test_trace_failure_persisted(self):
        import json
        with TemporaryDirectory() as folder:
            trace = CallTrace(Path(folder))
            with self.assertRaises(ValueError):
                trace.run("softTrigger(1)", lambda: (_ for _ in ()).throw(ValueError("SDK failure")))
            state = json.loads((Path(folder)/"call_state.json").read_text())
            self.assertEqual(state["phase"], "error")
            self.assertEqual(state["operation"], "softTrigger(1)")

    def test_parent_detects_blocked_child_without_sdk(self):
        import subprocess
        import sys
        import time
        import json
        with TemporaryDirectory() as folder:
            source = ("import time,sys; from pathlib import Path; "
                      "from test_soft_trigger import CallTrace; "
                      "CallTrace(Path(sys.argv[1]),0.2).run('fake_blocked_read',lambda:time.sleep(30))")
            child = subprocess.Popen([sys.executable, "-c", source, folder],
                                     cwd=Path(__file__).parent, stdout=subprocess.DEVNULL)
            try:
                until = time.perf_counter()+5
                reason = None
                while time.perf_counter() < until:
                    state_path = Path(folder)/"call_state.json"
                    if state_path.exists():
                        state = json.loads(state_path.read_text())
                        reason = watchdog_reason(state, time.perf_counter(), 30)
                        if reason:
                            break
                    time.sleep(0.02)
                self.assertIsNone(child.poll())
                self.assertIn("fake_blocked_read", reason or "")
            finally:
                child.terminate()
                child.wait(timeout=5)

    def test_rgbd_success_records_both_streams(self):
        import json
        cam = MagicMock()
        cam.read.return_value = (0, 0, 100, 10, 5, 1)
        cam.last_rgb = (0, 300, 10, 10)
        with TemporaryDirectory() as folder:
            args = SimpleNamespace(lib="unused", ip="test", mode="continuous", profile=0,
                                   streams="rgbd", rgb_profile=0, output=Path(folder),
                                   duration=0.25, timeout_ms=10, interval=0)
            with patch("test_soft_trigger.Camera", return_value=cam), \
                 patch("test_soft_trigger.time.monotonic", side_effect=[0, 0.1, 0.2, 0.3, 0.4]), \
                 patch("test_soft_trigger.time.sleep"):
                self.assertEqual(worker(args), 0)
            result = json.loads((Path(folder)/"capture_summary.json").read_text(encoding="utf-8"))
            self.assertEqual(result["frames_or_pairs"], 1)
            self.assertEqual(result["pairs_with_missing_timestamp"], 1)
            self.assertFalse(result["synchronization_proven"])
            self.assertIn("rgb_timestamp_ms", (Path(folder)/"frames.csv").read_text())
        cam.configure.assert_called_once_with("rgbd", 0)
        cam.lib.st_close.assert_called_once()

    def test_missing_rgb_is_failure(self):
        cam = MagicMock()
        cam.read.return_value = (0, 0, 100, 10, 5, 1)
        cam.last_rgb = None
        with TemporaryDirectory() as folder:
            args = SimpleNamespace(lib="unused", ip="test", mode="continuous", profile=0,
                                   streams="rgbd", rgb_profile=0, output=Path(folder),
                                   duration=10, timeout_ms=10, interval=0)
            with patch("test_soft_trigger.Camera", return_value=cam):
                with self.assertRaisesRegex(RuntimeError, "彩色帧"):
                    worker(args)
        cam.lib.st_close.assert_called_once()

    def test_sdk_c_types_are_not_in_cs_namespace(self):
        # Source regression guard, NOT a substitute for native SDK compilation.
        source = Path(__file__).with_name("bridge.cpp").read_text(encoding="utf-8")
        for name in ("PropertyExtension", "ERROR_CODE", "SUCCESS", "CameraInfo",
                     "StreamInfo", "STREAM_TYPE_DEPTH", "STREAM_FORMAT_Z16",
                     "PROPERTY_EXT_TRIGGER_MODE", "TRIGGER_MODE_SOFTWAER",
                     "TRIGGER_MODE_OFF", "ERROR_FRAME_TIMEOUT"):
            self.assertNotIn("cs::" + name, source)
            self.assertIn("::" + name, source)
        self.assertIn("cs::IFramePtr", source)

    def test_memory_is_current_not_peak(self):
        with patch("pathlib.Path.read_text", return_value="VmRSS:\t1024 kB\nVmHWM:\t8192 kB\nRssAnon: 512 kB\nThreads: 24\n"):
            m = memory_kib(123)
        self.assertEqual(m["VmRSS"], 1024)
        self.assertEqual(m["VmHWM"], 8192)

    def test_drain_then_quiet(self):
        self.assertEqual(verify_quiet(FakeCamera([0, 0, 1, 1, 1])), 2)

    def test_unsolicited_frame_resets_quiet_window(self):
        self.assertEqual(verify_quiet(FakeCamera([1, 1, 0, 1, 1, 1])), 1)

    def test_continuous_stream_does_not_pass(self):
        with self.assertRaises(RuntimeError):
            verify_quiet(FakeCamera([0] * 5), drain_limit=5)

    def test_trigger_timeout_closes_camera(self):
        cam = MagicMock()
        cam.read.return_value = (1, 0, 0, 0, 0, 0)
        with TemporaryDirectory() as folder:
            args = SimpleNamespace(lib="unused", ip="test", mode="soft", profile=0,
                                   output=Path(folder), duration=10, timeout_ms=10, interval=1)
            with patch("test_soft_trigger.Camera", return_value=cam):
                with self.assertRaisesRegex(RuntimeError, "超时"):
                    worker(args)
        cam.lib.st_trigger.assert_called_once()
        cam.lib.st_close.assert_called_once()

    def test_extra_frame_rejected_and_camera_closed(self):
        cam = MagicMock()
        timeout = (1, 0, 0, 0, 0, 0)
        frame = (0, 10, 100, 10, 5, 1)
        cam.read.side_effect = [timeout, timeout, timeout, frame, frame]
        with TemporaryDirectory() as folder:
            args = SimpleNamespace(lib="unused", ip="test", mode="soft", profile=0,
                                   output=Path(folder), duration=10, timeout_ms=10, interval=1)
            with patch("test_soft_trigger.Camera", return_value=cam):
                with self.assertRaisesRegex(RuntimeError, "额外帧"):
                    worker(args)
        cam.lib.st_close.assert_called_once()


if __name__ == "__main__":
    unittest.main()
