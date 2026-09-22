# -*- coding: utf-8 -*-
from __future__ import annotations

import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from handeye_calib.surfacepro50_adapter import SurfacePro50Backend


class _FakeStream:
    def __init__(self):
        self.read_count = 0

    def read_frame(self):
        self.read_count += 1
        return SimpleNamespace(frameIndex=self.read_count, timestamp=self.read_count * 1000)


class _AliveThread:
    @staticmethod
    def is_alive():
        return True


class SurfacePro50DiscardTests(unittest.TestCase):
    @staticmethod
    def _backend():
        backend = object.__new__(SurfacePro50Backend)
        backend._lock = threading.RLock()
        backend._connected = True
        backend.device = object()
        backend.image_stream = _FakeStream()
        backend.depth_stream = _FakeStream()
        backend.show_frame_output = False
        backend.last_discarded_frame_pairs = 0
        backend.last_discard_duration_s = 0.0
        return backend

    def test_discard_consumes_both_raw_streams_for_duration(self):
        backend = self._backend()
        clock = [0.0]
        def wait(streams, timeout):
            clock[0] += timeout
            return streams[0]
        backend.openni2 = SimpleNamespace(wait_for_any_stream=wait)
        with patch(
                "handeye_calib.surfacepro50_adapter.time.monotonic",
                side_effect=lambda: clock[0]):
            discarded = backend.discard_frames(1.0)

        self.assertGreater(discarded, 0)
        self.assertEqual(discarded, min(backend.image_stream.read_count, backend.depth_stream.read_count))
        self.assertAlmostEqual(backend.last_discard_duration_s, 1.0)

    def test_empty_depth_does_not_block_color_or_exceed_budget(self):
        backend = self._backend()
        clock = [0.0]
        def wait(streams, timeout):
            self.assertLessEqual(timeout, 0.05)
            clock[0] += timeout
            return streams[0] if streams[0] is backend.image_stream else None
        backend.openni2 = SimpleNamespace(wait_for_any_stream=wait)
        with patch('handeye_calib.surfacepro50_adapter.time.monotonic', side_effect=lambda: clock[0]):
            backend.discard_frames(4)
        self.assertEqual(backend.depth_stream.read_count, 0)
        self.assertGreater(backend.image_stream.read_count, 0)
        self.assertAlmostEqual(clock[0], 4)

    def test_missing_wait_fails_without_blocking_read(self):
        backend = self._backend()
        backend.openni2 = SimpleNamespace()
        with self.assertRaisesRegex(RuntimeError, 'wait_for_any_stream'):
            backend.discard_frames(4)
        self.assertEqual(backend.image_stream.read_count, 0)

    def test_final_frame_must_advance_past_discarded_frame(self):
        backend = self._backend()
        backend._discard_baselines = {id(backend.image_stream): {'frameIndex': 1, 'timestamp': 1000}}
        with self.assertRaisesRegex(RuntimeError, '未比丢弃帧更新'):
            backend._read_stream_frame(backend.image_stream, '图像')
        frame = backend._read_stream_frame(backend.image_stream, '图像')
        self.assertEqual(frame.frameIndex, 2)

    def test_zero_duration_resets_statistics_without_reading(self):
        backend = self._backend()
        backend.last_discarded_frame_pairs = 7
        backend.last_discard_duration_s = 4.0

        self.assertEqual(backend.discard_frames(0.0), 0)
        self.assertEqual(backend.image_stream.read_count, 0)
        self.assertEqual(backend.depth_stream.read_count, 0)
        self.assertEqual(backend.last_discarded_frame_pairs, 0)
        self.assertEqual(backend.last_discard_duration_s, 0.0)

    def test_zero_timestamp_accepts_advancing_frame_index_on_both_streams(self):
        for channel in ('图像', '深度'):
            backend = self._backend()
            stream = backend.image_stream
            backend._discard_baselines = {id(stream): {'frameIndex': 10, 'timestamp': 0}}
            with patch.object(stream, 'read_frame', return_value=SimpleNamespace(frameIndex=11, timestamp=0)):
                self.assertEqual(backend._read_stream_frame(stream, channel).frameIndex, 11)

    def test_zero_timestamp_does_not_hide_duplicate_valid_index(self):
        backend = self._backend()
        stream = backend.image_stream
        backend._discard_baselines = {id(stream): {'frameIndex': 10, 'timestamp': 0}}
        with patch.object(stream, 'read_frame', return_value=SimpleNamespace(frameIndex=10, timestamp=0)):
            with self.assertRaisesRegex(RuntimeError, 'frameIndex'):
                backend._read_stream_frame(stream, '图像')

    def test_unavailable_metadata_warns_instead_of_claiming_stale(self):
        backend = self._backend()
        stream = backend.image_stream
        backend._discard_baselines = {id(stream): {'frameIndex': 0, 'timestamp': 0}}
        with patch.object(stream, 'read_frame', return_value=SimpleNamespace(frameIndex=0, timestamp=0)):
            with self.assertLogs('vision_service.camera', level='WARNING'):
                backend._read_stream_frame(stream, '深度')

    @staticmethod
    def _latest_frame_backend():
        backend = object.__new__(SurfacePro50Backend)
        backend.fresh_frame_timeout_s = 0.5
        backend._frame_condition = threading.Condition()
        backend._frame_pump_thread = _AliveThread()
        backend._frame_pump_last_error = ""
        backend._image_raw_sequence = 3
        backend._depth_raw_sequence = 7
        backend._latest_image_raw = {
            "sequence": 3, "arrival_monotonic": time.monotonic(),
            "frameIndex": 30, "timestamp": 3000,
        }
        backend._latest_depth_raw = {
            "sequence": 7, "arrival_monotonic": time.monotonic(),
            "frameIndex": 70, "timestamp": 7000,
        }
        return backend

    def test_capture_waits_for_frames_newer_than_request(self):
        backend = self._latest_frame_backend()

        def publish_new_frames():
            with backend._frame_condition:
                now = time.monotonic()
                backend._image_raw_sequence = 4
                backend._depth_raw_sequence = 8
                backend._latest_image_raw = {
                    "sequence": 4, "arrival_monotonic": now,
                    "frameIndex": 31, "timestamp": 3100,
                }
                backend._latest_depth_raw = {
                    "sequence": 8, "arrival_monotonic": now + 0.001,
                    "frameIndex": 71, "timestamp": 7100,
                }
                backend._frame_condition.notify_all()

        timer = threading.Timer(0.02, publish_new_frames)
        timer.start()
        image, depth, metadata = backend._wait_for_fresh_raw_frames(True)
        timer.join()
        self.assertEqual(image["sequence"], 4)
        self.assertEqual(depth["sequence"], 8)
        self.assertTrue(metadata["fresh_frames_after_request"])
        self.assertGreaterEqual(metadata["fresh_frame_wait_ms"], 0.0)

    def test_new_color_waits_for_following_depth(self):
        backend = self._latest_frame_backend()

        def publish_image():
            with backend._frame_condition:
                backend._image_raw_sequence = 4
                backend._latest_image_raw = {
                    "sequence": 4, "arrival_monotonic": time.monotonic(),
                    "frameIndex": 31, "timestamp": 3100,
                }
                backend._frame_condition.notify_all()

        def publish_depth():
            with backend._frame_condition:
                backend._depth_raw_sequence = 8
                backend._latest_depth_raw = {
                    "sequence": 8, "arrival_monotonic": time.monotonic(),
                    "frameIndex": 71, "timestamp": 7100,
                }
                backend._frame_condition.notify_all()

        image_timer = threading.Timer(0.01, publish_image)
        depth_timer = threading.Timer(0.04, publish_depth)
        image_timer.start()
        depth_timer.start()
        image, depth, metadata = backend._wait_for_fresh_raw_frames(True)
        image_timer.join()
        depth_timer.join()
        self.assertEqual(image["sequence"], 4)
        self.assertEqual(depth["sequence"], 8)
        self.assertTrue(metadata["depth_frame_after_color"])
        self.assertGreaterEqual(metadata["fresh_frame_wait_ms"], 20.0)

    def test_timeout_uses_cached_color_when_only_depth_advances(self):
        backend = self._latest_frame_backend()
        backend.fresh_frame_timeout_s = 0.03

        def publish_new_depth():
            with backend._frame_condition:
                now = time.monotonic()
                backend._depth_raw_sequence = 8
                backend._latest_depth_raw = {
                    "sequence": 8, "arrival_monotonic": now,
                    "frameIndex": 71, "timestamp": 7100,
                }
                backend._frame_condition.notify_all()

        timer = threading.Timer(0.01, publish_new_depth)
        timer.start()
        image, depth, metadata = backend._wait_for_fresh_raw_frames(True)
        timer.join()
        self.assertEqual(image["sequence"], 3)
        self.assertEqual(depth["sequence"], 8)
        self.assertFalse(metadata["image_frame_after_request"])
        self.assertTrue(metadata["depth_frame_after_request"])
        self.assertTrue(metadata["cached_color_fallback"])
        self.assertLess(metadata["fresh_frame_wait_ms"], 200.0)

    def test_old_cached_frames_are_accepted_after_timeout(self):
        backend = self._latest_frame_backend()
        backend.fresh_frame_timeout_s = 0.03
        old_arrival = time.monotonic() - 1.0
        backend._latest_image_raw["arrival_monotonic"] = old_arrival
        backend._latest_depth_raw["arrival_monotonic"] = old_arrival
        image, depth, metadata = backend._wait_for_fresh_raw_frames(True)
        self.assertEqual(image["sequence"], 3)
        self.assertEqual(depth["sequence"], 7)
        self.assertTrue(metadata["cached_color_fallback"])
        self.assertGreaterEqual(
            metadata["image_cache_age_at_request_ms"], 900.0)


if __name__ == "__main__":
    unittest.main()
