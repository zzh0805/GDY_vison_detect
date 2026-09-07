# -*- coding: utf-8 -*-
from __future__ import annotations

import threading
import unittest
from unittest.mock import patch

from handeye_calib.surfacepro50_adapter import SurfacePro50Backend


class _FakeStream:
    def __init__(self):
        self.read_count = 0

    def read_frame(self):
        self.read_count += 1
        return object()


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
        monotonic_values = [0.0, 0.4, 0.8, 1.1, 1.1]
        with patch(
                "handeye_calib.surfacepro50_adapter.time.monotonic",
                side_effect=monotonic_values):
            discarded = backend.discard_frames(1.0)

        self.assertEqual(discarded, 3)
        self.assertEqual(backend.image_stream.read_count, 3)
        self.assertEqual(backend.depth_stream.read_count, 3)
        self.assertEqual(backend.last_discarded_frame_pairs, 3)
        self.assertAlmostEqual(backend.last_discard_duration_s, 1.1)

    def test_zero_duration_resets_statistics_without_reading(self):
        backend = self._backend()
        backend.last_discarded_frame_pairs = 7
        backend.last_discard_duration_s = 4.0

        self.assertEqual(backend.discard_frames(0.0), 0)
        self.assertEqual(backend.image_stream.read_count, 0)
        self.assertEqual(backend.depth_stream.read_count, 0)
        self.assertEqual(backend.last_discarded_frame_pairs, 0)
        self.assertEqual(backend.last_discard_duration_s, 0.0)


if __name__ == "__main__":
    unittest.main()
