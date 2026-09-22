# -*- coding: utf-8 -*-
from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import numpy as np

from ply_io import read_ply_xyz, write_ply_xyz


class PlyIoTests(unittest.TestCase):
    def test_original_binary_ply_round_trip_filters_invalid_points(self):
        points = np.array([
            [1.0, 2.0, 3.0],
            [np.nan, 0.0, 1.0],
            [4.0, 5.0, 6.0],
        ], dtype=np.float32)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "points.ply"
            write_ply_xyz(path, points)
            recovered = read_ply_xyz(path)
        self.assertTrue(np.array_equal(
            recovered, np.array([[1, 2, 3], [4, 5, 6]], dtype=np.float32)))


if __name__ == "__main__":
    unittest.main()

