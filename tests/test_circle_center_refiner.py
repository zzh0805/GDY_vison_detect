# -*- coding: utf-8 -*-
from __future__ import annotations

import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from vision_solver.circle_center_refiner import (  # noqa: E402
    find_nearest_circle_center, validate_circle_refinement_settings)


def circle_settings(**overrides):
    settings = {
        "enabled": True,
        "expand_ratio": 0.45,
        "min_score": 0.35,
        "min_radius_ratio": 0.14,
        "max_radius_ratio": 0.46,
        "hough_param2": 28,
        "max_center_distance_px": 250,
        "fallback_to_box_center": False,
    }
    settings.update(overrides)
    return settings


class CircleCenterRefinerTests(unittest.TestCase):
    def test_selects_only_circle_nearest_to_request_box_center(self):
        image = np.full((500, 500, 3), 210, dtype=np.uint8)
        cv2.circle(image, (130, 250), 62, (25, 25, 25), 6,
                   cv2.LINE_AA)
        cv2.circle(image, (320, 250), 62, (25, 25, 25), 6,
                   cv2.LINE_AA)
        corners = np.array([
            [150, 140], [350, 140], [350, 360], [150, 360],
        ], dtype=np.float64)

        match = find_nearest_circle_center(
            image, corners, circle_settings())

        self.assertIsNotNone(match)
        self.assertGreaterEqual(match.accepted_candidate_count, 2)
        self.assertTrue(np.allclose(
            match.center_px, [320, 250], atol=4.0), match)
        self.assertAlmostEqual(
            match.distance_to_request_center_px, 70.0, delta=4.0)

    def test_returns_none_when_no_circle_exists(self):
        image = np.full((300, 300, 3), 128, dtype=np.uint8)
        corners = np.array([
            [100, 100], [200, 100], [200, 200], [100, 200],
        ], dtype=np.float64)
        self.assertIsNone(find_nearest_circle_center(
            image, corners, circle_settings()))

    def test_rejects_invalid_radius_range(self):
        with self.assertRaisesRegex(ValueError, "半径比例"):
            validate_circle_refinement_settings(circle_settings(
                min_radius_ratio=0.4, max_radius_ratio=0.2))


if __name__ == "__main__":
    unittest.main()
