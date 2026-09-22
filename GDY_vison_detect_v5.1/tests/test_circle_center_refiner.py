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
        "min_radius_ratio": 0.08,
        "max_radius_ratio": 0.75,
        "min_edge_support": 0.55,
        "radius_band_count": 7,
        "center_cluster_tolerance_ratio": 0.18,
        "outer_circle_min_support_ratio": 0.65,
        "color_fusion": {
            "enabled": True,
            "center_mode": "edge",
            "min_coverage": 0.06,
            "score_weight": 0.20,
            "center_blend": 0.35,
            "max_center_shift_ratio": 0.20,
            "selection_weight": 0.20,
        },
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

    def test_selects_largest_valid_concentric_circle(self):
        image = np.full((500, 500, 3), 210, dtype=np.uint8)
        cv2.circle(image, (258, 246), 36, (25, 25, 25), 5,
                   cv2.LINE_AA)
        cv2.circle(image, (250, 250), 76, (35, 35, 35), 6,
                   cv2.LINE_AA)
        corners = np.array([
            [165, 165], [335, 165], [335, 335], [165, 335],
        ], dtype=np.float64)

        match = find_nearest_circle_center(
            image, corners, circle_settings(hough_param2=20))

        self.assertIsNotNone(match)
        self.assertGreaterEqual(match.selected_cluster_candidate_count, 2)
        self.assertTrue(np.allclose(
            match.center_px, [250, 250], atol=5.0), match)
        self.assertGreater(match.radius_px, 68.0, match)

    def test_fuses_red_green_and_black_color_evidence(self):
        color_values = {
            "red": (20, 20, 220),
            "green": (20, 180, 20),
            "black": (25, 25, 25),
        }
        corners = np.array([
            [165, 165], [335, 165], [335, 335], [165, 335],
        ], dtype=np.float64)
        for color_name, bgr in color_values.items():
            with self.subTest(color=color_name):
                image = np.full((500, 500, 3), 205, dtype=np.uint8)
                cv2.circle(image, (250, 250), 72, bgr, -1,
                           cv2.LINE_AA)
                cv2.circle(image, (250, 250), 72, (10, 10, 10), 4,
                           cv2.LINE_AA)

                settings = circle_settings(hough_param2=20)
                settings["color_fusion"]["center_mode"] = (
                    "weighted_centroid")
                match = find_nearest_circle_center(
                    image, corners, settings,
                    expected_color=color_name)

                self.assertIsNotNone(match)
                self.assertEqual(match.color_name, color_name)
                self.assertGreater(match.color_coverage, 0.5)
                self.assertTrue(match.color_fusion_applied)
                self.assertTrue(np.allclose(
                    match.center_px, [250, 250], atol=4.0), match)

    def test_expected_color_absent_falls_back_to_edge_circle(self):
        image = np.full((500, 500, 3), 205, dtype=np.uint8)
        cv2.circle(image, (250, 250), 72, (120, 120, 120), -1,
                   cv2.LINE_AA)
        cv2.circle(image, (250, 250), 72, (15, 15, 15), 5,
                   cv2.LINE_AA)
        corners = np.array([
            [165, 165], [335, 165], [335, 335], [165, 335],
        ], dtype=np.float64)

        match = find_nearest_circle_center(
            image, corners, circle_settings(hough_param2=20),
            expected_color="red")

        self.assertIsNotNone(match)
        self.assertIsNone(match.color_name)
        self.assertFalse(match.color_fusion_applied)
        self.assertTrue(np.allclose(
            match.center_px, [250, 250], atol=4.0), match)

    def test_expected_color_overrides_nearer_wrong_color_circle(self):
        image = np.full((500, 500, 3), 205, dtype=np.uint8)
        cv2.circle(image, (210, 250), 48, (20, 20, 220), -1,
                   cv2.LINE_AA)
        cv2.circle(image, (210, 250), 48, (10, 10, 10), 4,
                   cv2.LINE_AA)
        cv2.circle(image, (310, 250), 48, (20, 180, 20), -1,
                   cv2.LINE_AA)
        cv2.circle(image, (310, 250), 48, (10, 10, 10), 4,
                   cv2.LINE_AA)
        # 请求框中心x=245，更靠近红色圆；期望green时必须选择右侧绿色圆。
        corners = np.array([
            [145, 160], [345, 160], [345, 340], [145, 340],
        ], dtype=np.float64)

        match = find_nearest_circle_center(
            image, corners, circle_settings(hough_param2=20),
            expected_color="green")

        self.assertIsNotNone(match)
        self.assertEqual(match.color_name, "green")
        self.assertFalse(match.color_fusion_applied)
        self.assertTrue(np.allclose(
            match.center_px, [310, 250], atol=5.0), match)

    def test_rejects_invalid_radius_range(self):
        with self.assertRaisesRegex(ValueError, "半径比例"):
            validate_circle_refinement_settings(circle_settings(
                min_radius_ratio=0.4, max_radius_ratio=0.2))

    def test_rejects_invalid_color_center_mode(self):
        settings = circle_settings()
        settings["color_fusion"]["center_mode"] = "color_only"
        with self.assertRaisesRegex(ValueError, "center_mode"):
            validate_circle_refinement_settings(settings)


if __name__ == "__main__":
    unittest.main()
