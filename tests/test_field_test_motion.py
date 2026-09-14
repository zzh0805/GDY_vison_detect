# -*- coding: utf-8 -*-
from __future__ import annotations

import unittest

import numpy as np

from field_test.common import (base_x_approach_pose_mm_rpy_deg,
                               directional_approach_pose_mm_rpy_deg)


class FieldTestMotionTests(unittest.TestCase):
    def test_negative_offset_enters_along_positive_base_x(self):
        target = np.array([
            1215.9041, -166.0234, -96.4324,
            -95.3959, 46.1541, -5.6234,
        ])
        approach = base_x_approach_pose_mm_rpy_deg(target, -200.0)
        self.assertTrue(np.allclose(
            approach,
            [1015.9041, -166.0234, -96.4324,
             -95.3959, 46.1541, -5.6234]))
        self.assertTrue(np.array_equal(approach[1:], target[1:]))

    def test_positive_offset_reverses_base_x_direction(self):
        target = [100.0, 20.0, 30.0, 40.0, 50.0, 60.0]
        approach = base_x_approach_pose_mm_rpy_deg(target, 125.0)
        self.assertTrue(np.allclose(
            approach, [225.0, 20.0, 30.0, 40.0, 50.0, 60.0]))

    def test_auto_positive_x_matches_old_positive_x_entry(self):
        target = np.array([1000.0, 20.0, 30.0, 40.0, 50.0, 60.0])
        approach = directional_approach_pose_mm_rpy_deg(
            target, [1.0, 0.0, 0.0], 200.0)
        self.assertTrue(np.allclose(
            approach, [800.0, 20.0, 30.0, 40.0, 50.0, 60.0]))

    def test_auto_positive_and_negative_y_entries(self):
        target = np.array([100.0, 200.0, 300.0, 10.0, 20.0, 30.0])
        positive_y = directional_approach_pose_mm_rpy_deg(
            target, [0.0, 4.0, 0.0], 200.0)
        negative_y = directional_approach_pose_mm_rpy_deg(
            target, [0.0, -2.0, 0.0], 200.0)
        self.assertTrue(np.allclose(
            positive_y, [100.0, 0.0, 300.0, 10.0, 20.0, 30.0]))
        self.assertTrue(np.allclose(
            negative_y, [100.0, 400.0, 300.0, 10.0, 20.0, 30.0]))

    def test_auto_diagonal_uses_normalized_three_dimensional_direction(self):
        target = np.array([500.0, 600.0, 700.0, 90.0, -45.0, 91.0])
        approach = directional_approach_pose_mm_rpy_deg(
            target, [1.0, 1.0, 0.0], 200.0)
        delta = target[:3] - approach[:3]
        self.assertAlmostEqual(float(np.linalg.norm(delta)), 200.0)
        self.assertTrue(np.allclose(
            delta, [np.sqrt(20000.0), np.sqrt(20000.0), 0.0]))
        self.assertTrue(np.array_equal(approach[3:], target[3:]))

    def test_auto_direction_rejects_zero_vector_and_negative_distance(self):
        target = [100.0, 20.0, 30.0, 40.0, 50.0, 60.0]
        with self.assertRaises(ValueError):
            directional_approach_pose_mm_rpy_deg(
                target, [0.0, 0.0, 0.0], 200.0)
        with self.assertRaises(ValueError):
            directional_approach_pose_mm_rpy_deg(
                target, [1.0, 0.0, 0.0], -1.0)


if __name__ == "__main__":
    unittest.main()
