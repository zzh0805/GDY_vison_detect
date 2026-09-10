# -*- coding: utf-8 -*-
from __future__ import annotations

import unittest

import numpy as np

from field_test.common import base_x_approach_pose_mm_rpy_deg


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


if __name__ == "__main__":
    unittest.main()
