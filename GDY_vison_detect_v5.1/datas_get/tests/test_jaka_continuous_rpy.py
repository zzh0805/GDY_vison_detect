# -*- coding: utf-8 -*-
from __future__ import annotations

import unittest

import numpy as np

from handeye_calib.jaka_adapter import JAKARobotAdapter


class _FakeJaka:
    def __init__(self):
        self.target = None

    def linear_move_extend(self, target, *_args):
        self.target = target
        return 0


class JakaContinuousRpyTests(unittest.TestCase):
    def test_direct_jaka_pose_does_not_fold_angle_back_to_180(self):
        adapter = JAKARobotAdapter.__new__(JAKARobotAdapter)
        adapter.enable_motion = True
        adapter.robot = _FakeJaka()
        adapter._logged_in = True
        adapter.speed = 0.03
        adapter.accel = 0.1
        adapter.settle = 0.0
        adapter.move_to_mm_rpy_deg(
            np.array([1, 2, 3, 10, -20, 190.0]), "continuous")
        self.assertIsNotNone(adapter.robot.target)
        self.assertAlmostEqual(
            adapter.robot.target[5], np.radians(190.0), places=12)


if __name__ == "__main__":
    unittest.main()
