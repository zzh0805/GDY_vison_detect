# -*- coding: utf-8 -*-
from __future__ import annotations

import unittest

import numpy as np

from field_test.common import (base_x_approach_pose_mm_rpy_deg,
                               directional_approach_pose_mm_rpy_deg,
                               field_test_motion_waypoints,
                               resolve_base_label,
                               resolve_live_capture,
                               resolve_return_to_capture_pose)


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

    def test_return_to_capture_defaults_true_and_can_be_disabled(self):
        self.assertTrue(resolve_return_to_capture_pose({}))
        self.assertTrue(resolve_return_to_capture_pose(
            {"return_to_capture_pose": True}))
        self.assertFalse(resolve_return_to_capture_pose(
            {"return_to_capture_pose": False}))

    def test_return_to_capture_command_line_override_wins(self):
        self.assertFalse(resolve_return_to_capture_pose(
            {"return_to_capture_pose": True}, False))
        self.assertTrue(resolve_return_to_capture_pose(
            {"return_to_capture_pose": False}, True))

    def test_return_to_capture_rejects_yaml_strings(self):
        with self.assertRaisesRegex(ValueError, "true或false"):
            resolve_return_to_capture_pose(
                {"return_to_capture_pose": "false"})

    def test_live_and_base_label_are_read_from_case_yaml(self):
        case = {"live": True, "base_label": 5}
        self.assertTrue(resolve_live_capture(case))
        self.assertEqual(resolve_base_label(case), "5")

    def test_command_line_can_override_live_and_base_label(self):
        case = {"live": True, "base_label": 5}
        self.assertFalse(resolve_live_capture(case, False))
        self.assertEqual(resolve_base_label(case, "7"), "7")
        self.assertIsNone(resolve_base_label(case, "0"))

    def test_live_and_base_label_defaults_keep_legacy_behavior(self):
        self.assertFalse(resolve_live_capture({}))
        self.assertIsNone(resolve_base_label({}))
        with self.assertRaisesRegex(ValueError, "true或false"):
            resolve_live_capture({"live": "true"})
        with self.assertRaisesRegex(ValueError, "LabelMe标签"):
            resolve_base_label({"base_label": True})

    def test_disabled_return_stops_exactly_at_work_pose(self):
        approach = np.array([1, 2, 3, 4, 5, 6], dtype=float)
        work = np.array([11, 12, 13, 14, 15, 16], dtype=float)
        capture = np.array([21, 22, 23, 24, 25, 26], dtype=float)
        waypoints = field_test_motion_waypoints(
            approach, work, capture, False)
        self.assertEqual(
            [name for name, _pose in waypoints],
            ["field_test_work_approach_pose", "field_test_work_pose"])
        self.assertTrue(np.array_equal(waypoints[-1][1], work))

    def test_enabled_return_keeps_original_four_waypoint_path(self):
        approach = np.array([1, 2, 3, 4, 5, 6], dtype=float)
        work = np.array([11, 12, 13, 14, 15, 16], dtype=float)
        capture = np.array([21, 22, 23, 24, 25, 26], dtype=float)
        waypoints = field_test_motion_waypoints(
            approach, work, capture, True)
        self.assertEqual(
            [name for name, _pose in waypoints], [
                "field_test_work_approach_pose",
                "field_test_work_pose",
                "field_test_work_retreat_pose",
                "field_test_return_capture_pose",
            ])
        self.assertTrue(np.array_equal(waypoints[-1][1], capture))


if __name__ == "__main__":
    unittest.main()
