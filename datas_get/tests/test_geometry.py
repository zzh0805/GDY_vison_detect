# -*- coding: utf-8 -*-
from __future__ import annotations

import unittest
from pathlib import Path

import numpy as np

from datas_get.config import load_collection_config
from datas_get.geometry import (build_routes, load_handeye_matrix,
                                load_scaled_rgb_intrinsics,
                                make_reference_geometry)


ROOT = Path(__file__).resolve().parents[1]


class GeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_collection_config(ROOT / "config.yaml")
        reference = cls.config.reference
        cls.handeye = load_handeye_matrix(cls.config.resolve_path(
            reference["handeye_file"], must_exist=True))
        cls.intrinsics = load_scaled_rgb_intrinsics(
            cls.config.resolve_path(
                reference["camera_calibration_file"], must_exist=True),
            int(reference["image_width"]), int(reference["image_height"]))
        cls.geometry = make_reference_geometry(
            reference["simulation_tcp_mm_rpy_deg"], cls.handeye,
            cls.intrinsics, reference["roi_xyxy_px"],
            reference["target_depth_mm"])

    def test_intrinsics_are_scaled_to_output_image(self):
        self.assertEqual(self.intrinsics.width, 1920)
        self.assertEqual(self.intrinsics.height, 1080)
        self.assertAlmostEqual(self.intrinsics.K[0, 0], 2030.785888671875)
        self.assertAlmostEqual(self.intrinsics.K[1, 1], 2030.4217529296875)

    def test_default_routes_cover_sides_diagonals_and_distance(self):
        trajectory = self.config.trajectory
        plans = build_routes(
            self.geometry, trajectory["views"],
            trajectory["samples_per_path"], trajectory["image_margin_px"],
            trajectory["max_tcp_translation_mm"],
            trajectory["max_tcp_rotation_deg"], True,
            trajectory["max_position_step_mm"],
            trajectory["max_look_angle_step_deg"],
            trajectory["max_samples_per_path"],
            trajectory["max_view_incidence_deg"])
        self.assertEqual([item.name for item in plans], [
            "left", "right", "upper", "lower", "upper_left",
            "upper_right", "lower_left", "lower_right", "nearer", "farther"])
        self.assertTrue(all(item.accepted for item in plans),
                        [item.rejection_reason for item in plans])
        self.assertTrue(all(len(item.waypoints) >= 8 for item in plans))
        for route in plans:
            for first, second in zip(route.waypoints, route.waypoints[1:]):
                distance_mm = np.linalg.norm(
                    first.T_base_camera[:3, 3] -
                    second.T_base_camera[:3, 3]) * 1000.0
                self.assertLessEqual(distance_mm, 12.0 + 1e-9)

    def test_every_camera_pose_points_at_same_target_and_roundtrips_handeye(self):
        trajectory = self.config.trajectory
        plans = build_routes(
            self.geometry, trajectory["views"], 5, 0, 300, 40, False)
        for route in plans:
            self.assertTrue(route.accepted, route.rejection_reason)
            for point in route.waypoints:
                optical_axis = point.T_base_camera[:3, 2]
                direction = self.geometry.target_base_m - point.T_base_camera[:3, 3]
                direction /= np.linalg.norm(direction)
                self.assertGreater(float(np.dot(optical_axis, direction)), 1.0 - 1e-10)
                self.assertTrue(np.allclose(
                    point.T_base_tcp @ self.handeye,
                    point.T_base_camera, atol=1e-10))
                self.assertTrue(point.roi_visible)

    def test_each_route_ends_at_exact_reference_pose(self):
        trajectory = self.config.trajectory
        plans = build_routes(
            self.geometry, trajectory["views"], 5, 0, 300, 40, True)
        for route in plans:
            final = route.waypoints[-1]
            self.assertTrue(np.allclose(
                final.T_base_camera, self.geometry.T_base_camera, atol=1e-10))
            self.assertTrue(np.allclose(
                final.T_base_tcp, self.geometry.T_base_tcp, atol=1e-10))

    def test_impossible_view_is_rejected_before_motion(self):
        plans = build_routes(
            self.geometry, [{
                "name": "impossible", "azimuth_deg": 75,
                "elevation_deg": 55, "distance_offset_mm": -450}],
            5, 100, 100, 10, True)
        self.assertFalse(plans[0].accepted)
        self.assertTrue(plans[0].rejection_reason)

    def test_dense_route_has_continuous_jaka_euler_angles(self):
        plan = build_routes(
            self.geometry, [{
                "name": "dense", "azimuth_deg": 12,
                "elevation_deg": 10, "distance_offset_mm": 20}],
            3, 0, 300, 40, False, 8.0, 1.0, 100, 35.0)[0]
        self.assertTrue(plan.accepted, plan.rejection_reason)
        poses = np.asarray([point.tcp_mm_rpy_deg for point in plan.waypoints])
        self.assertLess(float(np.max(np.abs(np.diff(poses[:, 3:], axis=0)))), 20.0)


if __name__ == "__main__":
    unittest.main()
