# -*- coding: utf-8 -*-
from __future__ import annotations

import unittest

import numpy as np

from handeye_calib.approach.models import CameraModel
from handeye_calib.hardware_interfaces import CameraFrame
from vision_solver.dataset_reference import estimate_roi_reference


class DatasetReferenceTests(unittest.TestCase):
    def test_surrounding_plane_ignores_protruding_target(self):
        height, width = 100, 120
        K = np.array([[100.0, 0.0, 60.0],
                      [0.0, 100.0, 50.0],
                      [0.0, 0.0, 1.0]])
        yy, xx = np.mgrid[:height, :width]
        z = np.full((height, width), 500.0)
        x = (xx - K[0, 2]) * z / K[0, 0]
        y = (yy - K[1, 2]) * z / K[1, 1]
        cloud = np.stack([x, y, z], axis=-1)
        # ROI内模拟一个向相机凸出50mm的按钮；外围拟合不得被它带偏。
        z[30:71, 40:81] = 450.0
        cloud[30:71, 40:81, 0] = (
            xx[30:71, 40:81] - K[0, 2]) * 450.0 / K[0, 0]
        cloud[30:71, 40:81, 1] = (
            yy[30:71, 40:81] - K[1, 2]) * 450.0 / K[1, 1]
        cloud[30:71, 40:81, 2] = 450.0
        frame = CameraFrame(
            frame_id=1,
            color=np.zeros((height, width, 3), dtype=np.uint8),
            point_cloud=cloud,
            metadata={
                "point_cloud_unit": "mm",
                "point_cloud_pixel_aligned_to_image": True,
                "point_cloud_handeye_compatible": True,
            })
        model = CameraModel(
            K=K, distortion=np.zeros(5), width=width, height=height,
            image_frame="color", profile="test", intrinsics_source="test")
        result = estimate_roi_reference(
            frame, model, [[40, 30], [80, 30], [80, 70], [40, 70]], {
                "plane_expand_ratio": 0.5,
                "exclude_roi_margin_ratio": 0.0,
                "min_plane_points": 100,
                "max_plane_sample_points": 5000,
                "ransac_threshold_mm": 1.0,
                "ransac_iterations": 100,
                "min_inlier_ratio": 0.9,
                "max_plane_rms_mm": 0.1,
            })
        self.assertTrue(np.allclose(
            result["targetCameraMm"], [0, 0, 500], atol=1e-6))
        self.assertAlmostEqual(result["quality"]["planeRmsMm"], 0.0, places=6)
        self.assertGreater(result["quality"]["planeInlierRatio"], 0.99)


if __name__ == "__main__":
    unittest.main()
