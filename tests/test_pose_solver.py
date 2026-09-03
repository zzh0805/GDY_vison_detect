# -*- coding: utf-8 -*-
from __future__ import annotations

import json
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from handeye_calib.approach.models import HandEyeCalibration
from vision_solver.pose_solver import (
    TargetPoseSolver, _jaka_deg_to_matrix, _matrix_to_jaka_deg,
    _standard_to_tool)


ROOT = Path(__file__).resolve().parents[1]


def _calibration(transform=None) -> HandEyeCalibration:
    return HandEyeCalibration(
        mode="eye-in-hand",
        T_tcp_camera=(
            np.eye(4, dtype=np.float64)
            if transform is None else np.asarray(transform, dtype=np.float64)),
        accepted=True,
        quality_status="PASS",
        source_path=Path("test-handeye.json"),
    )


def _observation(center_camera_m, normal_camera=(0.0, 0.0, -1.0)):
    return SimpleNamespace(
        valid=True,
        class_name="panel",
        center_camera_m=np.asarray(center_camera_m, dtype=np.float64),
        surface_normal_camera=np.asarray(normal_camera, dtype=np.float64),
        error_code=None,
        error_message=None,
    )


def _tool(offset=None):
    return {
        "tool_id": "tool-panel",
        "standard_to_tool": offset or {
            "xyz_mm": [0.0, 0.0, 0.0],
            "rpy_deg": [0.0, 0.0, 0.0],
        },
    }


class UnifiedReferencePoseTests(unittest.TestCase):
    def test_production_redkonb_zero_offset_regression(self):
        handeye = json.loads((
            ROOT / "calibration" / "handeye_result.json"
        ).read_text(encoding="utf-8"))
        source = json.loads((
            ROOT / "migration" / "source" /
            "redkonb_camera_reference_result.json"
        ).read_text(encoding="utf-8"))
        T_tcp_camera = np.asarray(handeye["T_tcp_camera"], dtype=np.float64)
        capture_tcp = np.array([
            377.08412911136236, -42.371652002115866,
            408.4365799622925, 91.38043135153563,
            -45.78843056100925, 84.08608267467879,
        ])
        T_base_camera_capture = (
            _jaka_deg_to_matrix(capture_tcp) @ T_tcp_camera)
        rotation_capture = T_base_camera_capture[:3, :3]
        center_base = np.asarray(
            source["geometry"]["targetCenterBaseMm"], dtype=np.float64
        ) * 0.001
        normal_base = np.asarray(
            source["geometry"]["surfaceNormalBase"], dtype=np.float64)
        center_camera = rotation_capture.T @ (
            center_base - T_base_camera_capture[:3, 3])
        normal_camera = rotation_capture.T @ normal_base

        solver = TargetPoseSolver(
            _calibration(T_tcp_camera),
            {
                "alignment_mode": "tool",
                "standoff_mm": 50.0,
                "tcp_correction": {
                    "position_mm": [-0.1, 11.0, 6.1],
                    "rpy_deg": [1.2, 0.3, -6.0],
                },
            },
            {"panel": _tool()},
        )
        result = solver.solve(
            _observation(center_camera, normal_camera), capture_tcp)

        expected = np.array([
            932.107129731464, -108.42511907747019,
            327.9572223954454, 90.35319929558875,
            -45.39092295177533, 94.53171212071786,
        ])
        actual = np.asarray(result["standardTcpMmRpyDeg"])
        # result.json只保留了有限精度的基座中心/法向；由这些字段反建观测
        # 后仍应在0.01mm/0.005度内复现现场验证位姿。
        self.assertLess(np.linalg.norm(actual[:3] - expected[:3]), 0.01)
        self.assertLess(np.max(np.abs(actual[3:] - expected[3:])), 0.005)
        self.assertTrue(np.allclose(
            result["targetTcpMmRpyDeg"], actual, atol=1e-10))

    def test_zero_offset_uses_inverse_handeye_then_global_correction(self):
        T_tcp_camera = _jaka_deg_to_matrix(
            np.array([60.0, -15.0, 240.0, 5.0, -8.0, 12.0]))
        correction = np.array([-0.1, 11.0, 6.1, 1.2, 0.3, -6.0])
        solver = TargetPoseSolver(
            _calibration(T_tcp_camera),
            {
                "alignment_mode": "tool",
                "standoff_mm": 50.0,
                "tcp_correction": {
                    "position_mm": correction[:3],
                    "rpy_deg": correction[3:],
                },
            },
            {"panel": _tool()},
        )

        result = solver.solve(
            _observation([0.03, -0.02, 0.8]),
            np.array([300.0, -200.0, 400.0, 10.0, -20.0, 30.0]))

        camera_reference = np.asarray(result["TBaseCameraReference"])
        expected_inverse = camera_reference @ np.linalg.inv(T_tcp_camera)
        expected_inverse_pose = _matrix_to_jaka_deg(expected_inverse)
        expected_standard_pose = expected_inverse_pose + correction
        expected_standard = _jaka_deg_to_matrix(expected_standard_pose)

        self.assertTrue(np.allclose(
            result["TBaseTcpInverseHandeye"], expected_inverse, atol=1e-10))
        self.assertTrue(np.allclose(
            result["inverseHandeyeTcpMmRpyDeg"],
            expected_inverse_pose, atol=1e-9))
        self.assertTrue(np.allclose(
            result["standardTcpMmRpyDeg"],
            expected_standard_pose, atol=1e-9))
        self.assertTrue(np.allclose(
            result["TBaseTcpStandard"], expected_standard, atol=1e-10))
        self.assertTrue(np.allclose(
            result["TBaseTcpTarget"], expected_standard, atol=1e-10))
        self.assertTrue(np.allclose(
            result["targetTcpMmRpyDeg"], expected_standard_pose, atol=1e-9))

    def test_tool_offset_is_applied_after_standard_tcp(self):
        T_tcp_camera = _jaka_deg_to_matrix(
            np.array([60.0, -15.0, 240.0, 5.0, -8.0, 12.0]))
        tool = _tool({
            "xyz_mm": [106.130627, 56.354203, -16.391591],
            "rpy_deg": [-3.005278, -4.276208, -0.141311],
        })
        solver = TargetPoseSolver(
            _calibration(T_tcp_camera),
            {"alignment_mode": "tool", "standoff_mm": 50.0},
            {"panel": tool},
        )
        result = solver.solve(
            _observation([0.03, -0.02, 0.8]),
            np.array([300.0, -200.0, 400.0, 10.0, -20.0, 30.0]))

        expected = (
            np.asarray(result["TBaseTcpStandard"]) @
            _standard_to_tool(tool))
        self.assertTrue(np.allclose(
            result["TBaseTcpTarget"], expected, atol=1e-10))
        self.assertTrue(np.allclose(
            result["TStandardToolEffective"],
            _standard_to_tool(tool), atol=1e-10))

    def test_camera_center_ignores_tool_and_uses_inverse_handeye(self):
        T_tcp_camera = _jaka_deg_to_matrix(
            np.array([80.0, 20.0, 230.0, 2.0, -3.0, 40.0]))
        solver = TargetPoseSolver(
            _calibration(T_tcp_camera),
            {"alignment_mode": "camera_center", "standoff_mm": 50.0},
            {"panel": _tool({
                "xyz_mm": [500.0, 0.0, 0.0],
                "rpy_deg": [20.0, 0.0, 0.0],
            })},
        )
        result = solver.solve(
            _observation([0.0, 0.0, 1.0]),
            np.zeros(6, dtype=np.float64))

        expected = (
            np.asarray(result["TBaseCameraReference"]) @
            np.linalg.inv(T_tcp_camera))
        self.assertTrue(np.allclose(
            result["TBaseTcpTarget"], expected, atol=1e-10))
        self.assertIsNone(result["toolId"])

    def test_same_target_is_invariant_to_translated_capture_pose(self):
        solver = TargetPoseSolver(
            _calibration(),
            {"alignment_mode": "tool", "standoff_mm": 50.0},
            {"panel": _tool({
                "xyz_mm": [100.0, 20.0, 30.0],
                "rpy_deg": [0.0, 0.0, 0.0],
            })},
        )
        first = solver.solve(
            _observation([0.0, 0.0, 1.0]),
            np.zeros(6, dtype=np.float64))
        second = solver.solve(
            _observation([-0.2, 0.1, 0.95]),
            np.array([200.0, -100.0, 50.0, 0.0, 0.0, 0.0]))

        self.assertTrue(np.allclose(
            first["targetCenterBaseMm"],
            second["targetCenterBaseMm"], atol=1e-8))
        self.assertTrue(np.allclose(
            first["standardTcpMmRpyDeg"],
            second["standardTcpMmRpyDeg"], atol=1e-8))
        self.assertTrue(np.allclose(
            first["targetTcpMmRpyDeg"],
            second["targetTcpMmRpyDeg"], atol=1e-8))


if __name__ == "__main__":
    unittest.main()
