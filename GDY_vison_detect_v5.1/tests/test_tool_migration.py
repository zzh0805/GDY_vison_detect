# -*- coding: utf-8 -*-
from __future__ import annotations

import json
import unittest
from pathlib import Path

import numpy as np

from calibrate_tool_offset import _matrix_to_pose, _pose_to_matrix
from migrate_legacy_tool_offset import migrate


ROOT = Path(__file__).resolve().parents[1]


class LegacyToolMigrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.record = json.loads((
            ROOT / "migration" / "legacy_tool_migration.json"
        ).read_text(encoding="utf-8"))
        cls.handeye = np.asarray(json.loads((
            ROOT / "calibration" / "handeye_result.json"
        ).read_text(encoding="utf-8"))["T_tcp_camera"], dtype=np.float64)
        cls.correction = np.asarray(
            cls.record["global_tcp_correction_mm_rpy_deg"], dtype=np.float64)

    @staticmethod
    def _reference(name: str) -> tuple[dict, np.ndarray]:
        data = json.loads((
            ROOT / "migration" / "source" / name
        ).read_text(encoding="utf-8"))
        reference = np.asarray(
            data["cameraReference"]["TBaseCameraReference"],
            dtype=np.float64)
        return data, reference

    def test_anchor_reproduces_saved_taught_tcp(self):
        green = self.record["greenbtn"]
        _source, reference = self._reference(
            "greenbtn_reference_result.json")
        migrated = migrate(
            reference, self.handeye, self.correction,
            taught_tcp=np.asarray(
                green["saved_taught_tcp_mm_rpy_deg"], dtype=np.float64))

        self.assertTrue(np.allclose(
            migrated["standard_to_tool_mm_rpy_deg"],
            green["standard_to_tool_mm_rpy_deg"], atol=1e-9))
        self.assertLess(migrated["position_residual_mm"], 1e-8)

    def test_rounded_anchor_offset_preserves_second_v1_result(self):
        green = self.record["greenbtn"]
        source, reference = self._reference(
            "greenbtn_crosscheck_result.json")
        legacy = np.asarray(
            green["legacy_camera_to_tool_mm_rpy_deg"], dtype=np.float64)
        second = migrate(
            reference, self.handeye, self.correction,
            legacy_camera_to_tool=legacy)
        T_standard = _pose_to_matrix(
            second["standard_tcp_mm_rpy_deg"])
        T_offset = _pose_to_matrix(np.asarray(
            green["configured_rounded_standard_to_tool_mm_rpy_deg"],
            dtype=np.float64))
        predicted = _matrix_to_pose(T_standard @ T_offset)
        old_target = np.asarray(source["targetTcpMmRpyDeg"], dtype=np.float64)

        position_error = float(np.linalg.norm(
            predicted[:3] - old_target[:3]))
        max_rpy_component_error = float(np.max(np.abs(
            predicted[3:] - old_target[3:])))
        self.assertLess(position_error, 0.016)
        self.assertLess(max_rpy_component_error, 0.0001)


if __name__ == "__main__":
    unittest.main()
