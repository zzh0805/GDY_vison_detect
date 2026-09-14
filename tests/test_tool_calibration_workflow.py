# -*- coding: utf-8 -*-
from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import numpy as np
import yaml

from calibrate_tool_offset import _matrix_to_pose, _pose_to_matrix
from tool.tool_calibration_workflow import (
    calculate_adjustment, calculate_correction, calculate_from_zero,
    execute_workflow)


class ToolCalibrationMathTests(unittest.TestCase):
    def setUp(self):
        self.standard = np.array([
            959.9279, -170.8918, 430.8637,
            90.0636, -45.0013, 91.0221,
        ])
        self.offset = np.array([
            116.8298, 12.1000, -28.7966,
            -5.9920, -0.4126, -0.1006,
        ])
        self.old_work = _matrix_to_pose(
            _pose_to_matrix(self.standard) @ _pose_to_matrix(self.offset))

    def test_mode1_from_zero_reproduces_taught_pose(self):
        calculated = calculate_from_zero(self.standard, self.old_work)
        predicted = _pose_to_matrix(self.standard) @ _pose_to_matrix(calculated)
        self.assertTrue(np.allclose(
            predicted, _pose_to_matrix(self.old_work), atol=1e-10))

    def test_mode2_correction_preserves_rigid_transform_math(self):
        new_work = self.old_work + np.array([2.0, -3.0, 1.0, 0.2, 0.0, 0.0])
        calculated = calculate_correction(
            self.offset, self.old_work, new_work)
        predicted = _pose_to_matrix(self.standard) @ _pose_to_matrix(calculated)
        self.assertTrue(np.allclose(
            predicted, _pose_to_matrix(new_work), atol=1e-10))

    def test_mode3_adjusts_in_jaka_base_components(self):
        calculated, adjusted_work = calculate_adjustment(
            self.offset, self.old_work,
            [2.0, -3.0, 1.0], [0.2, 0.0, -0.1])
        self.assertTrue(np.allclose(
            adjusted_work,
            self.old_work + [2.0, -3.0, 1.0, 0.2, 0.0, -0.1]))
        predicted = _pose_to_matrix(self.standard) @ _pose_to_matrix(calculated)
        self.assertTrue(np.allclose(
            predicted, _pose_to_matrix(adjusted_work), atol=1e-10))


class ToolCalibrationWorkflowTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.config_path = self.root / "tool_calibration.yaml"
        self.record_path = self.root / "record.yaml"
        self.offsets_path = self.root / "tool_offsets.yaml"
        self.backup_dir = self.root / "backups"
        self.standard = [
            959.9279, -170.8918, 430.8637,
            90.0636, -45.0013, 91.0221,
        ]
        self.taught = [
            941.9317, -57.7485, 521.9384,
            90.0650, -45.0017, 91.0193,
        ]
        self.offsets_path.write_text(yaml.safe_dump({
            "version": 1,
            "target_selection": {"use_yolo": True},
            "tools": {
                "panel": {
                    "code": "9-8-1",
                    "target_color": "green",
                    "tool_id": "tool_panel",
                    "enabled": True,
                    "standard_to_tool": {
                        "xyz_mm": [0.0, 0.0, 0.0],
                        "rpy_deg": [0.0, 0.0, 0.0],
                    },
                },
            },
        }, allow_unicode=True, sort_keys=False), encoding="utf-8")
        self.write_config(mode=1, step=1)

    def tearDown(self):
        self.temp.cleanup()

    def write_config(self, *, mode: int, step: int,
                     apply: bool = False,
                     xyz=(0.0, 0.0, 0.0),
                     rpy=(0.0, 0.0, 0.0)) -> None:
        self.config_path.write_text(yaml.safe_dump({
            "version": 1,
            "operation": {"mode": mode, "step": step},
            "workpiece": {
                "class_name": "panel",
                "tool_id": "tool_panel",
                "code": "9-8-1",
                "target_color": "green",
            },
            "robot": {"ip": "127.0.0.1"},
            "adjustment": {"xyz_mm": list(xyz), "rpy_deg": list(rpy)},
            "files": {
                "record_file": str(self.record_path),
                "tool_offsets_file": str(self.offsets_path),
            },
            "output": {
                "apply_to_tool_offsets": apply,
                "backup_directory": str(self.backup_dir),
            },
        }, allow_unicode=True, sort_keys=False), encoding="utf-8")

    def test_mode1_three_steps_record_calculate_and_apply(self):
        first = execute_workflow(self.config_path, self.standard)
        self.assertEqual(first["field"], "standard_tcp_mm_rpy_deg")

        self.write_config(mode=1, step=2)
        second = execute_workflow(self.config_path, self.taught)
        self.assertEqual(second["field"], "taught_tcp_mm_rpy_deg")

        self.write_config(mode=1, step=3, apply=True)
        final = execute_workflow(self.config_path)
        self.assertEqual(final["action"], "calculated")
        self.assertTrue(final["result"]["applied_to_tool_offsets"])
        self.assertLess(
            final["result"]["verification"]["position_error_mm"], 1e-8)
        updated = yaml.safe_load(self.offsets_path.read_text(encoding="utf-8"))
        offset = updated["tools"]["panel"]["standard_to_tool"]
        self.assertFalse(np.allclose(offset["xyz_mm"], [0.0, 0.0, 0.0]))
        self.assertEqual(len(list(self.backup_dir.glob("*.yaml"))), 1)
        record = yaml.safe_load(self.record_path.read_text(encoding="utf-8"))
        self.assertIn("last_result", record)

    def test_mode3_step2_calculates_without_second_robot_read(self):
        current_offset = [10.0, 20.0, 30.0, 1.0, 2.0, 3.0]
        data = yaml.safe_load(self.offsets_path.read_text(encoding="utf-8"))
        data["tools"]["panel"]["standard_to_tool"] = {
            "xyz_mm": current_offset[:3], "rpy_deg": current_offset[3:]}
        self.offsets_path.write_text(
            yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
        self.write_config(mode=3, step=1)
        execute_workflow(self.config_path, self.taught)

        self.write_config(
            mode=3, step=2, xyz=(2.0, -1.0, 0.5), rpy=(0.0, 0.0, 0.2))
        outcome = execute_workflow(self.config_path)
        expected = np.asarray(self.taught) + [2, -1, 0.5, 0, 0, 0.2]
        self.assertTrue(np.allclose(
            outcome["result"]["expected_work_tcp_mm_rpy_deg"], expected))
        self.assertLess(
            outcome["result"]["verification"]["position_error_mm"], 1e-8)

    def test_mode2_three_steps_correct_existing_offset(self):
        current_offset = np.array([10.0, 20.0, 30.0, 1.0, 2.0, 3.0])
        data = yaml.safe_load(self.offsets_path.read_text(encoding="utf-8"))
        data["tools"]["panel"]["standard_to_tool"] = {
            "xyz_mm": current_offset[:3].tolist(),
            "rpy_deg": current_offset[3:].tolist(),
        }
        self.offsets_path.write_text(
            yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
        old_work = _matrix_to_pose(
            _pose_to_matrix(self.standard) @ _pose_to_matrix(current_offset))
        new_work = old_work + np.array([1.5, -2.0, 0.5, 0.1, 0.0, 0.0])

        self.write_config(mode=2, step=1)
        execute_workflow(self.config_path, old_work)
        self.write_config(mode=2, step=2)
        execute_workflow(self.config_path, new_work)
        self.write_config(mode=2, step=3)
        outcome = execute_workflow(self.config_path)

        calculated = np.concatenate([
            outcome["result"]["standard_to_tool"]["xyz_mm"],
            outcome["result"]["standard_to_tool"]["rpy_deg"],
        ])
        predicted = (
            _pose_to_matrix(self.standard) @ _pose_to_matrix(calculated))
        self.assertTrue(np.allclose(
            predicted, _pose_to_matrix(new_work), atol=2e-8))
        self.assertFalse(outcome["result"]["applied_to_tool_offsets"])

    def test_new_step1_clears_stale_following_pose(self):
        execute_workflow(self.config_path, self.standard)
        self.write_config(mode=1, step=2)
        execute_workflow(self.config_path, self.taught)

        self.write_config(mode=1, step=1)
        execute_workflow(self.config_path, np.asarray(self.standard) + 1.0)
        self.write_config(mode=1, step=3)
        with self.assertRaisesRegex(ValueError, "step 1和step 2"):
            execute_workflow(self.config_path)


if __name__ == "__main__":
    unittest.main()
