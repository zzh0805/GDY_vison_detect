# -*- coding: utf-8 -*-
from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import numpy as np
import yaml

from calibrate_tool_offset import _matrix_to_pose, _pose_to_matrix
from tool.interactive_tool_calibration import (
    calculate_reference_adjustment, run_interactive_calibration)


class _Inputs:
    def __init__(self, values):
        self.values = iter(values)

    def __call__(self, _prompt):
        return next(self.values)


class _Poses:
    def __init__(self, values):
        self.values = iter(values)

    def __call__(self):
        return np.asarray(next(self.values), dtype=np.float64)


class InteractiveToolCalibrationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.config = self.root / "interactive.yaml"
        self.offsets = self.root / "tool_offsets.yaml"
        self.backup = self.root / "tool_offsets.backup.yaml"
        self.result = self.root / "result.yaml"
        self.offset = np.array([
            116.8298, 12.1000, -28.7966,
            -5.9920, -0.4126, -0.1006,
        ])
        self.standard = np.array([
            959.9279, -170.8918, 430.8637,
            90.0636, -45.0013, 91.0221,
        ])
        self.work = _matrix_to_pose(
            _pose_to_matrix(self.standard) @ _pose_to_matrix(self.offset))
        self.offsets.write_text(yaml.safe_dump({
            "version": 1,
            "target_selection": {
                "use_yolo": False,
                "circle_refinement_enabled": True,
            },
            "tools": {
                "greenbtn": {
                    "code": "9-8-1",
                    "target_color": "green",
                    "tool_id": "tool_green_button",
                    "enabled": True,
                    "standard_to_tool": {
                        "xyz_mm": self.offset[:3].tolist(),
                        "rpy_deg": self.offset[3:].tolist(),
                    },
                },
            },
        }, allow_unicode=True, sort_keys=False), encoding="utf-8")
        self.config.write_text(yaml.safe_dump({
            "version": 1,
            "robot": {"ip": "fake"},
            "files": {
                "tool_offsets_file": str(self.offsets),
                "backup_file": str(self.backup),
                "result_file": str(self.result),
            },
        }, allow_unicode=True, sort_keys=False), encoding="utf-8")

    def tearDown(self):
        self.temp.cleanup()

    def _saved_offset(self):
        data = yaml.safe_load(self.offsets.read_text(encoding="utf-8"))
        transform = data["tools"]["greenbtn"]["standard_to_tool"]
        return np.asarray(
            transform["xyz_mm"] + transform["rpy_deg"],
            dtype=np.float64)

    def test_mode1_runs_in_one_process_and_updates_selected_code(self):
        original = self.offsets.read_bytes()
        outcome = run_interactive_calibration(
            self.config,
            input_fn=_Inputs([
                "9-8-1", "1", "", "", "", "",
            ]),
            print_fn=lambda _text: None,
            pose_reader=_Poses([self.standard, self.work]),
        )

        self.assertEqual(outcome["mode"], 1)
        self.assertEqual(outcome["workpiece"]["class_name"], "greenbtn")
        self.assertTrue(self.backup.is_file())
        self.assertEqual(self.backup.read_bytes(), original)
        self.assertTrue(self.result.is_file())
        self.assertTrue(np.allclose(
            _pose_to_matrix(self.standard) @
            _pose_to_matrix(self._saved_offset()),
            _pose_to_matrix(self.work), atol=2e-8))

    def test_mode2_corrects_existing_offset_and_keeps_one_backup(self):
        new_work = self.work + np.array([1.5, -2.0, 0.5, 0, 0, 0])
        run_interactive_calibration(
            self.config,
            input_fn=_Inputs([
                "9-8-1", "2", "", "", "", "",
            ]),
            print_fn=lambda _text: None,
            pose_reader=_Poses([self.work, new_work]),
        )

        self.assertEqual(len(list(self.root.glob("tool_offsets.backup*"))), 1)
        self.assertTrue(np.allclose(
            _pose_to_matrix(self.standard) @
            _pose_to_matrix(self._saved_offset()),
            _pose_to_matrix(new_work), atol=2e-8))

    def test_mode3_defaults_to_reference_frame_and_hot_writes_offset(self):
        outcome = run_interactive_calibration(
            self.config,
            input_fn=_Inputs([
                "9-8-1", "3", "", "", "", "0,-1,0", "",
            ]),
            print_fn=lambda _text: None,
            pose_reader=_Poses([self.work]),
        )

        self.assertEqual(
            outcome["detail"]["adjustment_frame"],
            "standard_camera_reference")
        self.assertTrue(np.allclose(
            self._saved_offset()[:3], self.offset[:3] + [0, -1, 0],
            atol=1e-6))
        self.assertEqual(
            outcome["verification"]["position_error_mm"], 0.0)

    def test_reference_adjustment_is_invariant_to_base_orientation(self):
        first_work = self.work
        change_of_base = _pose_to_matrix([
            400.0, -200.0, 100.0, 0.0, 0.0, 90.0])
        second_standard_matrix = (
            change_of_base @ _pose_to_matrix(self.standard))
        second_work = _matrix_to_pose(
            second_standard_matrix @ _pose_to_matrix(self.offset))

        first_offset, first_expected = calculate_reference_adjustment(
            self.offset, first_work, [0, -1, 2], [0, 0, 0.2])
        second_offset, second_expected = calculate_reference_adjustment(
            self.offset, second_work, [0, -1, 2], [0, 0, 0.2])

        self.assertTrue(np.allclose(
            _pose_to_matrix(first_offset),
            _pose_to_matrix(second_offset), atol=1e-10))
        self.assertTrue(np.allclose(
            np.linalg.inv(_pose_to_matrix(self.standard)) @
            _pose_to_matrix(first_expected),
            np.linalg.inv(second_standard_matrix) @
            _pose_to_matrix(second_expected), atol=1e-10))


if __name__ == "__main__":
    unittest.main()
