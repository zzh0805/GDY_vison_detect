# -*- coding: utf-8 -*-
from __future__ import annotations

import json
import tempfile
import threading
import unittest
from contextlib import contextmanager
from pathlib import Path

import numpy as np
import yaml

from handeye_calib.hardware_interfaces import CameraFrame
from handeye_calib.approach.yolo_detector import YoloTargetDetector
from vision_tool_tcp_api import VisionToolTcpSolver
from vision_solver.config import load_tool_offsets
from vision_solver.http_client import VisionHttpClient
from vision_solver.http_protocol import VisionHttpProtocol
from vision_solver.http_server import VisionHttpServer


PROJECT_ROOT = Path(__file__).resolve().parents[1]


class FakeBoxes:
    xywh = np.array([
        [50.0, 50.0, 40.0, 40.0],
        [20.0, 20.0, 10.0, 10.0],
    ], dtype=np.float32)
    xyxy = np.array([
        [30.0, 30.0, 70.0, 70.0],
        [15.0, 15.0, 25.0, 25.0],
    ], dtype=np.float32)
    conf = np.array([0.95, 0.80], dtype=np.float32)
    cls = np.array([0.0, 1.0], dtype=np.float32)


class FakeResult:
    boxes = FakeBoxes()
    names = {0: "panel", 1: "other"}


class FakeModel:
    names = {0: "panel", 1: "other"}

    def __init__(self):
        self.predict_calls = 0

    def predict(self, **_kwargs):
        self.predict_calls += 1
        return [FakeResult()]


class FakeCamera:
    def __init__(self):
        self.connected = False
        self.frame_id = 0
        self.discard_durations = []
        self.K = np.array([
            [100.0, 0.0, 50.0],
            [0.0, 100.0, 50.0],
            [0.0, 0.0, 1.0],
        ])

    def connect(self, _endpoint=None):
        self.connected = True

    def disconnect(self):
        self.connected = False

    def is_connected(self):
        return self.connected

    def get_intrinsics(self):
        return self.K.copy(), np.zeros((1, 5), dtype=np.float64)

    def _frame(self, with_cloud: bool) -> CameraFrame:
        height = width = 100
        color = np.zeros((height, width, 3), dtype=np.uint8)
        color[:, :, 1] = 128
        cloud = None
        if with_cloud:
            u, v = np.meshgrid(np.arange(width), np.arange(height))
            z = np.full((height, width), 1000.0, dtype=np.float32)
            # 选中目标内部深度全部丢失，只允许外围平面解算。
            z[30:71, 30:71] = np.nan
            x = (u.astype(np.float32) - 50.0) * z / 100.0
            y = (v.astype(np.float32) - 50.0) * z / 100.0
            cloud = np.stack([x, y, z], axis=-1)
        frame = CameraFrame(
            frame_id=self.frame_id,
            color=color,
            point_cloud=cloud,
            camera_frame="surfacepro50_color_optical",
            metadata={
                "point_cloud_unit": "mm",
                "point_cloud_handeye_compatible": True,
                "point_cloud_pixel_aligned_to_image": True,
                "calibration_camera_frame": "surfacepro50_color_optical",
            })
        self.frame_id += 1
        return frame

    def capture(self):
        return self._frame(with_cloud=True)

    def capture_3d(self):
        return self._frame(with_cloud=True)

    def capture_color(self):
        return self._frame(with_cloud=False)

    def discard_frames(self, duration_s):
        self.discard_durations.append(float(duration_s))
        return 3


def fake_detector(model=None) -> YoloTargetDetector:
    return YoloTargetDetector(
        "unused.pt", model=model or FakeModel(),
        confidence=0.5,
        min_plane_points=50,
        max_plane_sample_points=5000,
        plane_distance_threshold_mm=1.0,
        plane_min_inlier_ratio=0.5,
        plane_region_mode="surrounding_panel",
        surrounding_expand_ratio=0.5,
        surrounding_exclude_margin_ratio=0.0,
        require_handeye_compatible_cloud=True)


class SimulatedWorkflowTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        handeye = self.root / "hand_eye_result.json"
        handeye.write_text(json.dumps({
            "mode": "eye-in-hand",
            "T_tcp_camera": np.eye(4).tolist(),
            "quality": {"accepted": True, "status": "PASS"},
        }), encoding="utf-8")
        self.config_path = self.root / "workflow.yaml"
        self.tool_offsets_path = self.root / "tool_offsets.yaml"
        self.write_config("camera_center")

    def tearDown(self):
        self.temp.cleanup()

    def write_tool_offsets(self, x_offset_mm: float = 100.0,
                           use_yolo: bool = True,
                           selected_tool: str = "panel") -> None:
        self.tool_offsets_path.write_text(
            yaml.safe_dump({
                "version": 1,
                "target_selection": {
                    "use_yolo": use_yolo,
                    "selected_tool": selected_tool,
                },
                "tools": {
                    "panel": {
                        "tool_id": "tool-panel",
                        "enabled": True,
                        "standard_to_tool": {
                            "xyz_mm": [x_offset_mm, 0.0, 0.0],
                            "rpy_deg": [0.0, 0.0, 0.0],
                        },
                    },
                },
            }, allow_unicode=True, sort_keys=False),
            encoding="utf-8")

    def write_config(self, alignment_mode: str,
                     live_capture_delay_s: float = 0.0,
                     snapshot_delay_s: float = 0.0,
                     use_yolo: bool = True,
                     selected_tool: str = "panel") -> None:
        self.write_tool_offsets(
            use_yolo=use_yolo, selected_tool=selected_tool)
        data = {
            "system": {
                "initialization_timeout_s": 5,
                "task_timeout_s": 5,
                "output_directory": str(self.root / "output"),
                "save_report": False,
                "save_debug_image": False,
                "save_point_cloud": False,
            },
            "camera": {
                "ip": "fake",
                "calibration_file": str(
                    PROJECT_ROOT / "calibration" /
                    "chishine_192_168_16_122_calibration.yml"),
            },
            "http": {
                "listen_host": "127.0.0.1",
                "listen_port": 40051,
                "request_timeout_s": 5,
                "snapshot_cache_ttl_s": 30,
                "snapshot_directory": str(self.root / "http_images"),
                "snapshot_delay_s": snapshot_delay_s,
                "live_capture_delay_s": live_capture_delay_s,
                "jpeg_quality": 95,
            },
            "calibration": {
                "handeye_result_file": str(self.root / "hand_eye_result.json"),
                "require_accepted": True,
            },
            "yolo": {
                "model_file": str(PROJECT_ROOT / "models" / "xuncao.pt"),
                "load_model_at_start": False,
                "class_names": [],
            },
            "target_matching": {
                "source_image_width": 100,
                "source_image_height": 100,
                "prefer_center_inside_polygon": True,
                "max_match_distance_px": 30,
            },
            "plane_fitting": {
                "surrounding_expand_ratio": 0.5,
                "exclude_box_margin_ratio": 0.0,
                "min_points": 50,
                "ransac_threshold_mm": 1.0,
                "min_inlier_ratio": 0.5,
                "max_rms_mm": 1.0,
            },
            "pose": {
                "pipeline_version": 2,
                "alignment_mode": alignment_mode,
                "standoff_mm": 50.0,
                "output_reference_frame": "jaka_base",
                "output_pose_reference": "active_tcp",
            },
            "tool_offsets": {
                "file": str(self.tool_offsets_path),
            },
            "annotation_capture": {
                "root_directory": str(self.root / "images"),
                "image_format": "png",
                "save_metadata_json": True,
            },
        }
        self.config_path.write_text(
            yaml.safe_dump(data, allow_unicode=True, sort_keys=False),
            encoding="utf-8")

    @contextmanager
    def http_service(self, alignment_mode="camera_center",
                     live_capture_delay_s=0.0,
                     snapshot_delay_s=0.0,
                     use_yolo=True,
                     selected_tool="panel"):
        self.write_config(
            alignment_mode, live_capture_delay_s, snapshot_delay_s,
            use_yolo, selected_tool)
        camera = FakeCamera()
        model = FakeModel()
        camera.yolo_model = model
        solver = VisionToolTcpSolver(
            self.config_path, camera=camera, detector=fake_detector(model))
        server = VisionHttpServer(
            VisionHttpProtocol(solver), "127.0.0.1", 0)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        host, port = server.server_address
        client = VisionHttpClient(f"http://{host}:{port}", timeout_s=5)
        try:
            yield client, camera
        finally:
            server.stop()
            thread.join(timeout=2.0)
            solver.close()

    def test_snapshot_then_camera_center_solve_through_http(self):
        with self.http_service() as (client, camera):
            captured = client.snapshot()
            solved = client.get_tcp_pose(
                [0, 0, 0, 0, 0, 0], 40, 40, 60, 60)
        self.assertEqual(set(captured), {"code", "path"})
        self.assertEqual(captured["code"], 200, captured)
        image_path = Path(captured["path"])
        self.assertTrue(image_path.is_file())
        self.assertRegex(image_path.name, r"^\d{14}\.jpg$")
        self.assertEqual(set(solved), {"code", "pos"})
        self.assertEqual(solved["code"], 200, solved)
        self.assertTrue(np.allclose(
            solved["pos"], [0, 0, 950, 0, 0, 0],
            atol=1e-5))
        # /get_tcp_pose复用/snapshot缓存，不应再触发第二次相机采集。
        self.assertEqual(camera.frame_id, 1)
        self.assertFalse(camera.connected)

    def test_live_capture_solves_without_prior_snapshot(self):
        # live=true：不先调用/snapshot，直接实时采集+检测+解算一步完成。
        with self.http_service() as (client, camera):
            solved = client.get_tcp_pose(
                [0, 0, 0, 0, 0, 0], 40, 40, 60, 60, live=True)
        self.assertEqual(set(solved), {"code", "pos"})
        self.assertEqual(solved["code"], 200, solved)
        self.assertTrue(np.allclose(
            solved["pos"], [0, 0, 950, 0, 0, 0], atol=1e-5))
        # live模式实时采集一帧；无需先前快照。
        self.assertEqual(camera.frame_id, 1)
        self.assertFalse(camera.connected)

    def test_live_capture_discards_stale_frames_during_stable_period(self):
        with self.http_service(
                "camera_center", live_capture_delay_s=4.0
        ) as (client, camera):
            solved = client.get_tcp_pose(
                [0, 0, 0, 0, 0, 0], 40, 40, 60, 60, live=True)
            self.assertTrue(camera.connected)

        self.assertEqual(solved["code"], 200, solved)
        self.assertEqual(camera.discard_durations, [4.0])
        self.assertEqual(camera.frame_id, 1)

    def test_snapshot_discards_stale_frames_then_cached_solve_does_not(self):
        with self.http_service(
                "camera_center", snapshot_delay_s=4.0
        ) as (client, camera):
            self.assertEqual(client.snapshot()["code"], 200)
            solved = client.get_tcp_pose(
                [0, 0, 0, 0, 0, 0], 40, 40, 60, 60)

        self.assertEqual(solved["code"], 200, solved)
        self.assertEqual(camera.discard_durations, [4.0])
        self.assertEqual(camera.frame_id, 1)

    def test_cached_solve_does_not_repeat_live_delay(self):
        with self.http_service(
                "camera_center", live_capture_delay_s=4.0
        ) as (client, camera):
            self.assertEqual(client.snapshot()["code"], 200)
            solved = client.get_tcp_pose(
                [0, 0, 0, 0, 0, 0], 40, 40, 60, 60)

        self.assertEqual(solved["code"], 200, solved)
        self.assertEqual(camera.discard_durations, [])

    def test_base_panel_plane_solve_through_http(self):
        # 传 base(基座面板矩形) + target：目标中心深度以面板平面为准。
        with self.http_service() as (client, _camera):
            solved = client.get_tcp_pose(
                [0, 0, 0, 0, 0, 0], 40, 40, 60, 60,
                base={"x1": 10, "y1": 10, "x2": 90, "y2": 90},
                live=True)
        self.assertEqual(solved["code"], 200, solved)
        self.assertTrue(np.allclose(
            solved["pos"], [0, 0, 950, 0, 0, 0], atol=1e-5))

    def test_tool_mode_applies_offset_after_standard_tcp(self):
        # 测试手眼为单位矩阵且没有全局修正，因此标准TCP就是50mm光心
        # 参考位；再沿标准局部x叠加100mm工具偏移。
        with self.http_service("tool") as (client, _camera):
            self.assertEqual(client.snapshot()["code"], 200)
            solved = client.get_tcp_pose(
                [0, 0, 0, 0, 0, 0], 40, 40, 60, 60)
        self.assertEqual(set(solved), {"code", "pos"})
        self.assertEqual(solved["code"], 200, solved)
        self.assertTrue(np.allclose(
            solved["pos"], [100, 0, 950, 0, 0, 0],
            atol=1e-5))

    def test_box_center_mode_skips_yolo_and_uses_configured_tool(self):
        # 请求框中心为(80,50)，相机内参对应平面交点(300,0,1000)mm；
        # 50mm悬停后再叠加panel工具局部X+100mm。
        with self.http_service(
                "tool", use_yolo=False, selected_tool="panel"
        ) as (client, camera):
            solved = client.get_tcp_pose(
                [0, 0, 0, 0, 0, 0], 75, 45, 85, 55, live=True)

        self.assertEqual(set(solved), {"code", "pos"})
        self.assertEqual(solved["code"], 200, solved)
        self.assertTrue(np.allclose(
            solved["pos"], [400, 0, 950, 0, 0, 0], atol=1e-5))
        self.assertEqual(camera.yolo_model.predict_calls, 0)

    def test_target_selection_mode_is_hot_reloaded(self):
        with self.http_service("tool") as (client, camera):
            first = client.get_tcp_pose(
                [0, 0, 0, 0, 0, 0], 40, 40, 60, 60, live=True)
            self.write_tool_offsets(
                x_offset_mm=100.0, use_yolo=False,
                selected_tool="panel")
            second = client.get_tcp_pose(
                [0, 0, 0, 0, 0, 0], 75, 45, 85, 55, live=True)

        self.assertEqual(first["code"], 200, first)
        self.assertEqual(second["code"], 200, second)
        self.assertTrue(np.allclose(
            second["pos"], [400, 0, 950, 0, 0, 0], atol=1e-5))
        self.assertEqual(camera.yolo_model.predict_calls, 1)

    def test_box_center_mode_requires_existing_enabled_tool(self):
        self.write_tool_offsets(use_yolo=False, selected_tool="missing")
        with self.assertRaisesRegex(ValueError, "未在tools中定义"):
            load_tool_offsets(self.tool_offsets_path)

    def test_v32_tool_file_without_target_selection_defaults_to_yolo(self):
        data = yaml.safe_load(
            self.tool_offsets_path.read_text(encoding="utf-8"))
        data.pop("target_selection")
        self.tool_offsets_path.write_text(
            yaml.safe_dump(data, allow_unicode=True, sort_keys=False),
            encoding="utf-8")
        snapshot = load_tool_offsets(self.tool_offsets_path)
        self.assertTrue(snapshot.use_yolo)
        self.assertEqual(snapshot.selected_tool, "")

    def test_tool_offset_file_is_reloaded_without_restarting_service(self):
        with self.http_service("tool") as (client, camera):
            self.assertEqual(client.snapshot()["code"], 200)
            first = client.get_tcp_pose(
                [0, 0, 0, 0, 0, 0], 40, 40, 60, 60)

            self.write_tool_offsets(x_offset_mm=225.0)
            self.assertEqual(client.snapshot()["code"], 200)
            second = client.get_tcp_pose(
                [0, 0, 0, 0, 0, 0], 40, 40, 60, 60)

            self.assertTrue(camera.connected)

        self.assertEqual(first["code"], 200, first)
        self.assertEqual(second["code"], 200, second)
        self.assertTrue(np.allclose(
            first["pos"], [100, 0, 950, 0, 0, 0], atol=1e-5))
        self.assertTrue(np.allclose(
            second["pos"], [225, 0, 950, 0, 0, 0], atol=1e-5))
        self.assertEqual(camera.frame_id, 2)

    def test_invalid_hot_reload_is_rejected_and_service_recovers(self):
        with self.http_service("tool") as (client, camera):
            self.assertEqual(client.snapshot()["code"], 200)
            self.tool_offsets_path.write_text(
                "version: 1\ntools: [\n", encoding="utf-8")
            invalid = client.get_tcp_pose(
                [0, 0, 0, 0, 0, 0], 40, 40, 60, 60)

            self.write_tool_offsets(x_offset_mm=175.0)
            recovered = client.get_tcp_pose(
                [0, 0, 0, 0, 0, 0], 40, 40, 60, 60)
            self.assertTrue(camera.connected)

        self.assertEqual(invalid["code"], 500, invalid)
        self.assertEqual(recovered["code"], 200, recovered)
        self.assertTrue(np.allclose(
            recovered["pos"], [175, 0, 950, 0, 0, 0], atol=1e-5))
        # 热加载失败不丢弃快照，因此修复配置后可以直接重试。
        self.assertEqual(camera.frame_id, 1)

    def test_failed_http_request_and_client_disconnect_do_not_end_service(self):
        with self.http_service() as (client, _camera):
            no_snapshot = client.get_tcp_pose(
                [0, 0, 0, 0, 0, 0], 40, 40, 60, 60)
            invalid = client.get_tcp_pose(
                [0, 0], 40, 40, 60, 60)
            # 新客户端代表后台连接断开后重新启动。
            reconnect = VisionHttpClient(client.base_url, timeout_s=5)
            captured = reconnect.snapshot()
            solved = reconnect.get_tcp_pose(
                [0, 0, 0, 0, 0, 0], 40, 40, 60, 60)
        self.assertEqual(no_snapshot["code"], 409)
        self.assertEqual(invalid["code"], 400)
        self.assertEqual(captured["code"], 200)
        self.assertEqual(solved["code"], 200, solved)

    def test_three_http_cycles_with_new_clients_keep_one_service_alive(self):
        with self.http_service() as (client, camera):
            paths = []
            for _index in range(3):
                reconnect = VisionHttpClient(client.base_url, timeout_s=5)
                snapshot = reconnect.snapshot()
                solved = reconnect.get_tcp_pose(
                    [0, 0, 0, 0, 0, 0], 40, 40, 60, 60)
                self.assertEqual(snapshot["code"], 200, snapshot)
                self.assertEqual(solved["code"], 200, solved)
                paths.append(snapshot["path"])
        self.assertEqual(len(set(paths)), 3)
        self.assertEqual(camera.frame_id, 3)


if __name__ == "__main__":
    unittest.main()
