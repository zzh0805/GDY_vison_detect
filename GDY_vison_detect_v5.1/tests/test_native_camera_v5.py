import json
import mmap
from pathlib import Path
import shutil
import tempfile
import time
import unittest
from unittest.mock import patch, MagicMock

import numpy as np
from handeye_calib import native_surfacepro50 as native
from handeye_calib.chishine_registration import reconstruct_organized_cloud_rgb_frame
from handeye_calib.surfacepro50_adapter import SurfacePro50SyncAdapter


class NativeIpcTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        root = Path(self.temp.name)
        shutil.copyfile(Path(__file__).with_name("fake_native_worker.py"), root / "worker.py")
        (root / "gdy_camera_worker").touch()
        (root / "lib3DCamera.so").touch()
        (root / "sdk_location.json").write_text(json.dumps({"sdk_lib_dir": str(root)}))
        self.location = patch.object(native, "NATIVE", root)
        self.platform = patch.object(native.sys, "platform", "linux")
        self.location.start()
        self.platform.start()
        self.client = native.NativeCameraProcess()
        self.client._worker_command = lambda executable, buffer_path: [
            native.sys.executable, "-u", str(root / "worker.py"), str(executable), str(buffer_path)]

    def tearDown(self):
        self.client.abort()
        self.platform.stop()
        self.location.stop()
        self.temp.cleanup()

    def test_real_subprocess_shared_buffer_ownership_and_idle(self):
        self.client.start("normal")
        self.assertTrue(self.client.is_connected())
        depth1, rgb1, result1 = self.client.capture()
        time.sleep(0.05)
        depth2, rgb2, result2 = self.client.capture()
        self.assertEqual(result2["counter"], 2)  # No unsolicited captures during idle.
        self.assertTrue(np.all(depth1 == 501))
        self.assertTrue(np.all(depth2 == 502))
        self.assertEqual(rgb1[0, 0].tolist(), [255, 1, 0])
        self.assertEqual(rgb2[0, 0].tolist(), [255, 2, 0])
        process = self.client.process
        folder = self.client._temp.name
        self.client.close()
        self.assertIsNotNone(process.poll())
        self.assertIsNone(self.client.memory)
        self.assertFalse(Path(folder).exists())

    def test_blocked_sdk_is_killed_and_buffers_released(self):
        self.client.start("hang_capture")
        process = self.client.process
        started = time.monotonic()
        with self.assertRaises(TimeoutError):
            self.client.request("capture", 0.2)
        self.assertLess(time.monotonic()-started, 5)
        self.assertIsNotNone(process.poll())
        self.assertIsNone(self.client.memory)
        self.assertFalse(self.client.is_connected())

    def test_preview_copy_does_not_alias_capture_shared_memory(self):
        self.client.start("normal")
        self.client.preview_start()
        rgb, meta = self.client.preview_frame()
        self.assertEqual(rgb[0, 0].tolist(), [10, 20, 30])
        raw, captured, info = self.client.capture()
        self.assertEqual(rgb[0, 0].tolist(), [10, 20, 30])
        self.assertEqual(captured[0, 0].tolist(), [255, 1, 0])
        self.client.preview_frame()
        self.assertEqual(captured[0, 0].tolist(), [255, 1, 0])
        self.client.preview_stop()
        self.assertIsNone(self.client.preview_frame()[0])

    def test_capture_failure_never_returns_previous_frame(self):
        self.client.start("fail_capture")
        with self.assertRaisesRegex(RuntimeError, "simulated native timeout"):
            self.client.capture()
        self.assertIsNone(self.client.memory)

    def test_invalid_payload_rejected(self):
        self.client.start("invalid_size")
        with self.assertRaisesRegex(RuntimeError, "dimensions"):
            self.client.capture()
        self.assertFalse(self.client.is_connected())

    def test_blocked_close_does_not_block_service(self):
        self.client.start("hang_close")
        process = self.client.process
        request = self.client.request
        with patch.object(self.client, "request", side_effect=lambda op, timeout: request(op, .2)):
            self.client.close()
        self.assertIsNotNone(process.poll())
        self.assertIsNone(self.client.memory)

    def test_worker_command_carries_the_property_backup_path(self):
        """The C++ owner must be told where to persist the original device settings, otherwise a
        killed worker would adopt its own modified values as "original" on restart."""
        command = native.NativeCameraProcess()._worker_command(Path("/tmp/x"), Path("/tmp/f.bin"))
        self.assertEqual(len(command), 4)
        self.assertTrue(command[3].endswith("native_property_backup.txt"), command)

    def test_crashed_worker_is_rebuilt_after_backoff(self):
        self.client.start("normal")
        dead = self.client.process
        dead.kill()
        dead.wait(5)
        self.assertFalse(self.client.is_connected())
        backend = native.NativeSurfacePro50Backend("normal")
        backend.client = self.client
        backend.scale = .1
        backend.profiles = ((2, 2), (2, 2))
        backend._set_intrinsics(2, 2)
        self.assertTrue(backend.ensure_connected())
        self.assertTrue(self.client.is_connected())
        self.assertNotEqual(self.client.process.pid, dead.pid)
        raw, rgb, info = self.client.capture()
        self.assertEqual(raw.shape, (2, 2))

    def test_reconnect_backoff_blocks_a_retry_storm(self):
        backend = native.NativeSurfacePro50Backend("normal")
        backend.client = MagicMock()
        backend.client.is_connected.return_value = False
        backend.client.start.side_effect = RuntimeError("camera absent")
        self.assertFalse(backend.ensure_connected())
        self.assertFalse(backend.ensure_connected())  # Still inside the backoff window.
        self.assertEqual(backend.client.start.call_count, 1)


class NativeAdapterTests(unittest.TestCase):
    def setUp(self):
        self.backend = native.NativeSurfacePro50Backend("192.168.16.122")
        self.backend.calibration = {"source": "synthetic", "rgb_width": 2, "rgb_height": 2,
            "depth_width": 2, "depth_height": 2, "K_rgb": [100, 0, 0, 0, 100, 0, 0, 0, 1],
            "K_depth": [100, 0, 0, 0, 100, 0, 0, 0, 1], "D_rgb": [0]*5,
            "R_sdk_raw": np.eye(3).reshape(-1).tolist(), "T_depth_to_rgb": [0, 0, 0]}
        self.backend.scale = .1
        self.backend.profiles = ((2, 2), (2, 2))
        self.backend._set_intrinsics(2, 2)
        self.raw = np.full((2, 2), 5000, dtype=np.uint16)
        self.rgb = np.full((2, 2, 3), [255, 20, 5], dtype=np.uint8)
        self.info = {"widths": [2, 2], "heights": [2, 2], "stamps": [0, 0], "capture_s": 10.0}
        self.backend.client = MagicMock()
        self.backend.client.is_connected.return_value = True
        self.backend.client.capture.side_effect = lambda: (self.raw.copy(), self.rgb.copy(), self.info.copy())

    def test_sync_adapter_routes_to_native_not_openni(self):
        cam = SurfacePro50SyncAdapter("192.168.16.122", capture_3d=True)
        self.assertIsInstance(cam.backend, native.NativeSurfacePro50Backend)
        self.assertFalse(cam.is_connected())

    def test_color_scale_and_registration_are_unchanged(self):
        frame = self.backend.capture()
        self.assertEqual(frame.color[0, 0].tolist(), [5, 20, 255])
        self.assertTrue(np.allclose(frame.depth, 500))
        expected = reconstruct_organized_cloud_rgb_frame(self.raw, self.rgb[..., ::-1], .1,
                                                        self.backend.calibration, 100, 5000)
        np.testing.assert_allclose(frame.point_cloud, expected[1], equal_nan=True)
        np.testing.assert_array_equal(frame.point_colors, expected[3])
        self.assertEqual(frame.camera_frame, "surfacepro50_color_optical")
        self.assertTrue(frame.metadata["point_cloud_pixel_aligned_to_image"])
        self.assertFalse(frame.metadata["synchronization_proven"])

    def test_color_capture_skips_pointcloud_but_still_uses_triggered_pair(self):
        frame = self.backend.capture_color()
        self.backend.client.capture.assert_called_once()
        self.assertIsNone(frame.point_cloud)
        self.assertIsNone(frame.depth)
        self.assertIsNotNone(frame.color)

    def test_zero_depth_rejected(self):
        self.raw.fill(0)
        with self.assertRaisesRegex(RuntimeError, "no valid"):
            self.backend.capture()

    def test_settling_delay_never_reads_or_triggers(self):
        with patch.object(native.time, "sleep") as sleep:
            self.assertEqual(self.backend.discard_frames(4), 0)
        sleep.assert_called_once_with(4)
        self.backend.client.capture.assert_not_called()

    def test_profile_change_rejected(self):
        self.info["widths"] = [4, 4]
        with self.assertRaisesRegex(RuntimeError, "profile changed"):
            self.backend.capture()

    def test_intrinsics_match_existing_scaling(self):
        self.backend._set_intrinsics(4, 6)
        k, d = self.backend.get_intrinsics()
        self.assertEqual(k[0, 0], 200)
        self.assertEqual(k[1, 1], 300)
        self.assertEqual(d.shape, (1, 5))

    def test_memory_guard_runs_without_capture_request(self):
        process = MagicMock()
        process.poll.return_value = None
        stop = MagicMock()
        stop.wait.side_effect = [False, True]
        client = native.NativeCameraProcess()
        with patch.object(native.Path, "read_text", return_value="VmRSS: 2200000 kB\nVmSwap: 0 kB\n"):
            client._watch_memory(process, stop)
        process.kill.assert_called_once()

    def test_camera_manager_cleans_failed_adapter(self):
        from vision_solver.camera import CameraManager
        config = MagicMock()
        config.camera = {}
        adapter = MagicMock()
        adapter.is_connected.return_value = False
        manager = CameraManager(config, adapter)
        manager.disconnect()
        adapter.disconnect.assert_called_once()


if __name__ == "__main__":
    unittest.main()
