"""v5 native soft trigger backend. Business config, registration and pose math unchanged."""
from __future__ import annotations

import json
import logging
import mmap
import os
from pathlib import Path
import queue
import subprocess
import sys
import tempfile
import threading
import time

import cv2
import numpy as np

from .hardware_interfaces import CameraFrame
from .chishine_registration import reconstruct_organized_cloud_rgb_frame, scale_intrinsic

log = logging.getLogger(__name__)
ROOT = Path(__file__).resolve().parents[1]
NATIVE = Path(__file__).parent / "native_camera"
CAPACITY = 16 * 1024 * 1024
PREFIX = "GDY_NATIVE_V5 "


class NativeCameraProcess:
    """One persistent SDK owner, serialized RPC and a fixed 32MiB shared buffer.

    A native deadlock cannot hold the Python server's shutdown indefinitely.
    No automatic retry/reconnect or stale-image fallback after a failed capture.
    """
    def __init__(self):
        self.process = None
        self.memory = None
        self._file = None
        self._temp = None
        self._reader = None
        self._watcher = None
        self._stop_watch = threading.Event()
        self._responses = queue.Queue(maxsize=2)
        self._sequence = 0
        self._lock = threading.RLock()
        self.ready = False

    def _read_responses(self, process, responses):
        try:
            while True:
                line = process.stdout.readline(65536)
                if not line:
                    break
                if line.startswith(PREFIX):
                    try:
                        responses.put_nowait(json.loads(line[len(PREFIX):]))
                    except (ValueError, queue.Full):
                        log.error("native camera invalid/overflow response; stopping worker")
                        process.kill()
                        break
                elif line.strip():
                    log.info("native camera: %s", line.rstrip()[:2000])
        except (ValueError, OSError):
            pass

    def _watch_memory(self, process, stop):
        while not stop.wait(1):
            if process.poll() is not None:
                return
            try:
                status = Path(f"/proc/{process.pid}/status").read_text()
                usage = sum(int(line.split()[1]) for line in status.splitlines()
                            if line.startswith(("VmRSS:", "VmSwap:")))
                if usage > 2 * 1024 * 1024:
                    log.error("v5 SDK worker内存超过2GiB，终止相机进程；需要检查后重启服务")
                    process.kill()
                    return
            except (OSError, ValueError):
                continue

    def start(self, endpoint):
        with self._lock:
            if self.process is not None:
                raise RuntimeError("native worker already exists")
            if sys.platform != "linux":
                raise RuntimeError("v5实时相机目前仅支持Ubuntu原生SDK；Windows可运行离线/模拟测试")
            library = NATIVE / "libgdy_native_camera.so"
            location = NATIVE / "sdk_location.json"
            if not library.is_file() or not location.is_file():
                raise RuntimeError("v5相机桥接库未编译，请先执行 bash build_native_camera.sh")
            sdk_dir = Path(json.loads(location.read_text(encoding="utf-8"))["sdk_lib_dir"])
            if not (sdk_dir / "lib3DCamera.so").is_file():
                raise RuntimeError("原生SDK路径已变化，请在此Ubuntu重新执行 bash build_native_camera.sh")
            try:
                self._temp = tempfile.TemporaryDirectory(prefix="gdy_native_v5_")
                buffer_path = Path(self._temp.name) / "frames.bin"
                self._file = buffer_path.open("w+b")
                self._file.truncate(CAPACITY*2)
                self.memory = mmap.mmap(self._file.fileno(), CAPACITY*2)
                env = os.environ.copy()
                env.pop("LD_PRELOAD", None)  # Do not preload the service's torch libraries into SDK worker.
                env["LD_LIBRARY_PATH"] = str(sdk_dir) + os.pathsep + env.get("LD_LIBRARY_PATH", "")
                self._responses = queue.Queue(maxsize=2)
                self.process = subprocess.Popen(
                    [sys.executable, "-u", str(NATIVE / "worker.py"), str(library), str(buffer_path), str(os.getpid())],
                    stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, encoding="utf-8", errors="replace", env=env, bufsize=1,
                    start_new_session=True)
                self._reader = threading.Thread(target=self._read_responses,
                    args=(self.process, self._responses), name="native-camera-ipc", daemon=True)
                self._reader.start()
                self._stop_watch = threading.Event()
                self._watcher = threading.Thread(target=self._watch_memory,
                    args=(self.process, self._stop_watch), name="native-camera-memory", daemon=True)
                self._watcher.start()
                result = self.request("open", 45, ip=str(endpoint))
                self.ready = True
                log.info("v5相机原生软触发连接成功 PID=%s depthScale=%s profiles=%s/%s",
                         self.process.pid, result["depth_scale_mm"], result["widths"], result["heights"])
                return result
            except BaseException:
                self.abort()
                raise

    def request(self, operation, timeout, **fields):
        with self._lock:
            try:
                if self.process is None or self.process.poll() is not None:
                    raise RuntimeError("native camera worker is not running; restart service after checking camera")
                self._sequence += 1
                rid = self._sequence
                self.process.stdin.write(json.dumps({"id": rid, "op": operation, **fields}) + "\n")
                self.process.stdin.flush()
                deadline = time.monotonic() + timeout
                while True:
                    left = deadline - time.monotonic()
                    if left <= 0:
                        raise TimeoutError(f"native camera {operation}超过{timeout}s，终止子进程，不返回旧图")
                    # Bound native memory even while SDK is blocked. Does not cap model/pointcloud memory.
                    try:
                        status = Path(f"/proc/{self.process.pid}/status").read_text()
                        usage = sum(int(line.split()[1]) for line in status.splitlines()
                                    if line.startswith(("VmRSS:", "VmSwap:")))
                        if usage > 2 * 1024 * 1024:
                            raise MemoryError("native camera RSS+Swap超过2GiB保护上限")
                    except (FileNotFoundError, PermissionError):
                        pass
                    try:
                        response = self._responses.get(timeout=min(left, 0.25))
                    except queue.Empty:
                        if self.process.poll() is not None:
                            raise RuntimeError(f"native camera worker exited: {self.process.returncode}")
                        continue
                    if response.get("id") != rid:
                        raise RuntimeError(f"native camera response mismatch: {response}")
                    if not response.get("ok"):
                        raise RuntimeError(f"native camera {operation}: {response.get('error')}")
                    return response
            except BaseException:
                self.abort()
                raise

    def capture(self):
        with self._lock:
            result = self.request("capture", 25)
            try:
                arrays = []
                for i, channels, dtype in ((0, 1, np.dtype("<u2")), (1, 3, np.dtype("u1"))):
                    w, h = int(result["widths"][i]), int(result["heights"][i])
                    nbytes = w*h*channels*dtype.itemsize
                    if w <= 0 or h <= 0 or nbytes > CAPACITY or int(result["sizes"][i]) != nbytes:
                        raise RuntimeError("invalid native IPC image dimensions/size")
                    shape = (h, w) if channels == 1 else (h, w, channels)
                    arrays.append(np.ndarray(shape, dtype=dtype, buffer=self.memory, offset=i*CAPACITY).copy())
                return arrays[0], arrays[1], result
            except BaseException:
                self.abort()
                raise

    def is_connected(self):
        return self.ready and self.process is not None and self.process.poll() is None

    def close(self):
        with self._lock:
            try:
                if self.is_connected():
                    self.request("close", 8)
            except Exception as exc:
                log.warning("相机原生关闭未正常完成，已隔离终止SDK子进程: %s", exc)
            finally:
                self.abort()

    def abort(self):
        with self._lock:
            self.ready = False
            self._stop_watch.set()
            process = self.process
            self.process = None
            if process is not None:
                if process.poll() is None:
                    process.kill()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired as exc:
                    # Keep ownership: never start a second SDK owner if the kernel cannot reap this one.
                    self.process = process
                    raise RuntimeError("相机子进程在强制终止后仍未退出，禁止重复连接；请检查系统/设备") from exc
                if self._reader is not None and self._reader is not threading.current_thread():
                    self._reader.join(timeout=2)
                for stream in (process.stdin, process.stdout):
                    if stream is not None:
                        stream.close()
            self._reader = None
            if self._watcher is not None and self._watcher is not threading.current_thread():
                self._watcher.join(timeout=2)
            self._watcher = None
            if self.memory is not None:
                self.memory.close()
                self.memory = None
            if self._file is not None:
                self._file.close()
                self._file = None
            if self._temp is not None:
                self._temp.cleanup()
                self._temp = None


class NativeSurfacePro50Backend:
    adapter_name = "surfacepro50_native_soft_trigger_v5"

    def __init__(self, endpoint="auto", capture_3d=True, *legacy_args, **legacy_kwargs):
        # Accept the old adapter constructor without changing workflow.yaml.
        from .surfacepro50_adapter import _load_chishine_calibration_yaml, _positive_env_float
        self.endpoint = str(endpoint)
        self.capture_3d = bool(capture_3d)
        self.calibration = _load_chishine_calibration_yaml(Path(os.environ.get(
            "SURFACEPRO50_CALIBRATION_YAML", ROOT / "calibration/chishine_192_168_16_122_calibration.yml")))
        self.min_depth_mm = _positive_env_float("SURFACEPRO50_MIN_DEPTH_MM", 100.0)
        self.max_depth_mm = _positive_env_float("SURFACEPRO50_MAX_DEPTH_MM", 5000.0)
        if self.max_depth_mm <= self.min_depth_mm:
            raise ValueError("invalid depth range")
        self.client = NativeCameraProcess()
        self._lock = threading.RLock()
        self.frame_id = 0
        self.scale = None
        self.intrinsics = None
        self.profiles = None

    def connect(self, endpoint=None):
        with self._lock:
            if self.is_connected():
                return
            self.endpoint = str(endpoint or self.endpoint)
            try:
                info = self.client.start(self.endpoint)
                self.scale = float(info["depth_scale_mm"])
                if not np.isfinite(self.scale) or self.scale <= 0:
                    raise RuntimeError("invalid native depthScale")
                self.profiles = (tuple(info["widths"]), tuple(info["heights"]))
                self._set_intrinsics(info["widths"][1], info["heights"][1])
            except BaseException:
                self.client.abort()
                raise

    def _set_intrinsics(self, width, height):
        c = self.calibration
        self.intrinsics = (scale_intrinsic(c["K_rgb"], c["rgb_width"], c["rgb_height"], width, height),
                           np.asarray(c["D_rgb"], dtype=np.float64).reshape(1, 5).copy())

    def is_connected(self):
        return self.client.is_connected()

    def disconnect(self):
        with self._lock:
            self.client.close()

    def get_intrinsics(self):
        return self.intrinsics

    def discard_frames(self, duration_s):
        # Preserve the configured mechanical settling duration. Trigger only afterwards.
        duration = float(duration_s)
        if not np.isfinite(duration) or duration < 0:
            raise ValueError("invalid settling duration")
        with self._lock:
            if duration:
                log.info("v5稳定等待 %.3fs：不触发、不读取；随后按请求软触发", duration)
                time.sleep(duration)
        return 0

    def capture_color(self):
        return self.capture(save_images=False)

    def capture(self, save_images=None):
        with self._lock:
            if not self.is_connected():
                raise RuntimeError("v5相机未连接，请检查相机并重新启动服务")
            raw, rgb, info = self.client.capture()
            if ((tuple(info["widths"]), tuple(info["heights"])) != self.profiles):
                raise RuntimeError("native camera profile changed during capture")
            color = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
            build_3d = self.capture_3d if save_images is None else self.capture_3d and bool(save_images)
            depth = points = colors = None
            diagnostics = {}
            if build_3d:
                from .surfacepro50_adapter import _scale_and_validate_depth, _has_valid_points
                checked, valid, diagnostics = _scale_and_validate_depth(raw, self.scale, self.min_depth_mm, self.max_depth_mm)
                del checked, valid
                _, points, depth, colors, registration = reconstruct_organized_cloud_rgb_frame(
                    raw, color, self.scale, self.calibration, self.min_depth_mm, self.max_depth_mm)
                diagnostics.update(registration)
                if not _has_valid_points(points):
                    raise RuntimeError("native camera contains no valid RGB-aligned depth points; refusing pose solve")
            h, w = color.shape[:2]
            metadata = {
                "adapter": self.adapter_name, "endpoint": self.endpoint, "image_sensor": "color",
                "image_profile": f"{w}x{h}:RGB8", "profile": f"{w}x{h}:RGB8",
                "depth_profile": f"{raw.shape[1]}x{raw.shape[0]}:Z16",
                "depth_unit": "mm", "point_cloud_unit": "mm",
                "calibration_camera_frame": "surfacepro50_color_optical",
                "point_cloud_frame": "surfacepro50_color_optical",
                "point_cloud_handeye_compatible": True,
                "point_cloud_pixel_aligned_to_image": points is not None,
                "depth_registered_to_image": False, "depth_resampled_to_image": points is not None,
                "registration_method": "software_factory_depth_to_rgb_on_native_z16",
                "registration_mode_actual": "native_raw_depth_no_hardware_d2c", "registration_error": "",
                "depth_color_sync_enabled": False, "synchronization_proven": False,
                "raw_depth_size": (raw.shape[1], raw.shape[0]), "depth_output_size": (w, h) if build_3d else None,
                "intrinsics_source": "factory_RGB_K_from_yaml; distortion_from_factory_yaml",
                "software_registration_calibration": self.calibration["source"],
                "depth_scale_mm": self.scale, "depth_scale_source": "native_PROPERTY_EXT_DEPTH_SCALE",
                "native_timestamps_ms": info["stamps"], "native_capture_s": info["capture_s"],
                "colored_reconstruction_requested": bool(build_3d), "color_for_yolo": True,
                "discarded_frame_pairs_before_capture": 0, "discard_duration_s_before_capture": 0.0,
                "depth_valid_min_mm": self.min_depth_mm, "depth_valid_max_mm": self.max_depth_mm,
                **diagnostics,
            }
            frame = CameraFrame(frame_id=self.frame_id, color=color, gray=gray, depth=depth,
                                point_cloud=points, point_colors=colors,
                                camera_frame="surfacepro50_color_optical", metadata=metadata)
            self.frame_id += 1
            log.info("v5按请求软触发完成: frame=%s RGB=%sx%s SDK=%.3fs depthScale=%s",
                     frame.frame_id, w, h, info["capture_s"], self.scale)
            return frame
