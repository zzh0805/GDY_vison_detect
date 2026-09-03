# -*- coding: utf-8 -*-
"""知象光电 SurfacePro50 OpenNI2 相机适配器。"""
from __future__ import annotations

import ctypes
import contextlib
import io
import os
import platform
import re
import struct
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import numpy as np
try:
    from PyQt5 import QtCore
except ImportError:  # 同步 capture_snapshot 不需要 Qt；UI 环境仍应安装 PyQt5。
    class _UnavailableSignal:
        def connect(self, *_args, **_kwargs):
            return None

        def emit(self, *_args, **_kwargs):
            return None

    class _UnavailableQThread:
        def __init__(self, *_args, **_kwargs):
            pass

        def isRunning(self):
            return False

        def wait(self, *_args, **_kwargs):
            return True

    class _QtCoreFallback:
        QThread = _UnavailableQThread

        @staticmethod
        def pyqtSignal(*_args, **_kwargs):
            return _UnavailableSignal()

    QtCore = _QtCoreFallback()

try:
    from .hardware_interfaces import (CameraFrame, PARAM_EXPOSURE_MODE,
                                      PARAM_EXPOSURE_US, PARAM_GAIN,
                                      PARAM_TARGET_GRAY)
except ImportError:
    from hardware_interfaces import (CameraFrame, PARAM_EXPOSURE_MODE,
                                     PARAM_EXPOSURE_US, PARAM_GAIN,
                                     PARAM_TARGET_GRAY)


CS_PROPERTY_STREAM_INTRINSICS = 0xE0000001
DEFAULT_CHISHINE_CALIBRATION = (
    Path(__file__).resolve().parent / "config" /
    "chishine_192_168_16_122_calibration.yml"
)

# OpenNI2 驱动在进程内只能初始化一次；重复 initialize() 会破坏驱动状态
# （常见现象：第二次连接时 enumerate_uris() 为空、open_file 失败）。
_OPENNI_INIT_LOCK = threading.Lock()
_OPENNI_INITIALIZED = False


def _load_chishine_calibration_yaml(path: Path) -> Dict[str, Any]:
    """读取本项目导出的 OpenCV YAML，不依赖 cv2/PyYAML。"""
    source = Path(path).expanduser().resolve()
    text = source.read_text(encoding="utf-8")

    def scalar_int(name: str) -> int:
        match = re.search(rf"(?m)^{re.escape(name)}:\s*(\d+)\s*$", text)
        if not match:
            raise ValueError(f"标定 YAML 缺少 {name}")
        return int(match.group(1))

    def matrix(name: str, count: int) -> list[float]:
        match = re.search(
            rf"(?ms)^{re.escape(name)}:\s*!!opencv-matrix.*?^\s*data:\s*\[([^\]]+)\]",
            text)
        if not match:
            raise ValueError(f"标定 YAML 缺少矩阵 {name}")
        values = [float(item.strip()) for item in match.group(1).split(",")]
        if len(values) != count:
            raise ValueError(
                f"标定 YAML 的 {name} 应有 {count} 个数，实际 {len(values)}")
        return values

    return {
        "source": str(source),
        "depth_width": scalar_int("depth_width"),
        "depth_height": scalar_int("depth_height"),
        "rgb_width": scalar_int("rgb_width"),
        "rgb_height": scalar_int("rgb_height"),
        "K_depth": matrix("cameraMatrix_depth", 9),
        "K_rgb": matrix("cameraMatrix_rgb", 9),
        "D_depth": matrix("distCoeffs_depth_sdk_order", 5),
        "D_rgb": matrix("distCoeffs_rgb_sdk_order", 5),
        "R_sdk_raw": matrix("R_sdk_raw", 9),
        "T_depth_to_rgb": matrix("T_depth_to_rgb", 3),
    }


def _has_valid_points(points: Optional[np.ndarray]) -> bool:
    if points is None:
        return False
    array = np.asarray(points)
    return bool(array.ndim >= 2 and array.shape[-1] >= 3 and
                np.isfinite(array[..., :3]).all(axis=-1).any())


class CSIntrinsics(ctypes.Structure):
    """知象 OpenNI2 驱动返回的内参结构，与厂商 csdevice.py 一致。"""

    _fields_ = [
        ("width", ctypes.c_short), ("height", ctypes.c_short),
        ("fx", ctypes.c_float), ("zero01", ctypes.c_float),
        ("cx", ctypes.c_float), ("zero10", ctypes.c_float),
        ("fy", ctypes.c_float), ("cy", ctypes.c_float),
        ("zero20", ctypes.c_float), ("zero21", ctypes.c_float),
        ("one22", ctypes.c_float),
    ]


def _runtime_library_name(system: Optional[str] = None) -> str:
    system = system or platform.system()
    if system == "Windows":
        return "OpenNI2.dll"
    if system == "Linux":
        return "libOpenNI2.so"
    raise RuntimeError(f"SurfacePro50 OpenNI2 暂不支持当前系统: {system}")


def _expected_elf_machine(machine: Optional[str] = None) -> Optional[int]:
    """返回 ELF e_machine；ARM64/aarch64 对应 183。"""
    normalized = (machine or platform.machine()).strip().lower()
    return {
        "x86_64": 62, "amd64": 62,
        "aarch64": 183, "arm64": 183,
        "armv7l": 40, "armv8l": 40,
        "i386": 3, "i686": 3, "x86": 3,
    }.get(normalized)


def _elf_matches_python(library_path: Path) -> bool:
    """检查 Linux 动态库位数和 CPU 架构是否与当前 Python 一致。"""
    try:
        with library_path.open("rb") as stream:
            header = stream.read(20)
        if len(header) < 20 or header[:4] != b"\x7fELF":
            return False
        elf_class = header[4]
        expected_class = 2 if sys.maxsize > 2**32 else 1
        if elf_class != expected_class:
            return False
        byte_order = "little" if header[5] == 1 else "big" if header[5] == 2 else ""
        if not byte_order:
            return False
        elf_machine = int.from_bytes(header[18:20], byte_order)
        expected_machine = _expected_elf_machine()
        return expected_machine is not None and elf_machine == expected_machine
    except Exception:
        return False


def _library_matches_python(library_path: Path) -> bool:
    if platform.system() == "Windows":
        return _pe_matches_python(library_path)
    if platform.system() == "Linux":
        return _elf_matches_python(library_path)
    return False


def _redist_candidates(explicit: Optional[str]) -> list[Path]:
    values = [
        explicit,
        os.environ.get("SURFACEPRO50_OPENNI2_REDIST"),
        os.environ.get("OPENNI2_REDIST"),
        os.environ.get("OPENNI2_REDIST64"),
    ]
    if platform.system() == "Windows":
        values.extend([
            r"D:\Program Files\OpenNI2\Redist",
            r"C:\Program Files\OpenNI2\Redist",
        ])
    elif platform.system() == "Linux":
        values.extend([
            "/opt/OpenNI2/Redist",
            "/opt/openni2/Redist",
            "/opt/SurfacePro50/OpenNI2/Redist",
            "/usr/local/lib/OpenNI2/Redist",
            "/usr/lib/OpenNI2/Redist",
            "/usr/lib/aarch64-linux-gnu",
        ])
    result: list[Path] = []
    for value in values:
        if not value:
            continue
        path = Path(value).expanduser()
        if path not in result:
            result.append(path)
    return result


def find_openni2_redist(explicit: Optional[str] = None) -> str:
    """寻找与当前 Windows/Linux Python 架构匹配的 OpenNI2 Redist。"""
    library_name = _runtime_library_name()
    checked = []
    for candidate in _redist_candidates(explicit):
        roots = [candidate.parent] if candidate.name == library_name else [
            candidate, candidate / "Redist", candidate / "lib"]
        for root in roots:
            try:
                root = root.resolve()
            except OSError:
                continue
            library = root / library_name
            checked.append(str(library))
            if library.is_file() and _library_matches_python(library):
                return str(root)
    expected = f"{platform.system()} {platform.machine()} / {library_name}"
    hint = ("SURFACEPRO50_OPENNI2_REDIST（或 OPENNI2_REDIST）")
    detail = "\n  ".join(checked) if checked else "<没有配置候选目录>"
    raise FileNotFoundError(
        f"未找到匹配 {expected} 的 OpenNI2 Redist。请设置 {hint}。"
        f"\n已检查:\n  {detail}")


def _pe_matches_python(dll_path: Path) -> bool:
    """拒绝把 x86 OpenNI2.dll 加载到 x64 Python（或反向混用）。"""
    try:
        with dll_path.open("rb") as stream:
            stream.seek(0x3C)
            pe_offset = struct.unpack("<I", stream.read(4))[0]
            stream.seek(pe_offset + 4)
            machine = struct.unpack("<H", stream.read(2))[0]
        expected = 0x8664 if sys.maxsize > 2**32 else 0x014C
        return machine == expected
    except Exception:
        return False


def _add_openni_python_paths() -> None:
    """允许厂商把 openni Python 包放在 SDK 目录而非 site-packages。"""
    configured = os.environ.get("SURFACEPRO50_PYTHON_PATH", "")
    for value in reversed([v for v in configured.split(os.pathsep) if v]):
        path = str(Path(value).expanduser().resolve())
        if Path(path).is_dir() and path not in sys.path:
            sys.path.insert(0, path)


def _prepare_openni_runtime(redist: str) -> list[Any]:
    """配置 OpenNI2 搜索路径；由厂商 Python 包负责加载主库。"""
    handles: list[Any] = []
    os.environ["OPENNI2_REDIST"] = redist
    if platform.system() == "Windows":
        if hasattr(os, "add_dll_directory"):
            handles.append(os.add_dll_directory(redist))
        return handles
    # 知象 Linux Demo 使用 openni2.initialize() 自行加载主库。不要提前
    # ctypes.CDLL；部分厂商驱动依赖 oniInitialize 时的工作目录与加载时序。
    return handles


def _initialize_openni_once(openni2: Any, openni_redist: Optional[str],
                            dll_handles: list[Any]) -> None:
    """进程内只初始化一次 OpenNI2。

    优先无参 initialize()（与知象官方 Demo 一致）。关键：初始化前临时移除
    OPENNI2_REDIST 环境变量——openni 包无参初始化会读该变量选择 libOpenNI2.so，
    若它被外部脚本指向"标准 OpenNI2 官方 SDK"（只支持 USB、不支持知象网口
    相机），会导致 enumerate_uris() 为空、open_file(IP) 失败。
    仅当无参初始化失败时，才回退 find_openni2_redist + initialize(redist)。
    """
    global _OPENNI_INITIALIZED
    if _OPENNI_INITIALIZED:
        return
    redist: Optional[str] = None
    saved = os.environ.pop("OPENNI2_REDIST", None)
    try:
        openni2.initialize()
    except Exception:
        if saved is not None:
            os.environ["OPENNI2_REDIST"] = saved
        redist = find_openni2_redist(openni_redist)
        dll_handles.extend(_prepare_openni_runtime(redist))
        openni2.initialize(redist)
    _OPENNI_INITIALIZED = True


def _mode_key(mode: Any) -> str:
    return (f"{int(mode.resolutionX)}x{int(mode.resolutionY)}@{int(mode.fps)}"
            f"/fmt={int(mode.pixelFormat)}")


def _enum_value(value: Any) -> int:
    """兼容 ctypes 枚举、IntEnum 和普通整数。"""
    raw = getattr(value, "value", value)
    return int(raw)


def _positive_env_float(name: str, default: float) -> float:
    """读取正浮点配置，拒绝 NaN/Inf/非正数以避免静默破坏点云尺度。"""
    value = os.environ.get(name, str(default))
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"环境变量 {name} 不是有效数字: {value!r}") from exc
    if not np.isfinite(number) or number <= 0.0:
        raise ValueError(f"环境变量 {name} 必须是正有限数，当前为 {value!r}")
    return number


def _scale_and_validate_depth(raw: np.ndarray, scale_mm: float,
                              min_depth_mm: float,
                              max_depth_mm: float) -> Tuple[np.ndarray, np.ndarray,
                                                            Dict[str, Any]]:
    """转换深度为毫米并执行尺度门限；返回深度、有效掩码和统计量。"""
    image = np.asarray(raw)
    if image.ndim != 2:
        raise ValueError(f"深度图必须是二维数组，当前形状为 {image.shape}")
    if not np.isfinite(scale_mm) or scale_mm <= 0.0:
        raise ValueError(f"非法深度比例: {scale_mm}")
    if min_depth_mm < 0.0 or max_depth_mm <= min_depth_mm:
        raise ValueError(
            f"非法深度范围: [{min_depth_mm}, {max_depth_mm}] mm")

    raw_valid = image > 0
    raw_count = int(np.count_nonzero(raw_valid))
    depth_mm = image.astype(np.float32) * float(scale_mm)
    valid = (raw_valid & np.isfinite(depth_mm) &
             (depth_mm >= float(min_depth_mm)) &
             (depth_mm <= float(max_depth_mm)))
    valid_count = int(np.count_nonzero(valid))
    raw_median = float(np.median(image[raw_valid])) if raw_count else None
    converted_raw_median = (
        float(np.median(depth_mm[raw_valid])) if raw_count else None)

    # 至少 1% 的非零测量必须落入相机配置量程；错误的 0.1 mm 比例在本机
    # D2C 数据上会得到 5~12 mm，因此会在保存点云前被明确拒绝。
    required = max(1, int(np.ceil(raw_count * 0.01))) if raw_count else 0
    if raw_count and valid_count < required:
        raise RuntimeError(
            "SurfacePro50 深度尺度/量程异常: "
            f"raw_median={raw_median}, "
            f"converted_median={converted_raw_median} mm, "
            f"scale={scale_mm} mm/unit, "
            f"valid={valid_count}/{raw_count}, "
            f"range=[{min_depth_mm}, {max_depth_mm}] mm")

    depth_mm[~valid] = 0.0
    stats: Dict[str, Any] = {
        "raw_depth_nonzero_count": raw_count,
        "depth_valid_count": valid_count,
        "depth_valid_ratio": (float(valid_count / raw_count)
                              if raw_count else 0.0),
        "raw_depth_median": raw_median,
        "depth_median_mm": (float(np.median(depth_mm[valid]))
                            if valid_count else None),
    }
    return depth_mm, valid, stats


def _status_text(info: Any) -> str:
    if info is None:
        return "SurfacePro50"
    parts = []
    for name in ("vendor", "name", "uri"):
        value = getattr(info, name, "")
        if isinstance(value, bytes):
            value = value.decode("utf-8", errors="replace")
        value = str(value).strip()
        if value:
            parts.append(value)
    return " ".join(parts).strip() or "SurfacePro50"


def _open_openni_device(openni2: Any, endpoint: Any) -> Any:
    """按统一 endpoint 打开 OpenNI2 设备。

    ``auto``/``usb`` 保留 OpenNI2 自动发现行为；其余值严格按照知象官方
    Demo，以 bytes 形式传给 ``Device.open_file``。该驱动把网口 IP 复用为
    open_file 参数，不能依据方法名按普通文件路径处理。
    """
    # 兼容 bytes 输入（如直接来自 Device.enumerate_uris() 的 b'192.168.16.122'），
    # 避免 str(b'...') 生成带 b'' 字面量的错误 URI（b"b'192.168.16.122'"）。
    if isinstance(endpoint, bytes):
        uri = bytes(endpoint)
    else:
        value = str(endpoint or "auto").strip()
        if value.lower() in ("", "auto", "usb"):
            return openni2.Device.open_any()
        uri = value.encode("utf-8")
    try:
        return openni2.Device.open_file(uri)
    except Exception as exc:
        # 兜底：知象驱动会把网口相机也暴露给 open_any()（诊断实测 open_any
        # 能直接打开该网口相机）。当 open_file(IP) 因驱动/URI 差异失败时，
        # 尝试自动发现，避免"驱动能发现却打不开"的尴尬。
        try:
            return openni2.Device.open_any()
        except Exception:
            pass
        try:
            available = [
                item.decode("utf-8", errors="replace")
                if isinstance(item, bytes) else str(item)
                for item in openni2.Device.enumerate_uris()
            ]
        except Exception:
            available = []
        raise RuntimeError(
            f"无法按知象 OpenNI2 IP/URI 打开网口相机 {uri!r}；"
            f"驱动当前枚举 URI={available or '空'}。"
            "请确认相机与主机同网段、使用的 libOpenNI2.so 是知象驱动"
            "（而不是标准 OpenNI2 官方库），并使用厂家要求的 URI 格式。"
        ) from exc


class SurfacePro50Backend:
    """不依赖 Qt 的 OpenNI2 后端，供同步和异步适配器复用。"""

    adapter_name = "surfacepro50"

    def __init__(self, endpoint: str = "auto", capture_3d: bool = False,
                 openni_redist: Optional[str] = None,
                 unload_openni_on_disconnect: Optional[bool] = None):
        self.endpoint = str(endpoint or "auto")
        self.capture_3d = bool(capture_3d)
        calibration_value = os.environ.get(
            "SURFACEPRO50_CALIBRATION_YAML", str(DEFAULT_CHISHINE_CALIBRATION))
        self.software_registration_calibration = None
        self.software_registration_path = Path(calibration_value)
        self.last_point_colors: Optional[np.ndarray] = None
        # 厂家 YAML 的 RGB 畸变，按 OpenCV [k1,k2,p1,p2,k3] 顺序。
        # 棋盘 PnP 验证：用该畸变比按 0 处理重投影 RMS 从 ~0.35px 降到
        # ~0.25px，且符号/顺序均正确（反号/换序都会变差）。
        self.intrinsics_distortion_rgb: Optional[np.ndarray] = None
        try:
            if self.software_registration_path.is_file():
                _cal = _load_chishine_calibration_yaml(
                    self.software_registration_path)
                _d = np.asarray(_cal.get("D_rgb") or [], dtype=np.float64).reshape(-1)
                if _d.size >= 5:
                    # YAML 头标称 k1,k2,k3,k4,k5，但 PnP 实验证明第 3/4 项
                    # 是 p1/p2、第 5 项 k3，即 OpenCV [k1,k2,p1,p2,k3]。
                    self.intrinsics_distortion_rgb = _d[:5].reshape(1, 5).copy()
        except Exception:
            self.intrinsics_distortion_rgb = None
        self.openni_redist = openni_redist
        self.show_frame_output = str(os.environ.get(
            "SURFACEPRO50_SHOW_FRAME_OUTPUT", "0")).strip().lower() in (
                "1", "true", "yes", "on")
        self.unload_openni_on_disconnect = (
            platform.system() != "Linux"
            if unload_openni_on_disconnect is None
            else bool(unload_openni_on_disconnect))
        self.openni2 = None
        self._oni = None
        self.device = None
        self.image_stream = None
        self.image_sensor = ""
        self.depth_stream = None
        self.image_intrinsics: Optional[Tuple[np.ndarray, np.ndarray]] = None
        self.depth_intrinsics: Optional[Tuple[np.ndarray, np.ndarray]] = None
        self.image_intrinsics_source = ""
        self._last_intrinsics_source = ""
        self.depth_registered_to_image = False
        self.point_cloud_frame = "surfacepro50_depth_optical"
        self.point_cloud_handeye_compatible = False
        self.registration_method = "none"
        self.registration_error = ""
        self.registration_mode_actual = "off"
        self.depth_color_sync_enabled = False
        self.depth_resampled_to_image = False
        self.image_profile = ""
        self.depth_profile = ""
        self.last_raw_depth_size: Optional[Tuple[int, int]] = None
        self.last_depth_output_size: Optional[Tuple[int, int]] = None
        self.min_depth_mm = _positive_env_float(
            "SURFACEPRO50_MIN_DEPTH_MM", 100.0)
        self.max_depth_mm = _positive_env_float(
            "SURFACEPRO50_MAX_DEPTH_MM", 5000.0)
        if self.max_depth_mm <= self.min_depth_mm:
            raise ValueError(
                "SURFACEPRO50_MAX_DEPTH_MM 必须大于 SURFACEPRO50_MIN_DEPTH_MM")
        try:
            self.empty_depth_retry_count = int(os.environ.get(
                "SURFACEPRO50_EMPTY_DEPTH_RETRIES", "5"))
        except ValueError as exc:
            raise ValueError(
                "SURFACEPRO50_EMPTY_DEPTH_RETRIES必须是非负整数") from exc
        if self.empty_depth_retry_count < 0:
            raise ValueError(
                "SURFACEPRO50_EMPTY_DEPTH_RETRIES必须是非负整数")
        self.empty_depth_retry_interval_s = _positive_env_float(
            "SURFACEPRO50_EMPTY_DEPTH_RETRY_INTERVAL_S", 0.15)
        self.last_depth_scale_source = "none"
        self.last_depth_stats: Dict[str, Any] = {}
        self.frame_id = 0
        self._profiles: Dict[str, Any] = {}
        self._current_profile = ""
        self._lock = threading.RLock()
        self._connected = False
        self.device_description = ""
        self._dll_handles = []

    def connect(self, endpoint: Optional[str] = None) -> None:
        if endpoint:
            self.endpoint = str(endpoint)
        try:
            _add_openni_python_paths()
            try:
                from openni import _openni2, openni2
            except ImportError as exc:
                raise RuntimeError(
                    "无法导入 OpenNI2 Python 包 openni。请安装厂商 Python SDK，或将其目录"
                    "设置到 SURFACEPRO50_PYTHON_PATH。"
                ) from exc
            # 优先与知象官方 Demo 完全一致的无参数初始化（厂商包自带库加载逻辑，
            # 诊断实测在 conda 环境下无需 redist 即可连接网口相机）。
            # 仅当无参初始化失败时，才回退到 find_openni2_redist + initialize(redist)。
            # 进程内只初始化一次，避免重复 initialize 破坏驱动状态。
            with _OPENNI_INIT_LOCK:
                _initialize_openni_once(openni2, self.openni_redist, self._dll_handles)
            self.openni2, self._oni = openni2, _openni2
            self.device = _open_openni_device(openni2, self.endpoint)
            try:
                self.device_description = _status_text(self.device.get_device_info())
            except Exception:
                self.device_description = "SurfacePro50"
            self._open_image_stream()
            if self.capture_3d and self.image_sensor != "depth" and \
                    self.device.has_sensor(openni2.SENSOR_DEPTH):
                self.depth_stream = self.device.create_depth_stream()
                image_mode = self.image_stream.get_video_mode()
                target_size = (int(image_mode.resolutionX),
                               int(image_mode.resolutionY))
                self._select_default_mode(self.depth_stream, prefer_rgb=False,
                                          target_size=target_size,
                                          track_profile=False)
                self.depth_profile = _mode_key(
                    self.depth_stream.get_video_mode())
                self.depth_intrinsics = self._read_intrinsics(self.depth_stream)
                if self.image_sensor == "color" and \
                        self.software_registration_path.is_file():
                    # 保留原 OpenNI2 取流，但不打开会把深度压成约 10 mm 台阶的
                    # 硬件 D2C。保存时使用厂家 YAML 做软件 depth->RGB 配准。
                    self.software_registration_calibration = (
                        _load_chishine_calibration_yaml(
                            self.software_registration_path))
                    image_mode = self.image_stream.get_video_mode()
                    image_w = int(image_mode.resolutionX)
                    image_h = int(image_mode.resolutionY)
                    cal = self.software_registration_calibration
                    k_rgb = np.asarray(cal["K_rgb"], dtype=np.float64).reshape(3, 3).copy()
                    k_rgb[0, :] *= image_w / float(cal["rgb_width"])
                    k_rgb[1, :] *= image_h / float(cal["rgb_height"])
                    dist = self.intrinsics_distortion_rgb
                    self.image_intrinsics = (
                        k_rgb, (dist if dist is not None else
                                np.zeros((1, 5), dtype=np.float64)))
                    self.image_intrinsics_source = (
                        "factory_RGB_K_from_yaml; "
                        "distortion_from_factory_yaml"
                        if dist is not None
                        else "factory_RGB_K_from_yaml; "
                             "distortion_not_reapplied_for_vendor_texture_projection")
                    self.depth_stream.start()
                    self.depth_registered_to_image = False
                    self.depth_resampled_to_image = False
                    self.point_cloud_frame = "surfacepro50_color_optical"
                    self.point_cloud_handeye_compatible = True
                    self.registration_method = (
                        "software_factory_depth_to_rgb_on_raw_openni_z16")
                    self.registration_mode_actual = "openni_raw_depth_no_hardware_d2c"
                    self._enable_depth_color_sync()
                else:
                    # 软件配准为唯一路径：必须提供厂家 Depth→RGB 标定 YAML。
                    raise RuntimeError(
                        "缺少知象 Depth→RGB 标定 YAML，无法进行软件配准。"
                        f"请提供 {self.software_registration_path} "
                        "或设置环境变量 SURFACEPRO50_CALIBRATION_YAML")
            elif self.image_sensor == "depth":
                self.depth_stream = self.image_stream
                self.depth_intrinsics = self.image_intrinsics
                self.point_cloud_frame = "surfacepro50_depth_optical"
                self.point_cloud_handeye_compatible = True
            self._connected = True
        except Exception:
            self.disconnect()
            raise

    def _open_image_stream(self) -> None:
        o = self.openni2
        choices = (("color", o.SENSOR_COLOR), ("ir", o.SENSOR_IR),
                   ("depth", o.SENSOR_DEPTH))
        last_error = None
        for name, sensor in choices:
            try:
                if not self.device.has_sensor(sensor):
                    continue
                stream = self.device.create_stream(sensor)
                self._select_default_mode(stream, prefer_rgb=(name == "color"))
                stream.start()
                self.image_stream = stream
                self.image_sensor = name
                self.image_intrinsics = self._read_intrinsics(stream)
                self.image_intrinsics_source = self._last_intrinsics_source
                self.image_profile = _mode_key(stream.get_video_mode())
                return
            except Exception as exc:
                last_error = exc
        raise RuntimeError(f"设备没有可用的 Color/IR/Depth 图像流: {last_error}")

    def _select_default_mode(self, stream: Any, prefer_rgb: bool,
                             target_size: Optional[Tuple[int, int]] = None,
                             track_profile: bool = True) -> None:
        modes = list(stream.get_sensor_info().videoModes)
        if not modes:
            raise RuntimeError("OpenNI2 图像流没有可用模式")
        if prefer_rgb:
            rgb = [m for m in modes if int(m.pixelFormat) == int(
                self._oni.OniPixelFormat.ONI_PIXEL_FORMAT_RGB888)]
            if rgb:
                modes = rgb
        if target_size is not None:
            matching = [m for m in modes
                        if (int(m.resolutionX), int(m.resolutionY)) == target_size]
            if matching:
                modes = matching
        mode = max(modes, key=lambda m: (int(m.resolutionX) * int(m.resolutionY),
                                         int(m.fps)))
        stream.set_video_mode(mode)
        if track_profile:
            self._profiles = {_mode_key(m): m for m in modes}
            self._current_profile = _mode_key(mode)

    def _enable_depth_color_sync(self) -> None:
        self.depth_color_sync_enabled = False
        if self.software_registration_calibration is None or self.device is None:
            return
        try:
            self.device.set_depth_color_sync_enabled(True)
            getter = getattr(self.device, "get_depth_color_sync_enabled", None)
            self.depth_color_sync_enabled = bool(
                getter() if callable(getter) else True)
        except Exception:
            # 空间配准已经通过实际模式验证；不支持硬件同步时仍允许静止采集，
            # 但在 metadata 中明确记录 false。
            self.depth_color_sync_enabled = False

    def _assert_registration_active(self) -> None:
        """软件配准为唯一路径；连接阶段已保证标定 YAML 存在。"""
        if not self.capture_3d or self.image_sensor != "color":
            return
        if self.software_registration_calibration is None:
            raise RuntimeError(
                "缺少知象标定 YAML，无法进行软件 Depth-to-RGB 配准")

    def _read_intrinsics(self, stream: Any) -> Tuple[np.ndarray, np.ndarray]:
        try:
            intr = stream.get_property(CS_PROPERTY_STREAM_INTRINSICS, CSIntrinsics)
            if intr.fx > 0 and intr.fy > 0:
                mode = stream.get_video_mode()
                width, height = int(mode.resolutionX), int(mode.resolutionY)
                sx = width / float(intr.width) if intr.width > 0 else 1.0
                sy = height / float(intr.height) if intr.height > 0 else 1.0
                K = np.array([[intr.fx * sx, 0.0, intr.cx * sx],
                              [0.0, intr.fy * sy, intr.cy * sy],
                              [0.0, 0.0, 1.0]], dtype=np.float64)
                dist = self.intrinsics_distortion_rgb
                self._last_intrinsics_source = (
                    "vendor_property_K; distortion_from_factory_yaml"
                    if dist is not None
                    else "vendor_property_K; distortion_assumed_zero")
                return K, (dist if dist is not None else
                           np.zeros((1, 5), dtype=np.float64))
        except Exception:
            pass
        mode = stream.get_video_mode()
        width, height = int(mode.resolutionX), int(mode.resolutionY)
        try:
            fx = width / (2.0 * np.tan(float(stream.get_horizontal_fov()) / 2.0))
            fy = height / (2.0 * np.tan(float(stream.get_vertical_fov()) / 2.0))
        except Exception as exc:
            raise RuntimeError(f"SurfacePro50 内参读取失败: {exc}") from exc
        K = np.array([[fx, 0.0, (width - 1.0) / 2.0],
                      [0.0, fy, (height - 1.0) / 2.0],
                      [0.0, 0.0, 1.0]], dtype=np.float64)
        dist = self.intrinsics_distortion_rgb
        self._last_intrinsics_source = (
            "openni_fov_K; distortion_from_factory_yaml"
            if dist is not None
            else "openni_fov_K; distortion_assumed_zero")
        return K, (dist if dist is not None else
                   np.zeros((1, 5), dtype=np.float64))

    def disconnect(self) -> None:
        global _OPENNI_INITIALIZED
        with self._lock:
            seen = set()
            for stream in (self.image_stream, self.depth_stream):
                if stream is None or id(stream) in seen:
                    continue
                seen.add(id(stream))
                try:
                    stream.stop()
                except Exception:
                    pass
                try:
                    stream.close()
                except Exception:
                    pass
            try:
                if self.device is not None:
                    self.device.close()
            except Exception:
                pass
            self.image_stream = self.depth_stream = self.device = None
            self._connected = False
            if self.openni2 is not None and self.unload_openni_on_disconnect:
                try:
                    self.openni2.unload()
                    _OPENNI_INITIALIZED = False
                except Exception:
                    pass
            if self.unload_openni_on_disconnect:
                for handle in self._dll_handles:
                    try:
                        handle.close()
                    except Exception:
                        pass
                self._dll_handles.clear()

    def is_connected(self) -> bool:
        return bool(self._connected and self.device is not None and self.image_stream is not None)

    @staticmethod
    def _uint16_to_gray(image: np.ndarray) -> np.ndarray:
        valid = image[image > 0]
        if valid.size == 0:
            return np.zeros(image.shape, dtype=np.uint8)
        lo, hi = np.percentile(valid, [1.0, 99.0])
        if hi <= lo:
            hi = lo + 1.0
        return np.clip((image.astype(np.float32) - lo) * (255.0 / (hi - lo)),
                       0, 255).astype(np.uint8)

    def _read_stream_frame(self, stream: Any, channel: str):
        """默认隐藏厂商Python封装的逐帧print；异常时回放其诊断输出。"""
        if self.show_frame_output:
            return stream.read_frame()
        captured = io.StringIO()
        try:
            with contextlib.redirect_stdout(captured):
                return stream.read_frame()
        except BaseException:
            driver_output = captured.getvalue().strip()
            if driver_output:
                print(f"SurfacePro50 {channel}取帧异常时驱动输出:\n{driver_output}")
            raise

    def _read_image(self) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        frame = self._read_stream_frame(self.image_stream, "图像")
        h, w = int(frame.height), int(frame.width)
        if self.image_sensor == "color":
            rgb = np.asarray(frame.get_buffer_as_triplet(), dtype=np.uint8).reshape(h, w, 3).copy()
            bgr = rgb[:, :, ::-1].copy()
            # 与 OpenCV COLOR_BGR2GRAY 等价，避免后端强依赖 cv2。
            gray = np.clip(0.114 * bgr[:, :, 0] + 0.587 * bgr[:, :, 1] +
                           0.299 * bgr[:, :, 2], 0, 255).astype(np.uint8)
            return gray, bgr
        raw = np.asarray(frame.get_buffer_as_uint16(), dtype=np.uint16).reshape(h, w).copy()
        return self._uint16_to_gray(raw), None

    def _read_depth(self, color_bgr: Optional[np.ndarray] = None
                    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray],
                               float, Optional[np.ndarray]]:
        if self.depth_stream is None:
            return None, None, 1.0, None
        frame = self._read_stream_frame(self.depth_stream, "深度")
        h, w = int(frame.height), int(frame.width)
        raw = np.asarray(frame.get_buffer_as_uint16(), dtype=np.uint16).reshape(h, w).copy()
        self.last_raw_depth_size = (w, h)
        self.last_depth_output_size = (w, h)
        pixel_format = int(frame.videoMode.pixelFormat)
        if pixel_format == int(
                self._oni.OniPixelFormat.ONI_PIXEL_FORMAT_DEPTH_100_UM):
            scale_mm = 0.1
            self.last_depth_scale_source = "openni_depth_100_um"
        else:
            scale_mm = 1.0
            self.last_depth_scale_source = "openni_depth_1_mm"
        if self.software_registration_calibration is not None:
            if color_bgr is None:
                raise RuntimeError("软件 Depth-to-RGB 配准需要同次采集的彩色图")
            # 先执行尺度门限，明确拒绝驱动意外返回 10 mm D2C 数据的情况。
            _checked_depth, _valid, stats = _scale_and_validate_depth(
                raw, scale_mm, self.min_depth_mm, self.max_depth_mm)
            del _checked_depth, _valid
            try:
                from .chishine_registration import (
                    reconstruct_organized_cloud_rgb_frame)
            except ImportError:
                from chishine_registration import (
                    reconstruct_organized_cloud_rgb_frame)
            (depth_mm, point_cloud, depth_rgb_mm, point_colors,
             diagnostics) = reconstruct_organized_cloud_rgb_frame(
                raw, color_bgr, scale_mm,
                self.software_registration_calibration,
                self.min_depth_mm, self.max_depth_mm)
            self.last_depth_stats = {**stats, **diagnostics}
            self.last_point_colors = point_colors
            # 有组织对齐点云：深度以彩色图为准（RGB 光学坐标系，mm）。
            # 注意：不改 depth_registered_to_image，避免 _read_depth 顶部
            # 的硬件 D2C 重采样分支误触发。
            self.depth_resampled_to_image = True
            return depth_rgb_mm, point_cloud, scale_mm, point_colors
        depth_mm, valid, self.last_depth_stats = _scale_and_validate_depth(
            raw, scale_mm, self.min_depth_mm, self.max_depth_mm)
        point_cloud = None
        if self.depth_intrinsics is not None:
            K = self.depth_intrinsics[0]
            uu, vv = np.meshgrid(np.arange(w, dtype=np.float32),
                                 np.arange(h, dtype=np.float32))
            z = depth_mm
            x = (uu - float(K[0, 2])) * z / float(K[0, 0])
            y = (vv - float(K[1, 2])) * z / float(K[1, 1])
            point_cloud = np.stack([x, y, z], axis=-1).astype(np.float32)
            point_cloud[~valid] = np.nan
        self.last_point_colors = None
        return depth_mm, point_cloud, scale_mm, None

    def capture(self, save_images: Optional[bool] = None) -> CameraFrame:
        with self._lock:
            if not self.is_connected():
                raise RuntimeError("SurfacePro50 未连接")
            build_3d = self.capture_3d if save_images is None else (
                self.capture_3d and bool(save_images))
            self._assert_registration_active()
            attempts = self.empty_depth_retry_count + 1
            gray = color = depth = points = point_colors = None
            depth_scale = 1.0
            for attempt in range(attempts):
                gray, color = self._read_image()
                if build_3d:
                    depth, points, depth_scale, point_colors = self._read_depth(color)
                if not build_3d:
                    break
                if _has_valid_points(points):
                    break
                if attempt + 1 < attempts:
                    time.sleep(self.empty_depth_retry_interval_s)
            if build_3d and (
                    not _has_valid_points(points)):
                raise RuntimeError(
                    "SurfacePro50连续深度帧没有有效点: "
                    f"attempts={attempts}, "
                    f"raw_nonzero={self.last_depth_stats.get('raw_depth_nonzero_count')}, "
                    f"scale={depth_scale} mm/unit, "
                    f"raw_size={self.last_raw_depth_size}, "
                    f"output_size={self.last_depth_output_size}")
            frame = CameraFrame(
                frame_id=self.frame_id, gray=gray, color=color, depth=depth,
                point_cloud=points, point_colors=point_colors,
                camera_frame=self.point_cloud_frame,
                metadata={
                    "adapter": self.adapter_name,
                    "endpoint": self.endpoint,
                    "device": self.device_description,
                    "image_sensor": self.image_sensor,
                    "profile": self.image_profile or self._current_profile,
                    "image_profile": self.image_profile or self._current_profile,
                    "depth_profile": self.depth_profile,
                    "depth_unit": "mm",
                    "point_cloud_unit": "mm",
                    "calibration_camera_frame":
                        f"surfacepro50_{self.image_sensor}_optical",
                    "point_cloud_frame": self.point_cloud_frame,
                    "depth_registered_to_image": self.depth_registered_to_image,
                    "registration_method": self.registration_method,
                    "registration_mode_actual": self.registration_mode_actual,
                    "registration_error": self.registration_error,
                    "depth_color_sync_enabled": self.depth_color_sync_enabled,
                    "depth_resampled_to_image": self.depth_resampled_to_image,
                    "raw_depth_size": self.last_raw_depth_size,
                    "depth_output_size": self.last_depth_output_size,
                    "point_cloud_handeye_compatible":
                        self.point_cloud_handeye_compatible,
                    "point_cloud_pixel_aligned_to_image": bool(
                        points is not None and color is not None and
                        points.ndim == 3 and points.shape[:2] == color.shape[:2] and
                        (self.depth_registered_to_image or
                         self.software_registration_calibration is not None)),
                    "colored_reconstruction_requested": bool(build_3d),
                    "color_for_yolo": color is not None,
                    "software_registration_calibration": (
                        None if self.software_registration_calibration is None
                        else self.software_registration_calibration.get("source")),
                    "depth_scale_mm": depth_scale,
                    "depth_scale_source": self.last_depth_scale_source,
                    "depth_valid_min_mm": self.min_depth_mm,
                    "depth_valid_max_mm": self.max_depth_mm,
                    **self.last_depth_stats,
                })
            self.frame_id += 1
            return frame

    def get_intrinsics(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        if self.image_intrinsics is None:
            return None
        return self.image_intrinsics[0].copy(), self.image_intrinsics[1].copy()

    def get_parameters(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {"profile": self._current_profile,
                               "image_sensor": self.image_sensor,
                               "intrinsics_source": self.image_intrinsics_source}
        if self.image_stream is None or self._oni is None:
            return out
        queries = {
            PARAM_EXPOSURE_US: (self._oni.ONI_STREAM_PROPERTY_EXPOSURE, ctypes.c_uint32),
            PARAM_GAIN: (self._oni.ONI_STREAM_PROPERTY_GAIN, ctypes.c_uint32),
            PARAM_EXPOSURE_MODE: (self._oni.ONI_STREAM_PROPERTY_AUTO_EXPOSURE,
                                  ctypes.c_uint32),
        }
        for name, (prop, data_type) in queries.items():
            try:
                value = self.image_stream.get_property(prop, data_type).value
                out[name] = ("auto" if value else "manual") if \
                    name == PARAM_EXPOSURE_MODE else value
            except Exception:
                pass
        return out

    def set_parameter(self, name: str, value: Any) -> bool:
        if self.image_stream is None or self._oni is None:
            return False
        if name == PARAM_TARGET_GRAY:
            return False
        mapping = {
            PARAM_EXPOSURE_US: self._oni.ONI_STREAM_PROPERTY_EXPOSURE,
            PARAM_GAIN: self._oni.ONI_STREAM_PROPERTY_GAIN,
            PARAM_EXPOSURE_MODE: self._oni.ONI_STREAM_PROPERTY_AUTO_EXPOSURE,
        }
        if name not in mapping:
            return False
        if name == PARAM_EXPOSURE_MODE:
            value = 1 if str(value).strip().lower() in ("auto", "automatic", "1") else 0
        try:
            self.image_stream.set_property(mapping[name], ctypes.c_uint32(int(value)))
            return True
        except Exception:
            return False

    def list_profiles(self) -> list[str]:
        return list(self._profiles)

    def current_profile(self) -> str:
        return self._current_profile

    def select_profile(self, name: str) -> bool:
        mode = self._profiles.get(str(name))
        if mode is None or self.image_stream is None:
            return False
        with self._lock:
            try:
                self.image_stream.stop()
                self.image_stream.set_video_mode(mode)
                self.image_stream.start()
                self._current_profile = str(name)
                self.image_intrinsics = self._read_intrinsics(self.image_stream)
                self.image_intrinsics_source = self._last_intrinsics_source
                return True
            except Exception:
                try:
                    self.image_stream.start()
                except Exception:
                    pass
                return False


class SurfacePro50SyncAdapter:
    """供自动采集脚本使用的同步接口。"""

    adapter_name = "surfacepro50"

    def __init__(self, endpoint: str = "auto", capture_3d: bool = True, **kwargs):
        self.backend = SurfacePro50Backend(endpoint, capture_3d,
                                           kwargs.get("openni_redist"),
                                           kwargs.get(
                                               "unload_openni_on_disconnect"))

    def connect(self, endpoint: Optional[str] = None) -> None:
        self.backend.connect(endpoint)

    def disconnect(self) -> None:
        self.backend.disconnect()

    def is_connected(self) -> bool:
        return self.backend.is_connected()

    def capture(self) -> CameraFrame:
        return self.backend.capture(save_images=self.backend.capture_3d)

    def get_intrinsics(self):
        return self.backend.get_intrinsics()


class SurfacePro50AsyncAdapter(QtCore.QThread):
    """供标定 UI 使用的事件驱动异步接口。"""

    adapter_name = "surfacepro50"
    log_signal = QtCore.pyqtSignal(str)
    init_signal = QtCore.pyqtSignal(bool, str)
    capture_success_signal = QtCore.pyqtSignal(int, dict)
    capture_error_signal = QtCore.pyqtSignal(str)
    param_signal = QtCore.pyqtSignal(dict)

    def __init__(self, endpoint: str = "auto", capture_3d: bool = False,
                 parent=None, backend_factory=SurfacePro50Backend, **kwargs):
        super().__init__(parent)
        backend_kwargs = {
            "endpoint": endpoint,
            "capture_3d": capture_3d,
            "openni_redist": kwargs.get("openni_redist"),
        }
        if kwargs.get("unload_openni_on_disconnect") is not None:
            backend_kwargs["unload_openni_on_disconnect"] = kwargs[
                "unload_openni_on_disconnect"]
        self.backend = backend_factory(**backend_kwargs)
        self.running = True
        self.capture_event = threading.Event()
        self.finish_event = threading.Event()
        self._save_images_requested = False

    def run(self) -> None:
        try:
            self.backend.connect()
            message = (f"SurfacePro50 已连接: {self.backend.device_description}; "
                       f"标定图像流={self.backend.image_sensor}; "
                       f"内参={self.backend.image_intrinsics_source}")
            if self.backend.capture_3d:
                message += (
                    f"; D2C={self.backend.registration_mode_actual}; "
                    f"Color={self.backend.image_profile}; "
                    f"Depth={self.backend.depth_profile}; "
                    f"输出点云坐标系={self.backend.point_cloud_frame}"
                )
            self.log_signal.emit(message)
            self.init_signal.emit(True, message)
            self.param_signal.emit(self.get_parameters())
            while self.running:
                if not self.capture_event.wait(0.5):
                    continue
                if not self.running:
                    break
                self.capture_event.clear()
                try:
                    save_images = self._save_images_requested
                    self._save_images_requested = False
                    frame = self.backend.capture(save_images=save_images)
                    self.capture_success_signal.emit(frame.frame_id, frame.as_dict())
                except Exception as exc:
                    message = f"SurfacePro50 拍照失败: {exc}"
                    self.log_signal.emit(message)
                    self.capture_error_signal.emit(message)
                finally:
                    self.finish_event.set()
        except Exception as exc:
            message = f"SurfacePro50 初始化失败: {exc}"
            self.log_signal.emit(message)
            self.init_signal.emit(False, message)
        finally:
            self.backend.disconnect()

    def trigger(self, save_images: bool = False) -> bool:
        if not self.is_connected() or not self.running or self.capture_event.is_set():
            return False
        self.finish_event.clear()
        self._save_images_requested = bool(save_images)
        self.capture_event.set()
        return True

    def is_running(self) -> bool:
        return bool(self.isRunning())

    def is_connected(self) -> bool:
        return self.backend.is_connected()

    def shutdown(self) -> None:
        self.running = False
        self.capture_event.set()
        self.wait(5000)

    def get_intrinsics(self):
        return self.backend.get_intrinsics()

    def get_parameters(self) -> Dict[str, Any]:
        return self.backend.get_parameters()

    def set_parameter(self, name: str, value: Any) -> bool:
        return self.backend.set_parameter(name, value)

    def list_profiles(self) -> list[str]:
        return self.backend.list_profiles()

    def current_profile(self) -> str:
        return self.backend.current_profile()

    def select_profile(self, name: str) -> bool:
        ok = self.backend.select_profile(name)
        if ok:
            self.param_signal.emit(self.get_parameters())
        return ok

    def save_parameters(self) -> bool:
        # OpenNI2 通用接口不提供写入设备参数组；当前会话内设置已生效。
        return True
