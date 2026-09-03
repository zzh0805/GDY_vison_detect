# -*- coding: utf-8 -*-
"""JAKA Python SDK 的 RobotAdapter 实现。

JAKA SDK 的 CartesianPose 约定为平移 mm、姿态 RPY rad；项目统一接口约定为
T_base_tcp、平移 m、姿态旋转向量 rad。本模块在边界处完成这两种表示的转换。
"""
from __future__ import annotations

import importlib
import ctypes
import os
import platform
import struct
import sys
from pathlib import Path
from typing import Any, Optional, Tuple

import numpy as np

try:
    from . import handeye_math as hm
except ImportError:
    import handeye_math as hm


DEFAULT_JAKA_SDK_PATH = Path(
    r"D:\edge_downloade\20260104145805A007\SDK V2.3.1_beta3\Windows"
    r"\WINDOWS_MSVC_V142_x64\Windows\python3\x64"
)

DEFAULT_JAKA_LINUX_PATHS = (
    Path("/opt/JAKA/SDK/Linux/python3/aarch64-linux-gnu"),
    Path("/opt/jaka/Linux/python3/aarch64-linux-gnu"),
    Path("/usr/local/lib/jaka/python3/aarch64-linux-gnu"),
)


def _jaka_library_names(system: Optional[str] = None) -> Tuple[str, str]:
    """返回当前平台的 Python 扩展名和 JAKA 主动态库名。"""
    system = system or platform.system()
    if system == "Windows":
        return "jkrc.pyd", "jakaAPI.dll"
    if system == "Linux":
        return "jkrc.so", "libjakaAPI.so"
    raise RuntimeError(f"JAKA SDK 暂不支持当前系统: {system}")


def _expected_elf_machine(machine: Optional[str] = None) -> Optional[int]:
    normalized = (machine or platform.machine()).strip().lower()
    return {
        "x86_64": 62, "amd64": 62,
        "aarch64": 183, "arm64": 183,
        "armv7l": 40, "armv8l": 40,
        "i386": 3, "i686": 3, "x86": 3,
    }.get(normalized)


def _elf_matches_python(library_path: Path) -> bool:
    """检查 Linux .so 的位数和 CPU 架构是否匹配当前 Python。"""
    try:
        with library_path.open("rb") as stream:
            header = stream.read(20)
        if len(header) < 20 or header[:4] != b"\x7fELF":
            return False
        expected_class = 2 if sys.maxsize > 2**32 else 1
        if header[4] != expected_class:
            return False
        byte_order = "little" if header[5] == 1 else "big" if header[5] == 2 else ""
        if not byte_order:
            return False
        expected_machine = _expected_elf_machine()
        return (expected_machine is not None and
                int.from_bytes(header[18:20], byte_order) == expected_machine)
    except Exception:
        return False


def _pe_matches_python(library_path: Path) -> bool:
    """检查 Windows .pyd/.dll 的位数是否匹配当前 Python。"""
    try:
        with library_path.open("rb") as stream:
            stream.seek(0x3C)
            pe_offset = struct.unpack("<I", stream.read(4))[0]
            stream.seek(pe_offset + 4)
            machine = struct.unpack("<H", stream.read(2))[0]
        expected = 0x8664 if sys.maxsize > 2**32 else 0x014C
        return machine == expected
    except Exception:
        return False


def _binary_matches_python(path: Path) -> bool:
    if platform.system() == "Windows":
        return _pe_matches_python(path)
    if platform.system() == "Linux":
        return _elf_matches_python(path)
    return False


def _sdk_search_dirs(root: Path) -> list[Path]:
    """兼容直接指向二进制目录或指向 JAKA SDK 上层目录。"""
    machine = platform.machine().strip().lower()
    triplet = "aarch64-linux-gnu" if machine in ("aarch64", "arm64") else \
        "x86_64-linux-gnu" if machine in ("x86_64", "amd64") else machine
    candidates = [root]
    if platform.system() == "Linux":
        candidates.extend([
            root / "python3" / triplet,
            root / "Linux" / "python3" / triplet,
            root / "x86_64-aarch64-linux-gnu" / "Linux" / "python3" / triplet,
            root / "Linux" / "x86_64-aarch64-linux-gnu" / "Linux" /
            "python3" / triplet,
        ])
    result = []
    for candidate in candidates:
        try:
            candidate = candidate.expanduser().resolve()
        except OSError:
            continue
        if candidate not in result:
            result.append(candidate)
    return result


def _configured_library_dirs() -> list[Path]:
    values = os.environ.get("JAKA_LIBRARY_PATH", "")
    return [Path(value).expanduser().resolve()
            for value in values.split(os.pathsep) if value]


def jaka_rpy_to_matrix(rpy: np.ndarray) -> np.ndarray:
    """JAKA RPY (Rx,Ry,Rz) -> Rz @ Ry @ Rx。"""
    rx, ry, rz = np.asarray(rpy, dtype=np.float64).reshape(3)
    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)
    rx_m = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]])
    ry_m = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]])
    rz_m = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
    return rz_m @ ry_m @ rx_m


def matrix_to_jaka_rpy(rotation: np.ndarray) -> np.ndarray:
    """旋转矩阵 -> JAKA RPY；奇异位姿采用 rz=0 的确定性分支。"""
    rotation = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    sy = float(np.clip(-rotation[2, 0], -1.0, 1.0))
    ry = float(np.arcsin(sy))
    cy = float(np.cos(ry))
    if abs(cy) > 1e-8:
        rx = float(np.arctan2(rotation[2, 1], rotation[2, 2]))
        rz = float(np.arctan2(rotation[1, 0], rotation[0, 0]))
    else:
        rx = float(np.arctan2(-rotation[1, 2], rotation[1, 1]))
        rz = 0.0
    return np.array([rx, ry, rz], dtype=np.float64)


def jaka_pose_to_unified(pose_mm_rpy: np.ndarray) -> np.ndarray:
    """[mm, RPY rad] -> [m, rotvec rad]。"""
    pose = np.asarray(pose_mm_rpy, dtype=np.float64).reshape(6)
    return np.concatenate([pose[:3] * 0.001,
                           hm._mat_to_rotvec(jaka_rpy_to_matrix(pose[3:]))])


def unified_pose_to_jaka(pose_m_rotvec: np.ndarray) -> np.ndarray:
    """[m, rotvec rad] -> [mm, RPY rad]。"""
    pose = np.asarray(pose_m_rotvec, dtype=np.float64).reshape(6)
    return np.concatenate([pose[:3] * 1000.0,
                           matrix_to_jaka_rpy(hm._rotvec_to_mat(pose[3:]))])


def _split_result(result: Any) -> Tuple[int, Any]:
    """兼容 JAKA SDK 的 int / (code,) / (code, payload) 返回格式。"""
    if isinstance(result, (tuple, list)):
        if not result:
            return -1, None
        return int(result[0]), result[1] if len(result) > 1 else None
    return int(result), None


class JAKARobotAdapter:
    """JAKA SDK V2.3.1 RobotAdapter。

    连接仅执行 login，不会自动上电、使能或移动机器人。运动接口只有在
    enable_motion=True 时开放，且要求机器人已由操作者安全上电并使能。
    """

    adapter_name = "jaka"

    def __init__(self, endpoint: str = "", enable_motion: bool = False,
                 speed: float = 0.15, accel: float = 0.3,
                 settle: float = 2.0, timeout: float = 30.0,
                 sdk_path: Optional[str] = None, jkrc_module: Any = None):
        self.endpoint = str(endpoint)
        self.enable_motion = bool(enable_motion)
        self.speed = float(speed)
        self.accel = float(accel)
        self.settle = float(settle)
        self.timeout = float(timeout)
        self.sdk_path = Path(sdk_path) if sdk_path else None
        self._jkrc = jkrc_module
        self._dll_handles = []
        self.robot = None
        self._logged_in = False

    def _load_sdk(self):
        if self._jkrc is not None:
            return self._jkrc
        module_name, dependency_name = _jaka_library_names()
        roots = []
        configured = os.environ.get("JAKA_SDK_PATH")
        if self.sdk_path:
            roots.append(self.sdk_path)
        if configured:
            roots.extend(Path(value) for value in configured.split(os.pathsep) if value)
        if platform.system() == "Windows":
            roots.append(DEFAULT_JAKA_SDK_PATH)
        elif platform.system() == "Linux":
            roots.extend(DEFAULT_JAKA_LINUX_PATHS)

        candidates = []
        for root in roots:
            for path in _sdk_search_dirs(root):
                if path not in candidates:
                    candidates.append(path)

        import_errors = []
        try:
            self._jkrc = importlib.import_module("jkrc")
            return self._jkrc
        except Exception as exc:
            import_errors.append(f"系统 Python 路径: {exc}")

        for path in candidates:
            module_path = path / module_name
            if not module_path.is_file():
                continue
            if not _binary_matches_python(module_path):
                import_errors.append(
                    f"{module_path}: 二进制架构与 {platform.machine()} Python 不匹配")
                continue
            try:
                dependency_candidates = [path / dependency_name]
                dependency_candidates.extend(
                    library_dir / dependency_name
                    for library_dir in _configured_library_dirs())
                dependency = next(
                    (item for item in dependency_candidates if item.is_file()), None)
                if dependency is None:
                    raise FileNotFoundError(
                        f"未找到 {dependency_name}；请放到 {path}，或设置 "
                        "JAKA_LIBRARY_PATH")
                if not _binary_matches_python(dependency):
                    raise RuntimeError(
                        f"{dependency} 的二进制架构与 {platform.machine()} Python 不匹配")

                if platform.system() == "Windows" and hasattr(os, "add_dll_directory"):
                    self._dll_handles.append(os.add_dll_directory(str(path)))
                    if dependency.parent != path:
                        self._dll_handles.append(
                            os.add_dll_directory(str(dependency.parent)))
                elif platform.system() == "Linux":
                    self._dll_handles.append(ctypes.CDLL(
                        str(dependency), mode=ctypes.RTLD_GLOBAL))
                if str(path) not in sys.path:
                    sys.path.insert(0, str(path))
                importlib.invalidate_caches()
                self._jkrc = importlib.import_module("jkrc")
                self.sdk_path = path
                return self._jkrc
            except Exception as exc:
                import_errors.append(f"{path}: {exc}")
        detail = "; ".join(import_errors)
        raise ImportError(
            "无法加载 JAKA Python SDK。请设置 JAKA_SDK_PATH 指向同时包含 "
            f"{module_name} 和 {dependency_name} 的 {platform.machine()} 目录；"
            f"动态库分开放置时设置 JAKA_LIBRARY_PATH。详情: {detail}")

    def connect(self, endpoint: Optional[str] = None) -> None:
        if endpoint:
            self.endpoint = str(endpoint)
        if not self.endpoint:
            raise ValueError("JAKA 适配器需要控制器 IP")
        sdk = self._load_sdk()
        robot = sdk.RC(self.endpoint)
        login = getattr(robot, "login", None) or getattr(robot, "log_in", None)
        if login is None:
            raise AttributeError("JAKA SDK 缺少 login/log_in")
        code, _ = _split_result(login())
        if code != 0:
            raise ConnectionError(f"JAKA 登录失败: IP={self.endpoint}, code={code}")
        self.robot = robot
        self._logged_in = True

    def disconnect(self) -> None:
        if self.robot is not None and self._logged_in:
            try:
                logout = (getattr(self.robot, "logout", None) or
                          getattr(self.robot, "log_out", None))
                if logout is not None:
                    logout()
            except Exception:
                pass
        self.robot = None
        self._logged_in = False

    def is_connected(self) -> bool:
        return bool(self.robot is not None and self._logged_in)

    def _actual_jaka_pose(self) -> np.ndarray:
        if not self.is_connected():
            raise RuntimeError("JAKA 机器人未连接")
        getter = (getattr(self.robot, "get_actual_tcp_position", None) or
                  getattr(self.robot, "get_tcp_position", None))
        if getter is None:
            raise AttributeError("JAKA SDK 缺少 TCP 位姿读取接口")
        code, payload = _split_result(getter())
        if code != 0 or payload is None:
            raise RuntimeError(f"JAKA TCP 位姿读取失败: code={code}")
        pose = np.asarray(payload, dtype=np.float64).reshape(6)
        if not np.all(np.isfinite(pose)):
            raise RuntimeError("JAKA 返回了无效 TCP 位姿")
        return pose

    def get_tcp_pose(self) -> np.ndarray:
        return jaka_pose_to_unified(self._actual_jaka_pose())

    def get_tcp_matrix(self) -> np.ndarray:
        pose = self.get_tcp_pose()
        return hm.homogeneous(hm._rotvec_to_mat(pose[3:]), pose[:3])

    def move_linear(self, target: np.ndarray, name: str = "") -> None:
        if not self.enable_motion:
            raise RuntimeError("当前 JAKA 适配器未启用运动控制")
        if not self.is_connected():
            raise RuntimeError("JAKA 机器人未连接")
        target = np.asarray(target, dtype=np.float64)
        if target.shape == (4, 4):
            unified = np.concatenate([target[:3, 3],
                                      hm._mat_to_rotvec(target[:3, :3])])
        else:
            unified = target.reshape(6)
        jaka_target = tuple(float(x) for x in unified_pose_to_jaka(unified))
        speed_mm_s = self.speed * 1000.0
        accel_mm_s2 = self.accel * 1000.0
        if hasattr(self.robot, "linear_move_extend"):
            result = self.robot.linear_move_extend(
                jaka_target, 0, True, speed_mm_s, accel_mm_s2, 0.1)
        else:
            result = self.robot.linear_move(jaka_target, 0, True, speed_mm_s)
        code, _ = _split_result(result)
        if code != 0:
            raise RuntimeError(f"{name} JAKA linear_move 失败: code={code}")
        if self.settle > 0:
            import time
            time.sleep(self.settle)

    def stop_motion(self) -> None:
        """尽力停止当前运动，兼容不同 JAKA Python SDK 的方法命名。

        SDK 版本未提供停止方法时不伪造成功；上层仍会保留原始运动异常。
        """
        if self.robot is None:
            return
        for method_name in ("motion_abort", "stop_move", "stop_motion"):
            method = getattr(self.robot, method_name, None)
            if callable(method):
                code, _ = _split_result(method())
                if code != 0:
                    raise RuntimeError(
                        f"JAKA {method_name} 停止运动失败: code={code}")
                return

    def close(self) -> None:
        self.disconnect()
