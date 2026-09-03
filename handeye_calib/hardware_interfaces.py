# -*- coding: utf-8 -*-
"""手眼标定硬件抽象接口。

所有机器人适配器必须输出 T_base_tcp：平移 m、旋转向量 rad。
所有相机适配器必须输出与内参严格对应的图像帧。
接口采用 Protocol，第三方适配器无需继承本文件的基类，只要实现同名方法即可。
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Protocol, Tuple, runtime_checkable

import numpy as np


# UI/适配器之间使用的通用参数名；具体 SDK 名称由适配器内部映射。
PARAM_EXPOSURE_US = "exposure_us"
PARAM_GAIN = "gain"
PARAM_EXPOSURE_MODE = "exposure_mode"
PARAM_TARGET_GRAY = "target_gray"
# exposure_mode 的通用值为 "auto" / "manual"；厂商枚举由适配器转换。


@dataclass
class CameraFrame:
    """一次相机采集的统一数据结构。"""

    frame_id: int = -1
    timestamp: float = field(default_factory=time.time)
    gray: Optional[np.ndarray] = None
    color: Optional[np.ndarray] = None
    depth: Optional[np.ndarray] = None
    point_cloud: Optional[np.ndarray] = None
    point_colors: Optional[np.ndarray] = None
    camera_frame: str = "camera"
    metadata: Dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "CameraFrame":
        """从异步 Qt 相机信号的字典恢复统一帧。

        该方法是 ``as_dict`` 的逆向边界适配，不复制大型图像和点云数组。
        调用方应把数组视为只读数据。
        """
        if not isinstance(data, dict):
            raise TypeError("CameraFrame.from_dict 需要 dict")
        return cls(
            frame_id=int(data.get("frame_id", -1)),
            timestamp=float(data.get("timestamp", time.time())),
            gray=data.get("gray"),
            color=data.get("color"),
            depth=data.get("depth"),
            point_cloud=data.get("point_cloud"),
            point_colors=data.get("point_colors"),
            camera_frame=str(data.get("camera_frame", "camera")),
            metadata=dict(data.get("metadata") or {}),
        )

    def as_dict(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {
            "frame_id": int(self.frame_id),
            "timestamp": float(self.timestamp),
            "camera_frame": str(self.camera_frame),
            "metadata": dict(self.metadata),
        }
        for name in ("gray", "color", "depth", "point_cloud", "point_colors"):
            value = getattr(self, name)
            if value is not None:
                out[name] = value
        return out


@runtime_checkable
class RobotAdapter(Protocol):
    """机器人统一接口。get_tcp_pose 的定义固定为 T_base_tcp。"""

    adapter_name: str

    def connect(self, endpoint: Optional[str] = None) -> None: ...
    def disconnect(self) -> None: ...
    def is_connected(self) -> bool: ...
    def get_tcp_pose(self) -> np.ndarray: ...
    def get_tcp_matrix(self) -> np.ndarray: ...
    def move_linear(self, target: np.ndarray, name: str = "") -> None: ...


@runtime_checkable
class AsyncCameraAdapter(Protocol):
    """供 Qt UI 使用的异步相机接口。

    实现还需提供 log_signal/init_signal/capture_success_signal/
    capture_error_signal/param_signal 五个 Qt 信号。capture_success_signal
    固定发送 ``(frame_id, CameraFrame.as_dict())``。
    """

    adapter_name: str
    log_signal: Any
    init_signal: Any
    capture_success_signal: Any
    capture_error_signal: Any
    param_signal: Any

    def start(self) -> None: ...
    def is_running(self) -> bool: ...
    def is_connected(self) -> bool: ...
    def shutdown(self) -> None: ...
    def trigger(self) -> bool: ...
    def get_intrinsics(self) -> Optional[Tuple[np.ndarray, np.ndarray]]: ...
    def get_parameters(self) -> Dict[str, Any]: ...
    def set_parameter(self, name: str, value: Any) -> bool: ...
    def list_profiles(self) -> list[str]: ...
    def current_profile(self) -> str: ...
    def select_profile(self, name: str) -> bool: ...
    def save_parameters(self) -> bool: ...


@runtime_checkable
class SyncCameraAdapter(Protocol):
    """供自动采集脚本使用的同步相机接口。"""

    adapter_name: str

    def connect(self, endpoint: Optional[str] = None) -> None: ...
    def disconnect(self) -> None: ...
    def is_connected(self) -> bool: ...
    def capture(self) -> CameraFrame: ...
    def get_intrinsics(self) -> Optional[Tuple[np.ndarray, np.ndarray]]: ...


def assert_robot_adapter(adapter: Any) -> None:
    required = ("connect", "disconnect", "is_connected", "get_tcp_pose",
                "get_tcp_matrix", "move_linear")
    missing = [name for name in required if not callable(getattr(adapter, name, None))]
    if missing:
        raise TypeError(f"机器人适配器缺少接口: {missing}")


def assert_async_camera_adapter(adapter: Any) -> None:
    required = ("start", "is_running", "is_connected", "shutdown", "trigger",
                "get_intrinsics", "get_parameters", "set_parameter",
                "list_profiles", "current_profile", "select_profile", "save_parameters")
    signals = ("log_signal", "init_signal", "capture_success_signal",
               "capture_error_signal", "param_signal")
    missing = [name for name in required if not callable(getattr(adapter, name, None))]
    missing += [name for name in signals if not hasattr(adapter, name)]
    if missing:
        raise TypeError(f"异步相机适配器缺少接口: {missing}")


def assert_sync_camera_adapter(adapter: Any) -> None:
    required = ("connect", "disconnect", "is_connected", "capture", "get_intrinsics")
    missing = [name for name in required if not callable(getattr(adapter, name, None))]
    if missing:
        raise TypeError(f"同步相机适配器缺少接口: {missing}")
