"""YOLO三维定位所需的最小化模块。"""

from .models import CameraModel, HandEyeCalibration

__all__ = ["CameraModel", "HandEyeCalibration", "YoloTargetDetector"]


def __getattr__(name):
    # 手眼/位姿单测不应因导入models而被迫安装OpenCV和Ultralytics。
    if name == "YoloTargetDetector":
        from .yolo_detector import YoloTargetDetector
        return YoloTargetDetector
    raise AttributeError(name)
