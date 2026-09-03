"""持续运行的知象相机目标TCP解算服务。"""

__all__ = ["VisionToolTcpSolver"]


def __getattr__(name):
    # 避免仅导入位姿数学模块时提前加载OpenCV/YOLO和相机依赖。
    if name == "VisionToolTcpSolver":
        from .api import VisionToolTcpSolver
        return VisionToolTcpSolver
    raise AttributeError(name)
