# -*- coding: utf-8 -*-
"""
ply_io.py —— PLY / 深度图 / 图片 通用文件 I/O 工具模块

提供无状态、可复用的底层文件读写函数，供多个模块共用:

  - PLY 点云:      read_ply_header / read_ply_xyz / write_ply_xyz
  - 深度图:        read_depth   (.tiff/.tif 用 tifffile, 其余用 cv2)
  - 灰度图:        read_image   (cv2.imdecode, 兼容 Windows 中文路径)
  - 点云还原:      restore_organized_xyz (散点 PLY → H×W×3 有组织点云)

不包含任何 ROI 业务逻辑；纯计算部分见 ply_roi.py。
"""

from pathlib import Path

import cv2
import numpy as np

try:
    import tifffile
except ImportError:
    tifffile = None


# PLY 属性类型映射
PLY_TYPES = {
    "char": "i1", "int8": "i1", "uchar": "u1", "uint8": "u1",
    "short": "i2", "int16": "i2", "ushort": "u2", "uint16": "u2",
    "int": "i4", "int32": "i4", "uint": "u4", "uint32": "u4",
    "float": "f4", "float32": "f4", "double": "f8", "float64": "f8",
}


def read_ply_header(file) -> tuple:
    """解析 PLY 文件头, 返回 (format, vertex_count, vertex_props)。"""
    first = file.readline().decode("ascii", errors="strict").strip()
    if first != "ply":
        raise ValueError("不是PLY文件")
    fmt = None
    vertex_count = None
    vertex_props = []
    current_element = None
    while True:
        raw = file.readline()
        if not raw:
            raise ValueError("PLY头缺少end_header")
        line = raw.decode("ascii", errors="strict").strip()
        if not line or line.startswith("comment") or line.startswith("obj_info"):
            continue
        parts = line.split()
        if parts[0] == "format":
            fmt = parts[1]
        elif parts[0] == "element":
            current_element = parts[1]
            if current_element == "vertex":
                vertex_count = int(parts[2])
        elif parts[0] == "property" and current_element == "vertex":
            if parts[1] == "list":
                raise ValueError("不支持vertex中的list属性")
            vertex_props.append((parts[2], parts[1]))
        elif parts[0] == "end_header":
            break
    if fmt is None or vertex_count is None:
        raise ValueError("PLY头缺少format或vertex数量")
    names = [name for name, _ in vertex_props]
    for axis in ("x", "y", "z"):
        if axis not in names:
            raise ValueError(f"PLY缺少{axis}属性")
    return fmt, vertex_count, vertex_props


def read_ply_xyz(path: Path) -> np.ndarray:
    """读取 PLY 点云散点 (N×3), 支持 ascii 与二进制格式。"""
    with open(path, "rb") as file:
        fmt, count, props = read_ply_header(file)
        names = [name for name, _ in props]
        xyz_index = [names.index("x"), names.index("y"), names.index("z")]
        if fmt == "ascii":
            points = np.empty((count, 3), dtype=np.float32)
            for index in range(count):
                line = file.readline()
                if not line:
                    raise ValueError(f"ASCII PLY顶点不足，期望{count}个")
                values = line.split()
                points[index] = [float(values[i]) for i in xyz_index]
            return points
        if fmt not in ("binary_little_endian", "binary_big_endian"):
            raise ValueError(f"不支持的PLY格式:{fmt}")
        endian = "<" if fmt == "binary_little_endian" else ">"
        dtype_fields = []
        for name, type_name in props:
            if type_name not in PLY_TYPES:
                raise ValueError(f"不支持的PLY属性类型:{type_name}")
            dtype_fields.append((name, endian + PLY_TYPES[type_name]))
        data = np.fromfile(file, dtype=np.dtype(dtype_fields), count=count)
        if len(data) != count:
            raise ValueError(
                f"二进制PLY顶点不足，期望{count}个，实际{len(data)}个"
            )
        return np.column_stack(
            (data["x"], data["y"], data["z"])
        ).astype(np.float32, copy=False)


def write_ply_xyz(path: Path, points: np.ndarray) -> None:
    """将 N×3 散点写为二进制 PLY 文件 (过滤 NaN)。"""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    points = np.asarray(points, dtype="<f4")
    points = points[np.isfinite(points).all(axis=1)]
    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        f"element vertex {len(points)}\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "end_header\n"
    ).encode("ascii")
    with open(path, "wb") as file:
        file.write(header)
        points.tofile(file)


def read_depth(path: Path) -> np.ndarray:
    """读取单张深度图: .tiff/.tif 用 tifffile, 其他用 cv2。"""
    path = Path(path)
    is_tiff = path.suffix.lower() in (".tiff", ".tif")
    if is_tiff and tifffile is not None:
        return tifffile.imread(path)
    depth = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if depth is None:
        raise RuntimeError(
            f"读取深度文件失败: {path}"
            + (" (请安装 tifffile 或检查文件格式)" if is_tiff else "")
        )
    return depth


def read_image(path: Path) -> np.ndarray:
    """用 cv2 读取图片 (兼容 Windows 中文路径)。"""
    data = np.fromfile(str(path), dtype=np.uint8)
    image = cv2.imdecode(data, cv2.IMREAD_UNCHANGED)
    if image is None:
        raise ValueError(f"图片读取失败: {path}")
    return image


def restore_organized_xyz(
    points: np.ndarray,
    height: int,
    width: int,
    depth_path: Path,
) -> np.ndarray:
    """将散点 PLY 还原为 H×W×3 有组织点云 (借助深度图确定有效像素位置)。"""
    pixel_count = height * width
    if len(points) == pixel_count:
        return points.reshape(height, width, 3)
    depth_path = Path(depth_path)
    if not depth_path.exists():
        raise ValueError(
            f"PLY点数{len(points)}不等于图像像素数{pixel_count}，"
            f"且缺少{depth_path.name}，无法建立二维标注与点云的对应关系"
        )
    depth = np.asarray(read_depth(depth_path))
    if depth.ndim > 2:
        depth = depth.max(axis=2)
    if depth.shape[:2] != (height, width):
        raise ValueError(
            f"深度图尺寸{depth.shape[:2]}与图片尺寸{(height, width)}不一致"
        )
    valid = np.isfinite(depth)
    if np.issubdtype(depth.dtype, np.integer):
        valid &= depth != 0
    valid_count = int(valid.sum())
    if valid_count != len(points):
        raise ValueError(
            f"PLY点数{len(points)}与深度图有效像素数{valid_count}不一致"
        )
    xyz = np.full((pixel_count, 3), np.nan, dtype=np.float32)
    xyz[valid.reshape(-1)] = points
    return xyz.reshape(height, width, 3)


def load_point_cloud(path) -> np.ndarray:
    """加载点云文件 (.ply/.npy/.npz) 并返回连续的 N×3 float64 有效点数组。

    该函数是圆柱拟合的入口 I/O：任何 H×W×3 有组织点云都会被展平为
    无组织散点，仅保留 x/y/z 坐标，并剔除含 NaN/Inf 的点。
    """
    path = Path(path)
    suffix = path.suffix.lower()
    if suffix == ".ply":
        points = read_ply_xyz(path).astype(np.float64)
    elif suffix == ".npy":
        points = np.load(path, allow_pickle=False)
    elif suffix == ".npz":
        with np.load(path, allow_pickle=False) as data:
            if "xyz" in data:
                points = data["xyz"]
            elif "points" in data:
                points = data["points"]
            else:
                keys = list(data.keys())
                if not keys:
                    raise ValueError("NPZ文件中没有数组")
                points = data[keys[0]]
    else:
        raise ValueError("输入仅支持.ply、.npy、.npz")

    points = np.asarray(points)
    if points.ndim == 3 and points.shape[2] >= 3:
        points = points[..., :3].reshape(-1, 3)
    elif points.ndim == 2 and points.shape[1] >= 3:
        points = points[:, :3]
    else:
        raise ValueError(f"不支持的点云形状:{points.shape}")

    points = np.asarray(points, dtype=np.float64)
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) < 100:
        raise ValueError(f"有效点只有{len(points)}个，至少需要100个")
    return np.ascontiguousarray(points)


def write_ply_with_residual(
    path, points: np.ndarray, residual: np.ndarray, inlier: np.ndarray
) -> None:
    """将点云连同残差、内点标记写为二进制 PLY（附加 residual / inlier 属性）。"""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    points = np.asarray(points)
    residual = np.asarray(residual)
    inlier = np.asarray(inlier)
    dtype = np.dtype(
        [
            ("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
            ("residual", "<f4"), ("inlier", "u1"),
        ]
    )
    data = np.empty(len(points), dtype=dtype)
    data["x"] = points[:, 0]
    data["y"] = points[:, 1]
    data["z"] = points[:, 2]
    data["residual"] = residual
    data["inlier"] = inlier.astype(np.uint8)
    header = (
        "ply\nformat binary_little_endian 1.0\n"
        f"element vertex {len(points)}\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property float residual\nproperty uchar inlier\nend_header\n"
    ).encode("ascii")
    with open(path, "wb") as file:
        file.write(header)
        data.tofile(file)
