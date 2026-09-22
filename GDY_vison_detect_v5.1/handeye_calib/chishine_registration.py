# -*- coding: utf-8 -*-
"""知象原始深度到 RGB 的低内存软件配准算法（不负责相机取流）。"""
from __future__ import annotations

from typing import Any

import numpy as np


def scale_intrinsic(matrix: Any, calibration_width: int,
                    calibration_height: int, image_width: int,
                    image_height: int) -> np.ndarray:
    result = np.asarray(matrix, dtype=np.float64).reshape(3, 3).copy()
    result[0, :] *= image_width / float(calibration_width)
    result[1, :] *= image_height / float(calibration_height)
    return result


def reconstruct_organized_cloud_rgb_frame(
        depth_raw: np.ndarray, color_bgr: np.ndarray, depth_scale_mm: float,
        calibration: dict[str, Any], min_depth_mm: float,
        max_depth_mm: float, row_chunk: int = 64,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, dict[str, Any]]:
    """返回 (depth_mm, organized_rgb, depth_rgb_mm, colors_rgb, diagnostics)。

    外参公式与厂家 Processing.hpp::Pointcloud::insertPoints 一致：
    P_rgb = R_sdk_raw @ (P_depth + T)。输出为**与彩色图逐像素对齐**的
    有组织点云：
      - organized_rgb: H×W×3，像素 (v,u) 处为 RGB 光学坐标系下的最近三维
        点（毫米），无有效点处为 NaN；
      - depth_rgb_mm:  H×W，对齐彩色图的深度 z（毫米），无效处为 NaN；
      - colors_rgb:    H×W×3，与点云同位置的 RGB 颜色（无效点处为 0）。
    organized_rgb 可直接用于 YOLO 框 point_cloud[v, u] 索引与 ply_roi 裁剪。
    实现：深度像素前向投影到彩色图并用 z-buffer 保留最近深度，再按 RGB
    内参反投影生成 RGB 光学坐标系点云（不依赖深度像素排列）。
    """
    raw = np.asarray(depth_raw, dtype=np.uint16)
    if raw.ndim != 2:
        raise ValueError(f"深度图必须是二维数组，当前为 {raw.shape}")
    image = np.asarray(color_bgr, dtype=np.uint8)
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError(f"彩色图必须是 HxWx3 BGR，当前为 {image.shape}")
    dh, dw = raw.shape
    rh, rw = image.shape[:2]
    k_depth = scale_intrinsic(
        calibration["K_depth"], int(calibration["depth_width"]),
        int(calibration["depth_height"]), dw, dh)
    k_rgb = scale_intrinsic(
        calibration["K_rgb"], int(calibration["rgb_width"]),
        int(calibration["rgb_height"]), rw, rh)
    rotation = np.asarray(
        calibration["R_sdk_raw"], dtype=np.float32).reshape(3, 3)
    translation = np.asarray(
        calibration["T_depth_to_rgb"], dtype=np.float32).reshape(1, 3)
    depth_mm = raw.astype(np.float32) * np.float32(depth_scale_mm)
    valid_depth = (
        (raw > 0) & np.isfinite(depth_mm) &
        (depth_mm >= float(min_depth_mm)) &
        (depth_mm <= float(max_depth_mm)))
    capacity = int(np.count_nonzero(valid_depth))

    fx_d, fy_d = np.float32(k_depth[0, 0]), np.float32(k_depth[1, 1])
    cx_d, cy_d = np.float32(k_depth[0, 2]), np.float32(k_depth[1, 2])
    fx_c, fy_c = np.float32(k_rgb[0, 0]), np.float32(k_rgb[1, 1])
    cx_c, cy_c = np.float32(k_rgb[0, 2]), np.float32(k_rgb[1, 2])

    # RGB 像素网格上的最近深度（z-buffer，像素扁平索引）。
    rgb_z = np.full(rh * rw, np.inf, dtype=np.float32)
    block = max(1, int(row_chunk))
    for y0 in range(0, dh, block):
        y1 = min(dh, y0 + block)
        local_y, pixel_x = np.nonzero(valid_depth[y0:y1])
        if not len(pixel_x):
            continue
        pixel_y = local_y + y0
        z = depth_mm[pixel_y, pixel_x]
        points_depth = np.empty((len(z), 3), dtype=np.float32)
        points_depth[:, 0] = (pixel_x.astype(np.float32) - cx_d) * z / fx_d
        points_depth[:, 1] = (pixel_y.astype(np.float32) - cy_d) * z / fy_d
        points_depth[:, 2] = z
        points_rgb = (points_depth + translation) @ rotation.T
        z_rgb = points_rgb[:, 2]
        u = np.rint(fx_c * points_rgb[:, 0] / z_rgb + cx_c).astype(np.int32)
        v = np.rint(fy_c * points_rgb[:, 1] / z_rgb + cy_c).astype(np.int32)
        inside = (
            np.isfinite(points_rgb).all(axis=1) & (z_rgb > 0.0) &
            (u >= 0) & (u < rw) & (v >= 0) & (v < rh))
        if not np.any(inside):
            continue
        flat = (v[inside].astype(np.int64) * rw +
                u[inside].astype(np.int64))
        np.minimum.at(rgb_z, flat, z_rgb[inside])

    rgb_z = rgb_z.reshape(rh, rw)
    valid_rgb = np.isfinite(rgb_z) & (rgb_z > 0.0)
    # 从对齐深度反投影生成 RGB 光学坐标系点云（毫米）。
    organized = np.full((rh, rw, 3), np.nan, dtype=np.float32)
    vv, uu = np.nonzero(valid_rgb)
    zz = rgb_z[vv, uu]
    organized[vv, uu, 0] = (uu.astype(np.float32) - cx_c) * zz / fx_c
    organized[vv, uu, 1] = (vv.astype(np.float32) - cy_c) * zz / fy_c
    organized[vv, uu, 2] = zz
    depth_rgb = rgb_z.astype(np.float32, copy=True)
    depth_rgb[~valid_rgb] = np.nan
    colors_rgb = np.zeros((rh, rw, 3), dtype=np.uint8)
    colors_rgb[valid_rgb] = image[valid_rgb, ::-1]

    diagnostics = {
        "full_depth_points": capacity,
        "rgb_covered_pixels": int(valid_rgb.sum()),
        "rgb_cover_ratio": (
            float(valid_rgb.mean()) if rh and rw else 0.0),
        "unique_native_depth_codes": int(len(np.unique(raw[valid_depth]))),
        "K_depth_scaled": k_depth.tolist(),
        "K_rgb_scaled": k_rgb.tolist(),
        "rgb_distortion_applied": False,
        "extrinsics_equation": "P_rgb = R_sdk_raw @ (P_depth + T)",
        "point_cloud_coordinate_frame": "surfacepro50_color_optical",
        "point_cloud_pixel_aligned_to_image": True,
        "depth_resampled_to_image": True,
        "reconstruction_row_chunk": block,
    }
    return depth_mm, organized, depth_rgb, colors_rgb, diagnostics
