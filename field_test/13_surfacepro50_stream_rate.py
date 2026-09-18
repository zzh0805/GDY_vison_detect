# -*- coding: utf-8 -*-
"""SurfacePro50 原始流速率诊断，不执行配准、点云或YOLO。

在目标机分别运行：
    python field_test/13_surfacepro50_stream_rate.py --mode color-only
    python field_test/13_surfacepro50_stream_rate.py --mode color-depth

若 color-only 明显快、color-depth 明显变慢，主要是双流带宽/驱动竞争；
若两种模式的彩色都慢，再重点检查曝光和相机彩色管线。
"""
from __future__ import annotations

import argparse
import contextlib
import io
import sys
import time
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from handeye_calib.surfacepro50_adapter import SurfacePro50Backend  # noqa: E402


def _snapshot(backend: SurfacePro50Backend):
    with backend._frame_condition:
        image = backend._latest_image_raw
        depth = backend._latest_depth_raw
        return {
            "image_sequence": backend._image_raw_sequence,
            "depth_sequence": backend._depth_raw_sequence,
            "image_marker": None if image is None else (
                image.get("frameIndex"), image.get("timestamp")),
            "depth_marker": None if depth is None else (
                depth.get("frameIndex"), depth.get("timestamp")),
            "image_arrival": None if image is None else
                image.get("arrival_monotonic"),
            "depth_arrival": None if depth is None else
                depth.get("arrival_monotonic"),
            "last_error": backend._frame_pump_last_error,
        }


def _direct_rate_test(backend: SurfacePro50Backend, seconds: float) -> int:
    """兼容没有后台最新帧缓存的旧版适配器。"""
    streams = [(backend.image_stream, "color")]
    if (backend.depth_stream is not None and
            backend.depth_stream is not backend.image_stream):
        streams.append((backend.depth_stream, "depth"))
    wait = getattr(backend.openni2, "wait_for_any_stream", None)
    if not callable(wait):
        raise RuntimeError("当前OpenNI Python包缺少wait_for_any_stream")

    counts = {name: 0 for _, name in streams}
    previous_counts = dict(counts)
    markers = {name: None for _, name in streams}
    arrivals = {name: None for _, name in streams}
    start = time.monotonic()
    next_report = start + 1.0
    tick = 0
    print("检测到旧版适配器：改用直接OpenNI原始流测速。")
    print("秒  彩色新增  深度新增  彩色帧号/时间戳  深度帧号/时间戳  "
          "彩色帧龄(ms)  深度帧龄(ms)")
    while time.monotonic() - start < seconds:
        ready = wait([stream for stream, _ in streams], timeout=0.2)
        if ready is not None:
            if isinstance(ready, int):
                ready = streams[int(ready)][0]
            name = next(
                (stream_name for stream, stream_name in streams
                 if ready is stream), None)
            if name is None:
                raise RuntimeError("wait_for_any_stream返回未知流")
            captured = io.StringIO()
            with contextlib.redirect_stdout(captured):
                frame = ready.read_frame()
            counts[name] += 1
            markers[name] = (
                getattr(frame, "frameIndex", None),
                getattr(frame, "timestamp", None))
            arrivals[name] = time.monotonic()
            del frame

        now = time.monotonic()
        if now < next_report:
            continue
        tick += 1

        def delta(name: str) -> int:
            return counts.get(name, 0) - previous_counts.get(name, 0)

        def age_ms(name: str) -> str:
            arrival = arrivals.get(name)
            return "-" if arrival is None else f"{(now-arrival)*1000.0:.1f}"

        print(
            f"{tick:>2}  {delta('color'):>8}  {delta('depth'):>8}  "
            f"{str(markers.get('color')):>17}  "
            f"{str(markers.get('depth')):>17}  "
            f"{age_ms('color'):>12}  {age_ms('depth'):>12}")
        previous_counts = dict(counts)
        next_report += 1.0

    elapsed = time.monotonic() - start
    print(f"结果: 彩色={counts.get('color', 0)/elapsed:.3f} fps "
          f"深度={counts.get('depth', 0)/elapsed:.3f} fps "
          f"统计时长={elapsed:.1f}s")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="SurfacePro50原始流速率诊断")
    parser.add_argument("--ip", default="192.168.16.122")
    parser.add_argument(
        "--mode", choices=("color-only", "color-depth"),
        default="color-depth")
    parser.add_argument("--seconds", type=float, default=15.0)
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds必须大于0")

    backend = SurfacePro50Backend(
        endpoint=args.ip, capture_3d=args.mode == "color-depth")
    try:
        backend.connect()
        print(f"mode={args.mode}")
        print(f"color_profile={backend.image_profile}")
        print(f"depth_profile={backend.depth_profile or 'disabled'}")
        print(f"parameters={backend.get_parameters()}")
        if not hasattr(backend, "_frame_condition"):
            return _direct_rate_test(backend, args.seconds)
        print("秒  彩色新增  深度新增  彩色帧号/时间戳  深度帧号/时间戳  "
              "彩色帧龄(ms)  深度帧龄(ms)")
        start = time.monotonic()
        previous = _snapshot(backend)
        total_image_start = previous["image_sequence"]
        total_depth_start = previous["depth_sequence"]
        tick = 0
        while time.monotonic() - start < args.seconds:
            time.sleep(1.0)
            tick += 1
            now = time.monotonic()
            current = _snapshot(backend)

            def age_ms(name: str):
                arrival = current[name]
                return "-" if arrival is None else f"{(now-arrival)*1000.0:.1f}"

            print(
                f"{tick:>2}  "
                f"{current['image_sequence']-previous['image_sequence']:>8}  "
                f"{current['depth_sequence']-previous['depth_sequence']:>8}  "
                f"{str(current['image_marker']):>17}  "
                f"{str(current['depth_marker']):>17}  "
                f"{age_ms('image_arrival'):>12}  "
                f"{age_ms('depth_arrival'):>12}")
            if current["last_error"]:
                print(f"后台错误: {current['last_error']}")
            previous = current

        elapsed = time.monotonic() - start
        final = _snapshot(backend)
        image_fps = (
            final["image_sequence"] - total_image_start) / elapsed
        depth_fps = (
            final["depth_sequence"] - total_depth_start) / elapsed
        print(f"结果: 彩色={image_fps:.3f} fps 深度={depth_fps:.3f} fps "
              f"统计时长={elapsed:.1f}s")
        return 0
    finally:
        backend.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
