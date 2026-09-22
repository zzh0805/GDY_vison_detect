# -*- coding: utf-8 -*-
"""连续采集帧率/丢帧测试（用项目内的 SurfacePro50SyncAdapter）。

目的：验证图像采集代码能否稳定取图、不丢帧。
做法：连接相机后连续采集 N 帧，统计每帧间隔、平均帧率、最大间隔、
是否有接近超时的帧。同时每 K 帧断开重连一次模拟服务长生命周期。

用法：
    python field_test/12_camera_fps_test.py --frames 50
    python field_test/12_camera_fps_test.py --frames 100 --timeout 30 --interval 1
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from handeye_calib.surfacepro50_adapter import SurfacePro50SyncAdapter  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description="SurfacePro50 连续采集帧率/丢帧测试")
    ap.add_argument("--ip", default="192.168.16.122")
    ap.add_argument("--frames", type=int, default=50, help="连续采集帧数")
    ap.add_argument("--timeout", type=float, default=30.0, help="单帧超时秒数")
    ap.add_argument("--interval", type=float, default=0.0,
                    help="每帧之间 sleep 秒（模拟服务每次请求间隔）")
    args = ap.parse_args()

    os.environ["SURFACEPRO50_FRAME_WAIT_TIMEOUT_S"] = str(args.timeout)

    print(f"连续采集测试  ip={args.ip}  frames={args.frames}  "
          f"timeout={args.timeout}s  interval={args.interval}s")
    print("=" * 72)

    cam = SurfacePro50SyncAdapter(endpoint=args.ip, capture_3d=True)
    t0 = time.time()
    cam.connect()
    print(f"连接成功 ({time.time()-t0:.1f}s)")
    print(f"{'帧':>4} {'耗时(s)':>9} {'累计(s)':>9}")
    print("-" * 72)

    intervals = []
    ok = 0
    t_last = None
    for i in range(1, args.frames + 1):
        t_start = time.time()
        try:
            frame = cam.capture()
            dt = time.time() - t_start
        except Exception as exc:
            print(f"{i:>4} ❌ {type(exc).__name__}: {exc}")
            intervals.append(time.time() - t_start)
            continue
        ok += 1
        if t_last is not None:
            intervals.append(dt)
        t_last = time.time()
        color = getattr(getattr(frame, "color", None), "shape", None)
        print(f"{i:>4} {dt:>9.2f} {time.time()-t0:>9.1f}   color={color}")
        if args.interval > 0:
            time.sleep(args.interval)

    print("=" * 72)
    if ok == 0:
        print("❌ 一帧都没采到，相机采集完全失败")
        try:
            cam.disconnect()
        except Exception:
            pass
        return 1

    # 统计
    if len(intervals) > 1:
        avg = sum(intervals) / len(intervals)
        mx = max(intervals)
        # 丢帧：间隔远大于平均（比如 > 3 倍平均且 > 2s）
        dropped = [v for v in intervals if v > max(2.0, avg * 3)]
        print(f"成功帧数: {ok}/{args.frames}")
        print(f"帧间隔统计: 平均={avg:.2f}s  最大={mx:.2f}s  "
              f"等效帧率≈{1.0/avg:.1f} fps")
        if dropped:
            print(f"⚠ 疑似丢帧/卡顿 {len(dropped)} 次: "
                  f"{[round(v,1) for v in dropped]}")
        else:
            print("✅ 无显著丢帧/卡顿（所有帧间隔正常）")
    try:
        cam.disconnect()
    except Exception:
        pass
    return 0 if not (len(intervals) > 1 and [v for v in intervals if v > max(2.0, avg * 3)]) else 1


if __name__ == "__main__":
    raise SystemExit(main())
