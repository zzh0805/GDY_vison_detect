"""Record the service's continuous RGB preview. Never opens a camera or moves a robot."""
import argparse
import json
import math
from pathlib import Path
import sys
import time
from datetime import datetime
from urllib.request import Request, urlopen

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from vision_solver.preview import preview_settings


def post(base, path, data=None):
    request = Request(base + path, data=json.dumps(data or {}).encode(),
                      headers={"Content-Type": "application/json"}, method="POST")
    with urlopen(request, timeout=110) as response:
        result = json.load(response)
    if result.get("code") != 200:
        raise RuntimeError(result)
    return result


def read_frame(stream):
    line = stream.readline(1025)
    while line in (b"\r\n", b"\n"):
        line = stream.readline(1025)
    if line.strip() != b"--frame":
        raise RuntimeError("preview stream ended or invalid multipart boundary")
    headers = {}
    for _ in range(16):
        line = stream.readline(1025)
        if line == b"\r\n":
            break
        if not line or len(line) > 1024 or b":" not in line:
            raise RuntimeError("invalid preview header")
        name, value = line.decode("ascii").split(":", 1)
        headers[name.lower()] = value.strip()
    else:
        raise RuntimeError("too many preview headers")
    size = int(headers.get("content-length", 0))
    if not 0 < size <= 16 * 1024 * 1024:
        raise RuntimeError("invalid preview JPEG size")
    data = bytearray()
    while len(data) < size:
        chunk = stream.read(size - len(data))
        if not chunk:
            raise RuntimeError("truncated preview JPEG")
        data.extend(chunk)
    frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        raise RuntimeError("preview JPEG decode failed")
    return frame, headers


class TimelineWriter:
    """Constant-rate MP4 timeline, holding ONE image; duplicate frames are counted."""
    def __init__(self, path, fps, codec):
        self.path, self.fps, self.codec = str(path), fps, codec
        self.writer = None
        self.latest = None
        self.origin = None
        self.written = 0
        self.source_frames = 0
        self.duplicates = 0
        self._latest_written = False

    def _fill(self, elapsed):
        target = max(1, int(math.floor(elapsed * self.fps)) + 1)
        while self.written < target:
            self.writer.write(self.latest)
            self.duplicates += int(self._latest_written)
            self._latest_written = True
            self.written += 1

    def add(self, frame, now):
        if self.writer is None:
            h, w = frame.shape[:2]
            self.writer = cv2.VideoWriter(self.path, cv2.VideoWriter_fourcc(*self.codec), self.fps, (w, h))
            if not self.writer.isOpened():
                self.writer.release()
                self.writer = None
                raise RuntimeError(f"MP4编码器不可用: {self.codec}；检查OpenCV/FFmpeg安装")
            self.origin = now
            self.latest = frame
        else:
            if frame.shape != self.latest.shape:
                raise RuntimeError("preview resolution changed during recording")
            self._fill(now - self.origin)
            self.latest = frame
        self._latest_written = False
        self.source_frames += 1
        if self.written == 0:
            self._fill(0)

    def close(self, now):
        if self.writer is not None:
            try:
                self._fill(max(0, now - self.origin))
            finally:
                self.writer.release()
                self.writer = None


def main():
    parser = argparse.ArgumentParser(description="连续RGB预览录制MP4，不触发拍照、不控制机械臂")
    parser.add_argument("--config", default=str(ROOT / "config/workflow.yaml"))
    parser.add_argument("--base-url", help="跨设备时填写视觉服务地址，例如http://192.168.16.100:48051")
    parser.add_argument("--seconds", type=float)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    source = Path(args.config).resolve()
    config = yaml.safe_load(source.read_text(encoding="utf-8-sig")) or {}
    cfg = preview_settings(config.get("preview"))
    seconds = cfg["record_seconds"] if args.seconds is None else args.seconds
    if not math.isfinite(seconds) or not 0 < seconds <= cfg["max_session_s"]:
        raise ValueError("seconds必须>0且不超过preview.max_session_s")
    http = config.get("http", {})
    host = http.get("listen_host", "127.0.0.1")
    if host in ("0.0.0.0", "::"):
        host = "127.0.0.1"
    base = (args.base_url or f"http://{host}:{http.get('listen_port', 48051)}").rstrip("/")
    directory = (source.parent / cfg["output_directory"]).resolve()
    output = args.output or directory / (datetime.now().strftime("rgb_%Y%m%d_%H%M%S_%f") + ".mp4")
    output = output.resolve()
    if output.suffix.lower() != ".mp4" or output.exists() or output.with_suffix(".json").exists():
        raise ValueError("output必须是尚不存在的.mp4文件，禁止覆盖原录像")
    output.parent.mkdir(parents=True, exist_ok=True)
    writer = TimelineWriter(output, cfg["record_fps"], cfg["record_codec"])
    session = None
    received = 0
    failure = None
    started = last_print = time.monotonic()
    last_count = 0
    acquisition_fps = 0
    try:
        session = post(base, "/preview/start")
        print(f"连续预览已开启，录制 {seconds}s → {output}", flush=True)
        print(f"MP4时间轴={cfg['record_fps']}fps；不足时重复帧补时长，不代表相机帧率。", flush=True)
        started = last_print = time.monotonic()
        with urlopen(base + "/preview/rgb.mjpg", timeout=10) as stream:
            while time.monotonic() - started < seconds:
                frame, headers = read_frame(stream)
                now = time.monotonic()
                if now - started > seconds:
                    break
                received += 1
                acquisition_fps = float(headers.get("x-acquisition-fps", 0))
                writer.add(frame, now)
                if now - last_print >= 1:
                    print(f"{now-started:6.1f}s SDK采集={acquisition_fps:.3f}fps "
                          f"接收={(received-last_count)/(now-last_print):.3f}fps "
                          f"平均接收={received/(now-started):.3f}fps 帧数={received}", flush=True)
                    last_print, last_count = now, received
        if received == 0:
            raise RuntimeError("没有收到可录制的RGB帧")
    except KeyboardInterrupt:
        failure = "用户中断，已保留收到的帧"
    except Exception as exc:
        failure = str(exc)
    finally:
        ended = time.monotonic()
        try:
            writer.close(min(ended, started + seconds))
        except Exception as exc:
            failure = failure or f"MP4封口失败: {exc}"
        if session and session.get("started_new"):
            try:
                post(base, "/preview/stop", {"session": session["session"]})
            except Exception as exc:
                print(f"停止预览失败，请检查服务状态: {exc}", file=sys.stderr)
                failure = failure or str(exc)
        elapsed = max(.001, min(ended - started, seconds))
        stats = {"received_frames": received, "receive_fps": received / elapsed,
                 "last_sdk_acquisition_fps": acquisition_fps, "elapsed_s": elapsed,
                 "mp4_timeline_fps": cfg["record_fps"], "written_frames": writer.written,
                 "duplicate_frames": writer.duplicates, "error": failure}
        if received:
            output.with_suffix(".json").write_text(json.dumps(stats, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps(stats, ensure_ascii=False, indent=2))
    return 1 if failure else 0


if __name__ == "__main__":
    raise SystemExit(main())
