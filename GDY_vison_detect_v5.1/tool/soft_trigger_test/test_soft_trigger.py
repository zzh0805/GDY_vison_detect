"""Linux native SDK depth / paired RGB-depth diagnostic, independent of the service."""
import argparse
import csv
import ctypes as C
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback


class CallTrace:
    """File IPC survives a blocked ctypes call; parent enforces the deadline."""
    def __init__(self, output, call_timeout=15, open_timeout=90):
        self.output = output
        self.call_timeout = call_timeout
        self.open_timeout = open_timeout
        self.sequence = 0

    def publish(self, state):
        temp = self.output / "call_state.tmp"
        temp.write_text(json.dumps(state, ensure_ascii=False), encoding="utf-8")
        os.replace(temp, self.output / "call_state.json")
        with (self.output / "calls.jsonl").open("a", encoding="utf-8") as f:
            f.write(json.dumps(state, ensure_ascii=False) + "\n")

    def run(self, name, function, limit=None):
        self.sequence += 1
        start = time.perf_counter()
        if limit is None:
            limit = self.open_timeout if name in ("load_library", "open") else self.call_timeout
        state = {"sequence": self.sequence, "operation": name, "phase": "begin",
                 "started": start, "updated": start, "deadline": start + limit}
        self.publish(state)
        print(f"CALL {self.sequence} BEGIN {name} watchdog={limit}s", flush=True)
        try:
            value = function()
        except BaseException as exc:
            state.update(phase="error", error=str(exc), updated=time.perf_counter())
            self.publish(state)
            print(f"CALL {self.sequence} ERROR {name}: {exc}", flush=True)
            raise
        state.update(phase="end", updated=time.perf_counter())
        if isinstance(value, tuple) and value and isinstance(value[0], int):
            state["result_code"] = value[0]  # 0 frame, 1 SDK timeout
        state["elapsed_ms"] = round((state["updated"]-start)*1000, 3)
        self.publish(state)
        print(f"CALL {self.sequence} END {name} {state['elapsed_ms']}ms rc={state.get('result_code', 'ok')}", flush=True)
        return value


def watchdog_reason(state, now, idle_limit):
    if state.get("phase") == "begin":
        if now > state["deadline"]:
            return f"单次调用超时: {state['operation']} sequence={state['sequence']}"
        return None
    if now - state.get("updated", now) > idle_limit:
        return "工作进程长时间无调用进展"
    return None


class TracedCamera:
    def __init__(self, camera, trace, paired):
        self.camera, self.trace, self.paired = camera, trace, paired
        self.last_read = None

    def __getattr__(self, name):
        return getattr(self.camera, name)

    def read(self, timeout):
        name = "getPairedFrame" if self.paired else "getFrame(depth)"
        self.last_read = {"operation": name, "timeout_ms": timeout, "returned": False}
        result = self.trace.run(f"{name}(timeout_ms={timeout})", lambda: self.camera.read(timeout))
        self.last_read.update(returned=True, depth=result,
                              rgb=self.camera.last_rgb if self.paired else None)
        return result


def record_failure(output, filename, exc, context):
    # Write and print BEFORE cleanup, so blocked cleanup cannot hide the cause.
    report = {"exception_type": type(exc).__name__, "message": str(exc),
              "traceback": traceback.format_exc(), "context": context}
    print(f"FAILURE {filename}: {report['traceback']}", flush=True)
    try:
        (output / filename).write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    except Exception as write_error:
        print(f"无法保存异常文件: {write_error}", flush=True)


def memory_kib(pid):
    values = {}
    for line in Path(f"/proc/{pid}/status").read_text().splitlines():
        key, _, value = line.partition(":")
        if key in ("VmRSS", "RssAnon", "RssFile", "VmSwap", "VmHWM", "Threads"):
            values[key] = int(value.split()[0])
    return values


class ContentChanges:
    """Observed RGB byte changes, not exposure freshness or semantic scene changes."""
    def __init__(self):
        self.previous = None
        self.last_change = None
        self.last_seen = None
        self.changes = 0
        self.samples = 0
        self.max_unchanged = 0.0
        self.interval_sum = 0.0
        self.max_interval = 0.0

    def update(self, fingerprint, now):
        interval = None
        changed = self.previous is not None and fingerprint != self.previous
        if self.last_change is None:
            self.last_change = now
        else:
            span = now - self.last_change
            self.max_unchanged = max(self.max_unchanged, span)
            if changed:
                interval = span
                self.changes += 1
                self.interval_sum += span
                self.max_interval = max(self.max_interval, span)
                self.last_change = now
        self.previous, self.last_seen = fingerprint, now
        self.samples += 1
        return changed, interval

    def summary(self):
        return {"samples": self.samples, "content_changes": self.changes,
                "mean_observed_change_interval_s": self.interval_sum/self.changes if self.changes else None,
                "max_observed_change_interval_s": self.max_interval if self.changes else None,
                "max_observed_unchanged_span_s": self.max_unchanged,
                "trailing_unchanged_span_s": self.last_seen-self.last_change if self.samples else 0,
                "method": "full RGB8 FNV-1a 64-bit; host receipt time; noise also counts as change"}


class Camera:
    def __init__(self, lib):
        self.lib = C.CDLL(str(Path(lib).resolve()))
        self.lib.st_error.argtypes = []
        self.lib.st_error.restype = C.c_char_p
        self.lib.st_open.argtypes = [C.c_char_p, C.c_int, C.c_int]
        self.lib.st_open.restype = C.c_int
        for name in ("st_close", "st_trigger"):
            getattr(self.lib, name).argtypes = []
            getattr(self.lib, name).restype = C.c_int
        self.lib.st_read.argtypes = [C.c_int, C.POINTER(C.c_double)] + [C.POINTER(C.c_int)] * 3
        self.lib.st_read.restype = C.c_int
        self.lib.st_set_rgb_profile.argtypes = [C.c_int]
        self.lib.st_set_rgb_profile.restype = None
        self.lib.st_read_pair.argtypes = self.lib.st_read.argtypes
        self.lib.st_read_pair.restype = C.c_int
        self.lib.st_rgb_hash.argtypes = []
        self.lib.st_rgb_hash.restype = C.c_uint64
        self.lib.st_set_cleanup_log.argtypes = [C.c_char_p]
        self.lib.st_set_cleanup_log.restype = None
        self.paired = False
        self.last_rgb = None
        self.last_rgb_hash = None

    def configure(self, streams, rgb_profile):
        self.paired = streams == "rgbd"
        self.lib.st_set_rgb_profile(rgb_profile if self.paired else -1)

    def check(self, rc):
        if rc < 0:
            raise RuntimeError(self.lib.st_error().decode("utf-8", errors="replace"))
        return rc

    def read(self, timeout):
        if self.paired:
            stamps = (C.c_double * 2)()
            sizes, widths, heights = (C.c_int * 2)(), (C.c_int * 2)(), (C.c_int * 2)()
            start = time.monotonic()
            rc = self.check(self.lib.st_read_pair(timeout, stamps, sizes, widths, heights))
            self.last_rgb = (stamps[1], sizes[1], widths[1], heights[1]) if rc == 0 else None
            self.last_rgb_hash = f"{self.lib.st_rgb_hash():016x}" if rc == 0 else None
            return (rc, stamps[0], sizes[0], widths[0], heights[0], round((time.monotonic()-start)*1000, 3))
        stamp, size, width, height = C.c_double(), C.c_int(), C.c_int(), C.c_int()
        start = time.monotonic()
        rc = self.check(self.lib.st_read(timeout, C.byref(stamp), C.byref(size), C.byref(width), C.byref(height)))
        return (rc, stamp.value, size.value, width.value, height.value, round((time.monotonic()-start)*1000, 3))


def verify_quiet(cam, timeout=500, required=3, drain_limit=100):
    """Bounded startup drain, then require several consecutive no-trigger timeouts."""
    quiet = drained = 0
    while quiet < required:
        if cam.read(timeout)[0] == 1:
            quiet += 1
        else:
            drained += 1
            quiet = 0
            if drained >= drain_limit:
                raise RuntimeError("不触发仍持续出帧：软触发未验证通过（或启动积压超出排空上限）")
    return drained


def set_phase(output, name):
    temp = output / "phase.tmp"
    temp.write_text(name, encoding="utf-8")
    os.replace(temp, output / "phase.txt")
    print(f"PHASE {name}", flush=True)


def idle_phase(output, trace, name, seconds):
    if seconds <= 0:
        return
    set_phase(output, name)
    print(f"空闲 {seconds}s：保持连接，不触发、不读帧、不排空缓存。", flush=True)
    trace.run(name, lambda: time.sleep(seconds), limit=seconds + 10)


def warmup_capture(cam, trace, args, paired, context):
    required = getattr(args, "warmup_frames", 0)
    if not required:
        return
    set_phase(args.output, "warmup")
    context.update(stage="warmup", accepted_frames=0)
    with (args.output / "warmup.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["frame", "depth_timestamp_ms", "depth_bytes", "depth_width", "depth_height",
                         "read_ms", "rgb_timestamp_ms", "rgb_bytes", "rgb_width", "rgb_height"])
        f.flush()
        for i in range(required):
            context["attempt"] = i + 1
            if args.mode == "soft":
                trace.run("softTrigger(1) warmup", lambda: cam.check(cam.lib.st_trigger()))
            frame = cam.read(args.timeout_ms)
            if frame[0] != 0:
                raise RuntimeError(f"预采集第{i+1}组超时，未进入空闲阶段；不能判断空闲恢复能力")
            rgb = cam.last_rgb if paired else None
            if frame[2] <= 0 or (paired and (rgb is None or rgb[1] <= 0)):
                raise RuntimeError("预采集数据无效，未进入空闲阶段")
            writer.writerow([i + 1, *frame[1:], *(rgb or ("", "", "", ""))])
            f.flush()
            context["accepted_frames"] = i + 1
            print(f"WARMUP {i+1}/{required} 有效{'彩深组' if paired else '深度帧'}", flush=True)
            if args.mode == "soft":
                # Reject unsolicited pairs, including after the last warmup trigger.
                verify_quiet(cam, timeout=500, required=2, drain_limit=1)
    print("预采集通过，即将进入真正不触发、不读取的空闲阶段。", flush=True)


def worker(args):
    set_phase(args.output, "initializing")
    trace = CallTrace(args.output, getattr(args, "call_timeout", 15), getattr(args, "open_timeout", 90))
    cam = trace.run("load_library", lambda: Camera(args.lib))
    cam.lib.st_set_cleanup_log(str(args.output / "cleanup.log").encode("utf-8"))
    paired = getattr(args, "streams", "depth") == "rgbd"
    cam.configure("rgbd" if paired else "depth", getattr(args, "rgb_profile", 0))
    cam = TracedCamera(cam, trace, paired)
    primary_error = None
    frame_context = {"stage": "open"}
    try:
        trace.run("open", lambda: cam.check(cam.lib.st_open(args.ip.encode(), int(args.mode == "soft"), args.profile)))
        if args.mode == "soft":
            n = verify_quiet(cam)
            print(f"不触发检查通过；排空启动旧帧 {n} 张。", flush=True)
        warmup_capture(cam, trace, args, paired, frame_context)
        frame_context = {"stage": "idle_before"}
        idle_phase(args.output, trace, "idle_before", getattr(args, "idle_before", 0))
        set_phase(args.output, "capture")
        start = time.monotonic()
        count = 0
        previous = None
        previous_rgb = None
        zero_timestamp_pairs = 0
        rgb_timestamp_warnings = 0
        changes = ContentChanges()
        with (args.output / "frames.csv").open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["frame", "elapsed_s", "timestamp_ms", "bytes", "width", "height", "read_ms",
                             "rgb_timestamp_ms", "rgb_bytes", "rgb_width", "rgb_height", "rgb_minus_depth_ms",
                             "rgb_hash", "rgb_content_changed", "rgb_change_interval_s", "rgb_timestamp_warning"])
            f.flush()
            while time.monotonic() - start < args.duration:
                frame_context = {"stage": "capture", "attempt": count + 1, "accepted_frames": count,
                                 "previous_depth_timestamp": previous, "previous_rgb_timestamp": previous_rgb}
                if args.mode == "soft":
                    trace.run("softTrigger(1)", lambda: cam.check(cam.lib.st_trigger()))
                rc, stamp, size, width, height, latency = cam.read(args.timeout_ms)
                if rc == 1:
                    if count == 0 and getattr(args, "warmup_frames", 0) and getattr(args, "idle_before", 0):
                        raise RuntimeError("预采集已成功，但空闲后首次触发/取帧超时：恢复测试失败")
                    raise RuntimeError("触发/取帧超时；不能判定软触发可用或内存测试成功")
                if stamp > 0 and previous is not None and stamp <= previous:
                    raise RuntimeError(f"时间戳未递增: {previous} -> {stamp}，疑似旧帧或设备时钟变化")
                previous = stamp if stamp > 0 else None
                rgb = cam.last_rgb if paired else None
                delta = ""
                changed, interval, fingerprint, timestamp_warning = False, None, "", False
                if paired:
                    if rgb is None or rgb[1] <= 0:
                        raise RuntimeError("配对接口未提供有效彩色帧")
                    if rgb[0] > 0 and previous_rgb is not None and rgb[0] <= previous_rgb:
                        rgb_timestamp_warnings += 1
                        timestamp_warning = True
                        print(f"WARNING 彩色时间戳未递增: {previous_rgb} -> {rgb[0]}；继续内容变化测试", flush=True)
                    previous_rgb = rgb[0] if rgb[0] > 0 else None
                    fingerprint = str(cam.last_rgb_hash)
                    changed, interval = changes.update(fingerprint, time.perf_counter())
                    (args.output / "rgb_changes.json").write_text(json.dumps({
                        **changes.summary(), "timestamp_warnings": rgb_timestamp_warnings
                    }, ensure_ascii=False, indent=2), encoding="utf-8")
                    if changed:
                        print(f"RGB CONTENT CHANGE #{changes.changes}: 观察间隔={interval:.3f}s hash={fingerprint}", flush=True)
                    if stamp > 0 and rgb[0] > 0:
                        delta = rgb[0] - stamp
                    else:
                        zero_timestamp_pairs += 1
                count += 1
                if count == 1 and getattr(args, "warmup_frames", 0) and getattr(args, "idle_before", 0):
                    print("RESUME SUCCESS：空闲后首次触发已取得有效数据（不代表严格同步/绝对新鲜度）。", flush=True)
                writer.writerow([count, round(time.monotonic()-start, 3), stamp, size, width, height, latency,
                                 *(rgb or ("", "", "", "")), delta, fingerprint,
                                 changed if paired else "", interval if interval is not None else "", timestamp_warning])
                f.flush()
                if count == 1 or count % 50 == 0:
                    print(f"{args.mode}: frame={count} {width}x{height} bytes={size} timestamp={stamp} read={latency}ms", flush=True)
                    if rgb:
                        print(f"  RGB: {rgb[2]}x{rgb[3]} bytes={rgb[1]} timestamp={rgb[0]} delta_ms={delta if delta != '' else '未知'}", flush=True)
                if args.mode == "soft":
                    # Observe the entire inter-trigger interval: no hidden continuous stream.
                    until = time.monotonic() + args.interval
                    while time.monotonic() < until:
                        ms = max(1, min(500, int((until-time.monotonic())*1000)))
                        if cam.read(ms)[0] == 0:
                            raise RuntimeError("一次触发后收到额外帧；不能确认一触发一帧")
                elif args.interval:
                    time.sleep(args.interval)
        if count == 0:
            raise RuntimeError("未取得帧，测试无效")
        print(f"采集完成，共 {count} 帧。时间戳为0时无法证明绝对新鲜度。", flush=True)
        (args.output / "capture_summary.json").write_text(json.dumps({
            "streams": "rgbd" if paired else "depth", "frames_or_pairs": count,
            "elapsed_s": time.monotonic()-start, "pairs_with_missing_timestamp": zero_timestamp_pairs,
            "synchronization_proven": False,
            "rgb_content": changes.summary() if paired else None,
            "rgb_timestamp_warnings": rgb_timestamp_warnings,
            "note": "SDK返回配对不等于独立验证同时曝光；测试不含点云转换和YOLO。"
        }, ensure_ascii=False, indent=2), encoding="utf-8")
        idle_phase(args.output, trace, "idle_after", getattr(args, "idle_after", 0))
    except BaseException as exc:
        primary_error = exc
        record_failure(args.output, "worker_error.json", exc,
                       {**frame_context, "last_read": cam.last_read})
        raise
    finally:
        set_phase(args.output, "closing")
        try:
            trace.run("close", lambda: cam.check(cam.lib.st_close()))
        except BaseException as exc:
            record_failure(args.output, "cleanup_error.json", exc, {"primary_error": str(primary_error) if primary_error else None})
            if primary_error is None:
                raise
    print("已关闭流、恢复原触发模式并断开相机。", flush=True)
    set_phase(args.output, "closed")
    # Parent continues observing whether native allocations return after close.
    time.sleep(5)
    return 0


def monitor(args):
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / "settings.json").write_text(json.dumps(vars(args), default=str, indent=2), encoding="utf-8")
    cmd = [sys.executable, "-u", str(Path(__file__).resolve()), *sys.argv[1:], "--output", str(args.output), "--worker"]
    child = subprocess.Popen(cmd, start_new_session=True)
    started = time.monotonic()
    reason = None
    baseline = latest = None
    next_print = 0
    last_call = None
    forced_kill = False
    phase_memory = {}
    keys = ["VmRSS", "RssAnon", "RssFile", "VmSwap", "VmHWM", "Threads"]
    try:
        with (args.output / "memory.csv").open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["elapsed_s", "pid", *keys, "phase"])
            while child.poll() is None:
                elapsed = time.monotonic() - started
                try:
                    last_call = json.loads((args.output / "call_state.json").read_text(encoding="utf-8"))
                except FileNotFoundError:
                    pass
                if last_call:
                    reason = watchdog_reason(last_call, time.perf_counter(), max(30, args.interval + 15))
                elif elapsed > args.open_timeout:
                    reason = "工作进程启动后未发布调用状态"
                if reason:
                    print(reason, flush=True)
                    break
                try:
                    mem = memory_kib(child.pid)
                except (FileNotFoundError, ProcessLookupError):
                    break
                try:
                    phase = (args.output / "phase.txt").read_text(encoding="utf-8")
                except FileNotFoundError:
                    phase = "starting"
                writer.writerow([round(elapsed, 3), child.pid, *[mem.get(k, "") for k in keys], phase])
                if "VmRSS" in mem:
                    stats = phase_memory.setdefault(phase, {"samples": 0, "first_rss_kib": mem["VmRSS"],
                                                           "first_elapsed_s": elapsed, "max_rss_kib": 0})
                    stats.update(samples=stats["samples"]+1, last_rss_kib=mem["VmRSS"],
                                 last_elapsed_s=elapsed, max_rss_kib=max(stats["max_rss_kib"], mem["VmRSS"]),
                                 rss_delta_kib=mem["VmRSS"]-stats["first_rss_kib"])
                f.flush()
                if "VmRSS" in mem:
                    latest = mem
                    if elapsed >= 60 and baseline is None:
                        baseline = mem.copy()
                    if elapsed >= next_print:
                        print(f"[{elapsed:.0f}s] PID={child.pid} RSS={mem['VmRSS']/1024:.1f}MiB Anon={mem.get('RssAnon',0)/1024:.1f}MiB Swap={mem.get('VmSwap',0)/1024:.1f}MiB", flush=True)
                        next_print = elapsed + 10
                    if mem['VmRSS'] + mem.get('VmSwap', 0) > args.max_memory_mb * 1024:
                        reason = "触及 RSS+Swap 内存保护上限"
                if elapsed > args.duration + args.idle_before + args.idle_after + args.open_timeout * 2 + 120 + args.warmup_frames * (args.call_timeout * 2 + 5):
                    reason = "超过总测试时限（SDK可能阻塞）"
                if reason:
                    break
                time.sleep(1)
    except KeyboardInterrupt:
        reason = "用户中断"
    finally:
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=10)
            except subprocess.TimeoutExpired:
                forced_kill = True
                child.kill()
                child.wait()
                print("SDK阻塞，已强制终止独立测试进程；触发模式可能未恢复，请重连检查，必要时相机断电重启。")
    capture = None
    if (args.output / "capture_summary.json").exists():
        capture = json.loads((args.output / "capture_summary.json").read_text(encoding="utf-8"))
    passed = (not reason and child.returncode == 0 and capture is not None
              and capture.get("frames_or_pairs", 0) > 0
              and capture.get("elapsed_s", 0) >= args.duration)
    summary = {"exit_code": child.returncode, "stop_reason": reason,
               "phase_memory": phase_memory,
               "acquisition_test_passed": passed, "last_call_at_stop": last_call,
               "forced_kill": forced_kill, "capture": capture,
               "memory_at_60s_kib": baseline, "last_memory_kib": latest,
               "streams": args.streams,
               "note": "原生SDK独立测试；不能据此证明生产服务已解决泄漏或严格彩深同步。"}
    for filename in ("worker_error.json", "cleanup_error.json"):
        path = args.output / filename
        if path.exists():
            try:
                summary[filename] = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, ValueError):
                summary[filename] = "异常文件未完整写入，请查看终端输出"
    if (args.output / "cleanup.log").exists():
        summary["cleanup_tail"] = (args.output / "cleanup.log").read_text(encoding="utf-8", errors="replace").splitlines()[-12:]
    (args.output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"结果目录: {args.output}\n退出码={child.returncode} 原因={reason or '测试流程结束'}")
    print(f"采集流程验证通过={passed}（不代表长期稳定性或严格同步通过）")
    return 0 if passed else 1


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--ip", default="192.168.16.122")
    p.add_argument("--lib", default=str(Path(__file__).with_name("libsoft_trigger_test.so")))
    p.add_argument("--mode", choices=["soft", "continuous"], default="soft")
    p.add_argument("--duration", type=float, default=1200)
    p.add_argument("--idle-before", type=float, default=0, help="seconds connected without trigger/read before capture")
    p.add_argument("--warmup-frames", type=int, default=0, help="valid frames required before entering idle")
    p.add_argument("--idle-after", type=float, default=0, help="seconds connected without trigger/read after capture")
    p.add_argument("--streams", choices=["depth", "rgbd"], default="rgbd")
    p.add_argument("--rgb-profile", type=int, default=0, help="RGB8 profile index, printed at startup")
    p.add_argument("--interval", type=float, default=1, help="soft: no-trigger observation seconds; continuous: pause between reads")
    p.add_argument("--profile", type=int, default=0, help="Z16 stream profile index, same for both tests")
    p.add_argument("--timeout-ms", type=int, default=3000)
    p.add_argument("--call-timeout", type=float, default=15, help="parent watchdog seconds per read/trigger/close")
    p.add_argument("--open-timeout", type=float, default=90, help="parent watchdog seconds for library load / open")
    p.add_argument("--max-memory-mb", type=float, default=2048, help="child RSS+Swap guard, MiB")
    p.add_argument("--output", type=Path, default=Path(__file__).parent / "output" / time.strftime("%Y%m%d_%H%M%S"))
    p.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    args = p.parse_args()
    if args.warmup_frames < 0:
        p.error("warmup-frames不能为负数")
    if args.idle_before < 0 or args.idle_after < 0:
        p.error("空闲时长不能为负数")
    if args.call_timeout <= args.timeout_ms / 1000 or args.open_timeout <= 0:
        p.error("call-timeout必须大于SDK取帧超时，open-timeout必须为正数")
    if sys.platform != "linux":
        p.error("此测试在Ubuntu上执行；本机可运行离线单元测试")
    if args.duration <= 0 or args.interval < 0 or args.timeout_ms <= 0 or args.max_memory_mb <= 0 or args.profile < 0 or args.rgb_profile < 0:
        p.error("duration/timeout/memory必须为正数，interval不可为负")
    if args.mode == "soft" and args.interval < 0.1:
        p.error("soft模式interval至少0.1秒，用于不触发检查")
    args.output = args.output.resolve()
    return worker(args) if args.worker else monitor(args)


if __name__ == "__main__":
    raise SystemExit(main())
