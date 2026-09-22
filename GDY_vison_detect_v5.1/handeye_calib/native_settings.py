"""Validated v5.1 settings; no SDK imports or camera side effects."""
import math


DEFAULTS = {
    "capture_timeout_ms": 3000,
    "preview_read_timeout_ms": 100,
    "startup_quiet_reads": 3,
    "startup_max_reads": 60,
    "memory_limit_mb": 2048,
    "close_timeout_s": 8,
    "rgb_exposure_us": None,
    "rgb_gain": None,
    "depth_exposure": None,
    "depth_frame_time": None,
    "depth_gain": None,
    "match_enabled": False,
    "match_threshold_ms": 60,
    "rgb_offset_ms": 0,
    "rgb_after_depth": False,
    "timestamp_policy": "warn",
    "preview_with_depth": False,
    "rgb_read_timeout_ms": 500,
    "drain_timeout_ms": 100,
    "drain_max_frames": 10,
    "periodic_clear_s": 60,
    "memory_report_s": 300,
}

# Fixed numeric private wire format, parsed in C++ with strict bounds. The order MUST stay in
# step with Settings::parse() in handeye_calib/native_camera/worker.cpp.
WIRE_KEYS = ("capture_timeout_ms", "preview_read_timeout_ms", "startup_quiet_reads",
             "startup_max_reads", "rgb_exposure_us", "rgb_gain", "depth_exposure",
             "depth_frame_time", "depth_gain", "match_enabled", "match_threshold_ms",
             "rgb_offset_ms", "rgb_after_depth", "timestamp_policy", "preview_with_depth",
             "rgb_read_timeout_ms", "drain_timeout_ms", "drain_max_frames",
             "periodic_clear_s", "memory_report_s")


def native_settings(value=None):
    if value is not None and not isinstance(value, dict):
        raise ValueError("camera.native must be a mapping")
    unknown = set(value or {}) - set(DEFAULTS)
    if unknown:
        raise ValueError(f"unknown camera.native settings: {sorted(unknown)}")
    result = {**DEFAULTS, **(value or {})}
    bounds = {"capture_timeout_ms": (100, 60000), "preview_read_timeout_ms": (10, 500),
              "startup_quiet_reads": (1, 10), "startup_max_reads": (3, 60),
              "memory_limit_mb": (128, 8192), "close_timeout_s": (1, 30),
              "match_threshold_ms": (1, 10000), "rgb_offset_ms": (-10000, 10000),
              "rgb_read_timeout_ms": (50, 60000), "drain_timeout_ms": (1, 1000),
              "drain_max_frames": (1, 64), "periodic_clear_s": (0, 3600),
              "memory_report_s": (0, 3600)}
    for key, (low, high) in bounds.items():
        v = result[key]
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or int(v) != v or not low <= v <= high:
            raise ValueError(f"camera.native.{key} must be an integer in [{low}, {high}]")
        result[key] = int(v)
    for key in ("rgb_exposure_us", "rgb_gain", "depth_exposure", "depth_frame_time", "depth_gain"):
        v = result[key]
        if v is not None and (isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v) or not 0 < v <= 10000000):
            raise ValueError(f"camera.native.{key} must be null or a positive finite number")
    if result["rgb_exposure_us"] is not None and int(result["rgb_exposure_us"]) != result["rgb_exposure_us"]:
        raise ValueError("rgb_exposure_us must be an integer")
    for key in ("match_enabled", "rgb_after_depth", "preview_with_depth"):
        if not isinstance(result[key], bool):
            raise ValueError(f"camera.native.{key} must be boolean")
    if result["timestamp_policy"] not in ("warn", "strict"):
        raise ValueError("timestamp_policy must be warn or strict")
    if result["startup_max_reads"] < result["startup_quiet_reads"]:
        raise ValueError("startup_max_reads must be >= startup_quiet_reads")
    exp, period = result["depth_exposure"], result["depth_frame_time"]
    if exp is not None and period is not None and exp >= period:
        raise ValueError("depth_exposure must be < depth_frame_time")
    return result


def wire_settings(settings):
    values = []
    for key in WIRE_KEYS:
        value = settings[key]
        if key == "timestamp_policy":
            value = int(value == "strict")
        elif value is None:
            value = -1
        elif isinstance(value, bool):
            value = int(value)
        values.append(str(value))
    return " ".join(values)
