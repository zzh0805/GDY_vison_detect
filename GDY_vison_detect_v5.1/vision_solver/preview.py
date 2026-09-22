"""Bounded latest-frame JPEG publication. Uses the existing C++ camera owner."""
import math
import threading
import time
import uuid

import cv2


DEFAULTS = {"enabled": True, "publish_fps": 15, "jpeg_quality": 85,
            "max_clients": 2, "max_session_s": 600, "stale_after_s": 3,
            "socket_timeout_s": 10, "record_seconds": 30, "record_fps": 15,
            "record_codec": "mp4v", "output_directory": "../preview_recordings"}


def preview_settings(value=None):
    if value is not None and not isinstance(value, dict):
        raise ValueError("preview must be a mapping")
    if set(value or {}) - set(DEFAULTS):
        raise ValueError("unknown preview setting")
    cfg = {**DEFAULTS, **(value or {})}
    if not isinstance(cfg["enabled"], bool):
        raise ValueError("preview.enabled must be boolean")
    for key, low, high in (("publish_fps", .1, 120), ("jpeg_quality", 1, 100),
                           ("max_clients", 1, 16), ("max_session_s", 1, 86400),
                           ("stale_after_s", .1, 120), ("socket_timeout_s", 1, 120),
                           ("record_seconds", 1, 86400), ("record_fps", .1, 120)):
        v = cfg[key]
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or not low <= v <= high:
            raise ValueError(f"preview.{key} must be in [{low}, {high}]")
    for key in ("jpeg_quality", "max_clients"):
        if int(cfg[key]) != cfg[key]:
            raise ValueError(f"preview.{key} must be an integer")
        cfg[key] = int(cfg[key])
    if not isinstance(cfg["record_codec"], str) or len(cfg["record_codec"]) != 4:
        raise ValueError("preview.record_codec must be a four-character code")
    if not isinstance(cfg["output_directory"], str) or not cfg["output_directory"].strip():
        raise ValueError("preview.output_directory must be a nonempty path")
    return cfg


class PreviewService:
    def __init__(self, camera, config=None):
        self.camera = camera
        self.config = preview_settings(config)
        self.clients = threading.BoundedSemaphore(self.config["max_clients"])
        self._control = threading.Lock()
        self._condition = threading.Condition()
        self._stop = threading.Event()
        self._thread = None
        self._jpeg = None
        self._info = {}
        self._published_at = 0
        self._published = 0
        self._started = 0
        self._session = None
        self._error = None
        self._subscribers = 0

    def subscriber_added(self):
        """An MJPEG client started reading; frames must be turned into JPEG again."""
        with self._condition:
            self._subscribers += 1

    def subscriber_removed(self):
        with self._condition:
            self._subscribers = max(0, self._subscribers - 1)
            self._condition.notify_all()

    def start(self):
        with self._control:
            if not self.config["enabled"]:
                raise RuntimeError("preview disabled in workflow.yaml")
            if self._thread is not None and self._thread.is_alive():
                return {**self.status(), "started_new": False}
            client = self.camera.preview_client()
            client.preview_start()
            with self._condition:
                self._stop.clear()
                self._jpeg = None
                self._info = {}
                self._published = 0
                self._error = None
                self._started = time.monotonic()
                self._session = uuid.uuid4().hex
            self._thread = threading.Thread(target=self._run, args=(client,), name="rgb-preview", daemon=True)
            try:
                self._thread.start()
            except BaseException:
                self._thread = None
                self._stop.set()
                if client.is_connected():
                    client.preview_stop()
                raise
            return {**self.status(), "started_new": True}

    def _run(self, client):
        previous = None
        try:
            while not self._stop.is_set():
                begin = time.monotonic()
                if begin - self._started >= self.config["max_session_s"]:
                    break
                with self._condition:
                    watching = self._subscribers > 0
                # preview_frame() still runs while nobody is reading: it is what keeps a failing
                # worker visible in /preview/status. The 1080p JPEG encode below is the expensive
                # part and is pure waste without a reader, so it is skipped.
                rgb, info = client.preview_frame()
                key = (info.get("epoch"), info.get("sequence"))
                if watching and rgb is not None and key != previous and info.get("age_ms", 0) <= self.config["stale_after_s"] * 1000:
                    previous = key
                    ok, encoded = cv2.imencode(".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                                               [cv2.IMWRITE_JPEG_QUALITY, self.config["jpeg_quality"]])
                    if not ok:
                        raise RuntimeError("preview JPEG encoding failed")
                    with self._condition:
                        self._jpeg = encoded.tobytes()
                        self._info = info
                        self._published_at = time.monotonic()
                        self._published += 1
                        self._condition.notify_all()
                self._stop.wait(max(0, 1 / self.config["publish_fps"] - (time.monotonic() - begin)))
        except Exception as exc:
            with self._condition:
                self._error = str(exc)
        finally:
            try:
                if client.is_connected():
                    client.preview_stop()
            except Exception as exc:
                with self._condition:
                    self._error = str(exc)
            with self._condition:
                self._stop.set()
                self._jpeg = None
                self._condition.notify_all()

    def stop(self, session=None):
        with self._control:
            if session is not None and session != self._session:
                raise ValueError("preview session mismatch; refusing to stop another session")
            self._stop.set()
            with self._condition:
                self._condition.notify_all()
            if self._thread is not None:
                # Includes a concurrent triggered capture and SDK mode restoration.
                self._thread.join(timeout=100)
                if self._thread.is_alive():
                    raise TimeoutError("preview stop is still waiting for camera operation")
                self._thread = None
            return self.status()

    def status(self):
        with self._condition:
            elapsed = max(.001, time.monotonic() - self._started)
            return {"active": self._thread is not None and self._thread.is_alive() and not self._stop.is_set(),
                    "session": self._session, "published_frames": self._published,
                    "publish_fps": self._published / elapsed if self._started else 0,
                    "acquisition_fps": self._info.get("acquisition_fps", 0),
                    "frame_age_ms": (time.monotonic() - self._published_at)*1000 if self._jpeg else None,
                    "error": self._error}

    def next_frame(self, previous, timeout=1):
        with self._condition:
            self._condition.wait_for(lambda: self._stop.is_set() or (
                self._jpeg is not None and (self._session, self._published) != previous and
                time.monotonic() - self._published_at <= self.config["stale_after_s"]),
                                     timeout=timeout)
            key = (self._session, self._published)
            if (self._stop.is_set() or self._jpeg is None or key == previous or
                    time.monotonic() - self._published_at > self.config["stale_after_s"]):
                return None
            return key, self._jpeg, dict(self._info)
