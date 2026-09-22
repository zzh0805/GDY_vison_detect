"""Private native SDK subprocess. No NumPy, OpenNI, torch, YOLO or robot imports."""
import ctypes as C
import json
import mmap
import os
from pathlib import Path
import sys
import time

CAPACITY = 16 * 1024 * 1024
PREFIX = "GDY_NATIVE_V5 "


def emit(data):
    # Leading newline separates a response from SDK output without a newline.
    sys.stdout.write("\n" + PREFIX + json.dumps(data, allow_nan=False) + "\n")
    sys.stdout.flush()


def main():
    parent_pid = int(sys.argv[3])
    # Linux worker must not outlive an abruptly killed service.
    libc = C.CDLL(None)
    if libc.prctl(1, 9, 0, 0, 0) != 0:  # PR_SET_PDEATHSIG, SIGKILL
        raise RuntimeError("cannot configure camera worker parent-death protection")
    if os.getppid() != parent_pid:
        raise RuntimeError("camera worker parent already exited")
    lib = C.CDLL(sys.argv[1])
    lib.st_error.argtypes = []
    lib.st_error.restype = C.c_char_p
    lib.st_set_rgb_profile.argtypes = [C.c_int]
    lib.st_set_rgb_profile.restype = None
    lib.st_open.argtypes = [C.c_char_p, C.c_int, C.c_int]
    lib.st_open.restype = C.c_int
    for name in ("st_close", "st_trigger"):
        getattr(lib, name).argtypes = []
        getattr(lib, name).restype = C.c_int
    lib.st_read_pair.argtypes = [C.c_int, C.POINTER(C.c_double)] + [C.POINTER(C.c_int)]*3
    lib.st_read_pair.restype = C.c_int
    lib.v5_depth_scale.argtypes = [C.POINTER(C.c_double)]
    lib.v5_depth_scale.restype = C.c_int
    lib.v5_profile_sizes.argtypes = [C.POINTER(C.c_int), C.POINTER(C.c_int)]
    lib.v5_profile_sizes.restype = C.c_int
    lib.v5_read_pair.argtypes = [C.c_int, C.c_void_p, C.c_int, C.c_void_p, C.c_int,
                                C.POINTER(C.c_double)] + [C.POINTER(C.c_int)]*3
    lib.v5_read_pair.restype = C.c_int

    def check(rc):
        if rc != 0:
            detail = "paired frame timeout" if rc == 1 else lib.st_error().decode("utf-8", "replace")
            raise RuntimeError(detail)

    with open(sys.argv[2], "r+b") as file:
        memory = mmap.mmap(file.fileno(), CAPACITY*2)
        # Linux mappings/open fds remain valid; crashes cannot leave a 32MiB file behind.
        os.unlink(sys.argv[2])
        owner = (C.c_ubyte * (CAPACITY*2)).from_buffer(memory)
        depth_ptr = C.addressof(owner)
        rgb_ptr = depth_ptr + CAPACITY
        connected = False
        for line in sys.stdin:
            request = json.loads(line)
            rid = request["id"]
            try:
                op = request["op"]
                if op == "open":
                    if connected:
                        raise RuntimeError("camera already connected")
                    lib.st_set_rgb_profile(0)
                    check(lib.st_open(request["ip"].encode(), 1, 0))
                    connected = True
                    scale = C.c_double()
                    check(lib.v5_depth_scale(C.byref(scale)))
                    profile_widths, profile_heights = (C.c_int*2)(), (C.c_int*2)()
                    check(lib.v5_profile_sizes(profile_widths, profile_heights))
                    # Same bounded startup drain/no-trigger verification as the test.
                    quiet = 0
                    for _ in range(103):
                        stamps = (C.c_double*2)()
                        sizes, widths, heights = (C.c_int*2)(), (C.c_int*2)(), (C.c_int*2)()
                        rc = lib.st_read_pair(500, stamps, sizes, widths, heights)
                        if rc == 1:
                            quiet += 1
                            if quiet == 3:
                                break
                        else:
                            check(rc)
                            quiet = 0
                    if quiet < 3:
                        raise RuntimeError("unsolicited paired frames in software trigger mode")
                    emit({"id": rid, "ok": True, "depth_scale_mm": scale.value,
                          "widths": list(profile_widths), "heights": list(profile_heights)})
                elif op == "capture":
                    if not connected:
                        raise RuntimeError("camera not connected")
                    begin = time.monotonic()
                    check(lib.st_trigger())
                    stamps = (C.c_double*2)()
                    sizes, widths, heights = (C.c_int*2)(), (C.c_int*2)(), (C.c_int*2)()
                    # 10000ms is the configuration that passed the user's long test.
                    check(lib.v5_read_pair(10000, depth_ptr, CAPACITY, rgb_ptr, CAPACITY,
                                           stamps, sizes, widths, heights))
                    emit({"id": rid, "ok": True, "stamps": list(stamps), "sizes": list(sizes),
                          "widths": list(widths), "heights": list(heights),
                          "capture_s": time.monotonic()-begin})
                elif op == "close":
                    check(lib.st_close())
                    connected = False
                    emit({"id": rid, "ok": True})
                    break
                else:
                    raise RuntimeError("unknown private camera operation")
            except Exception as exc:
                emit({"id": rid, "ok": False, "error": str(exc)})
        # Normally close was requested; on EOF do not silently keep owning camera.
        if connected:
            lib.st_close()
        del owner
        memory.close()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        emit({"id": -1, "ok": False, "error": str(exc)})
        raise SystemExit(1)
