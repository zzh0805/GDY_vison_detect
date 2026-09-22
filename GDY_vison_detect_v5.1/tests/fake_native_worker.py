"""Offline IPC fixture. Never imports native SDK, opens a camera or controls a robot."""
import json
import mmap
import struct
import sys
import time

with open(sys.argv[2], "r+b") as f:
    memory = mmap.mmap(f.fileno(), 32 * 1024 * 1024)
    mode = ""
    counter = 0
    preview = False
    preview_counter = 0
    for line in sys.stdin:
        parts = line.split()
        req = {"id": int(parts[0]), "op": parts[1]}
        if req["op"] == "open":
            req["ip"] = parts[2]
        response = {"id": req["id"], "ok": True}
        if req["op"] == "open":
            mode = req["ip"]
            response.update(depth_scale_mm=1.0, widths=[2, 2], heights=[2, 2])
        elif req["op"] == "capture":
            if mode == "hang_capture":
                time.sleep(60)
            if mode == "fail_capture":
                response.update(ok=False, error="simulated native timeout")
            else:
                counter += 1
                memory[:8] = struct.pack("<4H", *([500+counter]*4))
                memory[16*1024*1024:16*1024*1024+12] = bytes([255, counter, 0]*4)
                response.update(widths=[2, 2], heights=[2, 2], sizes=[8, 12],
                                stamps=[0, 0], capture_s=0.01, counter=counter)
                if mode == "invalid_size":
                    response["sizes"] = [90000000, 12]
        elif req["op"] == "preview_start":
            preview = True
        elif req["op"] == "preview_stop":
            preview = False
        elif req["op"] == "preview_frame":
            preview_counter += 1
            memory[16*1024*1024:16*1024*1024+12] = bytes([10, 20, 30]*4)
            response.update(available=preview, width=2, height=2, size=12,
                            sequence=preview_counter, epoch=1, age_ms=0, acquisition_fps=10)
        elif req["op"] == "close" and mode == "hang_close":
            time.sleep(60)
        print("GDY_NATIVE_V5 " + json.dumps(response), flush=True)
        if req["op"] == "close":
            break
    memory.close()
