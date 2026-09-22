// Standalone vendor-SDK owner. No OpenNI, robot, service or persistent camera settings.
//
// Lifecycle contract (see 3DCamerav3.2.207-zh.chm "开发指引"):
//   1. setSdkEnableNetworking / setEnableNetworking -> getSystemPtr
//   2. queryCameras(cameras, 5000)                     (FAQ 1.1: timeout must be passed)
//   3. getCameraPtr -> connect(info)                    (guide 1.2)
//   4. getStreamInfos -> startStream(STREAM_TYPE, ...)  (one call per stream, as the official
//      SampleDepthRGBFrameMatch.cpp does; there is no documented paired active-poll form)
//   5. stream properties (frame time / exposure / gain) may only be written while the stream is
//      running, frame time before exposure, exposure <= frame time   (guide 1.7, FAQ 2.4)
//   6. pauseStream + TRIGGER_MODE_SOFTWAER -> softTrigger(1) -> getFrame   (SampleSoftTrigger)
//   7. Cleanup order that survives the known camera.reset() hang:
//      restore trigger mode -> stopStream -> disconnect -> reset camera -> reset system.
//      The parent process kills this one on timeout, so a hanging reset never blocks shutdown.
#include "3DCamera.hpp"
#include <cstdio>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

// --- Legacy ABI surface kept for the old ctypes bridge and diagnostics -------------------
static std::uint64_t rgb_hash = 0;
extern "C" std::uint64_t st_rgb_hash() { return rgb_hash; }

static cs::ISystemPtr system_ptr;
static cs::ICameraPtr camera;
// Types.hpp declares C structs/enums globally; only C++ interfaces live in cs.
static ::PropertyExtension original{};
static bool connected = false, started = false, saved = false;
static bool streams_running = false;  // both single-stream startStream calls succeeded
static std::string st_camera_serial;  // bounded copy of CameraInfo::serial of the live camera
static int rgb_profile = -1;
extern "C" void st_set_rgb_profile(int profile) { rgb_profile = profile; }
static std::string error;
static std::string cleanup_log;
extern "C" void st_set_cleanup_log(const char* path) { cleanup_log = path ? path : ""; }

static void cleanup_event(const char* phase, const char* op, const char* detail = "") {
    std::printf("CLEANUP %s %s %s\n", phase, op, detail);
    std::fflush(stdout);
    if (!cleanup_log.empty()) {
        if (auto f = std::fopen(cleanup_log.c_str(), "a")) {
            std::fprintf(f, "CLEANUP %s %s %s\n", phase, op, detail);
            std::fclose(f);  // Flush before entering any potentially blocking SDK call.
        } else {
            std::fprintf(stderr, "Cannot append cleanup log\n");
            std::fflush(stderr);
        }
    }
}

static void check(::ERROR_CODE rc, const char* op) {
    if (rc != ::SUCCESS) {
        const char* detail = ::cs::getCameraErrorString(rc);
        throw std::runtime_error(std::string(op) + ": SDK error=" + std::to_string(int(rc)) +
                                 (detail && *detail ? std::string(" (") + detail + ")" : ""));
    }
}

// CameraInfo fields are fixed-size arrays with NO guaranteed NUL terminator (the vendor sample
// itself prints them with "%.32s"). std::string(ptr) would read past the array, so length is
// bounded explicitly.
static std::string bounded_field(const char* text, std::size_t size) {
    std::size_t len = 0;
    while (len < size && text[len] != '\0') ++len;
    return std::string(text, len);
}

extern "C" const char* st_error() { return error.c_str(); }

extern "C" int st_close() {
    bool ok = true;
    auto attempt = [&](const char* op, auto fn) {
        cleanup_event("BEGIN", op);
        try {
            check(fn(), op);
            cleanup_event("END", op);
        } catch (const std::exception& e) {
            cleanup_event("ERROR", op, e.what());
            error += std::string("; cleanup: ") + e.what();
            ok = false;
        } catch (...) {
            cleanup_event("ERROR", op, "unknown exception");
            error += "; cleanup exception";
            ok = false;
        }
    };
    if (camera && connected) {
        if (saved) attempt("restore trigger mode", [] { return camera->setPropertyExtension(::PROPERTY_EXT_TRIGGER_MODE, original); });
        if (started) attempt("stop streams", [] { return camera->stopStream(); });
        attempt("disconnect", [] { return camera->disconnect(); });
    }
    started = connected = saved = streams_running = false;
    attempt("release camera object", [] { camera.reset(); return ::SUCCESS; });
    attempt("release system object", [] { system_ptr.reset(); return ::SUCCESS; });
    return ok ? 0 : -1;
}

// --- profile selection -------------------------------------------------------------------
struct StreamSelection {
    ::StreamInfo depth{};
    ::StreamInfo rgb{};
    bool have_depth = false;
    bool have_rgb = false;
};
static StreamSelection selection;

// FAQ 2.1: a stream may only be started from an entry returned by getStreamInfos.
static bool pick_profiles(const char* endpoint, int z16_index, bool want_rgb) {
    selection = StreamSelection{};
    std::vector<::StreamInfo> depth;
    check(camera->getStreamInfos(::STREAM_TYPE_DEPTH, depth), "getStreamInfos(depth)");
    std::vector<::StreamInfo> z16;
    for (const auto& info : depth)
        if (info.format == ::STREAM_FORMAT_Z16) {
            std::printf("Z16 profile %zu: %dx%d %.2f fps\n", z16.size(), info.width, info.height, info.fps);
            z16.push_back(info);
        }
    if (z16_index < 0 || z16_index >= int(z16.size()))
        throw std::runtime_error("invalid Z16 profile index (device reported " +
                                 std::to_string(z16.size()) + ")");
    selection.depth = z16[std::size_t(z16_index)];
    selection.have_depth = true;

    if (want_rgb) {
        std::vector<::StreamInfo> rgb;
        check(camera->getStreamInfos(::STREAM_TYPE_RGB, rgb), "getStreamInfos(rgb)");
        for (const auto& info : rgb)
            if (info.format == ::STREAM_FORMAT_RGB8) selection.rgb = info, selection.have_rgb = true;
        if (!selection.have_rgb)
            throw std::runtime_error("no RGB8 profile (no silent format fallback)");
    }
    std::printf("Selected depth=%dx%d rgb=%dx%d endpoint=%s\n", selection.depth.width,
                selection.depth.height, selection.rgb.width, selection.rgb.height,
                endpoint ? endpoint : "");
    std::fflush(stdout);
    return true;
}

// --- connection --------------------------------------------------------------------------
// Everything up to (but excluding) startStream, so the caller can apply stream properties
// between "stream running" and "stream suspended" exactly like SampleSoftTrigger does.
static bool connect_impl(const char* endpoint) {
    error.clear();
    ::cs::setSdkEnableNetworking(true);
    ::cs::setEnableNetworking(true);
    system_ptr = ::cs::getSystemPtr();
    if (!system_ptr) throw std::runtime_error("getSystemPtr returned null");
    std::vector<::CameraInfo> devices;
    check(system_ptr->queryCameras(devices, 5000), "queryCameras");
    const std::string wanted = endpoint ? endpoint : "";
    bool found = false;
    for (const auto& info : devices) {
        std::printf("Device: %.32s serial=%.32s id=%.32s\n", info.name, info.serial, info.uniqueId);
        const std::string unique = bounded_field(info.uniqueId, sizeof(info.uniqueId));
        const std::string serial = bounded_field(info.serial, sizeof(info.serial));
        if (unique == wanted || serial == wanted || (wanted == "auto" && devices.size() == 1)) {
            camera = ::cs::getCameraPtr();
            if (!camera) throw std::runtime_error("getCameraPtr returned null");
            check(camera->connect(info), "connect");
            connected = true;
            st_camera_serial = serial;
            found = true;
            break;
        }
    }
    if (!found) throw std::runtime_error("requested endpoint not enumerated; no fallback to another camera");
    check(camera->getPropertyExtension(::PROPERTY_EXT_TRIGGER_MODE, original), "read original trigger mode");
    saved = true;
    std::printf("Camera connected; original trigger mode=%d\n", int(original.triggerMode));
    std::fflush(stdout);
    return true;
}

extern "C" int st_connect(const char* endpoint, int z16_index) {
    try {
        connect_impl(endpoint);
        pick_profiles(endpoint, z16_index, /*want_rgb=*/true);
        return 0;
    } catch (const std::exception& e) {
        error = e.what();
    } catch (...) {
        error = "unknown SDK exception in connect";
    }
    st_close();
    return -1;
}

// One startStream per stream: the documented active-polling form (guide 1.6, FAQ 2.6).
static bool start_streams() {
    try {
        if (!camera || !connected) throw std::runtime_error("start streams: camera not connected");
        if (!selection.have_depth) throw std::runtime_error("start streams: no depth profile selected");
        check(camera->startStream(::STREAM_TYPE_DEPTH, selection.depth), "start depth stream");
        started = true;  // Cleanup must stop the stream even if the RGB start below fails.
        if (selection.have_rgb) {
            check(camera->startStream(::STREAM_TYPE_RGB, selection.rgb), "start RGB stream");
        }
        streams_running = true;
        return true;
    } catch (const std::exception& e) {
        error = e.what();
    } catch (...) {
        error = "unknown SDK exception starting streams";
    }
    return false;
}

// --- legacy entry point (kept byte-compatible for the old ctypes bridge / diagnostics) ----
// Old contract: connect, start depth (+ optional RGB), suspend depth, enter the requested
// trigger mode. New code uses st_connect()/start_streams() so it can set stream properties
// while both streams are still running.
extern "C" int st_open(const char* ip, int soft, int profile) {
    try {
        if (st_connect(ip, profile) != 0) return -1;
        if (!start_streams()) throw std::runtime_error(error);
        check(camera->pauseStream(::STREAM_TYPE_DEPTH), "pause depth");
        ::PropertyExtension mode{};
        mode.triggerMode = soft ? ::TRIGGER_MODE_SOFTWAER : ::TRIGGER_MODE_OFF;
        check(camera->setPropertyExtension(::PROPERTY_EXT_TRIGGER_MODE, mode), "set trigger mode");
        ::PropertyExtension actual{};
        check(camera->getPropertyExtension(::PROPERTY_EXT_TRIGGER_MODE, actual), "read back trigger mode");
        if (actual.triggerMode != mode.triggerMode) throw std::runtime_error("trigger mode readback mismatch");
        return 0;
    } catch (const std::exception& e) {
        error = e.what();
    } catch (...) {
        error = "unknown SDK exception in open";
    }
    st_close();
    return -1;
}

// --- legacy read helpers ------------------------------------------------------------------
// SDK paired API; this does NOT assert simultaneous exposures or triggerable RGB.
// Fixed primitive arrays avoid passing vendor C++ structures across ctypes ABI.
extern "C" int st_read_pair(int timeout, double* stamps, int* sizes, int* widths, int* heights) {
    try {
        cs::IFramePtr depth, rgb;
        auto rc = camera->getPairedFrame(depth, rgb, timeout);
        if (rc == ::ERROR_FRAME_TIMEOUT) return 1;
        check(rc, "getPairedFrame");
        cs::IFramePtr frames[2] = {depth, rgb};
        for (int i = 0; i < 2; ++i) {
            const auto& f = frames[i];
            if (!f || f->empty() || !f->getData() || f->getWidth() <= 0 || f->getHeight() <= 0)
                throw std::runtime_error("paired API returned invalid depth/RGB frame");
            auto expected = i == 0 ? ::STREAM_FORMAT_Z16 : ::STREAM_FORMAT_RGB8;
            if (f->getFormat() != expected || f->getSize() != f->getWidth() * f->getHeight() * (i == 0 ? 2 : 3))
                throw std::runtime_error("paired frame format/size mismatch");
            stamps[i] = f->getTimeStamp();
            sizes[i] = f->getSize();
            widths[i] = f->getWidth();
            heights[i] = f->getHeight();
        }
        rgb_hash = 14695981039346656037ULL;
        const auto pixels = reinterpret_cast<const unsigned char*>(rgb->getData());
        for (int i = 0; i < rgb->getSize(); ++i) {
            rgb_hash ^= pixels[i];
            rgb_hash *= 1099511628211ULL;
        }
        return 0;  // Both SDK frames released by RAII, including error paths.
    } catch (const std::exception& e) {
        error = e.what();
    } catch (...) {
        error = "unknown SDK exception in paired read";
    }
    return -1;
}

extern "C" int st_trigger() {
    try {
        check(camera->softTrigger(1), "softTrigger(1)");
        return 0;
    } catch (const std::exception& e) {
        error = e.what();
    } catch (...) {
        error = "unknown SDK exception in trigger";
    }
    return -1;
}

// Return 0=frame, 1=documented frame timeout, -1=other error.
// The SDK shared_ptr is destroyed before this call returns; no images accumulate.
extern "C" int st_read(int timeout, double* timestamp, int* size, int* width, int* height) {
    try {
        cs::IFramePtr frame;
        auto rc = camera->getFrame(::STREAM_TYPE_DEPTH, frame, timeout);
        if (rc == ::ERROR_FRAME_TIMEOUT) return 1;
        check(rc, "getFrame");
        if (!frame || frame->empty() || !frame->getData() || frame->getSize() <= 0)
            throw std::runtime_error("SDK returned an empty frame");
        *timestamp = frame->getTimeStamp();
        *size = frame->getSize();
        *width = frame->getWidth();
        *height = frame->getHeight();
        return 0;
    } catch (const std::exception& e) {
        error = e.what();
    } catch (...) {
        error = "unknown SDK exception in read";
    }
    return -1;
}
