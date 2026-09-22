// Standalone diagnostic. No OpenNI, robot, service or persistent camera settings.
#include "3DCamera.hpp"
#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>
#include <cstdint>
static std::uint64_t rgb_hash=0;
extern "C" std::uint64_t st_rgb_hash() { return rgb_hash; }

static cs::ISystemPtr system_ptr;
static cs::ICameraPtr camera;
// Types.hpp declares C structs/enums globally; only C++ interfaces live in cs.
static ::PropertyExtension original{};
static bool connected=false, started=false, saved=false;
static int rgb_profile=-1;
extern "C" void st_set_rgb_profile(int profile) { rgb_profile=profile; }
static std::string error;
static std::string cleanup_log;
extern "C" void st_set_cleanup_log(const char* path) { cleanup_log=path ? path : ""; }
static void cleanup_event(const char* phase, const char* op, const char* detail="") {
    std::printf("CLEANUP %s %s %s\n", phase, op, detail);
    std::fflush(stdout);
    if (!cleanup_log.empty()) {
        if (auto f=std::fopen(cleanup_log.c_str(), "a")) {
            std::fprintf(f, "CLEANUP %s %s %s\n", phase, op, detail);
            std::fclose(f); // Flush before entering any potentially blocking SDK call.
        } else {
            std::fprintf(stderr, "Cannot append cleanup log\n");
            std::fflush(stderr);
        }
    }
}
static void check(::ERROR_CODE rc, const char* op) {
    if (rc != ::SUCCESS)
        throw std::runtime_error(std::string(op)+": SDK error="+std::to_string(int(rc)));
}
extern "C" const char* st_error() { return error.c_str(); }
extern "C" int st_close() {
    bool ok=true;
    auto attempt=[&](const char* op, auto fn) {
        cleanup_event("BEGIN", op);
        try { check(fn(), op); cleanup_event("END", op); }
        catch (const std::exception& e) {
            cleanup_event("ERROR", op, e.what());
            error += std::string("; cleanup: ")+e.what(); ok=false;
        }
        catch (...) { cleanup_event("ERROR", op, "unknown exception"); error += "; cleanup exception"; ok=false; }
    };
    if (camera && connected) {
        if (saved) attempt("restore trigger mode", [] { return camera->setPropertyExtension(::PROPERTY_EXT_TRIGGER_MODE, original); });
        if (started) attempt("stop streams", [] { return camera->stopStream(); });
        attempt("disconnect", [] { return camera->disconnect(); });
    }
    started=connected=saved=false;
    attempt("release camera object", [] { camera.reset(); return ::SUCCESS; });
    attempt("release system object", [] { system_ptr.reset(); return ::SUCCESS; });
    return ok ? 0 : -1;
}
extern "C" int st_open(const char* ip, int soft, int profile) {
    try {
        error.clear();
        cs::setSdkEnableNetworking(true);
        cs::setEnableNetworking(true);
        system_ptr=cs::getSystemPtr();
        if (!system_ptr) throw std::runtime_error("getSystemPtr returned null");
        std::vector<::CameraInfo> devices;
        check(system_ptr->queryCameras(devices, 5000), "queryCameras");
        bool found=false;
        for (const auto& info: devices) {
            std::printf("Device: %.32s serial=%.32s id=%.32s\n", info.name, info.serial, info.uniqueId);
            if (std::string(info.uniqueId)==ip || std::string(info.serial)==ip ||
                (std::string(ip)=="auto" && devices.size()==1)) {
                camera=cs::getCameraPtr();
                if (!camera) throw std::runtime_error("getCameraPtr returned null");
                check(camera->connect(info), "connect"); connected=true; found=true; break;
            }
        }
        if (!found) throw std::runtime_error("requested IP/serial not enumerated; no fallback to another camera");
        check(camera->getPropertyExtension(::PROPERTY_EXT_TRIGGER_MODE, original), "read original trigger mode");
        saved=true;
        std::vector<::StreamInfo> all, depth;
        check(camera->getStreamInfos(::STREAM_TYPE_DEPTH, all), "getStreamInfos");
        for (const auto& info: all) if (info.format==::STREAM_FORMAT_Z16) {
            std::printf("Z16 profile %zu: %dx%d %.2f fps\n", depth.size(), info.width, info.height, info.fps);
            depth.push_back(info);
        }
        if (profile<0 || profile>=int(depth.size())) throw std::runtime_error("invalid Z16 profile index");
        if (rgb_profile >= 0) {
            std::vector<::StreamInfo> rgb;
            all.clear();
            check(camera->getStreamInfos(::STREAM_TYPE_RGB, all), "get RGB StreamInfos");
            for (const auto& info: all) if (info.format==::STREAM_FORMAT_RGB8) {
                std::printf("RGB8 profile %zu: %dx%d %.2f fps\n", rgb.size(), info.width, info.height, info.fps);
                rgb.push_back(info);
            }
            std::fflush(stdout);
            if (rgb_profile >= int(rgb.size())) throw std::runtime_error("invalid RGB8 profile index (no silent format fallback)");
            started=true; // Also attempt cleanup if paired startup partially fails.
            check(camera->startStream(depth[profile], rgb[rgb_profile], nullptr, nullptr), "start paired depth+RGB");
        } else {
            started=true;
            check(camera->startStream(::STREAM_TYPE_DEPTH, depth[profile]), "start depth");
        }
        // Same pause -> trigger-mode -> softTrigger sequence as SampleSoftTrigger.cpp.
        check(camera->pauseStream(::STREAM_TYPE_DEPTH), "pause depth");
        ::PropertyExtension mode{};
        mode.triggerMode=soft ? ::TRIGGER_MODE_SOFTWAER : ::TRIGGER_MODE_OFF;
        check(camera->setPropertyExtension(::PROPERTY_EXT_TRIGGER_MODE, mode), "set trigger mode");
        ::PropertyExtension actual{};
        check(camera->getPropertyExtension(::PROPERTY_EXT_TRIGGER_MODE, actual), "read back trigger mode");
        if (actual.triggerMode!=mode.triggerMode) throw std::runtime_error("trigger mode readback mismatch");
        if (!soft) check(camera->resumeStream(::STREAM_TYPE_DEPTH), "resume continuous depth");
        std::printf("Selected profile=%d; mode=%d; original=%d\n", profile, int(actual.triggerMode), int(original.triggerMode));
        std::fflush(stdout);
        return 0;
    } catch (const std::exception& e) { error=e.what(); }
      catch (...) { error="unknown SDK exception in open"; }
    st_close(); return -1;
}
// SDK paired API; this does NOT assert simultaneous exposures or triggerable RGB.
// Fixed primitive arrays avoid passing vendor C++ structures across ctypes ABI.
extern "C" int st_read_pair(int timeout, double* stamps, int* sizes, int* widths, int* heights) {
    try {
        cs::IFramePtr depth, rgb;
        auto rc=camera->getPairedFrame(depth, rgb, timeout);
        if (rc==::ERROR_FRAME_TIMEOUT) return 1;
        check(rc, "getPairedFrame");
        cs::IFramePtr frames[2]={depth, rgb};
        for (int i=0; i<2; ++i) {
            const auto& f=frames[i];
            if (!f || f->empty() || !f->getData() || f->getWidth()<=0 || f->getHeight()<=0)
                throw std::runtime_error("paired API returned invalid depth/RGB frame");
            auto expected=i==0 ? ::STREAM_FORMAT_Z16 : ::STREAM_FORMAT_RGB8;
            if (f->getFormat()!=expected || f->getSize()!=f->getWidth()*f->getHeight()*(i==0 ? 2 : 3))
                throw std::runtime_error("paired frame format/size mismatch");
            stamps[i]=f->getTimeStamp(); sizes[i]=f->getSize();
            widths[i]=f->getWidth(); heights[i]=f->getHeight();
        }
        // Hash all RGB bytes while SDK frame is alive. Keep no image history.
        rgb_hash=14695981039346656037ULL;
        const auto pixels=reinterpret_cast<const unsigned char*>(rgb->getData());
        for (int i=0; i<rgb->getSize(); ++i) {
            rgb_hash ^= pixels[i];
            rgb_hash *= 1099511628211ULL;
        }
        return 0; // Both SDK frames released by RAII, including error paths.
    } catch (const std::exception& e) { error=e.what(); }
      catch (...) { error="unknown SDK exception in paired read"; }
    return -1;
}
extern "C" int st_trigger() {
    try { check(camera->softTrigger(1), "softTrigger(1)"); return 0; }
    catch (const std::exception& e) { error=e.what(); }
    catch (...) { error="unknown SDK exception in trigger"; }
    return -1;
}
// Return 0=frame, 1=documented frame timeout, -1=other error.
// The SDK shared_ptr is destroyed before this call returns; no images accumulate.
extern "C" int st_read(int timeout, double* timestamp, int* size, int* width, int* height) {
    try {
        cs::IFramePtr frame;
        auto rc=camera->getFrame(::STREAM_TYPE_DEPTH, frame, timeout);
        if (rc==::ERROR_FRAME_TIMEOUT) return 1;
        check(rc, "getFrame");
        if (!frame || frame->empty() || !frame->getData() || frame->getSize()<=0)
            throw std::runtime_error("SDK returned an empty frame");
        *timestamp=frame->getTimeStamp(); *size=frame->getSize();
        *width=frame->getWidth(); *height=frame->getHeight();
        return 0;
    } catch (const std::exception& e) { error=e.what(); }
      catch (...) { error="unknown SDK exception in read"; }
    return -1;
}
