// Linux SDK owner for v5.1. One thread calls the SDK; Python never handles vendor pointers.
// The private command protocol is bounded, numeric and local stdin, NOT a network API.
//
// State machine (v5.1.1):
//
//   STANDBY (most of the time)
//       both streams started and SUSPENDED, TRIGGER_MODE_SOFTWAER.
//       The sensor emits nothing, so the SDK frame queues cannot grow while idle.
//       capture = clear both queues -> drain until timeout proves them empty -> softTrigger(1)
//                 -> getFrame(depth) + getFrame(rgb).
//       No stream is stopped, restarted, re-connected or re-configured for a shot.
//
//   STREAMING (short, on request)
//       RGB resumed (free-running) and consumed by pump(); depth stays suspended unless
//       preview_with_depth asks for it. Entering/leaving streaming only pauses or resumes a
//       stream - never a rebuild - so the tested SDK lifecycle stays untouched.
//
// Anything that leaves the worker in an unknown state throws: the Python owner then kills this
// process instead of continuing with a camera whose mode is unclear (fail closed).
#include "bridge.cpp"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#ifndef GDY_NATIVE_STATE_TEST
#include <poll.h>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <signal.h>
#endif

namespace {
constexpr size_t capacity = 16 * 1024 * 1024;
using Clock = std::chrono::steady_clock;

#ifdef GDY_NATIVE_STATE_TEST
struct Mapping {
    std::vector<unsigned char> storage = std::vector<unsigned char>(capacity * 2);
    void* data = storage.data();
};
// Prevent two v5.1 processes (including different project copies) owning the SDK.
struct OwnerLock {
    int fd = -1;
    explicit OwnerLock(const char* = "") {}
    ~OwnerLock() {}
};
#else
struct Mapping {
    int fd = -1;
    void* data = MAP_FAILED;
    explicit Mapping(const char* path) {
        fd = ::open(path, O_RDWR);
        if (fd < 0) throw std::runtime_error("open shared buffer failed");
        struct stat s{};
        if (fstat(fd, &s) || s.st_size != long(capacity * 2)) {
            ::close(fd);
            fd = -1;
            throw std::runtime_error("invalid shared buffer capacity");
        }
        data = mmap(nullptr, capacity * 2, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        if (data == MAP_FAILED) {
            ::close(fd);
            fd = -1;
            throw std::runtime_error("mmap failed");
        }
        ::unlink(path);
    }
    ~Mapping() {
        if (data != MAP_FAILED) munmap(data, capacity * 2);
        if (fd >= 0) ::close(fd);
    }
    Mapping(const Mapping&) = delete;
    Mapping& operator=(const Mapping&) = delete;
};
struct OwnerLock {
    int fd = -1;
    explicit OwnerLock(const char* extra = nullptr) {
        const std::string path = "/tmp/gdy_native_camera_" + std::to_string(getuid()) + ".lock";
        fd = ::open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
        if (fd < 0) throw std::runtime_error("cannot open camera owner lock");
        if (flock(fd, LOCK_EX | LOCK_NB)) {
            ::close(fd);
            fd = -1;
            throw std::runtime_error(std::string("another v5.1 SDK owner is running") +
                                     (extra ? extra : ""));
        }
    }
    ~OwnerLock() { if (fd >= 0) ::close(fd); }  // Never unlink a live lock inode.
};
#endif

#ifndef GDY_NATIVE_STATE_TEST
std::string escaped(const std::string& s) {
    std::string out;
    for (unsigned char c : s) {
        if (c == '"' || c == '\\') { out += '\\'; out += c; }
        else if (c < 32) out += ' ';
        else out += c;
    }
    return out;
}
void reply(long id, const std::string& fields) {
    std::cout << "\nGDY_NATIVE_V5 {\"id\":" << id << ',' << fields << "}\n" << std::flush;
}
#endif
// Translates the private v5_* return codes: 0=ok, 1=documented SDK timeout, other=error.
[[noreturn]] void fail() { throw std::runtime_error(st_error()); }
void checked_rc(int rc, const char* what) {
    if (rc == 0) return;
    if (rc == 1) throw std::runtime_error(std::string(what) + ": timed out");
    fail();
}

// ---------------------------------------------------------------------------------------
// Original-settings persistence.
// save_settings() must remember what the device looked like BEFORE the first write. A worker
// that is killed and restarted would otherwise read the already-modified values and adopt them
// as "original", silently making the restore-on-close promise decay. Persisting the first
// observation fixes that across process restarts.
struct PropertyBackup {
    std::string path;
    std::string serial;
    std::map<std::string, std::string> values;
    bool dirty = false;
    bool active = false;

    void open(const std::string& file, const std::string& camera_serial) {
        path = file;
        serial = camera_serial;
        active = !path.empty();
        if (!active) return;
        std::FILE* f = std::fopen(path.c_str(), "r");
        if (!f) return;
        char line[512];
        std::string header;
        if (std::fgets(line, sizeof(line), f)) header = line;
        if (header.find("gdy_native_property_backup v1") == std::string::npos ||
            header.find(serial) == std::string::npos) {
            std::fclose(f);  // Another camera (or an older format): start a fresh file.
            values.clear();
            return;
        }
        while (std::fgets(line, sizeof(line), f)) {
            std::string text(line);
            while (!text.empty() && (text.back() == '\n' || text.back() == '\r')) text.pop_back();
            const auto split = text.find('=');
            if (split == std::string::npos || split == 0) continue;
            values[text.substr(0, split)] = text.substr(split + 1);
        }
        std::fclose(f);
        std::printf("Property backup loaded: %s entries=%zu\n", path.c_str(), values.size());
    }
    bool get(const std::string& key, std::string& out) const {
        const auto it = values.find(key);
        if (it == values.end()) return false;
        out = it->second;
        return true;
    }
    void put(const std::string& key, const std::string& value) {
        values[key] = value;
        dirty = true;
    }
    void flush() {
        if (!active || !dirty) return;
        const std::string temp = path + ".tmp";
        std::FILE* f = std::fopen(temp.c_str(), "w");
        if (!f) {
            std::fprintf(stderr, "Cannot write property backup %s\n", temp.c_str());
            return;
        }
        std::fprintf(f, "# gdy_native_property_backup v1 serial=%s\n", serial.c_str());
        for (const auto& item : values) std::fprintf(f, "%s=%s\n", item.first.c_str(), item.second.c_str());
        std::fclose(f);
        if (std::rename(temp.c_str(), path.c_str())) {
            std::fprintf(stderr, "Cannot replace property backup %s\n", path.c_str());
            std::remove(temp.c_str());
            return;
        }
        dirty = false;
    }
};

struct Settings {
    int capture_ms = 3000, preview_ms = 100, quiet = 3, drain = 60;
    double rgb_exp = -1, rgb_gain = -1, depth_exp = -1, depth_period = -1, depth_gain = -1;
    int match = 0, threshold = 60, offset = 0, after = 0, strict = 0, preview_depth = 0;
    int rgb_read_ms = 500, drain_ms = 100, drain_frames = 10;
    int periodic_clear_s = 60, memory_report_s = 300;

    void parse(std::istream& in) {
        if (!(in >> capture_ms >> preview_ms >> quiet >> drain >> rgb_exp >> rgb_gain >>
              depth_exp >> depth_period >> depth_gain >> match >> threshold >> offset >>
              after >> strict >> preview_depth >> rgb_read_ms >> drain_ms >> drain_frames >>
              periodic_clear_s >> memory_report_s))
            throw std::runtime_error("incomplete native settings");
        if (capture_ms < 100 || capture_ms > 60000 || preview_ms < 10 || preview_ms > 500 ||
            quiet < 1 || quiet > 10 || drain < quiet || drain > 60 || threshold < 1 ||
            threshold > 10000 || std::abs(offset) > 10000)
            throw std::runtime_error("native settings out of bounds");
        if (rgb_read_ms < 50 || rgb_read_ms > 60000 || drain_ms < 1 || drain_ms > 1000 ||
            drain_frames < 1 || drain_frames > 64 || periodic_clear_s < 0 ||
            periodic_clear_s > 3600 || memory_report_s < 0 || memory_report_s > 3600)
            throw std::runtime_error("native queue settings out of bounds");
        for (double v : {rgb_exp, rgb_gain, depth_exp, depth_period, depth_gain})
            if (!std::isfinite(v) || (v != -1 && (v <= 0 || v > 10000000)))
                throw std::runtime_error("invalid exposure/gain");
        for (int v : {match, after, strict, preview_depth})
            if (v != 0 && v != 1) throw std::runtime_error("invalid boolean setting");
        if (depth_exp > 0 && depth_period > 0 && depth_exp >= depth_period)
            throw std::runtime_error("depth exposure must be below frame time");
    }
};

struct CameraOwner {
    Settings cfg;
    bool streaming = false, opened = false;
    StreamInfo depth_info{}, rgb_info{};
    std::string label;
    PropertyBackup backup;
    std::vector<std::function<void()>> restore;
    std::vector<unsigned char> latest;  // Exactly ONE retained preview frame, <=16MiB.
    int width = 0, height = 0;
    double stamp = 0;
    uint64_t sequence = 0, epoch = 0, count = 0;
    Clock::time_point began = Clock::now(), arrival = began;
    Clock::time_point last_clear = began, last_memory = began;
    double previous_depth = 0, previous_rgb = 0;
    bool previous_valid = false;

    explicit CameraOwner(std::string where = std::string()) : label(std::move(where)) {}
    ~CameraOwner() { close(); }

    bool close() noexcept {
        bool ok = true;
        for (auto it = restore.rbegin(); it != restore.rend(); ++it) {
            try {
                (*it)();
            } catch (...) {
                std::cerr << "property restoration failed\n";
                ok = false;
            }
        }
        restore.clear();
        try {
            backup.flush();
        } catch (...) {
            std::cerr << "property backup flush failed\n";
            ok = false;
        }
        if ((camera || system_ptr) && st_close() != 0) ok = false;
        opened = streaming = false;
        return ok;
    }

    // ---- property helpers -------------------------------------------------------------
    // `remember` gives the restore step the value observed on the FIRST run of this camera, not
    // the value the device happens to hold now (which may already be ours).
    template <class ReadFn, class RestoreFn>
    void remember(const char* key, ReadFn read, RestoreFn restore_op) {
        std::string text;
        if (!backup.get(key, text)) text = read(), backup.put(key, text);
        restore_op(text);
    }
    void remember_float(const char* key, STREAM_TYPE stream, PROPERTY_TYPE prop) {
        remember(key,
                 [&] {
                     float value = 0;
                     check(camera->getProperty(stream, prop, value), "read original property");
                     return std::to_string(value);
                 },
                 [&](const std::string& text) {
                     const float target = std::stof(text);
                     restore.emplace_back([stream, prop, target] {
                         check(camera->setProperty(stream, prop, target), "restore property");
                     });
                 });
    }
    void remember_extension(const char* key, PROPERTY_TYPE_EXTENSION ext,
                            std::function<std::string(const ::PropertyExtension&)> encode,
                            std::function<void(::PropertyExtension&, const std::string&)> decode) {
        remember(key,
                 [&] {
                     ::PropertyExtension value{};
                     check(camera->getPropertyExtension(ext, value), "read original extension");
                     return encode(value);
                 },
                 [&](const std::string& text) {
                     ::PropertyExtension target{};
                     decode(target, text);
                     restore.emplace_back([ext, target] {
                         check(camera->setPropertyExtension(ext, target), "restore extension");
                     });
                 });
    }
    // Registers a restore for exactly the keys settings() will modify.
    void save_settings() {
        const bool rgb_write = cfg.rgb_exp > 0 || cfg.rgb_gain > 0;
        if (rgb_write) remember_float("rgb_auto_exposure", STREAM_TYPE_RGB, PROPERTY_ENABLE_AUTO_EXPOSURE);
        if (cfg.rgb_exp > 0)
            remember_extension(
                "rgb_exposure_us", PROPERTY_EXT_EXPOSURE_TIME_RGB,
                [](const ::PropertyExtension& v) { return std::to_string(v.uiExposureTime); },
                [](::PropertyExtension& v, const std::string& t) {
                    v.uiExposureTime = static_cast<unsigned int>(std::stoul(t));
                });
        if (cfg.rgb_gain > 0) remember_float("rgb_gain", STREAM_TYPE_RGB, PROPERTY_GAIN);
        const bool depth_write = cfg.depth_exp > 0 || cfg.depth_gain > 0 || cfg.depth_period > 0;
        if (depth_write)
            remember_extension(
                "depth_auto_exposure", PROPERTY_EXT_AUTO_EXPOSURE_MODE,
                [](const ::PropertyExtension& v) { return std::to_string(int(v.autoExposureMode)); },
                [](::PropertyExtension& v, const std::string& t) {
                    v.autoExposureMode = static_cast<AUTO_EXPOSURE_MODE>(std::stoi(t));
                });
        if (cfg.depth_period > 0) remember_float("depth_frametime", STREAM_TYPE_DEPTH, PROPERTY_FRAMETIME);
        if (cfg.depth_exp > 0) remember_float("depth_exposure", STREAM_TYPE_DEPTH, PROPERTY_EXPOSURE);
        if (cfg.depth_gain > 0) remember_float("depth_gain", STREAM_TYPE_DEPTH, PROPERTY_GAIN);
        if (cfg.match)
            remember_extension(
                "depth_rgb_match", PROPERTY_EXT_DEPTH_RGB_MATCH_PARAM,
                [](const ::PropertyExtension& v) {
                    return std::to_string(v.depthRgbMatchParam.iDifThreshold) + "," +
                           std::to_string(v.depthRgbMatchParam.iRgbOffset) + "," +
                           std::to_string(int(v.depthRgbMatchParam.bMakeSureRgbIsAfterDepth));
                },
                [](::PropertyExtension& v, const std::string& t) {
                    const auto a = t.find(',');
                    const auto b = t.find(',', a == std::string::npos ? t.size() : a + 1);
                    if (a == std::string::npos || b == std::string::npos)
                        throw std::runtime_error("corrupt depth_rgb_match backup entry");
                    v.depthRgbMatchParam.iDifThreshold = std::stoi(t.substr(0, a));
                    v.depthRgbMatchParam.iRgbOffset = std::stoi(t.substr(a + 1, b - a - 1));
                    v.depthRgbMatchParam.bMakeSureRgbIsAfterDepth = std::stoi(t.substr(b + 1)) != 0;
                });
    }
    void set_prop(STREAM_TYPE stream, PROPERTY_TYPE key, double value) {
        float lo = 0, hi = 0, step = 0, actual = 0;
        check(camera->getPropertyRange(stream, key, lo, hi, step), "get property range");
        std::cout << "property stream=" << int(stream) << " key=" << int(key) << " range=" << lo
                  << ',' << hi << ',' << step << " requested=" << value << '\n';
        if (value < lo || value > hi) throw std::runtime_error("requested property outside device range");
        check(camera->setProperty(stream, key, static_cast<float>(value)), "set property");
        check(camera->getProperty(stream, key, actual), "read back property");
        if (!std::isfinite(actual) ||
            std::abs(actual - value) > std::max(0.001, std::abs(value) * 0.00001))
            throw std::runtime_error("property readback differs from request");
        std::cout << "property readback=" << actual << '\n';
    }
    // Only valid while the stream is RUNNING: guide 1.7 requires the stream to be started for
    // frame time / exposure / gain to take effect, so callers must not suspend first.
    void settings(bool rgb_only = false) {
        if (cfg.rgb_exp > 0 || cfg.rgb_gain > 0) {
            check(camera->setProperty(STREAM_TYPE_RGB, PROPERTY_ENABLE_AUTO_EXPOSURE, 0),
                  "disable RGB auto exposure");
            float actual = 1;
            check(camera->getProperty(STREAM_TYPE_RGB, PROPERTY_ENABLE_AUTO_EXPOSURE, actual),
                  "read RGB auto exposure");
            if (std::abs(actual) > 0.5f) throw std::runtime_error("RGB auto exposure remains enabled");
        }
        if (cfg.rgb_exp > 0) {
            ::PropertyExtension range{}, p{}, actual{};
            check(camera->getPropertyExtension(PROPERTY_EXT_EXPOSURE_TIME_RANGE_RGB, range),
                  "RGB exposure range");
            if (cfg.rgb_exp < range.objVRange_.fMin_ || cfg.rgb_exp > range.objVRange_.fMax_ ||
                std::floor(cfg.rgb_exp) != cfg.rgb_exp)
                throw std::runtime_error(
                    "RGB exposure outside the range the device reports; 3DCameraSDK 3.2.229 "
                    "hpp/Types.hpp describes uiExposureTime as milliseconds in Chinese and as us "
                    "in English on the same line, so the unit must be confirmed against the "
                    "observed range before trusting the configured value");
            p.uiExposureTime = static_cast<unsigned int>(cfg.rgb_exp);
            check(camera->setPropertyExtension(PROPERTY_EXT_EXPOSURE_TIME_RGB, p), "set RGB exposure");
            check(camera->getPropertyExtension(PROPERTY_EXT_EXPOSURE_TIME_RGB, actual),
                  "read RGB exposure");
            if (actual.uiExposureTime != p.uiExposureTime)
                throw std::runtime_error("RGB exposure readback mismatch");
            std::cout << "RGB exposure readback=" << actual.uiExposureTime << '\n';
        }
        if (cfg.rgb_gain > 0) set_prop(STREAM_TYPE_RGB, PROPERTY_GAIN, cfg.rgb_gain);
        if (rgb_only) return;
        if (cfg.depth_exp > 0 || cfg.depth_gain > 0 || cfg.depth_period > 0) {
            ::PropertyExtension p{}, actual{};
            p.autoExposureMode = AUTO_EXPOSURE_MODE_CLOSE;
            check(camera->setPropertyExtension(PROPERTY_EXT_AUTO_EXPOSURE_MODE, p),
                  "disable depth auto exposure");
            check(camera->getPropertyExtension(PROPERTY_EXT_AUTO_EXPOSURE_MODE, actual),
                  "read depth auto exposure");
            if (actual.autoExposureMode != p.autoExposureMode)
                throw std::runtime_error("depth auto exposure remains enabled");
        }
        // FAQ 2.4: frame time first, then exposure, and exposure must stay below frame time.
        if (cfg.depth_period > 0) set_prop(STREAM_TYPE_DEPTH, PROPERTY_FRAMETIME, cfg.depth_period);
        if (cfg.depth_exp > 0) {
            float period = 0;
            check(camera->getProperty(STREAM_TYPE_DEPTH, PROPERTY_FRAMETIME, period),
                  "read depth frame time");
            if (cfg.depth_exp >= period) throw std::runtime_error("depth exposure >= current frame time");
            set_prop(STREAM_TYPE_DEPTH, PROPERTY_EXPOSURE, cfg.depth_exp);
        }
        if (cfg.depth_gain > 0) set_prop(STREAM_TYPE_DEPTH, PROPERTY_GAIN, cfg.depth_gain);
        if (cfg.match) {
            ::PropertyExtension p{}, actual{};
            p.depthRgbMatchParam.iDifThreshold = cfg.threshold;
            p.depthRgbMatchParam.iRgbOffset = cfg.offset;
            p.depthRgbMatchParam.bMakeSureRgbIsAfterDepth = cfg.after;
            check(camera->setPropertyExtension(PROPERTY_EXT_DEPTH_RGB_MATCH_PARAM, p),
                  "set depth RGB match");
            check(camera->getPropertyExtension(PROPERTY_EXT_DEPTH_RGB_MATCH_PARAM, actual),
                  "read depth RGB match");
            if (actual.depthRgbMatchParam.iDifThreshold != cfg.threshold ||
                actual.depthRgbMatchParam.iRgbOffset != cfg.offset ||
                actual.depthRgbMatchParam.bMakeSureRgbIsAfterDepth != bool(cfg.after))
                throw std::runtime_error("depth RGB match readback mismatch");
        }
    }
    // SampleSoftTrigger order: the stream is already suspended when the trigger mode is written.
    void mode(bool soft) {
        ::PropertyExtension p{}, actual{};
        p.triggerMode = soft ? TRIGGER_MODE_SOFTWAER : TRIGGER_MODE_OFF;
        check(camera->setPropertyExtension(PROPERTY_EXT_TRIGGER_MODE, p), "set trigger mode");
        check(camera->getPropertyExtension(PROPERTY_EXT_TRIGGER_MODE, actual), "read trigger mode");
        if (actual.triggerMode != p.triggerMode)
            throw std::runtime_error("trigger mode readback mismatch");
    }
    void clear_both() { checked_rc(v5_clear_frame_buffer(0), "clear depth"); checked_rc(v5_clear_frame_buffer(1), "clear RGB"); }

    // A suspended stream emits nothing, so N consecutive timeouts prove the queue is empty.
    void require_idle(int stream, const char* what) {
        int quiet = 0, frames = 0;
        for (int i = 0; i < cfg.drain && quiet < cfg.quiet; ++i) {
            cs::IFramePtr frame;
            const auto rc = camera->getFrame(stream == 0 ? STREAM_TYPE_DEPTH : STREAM_TYPE_RGB,
                                             frame, 500);
            if (rc == ERROR_FRAME_TIMEOUT) {
                ++quiet;
                continue;
            }
            check(rc, "standby read");
            ++frames;
            quiet = 0;
        }
        if (frames > 0 || quiet < cfg.quiet)
            throw std::runtime_error(std::string(what) +
                                     ": stream still producing while it should be suspended");
    }
    // Belt and braces on top of require_idle: also empties anything the queue still holds.
    void drain_or_fail(int stream, const char* what) {
        const int drained = v5_drain_stream(stream, cfg.drain_ms, cfg.drain_frames);
        if (drained < 0) fail();
        if (drained >= cfg.drain_frames)
            throw std::runtime_error(std::string(what) + ": frame queue did not go empty");
    }

    // ---- lifecycle --------------------------------------------------------------------
    void open(const std::string& ip) {
        st_set_rgb_profile(0);
        checked_rc(st_connect(ip.c_str(), 0), "connect");
        depth_info = selection.depth;
        rgb_info = selection.rgb;
        backup.open(label, st_camera_serial);
        if (!start_streams()) fail();
        // Streams are running here: safe point for frame time / exposure / gain (guide 1.7).
        save_settings();
        settings();
        check(camera->pauseStream(STREAM_TYPE_DEPTH), "suspend depth");
        check(camera->pauseStream(STREAM_TYPE_RGB), "suspend RGB");
        mode(true);
        clear_both();
        require_idle(0, "startup");
        require_idle(1, "startup");
        opened = true;
        ++epoch;
        std::printf("Native camera ready: depth=%dx%d rgb=%dx%d standby(suspended)=true\n",
                    depth_info.width, depth_info.height, rgb_info.width, rgb_info.height);
        std::fflush(stdout);
    }
    // No stream rebuild: only the RGB (and optionally depth) stream is suspended or resumed.
    void set_streaming(bool enable) {
        if (streaming == enable) return;
        if (enable) {
            mode(false);  // Continuous acquisition needs the trigger source switched off first.
            check(camera->resumeStream(STREAM_TYPE_RGB), "resume RGB stream");
            if (cfg.preview_depth) check(camera->resumeStream(STREAM_TYPE_DEPTH), "resume depth stream");
            settings(true);  // RGB stream is running now.
            checked_rc(v5_clear_frame_buffer(1), "clear RGB");
            if (cfg.preview_depth) checked_rc(v5_clear_frame_buffer(0), "clear depth");
            began = Clock::now();
            count = 0;
            last_clear = began;
        } else {
            check(camera->pauseStream(STREAM_TYPE_RGB), "suspend RGB stream");
            if (cfg.preview_depth) check(camera->pauseStream(STREAM_TYPE_DEPTH), "suspend depth stream");
            mode(true);
            clear_both();
            drain_or_fail(1, "streaming stop RGB");  // Never leave streaming residue for a shot.
            if (cfg.preview_depth) drain_or_fail(0, "streaming stop depth");
        }
        latest.clear();
        ++epoch;
        previous_valid = false;
        streaming = enable;
        std::printf("Streaming %s: epoch=%llu (no stream restart)\n", enable ? "started" : "stopped",
                    static_cast<unsigned long long>(epoch));
        std::fflush(stdout);
    }
    void pump() {
        cs::IFramePtr rgb;
        const auto rc = camera->getFrame(STREAM_TYPE_RGB, rgb, cfg.preview_ms);
        if (rc == ERROR_FRAME_TIMEOUT) return;
        check(rc, "continuous RGB getFrame");
        if (!rgb || rgb->empty() || !rgb->getData() || rgb->getWidth() <= 0 || rgb->getHeight() <= 0)
            throw std::runtime_error("empty preview RGB");
        const uint64_t size = uint64_t(rgb->getWidth()) * rgb->getHeight() * 3;
        if (rgb->getFormat() != STREAM_FORMAT_RGB8 || size > capacity || rgb->getSize() <= 0 ||
            size != uint64_t(rgb->getSize()))
            throw std::runtime_error("preview RGB exceeds fixed capacity or wrong format");
        latest.resize(static_cast<size_t>(size));
        std::memcpy(latest.data(), rgb->getData(), latest.size());
        width = rgb->getWidth();
        height = rgb->getHeight();
        stamp = rgb->getTimeStamp();
        arrival = Clock::now();
        ++sequence;
        ++count;
        if (cfg.preview_depth) {  // Compatibility depth stream: consume, keep nothing.
            for (int i = 0; i < 4; ++i) {
                cs::IFramePtr depth;
                const auto dr = camera->getFrame(STREAM_TYPE_DEPTH, depth, 1);
                if (dr == ERROR_FRAME_TIMEOUT) break;
                check(dr, "preview drain depth");
            }
        }
    }
    // Runs on every main-loop turn, streaming or not.
    void tick() {
        const auto now = Clock::now();
        if (cfg.periodic_clear_s > 0 && streaming &&
            std::chrono::duration<double>(now - last_clear).count() >= cfg.periodic_clear_s) {
            last_clear = now;
            try {
                checked_rc(v5_clear_frame_buffer(1), "periodic clear RGB");
                if (cfg.preview_depth) checked_rc(v5_clear_frame_buffer(0), "periodic clear depth");
                std::printf("Periodic frame queue clear while streaming\n");
                std::fflush(stdout);
            } catch (const std::exception& e) {
                std::fprintf(stderr, "Periodic clear failed: %s\n", e.what());
            }
        }
#ifndef GDY_NATIVE_STATE_TEST
        if (cfg.memory_report_s > 0 &&
            std::chrono::duration<double>(now - last_memory).count() >= cfg.memory_report_s) {
            last_memory = now;
            long rss = -1, swap = -1;
            if (std::FILE* f = std::fopen("/proc/self/status", "r")) {
                char line[256];
                while (std::fgets(line, sizeof(line), f)) {
                    if (!std::strncmp(line, "VmRSS:", 6)) rss = std::atol(line + 6);
                    else if (!std::strncmp(line, "VmSwap:", 7)) swap = std::atol(line + 7);
                }
                std::fclose(f);
            }
            std::printf("MEMORY worker rss_kb=%ld swap_kb=%ld streaming=%d\n", rss, swap,
                        streaming ? 1 : 0);
            std::fflush(stdout);
        }
#endif
    }
    std::string preview_frame(Mapping& memory) {
        if (!streaming) throw std::runtime_error("streaming is not active");
        if (latest.empty()) return "\"ok\":true,\"available\":false";
        std::memcpy(static_cast<unsigned char*>(memory.data) + capacity, latest.data(), latest.size());
        const auto now = Clock::now();
        std::ostringstream s;
        s << std::setprecision(12) << "\"ok\":true,\"available\":true,\"width\":" << width
          << ",\"height\":" << height << ",\"size\":" << latest.size()
          << ",\"sequence\":" << sequence << ",\"epoch\":" << epoch
          << ",\"timestamp_ms\":" << (std::isfinite(stamp) ? stamp : 0)
          << ",\"age_ms\":" << std::chrono::duration<double, std::milli>(now - arrival).count()
          << ",\"acquisition_fps\":"
          << count / std::max(0.001, std::chrono::duration<double>(now - began).count());
        return s.str();
    }
    std::string capture(Mapping& memory) {
        const bool resume = streaming;
        if (resume) set_streaming(false);  // Suspend streaming first: no startStream/stopStream.
        const auto begin = Clock::now();
        clear_both();
        const auto cleared = Clock::now();
        drain_or_fail(0, "capture depth");
        drain_or_fail(1, "capture RGB");
        const auto drained = Clock::now();
        checked_rc(st_trigger(), "softTrigger");
        const auto triggered = Clock::now();

        double stamps[2]{};
        int sizes[2]{}, widths[2]{}, heights[2]{};
        int rc = v5_read_frame(0, cfg.capture_ms, memory.data, int(capacity), &stamps[0], &sizes[0],
                               &widths[0], &heights[0]);
        if (rc == 1)
            throw std::runtime_error(
                "depth frame timeout after trigger (check PROPERTY_FRAMETIME / trigger mode)");
        if (rc != 0) fail();
        const auto depth_at = Clock::now();

        rc = v5_read_frame(1, cfg.rgb_read_ms, static_cast<unsigned char*>(memory.data) + capacity,
                           int(capacity), &stamps[1], &sizes[1], &widths[1], &heights[1]);
        if (rc == 1)
            throw std::runtime_error(
                "RGB frame timeout after trigger: the RGB stream may not follow the soft trigger "
                "(verify assumption A2 in V5_1_PREVIEW.md, then use preview_with_depth or the "
                "resume-then-suspend fallback)");
        if (rc != 0) fail();
        const auto rgb_at = Clock::now();

        const bool valid = std::isfinite(stamps[0]) && std::isfinite(stamps[1]) && stamps[0] > 0 &&
                           stamps[1] > 0;
        const bool advancing =
            !previous_valid || (stamps[0] > previous_depth && stamps[1] > previous_rgb);
        // Raw gap is diagnostic only: SDK offset matching is not proof of physical simultaneity.
        const double gap = valid ? stamps[1] - stamps[0] : 0;
        const bool within = valid && advancing && std::abs(gap) <= cfg.threshold &&
                            (!cfg.after || gap >= 0);
        if (!valid || !advancing) {
            if (cfg.strict)
                throw std::runtime_error("timestamps unusable or not advancing (timestamp_policy=strict)");
            std::cerr << "WARNING: RGB-depth timestamps not verified: depth=" << stamps[0]
                      << " RGB=" << stamps[1] << " raw_gap_ms=" << gap << '\n';
        } else if (!within && cfg.strict) {
            throw std::runtime_error("raw RGB-depth gap above the configured limit");
        }
        previous_depth = stamps[0];
        previous_rgb = stamps[1];
        previous_valid = valid;
        const auto capture_epoch = epoch;
        const double seconds = std::chrono::duration<double>(Clock::now() - begin).count();
        if (resume) set_streaming(true);

        std::printf(
            "CAPTURE depth=%.1fms rgb=%.1fms trigger=%.1fms clear=%.1fms drain=%.1fms total=%.1fms\n",
            std::chrono::duration<double, std::milli>(depth_at - triggered).count(),
            std::chrono::duration<double, std::milli>(rgb_at - depth_at).count(),
            std::chrono::duration<double, std::milli>(triggered - drained).count(),
            std::chrono::duration<double, std::milli>(cleared - begin).count(),
            std::chrono::duration<double, std::milli>(drained - cleared).count(), seconds * 1000.0);
        std::fflush(stdout);

        std::ostringstream s;
        s << std::setprecision(12) << "\"ok\":true,\"widths\":[" << widths[0] << ',' << widths[1]
          << "],\"heights\":[" << heights[0] << ',' << heights[1] << "],\"sizes\":[" << sizes[0]
          << ',' << sizes[1] << "],\"stamps\":[" << (std::isfinite(stamps[0]) ? stamps[0] : 0) << ','
          << (std::isfinite(stamps[1]) ? stamps[1] : 0) << "],\"capture_s\":" << seconds
          << ",\"epoch\":" << capture_epoch << ",\"timestamp_valid\":" << (valid ? "true" : "false")
          << ",\"timestamp_within_limit\":" << (within ? "true" : "false")
          << ",\"raw_gap_ms\":" << gap << ",\"depth_ms\":"
          << std::chrono::duration<double, std::milli>(depth_at - triggered).count()
          << ",\"rgb_ms\":" << std::chrono::duration<double, std::milli>(rgb_at - depth_at).count()
          << ",\"trigger_ms\":"
          << std::chrono::duration<double, std::milli>(triggered - drained).count()
          << ",\"clear_ms\":"
          << std::chrono::duration<double, std::milli>(cleared - begin).count()
          << ",\"drain_ms\":" << std::chrono::duration<double, std::milli>(drained - cleared).count()
          << ",\"stream_restarts\":0";
        return s.str();
    }
};

#ifndef GDY_NATIVE_STATE_TEST
bool read_line(std::string& line) {
    line.clear();
    char c;
    while (true) {
        auto n = ::read(STDIN_FILENO, &c, 1);
        if (n == 0) return false;
        if (n < 0) {
            if (errno == EINTR) continue;
            throw std::runtime_error("stdin read failed");
        }
        if (c == '\n') return true;
        if (line.size() >= 4096) throw std::runtime_error("command too large");
        line += c;
    }
}
#endif
}  // namespace

#ifndef GDY_NATIVE_STATE_TEST
int main(int argc, char** argv) {
    long id = -1;
    try {
        if (argc != 3 && argc != 4)
            throw std::runtime_error("usage: worker shared-buffer parent-pid [property-backup]");
        if (prctl(PR_SET_PDEATHSIG, SIGKILL) || getppid() != std::stoi(argv[2]))
            throw std::runtime_error("parent death protection failed");
        OwnerLock lock;
        Mapping memory(argv[1]);
        // The backup path is optional so older launchers keep working.
        CameraOwner owner(argc == 4 ? std::string(argv[3]) : std::string());
        while (true) {
            pollfd fd{STDIN_FILENO, POLLIN, 0};
            // Idle normally blocks forever, but a slow poll keeps the memory report alive so a
            // long standby can be watched from the service log.
            const int wait_ms = owner.streaming ? 0 : (owner.cfg.memory_report_s > 0 ? 1000 : -1);
            int rc = poll(&fd, 1, wait_ms);
            if (rc < 0) {
                if (errno == EINTR) continue;
                throw std::runtime_error("poll failed");
            }
            if (fd.revents & POLLIN) {
                std::string line, op;
                if (!read_line(line)) break;
                std::istringstream in(line);
                if (!(in >> id >> op) || id < 1) throw std::runtime_error("bad command header");
                if (op == "open") {
                    if (owner.opened) throw std::runtime_error("already open");
                    std::string ip;
                    in >> ip;
                    owner.cfg.parse(in);
                    owner.open(ip);
                    double scale = 0;
                    checked_rc(v5_depth_scale(&scale), "depthScale");
                    std::ostringstream s;
                    s << std::setprecision(12) << "\"ok\":true,\"depth_scale_mm\":" << scale
                      << ",\"widths\":[" << owner.depth_info.width << ',' << owner.rgb_info.width
                      << "],\"heights\":[" << owner.depth_info.height << ',' << owner.rgb_info.height
                      << "],\"standby\":\"suspended\"";
                    reply(id, s.str());
                } else if (op == "close") {
                    if (!owner.close())
                        throw std::runtime_error("SDK cleanup/property restore failed; see cleanup log");
                    reply(id, "\"ok\":true");
                    break;
                } else {
                    if (!owner.opened) throw std::runtime_error("camera not open");
                    if (op == "capture")
                        reply(id, owner.capture(memory));
                    else if (op == "preview_start" || op == "preview_stop") {
                        owner.set_streaming(op == "preview_start");
                        reply(id, "\"ok\":true");
                    } else if (op == "preview_frame")
                        reply(id, owner.preview_frame(memory));
                    else
                        throw std::runtime_error("unknown command");
                }
            } else if (fd.revents & (POLLHUP | POLLERR | POLLNVAL)) {
                break;
            }
            if (owner.streaming) owner.pump();
            owner.tick();
        }
        return 0;
    } catch (const std::exception& e) {
        reply(id, "\"ok\":false,\"error\":\"" + escaped(e.what()) + "\"");
        return 1;
    } catch (...) {
        reply(id, "\"ok\":false,\"error\":\"unknown native exception\"");
        return 1;
    }
}
#endif
