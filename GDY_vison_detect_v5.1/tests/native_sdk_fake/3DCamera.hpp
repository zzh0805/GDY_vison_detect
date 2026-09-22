// OFFLINE TEST DOUBLE ONLY. Never put this include directory in a production build.
//
// It mirrors the real 3DCamera SDK where the offline state test depends on it:
//   * PropertyExtension is a UNION: a get fills ONLY the field of the requested property and
//     a set reads ONLY that field. Writing the wrong member is a silent no-op on the real
//     device, so the fake must behave the same way to keep tests meaningful.
//   * Real numeric values: TRIGGER_MODE_OFF=0/SOFTWAER=2, STREAM_FORMAT_RGB8=0x01/Z16=0x02,
//     SUCCESS=0 and ERROR_FRAME_TIMEOUT being a distinct non-zero code.
//   * CameraInfo fields are fixed-size arrays with no guaranteed NUL terminator.
//   * pauseStream() stops the sensor from producing frames; softTrigger(n) produces a frame on
//     every started stream; getFrame() is FIFO over the SDK queue, which is why a stale frame
//     can only be avoided by clearing + draining before the trigger.
#pragma once
#include <deque>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// ---------------------------------------------------------------- vendor enums / POD types
enum ERROR_CODE {
    SUCCESS = 0,
    ERROR_PARAM,
    ERROR_DEVICE_NOT_FOUND,
    ERROR_DEVICE_NOT_CONNECT,
    ERROR_GET_STREAM_INFO_FAILED,
    ERROR_START_STREAM_FAILED,
    ERROR_STOP_STREAM_FAILED,
    ERROR_STREAM_BUSY,
    ERROR_FRAME_TIMEOUT,
    ERROR_TEST
};

enum STREAM_TYPE { STREAM_TYPE_DEPTH = 0, STREAM_TYPE_RGB = 1, STREAM_TYPE_COUNT };
enum STREAM_FORMAT {
    STREAM_FORMAT_MJPG = 0x00,
    STREAM_FORMAT_RGB8 = 0x01,
    STREAM_FORMAT_Z16 = 0x02,
};
enum PROPERTY_TYPE {
    PROPERTY_GAIN = 0x00,
    PROPERTY_EXPOSURE = 0x01,
    PROPERTY_FRAMETIME = 0x02,
    PROPERTY_ENABLE_AUTO_EXPOSURE = 0x05,
};
enum PROPERTY_TYPE_EXTENSION {
    PROPERTY_EXT_DEPTH_SCALE = 0x0,
    PROPERTY_EXT_TRIGGER_MODE = 0x1,
    PROPERTY_EXT_DEPTH_RGB_MATCH_PARAM = 0x0C,
    PROPERTY_EXT_PAUSE_DEPTH_STREAM = 0x0D,
    PROPERTY_EXT_RESUME_DEPTH_STREAM = 0x0E,
    PROPERTY_EXT_CLEAR_FRAME_BUFFER = 0x13,
    PROPERTY_EXT_EXPOSURE_TIME_RGB = 0x15,
    PROPERTY_EXT_EXPOSURE_TIME_RANGE_RGB = 0x16,
    PROPERTY_EXT_AUTO_EXPOSURE_MODE = 0x912,
};
enum TRIGGER_MODE { TRIGGER_MODE_OFF = 0, TRIGGER_MODE_HARDWAER = 1, TRIGGER_MODE_SOFTWAER = 2 };
enum AUTO_EXPOSURE_MODE {
    AUTO_EXPOSURE_MODE_CLOSE = 0,
    AUTO_EXPOSURE_MODE_FIX_FRAMETIME = 1,
    AUTO_EXPOSURE_MODE_HIGH_QUALITY = 2,
    AUTO_EXPOSURE_MODE_FORE_GROUND = 3,
};

struct StreamInfo {
    STREAM_FORMAT format = STREAM_FORMAT_Z16;
    int width = 2;
    int height = 2;
    float fps = 30.0f;
};
struct CameraInfo {
    char name[32];
    char serial[32];
    char uniqueId[32];
    char firmwareVersion[32];
    char algorithmVersion[32];
};
// No default member initializers: the real union is a trivial C type and `T x{}` must stay valid.
struct DepthRgbMatchParam {
    int iRgbOffset;
    int iDifThreshold;
    bool bMakeSureRgbIsAfterDepth;
};
struct ValueRange {
    float fMin_;
    float fMax_;
    float fStep_;
};
union PropertyExtension {
    float depthScale;
    TRIGGER_MODE triggerMode;
    TRIGGER_MODE triggerInMode;
    AUTO_EXPOSURE_MODE autoExposureMode;
    STREAM_TYPE streamType;
    unsigned int uiExposureTime;
    ValueRange objVRange_;
    DepthRgbMatchParam depthRgbMatchParam;
};

// ---------------------------------------------------------------- fake device state
namespace fake {
// What the emulated device remembers. Kept OUTSIDE PropertyExtension on purpose: the real get
// fills exactly one union field, so the double must be able to hand back one field at a time.
struct Device {
    TRIGGER_MODE triggerMode = TRIGGER_MODE_OFF;
    float depthScale = 0.1f;
    unsigned int rgbExposure = 5000;
    float rgbExpMin = 1.0f, rgbExpMax = 1000000.0f, rgbExpStep = 1.0f;
    AUTO_EXPOSURE_MODE depthAutoExposure = AUTO_EXPOSURE_MODE_HIGH_QUALITY;
    // Device default 0 reproduces the field symptom: pairing never matches until configured.
    int matchDifThreshold = 0;
    int matchRgbOffset = 0;
    bool matchRgbAfterDepth = false;
    float depthFrametime = 10000.0f, depthExposure = 8000.0f, depthGain = 1.0f;
    float rgbGain = 1.0f, rgbAutoExposure = 1.0f;
};
// The double is deliberately a single-translation-unit fixture (the state test is the only
// consumer), so the emulated device state is file-static and stays C++14-clean.
static Device device;

struct Frame {
    STREAM_FORMAT format = STREAM_FORMAT_Z16;
    int width = 2, height = 2;
    double stamp = 0.0;
    std::vector<unsigned char> bytes;
};

static int frames_live = 0, frames_peak = 0;
static int triggers = 0, stream_starts = 0, stream_stops = 0, paired_reads = 0;
static int clears[2] = {0, 0};                 // lifetime count per stream
static int clears_since_trigger[2] = {0, 0};   // reset by softTrigger: proves "clear every shot"
static int triggered_with_backlog = 0;         // >0 means a shot started from a dirty queue
static int produced[2] = {0, 0};               // frames the device emitted, per stream
static bool started[2] = {false, false};
static bool paused[2] = {true, true};
static bool connected = false;
static std::deque<Frame> queue[2];
static double tick = 1.0;

// Fault injection.
static bool timeout = false;          // every getFrame reports the documented timeout
static bool bad_rgb = false;          // RGB frame comes back with the wrong format
static bool zero_stamp = false;       // timestamps stop being usable
static bool fail_clear = false;       // PROPERTY_EXT_CLEAR_FRAME_BUFFER returns an error
static bool fail_gain = false;        // PROPERTY_GAIN set returns an error
static bool fail_start = false;       // startStream returns an error
static bool fail_pause = false;       // pauseStream returns an error
static bool fail_resume = false;      // resumeStream returns an error
static bool fail_disconnect = false;  // disconnect returns an error
static bool nonterminated_id = false; // CameraInfo.uniqueId is filled with 32 non-NUL bytes

inline const char* id_text() { return nonterminated_id ? "ABCDEFGHIJKLMNOPQRSTUVWXYZ012345" : "test"; }

inline Frame make(STREAM_FORMAT format, double stamp_bias) {
    Frame frame;
    frame.format = format;
    frame.bytes.assign(format == STREAM_FORMAT_Z16 ? 8 : 12, 7);
    frame.stamp = zero_stamp ? 0.0 : (tick += 1.0) + stamp_bias;
    return frame;
}
// A frame the emulated device emits on its own (free-running stream or a trigger).
inline void emit(STREAM_TYPE stream) {
    ++produced[stream];
    queue[stream].push_back(make(stream == STREAM_TYPE_DEPTH ? STREAM_FORMAT_Z16
                                                             : STREAM_FORMAT_RGB8,
                                 0.0));
}
// Test helper: pretend the device kept producing while nobody was reading.
inline void inject_backlog(STREAM_TYPE stream, int count) {
    for (int i = 0; i < count; ++i) emit(stream);
}
inline void reset() {
    frames_live = frames_peak = triggers = stream_starts = stream_stops = paired_reads = 0;
    clears[0] = clears[1] = clears_since_trigger[0] = clears_since_trigger[1] = 0;
    triggered_with_backlog = 0;
    produced[0] = produced[1] = 0;
    started[0] = started[1] = false;
    paused[0] = paused[1] = true;
    connected = false;
    queue[0].clear();
    queue[1].clear();
    tick = 1.0;
}
}  // namespace fake

// ---------------------------------------------------------------- cs interfaces
namespace cs {

class IFrame {
    STREAM_FORMAT format_;
    int width_, height_;
    double stamp_;
    std::vector<unsigned char> bytes_;

public:
    IFrame(const fake::Frame& source) : format_(source.format), width_(source.width),
                                        height_(source.height), stamp_(source.stamp),
                                        bytes_(source.bytes) {
        ++fake::frames_live;
        if (fake::frames_live > fake::frames_peak) fake::frames_peak = fake::frames_live;
    }
    ~IFrame() { --fake::frames_live; }
    bool empty() const { return bytes_.empty(); }
    double getTimeStamp() const { return stamp_; }
    const char* getData() const { return reinterpret_cast<const char*>(bytes_.data()); }
    int getSize() const { return static_cast<int>(bytes_.size()); }
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    STREAM_FORMAT getFormat() const {
        return (fake::bad_rgb && format_ == STREAM_FORMAT_RGB8) ? STREAM_FORMAT_Z16 : format_;
    }
};
using IFramePtr = std::shared_ptr<IFrame>;

class ICamera {
public:
    ERROR_CODE connect(CameraInfo) { fake::connected = true; return SUCCESS; }
    ERROR_CODE disconnect() {
        if (fake::fail_disconnect) return ERROR_TEST;
        fake::connected = false;
        return SUCCESS;
    }
    ERROR_CODE getStreamInfos(STREAM_TYPE type, std::vector<StreamInfo>& out) {
        StreamInfo info;
        info.format = (type == STREAM_TYPE_DEPTH) ? STREAM_FORMAT_Z16 : STREAM_FORMAT_RGB8;
        out = {info};
        return SUCCESS;
    }
    // The two single-stream overloads are the documented way to run getFrame and are the only
    // startStream forms the production worker may use.
    ERROR_CODE startStream(STREAM_TYPE type, StreamInfo) {
        if (fake::fail_start) return ERROR_START_STREAM_FAILED;
        ++fake::stream_starts;
        fake::started[type] = true;
        fake::paused[type] = false;  // The sensor free-runs until pauseStream().
        return SUCCESS;
    }
    ERROR_CODE startStream(StreamInfo, StreamInfo, void*, void*) {
        return ERROR_PARAM;  // Paired callback form is deliberately unavailable: v5.1 must not use it.
    }
    ERROR_CODE pauseStream(STREAM_TYPE type) {
        if (fake::fail_pause) return ERROR_TEST;
        if (!fake::started[type]) return ERROR_PARAM;
        fake::paused[type] = true;
        return SUCCESS;
    }
    ERROR_CODE resumeStream(STREAM_TYPE type) {
        if (fake::fail_resume) return ERROR_TEST;
        if (!fake::started[type]) return ERROR_PARAM;
        fake::paused[type] = false;
        return SUCCESS;
    }
    ERROR_CODE stopStream() {
        ++fake::stream_stops;
        fake::started[0] = fake::started[1] = false;
        fake::paused[0] = fake::paused[1] = true;
        fake::queue[0].clear();
        fake::queue[1].clear();
        return SUCCESS;
    }
    ERROR_CODE stopStream(STREAM_TYPE type) { (void)type; return stopStream(); }
    ERROR_CODE getPropertyRange(STREAM_TYPE, PROPERTY_TYPE, float& low, float& high, float& step) {
        low = 0.0f;
        high = 1000000.0f;
        step = 1.0f;
        return SUCCESS;
    }
    ERROR_CODE getProperty(STREAM_TYPE type, PROPERTY_TYPE prop, float& value) {
        if (type == STREAM_TYPE_RGB) {
            value = (prop == PROPERTY_GAIN) ? fake::device.rgbGain
                   : (prop == PROPERTY_ENABLE_AUTO_EXPOSURE) ? fake::device.rgbAutoExposure
                                                             : 1.0f;
            return SUCCESS;
        }
        value = (prop == PROPERTY_FRAMETIME) ? fake::device.depthFrametime
               : (prop == PROPERTY_EXPOSURE) ? fake::device.depthExposure
               : (prop == PROPERTY_GAIN)     ? fake::device.depthGain
                                             : 1.0f;
        return SUCCESS;
    }
    ERROR_CODE setProperty(STREAM_TYPE type, PROPERTY_TYPE prop, float value) {
        if (fake::fail_gain && prop == PROPERTY_GAIN) return ERROR_TEST;
        if (type == STREAM_TYPE_RGB) {
            if (prop == PROPERTY_GAIN) fake::device.rgbGain = value;
            if (prop == PROPERTY_ENABLE_AUTO_EXPOSURE) fake::device.rgbAutoExposure = value;
            return SUCCESS;
        }
        if (prop == PROPERTY_FRAMETIME) fake::device.depthFrametime = value;
        if (prop == PROPERTY_EXPOSURE) fake::device.depthExposure = value;
        if (prop == PROPERTY_GAIN) fake::device.depthGain = value;
        return SUCCESS;
    }
    // Only the union field that belongs to the requested property is read or written.
    ERROR_CODE getPropertyExtension(PROPERTY_TYPE_EXTENSION key, PropertyExtension& value) {
        switch (key) {
            case PROPERTY_EXT_DEPTH_SCALE: value.depthScale = fake::device.depthScale; break;
            case PROPERTY_EXT_TRIGGER_MODE: value.triggerMode = fake::device.triggerMode; break;
            case PROPERTY_EXT_EXPOSURE_TIME_RGB: value.uiExposureTime = fake::device.rgbExposure; break;
            case PROPERTY_EXT_EXPOSURE_TIME_RANGE_RGB:
                value.objVRange_.fMin_ = fake::device.rgbExpMin;
                value.objVRange_.fMax_ = fake::device.rgbExpMax;
                value.objVRange_.fStep_ = fake::device.rgbExpStep;
                break;
            case PROPERTY_EXT_AUTO_EXPOSURE_MODE: value.autoExposureMode = fake::device.depthAutoExposure; break;
            case PROPERTY_EXT_DEPTH_RGB_MATCH_PARAM:
                value.depthRgbMatchParam.iDifThreshold = fake::device.matchDifThreshold;
                value.depthRgbMatchParam.iRgbOffset = fake::device.matchRgbOffset;
                value.depthRgbMatchParam.bMakeSureRgbIsAfterDepth = fake::device.matchRgbAfterDepth;
                break;
            default: return ERROR_PARAM;
        }
        return SUCCESS;
    }
    ERROR_CODE setPropertyExtension(PROPERTY_TYPE_EXTENSION key, PropertyExtension value) {
        switch (key) {
            case PROPERTY_EXT_TRIGGER_MODE:
                fake::device.triggerMode = value.triggerMode;
                break;
            case PROPERTY_EXT_EXPOSURE_TIME_RGB:
                fake::device.rgbExposure = value.uiExposureTime;
                break;
            case PROPERTY_EXT_AUTO_EXPOSURE_MODE:
                fake::device.depthAutoExposure = value.autoExposureMode;
                break;
            case PROPERTY_EXT_DEPTH_RGB_MATCH_PARAM:
                fake::device.matchDifThreshold = value.depthRgbMatchParam.iDifThreshold;
                fake::device.matchRgbOffset = value.depthRgbMatchParam.iRgbOffset;
                fake::device.matchRgbAfterDepth = value.depthRgbMatchParam.bMakeSureRgbIsAfterDepth;
                break;
            case PROPERTY_EXT_CLEAR_FRAME_BUFFER: {
                if (value.streamType != STREAM_TYPE_DEPTH && value.streamType != STREAM_TYPE_RGB)
                    return ERROR_PARAM;
                if (fake::fail_clear) return ERROR_TEST;
                ++fake::clears[value.streamType];
                ++fake::clears_since_trigger[value.streamType];
                fake::queue[value.streamType].clear();  // Empties the SDK frame queue.
                break;
            }
            case PROPERTY_EXT_PAUSE_DEPTH_STREAM:
                fake::paused[STREAM_TYPE_DEPTH] = true;
                break;
            case PROPERTY_EXT_RESUME_DEPTH_STREAM:
                fake::paused[STREAM_TYPE_DEPTH] = false;
                break;
            default:
                return ERROR_PARAM;
        }
        return SUCCESS;
    }
    ERROR_CODE softTrigger(int count = 1) {
        if (count != 1) return ERROR_PARAM;
        if (!fake::started[STREAM_TYPE_DEPTH]) throw std::runtime_error("trigger before startStream");
        // A dirty queue at trigger time is exactly how a stale frame reaches the caller.
        if (!fake::queue[0].empty() || !fake::queue[1].empty()) ++fake::triggered_with_backlog;
        ++fake::triggers;
        // A trigger emits one frame on every stream that was started, suspended or not.
        if (fake::started[STREAM_TYPE_DEPTH]) fake::emit(STREAM_TYPE_DEPTH);
        if (fake::started[STREAM_TYPE_RGB]) fake::emit(STREAM_TYPE_RGB);
        fake::clears_since_trigger[0] = fake::clears_since_trigger[1] = 0;
        return SUCCESS;
    }
    // FIFO on the emulated SDK queue; a suspended stream produces nothing on its own.
    ERROR_CODE getFrame(STREAM_TYPE type, IFramePtr& frame, int) {
        if (fake::timeout) return ERROR_FRAME_TIMEOUT;
        if (!fake::started[type]) return ERROR_PARAM;
        if (fake::queue[type].empty()) {
            if (fake::paused[type]) return ERROR_FRAME_TIMEOUT;  // Suspended: only a trigger emits.
            fake::emit(type);                                    // Free-running: emit and take it.
        }
        frame = std::make_shared<IFrame>(fake::queue[type].front());
        fake::queue[type].pop_front();
        return SUCCESS;
    }
    ERROR_CODE getPairedFrame(IFramePtr& depth, IFramePtr& rgb, int) {
        ++fake::paired_reads;
        if (fake::timeout) return ERROR_FRAME_TIMEOUT;
        if (fake::queue[0].empty() || fake::queue[1].empty()) return ERROR_FRAME_TIMEOUT;
        depth = std::make_shared<IFrame>(fake::queue[0].front());
        rgb = std::make_shared<IFrame>(fake::queue[1].front());
        fake::queue[0].pop_front();
        fake::queue[1].pop_front();
        return SUCCESS;
    }
};
using ICameraPtr = std::shared_ptr<ICamera>;

class ISystem {
public:
    ERROR_CODE queryCameras(std::vector<CameraInfo>& out, int) {
        CameraInfo info{};
        std::strncpy(info.name, "fake", sizeof(info.name) - 1);
        std::strncpy(info.serial, "test", sizeof(info.serial) - 1);
        if (fake::nonterminated_id) {
            // 32 usable bytes, zero terminators: a real device may do exactly this. The following
            // fields are poisoned so that an unbounded std::string(CameraInfo::uniqueId) would
            // keep reading into them and no longer compare equal - i.e. the offline test really
            // distinguishes a bounded read from an over-read instead of passing by luck.
            const char* text = "0123456789ABCDEFGHIJKLMNOPQRSTUV";
            std::memcpy(info.uniqueId, text, sizeof(info.uniqueId));
            std::memset(info.firmwareVersion, 'X', sizeof(info.firmwareVersion));
            std::memset(info.algorithmVersion, 'Y', sizeof(info.algorithmVersion));
        } else {
            std::strncpy(info.uniqueId, "test", sizeof(info.uniqueId) - 1);
        }
        out = {info};
        return SUCCESS;
    }
};
using ISystemPtr = std::shared_ptr<ISystem>;

inline ISystemPtr getSystemPtr() { return std::make_shared<ISystem>(); }
inline ICameraPtr getCameraPtr() { return std::make_shared<ICamera>(); }
inline void setSdkEnableNetworking(bool) {}
inline void setEnableNetworking(bool) {}
inline const char* getCameraErrorString(ERROR_CODE code) {
    return code == SUCCESS ? "SUCCESS" : "device error";
}

}  // namespace cs
