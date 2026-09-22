// Thin extern "C" layer used by the v5.1 C++ worker (and, unchanged, by the legacy ctypes
// bridge). Every raw SDK frame is copied into a caller-owned fixed buffer BEFORE the SDK
// shared_ptr is released, so no vendor pointer ever escapes this translation unit.
#include "sdk_lifecycle.hpp"
#include <cstring>
#include <cmath>

// Private selector: 0=depth, 1=RGB (NOT the vendor enum numeric values).
static ::STREAM_TYPE stream_of(int selector) {
    if (selector == 0) return ::STREAM_TYPE_DEPTH;
    if (selector == 1) return ::STREAM_TYPE_RGB;
    throw std::runtime_error("invalid stream selector");
}

// PROPERTY_EXT_CLEAR_FRAME_BUFFER empties the SDK's internal frame queue for one stream
// (guide: "清除SDK内部的Frame队列"). Both queues must be cleared before a trigger, otherwise
// getFrame/getPairedFrame can hand back a frame captured before the trigger.
extern "C" int v5_clear_frame_buffer(int stream) {
    try {
        if (!camera || !started) throw std::runtime_error("clear frame buffer: camera not streaming");
        const ::STREAM_TYPE type = stream_of(stream);
        ::PropertyExtension property{};
        property.streamType = type;
        check(camera->setPropertyExtension(::PROPERTY_EXT_CLEAR_FRAME_BUFFER, property),
              stream == 0 ? "clear depth frame buffer" : "clear RGB frame buffer");
        return 0;
    } catch (const std::exception& e) {
        error = e.what();
    } catch (...) {
        error = "unknown exception clearing frame buffer";
    }
    return -1;
}

// Single-stream active polling (guide 1.6 "主动取帧方式"). This is the documented way to take
// exactly the frame produced by one softTrigger, without the pairing matcher in between.
extern "C" int v5_read_frame(int stream, int timeout_ms, void* buffer, int capacity,
                             double* stamp, int* size, int* width, int* height) {
    try {
        if (!camera || !started) throw std::runtime_error("read frame: camera not streaming");
        if (!buffer || capacity <= 0) throw std::runtime_error("read frame: invalid buffer");
        const ::STREAM_TYPE type = stream_of(stream);
        const ::STREAM_FORMAT expected = stream == 0 ? ::STREAM_FORMAT_Z16 : ::STREAM_FORMAT_RGB8;
        const int channels = stream == 0 ? 2 : 3;

        cs::IFramePtr frame;
        const auto rc = camera->getFrame(type, frame, timeout_ms);
        if (rc == ::ERROR_FRAME_TIMEOUT) return 1;  // Documented timeout: no stale frame returned.
        check(rc, "getFrame");
        if (!frame || frame->empty() || !frame->getData() || frame->getWidth() <= 0 ||
            frame->getHeight() <= 0)
            throw std::runtime_error("SDK returned an invalid frame");
        const std::uint64_t bytes =
            static_cast<std::uint64_t>(frame->getWidth()) * frame->getHeight() * channels;
        if (frame->getFormat() != expected || frame->getSize() <= 0 ||
            bytes > static_cast<std::uint64_t>(capacity) ||
            static_cast<std::uint64_t>(frame->getSize()) != bytes)
            throw std::runtime_error("frame format/size exceeds fixed buffer");
        std::memcpy(buffer, frame->getData(), static_cast<std::size_t>(bytes));
        if (stamp) *stamp = frame->getTimeStamp();
        if (size) *size = frame->getSize();
        if (width) *width = frame->getWidth();
        if (height) *height = frame->getHeight();
        return 0;
    } catch (const std::exception& e) {
        error = e.what();
    } catch (...) {
        error = "unknown exception copying native frame";
    }
    return -1;
}

// Discard queued frames until the documented timeout proves the queue is empty. A suspended
// stream emits nothing on its own, so a healthy standby queue is empty and this costs one
// timeout period. Returning max_frames means the stream is still producing, i.e. not suspended.
extern "C" int v5_drain_stream(int stream, int timeout_ms, int max_frames) {
    try {
        if (!camera || !started) throw std::runtime_error("drain: camera not streaming");
        if (max_frames <= 0) throw std::runtime_error("drain: invalid frame budget");
        const ::STREAM_TYPE type = stream_of(stream);
        int drained = 0;
        for (; drained < max_frames; ++drained) {
            cs::IFramePtr frame;
            const auto rc = camera->getFrame(type, frame, timeout_ms);
            if (rc == ::ERROR_FRAME_TIMEOUT) return drained;
            check(rc, "drain getFrame");
        }
        return drained;  // Budget exhausted: the caller decides whether that is fatal.
    } catch (const std::exception& e) {
        error = e.what();
    } catch (...) {
        error = "unknown exception draining stream";
    }
    return -1;
}

// --- helpers kept from the v5 bridge (selector/profile/scale) -----------------------------

extern "C" int v5_profile_sizes(int* widths, int* heights) {
    try {
        for (int i = 0; i < 2; ++i) {
            std::vector<::StreamInfo> infos;
            check(camera->getStreamInfos(i == 0 ? ::STREAM_TYPE_DEPTH : ::STREAM_TYPE_RGB, infos),
                  "stream info");
            bool found = false;
            for (const auto& info : infos)
                if (info.format == (i == 0 ? ::STREAM_FORMAT_Z16 : ::STREAM_FORMAT_RGB8)) {
                    widths[i] = info.width;
                    heights[i] = info.height;
                    found = true;
                    break;
                }
            if (!found) throw std::runtime_error("selected profile missing");
        }
        return 0;
    } catch (const std::exception& e) {
        error = e.what();
    } catch (...) {
        error = "unknown profile exception";
    }
    return -1;
}

extern "C" int v5_depth_scale(double* value) {
    try {
        ::PropertyExtension property{};
        check(camera->getPropertyExtension(::PROPERTY_EXT_DEPTH_SCALE, property), "read depthScale");
        if (!std::isfinite(property.depthScale) || property.depthScale <= 0)
            throw std::runtime_error("invalid native depthScale; refusing guessed units");
        *value = property.depthScale;
        return 0;
    } catch (const std::exception& e) {
        error = e.what();
    } catch (...) {
        error = "unknown exception reading depthScale";
    }
    return -1;
}

// Legacy paired read used by the old ctypes worker only. v5.1 production uses v5_read_frame()
// twice instead: the pairing matcher can only answer on its own timeout when nothing matches.
extern "C" int v5_read_pair(int timeout, void* depth_buffer, int depth_capacity,
                            void* rgb_buffer, int rgb_capacity, double* stamps, int* sizes,
                            int* widths, int* heights) {
    try {
        cs::IFramePtr depth, rgb;
        auto rc = camera->getPairedFrame(depth, rgb, timeout);
        if (rc == ::ERROR_FRAME_TIMEOUT) return 1;
        check(rc, "getPairedFrame");
        cs::IFramePtr frames[2] = {depth, rgb};
        void* buffers[2] = {depth_buffer, rgb_buffer};
        int capacities[2] = {depth_capacity, rgb_capacity};
        for (int i = 0; i < 2; ++i) {
            auto& f = frames[i];
            if (!f || f->empty() || !f->getData() || f->getWidth() <= 0 || f->getHeight() <= 0)
                throw std::runtime_error("invalid native paired frame");
            const auto expected = i == 0 ? ::STREAM_FORMAT_Z16 : ::STREAM_FORMAT_RGB8;
            const auto bytes = static_cast<std::uint64_t>(f->getWidth()) * f->getHeight() *
                               (i == 0 ? 2 : 3);
            if (f->getFormat() != expected || f->getSize() <= 0 ||
                bytes > static_cast<std::uint64_t>(capacities[i]) ||
                static_cast<std::uint64_t>(f->getSize()) != bytes)
                throw std::runtime_error("native frame format/size exceeds fixed buffer");
            stamps[i] = f->getTimeStamp();
            sizes[i] = f->getSize();
            widths[i] = f->getWidth();
            heights[i] = f->getHeight();
        }
        for (int i = 0; i < 2; ++i) std::memcpy(buffers[i], frames[i]->getData(), sizes[i]);
        return 0;
    } catch (const std::exception& e) {
        error = e.what();
    } catch (...) {
        error = "unknown exception copying native pair";
    }
    return -1;
}
