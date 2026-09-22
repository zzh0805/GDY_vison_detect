// Reuse the exact tested open/pause/software-trigger/paired acquisition lifecycle.
#include "../../tool/soft_trigger_test/bridge.cpp"
#include <cstring>
#include <cmath>

extern "C" int v5_profile_sizes(int* widths, int* heights) {
    try {
        for (int i=0; i<2; ++i) {
            std::vector<::StreamInfo> infos;
            check(camera->getStreamInfos(i==0 ? ::STREAM_TYPE_DEPTH : ::STREAM_TYPE_RGB, infos), "stream info");
            bool found=false;
            for (const auto& info: infos) if (info.format==(i==0 ? ::STREAM_FORMAT_Z16 : ::STREAM_FORMAT_RGB8)) {
                widths[i]=info.width; heights[i]=info.height; found=true; break;
            }
            if (!found) throw std::runtime_error("selected profile missing");
        }
        return 0;
    } catch (const std::exception& e) { error=e.what(); }
      catch (...) { error="unknown profile exception"; }
    return -1;
}

extern "C" int v5_depth_scale(double* value) {
    try {
        ::PropertyExtension property{};
        check(camera->getPropertyExtension(::PROPERTY_EXT_DEPTH_SCALE, property), "read depthScale");
        if (!std::isfinite(property.depthScale) || property.depthScale <= 0)
            throw std::runtime_error("invalid native depthScale; refusing guessed units");
        *value=property.depthScale;
        return 0;
    } catch (const std::exception& e) { error=e.what(); }
      catch (...) { error="unknown exception reading depthScale"; }
    return -1;
}

// Raw RGB8 and Z16 are copied into bounded shared buffers BEFORE SDK frames release.
// No frame pointers or vendor C++ structures cross the process/ctypes boundary.
extern "C" int v5_read_pair(int timeout, void* depth_buffer, int depth_capacity,
                           void* rgb_buffer, int rgb_capacity, double* stamps,
                           int* sizes, int* widths, int* heights) {
    try {
        cs::IFramePtr depth, rgb;
        auto rc=camera->getPairedFrame(depth, rgb, timeout);
        if (rc==::ERROR_FRAME_TIMEOUT) return 1;
        check(rc, "getPairedFrame");
        cs::IFramePtr frames[2]={depth, rgb};
        void* buffers[2]={depth_buffer, rgb_buffer};
        int capacities[2]={depth_capacity, rgb_capacity};
        for (int i=0; i<2; ++i) {
            auto& f=frames[i];
            if (!f || f->empty() || !f->getData() || f->getWidth()<=0 || f->getHeight()<=0)
                throw std::runtime_error("invalid native paired frame");
            const auto expected=i==0 ? ::STREAM_FORMAT_Z16 : ::STREAM_FORMAT_RGB8;
            const auto bytes=static_cast<long long>(f->getWidth())*f->getHeight()*(i==0 ? 2 : 3);
            if (f->getFormat()!=expected || f->getSize()!=bytes || bytes>capacities[i])
                throw std::runtime_error("native frame format/size exceeds fixed buffer");
            stamps[i]=f->getTimeStamp(); sizes[i]=f->getSize();
            widths[i]=f->getWidth(); heights[i]=f->getHeight();
        }
        for (int i=0; i<2; ++i) std::memcpy(buffers[i], frames[i]->getData(), sizes[i]);
        return 0;
    } catch (const std::exception& e) { error=e.what(); }
      catch (...) { error="unknown exception copying native pair"; }
    return -1;
}
