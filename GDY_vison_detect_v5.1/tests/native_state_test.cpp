// Offline state test for the v5.1.1 camera worker. No camera, no SDK, no robot, no network.
//
//   g++ -std=c++14 -Itests/native_sdk_fake tests/native_state_test.cpp -o /tmp/gdy_state_test
//   /tmp/gdy_state_test
//
// It proves the properties the field acceptance run depends on:
//   * standby really is suspended (the device emits nothing, queues cannot grow while idle)
//   * a shot is clear -> drain -> one trigger -> two getFrame, and never starts from a dirty queue
//   * streaming is entered/left with pause/resume only: no startStream/stopStream ever repeats
//   * every SDK frame is released on every path, including faults (frames_live back to zero)
//   * the original device settings survive a killed worker via the property backup file
#define GDY_NATIVE_STATE_TEST
#include "../handeye_calib/native_camera/worker.cpp"
#include <cassert>
#include <cstdio>
#include <string>

static const char* BACKUP = "gdy_state_test_backup.txt";

static int failures = 0;
static void check(bool ok, const std::string& what) {
    if (!ok) {
        ++failures;
        std::printf("FAIL: %s\n", what.c_str());
    }
}

template <class F>
static void expect_failure(F fn, const std::string& what) {
    bool failed = false;
    try {
        fn();
    } catch (const std::exception& e) {
        failed = true;
        std::printf("  expected failure: %s -> %s\n", what.c_str(), e.what());
    }
    check(failed, what + " must fail closed");
    check(fake::frames_live == 0, what + " must not leak SDK frames");
}

// Baseline settings a field configuration would use; no writes unless a test asks for one.
static void base_settings(CameraOwner& owner) {
    owner.cfg.capture_ms = 3000;
    owner.cfg.preview_ms = 100;
    owner.cfg.quiet = 3;
    owner.cfg.drain = 60;
    owner.cfg.rgb_read_ms = 500;
    owner.cfg.drain_ms = 100;
    owner.cfg.drain_frames = 10;
    owner.cfg.periodic_clear_s = 0;
    owner.cfg.memory_report_s = 0;
}

int main() {
    std::remove(BACKUP);

    // ---------------------------------------------------------------- standby, shots, no rebuild
    {
        fake::reset();
        CameraOwner owner;
        base_settings(owner);
        owner.open("test");
        check(fake::started[0] && fake::started[1], "both streams must be started after open");
        check(fake::paused[0] && fake::paused[1], "both streams must be suspended in standby");
        check(fake::device.triggerMode == TRIGGER_MODE_SOFTWAER, "standby must be in software trigger mode");
        check(fake::stream_starts == 2, "open must start exactly two streams (one call each)");
        check(fake::stream_stops == 0, "open must not stop any stream");
        check(!owner.streaming, "open must leave the worker in standby");
        check(fake::queue[0].empty() && fake::queue[1].empty(), "standby queues must be empty after open");

        // A long idle period must not make the device produce anything.
        const int idle_depth = fake::produced[0], idle_rgb = fake::produced[1];
        for (int i = 0; i < 200; ++i) {
            owner.pump();  // Main loop turn: suspended streams answer with their timeout.
            owner.tick();
        }
        check(fake::produced[0] == idle_depth && fake::produced[1] == idle_rgb,
              "standby must not produce frames");
        check(fake::frames_live == 0, "idle polling must not retain frames");

        Mapping memory;
        for (int n = 0; n < 100; ++n) {
            const int triggers_before = fake::triggers;
            const std::string reply = owner.capture(memory);
            check(fake::triggers == triggers_before + 1, "one shot must trigger exactly once");
            check(reply.find("\"stream_restarts\":0") != std::string::npos,
                  "a shot must report zero stream restarts");
            check(owner.latest.empty(), "standby shots must not publish preview frames");
            check(fake::frames_live == 0, "a shot must release both SDK frames");
        }
        check(fake::triggered_with_backlog == 0, "no shot may start from a non-empty queue");
        check(fake::stream_starts == 2 && fake::stream_stops == 0,
              "100 shots must not restart or stop any stream");
        check(fake::produced[0] == 100, "depth must come from the 100 triggers only");
        check(fake::frames_peak <= 2, "at most one depth and one RGB frame may be alive at once");

        // ------------------------------------------------------------ streaming: pause/resume only
        owner.set_streaming(true);
        check(owner.streaming, "set_streaming(true) must enter streaming");
        check(fake::paused[0] && !fake::paused[1], "streaming must resume RGB and keep depth suspended");
        check(fake::device.triggerMode == TRIGGER_MODE_OFF, "streaming must leave software trigger mode");
        const int rgb_before = fake::produced[1];
        for (int i = 0; i < 50; ++i) {
            owner.pump();
            owner.tick();
            check(fake::frames_live == 0, "each pumped frame must be released");
        }
        check(fake::produced[1] == rgb_before + 50, "streaming must consume one RGB frame per pump");
        check(owner.latest.size() == 12, "the single retained preview frame is 2x2 RGB8");
        check(owner.preview_frame(memory).find("\"available\":true") != std::string::npos,
              "preview_frame must publish the retained frame");
        check(fake::stream_starts == 2 && fake::stream_stops == 0,
              "streaming must not restart or stop a stream");

        // A shot requested while streaming must suspend, shoot and resume - still no rebuild.
        const std::string streaming_shot = owner.capture(memory);
        check(owner.streaming, "a shot taken during streaming must restore streaming");
        check(streaming_shot.find("\"stream_restarts\":0") != std::string::npos,
              "a shot during streaming must still not restart streams");
        check(fake::stream_starts == 2 && fake::stream_stops == 0,
              "a shot during streaming must not restart or stop a stream");
        check(fake::frames_live == 0, "a shot during streaming must release its frames");

        owner.set_streaming(false);
        check(!owner.streaming, "set_streaming(false) must return to standby");
        check(fake::paused[0] && fake::paused[1], "standby must suspend both streams again");
        check(fake::device.triggerMode == TRIGGER_MODE_SOFTWAER, "standby trigger mode must be restored");
        check(fake::queue[1].empty(), "leaving streaming must not leave RGB residue");
        check(owner.latest.empty(), "leaving streaming must drop the preview frame");
        check(fake::stream_starts == 2 && fake::stream_stops == 0, "streaming cycles must not rebuild");

        // Idempotent switches must not disturb the device.
        const int starts = fake::stream_starts;
        owner.set_streaming(true);
        owner.set_streaming(true);
        owner.set_streaming(false);
        owner.set_streaming(false);
        check(fake::stream_starts == starts, "repeated identical switches must be no-ops");

        // -------------------------------------------------- a dirty queue must never reach a shot
        fake::inject_backlog(STREAM_TYPE_DEPTH, 3);  // e.g. an SDK residue nobody consumed
        fake::inject_backlog(STREAM_TYPE_RGB, 3);
        const std::string drained_shot = owner.capture(memory);
        check(fake::triggered_with_backlog == 0, "clear+drain must empty the queue before the trigger");
        check(drained_shot.find("\"stream_restarts\":0") != std::string::npos, "shot stays rebuild-free");
        check(fake::frames_live == 0, "drain must release every discarded frame");

        // ---------------------------------------------------------------- faults stay fail-closed
        fake::fail_clear = true;
        const int triggers_before = fake::triggers;
        expect_failure([&] { owner.capture(memory); }, "clear failure");
        check(fake::triggers == triggers_before, "a failed clear must not trigger");
        fake::fail_clear = false;

        fake::bad_rgb = true;
        expect_failure([&] { owner.capture(memory); }, "wrong RGB frame format");
        fake::bad_rgb = false;

        fake::timeout = true;
        expect_failure([&] { owner.capture(memory); }, "SDK read timeout");
        fake::timeout = false;

        fake::zero_stamp = true;
        owner.cfg.strict = 1;
        expect_failure([&] { owner.capture(memory); }, "unusable timestamps with timestamp_policy=strict");
        owner.cfg.strict = 0;
        check(owner.capture(memory).find("\"timestamp_valid\":false") != std::string::npos,
              "warn policy must still return the frame and report unusable timestamps");
        fake::zero_stamp = false;
        check(fake::frames_live == 0, "fault paths must release every frame");

        // A stream that refuses to suspend must be detected instead of silently serving stale
        // frames: the shot asks streaming to stop first, so a failing pauseStream is fatal.
        owner.set_streaming(true);
        fake::fail_pause = true;
        expect_failure([&] { owner.capture(memory); }, "suspend failure");
        fake::fail_pause = false;
        owner.set_streaming(false);  // Recover the emulated device state for the next checks.
        check(fake::paused[0] && fake::paused[1], "standby must be re-established after the fault");

        // -------------------------------------------------- stream properties (while stream runs)
        owner.cfg.rgb_exp = 10000;
        owner.cfg.rgb_gain = 1;
        owner.cfg.depth_exp = 7000;
        owner.cfg.depth_period = 10000;
        owner.cfg.depth_gain = 1;
        owner.cfg.match = 1;
        owner.save_settings();
        owner.settings();
        check(fake::device.rgbExposure == 10000, "RGB exposure must be applied and read back");
        check(fake::device.depthExposure == 7000 && fake::device.depthFrametime == 10000,
              "depth frame time must be written before exposure");
        check(fake::device.matchDifThreshold == 60, "depth/RGB match parameter must be applied");
        fake::fail_gain = true;
        expect_failure([&] { owner.settings(); }, "property write failure");
        fake::fail_gain = false;
        owner.save_settings();  // Re-register after the deliberate fault.

        // ------------------------------------------------------------- clean shutdown accounting
        check(owner.close(), "close must succeed");
        check(fake::stream_stops == 1, "close must stop the streams exactly once");
        check(!camera && !system_ptr, "close must release the camera and system objects");
        check(fake::device.triggerMode == TRIGGER_MODE_OFF,
              "close must restore the trigger mode it found (fake device default)");

        // -------------------------------------------------- a device id without a terminator
        fake::reset();
        fake::nonterminated_id = true;
        CameraOwner bounded;
        base_settings(bounded);
        bounded.open("0123456789ABCDEFGHIJKLMNOPQRSTUV");  // Full 32 bytes, no NUL inside.
        check(fake::started[0], "a 32-byte unterminated uniqueId must still match");
        check(bounded.close(), "close after matching an unterminated id");
        fake::nonterminated_id = false;
    }
    check(fake::frames_live == 0 && !camera && !system_ptr, "everything must be released at scope exit");

    // ------------------------------------------------- original settings survive a killed worker
    {
        fake::reset();
        {
            CameraOwner first(BACKUP);
            base_settings(first);
            first.cfg.rgb_exp = 10000;  // Force exactly one property write.
            first.open("test");
            check(fake::device.rgbExposure == 10000, "the configured exposure must be on the device");
            check(first.close(), "first close");
            check(fake::device.rgbExposure == 5000, "close must restore the original exposure");
        }
        // Simulate a worker that was killed before it could restore anything.
        fake::device.rgbExposure = 10000;
        {
            CameraOwner second(BACKUP);
            base_settings(second);
            second.cfg.rgb_exp = 10000;
            second.open("test");
            check(second.close(), "second close");
            check(fake::device.rgbExposure == 5000,
                  "the persisted original value must win over the modified device value");
        }
        std::remove(BACKUP);
        std::remove((std::string(BACKUP) + ".tmp").c_str());
    }

    // ---------------------------------------- private wire format vs native_settings.py
    // The same literal string is asserted in tests/test_preview_v51.py. Both numbers and their
    // order are positional: changing one side without the other breaks the camera worker.
    {
        Settings parsed;
        std::istringstream line("3000 100 3 60 -1 -1 -1 -1 -1 0 60 0 0 0 0 500 100 10 60 300");
        bool ok = true;
        try {
            parsed.parse(line);
        } catch (const std::exception& e) {
            ok = false;
            std::printf("  wire parse: %s\n", e.what());
        }
        check(ok, "the default wire line emitted by native_settings.py must parse");
        check(parsed.capture_ms == 3000 && parsed.preview_ms == 100 && parsed.quiet == 3 &&
                  parsed.drain == 60,
              "wire fields 1..4 = capture_ms preview_ms quiet drain");
        check(parsed.rgb_exp == -1 && parsed.rgb_gain == -1 && parsed.depth_exp == -1 &&
                  parsed.depth_period == -1 && parsed.depth_gain == -1,
              "wire fields 5..9 = rgb_exposure rgb_gain depth_exposure depth_frame_time depth_gain");
        check(parsed.match == 0 && parsed.threshold == 60 && parsed.offset == 0 && parsed.after == 0 &&
                  parsed.strict == 0 && parsed.preview_depth == 0,
              "wire fields 10..15 = match_enabled threshold offset after strict preview_with_depth");
        check(parsed.rgb_read_ms == 500 && parsed.drain_ms == 100 && parsed.drain_frames == 10 &&
                  parsed.periodic_clear_s == 60 && parsed.memory_report_s == 300,
              "wire fields 16..20 = rgb_read drain_timeout drain_frames periodic_clear memory_report");
        Settings missing;
        std::istringstream short_line("3000 100 3 60 -1 -1 -1 -1 -1 0 60 0 0 0 0 500 100 10 60");
        expect_failure([&] { missing.parse(short_line); }, "a short wire line");
        Settings unreadable;
        std::istringstream endless_line(
            "3000 100 3 60 -1 -1 -1 -1 -1 0 60 0 0 0 0 500 100 10 60 300 7 7 7 7 7 7");
        check((unreadable.parse(endless_line), unreadable.capture_ms == 3000),
              "extra trailing fields must be ignored, not misread");
        Settings bad;
        std::istringstream out_of_range("3000 100 3 60 -1 -1 -1 -1 -1 0 60 0 0 0 0 500 100 10 60 99999");
        expect_failure([&] { bad.parse(out_of_range); }, "an out-of-range wire value");
    }

    if (failures == 0)
        std::printf("PASS: standby is suspended; 100 rebuild-free shots; streaming via pause/resume; "
                    "dirty queues drained; faults fail closed; RAII; property backup\n");
    else
        std::printf("FAILED: %d checks\n", failures);
    return failures == 0 ? 0 : 1;
}
