#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

// Global application settings — persisted to
// ~/.config/robocorea_gui/settings.json.
//
// Atomic members are safe to read directly from worker threads (thermal render,
// video workers). Non-atomic members (video_streams, vosk_grammar, ppm_calib)
// must be accessed under their respective mutex.
struct AppSettings {
    static AppSettings& instance() {
        static AppSettings s;
        return s;
    }

    void load();
    void save();

    // ── Video streaming ──────────────────────────────────────────────────────
    // The C920s on the Jetson are pushed to the workstation as H.264-over-SRT
    // (see scripts/c920_srt_stream.sh). Each VideoStream below becomes a
    // selectable "gst:" source whose receive pipeline is built by SourceManager.
    // The RF analog driving cameras are NOT listed here — they are local
    // /dev/videoN webcams discovered automatically as "local:N" sources.
    struct VideoStream {
        std::string name;        // shown in the source dropdown, e.g. "C920 Front"
        std::string host;        // Jetson IP; empty => use default_robot_host
        int         port{8890};  // SRT port (must match the robot-side streamer)
        int         latency_ms{120};  // SRT receive latency / ARQ budget
        bool        audio{false};     // true => stream also carries Opus audio
                                      //         (demuxed natively + fed to Vosk)
    };

    std::mutex               video_mutex;
    std::string              default_robot_host{"192.168.1.10"};  // TODO: set to your Jetson IP
    std::vector<VideoStream> video_streams{
        {"C920 Front (SRT A/V)", "", 8890, 120, true},   // video + C920 mic (Opus)
        {"C920 Rear (SRT)",      "", 8891, 120, false},  // video only
    };
    // Snapshot copy for worker/UI threads (avoids holding the mutex while building UI).
    std::vector<VideoStream> videoStreams();

    // ── Thermal camera (/sensors/thermal) ────────────────────────────────────
    // colormap: 14=Inferno, 2=Jet, 11=Hot, 15=Plasma, 16=Viridis (cv::ColormapTypes)
    std::atomic<int> thermal_colormap{14};
    // interp: 0=Nearest, 1=Linear, 2=Cubic, 4=Lanczos4 (cv::InterpolationFlags)
    std::atomic<int> thermal_interp{2};
    // 0×0 = auto-fit to widget
    std::atomic<int> thermal_upscale_w{0};
    std::atomic<int> thermal_upscale_h{0};

    // ── Detection labels (used by the in-GUI CV filters, ported later) ───────
    // actual scale = value / 100.0f
    std::atomic<int> label_font_scale_x100{80};

    // ── Audio / speech (SpeechProcessor) ─────────────────────────────────────
    // Whether the operator's speakers monitor the robot mic on startup.
    std::atomic<bool> audio_start_enabled{true};
    // Comma-separated Vosk vocabulary; empty = unrestricted recognition.
    std::mutex   strings_mutex;
    std::string  vosk_grammar;

    // (The per-channel keybind table was removed — the RC uses a fixed control
    // scheme hardcoded in the firmware. Only PPM calibration is configurable.)

    // ── RC PPM calibration (pushed as MSG_PPM_CALIB via /robot/ppm_calib) ─────
    // Per-channel min / neutral / max in raw µs (6 channels). The firmware maps
    // raw PPM into the calibrated range to produce a normalized [-1,1] stick.
    struct PpmChannelCalib { int min_us{1000}, neutral_us{1500}, max_us{2000}; };
    std::mutex      ppm_calib_mutex;
    PpmChannelCalib ppm_calib[6] = {};

    // Global RC stick deadband, normalized ×1000 (50 = 0.05). Sent as the 19th value
    // of /robot/ppm_calib; the firmware (RC stick neutral zone) and the esp32_bridge
    // (autonomy→teleop override threshold) both consume it. Atomic — no mutex needed.
    std::atomic<int> ppm_deadband_1000{50};

private:
    AppSettings() = default;
    AppSettings(const AppSettings&) = delete;
    AppSettings& operator=(const AppSettings&) = delete;

    std::string settingsPath() const;
};
