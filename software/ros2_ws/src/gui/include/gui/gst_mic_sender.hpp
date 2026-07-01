#pragma once

#include <gst/gst.h>

#include <atomic>
#include <string>
#include <thread>

// Native-GStreamer SENDER for the operator→robot talkback path — the reverse of
// GstAvStream. Captures the workstation microphone, Opus-encodes it, muxes it
// into MPEG-TS and pushes it over SRT (as the SRT *caller*, mirroring how the
// GUI calls the robot's video listener) to the robot, where a listener pipeline
// decodes it and plays it on a Jetson-attached speaker
// (see scripts/jetson_speaker_sink.sh).
//
//   [mic] ! audioconvert ! audioresample ! opusenc ! mpegtsmux ! srtsink(caller)
//
// Transmit is gated entirely by start()/stop(): the mic is only opened while
// talkback is ON, so there is no capture (and no echo path) when muted. Like
// GstAvStream, a monitor thread relaunches the pipeline on error/EOS (e.g. the
// robot-side listener isn't up yet), so toggling talkback ON before the robot is
// ready still connects once it appears.
//
// The Jetson speaker hardware (Bluetooth vs USB) is not finalized; that choice
// lives entirely on the robot side (the sink element in jetson_speaker_sink.sh),
// so nothing here needs to change when it's decided.
class GstMicSender {
public:
    // host/port/latency_ms: where the robot's talkback listener is reachable.
    // mic_device: ALSA device string (e.g. "hw:CARD=..."); empty => system
    //   default input via autoaudiosrc.
    // opus_kbps: voice-grade Opus bitrate (24 kbps mono is clean speech).
    GstMicSender(std::string host, int port, int latency_ms,
                 std::string mic_device, int opus_kbps);
    ~GstMicSender();

    GstMicSender(const GstMicSender&) = delete;
    GstMicSender& operator=(const GstMicSender&) = delete;

    void start();   // open the mic + begin transmitting (idempotent)
    void stop();    // stop transmitting + close the mic (idempotent)
    bool transmitting() const { return running_.load(); }

private:
    std::string buildPipeline() const;
    bool launch();
    void teardown();
    void monitorLoop();   // watches the bus, relaunches on error/EOS

    const std::string host_;
    const int         port_;
    const int         latency_ms_;
    const std::string mic_device_;   // empty => autoaudiosrc default
    const int         opus_kbps_;

    GstElement*       pipeline_{nullptr};
    std::atomic<bool> running_{false};
    std::thread       monitor_thread_;
};
