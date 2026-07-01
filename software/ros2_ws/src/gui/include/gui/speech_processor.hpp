#pragma once

#include <QObject>
#include <QString>

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>

struct VoskModel;
struct VoskRecognizer;

// Speech-to-text over the robot's audio using Vosk. Audio no longer arrives on a
// ROS topic — it is the Opus track demuxed from the C920 A/V SRT stream, so the
// GStreamer receiver calls pushAudio() with decoded 16 kHz mono PCM (from a
// GStreamer streaming thread; Vosk runs there, results are emitted to the GUI
// thread via a queued signal). Playback is handled by the GStreamer pipeline,
// not here.
class SpeechProcessor : public QObject {
    Q_OBJECT
public:
    explicit SpeechProcessor(rclcpp::Node::SharedPtr node, QObject* parent = nullptr);
    ~SpeechProcessor() override;

    // Called from the GStreamer audio thread.
    void pushAudio(const int16_t* pcm, size_t samples);

    // When disabled, pushAudio() is a no-op so no transcription is produced.
    // Follows the dashboard audio/transcription toggle (off at startup).
    void setEnabled(bool enabled) { enabled_ = enabled; }

    void clearTranscription();
    void setGrammar(const std::string& words_csv);
    bool isModelLoaded() const { return vosk_model_ != nullptr; }

signals:
    void transcriptionUpdated(const QString& text);

private:
    bool loadModel();

    rclcpp::Node::SharedPtr node_;
    VoskModel* vosk_model_{nullptr};
    VoskRecognizer* vosk_recognizer_{nullptr};

    std::atomic<bool> enabled_{false};
    std::mutex recognizer_mutex_;
    std::mutex text_mutex_;
    QString full_transcription_;
};
