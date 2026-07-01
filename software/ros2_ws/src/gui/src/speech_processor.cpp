#include "gui/speech_processor.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <vosk_api.h>

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include <filesystem>

static const std::string& voskModelPath()
{
    static const std::string path =
        ament_index_cpp::get_package_share_directory("gui") + "/assets/audio";
    return path;
}

SpeechProcessor::SpeechProcessor(rclcpp::Node::SharedPtr node, QObject* parent)
    : QObject(parent), node_(node)
{
    vosk_set_log_level(-1);

    if (!loadModel()) {
        RCLCPP_WARN(node_->get_logger(),
                    "Vosk model not found at %s — speech transcription disabled. "
                    "See gui/assets/audio/README.md.",
                    voskModelPath().c_str());
        return;
    }

    vosk_recognizer_ = vosk_recognizer_new(vosk_model_, 16000.0f);
    if (!vosk_recognizer_)
        RCLCPP_ERROR(node_->get_logger(), "Failed to create Vosk recognizer");
    else
        RCLCPP_INFO(node_->get_logger(), "Speech processor ready");
}

SpeechProcessor::~SpeechProcessor()
{
    std::lock_guard<std::mutex> lock(recognizer_mutex_);
    if (vosk_recognizer_) vosk_recognizer_free(vosk_recognizer_);
    if (vosk_model_) vosk_model_free(vosk_model_);
}

bool SpeechProcessor::loadModel()
{
    if (std::filesystem::is_directory(voskModelPath())) {
        vosk_model_ = vosk_model_new(voskModelPath().c_str());
        if (vosk_model_) {
            RCLCPP_INFO(node_->get_logger(), "Loaded Vosk model: %s",
                        voskModelPath().c_str());
            return true;
        }
    }
    return false;
}

void SpeechProcessor::setGrammar(const std::string& words_csv)
{
    if (!vosk_model_) return;

    // "word1, word2" → JSON ["word1","word2","[unk]"]
    std::string grammar_json;
    if (!words_csv.empty()) {
        QJsonArray arr;
        for (const auto& w : QString::fromStdString(words_csv).split(',', Qt::SkipEmptyParts)) {
            QString t = w.trimmed().toLower();
            if (!t.isEmpty()) arr.append(t);
        }
        arr.append("[unk]");
        grammar_json = QJsonDocument(arr).toJson(QJsonDocument::Compact).toStdString();
    }

    std::lock_guard<std::mutex> lock(recognizer_mutex_);
    if (vosk_recognizer_) vosk_recognizer_free(vosk_recognizer_);
    vosk_recognizer_ = grammar_json.empty()
        ? vosk_recognizer_new(vosk_model_, 16000.0f)
        : vosk_recognizer_new_grm(vosk_model_, 16000.0f, grammar_json.c_str());

    RCLCPP_INFO(node_->get_logger(), "[Speech] Grammar %s",
                grammar_json.empty() ? "cleared (unrestricted)" : "updated");
}

void SpeechProcessor::clearTranscription()
{
    {
        std::lock_guard<std::mutex> lock(recognizer_mutex_);
        if (vosk_recognizer_) vosk_recognizer_reset(vosk_recognizer_);
    }
    {
        std::lock_guard<std::mutex> lock(text_mutex_);
        full_transcription_.clear();
    }
    emit transcriptionUpdated("");
}

void SpeechProcessor::pushAudio(const int16_t* pcm, size_t samples)
{
    if (!enabled_.load()) return;  // transcription toggled off (default at startup)
    if (!pcm || samples == 0) return;

    std::lock_guard<std::mutex> lock(recognizer_mutex_);
    if (!vosk_recognizer_) return;

    int result = vosk_recognizer_accept_waveform_s(
        vosk_recognizer_, pcm, static_cast<int>(samples));
    if (result <= 0) return;

    const char* json_str = vosk_recognizer_result(vosk_recognizer_);
    QString text = QJsonDocument::fromJson(QByteArray(json_str))
                       .object().value("text").toString().trimmed();
    if (text.isEmpty()) return;

    std::lock_guard<std::mutex> lock2(text_mutex_);
    if (!full_transcription_.isEmpty())
        full_transcription_ += "\n";
    full_transcription_ += text;
    emit transcriptionUpdated(full_transcription_);
}
