#include "gui/gst_mic_sender.hpp"

#include <chrono>
#include <utility>

GstMicSender::GstMicSender(std::string host, int port, int latency_ms,
                           std::string mic_device, int opus_kbps)
    : host_(std::move(host)), port_(port), latency_ms_(latency_ms),
      mic_device_(std::move(mic_device)), opus_kbps_(opus_kbps)
{
}

GstMicSender::~GstMicSender()
{
    stop();
}

std::string GstMicSender::buildPipeline() const
{
    // Source: a specific ALSA device if configured, else autodetect the system
    // default input. 48 kHz mono is what opusenc wants for voice.
    const std::string src =
        mic_device_.empty()
            ? "autoaudiosrc"
            : ("alsasrc device=\"" + mic_device_ + "\"");

    const int opus_bps = opus_kbps_ * 1000;

    // Opus in MPEG-TS over SRT, the exact reverse of the robot's A/V stream so
    // the Jetson side can reuse the same tsdemux ! opusparse ! opusdec chain.
    // srtsink is the caller; the robot runs the listener. wait-for-connection
    // =false lets us launch before the robot listener exists (monitorLoop
    // relaunches on the resulting error until it connects).
    return
        src + " ! queue leaky=downstream ! audioconvert ! audioresample ! "
        "audio/x-raw,rate=48000,channels=1 ! "
        "opusenc bitrate=" + std::to_string(opus_bps) + " audio-type=voice ! "
        "mpegtsmux alignment=7 ! "
        "srtsink uri=\"srt://" + host_ + ":" + std::to_string(port_) +
        "?mode=caller&latency=" + std::to_string(latency_ms_) +
        "\" wait-for-connection=false";
}

bool GstMicSender::launch()
{
    GError* err = nullptr;
    pipeline_ = gst_parse_launch(buildPipeline().c_str(), &err);
    if (!pipeline_) {
        if (err) g_error_free(err);
        return false;
    }
    if (err) g_error_free(err);   // non-fatal warnings

    gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    return true;
}

void GstMicSender::teardown()
{
    if (pipeline_) {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
    }
}

void GstMicSender::start()
{
    if (running_.exchange(true))
        return;
    monitor_thread_ = std::thread(&GstMicSender::monitorLoop, this);
}

void GstMicSender::stop()
{
    if (!running_.exchange(false))
        return;
    if (monitor_thread_.joinable())
        monitor_thread_.join();
}

void GstMicSender::monitorLoop()
{
    while (running_) {
        if (!pipeline_ && !launch()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }

        GstBus* bus = gst_element_get_bus(pipeline_);
        GstMessage* msg = gst_bus_timed_pop_filtered(
            bus, 200 * GST_MSECOND,
            static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
        gst_object_unref(bus);

        if (msg) {
            // Error or EOS (robot listener not up yet, link dropped, mic
            // unplugged, …) → tear down and let the loop relaunch.
            gst_message_unref(msg);
            teardown();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
    teardown();
}
