#include "gui/camera_hub.hpp"

#include <chrono>

CameraHub::~CameraHub()
{
    std::lock_guard<std::mutex> lock(streams_mutex_);
    for (auto& [id, stream] : streams_)
        stream->running = false;
    // captureLoop never locks streams_mutex_, so joining under lock is safe.
    for (auto& [id, stream] : streams_) {
        if (stream->thread.joinable())
            stream->thread.join();
    }
}

void CameraHub::registerFrameSource(const std::string& source_id,
                                    std::function<cv::Mat()> frame_getter,
                                    std::function<bool()> connected_getter)
{
    std::lock_guard<std::mutex> lock(streams_mutex_);
    external_[source_id] = {std::move(frame_getter), std::move(connected_getter)};
}

void CameraHub::unregisterFrameSource(const std::string& source_id)
{
    std::lock_guard<std::mutex> lock(streams_mutex_);
    external_.erase(source_id);
}

void CameraHub::subscribe(const std::string& source_id)
{
    std::lock_guard<std::mutex> lock(streams_mutex_);
    if (external_.count(source_id))
        return;  // external sources have no hub-owned capture thread
    auto it = streams_.find(source_id);
    if (it != streams_.end()) {
        it->second->ref_count++;
        return;
    }

    auto stream = std::make_unique<Stream>();
    stream->ref_count = 1;
    auto* ptr = stream.get();
    streams_[source_id] = std::move(stream);
    streams_[source_id]->thread =
        std::thread(&CameraHub::captureLoop, this, source_id, ptr);
}

void CameraHub::unsubscribe(const std::string& source_id)
{
    std::unique_ptr<Stream> to_destroy;
    {
        std::lock_guard<std::mutex> lock(streams_mutex_);
        if (external_.count(source_id))
            return;
        auto it = streams_.find(source_id);
        if (it == streams_.end()) return;

        it->second->ref_count--;
        if (it->second->ref_count <= 0) {
            it->second->running = false;
            to_destroy = std::move(it->second);
            streams_.erase(it);
        }
    }
    // Join outside the lock.
    if (to_destroy && to_destroy->thread.joinable())
        to_destroy->thread.join();
}

cv::Mat CameraHub::getLatestFrame(const std::string& source_id)
{
    std::function<cv::Mat()> getter;
    {
        std::lock_guard<std::mutex> lock(streams_mutex_);
        auto ext = external_.find(source_id);
        if (ext != external_.end()) {
            getter = ext->second.frame_getter;
        } else {
            auto it = streams_.find(source_id);
            if (it == streams_.end()) return cv::Mat();
            std::lock_guard<std::mutex> flock(it->second->frame_mutex);
            if (it->second->latest_frame.empty()) return cv::Mat();
            return it->second->latest_frame.clone();
        }
    }
    return getter ? getter() : cv::Mat();  // call external getter outside the lock
}

bool CameraHub::isConnected(const std::string& source_id) const
{
    std::function<bool()> getter;
    {
        std::lock_guard<std::mutex> lock(streams_mutex_);
        auto ext = external_.find(source_id);
        if (ext != external_.end()) {
            getter = ext->second.connected_getter;
        } else {
            auto it = streams_.find(source_id);
            return it != streams_.end() && it->second->connected.load();
        }
    }
    return getter ? getter() : false;
}

std::vector<std::string> CameraHub::activeSourceIds() const
{
    std::lock_guard<std::mutex> lock(streams_mutex_);
    std::vector<std::string> ids;
    ids.reserve(streams_.size());
    for (const auto& [id, _] : streams_)
        ids.push_back(id);
    return ids;
}

bool CameraHub::openSource(cv::VideoCapture& cap, const std::string& source_id)
{
    if (source_id.rfind("local:", 0) == 0) {
        int index = std::stoi(source_id.substr(6));
        if (!cap.open(index, cv::CAP_V4L2))
            return false;
        // Request MJPG (compressed) for USB capture devices. Several analog
        // grabbers on one shared USB 2.0 bus (a hub) cannot all reserve the
        // isochronous bandwidth uncompressed YUYV needs, so only one opens at a
        // time; MJPG shrinks the reservation ~10x so they stream in parallel.
        // 640x480 is supported by the RF driving-cam digitizers and is plenty
        // for a low-latency driving feed. Cameras without MJPG just ignore this.
        cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        return true;
    }
    if (source_id.rfind("gst:", 0) == 0) {
        std::string pipeline = source_id.substr(4);
        return cap.open(pipeline, cv::CAP_GSTREAMER);
    }
    // Unknown scheme.
    return false;
}

void CameraHub::captureLoop(const std::string& source_id, Stream* stream)
{
    using namespace std::chrono_literals;

    while (stream->running) {
        // (Re)open the source if it isn't open. For SRT this blocks inside
        // GStreamer until the robot connects, so a not-yet-streaming robot just
        // means this widget sits on "Connecting…" rather than erroring out.
        if (!stream->capture.isOpened()) {
            stream->connected = false;
            if (!openSource(stream->capture, source_id)) {
                std::this_thread::sleep_for(500ms);  // backoff before retrying
                continue;
            }
        }

        cv::Mat frame;
        if (!stream->capture.read(frame) || frame.empty()) {
            // Read failed → the device/stream dropped. Release and let the loop
            // reopen it (the robot may have restarted its streamer, etc.).
            stream->connected = false;
            stream->capture.release();
            std::this_thread::sleep_for(200ms);
            continue;
        }

        stream->connected = true;
        {
            std::lock_guard<std::mutex> lock(stream->frame_mutex);
            stream->latest_frame = std::move(frame);
        }
    }

    stream->capture.release();
}
