#include "gui/source_manager.hpp"
#include "gui/camera_hub.hpp"
#include "gui/app_settings.hpp"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include <opencv2/opencv.hpp>

#include <algorithm>

SourceManager::SourceManager(rclcpp::Node::SharedPtr node,
                             std::shared_ptr<CameraHub> hub,
                             QObject* parent)
    : QObject(parent), node_(node), camera_hub_(hub)
{
    // /config (transient-local) lets the robot advertise extra ROS image topics
    // (kept for compatibility — the C920 SRT streams come from AppSettings).
    auto qos = rclcpp::QoS(1).transient_local().reliable();
    config_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/config", qos,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            onConfigReceived(msg);
        });

    // /robot/camera_streams (latched JSON) is the robot's camera_streamer node
    // advertising its auto-detected SRT cameras (name + port). Late-joining GUIs
    // get the current list immediately; hot-plugged cameras arrive as updates.
    camera_streams_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/robot/camera_streams", qos,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            onCameraStreamsReceived(msg);
        });
}

SourceManager::~SourceManager()
{
    if (probe_thread_.joinable())
        probe_thread_.join();
}

std::string SourceManager::buildSrtRxPipeline(const std::string& host, int port,
                                              int latency_ms)
{
    // H.264-over-MPEG-TS pulled from the robot via SRT (caller mode: we connect
    // to the Jetson's listener). SRT's ARQ + latency budget smooth the lossy
    // wireless link; avdec_h264 is portable software decode (swap for a HW
    // decoder element later if the laptop has one). Ends in appsink for OpenCV.
    return "srtsrc uri=\"srt://" + host + ":" + std::to_string(port) +
           "?mode=caller&latency=" + std::to_string(latency_ms) + "\" ! "
           "queue ! tsdemux ! h264parse ! avdec_h264 ! videoconvert ! "
           "video/x-raw,format=BGR ! "
           "appsink drop=true max-buffers=1 sync=false";
}

void SourceManager::discoverSources()
{
    if (probing_.exchange(true))
        return;  // already probing

    if (probe_thread_.joinable())
        probe_thread_.join();

    probe_thread_ = std::thread([this]() {
        probeLocalCameras();
        buildNetworkStreams();
        probeThermalTopics();
        rebuildSourceList();
        probing_ = false;
        emit sourcesUpdated();
    });
}

QStringList SourceManager::sourceNames() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return source_names_;
}

QStringList SourceManager::sourceIdentifiers() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return source_ids_;
}

void SourceManager::probeLocalCameras()
{
    std::vector<int> found;

    // Cameras already streaming in the hub can't be re-opened for probing,
    // so include them directly.
    std::vector<std::string> active;
    if (camera_hub_)
        active = camera_hub_->activeSourceIds();

    for (int i = 0; i < 10; ++i) {
        std::string id = "local:" + std::to_string(i);
        if (std::find(active.begin(), active.end(), id) != active.end()) {
            found.push_back(i);
            continue;
        }
        cv::VideoCapture cap;
        if (cap.open(i, cv::CAP_V4L2)) {
            if (cap.isOpened()) {
                found.push_back(i);
                cap.release();
            }
        }
    }

    std::lock_guard<std::mutex> lock(local_mutex_);
    local_camera_ids_ = std::move(found);
}

void SourceManager::buildNetworkStreams()
{
    auto streams = AppSettings::instance().videoStreams();
    std::string default_host;
    {
        std::lock_guard<std::mutex> lk(AppSettings::instance().video_mutex);
        default_host = AppSettings::instance().default_robot_host;
    }

    std::vector<std::pair<std::string, std::string>> built;
    for (size_t i = 0; i < streams.size(); ++i) {
        const auto& s = streams[i];
        const std::string host = s.host.empty() ? default_host : s.host;
        if (host.empty())
            continue;  // nothing to connect to yet
        if (s.audio) {
            // A/V streams are owned by MainWindow's native GStreamer receiver and
            // registered with CameraHub as "av:<index>" (index into video_streams).
            built.emplace_back(s.name, "av:" + std::to_string(i));
        } else {
            built.emplace_back(s.name,
                               "gst:" + buildSrtRxPipeline(host, s.port, s.latency_ms));
        }
    }

    std::lock_guard<std::mutex> lock(net_mutex_);
    network_streams_ = std::move(built);
}

void SourceManager::probeThermalTopics()
{
    // Any sensor_msgs/Image topic whose name contains "thermal" is offered as a
    // thermal source even when /config is absent.
    auto topic_map = node_->get_topic_names_and_types();
    std::vector<std::string> found;
    for (const auto& [name, types] : topic_map) {
        for (const auto& type : types) {
            if (type == "sensor_msgs/msg/Image" &&
                name.find("thermal") != std::string::npos) {
                found.push_back(name);
            }
        }
    }

    if (!found.empty()) {
        std::lock_guard<std::mutex> lock(config_mutex_);
        for (const auto& t : found) {
            if (std::find(thermal_topics_.begin(), thermal_topics_.end(), t)
                    == thermal_topics_.end())
                thermal_topics_.push_back(t);
        }
    }
}

void SourceManager::onConfigReceived(const std_msgs::msg::String::SharedPtr msg)
{
    // { "camera_topics": ["/cam/image_raw", ...], "thermal_topics": [...] }
    QJsonDocument doc = QJsonDocument::fromJson(
        QByteArray::fromStdString(msg->data));

    std::vector<std::string> cam_topics, therm_topics;
    if (doc.isObject()) {
        for (const auto& val : doc.object().value("camera_topics").toArray())
            if (val.isString())
                cam_topics.push_back(val.toString().toStdString());
        for (const auto& val : doc.object().value("thermal_topics").toArray())
            if (val.isString())
                therm_topics.push_back(val.toString().toStdString());
    }

    {
        std::lock_guard<std::mutex> lock(config_mutex_);
        config_topics_ = std::move(cam_topics);
        thermal_topics_ = std::move(therm_topics);
    }

    rebuildSourceList();
    emit sourcesUpdated();
}

void SourceManager::onCameraStreamsReceived(const std_msgs::msg::String::SharedPtr msg)
{
    // { "cameras": [ {"name": "...", "port": 8900}, ... ] } — video-only SRT
    // streams the robot auto-detected. The A/V primary (C920) is intentionally
    // NOT here; it stays a static A/V source (AppSettings + GstAvStream).
    QJsonDocument doc = QJsonDocument::fromJson(QByteArray::fromStdString(msg->data));

    std::vector<std::pair<std::string, int>> cams;
    if (doc.isObject()) {
        for (const auto& v : doc.object().value("cameras").toArray()) {
            if (!v.isObject()) continue;
            QJsonObject c = v.toObject();
            const int port = c.value("port").toInt(0);
            const std::string name = c.value("name").toString().toStdString();
            if (port > 0 && !name.empty())
                cams.emplace_back(name, port);
        }
    }

    {
        std::lock_guard<std::mutex> lock(dyn_mutex_);
        dynamic_cameras_ = std::move(cams);
    }
    rebuildSourceList();
    emit sourcesUpdated();
}

void SourceManager::rebuildSourceList()
{
    QStringList names, ids;

    names << "None";
    ids << "";

    // RF driving cams + USB webcams (local V4L2 devices).
    {
        std::lock_guard<std::mutex> lock(local_mutex_);
        for (int id : local_camera_ids_) {
            names << QString("Camera %1").arg(id);
            ids << QString("local:%1").arg(id);
        }
    }

    // C920 SRT streams from the Jetson (static AppSettings config).
    {
        std::lock_guard<std::mutex> lock(net_mutex_);
        for (const auto& [name, id] : network_streams_) {
            names << QString::fromStdString(name);
            ids << QString::fromStdString(id);
        }
    }

    // Robot-advertised, auto-detected SRT cameras (camera_streamer node). Built
    // into "gst:" receive pipelines with the configured Jetson host + latency.
    {
        std::string host;
        {
            std::lock_guard<std::mutex> lk(AppSettings::instance().video_mutex);
            host = AppSettings::instance().default_robot_host;
        }
        std::lock_guard<std::mutex> lock(dyn_mutex_);
        for (const auto& [name, port] : dynamic_cameras_) {
            if (host.empty()) break;   // nothing to connect to yet
            names << QString::fromStdString(name);
            ids << QString::fromStdString(
                "gst:" + buildSrtRxPipeline(host, port, /*latency_ms=*/120));
        }
    }

    // Generic ROS image topics + thermal.
    {
        std::lock_guard<std::mutex> lock(config_mutex_);
        for (const auto& topic : config_topics_) {
            QString qtopic = QString::fromStdString(topic);
            names << qtopic;
            ids << QString("topic:") + qtopic;
        }
        for (const auto& topic : thermal_topics_) {
            QString qtopic = QString::fromStdString(topic);
            names << "Thermal: " + qtopic;
            ids << "thermal:" + qtopic;
        }
    }

    std::lock_guard<std::mutex> lock(mutex_);
    source_names_ = std::move(names);
    source_ids_ = std::move(ids);
}
