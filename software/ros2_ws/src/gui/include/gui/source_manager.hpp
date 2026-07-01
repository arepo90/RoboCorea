#pragma once

#include <QObject>
#include <QString>
#include <QStringList>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

class CameraHub;

// Discovers the video sources the GUI can show and exposes them as parallel
// (display-name, identifier) lists for the source dropdowns. Identifiers use the
// scheme understood by CameraHub / VideoWidget:
//   ""              → None
//   "local:N"       → V4L2 /dev/videoN (RF driving cams, USB webcams)
//   "gst:<pipe>"    → GStreamer pipeline (C920 SRT + robot-advertised SRT cams)
//   "topic:/foo"    → generic sensor_msgs/Image topic
//   "thermal:/foo"  → thermal sensor_msgs/Image topic (colormapped on display)
class SourceManager : public QObject {
    Q_OBJECT
public:
    explicit SourceManager(rclcpp::Node::SharedPtr node,
                           std::shared_ptr<CameraHub> hub,
                           QObject* parent = nullptr);
    ~SourceManager() override;

    void discoverSources();

    QStringList sourceNames() const;
    QStringList sourceIdentifiers() const;

    // GUI-side SRT receive pipeline for one C920 stream. Public so the settings
    // dialog (later) can preview it; ends in `appsink` for OpenCV.
    static std::string buildSrtRxPipeline(const std::string& host, int port,
                                          int latency_ms);

signals:
    void sourcesUpdated();

private:
    void probeLocalCameras();
    void probeThermalTopics();   // ROS introspection fallback for thermal sources
    void buildNetworkStreams();  // from AppSettings::video_streams
    void onConfigReceived(const std_msgs::msg::String::SharedPtr msg);
    void onCameraStreamsReceived(const std_msgs::msg::String::SharedPtr msg);
    void rebuildSourceList();

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<CameraHub> camera_hub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr config_sub_;
    // Robot-advertised, auto-detected SRT cameras (camera_streamer node).
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr camera_streams_sub_;

    mutable std::mutex mutex_;
    QStringList source_names_;
    QStringList source_ids_;

    std::mutex local_mutex_;
    std::vector<int> local_camera_ids_;

    std::mutex net_mutex_;
    std::vector<std::pair<std::string, std::string>> network_streams_;  // (name, id)

    // Dynamic SRT cameras advertised by the robot's camera_streamer node
    // (name, port). Turned into "gst:" sources at rebuild using default_robot_host.
    std::mutex dyn_mutex_;
    std::vector<std::pair<std::string, int>> dynamic_cameras_;

    std::mutex config_mutex_;
    std::vector<std::string> config_topics_;
    std::vector<std::string> thermal_topics_;

    std::thread probe_thread_;
    std::atomic<bool> probing_{false};
};
