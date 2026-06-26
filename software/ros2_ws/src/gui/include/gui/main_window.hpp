#pragma once

#include <QMainWindow>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>

#include <atomic>
#include <memory>
#include <thread>
#include <vector>

class CameraHub;
class VideoPanel;
class SourceManager;
class DashboardPanel;
class OdometryPanel;
class DigitalTwinPanel;
class GstAvStream;
class SettingsDialog;
class SystemsWindow;
class QWidget;
class QCloseEvent;

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(rclcpp::Node::SharedPtr node, QWidget* parent = nullptr);
    ~MainWindow() override;

protected:
    // Stop the robot's perception stacks (via the dashboard) before the window
    // closes, so quitting the GUI leaves nothing running on the Jetson.
    void closeEvent(QCloseEvent* event) override;

private slots:
    void onSourcesUpdated();
    void onSettingsRequested();   // open the SettingsDialog (lazy)
    void onSettingsApplied();     // republish ppm_calib + push grammar/audio

private:
    void startRosSpinThread();
    void publishPpmCalib();       // AppSettings.ppm_calib → /robot/ppm_calib (18 u16)
    QWidget* buildRightColumn();      // digital twin (top) + dashboard (bottom)
    QWidget* wrapScroll(QWidget* w);  // scrollable container for a tall panel
#ifdef HAVE_GSTREAMER
    void setupAvStreams();   // create + register the C920 A/V SRT receivers
#endif

    rclcpp::Node::SharedPtr node_;

    VideoPanel* video_panel_;
    SourceManager* source_manager_;
    DashboardPanel* dashboard_panel_;
    OdometryPanel* odometry_panel_;
    DigitalTwinPanel* digital_twin_panel_;
    SettingsDialog* settings_dialog_{nullptr};
    SystemsWindow* systems_window_{nullptr};   // created eagerly (hidden); toolbar icon shows it
    std::shared_ptr<CameraHub> camera_hub_;

    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr ppm_calib_pub_;
#ifdef HAVE_GSTREAMER
    std::vector<std::shared_ptr<GstAvStream>> av_streams_;
#endif

    std::thread ros_thread_;
    std::atomic<bool> ros_running_{true};
};
