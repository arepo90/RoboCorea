#include "gui/main_window.hpp"
#include "gui/camera_hub.hpp"
#include "gui/video_panel.hpp"
#include "gui/source_manager.hpp"
#include "gui/dashboard_panel.hpp"
#include "gui/odometry_panel.hpp"
#include "gui/digital_twin_panel.hpp"
#include "gui/filter_registry.hpp"
#include "gui/app_settings.hpp"
#include "gui/settings_dialog.hpp"
#include "gui/systems_window.hpp"
#ifdef HAVE_GSTREAMER
#include "gui/speech_processor.hpp"
#include "gui/gst_av_stream.hpp"
#include "gui/gst_mic_sender.hpp"
#endif

#include <QCloseEvent>
#include <QFrame>
#include <QScrollArea>
#include <QSplitter>
#include <QStatusBar>

#include <memory>

MainWindow::MainWindow(rclcpp::Node::SharedPtr node, QWidget* parent)
    : QMainWindow(parent), node_(node)
{
    setWindowTitle("RoboCorea Operator Console");
    resize(1700, 1100);

    AppSettings::instance().load();

    camera_hub_ = std::make_shared<CameraHub>();
    video_panel_ = new VideoPanel(node_, camera_hub_, this);

    // Layout: the left section is a 2×2 grid of 3 video cells plus the telemetry
    // (odometry) panel in the bottom-right cell; the right section is the digital
    // twin on top over the dashboard (connections), which now fills the space the
    // telemetry used to share. buildRightColumn() creates the panels.
    auto* right_column = buildRightColumn();
    video_panel_->setCornerWidget(wrapScroll(odometry_panel_));

    auto* main_splitter = new QSplitter(Qt::Horizontal, this);
    main_splitter->addWidget(video_panel_);
    main_splitter->addWidget(right_column);
    main_splitter->setStretchFactor(0, 1);  // video takes the slack on resize
    main_splitter->setSizes({980, 620});
    setCentralWidget(main_splitter);

    // Populate the per-widget filter dropdowns from the CV filter registry
    // (registerFilters() runs in main() before the window is built).
    QStringList filter_names{"None"};
    for (const auto& name : FilterRegistry::instance().getFilterNames())
        filter_names << QString::fromStdString(name);
    video_panel_->updateFilters(filter_names);

    // A/V SRT receivers must be registered with CameraHub before sources are
    // discovered (so the "av:<i>" ids resolve to live frames).
#ifdef HAVE_GSTREAMER
    setupAvStreams();
#endif

    source_manager_ = new SourceManager(node_, camera_hub_, this);
    connect(source_manager_, &SourceManager::sourcesUpdated,
            this, &MainWindow::onSourcesUpdated);

    // Selecting a thermal source anywhere enables the thermal sensor bit.
    connect(video_panel_, &VideoPanel::thermalActiveChanged,
            dashboard_panel_, &DashboardPanel::setThermalEnabled);
    connect(dashboard_panel_, &DashboardPanel::resetSourcesRequested,
            source_manager_, &SourceManager::discoverSources);
    connect(dashboard_panel_, &DashboardPanel::settingsRequested,
            this, &MainWindow::onSettingsRequested);

    // Robot Systems window: perception stacks, the named-map library, and the
    // autonomous arm routine programmer. Created eagerly (hidden) so its stack
    // clients/subscriptions are live (and stop-on-close works) and a programmed
    // routine keeps running even when the window isn't shown.
    systems_window_ = new SystemsWindow(node_);   // top-level (no parent)
    connect(dashboard_panel_, &DashboardPanel::systemsRequested, this, [this]() {
        systems_window_->show();
        systems_window_->raise();
        systems_window_->activateWindow();
    });

    // PPM calibration goes to the ESP32 (via the Jetson bridge). Latched so the
    // bridge gets the latest calibration even if it joins after the GUI.
    auto cfg_qos = rclcpp::QoS(1).reliable().transient_local();
    ppm_calib_pub_ = node_->create_publisher<std_msgs::msg::UInt16MultiArray>(
        "/robot/ppm_calib", cfg_qos);
#ifdef HAVE_GSTREAMER
    // Audio-monitor toggle mutes/unmutes the speaker side of every A/V stream
    // (robot mic → this workstation's speaker).
    connect(dashboard_panel_, &DashboardPanel::audioMonitorToggled, this,
            [this](bool en) {
                for (auto& s : av_streams_) s->setPlaybackEnabled(en);
            });
    // Talkback toggle starts/stops streaming this workstation's mic to the robot
    // (operator mic → robot speaker). The mic is only opened while ON.
    connect(dashboard_panel_, &DashboardPanel::talkbackToggled, this,
            [this](bool en) {
                talkback_on_ = en;
                if (!mic_sender_) return;
                if (en) mic_sender_->start();
                else    mic_sender_->stop();
            });
    talkback_on_ = AppSettings::instance().talkback_start_enabled.load();
    setupMicSender();
#endif

    startRosSpinThread();
    source_manager_->discoverSources();

    // Push the persisted PPM calibration once at startup (latched).
    publishPpmCalib();
}

MainWindow::~MainWindow()
{
    ros_running_ = false;
    if (ros_thread_.joinable())
        ros_thread_.join();
#ifdef HAVE_GSTREAMER
    // Stop A/V receivers before the dashboard/speech processor (QObject children)
    // are torn down, so no audio callback fires into a dead SpeechProcessor.
    av_streams_.clear();
    // Stop the talkback sender too (closes the mic + SRT connection).
    if (mic_sender_) { mic_sender_->stop(); mic_sender_.reset(); }
#endif
    // Top-level, parentless window — delete explicitly (after the ROS spin thread
    // has stopped, so no callback fires into its destroyed subscriptions).
    delete systems_window_;
}

void MainWindow::closeEvent(QCloseEvent* event)
{
    // Closing the operator console stops the robot's perception stacks (sensors,
    // 2-D + 3-D mapping, localization) via robot_manager, so nothing is left running on
    // the Jetson. Best-effort + bounded: the spin thread is still alive here, so
    // the requests get delivered; if robot_manager is unreachable it's a no-op.
    if (systems_window_)
        systems_window_->stopAllStacks();
    QMainWindow::closeEvent(event);
}

#ifdef HAVE_GSTREAMER
void MainWindow::setupAvStreams()
{
    auto streams = AppSettings::instance().videoStreams();
    std::string default_host;
    bool playback;
    {
        std::lock_guard<std::mutex> lk(AppSettings::instance().video_mutex);
        default_host = AppSettings::instance().default_robot_host;
    }
    playback = AppSettings::instance().audio_start_enabled.load();

    SpeechProcessor* speech = dashboard_panel_->speechProcessor();

    for (size_t i = 0; i < streams.size(); ++i) {
        const auto& s = streams[i];
        if (!s.audio) continue;
        const std::string host = s.host.empty() ? default_host : s.host;
        if (host.empty()) continue;

        auto av = std::make_shared<GstAvStream>(
            host, s.port, s.latency_ms,
            [speech](const int16_t* pcm, size_t n) {
                if (speech) speech->pushAudio(pcm, n);
            });
        av->setPlaybackEnabled(playback);
        av->start();

        const std::string id = "av:" + std::to_string(i);
        std::weak_ptr<GstAvStream> w = av;
        camera_hub_->registerFrameSource(
            id,
            [w]() { auto sp = w.lock(); return sp ? sp->latestVideoFrame() : cv::Mat(); },
            [w]() { auto sp = w.lock(); return sp && sp->connected(); });

        av_streams_.push_back(std::move(av));
    }
}

void MainWindow::setupMicSender()
{
    // (Re)create the operator→robot talkback sender from settings. Called at
    // startup and after the settings dialog applies (so host/mic/port edits take
    // effect). The mic is only opened when talkback is ON (talkback_on_).
    auto& S = AppSettings::instance();
    std::string host, mic;
    int port, latency;
    {
        std::lock_guard<std::mutex> lk(S.video_mutex);
        host    = S.talkback_host.empty() ? S.default_robot_host : S.talkback_host;
        port    = S.talkback_port;
        latency = S.talkback_latency_ms;
    }
    {
        std::lock_guard<std::mutex> lk(S.strings_mutex);
        mic = S.mic_device;
    }
    const int kbps = S.talkback_opus_kbps.load();

    if (mic_sender_) { mic_sender_->stop(); mic_sender_.reset(); }
    if (host.empty()) return;   // no robot host configured yet

    mic_sender_ = std::make_shared<GstMicSender>(host, port, latency, mic, kbps);
    if (talkback_on_)
        mic_sender_->start();
}
#endif  // HAVE_GSTREAMER

QWidget* MainWindow::wrapScroll(QWidget* w)
{
    auto* sa = new QScrollArea(this);
    sa->setWidgetResizable(true);
    sa->setWidget(w);
    sa->setFrameShape(QFrame::NoFrame);
    return sa;
}

QWidget* MainWindow::buildRightColumn()
{
    // Top: the digital twin (3-D URDF view from /robot_description + /joint_states),
    // spanning the full width of the right section.
    digital_twin_panel_ = new DigitalTwinPanel(node_, this);

    // The odometry (telemetry) panel is created here but hosted in the video
    // grid's bottom-right cell (see the constructor); the dashboard (connections)
    // now occupies the whole bottom of this right section.
    odometry_panel_ = new OdometryPanel(node_, this);
    dashboard_panel_ = new DashboardPanel(node_, this);

    auto* right_splitter = new QSplitter(Qt::Vertical, this);
    right_splitter->addWidget(digital_twin_panel_);
    right_splitter->addWidget(wrapScroll(dashboard_panel_));
    right_splitter->setSizes({420, 480});
    return right_splitter;
}

void MainWindow::startRosSpinThread()
{
    ros_thread_ = std::thread([this]() {
        while (ros_running_) {
            rclcpp::spin_some(node_);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });
}

void MainWindow::onSourcesUpdated()
{
    video_panel_->updateSources(
        source_manager_->sourceNames(),
        source_manager_->sourceIdentifiers());
}

void MainWindow::onSettingsRequested()
{
    if (!settings_dialog_) {
        settings_dialog_ = new SettingsDialog(node_, this);
        connect(settings_dialog_, &SettingsDialog::settingsApplied,
                this, &MainWindow::onSettingsApplied);
    }
    settings_dialog_->reloadFromSettings();
    settings_dialog_->show();
    settings_dialog_->activateWindow();
    settings_dialog_->raise();
}

void MainWindow::onSettingsApplied()
{
    // Push speech/audio prefs into the live dashboard, then republish the PPM
    // calibration so the robot picks up calibration edits immediately.
    // applySpeechAudioSettings() drives the talkback toggle from settings, which
    // updates talkback_on_ via talkbackToggled before we rebuild the sender.
    dashboard_panel_->applySpeechAudioSettings();
#ifdef HAVE_GSTREAMER
    setupMicSender();   // pick up talkback host/mic/port/kbps edits
#endif
    publishPpmCalib();
}

void MainWindow::publishPpmCalib()
{
    auto& S = AppSettings::instance();
    std_msgs::msg::UInt16MultiArray msg;
    msg.data.resize(19);   // 6 channels × {min, neutral, max} + deadband×1000
    {
        std::lock_guard<std::mutex> lk(S.ppm_calib_mutex);
        for (int c = 0; c < 6; ++c) {
            msg.data[c * 3 + 0] = static_cast<uint16_t>(S.ppm_calib[c].min_us);
            msg.data[c * 3 + 1] = static_cast<uint16_t>(S.ppm_calib[c].neutral_us);
            msg.data[c * 3 + 2] = static_cast<uint16_t>(S.ppm_calib[c].max_us);
        }
    }
    // 19th value = global deadband ×1000 (firmware + bridge both require length 19).
    msg.data[18] = static_cast<uint16_t>(S.ppm_deadband_1000.load());
    ppm_calib_pub_->publish(msg);
}
