#include "gui/dashboard_panel.hpp"
#include "gui/speech_processor.hpp"
#include "gui/app_settings.hpp"
#include "gui/map_window.hpp"

#include <QFont>
#include <QGraphicsOpacityEffect>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QIcon>
#include <QMessageBox>
#include <QSignalBlocker>
#include <QTextEdit>
#include <QVBoxLayout>

#include <chrono>
#include <cmath>
#include <vector>

// Instant amber "pending" feedback on a stack's status LED+label the moment a
// start/stop is clicked, so the UI responds immediately instead of waiting for
// the status topic round-trip from the robot.
static void setStackPending(QLabel* indicator, QLabel* label, const QString& text)
{
    if (!indicator || !label) return;
    label->setText(text);
    indicator->setStyleSheet("color: #ccaa00; font-size: 14px;");
    label->setStyleSheet("color: #ccaa00; font-size: 12px;");
}

DashboardPanel::DashboardPanel(rclcpp::Node::SharedPtr node, QWidget* parent)
    : QWidget(parent), node_(node)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(10, 10, 10, 10);
    layout->setSpacing(8);

    const QString hdr_style = "color: #aaa; font-weight: bold;";
    const QString lbl_style = "color: #888;";
    const QString val_style = "color: #4fc3f7; font-size: 13px;";

    // ── Connection status ────────────────────────────────────────────────────
    auto* conn_hdr = new QLabel("Connection Status", this);
    conn_hdr->setAlignment(Qt::AlignHCenter);
    conn_hdr->setStyleSheet(hdr_style);
    layout->addWidget(conn_hdr);

    auto* conn_row = new QHBoxLayout();
    conn_row->setSpacing(6);
    conn_indicator_ = new QLabel("●", this);
    conn_indicator_->setStyleSheet("color: #cc3333; font-size: 14px;");
    conn_label_ = new QLabel("Offline", this);
    conn_label_->setStyleSheet(lbl_style);
    uptime_label_ = new QLabel("--", this);
    uptime_label_->setStyleSheet("color: #4fc3f7; font-size: 12px;");
    conn_row->addStretch();
    conn_row->addWidget(conn_indicator_);
    conn_row->addWidget(conn_label_);
    conn_row->addWidget(uptime_label_);
    conn_row->addStretch();
    layout->addLayout(conn_row);

    auto* opacity = new QGraphicsOpacityEffect(conn_indicator_);
    conn_indicator_->setGraphicsEffect(opacity);
    pulse_anim_ = new QPropertyAnimation(opacity, "opacity", this);
    pulse_anim_->setDuration(800);
    pulse_anim_->setKeyValueAt(0.0, 1.0);
    pulse_anim_->setKeyValueAt(0.4, 0.5);
    pulse_anim_->setKeyValueAt(1.0, 1.0);
    pulse_anim_->setEasingCurve(QEasingCurve::InOutSine);

    auto add_hsep = [&]() {
        auto* s = new QFrame(this);
        s->setFrameShape(QFrame::HLine);
        s->setStyleSheet("color: #444;");
        layout->addWidget(s);
    };
    add_hsep();

    // ── Helpers ──────────────────────────────────────────────────────────────
    auto make_val = [&]() {
        auto* l = new QLabel("--", this);
        l->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
        l->setStyleSheet(val_style);
        return l;
    };
    auto make_axis = [&](const char* text) {
        auto* l = new QLabel(text, this);
        l->setAlignment(Qt::AlignHCenter);
        l->setStyleSheet(lbl_style);
        return l;
    };
    auto make_sensor_toggle = [&]() {
        auto* btn = new QPushButton("OFF", this);
        btn->setCheckable(true);
        btn->setFixedSize(36, 18);
        QFont f = btn->font();
        f.setPointSize(7);
        btn->setFont(f);
        btn->setStyleSheet(
            "QPushButton { background-color: #3a2a2a; color: #888; padding: 1px; "
            "border: 1px solid #5a3a3a; border-radius: 3px; }"
            "QPushButton:hover { background-color: #4a3030; }"
            "QPushButton:checked { background-color: #1a5a1a; color: #8afa8a; "
            "border-color: #2a8a2a; }");
        return btn;
    };

    // ── Magnetometer ─────────────────────────────────────────────────────────
    {
        auto* mag_hdr_row = new QHBoxLayout();
        mag_hdr_row->setSpacing(4);
        auto* mag_hdr = new QLabel("Magnetometer (µT)", this);
        mag_hdr->setStyleSheet(hdr_style);
        mag_toggle_ = make_sensor_toggle();
        connect(mag_toggle_, &QPushButton::toggled, this, &DashboardPanel::onSensorToggled);
        mag_hdr_row->addWidget(mag_hdr, 1);
        mag_hdr_row->addWidget(mag_toggle_);
        layout->addLayout(mag_hdr_row);
    }
    mag_x_ = make_val(); mag_y_ = make_val(); mag_z_ = make_val();
    for (auto [lbl, val] : {std::pair{"X", mag_x_}, {"Y", mag_y_}, {"Z", mag_z_}}) {
        auto* row = new QHBoxLayout();
        row->addWidget(make_axis(lbl));
        row->addWidget(val, 1);
        layout->addLayout(row);
    }

    add_hsep();

    // ── Orientation (from the ZED2 IMU; no ESP32 IMU) ─────────────────────────
    // Sourced from the ZED2 camera driver, not the ESP32 — so there is no
    // sensor-enable toggle here (the ZED isn't controlled by the enable mask).
    {
        auto* imu_hdr = new QLabel("Orientation (ZED2)", this);
        imu_hdr->setStyleSheet(hdr_style);
        layout->addWidget(imu_hdr);
    }
    auto* imu_row = new QHBoxLayout();
    imu_row->setSpacing(8);
    imu_yaw_ = make_val(); imu_pitch_ = make_val(); imu_roll_ = make_val();
    for (auto [lbl, val] : {std::pair{"Yaw", imu_yaw_}, {"Pitch", imu_pitch_}, {"Roll", imu_roll_}}) {
        auto* col = new QVBoxLayout();
        col->setSpacing(1);
        col->addWidget(make_axis(lbl));
        col->addWidget(val);
        imu_row->addLayout(col);
    }
    layout->addLayout(imu_row);

    add_hsep();

    // ── Transcription + audio monitor ────────────────────────────────────────
    {
        auto* trans_hdr_row = new QHBoxLayout();
        trans_hdr_row->setSpacing(4);
        auto* trans_label = new QLabel("Transcription", this);
        trans_label->setStyleSheet(hdr_style);
        audio_btn_ = make_sensor_toggle();
        connect(audio_btn_, &QPushButton::toggled, this, &DashboardPanel::onAudioToggled);
        trans_hdr_row->addWidget(trans_label);
        trans_hdr_row->addStretch();
        trans_hdr_row->addWidget(audio_btn_);
        layout->addLayout(trans_hdr_row);
    }
    transcription_ = new QTextEdit(this);
    transcription_->setReadOnly(true);
    transcription_->setMaximumHeight(75);
    transcription_->setStyleSheet(
        "background-color: #1a1a2e; color: #c0c0c0; border: 1px solid #333; "
        "font-size: 14px; padding: 4px;");
    transcription_->setPlaceholderText("Waiting for audio…");
    layout->addWidget(transcription_);

    speech_processor_ = new SpeechProcessor(node_, this);
    connect(speech_processor_, &SpeechProcessor::transcriptionUpdated,
            this, &DashboardPanel::onTranscriptionUpdated, Qt::QueuedConnection);
    {
        std::lock_guard<std::mutex> lk(AppSettings::instance().strings_mutex);
        speech_processor_->setGrammar(AppSettings::instance().vosk_grammar);
    }
    audio_btn_->setChecked(AppSettings::instance().audio_start_enabled.load());

    // ── Subscriptions (ROS thread → Qt via queued signals) ───────────────────
    connect(this, &DashboardPanel::magnetometerUpdated,
            this, &DashboardPanel::onMagnetometerUpdated, Qt::QueuedConnection);
    connect(this, &DashboardPanel::imuUpdated,
            this, &DashboardPanel::onImuUpdated, Qt::QueuedConnection);
    connect(this, &DashboardPanel::telemetryReceived,
            this, &DashboardPanel::onTelemetryReceived, Qt::QueuedConnection);
    connect(this, &DashboardPanel::uptimeUpdated,
            this, &DashboardPanel::onUptimeUpdated, Qt::QueuedConnection);

    auto sensor_qos = rclcpp::QoS(10).best_effort();

    mag_sub_ = node_->create_subscription<sensor_msgs::msg::MagneticField>(
        "/sensors/mag", sensor_qos,
        [this](sensor_msgs::msg::MagneticField::SharedPtr msg) {
            constexpr double TESLA_TO_MICROTESLA = 1e6;
            emit magnetometerUpdated(
                msg->magnetic_field.x * TESLA_TO_MICROTESLA,
                msg->magnetic_field.y * TESLA_TO_MICROTESLA,
                msg->magnetic_field.z * TESLA_TO_MICROTESLA);
        });

    // Orientation comes from the ZED2 camera's IMU (the deferred ZED/nav stack),
    // not the ESP32. Exact topic depends on the ZED launch config; this is the
    // zed-ros2-wrapper default and stays blank until that node runs.
    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        "/zed/zed_node/imu/data", sensor_qos,   // namespace = camera_name (default 'zed')
        [this](sensor_msgs::msg::Imu::SharedPtr msg) {
            double qw = msg->orientation.w, qx = msg->orientation.x;
            double qy = msg->orientation.y, qz = msg->orientation.z;

            double sinr = 2.0 * (qw * qx + qy * qz);
            double cosr = 1.0 - 2.0 * (qx * qx + qy * qy);
            double roll = std::atan2(sinr, cosr) * 180.0 / M_PI;

            double sinp = 2.0 * (qw * qy - qz * qx);
            double pitch = std::abs(sinp) >= 1.0 ? std::copysign(90.0, sinp)
                                                 : std::asin(sinp) * 180.0 / M_PI;

            double siny = 2.0 * (qw * qz + qx * qy);
            double cosy = 1.0 - 2.0 * (qy * qy + qz * qz);
            double yaw = std::atan2(siny, cosy) * 180.0 / M_PI;

            emit imuUpdated(yaw, pitch, roll);
        });

    telemetry_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/robot/telemetry", sensor_qos,
        [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            emit telemetryReceived();
            if (msg->data.size() >= 4)
                emit uptimeUpdated(msg->data[3]);
        });

    heartbeat_timer_ = new QTimer(this);
    heartbeat_timer_->setInterval(1000);
    connect(heartbeat_timer_, &QTimer::timeout, this, &DashboardPanel::onHeartbeatCheck);
    heartbeat_timer_->start();

    // Latched (transient_local) so the jetson_sensors nodes pick up the last
    // enable choice when they (re)start — e.g. after an I2C start/stop. Must match
    // the nodes' enable_mask_qos() durability.
    sensor_mask_pub_ = node_->create_publisher<std_msgs::msg::UInt8>(
        "/sensors/enable_mask", rclcpp::QoS(10).reliable().transient_local());
    publishSensorMask();  // start with all sensors off

    layout->addStretch();
    add_hsep();

    // ── Controls ─────────────────────────────────────────────────────────────
    auto btn_style = [](const char* bg, const char* hover, const char* pressed) {
        return QString(
            "QPushButton { background-color: %1; color: white; padding: 6px; "
            "border: 1px solid %2; border-radius: 3px; }"
            "QPushButton:hover { background-color: %2; }"
            "QPushButton:pressed { background-color: %3; }")
            .arg(bg, hover, pressed);
    };

    estop_btn_ = new QPushButton("E-STOP", this);
    estop_btn_->setCheckable(true);
    estop_btn_->setMinimumHeight(50);
    QFont estop_font = estop_btn_->font();
    estop_font.setPointSize(16);
    estop_font.setBold(true);
    estop_btn_->setFont(estop_font);
    estop_btn_->setStyleSheet(
        "QPushButton { background-color: #5a1a1a; color: white; padding: 10px; "
        "border: 2px solid #8a2a2a; border-radius: 5px; }"
        "QPushButton:hover { background-color: #6a2a2a; }"
        "QPushButton:checked { background-color: #cc0000; border-color: #ff3333; }");
    connect(estop_btn_, &QPushButton::toggled, this, &DashboardPanel::onEstopToggled);
    layout->addWidget(estop_btn_);

    // ── Autonomy drive (/cmd_vel → tracks) ───────────────────────────────────
    // One-click allow/prevent for Nav2 driving the tracks. The firmware still
    // requires the RC drive sticks to be neutral; this is the extra software gate.
    // The button reflects /autonomy/state, so an operator RC override (which the
    // bridge latches) un-checks it and forces a deliberate re-enable here.
    autonomy_btn_ = new QPushButton("AUTO DRIVE: OFF", this);
    autonomy_btn_->setCheckable(true);
    autonomy_btn_->setMinimumHeight(34);
    autonomy_btn_->setToolTip(
        "Allow Nav2 /cmd_vel to drive the tracks (autonomy).\n"
        "Even when ON, the tracks only move while the RC drive sticks are neutral.\n"
        "Touching a drive stick or virtual-flip hands control back and turns this OFF.");
    autonomy_btn_->setStyleSheet(
        "QPushButton { background-color: #2a3a2a; color: #cfcfcf; padding: 6px; "
        "border: 2px solid #3a5a3a; border-radius: 5px; font-weight: bold; }"
        "QPushButton:hover { background-color: #344a34; }"
        "QPushButton:checked { background-color: #c8870a; color: black; "
        "border-color: #ffb13a; }");
    connect(autonomy_btn_, &QPushButton::toggled, this, &DashboardPanel::onAutonomyToggled);
    layout->addWidget(autonomy_btn_);

    // ── Arm lifecycle (boots disarmed; arm/disarm + dexterity/chassis idle) ───
    add_hsep();
    {
        auto* arm_hdr_row = new QHBoxLayout();
        auto* arm_hdr = new QLabel("Arm", this);
        arm_hdr->setStyleSheet(hdr_style);
        arm_state_indicator_ = new QLabel("●", this);
        arm_state_indicator_->setStyleSheet("color: #888; font-size: 14px;");
        arm_state_label_ = new QLabel("—", this);
        arm_state_label_->setStyleSheet("color: #888; font-size: 12px;");
        arm_hdr_row->addWidget(arm_hdr);
        arm_hdr_row->addStretch();
        arm_hdr_row->addWidget(arm_state_indicator_);
        arm_hdr_row->addWidget(arm_state_label_);
        layout->addLayout(arm_hdr_row);
    }
    {
        // Per-joint CAN presence (green once the joint's zero was captured at arm).
        auto* can_row = new QHBoxLayout();
        can_row->setSpacing(4);
        auto* can_lbl = new QLabel("CAN", this);
        can_lbl->setStyleSheet(lbl_style);
        can_row->addWidget(can_lbl);
        for (int j = 0; j < 6; ++j) {
            arm_can_dots_[j] = new QLabel(QString("J%1").arg(j + 1), this);
            arm_can_dots_[j]->setAlignment(Qt::AlignHCenter);
            arm_can_dots_[j]->setStyleSheet("color: #666; font-size: 10px; font-weight: bold;");
            can_row->addWidget(arm_can_dots_[j]);
        }
        can_row->addStretch();
        layout->addLayout(can_row);
    }
    {
        auto* arm_btn_row = new QHBoxLayout();
        arm_btn_row->setSpacing(4);

        arm_btn_ = new QPushButton("Arm", this);
        arm_btn_->setMinimumHeight(28);
        arm_btn_->setToolTip("Arm/init the manipulator (it boots disarmed for safety)");
        arm_btn_->setStyleSheet(btn_style("#1a5a2a", "#2a7a3a", "#0a3a1a"));
        connect(arm_btn_, &QPushButton::clicked, this, &DashboardPanel::onArmClicked);
        arm_btn_row->addWidget(arm_btn_);

        arm_disarm_btn_ = new QPushButton("Disarm", this);
        arm_disarm_btn_->setMinimumHeight(28);
        arm_disarm_btn_->setToolTip("Disarm (torque-off) the manipulator");
        arm_disarm_btn_->setStyleSheet(btn_style("#5a2a2a", "#7a3a3a", "#3a1a1a"));
        connect(arm_disarm_btn_, &QPushButton::clicked, this, &DashboardPanel::onDisarmClicked);
        arm_btn_row->addWidget(arm_disarm_btn_);

        arm_mode_btn_ = new QPushButton("Mode: Dexterity", this);
        arm_mode_btn_->setMinimumHeight(28);
        arm_mode_btn_->setToolTip("Toggle Dexterity ⇄ Chassis. Chassis idles the wrist "
                                  "(J5/J6 torque-off) — use it when the arm is parked.");
        arm_mode_btn_->setStyleSheet(btn_style("#2a4a7f", "#3a5a9f", "#1a3a6f"));
        connect(arm_mode_btn_, &QPushButton::clicked, this, &DashboardPanel::onArmModeToggle);
        arm_btn_row->addWidget(arm_mode_btn_);

        layout->addLayout(arm_btn_row);
    }

    // ── Sensor stack (ZED + RPLidar; started/stopped on the Jetson via systemd) ─
    add_hsep();
    {
        auto* sens_hdr_row = new QHBoxLayout();
        auto* sens_hdr = new QLabel("Sensors", this);
        sens_hdr->setStyleSheet(hdr_style);
        sensors_indicator_ = new QLabel("●", this);
        sensors_indicator_->setStyleSheet("color: #888; font-size: 14px;");
        sensors_label_ = new QLabel("—", this);
        sensors_label_->setStyleSheet("color: #888; font-size: 12px;");
        sens_hdr_row->addWidget(sens_hdr);
        sens_hdr_row->addStretch();
        sens_hdr_row->addWidget(sensors_indicator_);
        sens_hdr_row->addWidget(sensors_label_);
        layout->addLayout(sens_hdr_row);

        auto* sens_btn_row = new QHBoxLayout();
        sens_btn_row->setSpacing(4);

        sensors_start_btn_ = new QPushButton("Start ZED+Lidar", this);
        sensors_start_btn_->setMinimumHeight(28);
        sensors_start_btn_->setToolTip("Start the ZED + RPLidar stack on the robot "
                                       "(systemd rescue-sensors.target via robot_manager)");
        sensors_start_btn_->setStyleSheet(btn_style("#1a5a2a", "#2a7a3a", "#0a3a1a"));
        connect(sensors_start_btn_, &QPushButton::clicked, this,
                &DashboardPanel::onSensorsStartClicked);
        sens_btn_row->addWidget(sensors_start_btn_);

        sensors_stop_btn_ = new QPushButton("Stop", this);
        sensors_stop_btn_->setMinimumHeight(28);
        sensors_stop_btn_->setToolTip("Cleanly stop the ZED + RPLidar stack on the robot");
        sensors_stop_btn_->setStyleSheet(btn_style("#5a2a2a", "#7a3a3a", "#3a1a1a"));
        connect(sensors_stop_btn_, &QPushButton::clicked, this,
                &DashboardPanel::onSensorsStopClicked);
        sens_btn_row->addWidget(sensors_stop_btn_);

        layout->addLayout(sens_btn_row);
    }

    // ── I2C sensors (MLX90640 thermal + LIS3MDL mag; Jetson process + per-sensor) ─
    add_hsep();
    {
        auto* i2c_hdr_row = new QHBoxLayout();
        auto* i2c_hdr = new QLabel("I2C Sensors", this);
        i2c_hdr->setStyleSheet(hdr_style);
        i2c_indicator_ = new QLabel("●", this);
        i2c_indicator_->setStyleSheet("color: #888; font-size: 14px;");
        i2c_label_ = new QLabel("—", this);
        i2c_label_->setStyleSheet("color: #888; font-size: 12px;");
        i2c_hdr_row->addWidget(i2c_hdr);
        i2c_hdr_row->addStretch();
        i2c_hdr_row->addWidget(i2c_indicator_);
        i2c_hdr_row->addWidget(i2c_label_);
        layout->addLayout(i2c_hdr_row);

        // Layer 1: start/stop the jetson_sensors driver process on the Jetson.
        auto* i2c_btn_row = new QHBoxLayout();
        i2c_btn_row->setSpacing(4);
        i2c_start_btn_ = new QPushButton("Start I2C", this);
        i2c_start_btn_->setMinimumHeight(28);
        i2c_start_btn_->setToolTip("Start the MLX90640 + LIS3MDL driver on the robot "
                                   "(systemd jetson-sensors.service via robot_manager)");
        i2c_start_btn_->setStyleSheet(btn_style("#1a5a2a", "#2a7a3a", "#0a3a1a"));
        connect(i2c_start_btn_, &QPushButton::clicked, this, &DashboardPanel::onI2cStartClicked);
        i2c_btn_row->addWidget(i2c_start_btn_);

        i2c_stop_btn_ = new QPushButton("Stop", this);
        i2c_stop_btn_->setMinimumHeight(28);
        i2c_stop_btn_->setToolTip("Cleanly stop the I2C sensor driver on the robot");
        i2c_stop_btn_->setStyleSheet(btn_style("#5a2a2a", "#7a3a3a", "#3a1a1a"));
        connect(i2c_stop_btn_, &QPushButton::clicked, this, &DashboardPanel::onI2cStopClicked);
        i2c_btn_row->addWidget(i2c_stop_btn_);
        layout->addLayout(i2c_btn_row);

        // Layer 2: per-sensor runtime enable via /sensors/enable_mask. A disabled
        // sensor stops touching the bus, so this is the shared-bus-safe control.
        auto* en_row = new QHBoxLayout();
        en_row->setSpacing(4);
        auto* thermal_lbl = new QLabel("Thermal", this);
        thermal_lbl->setStyleSheet(lbl_style);
        thermal_toggle_ = make_sensor_toggle();   // OFF by default (mask bit cleared)
        thermal_toggle_->setToolTip("Enable thermal acquisition (enable_mask bit 1). "
                                    "Also auto-enabled when the thermal video source is selected.");
        connect(thermal_toggle_, &QPushButton::toggled, this, &DashboardPanel::onThermalToggled);
        en_row->addWidget(thermal_lbl);
        en_row->addWidget(thermal_toggle_);
        en_row->addStretch();
        layout->addLayout(en_row);
    }

    // ── Mapping / SLAM (slam_toolbox + EKF on the Jetson; map viewed here) ─────
    add_hsep();
    {
        auto* map_hdr_row = new QHBoxLayout();
        auto* map_hdr = new QLabel("Mapping", this);
        map_hdr->setStyleSheet(hdr_style);
        mapping_indicator_ = new QLabel("●", this);
        mapping_indicator_->setStyleSheet("color: #888; font-size: 14px;");
        mapping_label_ = new QLabel("—", this);
        mapping_label_->setStyleSheet("color: #888; font-size: 12px;");
        map_hdr_row->addWidget(map_hdr);
        map_hdr_row->addStretch();
        map_hdr_row->addWidget(mapping_indicator_);
        map_hdr_row->addWidget(mapping_label_);
        layout->addLayout(map_hdr_row);

        auto* map_btn_row = new QHBoxLayout();
        map_btn_row->setSpacing(4);
        mapping_start_btn_ = new QPushButton("Start SLAM", this);
        mapping_start_btn_->setMinimumHeight(28);
        mapping_start_btn_->setToolTip("Start slam_toolbox + EKF on the robot "
                                       "(systemd rescue-mapping.service). Start the "
                                       "sensors first.");
        mapping_start_btn_->setStyleSheet(btn_style("#1a5a2a", "#2a7a3a", "#0a3a1a"));
        connect(mapping_start_btn_, &QPushButton::clicked, this, &DashboardPanel::onMappingStartClicked);
        map_btn_row->addWidget(mapping_start_btn_);

        mapping_stop_btn_ = new QPushButton("Stop", this);
        mapping_stop_btn_->setMinimumHeight(28);
        mapping_stop_btn_->setToolTip("Cleanly stop SLAM + EKF on the robot");
        mapping_stop_btn_->setStyleSheet(btn_style("#5a2a2a", "#7a3a3a", "#3a1a1a"));
        connect(mapping_stop_btn_, &QPushButton::clicked, this, &DashboardPanel::onMappingStopClicked);
        map_btn_row->addWidget(mapping_stop_btn_);

        open_map_btn_ = new QPushButton("Open Map", this);
        open_map_btn_->setMinimumHeight(28);
        open_map_btn_->setToolTip("Open a live 2-D map window (subscribes /map + robot pose)");
        open_map_btn_->setStyleSheet(btn_style("#2a4a7f", "#3a5a9f", "#1a3a6f"));
        connect(open_map_btn_, &QPushButton::clicked, this, &DashboardPanel::onOpenMapClicked);
        map_btn_row->addWidget(open_map_btn_);

        layout->addLayout(map_btn_row);
    }

    // ── 3-D mapping (OctoMap on the Jetson; only the octree crosses the net) ──
    add_hsep();
    {
        auto* m3_hdr_row = new QHBoxLayout();
        auto* m3_hdr = new QLabel("3D Mapping", this);
        m3_hdr->setStyleSheet(hdr_style);
        mapping3d_indicator_ = new QLabel("●", this);
        mapping3d_indicator_->setStyleSheet("color: #888; font-size: 14px;");
        mapping3d_label_ = new QLabel("—", this);
        mapping3d_label_->setStyleSheet("color: #888; font-size: 12px;");
        m3_hdr_row->addWidget(m3_hdr);
        m3_hdr_row->addStretch();
        m3_hdr_row->addWidget(mapping3d_indicator_);
        m3_hdr_row->addWidget(mapping3d_label_);
        layout->addLayout(m3_hdr_row);

        auto* m3_btn_row = new QHBoxLayout();
        m3_btn_row->setSpacing(4);
        mapping3d_start_btn_ = new QPushButton("Start 3D", this);
        mapping3d_start_btn_->setMinimumHeight(28);
        mapping3d_start_btn_->setToolTip("Start OctoMap volumetric mapping on the robot "
                                         "(rescue-mapping3d.service). Start sensors + SLAM first.");
        mapping3d_start_btn_->setStyleSheet(btn_style("#1a5a2a", "#2a7a3a", "#0a3a1a"));
        connect(mapping3d_start_btn_, &QPushButton::clicked, this, &DashboardPanel::onMapping3dStartClicked);
        m3_btn_row->addWidget(mapping3d_start_btn_);

        mapping3d_stop_btn_ = new QPushButton("Stop", this);
        mapping3d_stop_btn_->setMinimumHeight(28);
        mapping3d_stop_btn_->setToolTip("Cleanly stop 3-D mapping on the robot");
        mapping3d_stop_btn_->setStyleSheet(btn_style("#5a2a2a", "#7a3a3a", "#3a1a1a"));
        connect(mapping3d_stop_btn_, &QPushButton::clicked, this, &DashboardPanel::onMapping3dStopClicked);
        m3_btn_row->addWidget(mapping3d_stop_btn_);

        open_3dmap_btn_ = new QPushButton("Open 3D Map", this);
        open_3dmap_btn_->setMinimumHeight(28);
        open_3dmap_btn_->setToolTip("Open the live map window (renders /robot/map3d voxels + robot)");
        open_3dmap_btn_->setStyleSheet(btn_style("#2a4a7f", "#3a5a9f", "#1a3a6f"));
        connect(open_3dmap_btn_, &QPushButton::clicked, this, &DashboardPanel::onOpen3dMapClicked);
        m3_btn_row->addWidget(open_3dmap_btn_);

        layout->addLayout(m3_btn_row);
    }

    auto* btn_row = new QHBoxLayout();
    btn_row->setSpacing(4);

    reset_btn_ = new QPushButton("Reset Sources", this);
    reset_btn_->setMinimumHeight(28);
    reset_btn_->setStyleSheet(btn_style("#2a4a7f", "#3a5a9f", "#1a3a6f"));
    connect(reset_btn_, &QPushButton::clicked, this,
            [this]() { emit resetSourcesRequested(); });
    btn_row->addWidget(reset_btn_);

    clear_btn_ = new QPushButton("Clear Data", this);
    clear_btn_->setMinimumHeight(28);
    clear_btn_->setStyleSheet(btn_style("#4a4a2a", "#6a6a3a", "#3a3a1a"));
    connect(clear_btn_, &QPushButton::clicked, this, &DashboardPanel::onClearAll);
    btn_row->addWidget(clear_btn_);

    settings_btn_ = new QPushButton(this);
    settings_btn_->setFixedSize(28, 28);
    settings_btn_->setToolTip("Settings");
    {
        QIcon icon = QIcon::fromTheme("preferences-system");
        if (!icon.isNull()) {
            settings_btn_->setIcon(icon);
            settings_btn_->setIconSize(QSize(16, 16));
        } else {
            settings_btn_->setText("⚙");
        }
    }
    settings_btn_->setStyleSheet(
        "QPushButton { background-color: #2d2d45; color: #ccc; padding: 2px; "
        "border: 1px solid #3a3a55; border-radius: 3px; }"
        "QPushButton:hover { background-color: #3a3a55; }");
    connect(settings_btn_, &QPushButton::clicked, this,
            [this]() { emit settingsRequested(); });
    btn_row->addWidget(settings_btn_);

    layout->addLayout(btn_row);

    auto estop_qos = rclcpp::QoS(10).reliable().transient_local();
    estop_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/robot/estop", estop_qos);
    estop_timer_ = new QTimer(this);
    connect(estop_timer_, &QTimer::timeout, this, &DashboardPanel::publishEstopState);
    estop_timer_->start(100);

    // ── Autonomy enable/state (ROS thread → Qt via queued signal) ─────────────
    // /autonomy/enable is the operator request (VOLATILE: a bridge restart must not
    // auto-resume from a latched "true"). /autonomy/state is the bridge's actual,
    // latched state that drives the button so RC overrides are reflected here.
    connect(this, &DashboardPanel::autonomyStateUpdated,
            this, &DashboardPanel::onAutonomyStateUpdated, Qt::QueuedConnection);
    autonomy_enable_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
        "/autonomy/enable", rclcpp::QoS(10).reliable());
    autonomy_state_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/autonomy/state", rclcpp::QoS(1).reliable().transient_local(),
        [this](std_msgs::msg::Bool::SharedPtr msg) {
            emit autonomyStateUpdated(msg->data);
        });

    // ── Arm lifecycle (ROS thread → Qt via queued signals) ───────────────────
    connect(this, &DashboardPanel::armStateUpdated,
            this, &DashboardPanel::onArmStateUpdated, Qt::QueuedConnection);
    connect(this, &DashboardPanel::armModeUpdated,
            this, &DashboardPanel::onArmModeUpdated, Qt::QueuedConnection);
    connect(this, &DashboardPanel::armPresenceUpdated,
            this, &DashboardPanel::onArmPresenceUpdated, Qt::QueuedConnection);

    // Latched to match the bridge so a late-joining GUI sees the current state.
    auto arm_qos = rclcpp::QoS(1).reliable().transient_local();
    arm_state_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/arm/state", arm_qos, [this](std_msgs::msg::String::SharedPtr msg) {
            emit armStateUpdated(QString::fromStdString(msg->data));
        });
    arm_mode_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/arm/operating_mode", arm_qos, [this](std_msgs::msg::String::SharedPtr msg) {
            emit armModeUpdated(QString::fromStdString(msg->data));
        });
    arm_presence_sub_ = node_->create_subscription<std_msgs::msg::UInt16>(
        "/arm/can_presence", arm_qos, [this](std_msgs::msg::UInt16::SharedPtr msg) {
            emit armPresenceUpdated(static_cast<int>(msg->data));
        });

    arm_cli_           = node_->create_client<std_srvs::srv::Trigger>("/arm/arm");
    arm_disarm_cli_    = node_->create_client<std_srvs::srv::Trigger>("/arm/disarm");
    arm_dexterity_cli_ = node_->create_client<std_srvs::srv::Trigger>("/arm/mode/dexterity");
    arm_chassis_cli_   = node_->create_client<std_srvs::srv::Trigger>("/arm/mode/chassis");

    // ── Sensor stack (robot_manager on the Jetson) ───────────────────────────
    connect(this, &DashboardPanel::sensorsStatusUpdated,
            this, &DashboardPanel::onSensorsStatusUpdated, Qt::QueuedConnection);
    sensors_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/robot/sensors/status", arm_qos,
        [this](std_msgs::msg::String::SharedPtr msg) {
            emit sensorsStatusUpdated(QString::fromStdString(msg->data));
        });
    sensors_start_cli_ = node_->create_client<std_srvs::srv::Trigger>("/robot/sensors/start");
    sensors_stop_cli_  = node_->create_client<std_srvs::srv::Trigger>("/robot/sensors/stop");

    // ── I2C sensor stack (robot_manager 'i2c' stack on the Jetson) ───────────
    connect(this, &DashboardPanel::i2cStatusUpdated,
            this, &DashboardPanel::onI2cStatusUpdated, Qt::QueuedConnection);
    i2c_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/robot/i2c/status", arm_qos,
        [this](std_msgs::msg::String::SharedPtr msg) {
            emit i2cStatusUpdated(QString::fromStdString(msg->data));
        });
    i2c_start_cli_ = node_->create_client<std_srvs::srv::Trigger>("/robot/i2c/start");
    i2c_stop_cli_  = node_->create_client<std_srvs::srv::Trigger>("/robot/i2c/stop");

    // ── Mapping/SLAM stack (robot_manager 'mapping' stack on the Jetson) ─────
    connect(this, &DashboardPanel::mappingStatusUpdated,
            this, &DashboardPanel::onMappingStatusUpdated, Qt::QueuedConnection);
    mapping_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/robot/mapping/status", arm_qos,
        [this](std_msgs::msg::String::SharedPtr msg) {
            emit mappingStatusUpdated(QString::fromStdString(msg->data));
        });
    mapping_start_cli_ = node_->create_client<std_srvs::srv::Trigger>("/robot/mapping/start");
    mapping_stop_cli_  = node_->create_client<std_srvs::srv::Trigger>("/robot/mapping/stop");

    // ── 3-D mapping (OctoMap) stack ──────────────────────────────────────────
    connect(this, &DashboardPanel::mapping3dStatusUpdated,
            this, &DashboardPanel::onMapping3dStatusUpdated, Qt::QueuedConnection);
    mapping3d_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/robot/mapping3d/status", arm_qos,
        [this](std_msgs::msg::String::SharedPtr msg) {
            emit mapping3dStatusUpdated(QString::fromStdString(msg->data));
        });
    mapping3d_start_cli_ = node_->create_client<std_srvs::srv::Trigger>("/robot/mapping3d/start");
    mapping3d_stop_cli_  = node_->create_client<std_srvs::srv::Trigger>("/robot/mapping3d/stop");
}

void DashboardPanel::setConnState(const QString& color, const QString& label)
{
    conn_indicator_->setStyleSheet(QString("color: %1; font-size: 14px;").arg(color));
    conn_label_->setText(label);
}

void DashboardPanel::publishSensorMask()
{
    // The enable toggles are the single source of truth: bit0 = mag, bit1 = thermal.
    // (Both toggles may not exist yet during early construction — treat as off.)
    uint8_t m = 0;
    if (mag_toggle_ && mag_toggle_->isChecked())         m |= static_cast<uint8_t>(1 << 0);
    if (thermal_toggle_ && thermal_toggle_->isChecked()) m |= static_cast<uint8_t>(1 << 1);
    sensor_mask_ = m;
    std_msgs::msg::UInt8 msg;
    msg.data = sensor_mask_;
    sensor_mask_pub_->publish(msg);
}

void DashboardPanel::onTelemetryReceived()
{
    hb_received_ = true;
    setConnState("#33cc33", "Online");
    if (pulse_anim_) {
        pulse_anim_->stop();
        pulse_anim_->start();
    }
}

void DashboardPanel::onUptimeUpdated(float uptime_s)
{
    int secs = static_cast<int>(uptime_s);
    int mins = secs / 60;
    int hours = mins / 60;
    QString text;
    if (hours > 0)      text = QString("%1h%2m").arg(hours).arg(mins % 60);
    else if (mins > 0)  text = QString("%1m%2s").arg(mins).arg(secs % 60);
    else                text = QString("%1s").arg(secs);
    uptime_label_->setText(text);
}

void DashboardPanel::onHeartbeatCheck()
{
    if (hb_received_) {
        hb_received_ = false;
        hb_miss_count_ = 0;
        return;
    }
    if (hb_miss_count_ < 100) hb_miss_count_++;
    if (hb_miss_count_ == 1) {
        setConnState("#ccaa00", "Intermittent");
    } else if (hb_miss_count_ >= 2) {
        setConnState("#cc3333", "Offline");
        uptime_label_->setText("--");
    }
}

void DashboardPanel::onSensorToggled()
{
    mag_toggle_->setText(mag_toggle_->isChecked() ? "ON" : "OFF");
    publishSensorMask();   // recomputes the full mask from both toggles
}

void DashboardPanel::onThermalToggled()
{
    thermal_toggle_->setText(thermal_toggle_->isChecked() ? "ON" : "OFF");
    publishSensorMask();
}

void DashboardPanel::setThermalEnabled(bool enabled)
{
    // Called by the VideoPanel when the thermal source is selected/deselected;
    // drive the thermal toggle so enable_mask bit1 has a single source of truth.
    // setChecked emits toggled() -> onThermalToggled() -> publishSensorMask().
    if (thermal_toggle_)
        thermal_toggle_->setChecked(enabled);
}

void DashboardPanel::onEstopToggled(bool checked)
{
    estop_active_ = checked;
    publishEstopState();
}

void DashboardPanel::onAutonomyToggled(bool checked)
{
    // One-click request to the bridge. Update the label optimistically to match the
    // click; the authoritative /autonomy/state echo (onAutonomyStateUpdated) confirms
    // or corrects it (e.g. the bridge may latch it straight back off on an RC override).
    autonomy_btn_->setText(checked ? "AUTO DRIVE: ON" : "AUTO DRIVE: OFF");
    std_msgs::msg::Bool msg;
    msg.data = checked;
    autonomy_enable_pub_->publish(msg);
    RCLCPP_INFO(node_->get_logger(), "Autonomy drive %s requested",
                checked ? "ENABLE" : "DISABLE");
}

void DashboardPanel::onAutonomyStateUpdated(bool enabled)
{
    // Reflect the bridge's authoritative state without re-emitting toggled() (which
    // would re-publish /autonomy/enable). The bridge latches this OFF on an RC override.
    QSignalBlocker block(autonomy_btn_);
    autonomy_btn_->setChecked(enabled);
    autonomy_btn_->setText(enabled ? "AUTO DRIVE: ON" : "AUTO DRIVE: OFF");
}

void DashboardPanel::onAudioToggled(bool checked)
{
    if (audio_btn_) audio_btn_->setText(checked ? "ON" : "OFF");
    emit audioMonitorToggled(checked);
}

void DashboardPanel::onTranscriptionUpdated(const QString& text)
{
    transcription_->setPlainText(text);
    transcription_->moveCursor(QTextCursor::End);
}

void DashboardPanel::onMagnetometerUpdated(double x, double y, double z)
{
    mag_x_->setText(QString::number(x, 'f', 2));
    mag_y_->setText(QString::number(y, 'f', 2));
    mag_z_->setText(QString::number(z, 'f', 2));
}

void DashboardPanel::onImuUpdated(double yaw, double pitch, double roll)
{
    imu_yaw_->setText(QString::number(yaw, 'f', 1) + "°");
    imu_pitch_->setText(QString::number(pitch, 'f', 1) + "°");
    imu_roll_->setText(QString::number(roll, 'f', 1) + "°");
}

void DashboardPanel::onClearAll()
{
    for (QLabel* l : {mag_x_, mag_y_, mag_z_, imu_yaw_, imu_pitch_, imu_roll_})
        l->setText("--");
    if (speech_processor_)
        speech_processor_->clearTranscription();
}

void DashboardPanel::publishEstopState()
{
    std_msgs::msg::Bool msg;
    msg.data = estop_active_.load();
    estop_pub_->publish(msg);
}

void DashboardPanel::applySpeechAudioSettings()
{
    auto& S = AppSettings::instance();
    if (speech_processor_) {
        std::lock_guard<std::mutex> lk(S.strings_mutex);
        speech_processor_->setGrammar(S.vosk_grammar);
    }
    // setChecked emits toggled() (→ audioMonitorToggled) only on a real change.
    if (audio_btn_)
        audio_btn_->setChecked(S.audio_start_enabled.load());
}

// ── Arm lifecycle ─────────────────────────────────────────────────────────────

void DashboardPanel::callArmTrigger(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr& cli, const char* what)
{
    if (!cli->service_is_ready()) {
        RCLCPP_WARN(node_->get_logger(),
                    "Arm '%s' requested but the service is not available "
                    "(is the esp32_bridge running?)", what);
        return;
    }
    cli->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    RCLCPP_INFO(node_->get_logger(), "Arm: '%s' requested", what);
}

void DashboardPanel::onArmClicked()    { callArmTrigger(arm_cli_, "arm"); }
void DashboardPanel::onDisarmClicked() { callArmTrigger(arm_disarm_cli_, "disarm"); }

void DashboardPanel::onArmModeToggle()
{
    // Chassis idles the wrist (J5/J6 torque-off); Dexterity restores full 6-DOF.
    if (arm_mode_ == "CHASSIS")
        callArmTrigger(arm_dexterity_cli_, "dexterity mode");
    else
        callArmTrigger(arm_chassis_cli_, "chassis mode");
}

void DashboardPanel::onArmStateUpdated(const QString& state)
{
    arm_state_ = state;
    arm_state_label_->setText(state);

    QString color = "#888";
    if (state == "READY")             color = "#33cc33";
    else if (state == "INITIALIZING") color = "#ccaa00";
    else if (state == "FAULT")        color = "#cc3333";
    arm_state_indicator_->setStyleSheet(QString("color: %1; font-size: 14px;").arg(color));
    arm_state_label_->setStyleSheet(QString("color: %1; font-size: 12px;").arg(color));

    const bool ready = (state == "READY");
    arm_btn_->setEnabled(!ready);                          // can't re-arm when ready
    arm_disarm_btn_->setEnabled(ready || state == "INITIALIZING");
    arm_mode_btn_->setEnabled(ready);                      // mode only meaningful when ready

    // One-shot startup prompt: the arm boots passive by design, so offer to arm
    // it the first time we learn it's connected but disarmed.
    if (!auto_arm_prompted_) {
        auto_arm_prompted_ = true;
        if (state == "UNINIT") {
            auto reply = QMessageBox::question(
                this, "Arm the manipulator?",
                "The arm is connected but disarmed (it boots passive for safety).\n\n"
                "Arm it now? It will then accept joint commands.",
                QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
            if (reply == QMessageBox::Yes)
                callArmTrigger(arm_cli_, "arm");
        }
    }
}

void DashboardPanel::onArmModeUpdated(const QString& mode)
{
    arm_mode_ = mode;
    arm_mode_btn_->setText(mode == "CHASSIS" ? "Mode: Chassis (wrist idle)"
                                             : "Mode: Dexterity");
}

void DashboardPanel::onArmPresenceUpdated(int mask)
{
    // Bits 0..5 = J1..J6 (ODrive J1-3, ZE300 J4, LKTech J5-6). Set when each
    // joint's zero is captured at arm time; 0 before arming.
    for (int j = 0; j < 6; ++j) {
        const bool present = (mask >> j) & 0x1;
        arm_can_dots_[j]->setStyleSheet(
            QString("color: %1; font-size: 10px; font-weight: bold;")
                .arg(present ? "#33cc33" : "#cc3333"));
    }
}

// ── Sensor stack (ZED + RPLidar via the Jetson robot_manager) ────────────────
void DashboardPanel::onSensorsStartClicked()
{
    if (!sensors_start_cli_->service_is_ready()) {
        RCLCPP_WARN(node_->get_logger(),
                    "Sensors start requested but the service is unavailable "
                    "(is robot_manager running on the Jetson?)");
        return;
    }
    sensors_start_cli_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());
    setStackPending(sensors_indicator_, sensors_label_, "activating…");
    RCLCPP_INFO(node_->get_logger(), "Sensors: start requested");
}

void DashboardPanel::onSensorsStopClicked()
{
    if (!sensors_stop_cli_->service_is_ready()) {
        RCLCPP_WARN(node_->get_logger(),
                    "Sensors stop requested but the service is unavailable "
                    "(is robot_manager running on the Jetson?)");
        return;
    }
    sensors_stop_cli_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());
    setStackPending(sensors_indicator_, sensors_label_, "deactivating…");
    RCLCPP_INFO(node_->get_logger(), "Sensors: stop requested");
}

void DashboardPanel::onSensorsStatusUpdated(const QString& status)
{
    // status is "<overall> (zed=… lidar=…)"; the leading word drives the LED.
    sensors_label_->setText(status);
    const QString s = status.section(' ', 0, 0);

    QString color = "#888";          // inactive / unknown
    if (s == "active")           color = "#33cc33";
    else if (s == "activating")  color = "#ccaa00";
    else if (s == "partial")     color = "#ccaa00";
    else if (s == "failed")      color = "#cc3333";
    sensors_indicator_->setStyleSheet(QString("color: %1; font-size: 14px;").arg(color));
    sensors_label_->setStyleSheet(QString("color: %1; font-size: 12px;").arg(color));

    const bool active = (s == "active");
    sensors_start_btn_->setEnabled(!active && s != "activating");
    sensors_stop_btn_->setEnabled(s != "inactive");
}

// ── I2C sensor stack (thermal + magnetometer via robot_manager 'i2c') ────────
void DashboardPanel::onI2cStartClicked()
{
    if (!i2c_start_cli_->service_is_ready()) {
        RCLCPP_WARN(node_->get_logger(),
                    "I2C start requested but the service is unavailable "
                    "(is robot_manager running on the Jetson?)");
        return;
    }
    i2c_start_cli_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    setStackPending(i2c_indicator_, i2c_label_, "activating…");
    RCLCPP_INFO(node_->get_logger(), "I2C sensors: start requested");
}

void DashboardPanel::onI2cStopClicked()
{
    if (!i2c_stop_cli_->service_is_ready()) {
        RCLCPP_WARN(node_->get_logger(),
                    "I2C stop requested but the service is unavailable "
                    "(is robot_manager running on the Jetson?)");
        return;
    }
    i2c_stop_cli_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    setStackPending(i2c_indicator_, i2c_label_, "deactivating…");
    RCLCPP_INFO(node_->get_logger(), "I2C sensors: stop requested");
}

void DashboardPanel::onI2cStatusUpdated(const QString& status)
{
    i2c_label_->setText(status);
    const QString s = status.section(' ', 0, 0);

    QString color = "#888";
    if (s == "active")           color = "#33cc33";
    else if (s == "activating")  color = "#ccaa00";
    else if (s == "partial")     color = "#ccaa00";
    else if (s == "failed")      color = "#cc3333";
    i2c_indicator_->setStyleSheet(QString("color: %1; font-size: 14px;").arg(color));
    i2c_label_->setStyleSheet(QString("color: %1; font-size: 12px;").arg(color));

    const bool active = (s == "active");
    i2c_start_btn_->setEnabled(!active && s != "activating");
    i2c_stop_btn_->setEnabled(s != "inactive");
    // The per-sensor enable toggles only do anything while the driver is running.
    thermal_toggle_->setEnabled(active);
    mag_toggle_->setEnabled(active);
}

// ── Mapping/SLAM stack (robot_manager 'mapping') ─────────────────────────────
void DashboardPanel::onMappingStartClicked()
{
    if (!mapping_start_cli_->service_is_ready()) {
        RCLCPP_WARN(node_->get_logger(),
                    "Mapping start requested but the service is unavailable "
                    "(is robot_manager running on the Jetson?)");
        return;
    }
    mapping_start_cli_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    setStackPending(mapping_indicator_, mapping_label_, "activating…");
    RCLCPP_INFO(node_->get_logger(), "Mapping/SLAM: start requested");
}

void DashboardPanel::onMappingStopClicked()
{
    if (!mapping_stop_cli_->service_is_ready()) {
        RCLCPP_WARN(node_->get_logger(),
                    "Mapping stop requested but the service is unavailable "
                    "(is robot_manager running on the Jetson?)");
        return;
    }
    mapping_stop_cli_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    setStackPending(mapping_indicator_, mapping_label_, "deactivating…");
    RCLCPP_INFO(node_->get_logger(), "Mapping/SLAM: stop requested");
}

void DashboardPanel::onMappingStatusUpdated(const QString& status)
{
    mapping_label_->setText(status);
    const QString s = status.section(' ', 0, 0);

    QString color = "#888";
    if (s == "active")           color = "#33cc33";
    else if (s == "activating")  color = "#ccaa00";
    else if (s == "partial")     color = "#ccaa00";
    else if (s == "failed")      color = "#cc3333";
    mapping_indicator_->setStyleSheet(QString("color: %1; font-size: 14px;").arg(color));
    mapping_label_->setStyleSheet(QString("color: %1; font-size: 12px;").arg(color));

    const bool active = (s == "active");
    mapping_start_btn_->setEnabled(!active && s != "activating");
    mapping_stop_btn_->setEnabled(s != "inactive");
}

void DashboardPanel::onOpenMapClicked()
{
    // Lazily create the standalone map window; just raise it if it exists.
    if (!map_window_)
        map_window_ = new MapWindow(node_, MapWindow::Mode::Map2D);   // top-level
    map_window_->show();
    map_window_->raise();
    map_window_->activateWindow();
}

void DashboardPanel::onOpen3dMapClicked()
{
    if (!map3d_window_)
        map3d_window_ = new MapWindow(node_, MapWindow::Mode::Map3D);   // top-level
    map3d_window_->show();
    map3d_window_->raise();
    map3d_window_->activateWindow();
}

// ── 3-D mapping (OctoMap) stack ──────────────────────────────────────────────
void DashboardPanel::onMapping3dStartClicked()
{
    if (!mapping3d_start_cli_->service_is_ready()) {
        RCLCPP_WARN(node_->get_logger(),
                    "3D mapping start requested but the service is unavailable "
                    "(is robot_manager running on the Jetson?)");
        return;
    }
    mapping3d_start_cli_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    setStackPending(mapping3d_indicator_, mapping3d_label_, "activating…");
    RCLCPP_INFO(node_->get_logger(), "3D mapping: start requested");
}

void DashboardPanel::onMapping3dStopClicked()
{
    if (!mapping3d_stop_cli_->service_is_ready()) {
        RCLCPP_WARN(node_->get_logger(),
                    "3D mapping stop requested but the service is unavailable "
                    "(is robot_manager running on the Jetson?)");
        return;
    }
    mapping3d_stop_cli_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    setStackPending(mapping3d_indicator_, mapping3d_label_, "deactivating…");
    RCLCPP_INFO(node_->get_logger(), "3D mapping: stop requested");
}

void DashboardPanel::onMapping3dStatusUpdated(const QString& status)
{
    mapping3d_label_->setText(status);
    const QString s = status.section(' ', 0, 0);

    QString color = "#888";
    if (s == "active")           color = "#33cc33";
    else if (s == "activating")  color = "#ccaa00";
    else if (s == "partial")     color = "#ccaa00";
    else if (s == "failed")      color = "#cc3333";
    mapping3d_indicator_->setStyleSheet(QString("color: %1; font-size: 14px;").arg(color));
    mapping3d_label_->setStyleSheet(QString("color: %1; font-size: 12px;").arg(color));

    const bool active = (s == "active");
    mapping3d_start_btn_->setEnabled(!active && s != "activating");
    mapping3d_stop_btn_->setEnabled(s != "inactive");
}

void DashboardPanel::stopAllStacks()
{
    // Dependents first (3-D + 2-D mapping consume the sensors), then sensors.
    struct NamedClient {
        const char* name;
        const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr& cli;
    };
    const NamedClient stacks[] = {
        {"mapping3d", mapping3d_stop_cli_},
        {"mapping",   mapping_stop_cli_},
        {"i2c",       i2c_stop_cli_},
        {"sensors",   sensors_stop_cli_},
    };

    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    std::vector<rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture> futures;
    for (const auto& s : stacks) {
        if (s.cli && s.cli->service_is_ready()) {
            futures.push_back(s.cli->async_send_request(req).future.share());
            RCLCPP_INFO(node_->get_logger(), "GUI closing: stop '%s' requested", s.name);
        }
    }
    if (futures.empty()) {
        RCLCPP_INFO(node_->get_logger(),
                    "GUI closing: no robot_manager stacks reachable to stop");
        return;
    }
    // The ROS spin thread is still running during closeEvent, so it delivers the
    // requests and completes these futures. Wait a bounded window so the stops
    // actually go out (and are acked) before the app tears the node down.
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    for (auto& f : futures)
        f.wait_until(deadline);
}
