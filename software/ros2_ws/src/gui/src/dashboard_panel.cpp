#include "gui/dashboard_panel.hpp"
#include "gui/speech_processor.hpp"
#include "gui/app_settings.hpp"

#include <QFont>
#include <QGraphicsOpacityEffect>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QIcon>
#include <QMessageBox>
#include <QSignalBlocker>
#include <QTextEdit>
#include <QVBoxLayout>

#include <cmath>

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
    // Battery: 6S LiPo pack voltage, sourced from any VESC's V_in (everything
    // shares the one pack). Color-coded by per-cell voltage in onBatteryUpdated.
    battery_label_ = new QLabel("--", this);
    battery_label_->setToolTip("Battery pack voltage (6S LiPo, from VESC V_in)");
    battery_label_->setStyleSheet("color: #4fc3f7; font-size: 12px;");
    auto* conn_vsep = new QFrame(this);
    conn_vsep->setFrameShape(QFrame::VLine);
    conn_vsep->setStyleSheet("color: #444;");
    conn_row->addStretch();
    conn_row->addWidget(conn_indicator_);
    conn_row->addWidget(conn_label_);
    conn_row->addWidget(uptime_label_);
    conn_row->addWidget(conn_vsep);
    conn_row->addWidget(battery_label_);
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

    // ── Thermal acquisition enable ────────────────────────────────────────────
    // The dashboard owns /sensors/enable_mask (bit0 mag, bit1 thermal); the I2C
    // *driver* lifecycle moved to the Robot Systems window, but these per-sensor
    // runtime enables stay here with the readouts. The mask is latched, so a
    // toggle takes effect whenever the jetson_sensors driver is (re)started.
    {
        auto* row = new QHBoxLayout();
        row->setSpacing(4);
        auto* thermal_lbl = new QLabel("Thermal acquisition", this);
        thermal_lbl->setStyleSheet(lbl_style);
        thermal_toggle_ = make_sensor_toggle();
        thermal_toggle_->setToolTip("Enable thermal acquisition (enable_mask bit 1). "
                                    "Also auto-enabled when the thermal video source is selected.");
        connect(thermal_toggle_, &QPushButton::toggled, this, &DashboardPanel::onThermalToggled);
        row->addWidget(thermal_lbl, 1);
        row->addWidget(thermal_toggle_);
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
    connect(this, &DashboardPanel::batteryUpdated,
            this, &DashboardPanel::onBatteryUpdated, Qt::QueuedConnection);

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

    // Battery: any VESC's V_in (index 6) — they all sit on the one 6S pack.
    // [id, erpm, current_A, duty, temp_fet, temp_motor, voltage, tacho]. Ignore
    // obviously-invalid readings (a VESC that hasn't reported V_in yet sends 0).
    vesc_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/motors/vesc_status", sensor_qos,
        [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            if (msg->data.size() >= 7 && msg->data[6] > 6.0f)
                emit batteryUpdated(msg->data[6]);
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

    // (Perception/mapping stack controls + the live map windows moved to the
    // Robot Systems window — opened with the toolbar icon below.)

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

    // Robot Systems window (perception stacks, maps, autonomous arm routines).
    systems_btn_ = new QPushButton(this);
    systems_btn_->setFixedSize(28, 28);
    systems_btn_->setToolTip("Robot Systems (sensors, mapping, maps, arm routines)");
    {
        QIcon icon = QIcon::fromTheme("applications-system");
        if (!icon.isNull()) {
            systems_btn_->setIcon(icon);
            systems_btn_->setIconSize(QSize(16, 16));
        } else {
            systems_btn_->setText("⊞");
        }
    }
    systems_btn_->setStyleSheet(
        "QPushButton { background-color: #2d2d45; color: #ccc; padding: 2px; "
        "border: 1px solid #3a3a55; border-radius: 3px; }"
        "QPushButton:hover { background-color: #3a3a55; }");
    connect(systems_btn_, &QPushButton::clicked, this,
            [this]() { emit systemsRequested(); });
    btn_row->addWidget(systems_btn_);

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

void DashboardPanel::onBatteryUpdated(double volts)
{
    // 6S LiPo: color by per-cell voltage. ~4.2 V/cell full (25.2 V), 3.7 nominal
    // (22.2 V), ~3.3-3.5 should-land-now, 3.0 hard floor (18 V). Thresholds are
    // per-cell so they read the same regardless of the 6S pack arithmetic.
    const double cell = volts / 6.0;
    QString color;
    if      (cell >= 3.80) color = "#33cc33";   // green  — healthy
    else if (cell >= 3.60) color = "#cccc33";   // yellow — getting there
    else if (cell >= 3.40) color = "#ff8800";   // orange — land soon
    else                   color = "#cc3333";   // red    — critical
    battery_label_->setText(QString::number(volts, 'f', 1) + "V");
    battery_label_->setStyleSheet(
        QString("color: %1; font-size: 12px; font-weight: bold;").arg(color));
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
        battery_label_->setText("--");
        battery_label_->setStyleSheet("color: #4fc3f7; font-size: 12px;");
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
    // Enabling thermal makes the robot publish /sensors/thermal; re-scan sources so
    // it shows up as a selectable video source without a manual "Reset Sources".
    // (Source discovery preserves each cell's current selection, so this is safe.)
    if (thermal_toggle_ && thermal_toggle_->isChecked())
        emit resetSourcesRequested();
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
