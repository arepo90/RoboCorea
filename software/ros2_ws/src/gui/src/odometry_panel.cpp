#include "gui/odometry_panel.hpp"

#include <QGridLayout>
#include <QHBoxLayout>

#include <cmath>

// VESC table rows. The real CAN IDs (firmware config.h / architecture.md §8.1)
// are NOT contiguous 1..6 — traction L/R = 60/50, flippers FL/FR/RL/RR =
// 20/10/40/30 — so an incoming status frame must be mapped to a fixed row slot
// by its CAN id (vescIdToSlot below), NOT used as a row index directly. Slot
// order: 1=TL 2=TR 3=FL 4=FR 5=RL 6=RR.
struct VescSlot { const char* name; int can_id; };
static const VescSlot VESC_SLOTS[7] = {
    {"?", -1},                                              // slot 0 unused
    {"TL", 60}, {"TR", 50}, {"FL", 20}, {"FR", 10}, {"RL", 40}, {"RR", 30},
};

// Map a raw VESC CAN id (as forwarded in /motors/vesc_status[0]) to its table
// row slot 1..6, or 0 if it isn't one of the six known VESCs.
static int vescIdToSlot(int can_id) {
    for (int s = 1; s <= 6; ++s)
        if (VESC_SLOTS[s].can_id == can_id) return s;
    return 0;
}

static const char* ARM_NAMES[7]  = {"?", "J1", "J2", "J3", "J4", "J5", "J6"};

// ODrive J1–J3 report RAW MOTOR turns over CAN (firmware OdriveStatusPayload.
// pos_turns_100); the reduction + sign are NOT applied there. Convert to true
// output degrees here so the readout matches the commanded joint angle, mirroring
// the firmware command path (config.h ODRIVE_GEAR_J*/ODRIVE_DIR_J*). J4/J5/J6
// already arrive as output degrees from the bridge. (ZE300 is firmware-zeroed;
// ODrive/LKTech are not, so onArmJointUpdated re-zeros all six at arm time.)
static constexpr double ODRIVE_GEAR = 48.0;
static constexpr double ODRIVE_DIR  = -1.0;

OdometryPanel::OdometryPanel(rclcpp::Node::SharedPtr node, QWidget* parent)
    : QWidget(parent), node_(node)
{
    setStyleSheet("background-color: #1e1e2e;");
    buildLayout();

    connect(this, &OdometryPanel::tracksUpdated,
            this, &OdometryPanel::onTracksUpdated, Qt::QueuedConnection);
    connect(this, &OdometryPanel::wheelOdomUpdated,
            this, &OdometryPanel::onWheelOdomUpdated, Qt::QueuedConnection);
    connect(this, &OdometryPanel::flipperExtUpdated,
            this, &OdometryPanel::onFlipperExtUpdated, Qt::QueuedConnection);
    connect(this, &OdometryPanel::modeUpdated,
            this, &OdometryPanel::onModeUpdated, Qt::QueuedConnection);
    connect(this, &OdometryPanel::flagsUpdated,
            this, &OdometryPanel::onFlagsUpdated, Qt::QueuedConnection);
    connect(this, &OdometryPanel::vescStatusUpdated,
            this, &OdometryPanel::onVescStatusUpdated, Qt::QueuedConnection);
    connect(this, &OdometryPanel::armJointUpdated,
            this, &OdometryPanel::onArmJointUpdated, Qt::QueuedConnection);
    connect(this, &OdometryPanel::armStateUpdated,
            this, &OdometryPanel::onArmStateUpdated, Qt::QueuedConnection);

    arm_need_zero_.fill(true);   // capture each joint's display zero on first sample

    auto qos = rclcpp::QoS(10).best_effort();

    tracks_sub_ = node_->create_subscription<geometry_msgs::msg::Vector3>(
        "/encoders/tracks", qos,
        [this](geometry_msgs::msg::Vector3::SharedPtr msg) {
            emit tracksUpdated(msg->x, msg->y);
        });

    // Track (wheel) odometry integrated from the VESC tachometers on the bridge.
    wheel_odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/wheel", qos,
        [this](nav_msgs::msg::Odometry::SharedPtr msg) {
            const auto& q = msg->pose.pose.orientation;
            double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                    1.0 - 2.0 * (q.y * q.y + q.z * q.z)) * 180.0 / M_PI;
            emit wheelOdomUpdated(msg->pose.pose.position.x, msg->pose.pose.position.y,
                                  yaw, msg->twist.twist.linear.x);
        });

    flipper_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/encoders/flipper", qos,
        [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            if (msg->data.size() >= 4)
                emit flipperExtUpdated(msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
        });

    mode_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/robot/mode", qos,
        [this](std_msgs::msg::String::SharedPtr msg) {
            emit modeUpdated(QString::fromStdString(msg->data));
        });

    flags_sub_ = node_->create_subscription<std_msgs::msg::UInt8>(
        "/robot/flags", qos,
        [this](std_msgs::msg::UInt8::SharedPtr msg) {
            emit flagsUpdated(static_cast<int>(msg->data));
        });

    // VESC: [id, erpm, current_A, duty, temp_fet, temp_motor, voltage, tacho].
    // Temperatures (data[4]/data[5]) are intentionally not surfaced here.
    vesc_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/motors/vesc_status", qos,
        [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            if (msg->data.size() >= 7)
                emit vescStatusUpdated(static_cast<int>(msg->data[0]),
                                       msg->data[1], msg->data[2], msg->data[3]);
        });

    // Arm telemetry → unified armJointUpdated(joint 1-6, output°, current_A). The
    // angle is the gear-reduced, re-zeroed output angle (see onArmJointUpdated);
    // the secondary readout is the motor current (Iq), not temperature.
    // ODrive J1-3: [joint, pos_turns(motor), vel, iq_A, busV, busA]
    odrive_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/motors/odrive_status", qos,
        [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            if (msg->data.size() >= 6) {
                int joint = static_cast<int>(msg->data[0]) + 1;  // 0-based → J1..J3
                double out_deg = msg->data[1] * 360.0 / ODRIVE_GEAR * ODRIVE_DIR;
                emit armJointUpdated(joint, out_deg, msg->data[3]);
            }
        });
    // ZE300 J4: [id, temp_C, iq_A, rpm, single_turn, pos_counts, out_deg]
    ze300_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/motors/ze300_status", qos,
        [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            if (msg->data.size() >= 7)
                emit armJointUpdated(4, msg->data[6], msg->data[2]);
        });
    // LKTech J5-6: [joint, motor_id, temp_C, iq_A, dps, angle, out_deg]
    lktech_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/motors/lktech_status", qos,
        [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            if (msg->data.size() >= 7) {
                int motor_id = static_cast<int>(msg->data[1]);
                int joint = (motor_id == 15) ? 6 : 5;  // 14→J5, 15→J6
                emit armJointUpdated(joint, msg->data[6], msg->data[3]);
            }
        });

    // Re-zero the displayed angles whenever the arm (re)arms: the firmware captures
    // each joint's boot-pose zero at arm time, so READY is when "0 = current pose".
    arm_state_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/arm/state", rclcpp::QoS(1).reliable().transient_local(),
        [this](std_msgs::msg::String::SharedPtr msg) {
            emit armStateUpdated(QString::fromStdString(msg->data));
        });
}

void OdometryPanel::buildLayout()
{
    main_layout_ = new QVBoxLayout(this);
    main_layout_->setContentsMargins(8, 6, 8, 6);
    main_layout_->setSpacing(4);

    // ── Status ───────────────────────────────────────────────────────────────
    main_layout_->addWidget(makeHeaderLabel("Telemetry Status"));
    auto* status_grid = new QGridLayout();
    status_grid->setSpacing(3);
    status_grid->addWidget(makeAxisLabel("Mode"), 0, 0);
    mode_label_ = makeValueLabel("--");
    status_grid->addWidget(mode_label_, 0, 1);
    status_grid->addWidget(makeAxisLabel("Flags"), 1, 0);
    flags_label_ = makeValueLabel("--");
    status_grid->addWidget(flags_label_, 1, 1);
    main_layout_->addLayout(status_grid);

    main_layout_->addWidget(makeHSep());

    // ── Traction ─────────────────────────────────────────────────────────────
    main_layout_->addWidget(makeHeaderLabel("Traction (RPM)"));
    auto* trac_row = new QHBoxLayout();
    trac_row->setSpacing(8);
    {
        auto* c = new QVBoxLayout(); c->setSpacing(1);
        c->addWidget(makeAxisLabel("Left"));
        trac_left_rpm_ = makeValueLabel("--");
        c->addWidget(trac_left_rpm_);
        trac_row->addLayout(c);
    }
    {
        auto* c = new QVBoxLayout(); c->setSpacing(1);
        c->addWidget(makeAxisLabel("Right"));
        trac_right_rpm_ = makeValueLabel("--");
        c->addWidget(trac_right_rpm_);
        trac_row->addLayout(c);
    }
    main_layout_->addLayout(trac_row);

    main_layout_->addWidget(makeHSep());

    // ── Track odometry (wheel, from the VESC tachometers) ─────────────────────
    main_layout_->addWidget(makeHeaderLabel("Track Odometry"));
    {
        auto* grid = new QGridLayout();
        grid->setSpacing(3);
        grid->addWidget(makeAxisLabel("X (m)"), 0, 0);
        odom_x_ = makeValueLabel("--"); grid->addWidget(odom_x_, 0, 1);
        grid->addWidget(makeAxisLabel("Y (m)"), 0, 2);
        odom_y_ = makeValueLabel("--"); grid->addWidget(odom_y_, 0, 3);
        grid->addWidget(makeAxisLabel("Yaw"), 1, 0);
        odom_yaw_ = makeValueLabel("--"); grid->addWidget(odom_yaw_, 1, 1);
        grid->addWidget(makeAxisLabel("vx (m/s)"), 1, 2);
        odom_vx_ = makeValueLabel("--"); grid->addWidget(odom_vx_, 1, 3);
        main_layout_->addLayout(grid);
    }

    main_layout_->addWidget(makeHSep());

    // ── Flippers ─────────────────────────────────────────────────────────────
    main_layout_->addWidget(makeHeaderLabel("Flippers"));
    {
        auto* grid = new QGridLayout();
        grid->setSpacing(3);
        grid->addWidget(makeAxisLabel("FL"), 0, 0);
        flip_fl_ = makeValueLabel("--"); grid->addWidget(flip_fl_, 0, 1);
        grid->addWidget(makeAxisLabel("FR"), 0, 2);
        flip_fr_ = makeValueLabel("--"); grid->addWidget(flip_fr_, 0, 3);
        grid->addWidget(makeAxisLabel("RL"), 1, 0);
        flip_rl_ = makeValueLabel("--"); grid->addWidget(flip_rl_, 1, 1);
        grid->addWidget(makeAxisLabel("RR"), 1, 2);
        flip_rr_ = makeValueLabel("--"); grid->addWidget(flip_rr_, 1, 3);
        main_layout_->addLayout(grid);
    }

    main_layout_->addWidget(makeHSep());

    // ── VESC telemetry table ─────────────────────────────────────────────────
    main_layout_->addWidget(makeHeaderLabel("Motor Telemetry (VESC)"));
    {
        auto* hdr = new QGridLayout();
        hdr->setSpacing(2);
        hdr->addWidget(makeAxisLabel("ID"),   0, 0);
        hdr->addWidget(makeAxisLabel("eRPM"), 0, 1);
        hdr->addWidget(makeAxisLabel("A"),    0, 2);
        hdr->addWidget(makeAxisLabel("Duty"), 0, 3);
        main_layout_->addLayout(hdr);

        for (int id = 1; id <= 6; ++id) {
            auto* row = new QGridLayout();
            row->setSpacing(2);
            // Label the row with the friendly name + the real CAN id (e.g. "TL·60").
            row->addWidget(makeAxisLabel(
                QString("%1·%2").arg(VESC_SLOTS[id].name).arg(VESC_SLOTS[id].can_id)), 0, 0);
            vesc_rows_[id].erpm       = makeValueLabel("--");
            vesc_rows_[id].current    = makeValueLabel("--");
            vesc_rows_[id].duty       = makeValueLabel("--");
            for (QLabel* l : {vesc_rows_[id].erpm, vesc_rows_[id].current,
                              vesc_rows_[id].duty})
                l->setStyleSheet("color: #4fc3f7; font-size: 10px;");
            row->addWidget(vesc_rows_[id].erpm,       0, 1);
            row->addWidget(vesc_rows_[id].current,    0, 2);
            row->addWidget(vesc_rows_[id].duty,       0, 3);
            main_layout_->addLayout(row);
        }
    }

    main_layout_->addWidget(makeHSep());

    // ── Arm telemetry ────────────────────────────────────────────────────────
    main_layout_->addWidget(makeHeaderLabel("Arm (ODrive / ZE300 / LKTech)"));
    {
        auto* hdr = new QGridLayout();
        hdr->setSpacing(2);
        hdr->addWidget(makeAxisLabel("Joint"), 0, 0);
        hdr->addWidget(makeAxisLabel("Angle"), 0, 1);
        hdr->addWidget(makeAxisLabel("Iq (A)"), 0, 2);
        main_layout_->addLayout(hdr);

        for (int j = 1; j <= 6; ++j) {
            auto* row = new QGridLayout();
            row->setSpacing(2);
            row->addWidget(makeAxisLabel(ARM_NAMES[j]), 0, 0);
            arm_rows_[j].angle   = makeValueLabel("--");
            arm_rows_[j].current = makeValueLabel("--");
            for (QLabel* l : {arm_rows_[j].angle, arm_rows_[j].current})
                l->setStyleSheet("color: #4fc3f7; font-size: 10px;");
            row->addWidget(arm_rows_[j].angle,   0, 1);
            row->addWidget(arm_rows_[j].current, 0, 2);
            main_layout_->addLayout(row);
        }
    }

    main_layout_->addStretch();
}

QLabel* OdometryPanel::makeValueLabel(const QString& initial)
{
    auto* l = new QLabel(initial, this);
    l->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
    l->setStyleSheet("color: #4fc3f7; font-size: 13px;");
    return l;
}

QLabel* OdometryPanel::makeHeaderLabel(const QString& text)
{
    auto* l = new QLabel(text, this);
    l->setAlignment(Qt::AlignHCenter);
    l->setStyleSheet("color: #aaa; font-weight: bold;");
    return l;
}

QLabel* OdometryPanel::makeAxisLabel(const QString& text)
{
    auto* l = new QLabel(text, this);
    l->setAlignment(Qt::AlignHCenter);
    l->setStyleSheet("color: #888;");
    return l;
}

QFrame* OdometryPanel::makeHSep()
{
    auto* s = new QFrame(this);
    s->setFrameShape(QFrame::HLine);
    s->setStyleSheet("color: #444;");
    return s;
}

void OdometryPanel::onTracksUpdated(double left_rpm, double right_rpm)
{
    trac_left_rpm_->setText(QString::number(left_rpm, 'f', 1));
    trac_right_rpm_->setText(QString::number(right_rpm, 'f', 1));
}

void OdometryPanel::onWheelOdomUpdated(double x_m, double y_m, double yaw_deg, double vx)
{
    odom_x_->setText(QString::number(x_m, 'f', 2));
    odom_y_->setText(QString::number(y_m, 'f', 2));
    odom_yaw_->setText(QString::number(yaw_deg, 'f', 1) + "°");
    odom_vx_->setText(QString::number(vx, 'f', 2));
}

void OdometryPanel::onFlipperExtUpdated(float fl, float fr, float rl, float rr)
{
    flip_fl_->setText(QString::number(fl, 'f', 1) + "°");
    flip_fr_->setText(QString::number(fr, 'f', 1) + "°");
    flip_rl_->setText(QString::number(rl, 'f', 1) + "°");
    flip_rr_->setText(QString::number(rr, 'f', 1) + "°");
}

void OdometryPanel::onModeUpdated(const QString& mode)
{
    mode_label_->setText(mode);
    if (mode == "ESTOP")
        mode_label_->setStyleSheet("color: #cc3333; font-size: 12px; font-weight: bold;");
    else if (mode == "STANDBY")
        mode_label_->setStyleSheet("color: #ccaa00; font-size: 12px;");
    else if (mode == "NORMAL" || mode == "FLIPPER")
        mode_label_->setStyleSheet("color: #33cc33; font-size: 12px;");
    else
        mode_label_->setStyleSheet("color: #4fc3f7; font-size: 12px;");
}

void OdometryPanel::onFlagsUpdated(int flags)
{
    QStringList parts;
    if (flags & 0x01) parts << "PPM";
    if (flags & 0x02) parts << "SENS";
    if (flags & 0x04) parts << "CAN";
    if (flags & 0x08) parts << "ESTOP";
    if (flags & 0x10) parts << "REVERSE";   // Ch6-up virtual flip (drive from other end)
    flags_label_->setText(parts.isEmpty() ? "none" : parts.join(" | "));
}

void OdometryPanel::onVescStatusUpdated(int id, float erpm, float current, float duty)
{
    // `id` is the raw VESC CAN id (60/50/20/10/40/30), not a 1..6 row index — map
    // it to the right table slot. Unknown ids (not one of the six VESCs) are dropped.
    const int slot = vescIdToSlot(id);
    if (slot == 0) return;
    auto& row = vesc_rows_[slot];
    row.erpm->setText(QString::number(static_cast<int>(erpm)));
    row.current->setText(QString::number(current, 'f', 1) + "A");
    row.duty->setText(QString::number(duty * 100.0f, 'f', 0) + "%");
}

void OdometryPanel::onArmJointUpdated(int joint, double angle_deg, double current_a)
{
    if (joint < 1 || joint > 6) return;

    double angle = angle_deg;
    // J5/J6 (LKTech) report single-turn motor angle ÷ 10:1 gear, wrapping every 36°
    // of output. Unwrap incrementally (samples are dense while servoing) so the
    // readout follows the full ±90° travel. ODrive/ZE300 are already multi-turn.
    if (joint == 5 || joint == 6) {
        constexpr double PERIOD = 360.0 / 10.0;   // output-deg per motor revolution
        if (arm_unwrap_started_[joint]) {
            double d = angle - arm_prev_raw_[joint];
            if (d >  PERIOD / 2.0)      arm_unwrap_accum_[joint] -= PERIOD;
            else if (d < -PERIOD / 2.0) arm_unwrap_accum_[joint] += PERIOD;
        } else {
            arm_unwrap_started_[joint] = true;
        }
        arm_prev_raw_[joint] = angle;
        angle += arm_unwrap_accum_[joint];
    }

    // ODrive/LKTech telemetry isn't zeroed in firmware (and ODrive arrives in motor
    // turns, gear-converted before this point), so snapshot a display zero on the
    // first sample / each arm so the angle reads ~0 at the captured boot pose.
    if (arm_need_zero_[joint]) {
        arm_zero_[joint] = angle;
        arm_need_zero_[joint] = false;
    }
    arm_rows_[joint].angle->setText(QString::number(angle - arm_zero_[joint], 'f', 1) + "°");
    arm_rows_[joint].current->setText(QString::number(current_a, 'f', 1) + "A");
}

void OdometryPanel::onArmStateUpdated(const QString& state)
{
    // On (re)arm the firmware recaptures each joint's boot-pose zero; recapture the
    // display zeros (and reset the J5/J6 unwrap) so the readout snaps back to 0 at
    // the new ready pose.
    if (state == "READY") {
        arm_need_zero_.fill(true);
        arm_unwrap_started_.fill(false);
        arm_unwrap_accum_.fill(0.0);
    }
}
