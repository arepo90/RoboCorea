#pragma once

#include <QFrame>
#include <QLabel>
#include <QPropertyAnimation>
#include <QPushButton>
#include <QTimer>
#include <QWidget>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <atomic>

class QTabWidget;
class QTextEdit;
class MagPlot;
class SpeechProcessor;

// Right-column dashboard: connection/heartbeat + uptime, a magnetometer readout
// with an enable toggle, an orientation readout (from the ZED2 IMU), the software
// e-stop, and the speech transcription panel + audio-monitor toggle.
//
// RoboCorea changes vs legacy: no gas sensor; no ESP32 IMU (orientation comes
// from the ZED2 camera, so it has no enable toggle). The magnetometer and thermal
// camera now live on the arm PCB (read by the arm ESP32, relayed by esp32_bridge)
// and honor /sensors/enable_mask: bit0 mag, bit1 thermal (driven by the video
// panel's thermal selection, not a button). The mask is unchanged — the bridge
// forwards it down as MSG_SENSOR_ENABLE. Audio is the Opus track demuxed from the
// C920 A/V stream, not a ROS topic.
class DashboardPanel : public QWidget {
    Q_OBJECT
public:
    explicit DashboardPanel(rclcpp::Node::SharedPtr node, QWidget* parent = nullptr);

    // Vosk transcriber owned by the dashboard; MainWindow routes the A/V stream's
    // decoded PCM into it via SpeechProcessor::pushAudio().
    SpeechProcessor* speechProcessor() const { return speech_processor_; }

    // Re-read the Vosk grammar + audio-monitor preference from AppSettings and
    // apply them to the live speech processor / audio toggle. Called by
    // MainWindow after the settings dialog applies/resets.
    void applySpeechAudioSettings();

signals:
    void resetSourcesRequested();
    void settingsRequested();
    void systemsRequested();   // open the Robot Systems window (toolbar icon)
    void audioMonitorToggled(bool enabled);  // → GstAvStream playback (robot mic → speaker)
    void talkbackToggled(bool enabled);      // → GstMicSender (operator mic → robot speaker)
    void magnetometerUpdated(double x, double y, double z);
    void imuUpdated(double yaw, double pitch, double roll);
    void telemetryReceived();   // heartbeat
    void uptimeUpdated(float uptime_s);
    void batteryUpdated(double volts);   // pack voltage (any VESC V_in)
    // Arm lifecycle (ROS thread → Qt thread).
    void armStateUpdated(const QString& state);
    void armModeUpdated(const QString& mode);
    void armPresenceUpdated(int mask);
    // Autonomy drive state from the bridge (ROS thread → Qt thread).
    void autonomyStateUpdated(bool enabled);

public slots:
    // Called by VideoPanel when any widget selects/deselects the thermal source.
    void setThermalEnabled(bool enabled);

private slots:
    void onEstopToggled(bool checked);
    void onAutonomyToggled(bool checked);
    void onAutonomyStateUpdated(bool enabled);
    void onAudioToggled(bool checked);
    void onTalkbackToggled(bool checked);
    void onTranscriptionUpdated(const QString& text);
    void onMagnetometerUpdated(double x, double y, double z);
    void onImuUpdated(double yaw, double pitch, double roll);
    void onTelemetryReceived();
    void onHeartbeatCheck();
    void onUptimeUpdated(float uptime_s);
    void onBatteryUpdated(double volts);
    void onClearAll();
    void publishEstopState();
    void onSensorToggled();
    // Arm lifecycle
    void onArmStateUpdated(const QString& state);
    void onArmModeUpdated(const QString& mode);
    void onArmPresenceUpdated(int mask);
    void onArmClicked();
    void onDisarmClicked();
    void onArmModeToggle();
    // Thermal acquisition enable (/sensors/enable_mask bit1).
    void onThermalToggled();

private:
    void setConnState(const QString& color, const QString& label);
    void publishSensorMask();
    void callArmTrigger(const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr& cli,
                        const char* what);
    // Recolor the per-joint CAN dots from the latest presence mask, but only
    // treat them as meaningful when the link is live AND the arm is up — so a
    // stale latched "all present" (or a dropped link) never shows misleading green.
    void updateArmCanDots();

    rclcpp::Node::SharedPtr node_;

    // Connection status
    QLabel*             conn_indicator_;
    QLabel*             conn_label_;
    QLabel*             uptime_label_;
    QLabel*             battery_label_;   // 6S LiPo pack voltage, color-coded
    QTimer*             heartbeat_timer_;
    QPropertyAnimation* pulse_anim_{nullptr};
    bool                hb_received_{false};
    int                 hb_miss_count_{3};
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr telemetry_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr vesc_sub_;

    // Magnetometer — numeric readout (Values tab) + live strip chart (Graph tab)
    QTabWidget* mag_tabs_{nullptr};
    QLabel*     mag_x_;
    QLabel*     mag_y_;
    QLabel*     mag_z_;
    MagPlot*    mag_plot_{nullptr};

    // Orientation (from the ZED2 IMU)
    QLabel* imu_yaw_;
    QLabel* imu_pitch_;
    QLabel* imu_roll_;

    // Speech / audio monitor
    QTextEdit*       transcription_{nullptr};
    QPushButton*     audio_btn_{nullptr};   // robot mic → laptop speaker + transcription
    QPushButton*     talk_btn_{nullptr};    // laptop mic → robot speaker (talkback)
    SpeechProcessor* speech_processor_{nullptr};

    rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    // Sensor enable mask toggles (bit0=mag, bit1=thermal driven by the video panel).
    QPushButton* mag_toggle_;
    uint8_t      sensor_mask_{0};
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr sensor_mask_pub_;

    // Controls
    QPushButton* clear_btn_;
    QPushButton* reset_btn_;
    QPushButton* settings_btn_;
    QPushButton* systems_btn_;   // opens the Robot Systems window
    QPushButton* estop_btn_;

    // E-STOP (republished ~10 Hz)
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
    QTimer*           estop_timer_;
    std::atomic<bool> estop_active_{false};

    // ── Autonomy drive (/cmd_vel → tracks) runtime allow/prevent ──────────────
    // Extra safety layer on top of the firmware's RC-stick-neutral arbitration: the
    // button gates whether Nav2's /cmd_vel can move the tracks. The bridge latches it
    // OFF when the operator touches a drive stick or virtual-flip, so the button is
    // driven by /autonomy/state (authoritative), not just local clicks.
    QPushButton* autonomy_btn_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr autonomy_enable_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr autonomy_state_sub_;

    // ── Arm lifecycle (boots disarmed; arm/disarm + dexterity/chassis idle) ───
    QLabel*      arm_state_indicator_;   // colored LED
    QLabel*      arm_state_label_;       // UNINIT / INITIALIZING / READY / FAULT
    QPushButton* arm_btn_;               // request arm/init
    QPushButton* arm_disarm_btn_;        // request disarm
    QPushButton* arm_mode_btn_;          // Dexterity ⇄ Chassis (idle J5/J6)
    QLabel*      arm_can_dots_[6];       // per-joint CAN presence (J1..J6)
    QString      arm_state_{"UNINIT"};
    QString      arm_mode_{"DEXTERITY"};
    bool         auto_arm_prompted_{false};   // one-shot startup arm prompt
    int          arm_presence_mask_{0};       // latest /arm/can_presence bits
    bool         link_online_{false};         // chassis heartbeat fresh (gates the dots)

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr arm_state_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr arm_mode_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr arm_presence_sub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr arm_cli_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr arm_disarm_cli_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr arm_dexterity_cli_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr arm_chassis_cli_;

    // ── Thermal acquisition enable (/sensors/enable_mask bit1) ────────────────
    // The dashboard owns the enable mask; esp32_bridge relays it to the arm PCB
    // (MSG_SENSOR_ENABLE). Also auto-driven by the video panel's thermal source
    // selection (setThermalEnabled).
    QPushButton* thermal_toggle_{nullptr};   // /sensors/enable_mask bit1
};
