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

class QTextEdit;
class SpeechProcessor;

// Right-column dashboard: connection/heartbeat + uptime, a magnetometer readout
// with an enable toggle, an orientation readout (from the ZED2 IMU), the software
// e-stop, and the speech transcription panel + audio-monitor toggle.
//
// RoboCorea changes vs legacy: no gas sensor; no ESP32 IMU (orientation comes
// from the ZED2 camera, so it has no enable toggle). The magnetometer and thermal
// camera are Jetson I2C nodes that honor /sensors/enable_mask: bit0 mag, bit1
// thermal (driven by the video panel's thermal selection, not a button). Audio is
// the Opus track demuxed from the C920 A/V stream, not a ROS topic.
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
    void audioMonitorToggled(bool enabled);  // → GstAvStream playback
    void magnetometerUpdated(double x, double y, double z);
    void imuUpdated(double yaw, double pitch, double roll);
    void telemetryReceived();   // heartbeat
    void uptimeUpdated(float uptime_s);
    // Arm lifecycle (ROS thread → Qt thread).
    void armStateUpdated(const QString& state);
    void armModeUpdated(const QString& mode);
    void armPresenceUpdated(int mask);
    // Sensor stack lifecycle (ZED + RPLidar via the Jetson robot_manager).
    void sensorsStatusUpdated(const QString& status);
    // I2C sensor stack lifecycle (thermal + magnetometer).
    void i2cStatusUpdated(const QString& status);
    // Mapping/SLAM stack lifecycle (slam_toolbox + EKF on the Jetson).
    void mappingStatusUpdated(const QString& status);
    // 3-D mapping (OctoMap) stack lifecycle.
    void mapping3dStatusUpdated(const QString& status);

public slots:
    // Called by VideoPanel when any widget selects/deselects the thermal source.
    void setThermalEnabled(bool enabled);

private slots:
    void onEstopToggled(bool checked);
    void onAudioToggled(bool checked);
    void onTranscriptionUpdated(const QString& text);
    void onMagnetometerUpdated(double x, double y, double z);
    void onImuUpdated(double yaw, double pitch, double roll);
    void onTelemetryReceived();
    void onHeartbeatCheck();
    void onUptimeUpdated(float uptime_s);
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
    // Sensor stack
    void onSensorsStatusUpdated(const QString& status);
    void onSensorsStartClicked();
    void onSensorsStopClicked();
    // I2C sensor stack (thermal + magnetometer)
    void onI2cStatusUpdated(const QString& status);
    void onI2cStartClicked();
    void onI2cStopClicked();
    void onThermalToggled();
    // Mapping/SLAM stack
    void onMappingStatusUpdated(const QString& status);
    void onMappingStartClicked();
    void onMappingStopClicked();
    void onOpenMapClicked();
    // 3-D mapping (OctoMap)
    void onMapping3dStatusUpdated(const QString& status);
    void onMapping3dStartClicked();
    void onMapping3dStopClicked();
    void onOpen3dMapClicked();

private:
    void setConnState(const QString& color, const QString& label);
    void publishSensorMask();
    void callArmTrigger(const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr& cli,
                        const char* what);

    rclcpp::Node::SharedPtr node_;

    // Connection status
    QLabel*             conn_indicator_;
    QLabel*             conn_label_;
    QLabel*             uptime_label_;
    QTimer*             heartbeat_timer_;
    QPropertyAnimation* pulse_anim_{nullptr};
    bool                hb_received_{false};
    int                 hb_miss_count_{3};
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr telemetry_sub_;

    // Magnetometer
    QLabel* mag_x_;
    QLabel* mag_y_;
    QLabel* mag_z_;

    // Orientation (from the ZED2 IMU)
    QLabel* imu_yaw_;
    QLabel* imu_pitch_;
    QLabel* imu_roll_;

    // Speech / audio monitor
    QTextEdit*       transcription_{nullptr};
    QPushButton*     audio_btn_{nullptr};
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
    QPushButton* estop_btn_;

    // E-STOP (republished ~10 Hz)
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
    QTimer*           estop_timer_;
    std::atomic<bool> estop_active_{false};

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

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr arm_state_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr arm_mode_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr arm_presence_sub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr arm_cli_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr arm_disarm_cli_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr arm_dexterity_cli_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr arm_chassis_cli_;

    // ── Sensor stack (ZED + RPLidar, started/stopped on the Jetson via systemd) ─
    QLabel*      sensors_indicator_{nullptr};   // colored LED
    QLabel*      sensors_label_{nullptr};       // active / inactive / activating / …
    QPushButton* sensors_start_btn_{nullptr};
    QPushButton* sensors_stop_btn_{nullptr};
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sensors_status_sub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr sensors_start_cli_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr sensors_stop_cli_;

    // ── I2C sensor stack (MLX90640 thermal + LIS3MDL mag, Jetson robot_manager) ─
    // Process lifecycle (start/stop the jetson_sensors driver) + per-sensor
    // runtime enable via /sensors/enable_mask (thermal bit1, mag bit0).
    QPushButton* thermal_toggle_{nullptr};   // /sensors/enable_mask bit1
    QLabel*      i2c_indicator_{nullptr};
    QLabel*      i2c_label_{nullptr};
    QPushButton* i2c_start_btn_{nullptr};
    QPushButton* i2c_stop_btn_{nullptr};
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr i2c_status_sub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr i2c_start_cli_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr i2c_stop_cli_;

    // ── Mapping/SLAM stack (slam_toolbox + EKF on the Jetson) ─────────────────
    QLabel*      mapping_indicator_{nullptr};
    QLabel*      mapping_label_{nullptr};
    QPushButton* mapping_start_btn_{nullptr};
    QPushButton* mapping_stop_btn_{nullptr};
    QPushButton* open_map_btn_{nullptr};     // opens the local 2D map window
    QWidget*     map_window_{nullptr};       // MapWindow (created lazily)
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mapping_status_sub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr mapping_start_cli_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr mapping_stop_cli_;

    // ── 3-D mapping (OctoMap) stack ──────────────────────────────────────────
    QLabel*      mapping3d_indicator_{nullptr};
    QLabel*      mapping3d_label_{nullptr};
    QPushButton* mapping3d_start_btn_{nullptr};
    QPushButton* mapping3d_stop_btn_{nullptr};
    QPushButton* open_3dmap_btn_{nullptr};
    QWidget*     map3d_window_{nullptr};     // 3-D MapWindow (created lazily)
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mapping3d_status_sub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr mapping3d_start_cli_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr mapping3d_stop_cli_;
};
