#pragma once

#include <QLabel>
#include <QPushButton>
#include <QVector>
#include <QWidget>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

// Perception / mapping / navigation stack controls — moved out of the (cluttered)
// dashboard into the Robot Systems window. Manages the robot_manager-driven stacks
// on the Jetson (start/stop over std_srvs/Trigger, live LED from
// /robot/<name>/status):
//
//   sensors      ZED + RPLidar             (rescue-sensors.target)
//   mapping      slam_toolbox + EKF        (rescue-mapping.service)    + Open 2D map
//   mapping3d    OctoMap                   (rescue-mapping3d.service)  + Open 3D map
//   localization AMCL + map_server + EKF   (rescue-localization.service)
//   navigation   Nav2 (planner/ctrl/BT)    (rescue-navigation.service)
//
// The Navigation start is gated by nav_preflight readiness (/nav/preflight): the
// critical scan/odom/tf/map checks must pass before Nav2 will be started.
//
// The MLX90640 + LIS3MDL no longer have a Jetson 'i2c' stack — they moved to the
// arm PCB (relayed by esp32_bridge). Their per-sensor enable toggles
// (mag/thermal /sensors/enable_mask) stay on the dashboard with their readouts.
class MapWindow;

class PerceptionPanel : public QWidget {
    Q_OBJECT
public:
    explicit PerceptionPanel(rclcpp::Node::SharedPtr node, QWidget* parent = nullptr);

    // Request a stop of every managed stack (dependents first: localization,
    // mapping3d, mapping, sensors). Called from MainWindow::closeEvent (via SystemsWindow) so
    // closing the GUI tears the robot's perception stacks down cleanly. Blocks
    // briefly (best-effort) so the requests are delivered before shutdown.
    void stopAllStacks();

signals:
    // ROS thread → Qt thread: (stack key, status string).
    void stackStatusReceived(const QString& key, const QString& status);
    // ROS thread → Qt thread: nav_preflight readiness ("<overall> (k=v …)").
    void preflightReceived(const QString& status);

private slots:
    void onStartClicked(const QString& key);
    void onStopClicked(const QString& key);
    void onStackStatus(const QString& key, const QString& status);
    void onPreflight(const QString& status);   // /nav/preflight readout
    void onOpenMap();      // 2-D MapWindow (lazy)
    void onOpen3dMap();    // 3-D MapWindow (lazy)

private:
    struct Stack {
        QString      key;
        QLabel*      indicator{nullptr};
        QLabel*      label{nullptr};
        QPushButton* start_btn{nullptr};
        QPushButton* stop_btn{nullptr};
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr   start_cli;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr   stop_cli;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub;
    };

    // Build one stack section (header + LED + start/stop, plus an optional extra
    // button such as "Open Map"); wires its Trigger clients + status subscription.
    Stack* addStack(class QVBoxLayout* layout, const QString& key, const QString& title,
                    const QString& start_text, const QString& start_tip,
                    const QString& stop_tip, QPushButton* extra_btn = nullptr);
    Stack* stack(const QString& key);

    rclcpp::Node::SharedPtr node_;
    QVector<Stack>          stacks_;

    QPushButton* open_map_btn_{nullptr};
    QPushButton* open_3dmap_btn_{nullptr};
    MapWindow*   map_window_{nullptr};
    MapWindow*   map3d_window_{nullptr};

    // nav_preflight readiness (gates the Navigation start; see onStartClicked).
    QLabel*      preflight_label_{nullptr};
    QString      preflight_overall_;   // "ready" | "degraded" | "blocked" | ""
    QString      preflight_detail_;    // full "<overall> (k=v …)" string
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr preflight_sub_;
};
