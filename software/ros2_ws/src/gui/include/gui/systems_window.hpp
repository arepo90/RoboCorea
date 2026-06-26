#pragma once

#include <QWidget>

#include <rclcpp/rclcpp.hpp>

class PerceptionPanel;
class MapsPanel;
class RoutineProgrammer;
class QTabWidget;

// "Robot Systems" — the top-level window opened from the dashboard's toolbar icon
// (next to ⚙ settings). Groups everything that doesn't belong on the always-on
// operator dashboard into tabs:
//
//   • Perception  — start/stop the Jetson sensor + SLAM + 3-D mapping stacks
//   • Maps        — named 2-D/3-D map library (save/load/delete with previews)
//   • Arm Program — block-programmed autonomous arm routines
//
// Created eagerly (hidden) by MainWindow so the perception stacks keep their ROS
// clients/subscriptions and a programmed routine keeps running regardless of
// whether the window is currently shown.
class SystemsWindow : public QWidget {
    Q_OBJECT
public:
    explicit SystemsWindow(rclcpp::Node::SharedPtr node, QWidget* parent = nullptr);

    // Forwarded to PerceptionPanel; MainWindow::closeEvent calls this so quitting
    // the GUI stops the robot's perception stacks cleanly.
    void stopAllStacks();

    // The Maps panel is the named-map library; MapWindow's "Save Map…" routes
    // through it (and refreshes after a robot-side save).
    MapsPanel* mapsPanel() const { return maps_panel_; }

    // Bring the window up on the given tab (by widget). Used so the dashboard icon
    // can open straight to a relevant section.
    void showTab(QWidget* tab);

private:
    rclcpp::Node::SharedPtr node_;
    QTabWidget*             tabs_{nullptr};
    PerceptionPanel*        perception_panel_{nullptr};
    MapsPanel*              maps_panel_{nullptr};
    RoutineProgrammer*      routine_panel_{nullptr};
};
