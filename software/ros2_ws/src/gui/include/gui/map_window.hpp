#pragma once

#include <QWidget>

#include <rclcpp/rclcpp.hpp>
#include <rescue_interfaces/srv/save_map.hpp>

class UrdfViewer;
class QLabel;
class QPushButton;

// Standalone top-level window showing either the live 2-D SLAM map (/map) or the
// live 3-D OctoMap (/robot/map3d), with the full robot URDF placed at its
// map->base_footprint pose. SLAM/EKF and 3-D mapping run on the robot; this only
// visualizes compact map products + TF + /joint_states over DDS.
// Mouse: drag = orbit, middle-drag = pan, wheel = zoom.
//
// Toolbar:
//   • 2-D: "Set Start Pose" (click-drag → /initialpose) + "Save Map…"
//   • 3-D: "Save 3D Map…"
// Save asks robot_manager's map_manager to persist the named map on the Jetson
// (authoritative), and writes a preview thumbnail the Maps panel reads back.
class MapWindow : public QWidget {
    Q_OBJECT
public:
    enum class Mode { Map2D, Map3D };

    explicit MapWindow(rclcpp::Node::SharedPtr node,
                       Mode mode = Mode::Map2D,
                       QWidget* parent = nullptr);

private slots:
    void onSaveMap();
    void onInitialPosePreview(double x, double y, double yaw_deg);
    void onInitialPosePicked(double x, double y, double yaw_deg);

private:
    rclcpp::Node::SharedPtr node_;
    Mode        mode_;
    UrdfViewer* viewer_{nullptr};
    QLabel*     status_{nullptr};
    QPushButton* setpose_btn_{nullptr};
    rclcpp::Client<rescue_interfaces::srv::SaveMap>::SharedPtr save_cli_;
};
