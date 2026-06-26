#pragma once

#include <QWidget>

#include <rclcpp/rclcpp.hpp>

class UrdfViewer;

// Standalone top-level window showing either the live 2-D SLAM map (/map) or the
// live 3-D OctoMap (/robot/map3d), with the full robot URDF placed at its
// map->base_footprint pose. SLAM/EKF and 3-D mapping run on the robot; this only
// visualizes compact map products + TF + /joint_states over DDS.
// Mouse: drag = orbit, middle-drag = pan, wheel = zoom.
class MapWindow : public QWidget {
    Q_OBJECT
public:
    enum class Mode { Map2D, Map3D };

    explicit MapWindow(rclcpp::Node::SharedPtr node,
                       Mode mode = Mode::Map2D,
                       QWidget* parent = nullptr);

private:
    UrdfViewer* viewer_{nullptr};
};
