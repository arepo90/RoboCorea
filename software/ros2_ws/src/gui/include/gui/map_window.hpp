#pragma once

#include <QImage>
#include <QPointF>
#include <QWidget>

#include <atomic>
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class QTimer;

// Standalone top-level window that renders the live SLAM map (/map,
// nav_msgs/OccupancyGrid) as a 2-D top-down view and overlays the robot's pose
// (footprint rectangle + heading) from the map->base_link transform. It is NOT
// RViz — just a focused "where is the robot on the map" view.
//
// Interaction: drag = pan, wheel = zoom, F = toggle follow-robot, R = reset view.
// The map is published on the Jetson by slam_toolbox and reaches the workstation
// over DDS; the robot pose comes from TF (slam map->odom + EKF odom->base).
class MapWindow : public QWidget {
    Q_OBJECT
public:
    explicit MapWindow(rclcpp::Node::SharedPtr node, QWidget* parent = nullptr);
    ~MapWindow() override;

signals:
    void mapUpdated();   // ROS thread -> Qt thread

protected:
    void paintEvent(QPaintEvent*) override;
    void wheelEvent(QWheelEvent*) override;
    void mousePressEvent(QMouseEvent*) override;
    void mouseMoveEvent(QMouseEvent*) override;
    void keyPressEvent(QKeyEvent*) override;

private slots:
    void onMapUpdated();
    void onTick();   // poll TF for the robot pose, then repaint

private:
    void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    QTimer* timer_{nullptr};

    // Map (guarded: written on the ROS thread, read on the Qt thread).
    std::mutex map_mutex_;
    QImage map_img_;                         // 1 px per cell; grayscale
    double res_{0.05};                       // m / cell
    double origin_x_{0.0}, origin_y_{0.0};   // map-frame coords of cell (0,0) corner
    bool   have_map_{false};

    // Robot pose in the map frame (from TF).
    std::atomic<bool> have_pose_{false};
    double robot_x_{0.0}, robot_y_{0.0}, robot_yaw_{0.0};

    // View: pixels per metre, plus a pan offset (screen px) and follow mode.
    double  px_per_m_{40.0};
    QPointF pan_{0.0, 0.0};
    bool    follow_{true};
    QPoint  last_mouse_;

    // Frames + robot footprint (metres). Footprint stands in for the full body
    // in this top-down view; sized to the rescue base by default.
    std::string map_frame_{"map"};
    double robot_len_{0.6};
    double robot_wid_{0.5};
};
