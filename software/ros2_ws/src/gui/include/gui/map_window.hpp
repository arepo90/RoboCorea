#pragma once

#include <QWidget>

#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
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
//   • 2-D: "Set Start Pose" (click-drag → /initialpose), "Set Nav Goal"
//          (click-drag → NavigateToPose action), "Cancel" + "Save Map…"
//   • 3-D: "Save 3D Map…"
// Save asks robot_manager's map_manager to persist the named map on the Jetson
// (authoritative), and writes a preview thumbnail the Maps panel reads back.
//
// The Nav goal is sent through a NavigateToPose *action client* (not a fire-and-
// forget /goal_pose) so the window can show live status (accepted / navigating
// with distance-to-go / succeeded / aborted / canceled) and cancel a goal.
class MapWindow : public QWidget {
    Q_OBJECT
public:
    enum class Mode { Map2D, Map3D };

    explicit MapWindow(rclcpp::Node::SharedPtr node,
                       Mode mode = Mode::Map2D,
                       QWidget* parent = nullptr);

signals:
    // ROS (action callback) thread → Qt thread: nav status text + color hint.
    void navStatusChanged(const QString& text, const QString& color);

private slots:
    void onSaveMap();
    void onInitialPosePreview(double x, double y, double yaw_deg);
    void onInitialPosePicked(double x, double y, double yaw_deg);
    void onGoalPosePreview(double x, double y, double yaw_deg);
    void onGoalPosePicked(double x, double y, double yaw_deg);
    void onCancelGoal();
    void onNavStatus(const QString& text, const QString& color);

private:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    rclcpp::Node::SharedPtr node_;
    Mode        mode_;
    UrdfViewer* viewer_{nullptr};
    QLabel*     status_{nullptr};
    QPushButton* setpose_btn_{nullptr};
    QPushButton* goal_btn_{nullptr};
    QPushButton* cancel_btn_{nullptr};
    QLabel*      nav_status_{nullptr};
    rclcpp::Client<rescue_interfaces::srv::SaveMap>::SharedPtr save_cli_;

    // NavigateToPose action client + the in-flight goal handle (touched from both
    // the ROS callback thread and the Qt thread → guarded by goal_mtx_).
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_cli_;
    std::mutex             goal_mtx_;
    GoalHandle::SharedPtr  goal_handle_;
};
