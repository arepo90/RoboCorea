#include "gui/map_window.hpp"
#include "gui/map_thumb.hpp"
#include "gui/urdf_viewer.hpp"

#include <QDir>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

#include <cmath>

namespace {
QString btnStyle(const char* bg, const char* hover, const char* pressed)
{
    return QString(
        "QPushButton { background-color: %1; color: white; padding: 5px; "
        "border: 1px solid %2; border-radius: 3px; }"
        "QPushButton:hover { background-color: %2; }"
        "QPushButton:pressed { background-color: %3; }"
        "QPushButton:checked { background-color: #c8870a; color: black; border-color: #ffb13a; }")
        .arg(bg, hover, pressed);
}
}  // namespace

MapWindow::MapWindow(rclcpp::Node::SharedPtr node, Mode mode, QWidget* parent)
    : QWidget(parent), node_(node), mode_(mode)
{
    setWindowTitle(mode == Mode::Map3D ? "RoboCorea — 3D Map" : "RoboCorea — Map");
    resize(900, 840);

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);

    // ── Toolbar ───────────────────────────────────────────────────────────────
    auto* bar = new QHBoxLayout();
    bar->setContentsMargins(6, 4, 6, 4);
    bar->setSpacing(6);

    if (mode == Mode::Map2D) {
        setpose_btn_ = new QPushButton("Set Start Pose", this);
        setpose_btn_->setCheckable(true);
        setpose_btn_->setToolTip(
            "Click the map to place the robot's start point; drag to aim its heading "
            "(yaw only). Publishes /initialpose for localization.");
        setpose_btn_->setStyleSheet(btnStyle("#2a4a7f", "#3a5a9f", "#1a3a6f"));
        bar->addWidget(setpose_btn_);

        goal_btn_ = new QPushButton("Set Nav Goal", this);
        goal_btn_->setCheckable(true);
        goal_btn_->setToolTip(
            "Click the map to set a Nav2 goal point; drag to aim the goal heading. "
            "Sends a NavigateToPose action (needs the Navigation stack running).");
        goal_btn_->setStyleSheet(btnStyle("#2a4a7f", "#3a5a9f", "#1a3a6f"));
        bar->addWidget(goal_btn_);

        cancel_btn_ = new QPushButton("Cancel", this);
        cancel_btn_->setToolTip("Cancel the active navigation goal");
        cancel_btn_->setStyleSheet(btnStyle("#5a2a2a", "#7a3a3a", "#3a1a1a"));
        connect(cancel_btn_, &QPushButton::clicked, this, &MapWindow::onCancelGoal);
        bar->addWidget(cancel_btn_);

        auto* save = new QPushButton("Save Map…", this);
        save->setToolTip("Save the current 2-D map under a name (on the robot) + a preview");
        save->setStyleSheet(btnStyle("#1a5a2a", "#2a7a3a", "#0a3a1a"));
        connect(save, &QPushButton::clicked, this, &MapWindow::onSaveMap);
        bar->addWidget(save);
    } else {
        auto* save = new QPushButton("Save 3D Map…", this);
        save->setToolTip("Save the current 3-D OctoMap under a name (on the robot) + a preview");
        save->setStyleSheet(btnStyle("#1a5a2a", "#2a7a3a", "#0a3a1a"));
        connect(save, &QPushButton::clicked, this, &MapWindow::onSaveMap);
        bar->addWidget(save);
    }

    status_ = new QLabel("—", this);
    status_->setStyleSheet("color: #888; font-size: 11px;");
    bar->addWidget(status_, 1);
    layout->addLayout(bar);

    // Dedicated navigation-status line (2-D only): accepted / navigating / result.
    if (mode == Mode::Map2D) {
        nav_status_ = new QLabel("nav: idle", this);
        nav_status_->setContentsMargins(8, 0, 8, 2);
        nav_status_->setStyleSheet("color: #888; font-size: 11px;");
        layout->addWidget(nav_status_);
        connect(this, &MapWindow::navStatusChanged,
                this, &MapWindow::onNavStatus, Qt::QueuedConnection);
    }

    // ── Viewer ────────────────────────────────────────────────────────────────
    viewer_ = new UrdfViewer(node, this);
    if (mode == Mode::Map3D)
        viewer_->setOctomapMode(true);
    else
        viewer_->setMapMode(true);
    layout->addWidget(viewer_, 1);

    if (setpose_btn_) {
        connect(setpose_btn_, &QPushButton::toggled, viewer_, &UrdfViewer::setInitialPoseMode);
        // Mutually exclusive with the goal pick (UI side; the viewer also enforces it).
        connect(setpose_btn_, &QPushButton::toggled, this, [this](bool on) {
            if (on && goal_btn_) goal_btn_->setChecked(false);
        });
        connect(viewer_, &UrdfViewer::initialPosePreview,
                this, &MapWindow::onInitialPosePreview);
        connect(viewer_, &UrdfViewer::initialPosePicked,
                this, &MapWindow::onInitialPosePicked);
    }
    if (goal_btn_) {
        connect(goal_btn_, &QPushButton::toggled, viewer_, &UrdfViewer::setGoalPoseMode);
        connect(goal_btn_, &QPushButton::toggled, this, [this](bool on) {
            if (on && setpose_btn_) setpose_btn_->setChecked(false);
        });
        connect(viewer_, &UrdfViewer::goalPosePreview,
                this, &MapWindow::onGoalPosePreview);
        connect(viewer_, &UrdfViewer::goalPosePicked,
                this, &MapWindow::onGoalPosePicked);
        nav_cli_ = rclcpp_action::create_client<NavigateToPose>(node_, "/navigate_to_pose");
    }

    save_cli_ = node_->create_client<rescue_interfaces::srv::SaveMap>("/robot/maps/save");
}

void MapWindow::onInitialPosePreview(double x, double y, double yaw_deg)
{
    status_->setText(QString("start pose: x=%1  y=%2  yaw=%3°")
                         .arg(x, 0, 'f', 2).arg(y, 0, 'f', 2).arg(yaw_deg, 0, 'f', 0));
    status_->setStyleSheet("color: #4fc3f7; font-size: 11px;");
}

void MapWindow::onInitialPosePicked(double x, double y, double yaw_deg)
{
    status_->setText(QString("✓ /initialpose set: x=%1 y=%2 yaw=%3°")
                         .arg(x, 0, 'f', 2).arg(y, 0, 'f', 2).arg(yaw_deg, 0, 'f', 0));
    status_->setStyleSheet("color: #8afa8a; font-size: 11px;");
    if (setpose_btn_) setpose_btn_->setChecked(false);   // one-shot
}

void MapWindow::onGoalPosePreview(double x, double y, double yaw_deg)
{
    status_->setText(QString("nav goal: x=%1  y=%2  yaw=%3°")
                         .arg(x, 0, 'f', 2).arg(y, 0, 'f', 2).arg(yaw_deg, 0, 'f', 0));
    status_->setStyleSheet("color: #4fc3f7; font-size: 11px;");
}

void MapWindow::onGoalPosePicked(double x, double y, double yaw_deg)
{
    if (goal_btn_) goal_btn_->setChecked(false);   // one-shot, like Set Start Pose
    if (!nav_cli_) return;
    if (!nav_cli_->action_server_is_ready()) {
        emit navStatusChanged(
            "nav: NavigateToPose server not available — start the Navigation stack",
            "#cc6666");
        return;
    }

    const double yaw = yaw_deg * 0.017453292519943295;   // deg → rad
    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = node_->now();
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    goal.pose.pose.orientation.z = std::sin(yaw * 0.5);   // yaw-only (roll/pitch = 0)
    goal.pose.pose.orientation.w = std::cos(yaw * 0.5);

    // Callbacks run on the ROS spin thread → marshal to the Qt thread via the
    // queued navStatusChanged signal (never touch widgets from the ROS thread).
    rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;
    opts.goal_response_callback = [this](GoalHandle::SharedPtr h) {
        if (!h) {
            emit navStatusChanged("nav: goal REJECTED by the server", "#cc6666");
            return;
        }
        { std::lock_guard<std::mutex> lk(goal_mtx_); goal_handle_ = h; }
        emit navStatusChanged("nav: goal accepted — navigating…", "#4fc3f7");
    };
    opts.feedback_callback = [this](
            GoalHandle::SharedPtr,
            const std::shared_ptr<const NavigateToPose::Feedback> fb) {
        emit navStatusChanged(QString("nav: navigating — %1 m to go")
                                  .arg(fb->distance_remaining, 0, 'f', 2), "#4fc3f7");
    };
    opts.result_callback = [this](const GoalHandle::WrappedResult& res) {
        { std::lock_guard<std::mutex> lk(goal_mtx_); goal_handle_.reset(); }
        switch (res.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                emit navStatusChanged("nav: ✓ goal reached", "#8afa8a"); break;
            case rclcpp_action::ResultCode::ABORTED:
                emit navStatusChanged("nav: ✗ aborted (planning/controller failure)",
                                      "#cc6666"); break;
            case rclcpp_action::ResultCode::CANCELED:
                emit navStatusChanged("nav: goal canceled", "#ccaa00"); break;
            default:
                emit navStatusChanged("nav: unknown result", "#cc6666"); break;
        }
    };
    nav_cli_->async_send_goal(goal, opts);
    emit navStatusChanged(QString("nav: sending goal x=%1 y=%2 yaw=%3°…")
                              .arg(x, 0, 'f', 2).arg(y, 0, 'f', 2)
                              .arg(yaw_deg, 0, 'f', 0), "#4fc3f7");
}

void MapWindow::onCancelGoal()
{
    GoalHandle::SharedPtr h;
    { std::lock_guard<std::mutex> lk(goal_mtx_); h = goal_handle_; }
    if (!nav_cli_ || !h) {
        emit navStatusChanged("nav: no active goal to cancel", "#888");
        return;
    }
    nav_cli_->async_cancel_goal(h);
    emit navStatusChanged("nav: canceling…", "#ccaa00");
}

void MapWindow::onNavStatus(const QString& text, const QString& color)
{
    if (!nav_status_) return;
    nav_status_->setText(text);
    nav_status_->setStyleSheet(QString("color: %1; font-size: 11px;").arg(color));
}

void MapWindow::onSaveMap()
{
    const QString kind = (mode_ == Mode::Map3D) ? "3d" : "2d";
    bool ok = false;
    QString name = QInputDialog::getText(this, "Save Map", "Map name:",
                                         QLineEdit::Normal, "", &ok).trimmed();
    if (!ok || name.isEmpty()) return;
    if (!save_cli_->service_is_ready()) {
        status_->setText("map_manager save service not available (is it running on the Jetson?)");
        status_->setStyleSheet("color: #cc6666; font-size: 11px;");
        return;
    }

    // Snapshot the current view now (GUI thread) for the preview thumbnail; persist
    // it only if the robot-side save succeeds.
    QImage thumb = viewer_ ? viewer_->grabFramebuffer() : QImage();

    auto req = std::make_shared<rescue_interfaces::srv::SaveMap::Request>();
    req->name = name.toStdString();
    req->kind = kind.toStdString();
    status_->setText(QString("saving '%1' (%2)…").arg(name, kind));
    status_->setStyleSheet("color: #4fc3f7; font-size: 11px;");
    save_cli_->async_send_request(
        req, [this, name, kind, thumb](
                 rclcpp::Client<rescue_interfaces::srv::SaveMap>::SharedFuture f) {
            auto r = f.get();
            if (r->success && !thumb.isNull()) {
                const QString path = mapThumbPath(name, kind);
                QDir().mkpath(QFileInfo(path).absolutePath());
                thumb.scaled(400, 360, Qt::KeepAspectRatio, Qt::SmoothTransformation)
                    .save(path, "PNG");
            }
            // Update the label on the Qt thread.
            QMetaObject::invokeMethod(this, [this, r]() {
                status_->setText(QString::fromStdString(r->message));
                status_->setStyleSheet(QString("color: %1; font-size: 11px;")
                                           .arg(r->success ? "#8afa8a" : "#cc6666"));
            });
        });
}
