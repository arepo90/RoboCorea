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

    // ── Viewer ────────────────────────────────────────────────────────────────
    viewer_ = new UrdfViewer(node, this);
    if (mode == Mode::Map3D)
        viewer_->setOctomapMode(true);
    else
        viewer_->setMapMode(true);
    layout->addWidget(viewer_, 1);

    if (setpose_btn_) {
        connect(setpose_btn_, &QPushButton::toggled, viewer_, &UrdfViewer::setInitialPoseMode);
        connect(viewer_, &UrdfViewer::initialPosePreview,
                this, &MapWindow::onInitialPosePreview);
        connect(viewer_, &UrdfViewer::initialPosePicked,
                this, &MapWindow::onInitialPosePicked);
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
