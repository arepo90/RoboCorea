#include "gui/perception_panel.hpp"
#include "gui/map_window.hpp"

#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

#include <chrono>
#include <vector>

namespace {
const QString kHdrStyle = "color: #aaa; font-weight: bold;";
const QString kLblStyle = "color: #888;";

QString btnStyle(const char* bg, const char* hover, const char* pressed)
{
    return QString(
        "QPushButton { background-color: %1; color: white; padding: 6px; "
        "border: 1px solid %2; border-radius: 3px; }"
        "QPushButton:hover { background-color: %2; }"
        "QPushButton:pressed { background-color: %3; }"
        "QPushButton:disabled { background-color: #2a2a35; color: #666; }")
        .arg(bg, hover, pressed);
}

// Instant amber "pending" feedback the moment start/stop is clicked, so the UI
// responds immediately instead of waiting for the status topic round-trip.
void setStackPending(QLabel* indicator, QLabel* label, const QString& text)
{
    if (!indicator || !label) return;
    label->setText(text);
    indicator->setStyleSheet("color: #ccaa00; font-size: 14px;");
    label->setStyleSheet("color: #ccaa00; font-size: 12px;");
}

// status is "<overall> (member=state …)"; the leading word drives the LED color.
QString statusColor(const QString& leading)
{
    if (leading == "active")           return "#33cc33";
    if (leading == "activating")       return "#ccaa00";
    if (leading == "partial")          return "#ccaa00";
    if (leading == "failed")           return "#cc3333";
    return "#888";                     // inactive / unknown
}
}  // namespace

PerceptionPanel::PerceptionPanel(rclcpp::Node::SharedPtr node, QWidget* parent)
    : QWidget(parent), node_(node)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(10, 10, 10, 10);
    layout->setSpacing(8);

    // ROS thread → Qt thread for every stack's status.
    connect(this, &PerceptionPanel::stackStatusReceived,
            this, &PerceptionPanel::onStackStatus, Qt::QueuedConnection);

    auto* intro = new QLabel(
        "Start/stop the robot's perception stacks on the Jetson (via robot_manager). "
        "Bring sensors up first, then mapping.", this);
    intro->setWordWrap(true);
    intro->setStyleSheet("color: #777; font-size: 11px;");
    layout->addWidget(intro);

    addStack(layout, "sensors", "Sensors (ZED + Lidar)", "Start ZED+Lidar",
             "Start the ZED + RPLidar stack on the robot "
             "(systemd rescue-sensors.target via robot_manager)",
             "Cleanly stop the ZED + RPLidar stack on the robot");

    // (No I2C-sensor stack: the MLX90640 thermal camera + LIS3MDL magnetometer
    // moved to the arm PCB and are relayed by esp32_bridge. Enable/disable is the
    // dashboard's thermal/mag toggles via /sensors/enable_mask, not a stack here.)

    open_map_btn_ = new QPushButton("Open 2D Map", this);
    open_map_btn_->setMinimumHeight(28);
    open_map_btn_->setToolTip("Open a live 2-D map window (subscribes /map + robot pose)");
    open_map_btn_->setStyleSheet(btnStyle("#2a4a7f", "#3a5a9f", "#1a3a6f"));
    connect(open_map_btn_, &QPushButton::clicked, this, &PerceptionPanel::onOpenMap);
    addStack(layout, "mapping", "Mapping / SLAM", "Start SLAM",
             "Start slam_toolbox + EKF on the robot (rescue-mapping.service). "
             "Start the sensors first.",
             "Cleanly stop SLAM + EKF on the robot", open_map_btn_);

    open_3dmap_btn_ = new QPushButton("Open 3D Map", this);
    open_3dmap_btn_->setMinimumHeight(28);
    open_3dmap_btn_->setToolTip("Open the live 3-D map window (renders /robot/map3d voxels)");
    open_3dmap_btn_->setStyleSheet(btnStyle("#2a4a7f", "#3a5a9f", "#1a3a6f"));
    connect(open_3dmap_btn_, &QPushButton::clicked, this, &PerceptionPanel::onOpen3dMap);
    addStack(layout, "mapping3d", "3D Mapping (OctoMap)", "Start 3D",
             "Start OctoMap volumetric mapping on the robot (rescue-mapping3d.service). "
             "Start sensors + SLAM first.",
             "Cleanly stop 3-D mapping on the robot", open_3dmap_btn_);

    addStack(layout, "localization", "Localization (saved map)", "Start Loc",
             "Localize on the last loaded map (AMCL + map_server). Normally you Load "
             "a map from the Maps tab — that (re)starts this with the right map; "
             "Start here re-localizes on the most recently loaded one.",
             "Stop 2-D localization on the robot");

    layout->addStretch();
}

PerceptionPanel::Stack* PerceptionPanel::addStack(
    QVBoxLayout* layout, const QString& key, const QString& title,
    const QString& start_text, const QString& start_tip, const QString& stop_tip,
    QPushButton* extra_btn)
{
    auto* sep = new QFrame(this);
    sep->setFrameShape(QFrame::HLine);
    sep->setStyleSheet("color: #444;");
    layout->addWidget(sep);

    stacks_.push_back(Stack{});
    Stack& s = stacks_.back();
    s.key = key;

    auto* hdr_row = new QHBoxLayout();
    auto* hdr = new QLabel(title, this);
    hdr->setStyleSheet(kHdrStyle);
    s.indicator = new QLabel("●", this);
    s.indicator->setStyleSheet("color: #888; font-size: 14px;");
    s.label = new QLabel("—", this);
    s.label->setStyleSheet("color: #888; font-size: 12px;");
    hdr_row->addWidget(hdr);
    hdr_row->addStretch();
    hdr_row->addWidget(s.indicator);
    hdr_row->addWidget(s.label);
    layout->addLayout(hdr_row);

    auto* btn_row = new QHBoxLayout();
    btn_row->setSpacing(4);
    s.start_btn = new QPushButton(start_text, this);
    s.start_btn->setMinimumHeight(28);
    s.start_btn->setToolTip(start_tip);
    s.start_btn->setStyleSheet(btnStyle("#1a5a2a", "#2a7a3a", "#0a3a1a"));
    connect(s.start_btn, &QPushButton::clicked, this, [this, key]() { onStartClicked(key); });
    btn_row->addWidget(s.start_btn);

    s.stop_btn = new QPushButton("Stop", this);
    s.stop_btn->setMinimumHeight(28);
    s.stop_btn->setToolTip(stop_tip);
    s.stop_btn->setStyleSheet(btnStyle("#5a2a2a", "#7a3a3a", "#3a1a1a"));
    connect(s.stop_btn, &QPushButton::clicked, this, [this, key]() { onStopClicked(key); });
    btn_row->addWidget(s.stop_btn);

    if (extra_btn) {
        extra_btn->setParent(this);
        btn_row->addWidget(extra_btn);
    }
    layout->addLayout(btn_row);

    // Latched status to match robot_manager so a late-joining GUI sees the state.
    auto qos = rclcpp::QoS(1).reliable().transient_local();
    s.start_cli = node_->create_client<std_srvs::srv::Trigger>(
        ("/robot/" + key + "/start").toStdString());
    s.stop_cli = node_->create_client<std_srvs::srv::Trigger>(
        ("/robot/" + key + "/stop").toStdString());
    s.status_sub = node_->create_subscription<std_msgs::msg::String>(
        ("/robot/" + key + "/status").toStdString(), qos,
        [this, key](std_msgs::msg::String::SharedPtr msg) {
            emit stackStatusReceived(key, QString::fromStdString(msg->data));
        });
    return &s;
}

PerceptionPanel::Stack* PerceptionPanel::stack(const QString& key)
{
    for (auto& s : stacks_)
        if (s.key == key) return &s;
    return nullptr;
}

void PerceptionPanel::onStartClicked(const QString& key)
{
    Stack* s = stack(key);
    if (!s) return;
    if (!s->start_cli->service_is_ready()) {
        RCLCPP_WARN(node_->get_logger(),
                    "'%s' start requested but the service is unavailable "
                    "(is robot_manager running on the Jetson?)", key.toStdString().c_str());
        return;
    }
    s->start_cli->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    setStackPending(s->indicator, s->label, "activating…");
    RCLCPP_INFO(node_->get_logger(), "%s: start requested", key.toStdString().c_str());
}

void PerceptionPanel::onStopClicked(const QString& key)
{
    Stack* s = stack(key);
    if (!s) return;
    if (!s->stop_cli->service_is_ready()) {
        RCLCPP_WARN(node_->get_logger(),
                    "'%s' stop requested but the service is unavailable "
                    "(is robot_manager running on the Jetson?)", key.toStdString().c_str());
        return;
    }
    s->stop_cli->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    setStackPending(s->indicator, s->label, "deactivating…");
    RCLCPP_INFO(node_->get_logger(), "%s: stop requested", key.toStdString().c_str());
}

void PerceptionPanel::onStackStatus(const QString& key, const QString& status)
{
    Stack* s = stack(key);
    if (!s) return;
    s->label->setText(status);
    const QString leading = status.section(' ', 0, 0);
    const QString color = statusColor(leading);
    s->indicator->setStyleSheet(QString("color: %1; font-size: 14px;").arg(color));
    s->label->setStyleSheet(QString("color: %1; font-size: 12px;").arg(color));

    const bool active = (leading == "active");
    s->start_btn->setEnabled(!active && leading != "activating");
    s->stop_btn->setEnabled(leading != "inactive");
}

void PerceptionPanel::onOpenMap()
{
    if (!map_window_)
        map_window_ = new MapWindow(node_, MapWindow::Mode::Map2D);   // top-level
    map_window_->show();
    map_window_->raise();
    map_window_->activateWindow();
}

void PerceptionPanel::onOpen3dMap()
{
    if (!map3d_window_)
        map3d_window_ = new MapWindow(node_, MapWindow::Mode::Map3D);   // top-level
    map3d_window_->show();
    map3d_window_->raise();
    map3d_window_->activateWindow();
}

void PerceptionPanel::stopAllStacks()
{
    // Dependents first (localization + 3-D/2-D mapping consume the sensors), then sensors.
    const char* order[] = {"localization", "mapping3d", "mapping", "sensors"};

    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    std::vector<rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture> futures;
    for (const char* key : order) {
        Stack* s = stack(key);
        if (s && s->stop_cli && s->stop_cli->service_is_ready()) {
            futures.push_back(s->stop_cli->async_send_request(req).future.share());
            RCLCPP_INFO(node_->get_logger(), "GUI closing: stop '%s' requested", key);
        }
    }
    if (futures.empty()) {
        RCLCPP_INFO(node_->get_logger(),
                    "GUI closing: no robot_manager stacks reachable to stop");
        return;
    }
    // The ROS spin thread is still running during closeEvent, so it delivers the
    // requests and completes these futures. Wait a bounded window so the stops
    // actually go out (and are acked) before the app tears the node down.
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    for (auto& f : futures)
        f.wait_until(deadline);
}
