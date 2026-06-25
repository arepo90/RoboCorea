#include "gui/map_window.hpp"

#include <QKeyEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QTimer>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>

MapWindow::MapWindow(rclcpp::Node::SharedPtr node, QWidget* parent)
    : QWidget(parent), node_(std::move(node))
{
    setWindowTitle("RoboCorea — Map");
    resize(800, 800);
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(false);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // slam_toolbox publishes /map latched (reliable + transient_local).
    auto map_qos = rclcpp::QoS(1).reliable().transient_local();
    map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", map_qos,
        [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) { onMap(msg); });

    connect(this, &MapWindow::mapUpdated, this, &MapWindow::onMapUpdated,
            Qt::QueuedConnection);

    // Poll TF for the robot pose + repaint at ~15 Hz.
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &MapWindow::onTick);
    timer_->start(66);
}

MapWindow::~MapWindow() = default;

void MapWindow::onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    const int w = static_cast<int>(msg->info.width);
    const int h = static_cast<int>(msg->info.height);
    if (w <= 0 || h <= 0)
        return;

    // One pixel per cell, grayscale: free=light, occupied=dark, unknown=mid.
    QImage img(w, h, QImage::Format_RGB888);
    for (int row = 0; row < h; ++row) {
        for (int col = 0; col < w; ++col) {
            const int8_t v = msg->data[row * w + col];
            QRgb c;
            if (v < 0)        c = qRgb(70, 70, 80);     // unknown
            else if (v >= 65) c = qRgb(20, 20, 25);     // occupied
            else              c = qRgb(232, 232, 235);  // free (lighter for higher occ)
            img.setPixel(col, row, c);
        }
    }

    {
        std::lock_guard<std::mutex> lk(map_mutex_);
        map_img_ = std::move(img);
        res_ = msg->info.resolution;
        origin_x_ = msg->info.origin.position.x;
        origin_y_ = msg->info.origin.position.y;
        have_map_ = true;
    }
    emit mapUpdated();
}

void MapWindow::onMapUpdated() { update(); }

void MapWindow::onTick()
{
    // Robot pose in the map frame. The mapping_ekf stack's chain ends at
    // base_footprint (map->odom->base_footprint->base_laser, no base_link); the
    // rescue_nav frontend chain has base_link too. Try base_footprint first,
    // fall back to base_link, so both launch paths work.
    static const char* base_candidates[] = {"base_footprint", "base_link"};
    bool ok = false;
    for (const char* bf : base_candidates) {
        try {
            auto tf = tf_buffer_->lookupTransform(map_frame_, bf, tf2::TimePointZero);
            robot_x_ = tf.transform.translation.x;
            robot_y_ = tf.transform.translation.y;
            const auto& q = tf.transform.rotation;   // planar yaw from quaternion
            robot_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
            ok = true;
            break;
        } catch (const tf2::TransformException&) {
            // try the next candidate
        }
    }
    have_pose_ = ok;
    update();
}

void MapWindow::paintEvent(QPaintEvent*)
{
    QPainter p(this);
    p.fillRect(rect(), QColor(30, 30, 40));

    QImage img;
    double res, ox, oy;
    bool have_map;
    {
        std::lock_guard<std::mutex> lk(map_mutex_);
        img = map_img_;   // shallow copy (implicitly shared)
        res = res_; ox = origin_x_; oy = origin_y_;
        have_map = have_map_;
    }

    if (!have_map || img.isNull()) {
        p.setPen(Qt::lightGray);
        p.drawText(rect(), Qt::AlignCenter,
                   "Waiting for /map …\n(start SLAM on the robot)");
        return;
    }

    // World(metres) -> screen(px). Centre on the robot (follow) or the map centre,
    // +y up. p.transform handles meters; we draw map + robot in metres.
    const double map_cx = ox + img.width() * res * 0.5;
    const double map_cy = oy + img.height() * res * 0.5;
    const double focus_x = (follow_ && have_pose_) ? robot_x_ : map_cx;
    const double focus_y = (follow_ && have_pose_) ? robot_y_ : map_cy;

    p.save();
    p.translate(width() * 0.5 + pan_.x(), height() * 0.5 + pan_.y());
    p.scale(px_per_m_, -px_per_m_);          // metres -> px, flip y so +y is up
    p.translate(-focus_x, -focus_y);

    // Draw the occupancy grid. The image row 0 is the map origin (bottom), so
    // place it at world rect [ox, oy] .. and flip it vertically to match +y up.
    const QRectF world_rect(ox, oy, img.width() * res, img.height() * res);
    p.drawImage(world_rect, img.mirrored(false, true));

    // Robot footprint (oriented rectangle) + heading.
    if (have_pose_) {
        p.save();
        p.translate(robot_x_, robot_y_);
        p.rotate(robot_yaw_ * 180.0 / M_PI);
        QPen pen(QColor(40, 200, 255));
        pen.setWidthF(0.03);
        p.setPen(pen);
        p.setBrush(QColor(40, 200, 255, 90));
        p.drawRect(QRectF(-robot_len_ * 0.5, -robot_wid_ * 0.5, robot_len_, robot_wid_));
        // heading triangle pointing +x
        QPolygonF tri;
        tri << QPointF(robot_len_ * 0.5, 0.0)
            << QPointF(robot_len_ * 0.2,  robot_wid_ * 0.3)
            << QPointF(robot_len_ * 0.2, -robot_wid_ * 0.3);
        p.setBrush(QColor(255, 220, 40));
        p.drawPolygon(tri);
        p.restore();
    }
    p.restore();

    // HUD
    p.setPen(Qt::lightGray);
    QString hud = have_pose_
        ? QString("x=%1  y=%2  yaw=%3°   %4")
              .arg(robot_x_, 0, 'f', 2).arg(robot_y_, 0, 'f', 2)
              .arg(robot_yaw_ * 180.0 / M_PI, 0, 'f', 0)
              .arg(follow_ ? "[follow]" : "")
        : QString("no robot pose (TF map→base_footprint)   %1")
              .arg(follow_ ? "[follow]" : "");
    p.drawText(8, 18, hud);
    p.drawText(8, height() - 10, "drag: pan   wheel: zoom   F: follow   R: reset");
}

void MapWindow::wheelEvent(QWheelEvent* e)
{
    const double factor = (e->angleDelta().y() > 0) ? 1.15 : 1.0 / 1.15;
    px_per_m_ = std::clamp(px_per_m_ * factor, 2.0, 400.0);
    update();
}

void MapWindow::mousePressEvent(QMouseEvent* e)
{
    last_mouse_ = e->pos();
    follow_ = false;   // manual pan disables follow
}

void MapWindow::mouseMoveEvent(QMouseEvent* e)
{
    if (e->buttons() & Qt::LeftButton) {
        pan_ += QPointF(e->pos() - last_mouse_);
        last_mouse_ = e->pos();
        update();
    }
}

void MapWindow::keyPressEvent(QKeyEvent* e)
{
    if (e->key() == Qt::Key_F) {
        follow_ = !follow_;
        pan_ = QPointF(0, 0);
        update();
    } else if (e->key() == Qt::Key_R) {
        follow_ = true;
        pan_ = QPointF(0, 0);
        px_per_m_ = 40.0;
        update();
    } else {
        QWidget::keyPressEvent(e);
    }
}
