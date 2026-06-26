#include "gui/map_window.hpp"
#include "gui/urdf_viewer.hpp"

#include <QVBoxLayout>

MapWindow::MapWindow(rclcpp::Node::SharedPtr node, Mode mode, QWidget* parent)
    : QWidget(parent)
{
    setWindowTitle(mode == Mode::Map3D ? "RoboCorea - 3D Map"
                                       : "RoboCorea - Map");
    resize(900, 800);

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    viewer_ = new UrdfViewer(node, this);
    if (mode == Mode::Map3D)
        viewer_->setOctomapMode(true);
    else
        viewer_->setMapMode(true);
    layout->addWidget(viewer_);
}
