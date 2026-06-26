#include "gui/systems_window.hpp"
#include "gui/perception_panel.hpp"
#include "gui/maps_panel.hpp"
#include "gui/routine_programmer.hpp"

#include <QScrollArea>
#include <QTabWidget>
#include <QVBoxLayout>

namespace {
QWidget* wrapScroll(QWidget* w)
{
    auto* sa = new QScrollArea();
    sa->setWidgetResizable(true);
    sa->setWidget(w);
    sa->setFrameShape(QFrame::NoFrame);
    return sa;
}
}  // namespace

SystemsWindow::SystemsWindow(rclcpp::Node::SharedPtr node, QWidget* parent)
    : QWidget(parent), node_(node)
{
    setWindowTitle("RoboCorea — Robot Systems");
    resize(560, 820);

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    tabs_ = new QTabWidget(this);

    perception_panel_ = new PerceptionPanel(node_, this);
    maps_panel_       = new MapsPanel(node_, this);
    routine_panel_    = new RoutineProgrammer(node_, this);

    tabs_->addTab(wrapScroll(perception_panel_), "Perception");
    tabs_->addTab(wrapScroll(maps_panel_), "Maps");
    tabs_->addTab(wrapScroll(routine_panel_), "Arm Program");

    layout->addWidget(tabs_);
}

void SystemsWindow::stopAllStacks()
{
    if (perception_panel_)
        perception_panel_->stopAllStacks();
}

void SystemsWindow::showTab(QWidget* tab)
{
    if (tab) tabs_->setCurrentWidget(tab->parentWidget() ? tab->parentWidget() : tab);
    show();
    raise();
    activateWindow();
}
