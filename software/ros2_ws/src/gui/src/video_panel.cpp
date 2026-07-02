#include "gui/video_panel.hpp"
#include "gui/video_widget.hpp"

#include <algorithm>

VideoPanel::VideoPanel(rclcpp::Node::SharedPtr node,
                       std::shared_ptr<CameraHub> hub,
                       QWidget* parent)
    : QWidget(parent)
{
    grid_ = new QGridLayout(this);
    grid_->setContentsMargins(0, 0, 0, 0);
    grid_->setSpacing(2);

    widgets_.reserve(VIDEO_CELLS);
    for (int i = 0; i < VIDEO_CELLS; ++i) {
        auto* w = new VideoWidget(node, hub, this);
        widgets_.push_back(w);
        grid_->addWidget(w, i / COLS, i % COLS);   // (0,0) (0,1) (1,0)

        connect(w, &VideoWidget::displayClicked, this,
                [this, i]() { onWidgetClicked(i); });
        connect(w, &VideoWidget::thermalActiveChanged, this,
                &VideoPanel::onWidgetThermalChanged);
    }

    for (int r = 0; r < ROWS; ++r) grid_->setRowStretch(r, 1);
    for (int c = 0; c < COLS; ++c) grid_->setColumnStretch(c, 1);
}

void VideoPanel::setCornerWidget(QWidget* w)
{
    corner_ = w;
    if (corner_)
        grid_->addWidget(corner_, ROWS - 1, COLS - 1);   // bottom-right (1,1)
}

void VideoPanel::updateSources(const QStringList& names,
                               const QStringList& identifiers)
{
    for (auto* w : widgets_)
        w->updateSources(names, identifiers);
}

void VideoPanel::updateFilters(const QStringList& names)
{
    for (auto* w : widgets_)
        w->updateFilters(names);
}

void VideoPanel::setThermalDisplayed(bool shown)
{
    if (!shown) {
        for (auto* w : widgets_)
            w->deselectThermal();
        return;
    }
    if (thermal_count_.load() > 0)
        return;   // some cell already shows thermal — nothing to do
    for (auto* w : widgets_)
        if (w->selectThermalIfIdle())
            return;
    // No idle cell (or no thermal source discovered yet): leave it to the
    // operator's per-cell source dropdown.
}

void VideoPanel::onWidgetThermalChanged(bool active)
{
    int prev = thermal_count_.load();
    int next = active ? prev + 1 : std::max(0, prev - 1);
    thermal_count_ = next;

    if (prev == 0 && next == 1)
        emit thermalActiveChanged(true);
    else if (prev > 0 && next == 0)
        emit thermalActiveChanged(false);
}

void VideoPanel::onWidgetClicked(int index)
{
    if (enlarged_index_ == index) {
        // Restore the full grid. Zoom/pan is only available while enlarged, so
        // disabling it here also resets each cell's view to fit.
        grid_->removeWidget(widgets_[index]);
        for (int i = 0; i < VIDEO_CELLS; ++i) {
            grid_->addWidget(widgets_[i], i / COLS, i % COLS);
            widgets_[i]->setInteractive(false);
            widgets_[i]->setPaused(false);
            widgets_[i]->show();
        }
        if (corner_) corner_->show();   // restore the telemetry cell
        for (int r = 0; r < ROWS; ++r) grid_->setRowStretch(r, 1);
        for (int c = 0; c < COLS; ++c) grid_->setColumnStretch(c, 1);
        enlarged_index_ = -1;
    } else {
        // Enlarge the clicked cell to span the whole grid; pause/hide the rest
        // (videos and the corner/telemetry cell). The enlarged cell becomes
        // interactive (pinch/scroll zoom, drag pan).
        for (int i = 0; i < VIDEO_CELLS; ++i) {
            if (i != index) {
                widgets_[i]->setPaused(true);
                widgets_[i]->hide();
            }
        }
        if (corner_) corner_->hide();
        grid_->removeWidget(widgets_[index]);
        grid_->addWidget(widgets_[index], 0, 0, ROWS, COLS);
        widgets_[index]->setPaused(false);
        widgets_[index]->setInteractive(true);
        widgets_[index]->show();
        enlarged_index_ = index;
    }
}
