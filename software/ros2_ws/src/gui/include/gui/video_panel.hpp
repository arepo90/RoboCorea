#pragma once

#include <QGridLayout>
#include <QWidget>

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <memory>
#include <vector>

class CameraHub;
class VideoWidget;

// A 2×2 grid of VideoWidgets. Clicking a cell enlarges it to span the whole
// grid; clicking again restores. The bottom-right cell holds a "corner" widget
// (the MainWindow places the telemetry/odometry panel there) — 3 video cells + 1
// telemetry cell — so the dashboard can use the freed space in the right section.
class VideoPanel : public QWidget {
    Q_OBJECT
public:
    explicit VideoPanel(rclcpp::Node::SharedPtr node,
                        std::shared_ptr<CameraHub> hub,
                        QWidget* parent = nullptr);

    // Place a widget in the bottom-right grid cell (e.g. the odometry panel).
    void setCornerWidget(QWidget* w);

    void updateSources(const QStringList& names, const QStringList& identifiers);
    void updateFilters(const QStringList& names);

public slots:
    // Dashboard "Thermal display" toggle (display-only — acquisition is always
    // on): true selects the thermal source in the first free cell (no-op if a
    // cell already shows it), false deselects it from every cell.
    void setThermalDisplayed(bool shown);

signals:
    void thermalActiveChanged(bool active);

private slots:
    void onWidgetClicked(int index);
    void onWidgetThermalChanged(bool active);

private:
    static constexpr int ROWS = 2;
    static constexpr int COLS = 2;
    static constexpr int VIDEO_CELLS = 3;   // (0,0) (0,1) (1,0); (1,1) is corner

    QGridLayout* grid_;
    std::vector<VideoWidget*> widgets_;
    QWidget* corner_{nullptr};              // bottom-right cell (telemetry)
    int enlarged_index_{-1};
    std::atomic<int> thermal_count_{0};
};
