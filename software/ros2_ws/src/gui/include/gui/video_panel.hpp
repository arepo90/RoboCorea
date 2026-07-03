#pragma once

#include <QGridLayout>
#include <QWidget>

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <memory>
#include <vector>

class CameraHub;
class VideoWidget;

// A full 2×2 grid of VideoWidgets. Clicking a cell enlarges it to span the
// whole grid; clicking again restores. (The bottom-right cell used to host the
// odometry/telemetry panel; that now lives in a tab next to the dashboard.)
class VideoPanel : public QWidget {
    Q_OBJECT
public:
    explicit VideoPanel(rclcpp::Node::SharedPtr node,
                        std::shared_ptr<CameraHub> hub,
                        QWidget* parent = nullptr);

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
    static constexpr int VIDEO_CELLS = ROWS * COLS;

    QGridLayout* grid_;
    std::vector<VideoWidget*> widgets_;
    int enlarged_index_{-1};
    std::atomic<int> thermal_count_{0};
};
