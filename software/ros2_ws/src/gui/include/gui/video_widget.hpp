#pragma once

#include <QComboBox>
#include <QLabel>
#include <QMouseEvent>
#include <QPointF>
#include <QWheelEvent>
#include <QWidget>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

class CameraHub;
struct FilterConfig;

// The video display surface. A plain (non-drag) click toggles enlarge/restore.
// While `interactive_` is set (i.e. the cell is enlarged) it also turns trackpad
// pinch + Ctrl/scroll into zoom and a drag / two-finger scroll into pan, emitted
// as zoomBy/panBy for the owning VideoWidget to fold into the frame ROI.
class ClickableLabel : public QLabel {
    Q_OBJECT
public:
    using QLabel::QLabel;
    void setInteractive(bool on) { interactive_ = on; }
signals:
    void clicked();
    void zoomBy(double factor, QPointF focus);  // pinch / Ctrl+scroll, focus in widget px
    void panBy(QPointF delta);                  // drag / two-finger scroll, in widget px
protected:
    void mousePressEvent(QMouseEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void mouseReleaseEvent(QMouseEvent* e) override;
    void wheelEvent(QWheelEvent* e) override;
    bool event(QEvent* e) override;             // QNativeGestureEvent (pinch)
private:
    bool   interactive_{false};
    bool   dragging_{false};
    bool   moved_{false};
    QPoint press_pos_;
    QPoint last_pos_;
};

// One video cell: a source dropdown, a (future) filter dropdown, and a display.
// A dedicated worker thread pulls frames from the selected source and emits them
// to the GUI thread via a queued signal — widgets are never touched off-thread.
//
// Sources:
//   CAPTURE   → pull frames from CameraHub (local:N V4L2 or gst:<pipe> SRT)
//   ROS_TOPIC → sensor_msgs/Image subscription
//   THERMAL   → sensor_msgs/Image subscription, colormapped on display
class VideoWidget : public QWidget {
    Q_OBJECT
public:
    explicit VideoWidget(rclcpp::Node::SharedPtr node,
                         std::shared_ptr<CameraHub> hub,
                         QWidget* parent = nullptr);
    ~VideoWidget() override;

    void setPaused(bool paused);
    void updateSources(const QStringList& names, const QStringList& identifiers);
    void updateFilters(const QStringList& names);

    // Enable zoom/pan (only while the cell is enlarged). Disabling resets the
    // view to fit, so collapsing back to the 2×2 grid always shows the full frame.
    void setInteractive(bool on);

protected:
    // Repositions the floating zoom overlay when the display is resized.
    bool eventFilter(QObject* obj, QEvent* ev) override;

signals:
    void displayClicked();
    void frameReady(const QImage& image);
    void statusChanged(const QString& text);
    void thermalActiveChanged(bool active);

private slots:
    void onSourceChanged(int index);
    void onFilterChanged(int index);
    void onFrameReady(const QImage& image);
    void onStatusChanged(const QString& text);
    void onZoomBy(double factor, QPointF focus);  // from ClickableLabel
    void onPanBy(QPointF delta);                  // from ClickableLabel

private:
    enum class SourceType { NONE, CAPTURE, ROS_TOPIC, THERMAL };

    void workerLoop();
    void onImageReceived(const sensor_msgs::msg::Image::SharedPtr& msg);
    void onThermalReceived(const sensor_msgs::msg::Image::SharedPtr& msg);
    static QImage matToQImage(const cv::Mat& mat);

    // Builds/tears down the per-filter options widgets shown below the display.
    void setupFilterOptions(const std::string& filter_name);
    void clearFilterOptions();

    // Keeps the zoomed ROI fully inside the frame (and centered at zoom == 1).
    void clampView();
    void positionZoomControls();   // anchor the overlay to the display's corner

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<CameraHub> camera_hub_;

    QComboBox* source_combo_;
    QComboBox* filter_combo_;
    QComboBox* aspect_combo_{nullptr};
    ClickableLabel* display_;

    // Display aspect mode: 0 = Fit (keep source aspect), 1 = Stretch (fill cell),
    // 2 = force 4:3, 3 = force 16:9. Read on the UI thread in onFrameReady.
    std::atomic<int> aspect_mode_{0};

    // Dynamic filter-options widget (sliders/toggles), rebuilt per filter.
    QWidget* options_container_{nullptr};

    // Floating +/−/Fit zoom overlay, shown only while the cell is enlarged
    // (pinch isn't deliverable on X11, so these give modifier-free zooming).
    QWidget* zoom_controls_{nullptr};

    std::thread worker_;
    std::atomic<bool> running_{true};
    std::atomic<bool> paused_{false};
    std::atomic<SourceType> source_type_{SourceType::NONE};

    // Current CAPTURE source id (e.g. "local:0", "gst:..."), guarded by mutex.
    std::mutex source_mutex_;
    std::string current_capture_id_;

    // ROS subscriptions.
    std::mutex sub_mutex_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr thermal_sub_;

    // Latest frame from a ROS callback.
    std::mutex frame_mutex_;
    cv::Mat latest_ros_frame_;

    // Current filter: a per-widget instance + its thread-safe config. The worker
    // thread reads current_filter_func_ under filter_mutex_; the UI thread writes
    // it on filter change and tweaks filter_config_ atomics live.
    std::mutex filter_mutex_;
    std::function<cv::Mat(const cv::Mat&)> current_filter_func_;
    std::shared_ptr<FilterConfig> filter_config_;

    // View transform (zoom/pan), applied to the source frame BEFORE the filter so
    // the CV pipeline sees the zoomed region. Written from the UI thread (gesture
    // slots), read from the worker thread. zoom_ ≥ 1; (pan_cx_,pan_cy_) is the ROI
    // centre in normalised source coords. Active only while interactive_.
    std::atomic<bool>   interactive_{false};
    std::atomic<double> zoom_{1.0};
    std::atomic<double> pan_cx_{0.5};
    std::atomic<double> pan_cy_{0.5};
};
