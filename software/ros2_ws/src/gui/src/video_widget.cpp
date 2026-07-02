#include "gui/video_widget.hpp"
#include "gui/camera_hub.hpp"
#include "gui/app_settings.hpp"
#include "gui/filter_registry.hpp"

#include <QButtonGroup>
#include <QHBoxLayout>
#include <QNativeGestureEvent>
#include <QPushButton>
#include <QSlider>
#include <QVBoxLayout>

#include <algorithm>
#include <chrono>
#include <cmath>

// ── Shared styles for the per-filter option controls ────────────────────────
static const QString TOGGLE_BTN_STYLE =
    "QPushButton { background-color: #2d2d45; color: #888; padding: 2px 8px; "
    "border: 1px solid #3a3a55; border-radius: 3px; font-size: 10px; "
    "max-height: 24px; }"
    "QPushButton:checked { background-color: #2a6a2a; color: white; "
    "border-color: #4aba4a; }"
    "QPushButton:hover { background-color: #3a3a55; }";

static const QString OPTION_CONTAINER_STYLE =
    "QLabel { color: #aaa; font-size: 10px; margin: 0; padding: 0; }"
    "QSlider { max-height: 14px; }"
    "QSlider::groove:horizontal { background: #2d2d45; height: 6px; "
    "border-radius: 2px; }"
    "QSlider::handle:horizontal { background: #4a9af5; width: 10px; "
    "margin: -3px 0; border-radius: 5px; }";

static const QString ZOOM_BTN_STYLE =
    "QPushButton { background-color: rgba(20,20,30,190); color: #e0e0e0; "
    "border: 1px solid #555; border-radius: 4px; font-size: 14px; "
    "font-weight: bold; padding: 0; }"
    "QPushButton:hover { background-color: rgba(60,100,170,210); "
    "border-color: #4a9af5; }"
    "QPushButton:pressed { background-color: rgba(40,70,130,230); }";

// ── ClickableLabel: click toggles enlarge; while enlarged, gestures zoom/pan ──

void ClickableLabel::mousePressEvent(QMouseEvent* e)
{
    if (e->button() == Qt::LeftButton) {
        dragging_ = true;
        moved_ = false;
        press_pos_ = e->position().toPoint();
        last_pos_ = press_pos_;
    }
}

void ClickableLabel::mouseMoveEvent(QMouseEvent* e)
{
    if (!dragging_ || !(e->buttons() & Qt::LeftButton)) return;
    const QPoint cur = e->position().toPoint();
    if ((cur - press_pos_).manhattanLength() > 3)
        moved_ = true;
    if (interactive_ && moved_)
        emit panBy(QPointF(cur - last_pos_));
    last_pos_ = cur;
}

void ClickableLabel::mouseReleaseEvent(QMouseEvent* e)
{
    if (e->button() != Qt::LeftButton) return;
    const bool was_click = dragging_ && !moved_;
    dragging_ = false;
    if (was_click)
        emit clicked();   // plain click → enlarge / restore
}

void ClickableLabel::wheelEvent(QWheelEvent* e)
{
    if (!interactive_) { e->ignore(); return; }
    const QPoint pd = e->pixelDelta();
    const QPoint ad = e->angleDelta();
    if (e->modifiers() & Qt::ControlModifier) {
        // Ctrl + scroll → zoom (mouse-wheel fallback for pinch).
        const double steps = !pd.isNull() ? pd.y() / 40.0 : ad.y() / 120.0;
        emit zoomBy(std::pow(1.2, steps), e->position());
    } else {
        // Two-finger scroll → pan (content follows the fingers).
        const QPointF d = !pd.isNull() ? QPointF(pd) : QPointF(ad) / 8.0;
        emit panBy(d);
    }
    e->accept();
}

bool ClickableLabel::event(QEvent* e)
{
    if (e->type() == QEvent::NativeGesture) {
        auto* ng = static_cast<QNativeGestureEvent*>(e);
        if (ng->gestureType() == Qt::ZoomNativeGesture) {
            if (interactive_)
                emit zoomBy(1.0 + ng->value(), ng->position());  // pinch
            return true;
        }
        // Swallow begin/end markers so they don't fall through as clicks.
        if (ng->gestureType() == Qt::BeginNativeGesture ||
            ng->gestureType() == Qt::EndNativeGesture)
            return true;
    }
    return QLabel::event(e);
}

VideoWidget::VideoWidget(rclcpp::Node::SharedPtr node,
                         std::shared_ptr<CameraHub> hub,
                         QWidget* parent)
    : QWidget(parent), node_(node), camera_hub_(hub)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(2, 2, 2, 2);
    layout->setSpacing(2);

    auto* top_bar = new QHBoxLayout();
    source_combo_ = new QComboBox(this);
    filter_combo_ = new QComboBox(this);
    source_combo_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    filter_combo_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    source_combo_->addItem("None", "");
    filter_combo_->addItem("None");
    filter_combo_->setEnabled(false);
    filter_combo_->setStyleSheet(
        "QComboBox:disabled { color: #555; background-color: #1e1e2e; "
        "border: 1px solid #2a2a3a; }");
    // Display aspect ratio for this cell.
    aspect_combo_ = new QComboBox(this);
    aspect_combo_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    aspect_combo_->addItem("Fit");      // 0: keep source aspect (letterbox)
    aspect_combo_->addItem("Stretch");  // 1: fill the cell, ignore aspect
    aspect_combo_->addItem("4:3");      // 2: force 4:3 (SD analog cams)
    aspect_combo_->addItem("16:9");     // 3: force 16:9
    aspect_combo_->setToolTip("Display aspect ratio");
    connect(aspect_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            [this](int idx) { aspect_mode_.store(idx); });

    top_bar->addWidget(source_combo_);
    top_bar->addWidget(filter_combo_);
    top_bar->addWidget(aspect_combo_);
    layout->addLayout(top_bar);

    display_ = new ClickableLabel(this);
    display_->setAlignment(Qt::AlignCenter);
    display_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    display_->setMinimumSize(160, 120);
    display_->setStyleSheet(
        "background-color: #000; border: 1px solid #333; color: #555;");
    display_->setText("No Source");
    layout->addWidget(display_);

    // Floating zoom overlay (child of the display, top-right). Hidden until the
    // cell is enlarged. Pinch can't be delivered on X11, so these buttons give
    // modifier-free zoom; Ctrl+scroll and drag-pan still work too.
    zoom_controls_ = new QWidget(display_);
    auto* zlay = new QHBoxLayout(zoom_controls_);
    zlay->setContentsMargins(0, 0, 0, 0);
    zlay->setSpacing(3);
    struct { const char* text; const char* tip; double factor; bool reset; } zbtns[] = {
        {"−", "Zoom out",        1.0 / 1.25, false},  // − (minus sign)
        {"+",      "Zoom in",         1.25,       false},
        {"Fit",    "Reset zoom (fit)", 0.0,       true},
    };
    for (const auto& b : zbtns) {
        auto* btn = new QPushButton(QString::fromUtf8(b.text), zoom_controls_);
        btn->setToolTip(b.tip);
        btn->setFocusPolicy(Qt::NoFocus);
        btn->setCursor(Qt::ArrowCursor);
        btn->setFixedSize(b.reset ? 34 : 26, 26);
        btn->setStyleSheet(ZOOM_BTN_STYLE);
        const double factor = b.factor;
        const bool reset = b.reset;
        connect(btn, &QPushButton::clicked, this, [this, factor, reset]() {
            if (reset) {
                zoom_.store(1.0);
                pan_cx_.store(0.5);
                pan_cy_.store(0.5);
            } else {
                // Buttons zoom about the view centre.
                onZoomBy(factor, QPointF(display_->width() / 2.0,
                                         display_->height() / 2.0));
            }
        });
        zlay->addWidget(btn);
    }
    zoom_controls_->hide();
    display_->installEventFilter(this);   // reposition overlay on resize

    connect(source_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &VideoWidget::onSourceChanged);
    connect(filter_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &VideoWidget::onFilterChanged);
    connect(display_, &ClickableLabel::clicked,
            this, &VideoWidget::displayClicked);
    connect(display_, &ClickableLabel::zoomBy, this, &VideoWidget::onZoomBy);
    connect(display_, &ClickableLabel::panBy, this, &VideoWidget::onPanBy);
    connect(this, &VideoWidget::frameReady, this, &VideoWidget::onFrameReady,
            Qt::QueuedConnection);
    connect(this, &VideoWidget::statusChanged, this, &VideoWidget::onStatusChanged,
            Qt::QueuedConnection);

    worker_ = std::thread(&VideoWidget::workerLoop, this);
}

VideoWidget::~VideoWidget()
{
    running_ = false;
    if (worker_.joinable())
        worker_.join();

    std::lock_guard<std::mutex> lock(source_mutex_);
    if (!current_capture_id_.empty() && camera_hub_)
        camera_hub_->unsubscribe(current_capture_id_);
}

void VideoWidget::setPaused(bool paused)
{
    paused_ = paused;
}

void VideoWidget::setInteractive(bool on)
{
    interactive_.store(on);
    display_->setInteractive(on);
    if (zoom_controls_) {
        zoom_controls_->setVisible(on);
        if (on) {
            positionZoomControls();
            zoom_controls_->raise();
        }
    }
    if (!on) {
        // Reset to fit when collapsing back to the grid.
        zoom_.store(1.0);
        pan_cx_.store(0.5);
        pan_cy_.store(0.5);
    }
}

bool VideoWidget::eventFilter(QObject* obj, QEvent* ev)
{
    if (obj == display_ && ev->type() == QEvent::Resize)
        positionZoomControls();
    return QWidget::eventFilter(obj, ev);
}

void VideoWidget::positionZoomControls()
{
    if (!zoom_controls_) return;
    zoom_controls_->adjustSize();
    const int x = display_->width() - zoom_controls_->width() - 8;
    zoom_controls_->move(std::max(0, x), 8);   // top-right corner, 8px inset
}

// ── Zoom / pan (UI thread) ──────────────────────────────────────────────────

static constexpr double kMaxZoom = 10.0;

void VideoWidget::clampView()
{
    double z = zoom_.load();
    if (z < 1.0) { z = 1.0; zoom_.store(1.0); }
    // ROI spans 1/z of the frame, so its centre must stay within [half, 1-half].
    const double half = 0.5 / z;
    pan_cx_.store(std::clamp(pan_cx_.load(), half, 1.0 - half));
    pan_cy_.store(std::clamp(pan_cy_.load(), half, 1.0 - half));
}

void VideoWidget::onZoomBy(double factor, QPointF focus)
{
    if (!interactive_.load() || factor <= 0.0) return;

    const double z  = zoom_.load();
    const double nz = std::clamp(z * factor, 1.0, kMaxZoom);
    if (nz == z) return;

    // Keep the source point under the cursor fixed across the zoom. focus is in
    // widget pixels; map to a normalised offset from the display centre.
    const double w = std::max(1, display_->width());
    const double h = std::max(1, display_->height());
    const double ox = std::clamp(focus.x() / w - 0.5, -0.5, 0.5);
    const double oy = std::clamp(focus.y() / h - 0.5, -0.5, 0.5);

    const double sx = pan_cx_.load() + ox / z;   // source point under cursor
    const double sy = pan_cy_.load() + oy / z;
    pan_cx_.store(sx - ox / nz);
    pan_cy_.store(sy - oy / nz);
    zoom_.store(nz);
    clampView();
}

void VideoWidget::onPanBy(QPointF delta)
{
    if (!interactive_.load()) return;
    const double z = zoom_.load();
    if (z <= 1.0) return;   // nothing to pan when fit to view

    // Display-pixel delta → normalised source delta (≈ exact when width-bound).
    // Content follows the fingers, so the ROI centre moves opposite the drag.
    const double w = std::max(1, display_->width());
    const double h = std::max(1, display_->height());
    pan_cx_.store(pan_cx_.load() - delta.x() / (w * z));
    pan_cy_.store(pan_cy_.load() - delta.y() / (h * z));
    clampView();
}

void VideoWidget::updateSources(const QStringList& names,
                                const QStringList& identifiers)
{
    QString current_id = source_combo_->currentData().toString();

    source_combo_->blockSignals(true);
    source_combo_->clear();
    for (int i = 0; i < names.size() && i < identifiers.size(); ++i)
        source_combo_->addItem(names[i], identifiers[i]);

    int idx = source_combo_->findData(current_id);
    source_combo_->setCurrentIndex(idx >= 0 ? idx : 0);
    source_combo_->blockSignals(false);

    if (idx < 0)
        onSourceChanged(0);
}

void VideoWidget::deselectThermal()
{
    if (source_combo_->currentData().toString().startsWith("thermal:"))
        source_combo_->setCurrentIndex(0);   // "None" — runs onSourceChanged
}

bool VideoWidget::selectThermalIfIdle()
{
    if (!source_combo_->currentData().toString().isEmpty())
        return false;   // cell already shows something else
    for (int i = 0; i < source_combo_->count(); ++i) {
        if (source_combo_->itemData(i).toString().startsWith("thermal:")) {
            source_combo_->setCurrentIndex(i);   // runs onSourceChanged
            return true;
        }
    }
    return false;   // no thermal source discovered yet
}

void VideoWidget::updateFilters(const QStringList& names)
{
    QString current = filter_combo_->currentText();

    filter_combo_->blockSignals(true);
    filter_combo_->clear();
    for (const auto& name : names)
        filter_combo_->addItem(name);

    int idx = filter_combo_->findText(current);
    filter_combo_->setCurrentIndex(idx >= 0 ? idx : 0);
    filter_combo_->blockSignals(false);
}

void VideoWidget::onSourceChanged(int index)
{
    if (index < 0) return;
    QString source = source_combo_->itemData(index).toString();

    // Unsubscribe from any previous CAPTURE source.
    {
        std::lock_guard<std::mutex> lock(source_mutex_);
        if (!current_capture_id_.empty() && camera_hub_)
            camera_hub_->unsubscribe(current_capture_id_);
        current_capture_id_.clear();
    }

    // Tear down ROS subscriptions + stale frame.
    {
        std::lock_guard<std::mutex> lock(sub_mutex_);
        image_sub_.reset();
        thermal_sub_.reset();
    }
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        latest_ros_frame_ = cv::Mat();
    }

    if (source_type_.load() == SourceType::THERMAL && !source.startsWith("thermal:"))
        emit thermalActiveChanged(false);

    if (source.startsWith("local:") || source.startsWith("gst:") ||
        source.startsWith("av:")) {
        // "av:" is an external A/V SRT source registered with CameraHub by
        // MainWindow (native GStreamer); local:/gst: are hub-owned captures.
        std::string id = source.toStdString();
        if (camera_hub_)
            camera_hub_->subscribe(id);
        {
            std::lock_guard<std::mutex> lock(source_mutex_);
            current_capture_id_ = id;
        }
        source_type_ = SourceType::CAPTURE;
    } else if (source.startsWith("topic:")) {
        std::string topic = source.mid(6).toStdString();
        auto sub = node_->create_subscription<sensor_msgs::msg::Image>(
            topic, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Image::SharedPtr msg) { onImageReceived(msg); });
        {
            std::lock_guard<std::mutex> lock(sub_mutex_);
            image_sub_ = sub;
        }
        source_type_ = SourceType::ROS_TOPIC;
    } else if (source.startsWith("thermal:")) {
        std::string topic = source.mid(8).toStdString();
        auto sub = node_->create_subscription<sensor_msgs::msg::Image>(
            topic, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Image::SharedPtr msg) { onThermalReceived(msg); });
        {
            std::lock_guard<std::mutex> lock(sub_mutex_);
            thermal_sub_ = sub;
        }
        source_type_ = SourceType::THERMAL;
        emit thermalActiveChanged(true);
    } else {
        source_type_ = SourceType::NONE;
        emit thermalActiveChanged(false);
        display_->setPixmap(QPixmap());
        display_->setText("No Source");
    }

    // Filters are enabled only with an active source. Dropping to "No Source"
    // also tears down the running filter + its options UI.
    if (source_type_.load() == SourceType::NONE) {
        filter_combo_->blockSignals(true);
        filter_combo_->setCurrentIndex(0);
        filter_combo_->blockSignals(false);
        filter_combo_->setEnabled(false);
        {
            std::lock_guard<std::mutex> lock(filter_mutex_);
            current_filter_func_ = nullptr;
            filter_config_.reset();
        }
        clearFilterOptions();
    } else {
        filter_combo_->setEnabled(true);
    }
}

void VideoWidget::onFilterChanged(int index)
{
    if (index < 0) return;
    std::string name = filter_combo_->itemText(index).toStdString();

    auto config = std::make_shared<FilterConfig>();
    std::function<cv::Mat(const cv::Mat&)> func;
    if (name != "None")
        func = FilterRegistry::instance().createFilter(name, config);

    {
        std::lock_guard<std::mutex> lock(filter_mutex_);
        current_filter_func_ = func;
        filter_config_ = config;
    }

    setupFilterOptions(name);
}

// ── Filter options UI ───────────────────────────────────────────────────────

void VideoWidget::clearFilterOptions()
{
    delete options_container_;
    options_container_ = nullptr;
}

void VideoWidget::setupFilterOptions(const std::string& name)
{
    clearFilterOptions();

    std::shared_ptr<FilterConfig> config;
    {
        std::lock_guard<std::mutex> lock(filter_mutex_);
        config = filter_config_;
    }
    if (!config || name == "None" || name.empty())
        return;

    options_container_ = new QWidget(this);
    options_container_->setStyleSheet(OPTION_CONTAINER_STYLE);
    options_container_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    auto* opts = new QVBoxLayout(options_container_);
    opts->setContentsMargins(2, 0, 2, 0);
    opts->setSpacing(0);

    if (name == "QR Code") {
        // Decoder backend: zbar (default) | OpenCV. ZBar decodes; OpenCV needs
        // QUIRC linked into the build to decode (apt OpenCV lacks it).
        auto* row = new QHBoxLayout();
        auto* btn_cv   = new QPushButton("OpenCV", options_container_);
        auto* btn_zbar = new QPushButton("zbar",   options_container_);
        btn_cv->setCheckable(true);
        btn_zbar->setCheckable(true);
        btn_zbar->setChecked(true);
        btn_cv->setStyleSheet(TOGGLE_BTN_STYLE);
        btn_zbar->setStyleSheet(TOGGLE_BTN_STYLE);

        auto* grp = new QButtonGroup(options_container_);
        grp->addButton(btn_cv,   0);
        grp->addButton(btn_zbar, 1);
        grp->setExclusive(true);

        connect(grp, QOverload<int>::of(&QButtonGroup::idClicked),
                [config](int id) { config->use_zbar.store(id == 1); });

        row->addWidget(btn_cv);
        row->addWidget(btn_zbar);
        opts->addLayout(row);

    } else if (name == "Hazmat") {
        // Confidence threshold slider.
        auto* row = new QHBoxLayout();
        auto* lbl = new QLabel("Confidence:", options_container_);
        auto* slider = new QSlider(Qt::Horizontal, options_container_);
        auto* val_lbl = new QLabel("25%", options_container_);
        val_lbl->setStyleSheet("color: #4fc3f7; font-size: 11px; min-width: 30px;");
        slider->setRange(1, 100);
        slider->setValue(25);

        connect(slider, &QSlider::valueChanged,
                [config, val_lbl](int v) {
                    config->conf_threshold_pct.store(v);
                    val_lbl->setText(QString::number(v) + "%");
                });

        row->addWidget(lbl);
        row->addWidget(slider, 1);
        row->addWidget(val_lbl);
        opts->addLayout(row);

    } else if (name == "Detect Shape") {
        // Binary threshold slider.
        auto* r1 = new QHBoxLayout();
        auto* t_lbl = new QLabel("Threshold:", options_container_);
        auto* t_sl  = new QSlider(Qt::Horizontal, options_container_);
        auto* t_val = new QLabel("100", options_container_);
        t_val->setStyleSheet("color: #4fc3f7; font-size: 11px; min-width: 30px;");
        t_sl->setRange(0, 255);
        t_sl->setValue(100);
        connect(t_sl, &QSlider::valueChanged,
                [config, t_val](int v) {
                    config->shape_threshold.store(v);
                    t_val->setText(QString::number(v));
                });
        r1->addWidget(t_lbl);
        r1->addWidget(t_sl, 1);
        r1->addWidget(t_val);
        opts->addLayout(r1);

        // Square-tolerance slider.
        auto* r2 = new QHBoxLayout();
        auto* tol_lbl = new QLabel("Tolerance:", options_container_);
        auto* tol_sl  = new QSlider(Qt::Horizontal, options_container_);
        auto* tol_val = new QLabel("0.04", options_container_);
        tol_val->setStyleSheet("color: #4fc3f7; font-size: 11px; min-width: 30px;");
        tol_sl->setRange(1, 100);
        tol_sl->setValue(4);
        connect(tol_sl, &QSlider::valueChanged,
                [config, tol_val](int v) {
                    config->shape_tolerance_pct.store(v);
                    tol_val->setText(QString::number(v * 0.01, 'f', 2));
                });
        r2->addWidget(tol_lbl);
        r2->addWidget(tol_sl, 1);
        r2->addWidget(tol_val);
        opts->addLayout(r2);

        // Detection mode.
        auto* r3 = new QHBoxLayout();
        auto* btn1 = new QPushButton("Mode1", options_container_);
        auto* btn2 = new QPushButton("Mode2", options_container_);
        btn1->setCheckable(true);
        btn2->setCheckable(true);
        btn1->setChecked(true);
        btn1->setStyleSheet(TOGGLE_BTN_STYLE);
        btn2->setStyleSheet(TOGGLE_BTN_STYLE);

        auto* grp = new QButtonGroup(options_container_);
        grp->addButton(btn1, 1);
        grp->addButton(btn2, 2);
        grp->setExclusive(true);

        connect(grp, QOverload<int>::of(&QButtonGroup::idClicked),
                [config](int id) { config->shape_mode.store(id); });

        r3->addWidget(btn1);
        r3->addWidget(btn2);
        opts->addLayout(r3);
    }

    static_cast<QVBoxLayout*>(layout())->addWidget(options_container_);
}

void VideoWidget::onFrameReady(const QImage& image)
{
    if (source_type_.load() == SourceType::NONE || image.isNull())
        return;
    const QSize cell = display_->size();
    const QPixmap pm = QPixmap::fromImage(image);
    switch (aspect_mode_.load()) {
        case 1:  // Stretch: fill the whole cell, ignore source aspect
            display_->setPixmap(
                pm.scaled(cell, Qt::IgnoreAspectRatio, Qt::FastTransformation));
            break;
        case 2:  // Force 4:3
        case 3:  // Force 16:9
        {
            const double r = (aspect_mode_.load() == 2) ? (4.0 / 3.0)
                                                        : (16.0 / 9.0);
            int bw = cell.width();
            int bh = static_cast<int>(bw / r);
            if (bh > cell.height()) {           // box must fit inside the cell
                bh = cell.height();
                bw = static_cast<int>(bh * r);
            }
            // Stretch the source into the chosen-aspect box; QLabel centers it.
            display_->setPixmap(
                pm.scaled(bw, bh, Qt::IgnoreAspectRatio, Qt::FastTransformation));
            break;
        }
        default:  // 0 = Fit: keep source aspect, letterboxed
            display_->setPixmap(
                pm.scaled(cell, Qt::KeepAspectRatio, Qt::FastTransformation));
            break;
    }
}

void VideoWidget::onStatusChanged(const QString& text)
{
    if (source_type_.load() == SourceType::NONE)
        return;
    display_->setPixmap(QPixmap());  // clear any stale frame so the text shows
    display_->setText(text);
}

void VideoWidget::onImageReceived(const sensor_msgs::msg::Image::SharedPtr& msg)
{
    int cv_type = CV_8UC3;
    const auto& enc = msg->encoding;
    if (enc == "mono8")
        cv_type = CV_8UC1;
    else if (enc == "rgba8" || enc == "bgra8")
        cv_type = CV_8UC4;

    cv::Mat raw(msg->height, msg->width, cv_type,
                const_cast<uint8_t*>(msg->data.data()), msg->step);

    cv::Mat bgr;
    if (enc == "rgb8")       cv::cvtColor(raw, bgr, cv::COLOR_RGB2BGR);
    else if (enc == "bgr8")  bgr = raw.clone();
    else if (enc == "mono8") cv::cvtColor(raw, bgr, cv::COLOR_GRAY2BGR);
    else if (enc == "rgba8") cv::cvtColor(raw, bgr, cv::COLOR_RGBA2BGR);
    else if (enc == "bgra8") cv::cvtColor(raw, bgr, cv::COLOR_BGRA2BGR);
    else                     bgr = raw.clone();

    std::lock_guard<std::mutex> lock(frame_mutex_);
    latest_ros_frame_ = std::move(bgr);
}

void VideoWidget::onThermalReceived(const sensor_msgs::msg::Image::SharedPtr& msg)
{
    int rows = static_cast<int>(msg->height);
    int cols = static_cast<int>(msg->width);
    if (rows <= 0 || cols <= 0) return;

    // 32FC1: each pixel is a float32 temperature in °C.
    cv::Mat temp;
    if (msg->encoding == "32FC1") {
        if (msg->data.size() < static_cast<size_t>(rows * cols * 4)) return;
        cv::Mat raw(rows, cols, CV_32FC1,
                    const_cast<uint8_t*>(msg->data.data()), msg->step);
        temp = raw.clone();
    } else {
        cv::Mat raw(rows, cols, CV_8UC1,
                    const_cast<uint8_t*>(msg->data.data()), msg->step);
        raw.convertTo(temp, CV_32F);
    }

    double min_val, max_val;
    cv::minMaxLoc(temp, &min_val, &max_val);

    cv::Mat normalized;
    if (max_val > min_val) {
        temp.convertTo(normalized, CV_8U, 255.0 / (max_val - min_val),
                       -min_val * 255.0 / (max_val - min_val));
    } else {
        normalized = cv::Mat::zeros(rows, cols, CV_8U);
    }

    auto& S = AppSettings::instance();
    cv::Mat colored;
    cv::applyColorMap(normalized, colored, S.thermal_colormap.load());

    int tw = S.thermal_upscale_w.load();
    int th = S.thermal_upscale_h.load();
    if (tw <= 0 || th <= 0) {
        int scale = std::max(1, 480 / rows);
        tw = cols * scale;
        th = rows * scale;
    }
    cv::Mat upscaled;
    cv::resize(colored, upscaled, cv::Size(tw, th), 0, 0, S.thermal_interp.load());

    std::lock_guard<std::mutex> lock(frame_mutex_);
    latest_ros_frame_ = std::move(upscaled);
}

QImage VideoWidget::matToQImage(const cv::Mat& mat)
{
    if (mat.empty()) return QImage();

    if (mat.channels() == 1) {
        return QImage(mat.data, mat.cols, mat.rows,
                      static_cast<int>(mat.step),
                      QImage::Format_Grayscale8).copy();
    }
    cv::Mat rgb;
    if (mat.channels() == 4) {
        cv::cvtColor(mat, rgb, cv::COLOR_BGRA2RGBA);
        return QImage(rgb.data, rgb.cols, rgb.rows,
                      static_cast<int>(rgb.step),
                      QImage::Format_RGBA8888).copy();
    }
    cv::cvtColor(mat, rgb, cv::COLOR_BGR2RGB);
    return QImage(rgb.data, rgb.cols, rgb.rows,
                  static_cast<int>(rgb.step),
                  QImage::Format_RGB888).copy();
}

void VideoWidget::workerLoop()
{
    using clock = std::chrono::steady_clock;
    auto last_status = clock::now() - std::chrono::seconds(5);
    bool warned_no_frame = false;

    while (running_) {
        auto frame_start = clock::now();

        SourceType type = source_type_.load();
        if (paused_.load() || type == SourceType::NONE) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        cv::Mat frame;
        if (type == SourceType::CAPTURE) {
            std::string id;
            {
                std::lock_guard<std::mutex> lock(source_mutex_);
                id = current_capture_id_;
            }
            if (!id.empty() && camera_hub_)
                frame = camera_hub_->getLatestFrame(id);
        } else {  // ROS_TOPIC or THERMAL
            std::lock_guard<std::mutex> lock(frame_mutex_);
            if (!latest_ros_frame_.empty())
                frame = latest_ros_frame_.clone();
        }

        if (frame.empty()) {
            // No frame yet — surface a status (throttled) so the operator sees
            // "Connecting…" rather than a silent black cell.
            if (clock::now() - last_status > std::chrono::milliseconds(1000)) {
                emit statusChanged(type == SourceType::CAPTURE ? "Connecting…"
                                                               : "Waiting for data…");
                last_status = clock::now();
                warned_no_frame = true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            continue;
        }
        warned_no_frame = false;
        (void)warned_no_frame;

        // Apply the zoom/pan view transform BEFORE filtering: crop to the ROI and
        // upscale back to the full frame size, so the filter (and display) see the
        // zoomed region centred and filling the view — exactly what the CV
        // functions expect when a target doesn't fill the original frame.
        const double zoom = interactive_.load() ? zoom_.load() : 1.0;
        if (zoom > 1.001 && frame.cols > 1 && frame.rows > 1) {
            int rw = std::max(1, static_cast<int>(frame.cols / zoom));
            int rh = std::max(1, static_cast<int>(frame.rows / zoom));
            int rx = static_cast<int>(pan_cx_.load() * frame.cols - rw / 2.0);
            int ry = static_cast<int>(pan_cy_.load() * frame.rows - rh / 2.0);
            rx = std::clamp(rx, 0, frame.cols - rw);
            ry = std::clamp(ry, 0, frame.rows - rh);
            cv::Mat zoomed;
            cv::resize(frame(cv::Rect(rx, ry, rw, rh)), zoomed,
                       cv::Size(frame.cols, frame.rows), 0, 0, cv::INTER_LINEAR);
            frame = std::move(zoomed);
        }

        // Run the selected CV filter (if any) on this worker thread.
        std::function<cv::Mat(const cv::Mat&)> filter;
        {
            std::lock_guard<std::mutex> lock(filter_mutex_);
            filter = current_filter_func_;
        }
        if (filter) {
            cv::Mat filtered = filter(frame);
            if (!filtered.empty())
                frame = std::move(filtered);
        }

        emit frameReady(matToQImage(frame));

        auto remaining = std::chrono::milliseconds(33) - (clock::now() - frame_start);
        if (remaining.count() > 0)
            std::this_thread::sleep_for(remaining);
    }
}
