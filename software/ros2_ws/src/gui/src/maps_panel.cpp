#include "gui/maps_panel.hpp"
#include "gui/map_thumb.hpp"

#include <QFile>
#include <QHBoxLayout>
#include <QIcon>
#include <QListWidgetItem>
#include <QMessageBox>
#include <QMutexLocker>
#include <QPixmap>
#include <QVBoxLayout>

namespace {
constexpr int kRoleName  = Qt::UserRole;
constexpr int kRoleHas2d = Qt::UserRole + 1;
constexpr int kRoleHas3d = Qt::UserRole + 2;

QString btnStyle(const char* bg, const char* hover, const char* pressed)
{
    return QString(
        "QPushButton { background-color: %1; color: white; padding: 5px; "
        "border: 1px solid %2; border-radius: 3px; }"
        "QPushButton:hover { background-color: %2; }"
        "QPushButton:pressed { background-color: %3; }"
        "QPushButton:disabled { background-color: #2a2a35; color: #666; }")
        .arg(bg, hover, pressed);
}
}  // namespace

MapsPanel::MapsPanel(rclcpp::Node::SharedPtr node, QWidget* parent)
    : QWidget(parent), node_(node)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(10, 10, 10, 10);
    layout->setSpacing(8);

    auto* intro = new QLabel(
        "Saved maps live on the robot. Save a map (with a preview) from the map "
        "window's “Save Map…”; Load starts localization against it — then set the "
        "robot's start point by click-dragging on the 2-D map.", this);
    intro->setWordWrap(true);
    intro->setStyleSheet("color: #777; font-size: 11px;");
    layout->addWidget(intro);

    auto* hdr_row = new QHBoxLayout();
    auto* hdr = new QLabel("Map Library", this);
    hdr->setStyleSheet("color: #aaa; font-weight: bold;");
    refresh_btn_ = new QPushButton("⟳ Refresh", this);
    refresh_btn_->setStyleSheet(btnStyle("#3a3a55", "#4a4a66", "#2a2a45"));
    connect(refresh_btn_, &QPushButton::clicked, this, &MapsPanel::refresh);
    hdr_row->addWidget(hdr);
    hdr_row->addStretch();
    hdr_row->addWidget(refresh_btn_);
    layout->addLayout(hdr_row);

    list_ = new QListWidget(this);
    list_->setIconSize(QSize(120, 90));
    list_->setStyleSheet(
        "QListWidget { background-color: #14141f; color: #c8c8c8; border: 1px solid #333; }"
        "QListWidget::item { padding: 4px; }"
        "QListWidget::item:selected { background-color: #3a3a7a; }");
    connect(list_, &QListWidget::itemSelectionChanged, this, &MapsPanel::onSelectionChanged);
    layout->addWidget(list_, 1);

    auto* btn_row = new QHBoxLayout();
    btn_row->setSpacing(4);
    load2d_btn_ = new QPushButton("Load 2D", this);
    load2d_btn_->setToolTip("Start 2-D localization (AMCL/slam_toolbox) on this map");
    load2d_btn_->setStyleSheet(btnStyle("#1a5a2a", "#2a7a3a", "#0a3a1a"));
    connect(load2d_btn_, &QPushButton::clicked, this, &MapsPanel::onLoad2d);
    load3d_btn_ = new QPushButton("Load 3D", this);
    load3d_btn_->setToolTip("Reload the 3-D OctoMap into the mapping node");
    load3d_btn_->setStyleSheet(btnStyle("#1a5a2a", "#2a7a3a", "#0a3a1a"));
    connect(load3d_btn_, &QPushButton::clicked, this, &MapsPanel::onLoad3d);
    loadboth_btn_ = new QPushButton("Load Both", this);
    loadboth_btn_->setToolTip("Load the 2-D and 3-D maps together (shared frame)");
    loadboth_btn_->setStyleSheet(btnStyle("#2a4a7f", "#3a5a9f", "#1a3a6f"));
    connect(loadboth_btn_, &QPushButton::clicked, this, &MapsPanel::onLoadBoth);
    delete_btn_ = new QPushButton("Delete", this);
    delete_btn_->setStyleSheet(btnStyle("#5a2a2a", "#7a3a3a", "#3a1a1a"));
    connect(delete_btn_, &QPushButton::clicked, this, &MapsPanel::onDelete);
    btn_row->addWidget(load2d_btn_);
    btn_row->addWidget(load3d_btn_);
    btn_row->addWidget(loadboth_btn_);
    btn_row->addStretch();
    btn_row->addWidget(delete_btn_);
    layout->addLayout(btn_row);

    status_ = new QLabel("—", this);
    status_->setWordWrap(true);
    status_->setStyleSheet("color: #888; font-size: 11px;");
    layout->addWidget(status_);

    connect(this, &MapsPanel::mapsRefreshed, this, &MapsPanel::onMapsRefreshed,
            Qt::QueuedConnection);
    connect(this, &MapsPanel::resultReady, this, &MapsPanel::onResultReady,
            Qt::QueuedConnection);

    list_cli_   = node_->create_client<rescue_interfaces::srv::ListMaps>("/robot/maps/list");
    load_cli_   = node_->create_client<rescue_interfaces::srv::LoadMap>("/robot/maps/load");
    delete_cli_ = node_->create_client<rescue_interfaces::srv::DeleteMap>("/robot/maps/delete");

    onSelectionChanged();
    refresh();
}

void MapsPanel::setStatus(const QString& text, const QString& color)
{
    status_->setText(text);
    status_->setStyleSheet(QString("color: %1; font-size: 11px;").arg(color));
}

void MapsPanel::refresh()
{
    if (!list_cli_->service_is_ready()) {
        setStatus("map_manager not connected (is it running on the Jetson?)", "#ccaa00");
        return;
    }
    list_cli_->async_send_request(
        std::make_shared<rescue_interfaces::srv::ListMaps::Request>(),
        [this](rclcpp::Client<rescue_interfaces::srv::ListMaps>::SharedFuture f) {
            auto r = f.get();
            QMutexLocker lk(&result_mutex_);
            pending_names_.clear();
            pending_has_2d_.clear();
            pending_has_3d_.clear();
            for (size_t i = 0; i < r->names.size(); ++i) {
                pending_names_ << QString::fromStdString(r->names[i]);
                pending_has_2d_ << (i < r->has_2d.size() ? r->has_2d[i] : false);
                pending_has_3d_ << (i < r->has_3d.size() ? r->has_3d[i] : false);
            }
            emit mapsRefreshed();
        });
}

void MapsPanel::onMapsRefreshed()
{
    QStringList names;
    QVector<bool> has2d, has3d;
    {
        QMutexLocker lk(&result_mutex_);
        names = pending_names_;
        has2d = pending_has_2d_;
        has3d = pending_has_3d_;
    }
    const QString keep = list_->currentItem()
                             ? list_->currentItem()->data(kRoleName).toString()
                             : QString();
    list_->clear();
    for (int i = 0; i < names.size(); ++i) {
        QStringList badges;
        if (has2d[i]) badges << "2D";
        if (has3d[i]) badges << "3D";
        auto* it = new QListWidgetItem(
            QString("%1   [%2]").arg(names[i], badges.join("+")), list_);
        it->setData(kRoleName, names[i]);
        it->setData(kRoleHas2d, has2d[i]);
        it->setData(kRoleHas3d, has3d[i]);
        // Prefer the 2-D preview thumbnail, fall back to the 3-D one.
        QPixmap pm(mapThumbPath(names[i], has2d[i] ? "2d" : "3d"));
        if (pm.isNull() && has3d[i]) pm = QPixmap(mapThumbPath(names[i], "3d"));
        if (!pm.isNull()) it->setIcon(QIcon(pm));
    }
    if (!keep.isEmpty()) {
        for (int i = 0; i < list_->count(); ++i)
            if (list_->item(i)->data(kRoleName).toString() == keep) {
                list_->setCurrentRow(i);
                break;
            }
    }
    onSelectionChanged();
    setStatus(QString("%1 map(s)").arg(names.size()), "#888");
}

void MapsPanel::onSelectionChanged()
{
    QListWidgetItem* it = list_->currentItem();
    const bool has2d = it && it->data(kRoleHas2d).toBool();
    const bool has3d = it && it->data(kRoleHas3d).toBool();
    load2d_btn_->setEnabled(has2d);
    load3d_btn_->setEnabled(has3d);
    loadboth_btn_->setEnabled(has2d && has3d);
    delete_btn_->setEnabled(it != nullptr);
}

void MapsPanel::load(const QString& kind)
{
    QListWidgetItem* it = list_->currentItem();
    if (!it) { setStatus("no map selected", "#ccaa00"); return; }
    if (!load_cli_->service_is_ready()) {
        setStatus("map_manager load service not available", "#cc6666");
        return;
    }
    const QString name = it->data(kRoleName).toString();
    auto req = std::make_shared<rescue_interfaces::srv::LoadMap::Request>();
    req->name = name.toStdString();
    req->kind = kind.toStdString();
    setStatus(QString("loading '%1' (%2)…").arg(name, kind), "#4fc3f7");
    load_cli_->async_send_request(
        req, [this](rclcpp::Client<rescue_interfaces::srv::LoadMap>::SharedFuture f) {
            auto r = f.get();
            emit resultReady(QString::fromStdString(r->message), r->success);
        });
}

void MapsPanel::onLoad2d()   { load("2d"); }
void MapsPanel::onLoad3d()   { load("3d"); }
void MapsPanel::onLoadBoth() { load("both"); }

void MapsPanel::onDelete()
{
    QListWidgetItem* it = list_->currentItem();
    if (!it) return;
    const QString name = it->data(kRoleName).toString();
    if (QMessageBox::question(this, "Delete map",
                              QString("Delete map '%1' from the robot?").arg(name))
        != QMessageBox::Yes)
        return;
    if (!delete_cli_->service_is_ready()) {
        setStatus("map_manager delete service not available", "#cc6666");
        return;
    }
    auto req = std::make_shared<rescue_interfaces::srv::DeleteMap::Request>();
    req->name = name.toStdString();
    delete_cli_->async_send_request(
        req, [this, name](rclcpp::Client<rescue_interfaces::srv::DeleteMap>::SharedFuture f) {
            auto r = f.get();
            if (r->success) {
                QFile::remove(mapThumbPath(name, "2d"));
                QFile::remove(mapThumbPath(name, "3d"));
            }
            emit resultReady(QString::fromStdString(r->message), r->success);
        });
}

void MapsPanel::onResultReady(const QString& message, bool ok)
{
    setStatus(message, ok ? "#8afa8a" : "#cc6666");
    if (ok) refresh();
}
