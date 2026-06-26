#pragma once

#include <QLabel>
#include <QListWidget>
#include <QMutex>
#include <QPushButton>
#include <QStringList>
#include <QVector>
#include <QWidget>

#include <rclcpp/rclcpp.hpp>

#include <rescue_interfaces/srv/list_maps.hpp>
#include <rescue_interfaces/srv/load_map.hpp>
#include <rescue_interfaces/srv/delete_map.hpp>

// Named map library (Maps tab of the Robot Systems window). Mirrors the arm-pose
// model: the map *data* is owned by the robot (map_manager persists files on the
// Jetson, authoritative via ListMaps); this panel shows the names + a preview
// thumbnail and offers Load (= start localization on the robot) and Delete.
// Saving is initiated from MapWindow (which can grab a live preview); this panel
// reconciles with the robot on Refresh / when shown.
class MapsPanel : public QWidget {
    Q_OBJECT
public:
    explicit MapsPanel(rclcpp::Node::SharedPtr node, QWidget* parent = nullptr);

public slots:
    void refresh();   // call ListMaps (also exposed so the window can refresh on show)

signals:
    void mapsRefreshed();   // ROS thread → Qt thread (results stashed under mutex)
    void resultReady(const QString& message, bool ok);

private slots:
    void onMapsRefreshed();
    void onResultReady(const QString& message, bool ok);
    void onSelectionChanged();
    void onLoad2d();
    void onLoad3d();
    void onLoadBoth();
    void onDelete();

private:
    void load(const QString& kind);
    void setStatus(const QString& text, const QString& color = "#888");

    rclcpp::Node::SharedPtr node_;

    QListWidget* list_{nullptr};
    QPushButton* refresh_btn_{nullptr};
    QPushButton* load2d_btn_{nullptr};
    QPushButton* load3d_btn_{nullptr};
    QPushButton* loadboth_btn_{nullptr};
    QPushButton* delete_btn_{nullptr};
    QLabel*      status_{nullptr};

    // Stash for the latched ListMaps result (written on the ROS thread).
    QMutex        result_mutex_;
    QStringList   pending_names_;
    QVector<bool> pending_has_2d_;
    QVector<bool> pending_has_3d_;

    rclcpp::Client<rescue_interfaces::srv::ListMaps>::SharedPtr  list_cli_;
    rclcpp::Client<rescue_interfaces::srv::LoadMap>::SharedPtr   load_cli_;
    rclcpp::Client<rescue_interfaces::srv::DeleteMap>::SharedPtr delete_cli_;
};
