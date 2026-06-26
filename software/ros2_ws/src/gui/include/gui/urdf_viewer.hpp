#pragma once

#include <QOpenGLWidget>
#include <QOpenGLExtraFunctions>
#include <QOpenGLShaderProgram>
#include <QMatrix4x4>
#include <QVector3D>
#include <QVector4D>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QTimer>

#include <QImage>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

class UrdfViewer : public QOpenGLWidget, protected QOpenGLExtraFunctions {
    Q_OBJECT
public:
    explicit UrdfViewer(rclcpp::Node::SharedPtr node, QWidget* parent = nullptr);
    ~UrdfViewer() override;

    // Map mode: render the live /map (OccupancyGrid) as a textured floor and
    // place the robot at its map->base_footprint pose (from TF), instead of the
    // plain twin grid at the origin. Off by default (digital-twin behaviour).
    void setMapMode(bool on);

    // 3-D map mode: render /robot/map3d occupied OctoMap leaves as a single
    // voxel VBO, colored by height, with the robot posed from map->base TF.
    void setOctomapMode(bool on);

    // Initial-pose pick (2-D map mode): when armed, a left click-drag on the map
    // sets the robot's start pose — click = X/Y, drag = yaw (pitch/roll always 0) —
    // and publishes it on /initialpose (map frame) for AMCL / slam_toolbox
    // localization. Camera orbit (left-drag) is suppressed while armed; off by
    // default. RViz "2D Pose Estimate" behaviour.
    void setInitialPoseMode(bool on);

    // Orientation follow: rotate the model's base by the robot's live attitude
    // (the orientation quaternion of an odometry topic) so the twin shows the
    // real behaviour. Camera control is unaffected. If no odometry arrives the
    // model returns to its default (upright) pose. Off by default.
    // Note: /odometry/filtered and /odom/wheel are planar (yaw only); point
    // twin_odom_topic at /zed/zed_node/odom for full roll/pitch/yaw.
    void setFollowOrientation(bool on);

signals:
    // Live during a drag (preview) and once on release (committed). x/y in the
    // map frame (m), yaw in degrees.
    void initialPosePreview(double x, double y, double yaw_deg);
    void initialPosePicked(double x, double y, double yaw_deg);

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
    void mousePressEvent(QMouseEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void mouseReleaseEvent(QMouseEvent* e) override;
    void wheelEvent(QWheelEvent* e) override;

private:
    // Geometry vertex
    struct Vertex {
        float pos[3];
        float normal[3];
    };

    struct VoxelVertex {
        float pos[3];
        float normal[3];
        float color[4];
    };

    // Geometry description (parsed from URDF, no GL resources)
    struct GeomDesc {
        enum Type { BOX, CYLINDER, SPHERE, MESH };
        Type type{BOX};
        float dims[3]{0, 0, 0};
        std::string mesh_uri;
        QMatrix4x4 visual_origin;
        QVector4D color{0.7f, 0.7f, 0.7f, 1.0f};
    };

    // GL renderable object
    struct RenderObject {
        GLuint vao{0}, vbo{0};
        int vertex_count{0};
        QVector4D color{0.7f, 0.7f, 0.7f, 1.0f};
        QMatrix4x4 visual_origin;
        QVector3D aabb_min{0, 0, 0};   // local-space bounds (for map-floor height)
        QVector3D aabb_max{0, 0, 0};
    };

    struct LinkData {
        std::vector<RenderObject> visuals;
        QMatrix4x4 world_transform;
    };

    struct JointData {
        std::string parent_link;
        std::string child_link;
        int type{6}; // urdf::Joint::FIXED
        QMatrix4x4 origin;
        QVector3D axis{0, 0, 1};
        double value{0.0};
    };

    // URDF parsing (runs on ROS thread, stores descriptions)
    void parseUrdf(const std::string& urdf_xml);

    // GL resource creation (runs on main/GL thread)
    void buildGLResources();
    void cleanupGLResources();

    // Forward kinematics
    void computeFKRecursive(const std::string& link_name,
                            const QMatrix4x4& parent_tf,
                            const std::map<std::string, JointData>& joints,
                            const std::map<std::string, std::vector<std::string>>& children);

    // Geometry generators
    RenderObject createBox(float sx, float sy, float sz);
    RenderObject createCylinder(float radius, float length, int segments = 24);
    RenderObject createSphere(float radius, int rings = 12, int sectors = 18);
    RenderObject loadMesh(const std::string& uri, float sx, float sy, float sz);
    void uploadMesh(RenderObject& obj, const std::vector<Vertex>& verts);

    // Grid
    void createGrid();
    GLuint grid_vao_{0}, grid_vbo_{0};
    int grid_vertex_count_{0};

    // ── Map mode: textured occupancy-grid floor + base pose from TF ──────────
    void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);  // ROS thread
    void buildFloorTexture();   // GL thread: (re)upload the floor texture + quad
    void drawFloor(const QMatrix4x4& view, const QMatrix4x4& projection);
    void pollBaseTransform();   // sets base_transform_ from map->base TF
    void ensureMapTfListener();

    bool map_mode_{false};
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    QOpenGLShaderProgram* tex_shader_{nullptr};
    GLuint floor_tex_{0}, floor_vao_{0}, floor_vbo_{0};
    int floor_vertex_count_{0};

    std::mutex map_mutex_;
    QImage map_img_;            // RGBA8888 render of the grid (row 0 = map origin)
    double map_res_{0.05};
    double map_ox_{0.0}, map_oy_{0.0};
    bool map_dirty_{false};
    bool have_map_{false};

    QMatrix4x4 base_transform_; // map->base (identity until the first TF arrives)
    bool have_base_{false};
    float map_floor_z_{-0.15f}; // floor quad z in the map frame (under the model)

    // ── 3-D map mode: occupied OctoMap leaves as colored voxels ──────────────
    void onOctomap(const octomap_msgs::msg::Octomap::SharedPtr msg);  // ROS thread
    void buildVoxelBuffer();  // GL thread: upload pending voxel vertices
    void drawVoxels(const QMatrix4x4& view, const QMatrix4x4& projection,
                    const QVector3D& eye);

    bool octomap_mode_{false};
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
    QOpenGLShaderProgram* voxel_shader_{nullptr};
    GLuint voxel_vao_{0}, voxel_vbo_{0};
    int voxel_vertex_count_{0};

    std::mutex voxel_mutex_;
    std::vector<VoxelVertex> pending_voxels_;
    bool voxels_dirty_{false};
    bool have_voxels_{false};

    // ── Orientation follow (twin attitude from an odometry topic) ────────────
    void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);  // ROS thread
    bool follow_orient_{false};
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr orient_sub_;
    std::mutex orient_mutex_;
    double orient_q_[4]{0.0, 0.0, 0.0, 1.0};  // x, y, z, w
    double orient_time_s_{-1e9};               // receipt time (ROS clock seconds)

    // ROS callbacks
    void onRobotDescription(const std_msgs::msg::String::SharedPtr msg);
    void onJointStates(const sensor_msgs::msg::JointState::SharedPtr msg);

    // ROS
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr desc_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

    // Shader
    QOpenGLShaderProgram* shader_{nullptr};

    // Shared data (protected by mutex)
    std::mutex data_mutex_;
    std::map<std::string, std::vector<GeomDesc>> pending_geoms_;
    std::map<std::string, JointData> joints_;
    std::map<std::string, std::vector<std::string>> link_children_;
    std::string root_link_;
    bool urdf_parsed_{false};
    bool needs_rebuild_{false};

    // GL-only data (main thread only)
    std::map<std::string, LinkData> links_;

    // Camera
    float cam_yaw_{135.0f};
    float cam_pitch_{40.0f};
    float cam_distance_{1.5f};
    QVector3D cam_target_{0, 0, 0.2f};
    QPoint last_mouse_pos_;

    // Build the same view+projection paintGL uses (for screen→world unprojection).
    void cameraMatrices(QMatrix4x4& view, QMatrix4x4& projection) const;
    // Unproject a widget pixel onto the world plane z=plane_z (map frame). Returns
    // false if the ray is parallel to the plane.
    bool worldOnPlane(const QPoint& px, float plane_z, QVector3D& out) const;

    // ── Initial-pose pick (2-D map mode) ─────────────────────────────────────
    bool      initial_pose_mode_{false};
    bool      picking_{false};
    QVector3D pick_start_{0, 0, 0};   // map-frame X/Y of the press (= robot position)
    QVector3D pick_cur_{0, 0, 0};     // map-frame X/Y of the current/release point
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initpose_pub_;

    QTimer* render_timer_;
};
