#include "gui/urdf_viewer.hpp"

#include <urdf/model.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <limits>
#include <memory>

static const char* VERT_SHADER = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform mat3 normalMatrix;
out vec3 FragPos;
out vec3 Normal;
void main() {
    FragPos = vec3(model * vec4(aPos, 1.0));
    Normal = normalize(normalMatrix * aNormal);
    gl_Position = projection * view * vec4(FragPos, 1.0);
}
)";

static const char* FRAG_SHADER = R"(
#version 330 core
in vec3 FragPos;
in vec3 Normal;
uniform vec4 objectColor;
uniform vec3 lightDir;
uniform vec3 viewPos;       // camera eye (for the rim term)
uniform float bodyMul;      // base-colour multiplier (1.0 twin, <1 darkens in map)
uniform float rimStrength;  // 0 in twin; >0 adds a fresnel edge glow in map mode
uniform vec3 rimColor;
out vec4 FragColor;
void main() {
    vec3 norm = normalize(Normal);
    float ambient = 0.35;
    float diff = max(dot(norm, normalize(lightDir)), 0.0);
    float back = max(dot(norm, normalize(-lightDir)), 0.0) * 0.15;
    vec3 color = (ambient + diff + back) * objectColor.rgb * bodyMul;
    // Rim / fresnel: bright on grazing-angle edges → outlines the robot against
    // both light and dark map regions.
    vec3 viewDir = normalize(viewPos - FragPos);
    float rim = pow(1.0 - max(dot(norm, viewDir), 0.0), 3.0);
    color += rimStrength * rim * rimColor;
    FragColor = vec4(color, objectColor.a);
}
)";

static const char* GRID_VERT = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
uniform mat4 view;
uniform mat4 projection;
out float dist;
void main() {
    gl_Position = projection * view * vec4(aPos, 1.0);
    dist = length(aPos.xy);
}
)";

static const char* GRID_FRAG = R"(
#version 330 core
in float dist;
out vec4 FragColor;
void main() {
    float alpha = clamp(1.0 - dist / 3.0, 0.05, 0.3);
    FragColor = vec4(0.5, 0.5, 0.5, alpha);
}
)";

// Textured floor (occupancy grid) — map mode only.
static const char* TEX_VERT = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec2 aUV;
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
out vec2 UV;
void main() {
    UV = aUV;
    gl_Position = projection * view * model * vec4(aPos, 1.0);
}
)";

static const char* TEX_FRAG = R"(
#version 330 core
in vec2 UV;
uniform sampler2D tex;
out vec4 FragColor;
void main() {
    FragColor = texture(tex, UV);
}
)";

// Occupied OctoMap leaves as one colored voxel VBO (3-D map mode only).
static const char* VOXEL_VERT = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec4 aColor;
uniform mat4 view;
uniform mat4 projection;
out vec3 FragPos;
out vec3 Normal;
out vec4 Color;
void main() {
    FragPos = aPos;
    Normal = normalize(aNormal);
    Color = aColor;
    gl_Position = projection * view * vec4(aPos, 1.0);
}
)";

static const char* VOXEL_FRAG = R"(
#version 330 core
in vec3 FragPos;
in vec3 Normal;
in vec4 Color;
uniform vec3 lightDir;
uniform vec3 viewPos;
out vec4 FragColor;
void main() {
    vec3 norm = normalize(Normal);
    float ambient = 0.32;
    float diff = max(dot(norm, normalize(lightDir)), 0.0);
    float back = max(dot(norm, normalize(-lightDir)), 0.0) * 0.12;
    vec3 viewDir = normalize(viewPos - FragPos);
    float rim = pow(1.0 - max(dot(norm, viewDir), 0.0), 2.5) * 0.20;
    vec3 color = Color.rgb * (ambient + diff + back) + rim * vec3(0.85, 0.95, 1.0);
    FragColor = vec4(color, Color.a);
}
)";

static std::string resolveUri(const std::string& uri)
{
    if (uri.size() > 10 && uri.substr(0, 10) == "package://") {
        auto rest = uri.substr(10);
        auto slash = rest.find('/');
        if (slash == std::string::npos) return "";
        auto pkg = rest.substr(0, slash);
        auto path = rest.substr(slash);
        try {
            return ament_index_cpp::get_package_share_directory(pkg) + path;
        } catch (...) {
            return "";
        }
    }
    if (uri.size() > 7 && uri.substr(0, 7) == "file://")
        return uri.substr(7);
    return uri;
}

UrdfViewer::UrdfViewer(rclcpp::Node::SharedPtr node, QWidget* parent)
    : QOpenGLWidget(parent), node_(node)
{
    setMinimumSize(200, 200);

    auto qos = rclcpp::QoS(1).transient_local().reliable();
    desc_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/robot_description", qos,
        [this](std_msgs::msg::String::SharedPtr msg) { onRobotDescription(msg); });

    joint_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::JointState::SharedPtr msg) { onJointStates(msg); });

    render_timer_ = new QTimer(this);
    connect(render_timer_, &QTimer::timeout, this, QOverload<>::of(&QOpenGLWidget::update));
    render_timer_->start(33); // ~30 fps
}

UrdfViewer::~UrdfViewer()
{
    makeCurrent();
    cleanupGLResources();
    if (grid_vao_) { glDeleteVertexArrays(1, &grid_vao_); glDeleteBuffers(1, &grid_vbo_); }
    if (floor_vao_) { glDeleteVertexArrays(1, &floor_vao_); glDeleteBuffers(1, &floor_vbo_); }
    if (floor_tex_) glDeleteTextures(1, &floor_tex_);
    if (voxel_vao_) { glDeleteVertexArrays(1, &voxel_vao_); glDeleteBuffers(1, &voxel_vbo_); }
    delete shader_;
    delete tex_shader_;
    delete voxel_shader_;
    doneCurrent();
}

// ── OpenGL ───────────────────────────────────────────────────────────────────

void UrdfViewer::initializeGL()
{
    initializeOpenGLFunctions();
    glClearColor(0.12f, 0.12f, 0.18f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    shader_ = new QOpenGLShaderProgram(this);
    shader_->addShaderFromSourceCode(QOpenGLShader::Vertex, VERT_SHADER);
    shader_->addShaderFromSourceCode(QOpenGLShader::Fragment, FRAG_SHADER);
    shader_->link();

    tex_shader_ = new QOpenGLShaderProgram(this);
    tex_shader_->addShaderFromSourceCode(QOpenGLShader::Vertex, TEX_VERT);
    tex_shader_->addShaderFromSourceCode(QOpenGLShader::Fragment, TEX_FRAG);
    tex_shader_->link();

    voxel_shader_ = new QOpenGLShaderProgram(this);
    voxel_shader_->addShaderFromSourceCode(QOpenGLShader::Vertex, VOXEL_VERT);
    voxel_shader_->addShaderFromSourceCode(QOpenGLShader::Fragment, VOXEL_FRAG);
    voxel_shader_->link();

    createGrid();
}

void UrdfViewer::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);
}

void UrdfViewer::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Snapshot shared data
    bool rebuild = false;
    bool parsed = false;
    std::map<std::string, JointData> joints_snap;
    std::string root;
    std::map<std::string, std::vector<std::string>> children;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        rebuild = needs_rebuild_;
        parsed = urdf_parsed_;
        if (parsed) {
            joints_snap = joints_;
            root = root_link_;
            children = link_children_;
        }
    }

    // View & projection matrices
    float yaw_r = qDegreesToRadians(cam_yaw_);
    float pitch_r = qDegreesToRadians(cam_pitch_);
    QVector3D eye(
        cam_distance_ * cosf(pitch_r) * cosf(yaw_r),
        cam_distance_ * cosf(pitch_r) * sinf(yaw_r),
        cam_distance_ * sinf(pitch_r));
    eye += cam_target_;

    QMatrix4x4 view, projection;
    cameraMatrices(view, projection);

    // Map mode: refresh the robot's map pose + the floor texture (the floor is
    // drawn after FK, once we know the model's lowest point so it sits under it).
    if (map_mode_) {
        pollBaseTransform();
        if (octomap_mode_)
            buildVoxelBuffer();
        else
            buildFloorTexture();
    }

    // Draw grid (twin mode, or map mode before a map has arrived)
    if (grid_vao_ && (!map_mode_ || octomap_mode_ || !have_map_)) {
        // Use a separate simple shader for grid — reuse main shader with identity model
        shader_->bind();
        QMatrix4x4 identity;
        shader_->setUniformValue("model", identity);
        shader_->setUniformValue("view", view);
        shader_->setUniformValue("projection", projection);
        shader_->setUniformValue("normalMatrix", identity.normalMatrix());
        shader_->setUniformValue("objectColor", QVector4D(0.4f, 0.4f, 0.4f, 0.25f));
        shader_->setUniformValue("lightDir", QVector3D(0.5f, 0.3f, 1.0f));
        shader_->setUniformValue("bodyMul", 1.0f);     // keep grid at full colour
        shader_->setUniformValue("rimStrength", 0.0f);
        glBindVertexArray(grid_vao_);
        glDrawArrays(GL_LINES, 0, grid_vertex_count_);
        shader_->release();
    }

    if (!parsed) {
        // No URDF yet: still show whichever map product has arrived.
        if (octomap_mode_ && have_voxels_)
            drawVoxels(view, projection, eye);
        else if (map_mode_ && have_map_)
            drawFloor(view, projection);
        return;
    }

    if (rebuild) {
        buildGLResources();
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            needs_rebuild_ = false;
        }
    }

    // Choose the FK root transform:
    //  • map mode → the robot's full map->base pose (set in pollBaseTransform);
    //  • follow-orientation → rotate the base by the live IMU attitude (no
    //    translation), reverting to upright if the data is stale/absent;
    //  • otherwise → identity (the plain twin at the origin).
    QMatrix4x4 root_tf;   // identity
    if (map_mode_) {
        root_tf = base_transform_;
    } else if (follow_orient_) {
        double qx, qy, qz, qw, t;
        {
            std::lock_guard<std::mutex> lk(orient_mutex_);
            qx = orient_q_[0]; qy = orient_q_[1]; qz = orient_q_[2]; qw = orient_q_[3];
            t = orient_time_s_;
        }
        const double n2 = qx * qx + qy * qy + qz * qz + qw * qw;
        if (n2 > 0.5 && node_->now().seconds() - t < 0.5)   // valid + fresh → follow
            root_tf.rotate(QQuaternion(static_cast<float>(qw), static_cast<float>(qx),
                                       static_cast<float>(qy), static_cast<float>(qz)));
    }
    computeFKRecursive(root, root_tf, joints_snap, children);

    // Map mode: place the floor just under the model's lowest world point (so the
    // textured grid sits below the robot instead of cutting through it), then draw.
    if (map_mode_ && !octomap_mode_ && have_map_) {
        float min_z = std::numeric_limits<float>::max();
        for (auto& [name, ld] : links_) {
            for (auto& obj : ld.visuals) {
                if (!obj.vao) continue;
                QMatrix4x4 m = ld.world_transform * obj.visual_origin;
                const QVector3D& a = obj.aabb_min;
                const QVector3D& b = obj.aabb_max;
                for (int i = 0; i < 8; ++i) {
                    QVector3D c((i & 1) ? b.x() : a.x(),
                                (i & 2) ? b.y() : a.y(),
                                (i & 4) ? b.z() : a.z());
                    min_z = std::min(min_z, m.map(c).z());
                }
            }
        }
        if (min_z < std::numeric_limits<float>::max())
            map_floor_z_ = min_z - 0.01f;   // 1 cm clearance under the lowest point
        drawFloor(view, projection);
    }

    if (octomap_mode_ && have_voxels_)
        drawVoxels(view, projection, eye);

    // Render links
    shader_->bind();
    shader_->setUniformValue("view", view);
    shader_->setUniformValue("projection", projection);
    shader_->setUniformValue("lightDir", QVector3D(0.5f, 0.3f, 1.0f));
    shader_->setUniformValue("viewPos", eye);
    // In map mode: darken the body and add a cyan rim glow so the robot reads
    // clearly over the grayscale map. Twin mode is unchanged (no rim, full body).
    shader_->setUniformValue("bodyMul", map_mode_ ? 0.55f : 1.0f);
    shader_->setUniformValue("rimStrength", map_mode_ ? 1.6f : 0.0f);
    shader_->setUniformValue("rimColor", QVector3D(0.20f, 0.85f, 1.0f));

    for (auto& [name, ld] : links_) {
        for (auto& obj : ld.visuals) {
            if (!obj.vao || obj.vertex_count == 0) continue;
            QMatrix4x4 model = ld.world_transform * obj.visual_origin;
            shader_->setUniformValue("model", model);
            shader_->setUniformValue("normalMatrix", model.normalMatrix());
            shader_->setUniformValue("objectColor", obj.color);
            glBindVertexArray(obj.vao);
            glDrawArrays(GL_TRIANGLES, 0, obj.vertex_count);
        }
    }
    shader_->release();
}

// ── Mouse ────────────────────────────────────────────────────────────────────

void UrdfViewer::mousePressEvent(QMouseEvent* e)
{
    last_mouse_pos_ = e->pos();
    // Initial-pose pick: left-press in 2-D map mode starts the pose (X/Y).
    if (initial_pose_mode_ && map_mode_ && !octomap_mode_ && (e->button() == Qt::LeftButton)) {
        QVector3D w;
        if (worldOnPlane(e->pos(), map_floor_z_, w)) {
            pick_start_ = w;
            pick_cur_ = w;
            picking_ = true;
            emit initialPosePreview(w.x(), w.y(), 0.0);
        }
        return;
    }
}

void UrdfViewer::mouseMoveEvent(QMouseEvent* e)
{
    // Initial-pose pick: dragging sets yaw from the press point toward the cursor.
    if (picking_) {
        QVector3D w;
        if (worldOnPlane(e->pos(), map_floor_z_, w)) {
            pick_cur_ = w;
            double yaw = std::atan2(pick_cur_.y() - pick_start_.y(),
                                    pick_cur_.x() - pick_start_.x());
            emit initialPosePreview(pick_start_.x(), pick_start_.y(),
                                    qRadiansToDegrees(yaw));
        }
        return;
    }

    QPoint delta = e->pos() - last_mouse_pos_;
    if (e->buttons() & Qt::LeftButton) {
        cam_yaw_ -= delta.x() * 0.4f;
        cam_pitch_ += delta.y() * 0.4f;
        cam_pitch_ = qBound(-89.0f, cam_pitch_, 89.0f);
    } else if (e->buttons() & Qt::MiddleButton) {
        float yaw_r = qDegreesToRadians(cam_yaw_);
        QVector3D right(-sinf(yaw_r), cosf(yaw_r), 0);
        QVector3D up(0, 0, 1);
        cam_target_ -= right * delta.x() * 0.002f * cam_distance_;
        cam_target_ += up * delta.y() * 0.002f * cam_distance_;
    }
    last_mouse_pos_ = e->pos();
    update();
}

void UrdfViewer::mouseReleaseEvent(QMouseEvent* e)
{
    if (!picking_ || e->button() != Qt::LeftButton) return;
    picking_ = false;

    const double yaw = std::atan2(pick_cur_.y() - pick_start_.y(),
                                  pick_cur_.x() - pick_start_.x());
    if (initpose_pub_) {
        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.frame_id = "map";
        msg.header.stamp = node_->now();
        msg.pose.pose.position.x = pick_start_.x();
        msg.pose.pose.position.y = pick_start_.y();
        msg.pose.pose.position.z = 0.0;
        msg.pose.pose.orientation.z = std::sin(yaw * 0.5);   // yaw-only (roll/pitch=0)
        msg.pose.pose.orientation.w = std::cos(yaw * 0.5);
        // AMCL-style default covariance (x, y, yaw); rest left at 0.
        msg.pose.covariance[0] = 0.25;     // x
        msg.pose.covariance[7] = 0.25;     // y
        msg.pose.covariance[35] = 0.06853; // yaw
        initpose_pub_->publish(msg);
    }
    emit initialPosePicked(pick_start_.x(), pick_start_.y(), qRadiansToDegrees(yaw));
}

void UrdfViewer::setInitialPoseMode(bool on)
{
    initial_pose_mode_ = on;
    picking_ = false;
    if (on && !initpose_pub_) {
        initpose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", rclcpp::QoS(1).reliable());
    }
    setCursor(on ? Qt::CrossCursor : Qt::ArrowCursor);
}

void UrdfViewer::cameraMatrices(QMatrix4x4& view, QMatrix4x4& projection) const
{
    float yaw_r = qDegreesToRadians(cam_yaw_);
    float pitch_r = qDegreesToRadians(cam_pitch_);
    QVector3D eye(
        cam_distance_ * cosf(pitch_r) * cosf(yaw_r),
        cam_distance_ * cosf(pitch_r) * sinf(yaw_r),
        cam_distance_ * sinf(pitch_r));
    eye += cam_target_;
    view.setToIdentity();
    view.lookAt(eye, cam_target_, QVector3D(0, 0, 1));
    projection.setToIdentity();
    float aspect = width() > 0 ? float(width()) / height() : 1.0f;
    projection.perspective(45.0f, aspect, 0.01f, 100.0f);
}

bool UrdfViewer::worldOnPlane(const QPoint& px, float plane_z, QVector3D& out) const
{
    QMatrix4x4 view, projection;
    cameraMatrices(view, projection);
    bool ok = false;
    QMatrix4x4 inv = (projection * view).inverted(&ok);
    if (!ok) return false;

    const float w = width() > 0 ? float(width()) : 1.0f;
    const float h = height() > 0 ? float(height()) : 1.0f;
    const float xn = 2.0f * px.x() / w - 1.0f;
    const float yn = 1.0f - 2.0f * px.y() / h;   // widget Y is top-down

    QVector4D near_c = inv * QVector4D(xn, yn, -1.0f, 1.0f);
    QVector4D far_c  = inv * QVector4D(xn, yn,  1.0f, 1.0f);
    if (qFuzzyIsNull(near_c.w()) || qFuzzyIsNull(far_c.w())) return false;
    QVector3D p0 = near_c.toVector3DAffine();
    QVector3D p1 = far_c.toVector3DAffine();

    QVector3D dir = p1 - p0;
    if (qFuzzyIsNull(dir.z())) return false;     // ray parallel to the ground plane
    float t = (plane_z - p0.z()) / dir.z();
    if (t < 0.0f) return false;                  // intersection behind the camera
    out = p0 + dir * t;
    return true;
}

void UrdfViewer::wheelEvent(QWheelEvent* e)
{
    cam_distance_ *= (e->angleDelta().y() > 0) ? 0.9f : 1.1f;
    cam_distance_ = qBound(0.05f, cam_distance_, 50.0f);
    update();
}

// ── Map mode ─────────────────────────────────────────────────────────────────

void UrdfViewer::setMapMode(bool on)
{
    map_mode_ = on;
    if (on && !map_sub_) {
        ensureMapTfListener();
        auto qos = rclcpp::QoS(1).reliable().transient_local();
        map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", qos,
            [this](nav_msgs::msg::OccupancyGrid::SharedPtr m) { onMap(m); });
        // A higher pitch + further camera reads better for a map-scale scene.
        cam_pitch_ = 55.0f;
        cam_distance_ = 6.0f;
    }
}

void UrdfViewer::setOctomapMode(bool on)
{
    octomap_mode_ = on;
    map_mode_ = on;
    if (on && !octomap_sub_) {
        ensureMapTfListener();
        auto qos = rclcpp::QoS(1).reliable().transient_local();
        octomap_sub_ = node_->create_subscription<octomap_msgs::msg::Octomap>(
            "/robot/map3d", qos,
            [this](octomap_msgs::msg::Octomap::SharedPtr m) { onOctomap(m); });
        cam_pitch_ = 38.0f;
        cam_distance_ = 7.0f;
        cam_target_ = QVector3D(0.0f, 0.0f, 0.8f);
        RCLCPP_INFO(node_->get_logger(),
                    "3D map viewer: subscribed to /robot/map3d");
    }
}

void UrdfViewer::ensureMapTfListener()
{
    if (!tf_buffer_) {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }
}

void UrdfViewer::onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    const int w = static_cast<int>(msg->info.width);
    const int h = static_cast<int>(msg->info.height);
    if (w <= 0 || h <= 0) return;

    // RGBA8888 keeps rows 4-byte aligned for glTexImage2D. Row 0 = grid row 0
    // (map origin / lowest y); we map UV v=0 to that row so it lines up with the
    // quad's -y edge at map origin.
    QImage img(w, h, QImage::Format_RGBA8888);   // byte order R,G,B,A in memory
    for (int row = 0; row < h; ++row) {
        uchar* line = img.scanLine(row);
        for (int col = 0; col < w; ++col) {
            const int8_t v = msg->data[row * w + col];
            uchar r, g, b;
            if (v < 0)        { r = 70;  g = 70;  b = 80;  }   // unknown
            else if (v >= 65) { r = 20;  g = 20;  b = 25;  }   // occupied
            else              { r = 232; g = 232; b = 235; }   // free
            uchar* px = line + col * 4;
            px[0] = r; px[1] = g; px[2] = b; px[3] = 255;
        }
    }

    std::lock_guard<std::mutex> lk(map_mutex_);
    map_img_ = std::move(img);
    map_res_ = msg->info.resolution;
    map_ox_ = msg->info.origin.position.x;
    map_oy_ = msg->info.origin.position.y;
    map_dirty_ = true;
    have_map_ = true;
}

void UrdfViewer::onOctomap(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
    std::unique_ptr<octomap::AbstractOcTree> abstract_tree(octomap_msgs::msgToMap(*msg));
    if (!abstract_tree) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                             "3D map viewer: could not deserialize OctoMap message");
        return;
    }

    auto* tree = dynamic_cast<octomap::OcTree*>(abstract_tree.get());
    if (!tree) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                             "3D map viewer: unsupported OctoMap tree type '%s'",
                             msg->id.c_str());
        return;
    }

    size_t occupied_count = 0;
    double min_z = std::numeric_limits<double>::max();
    double max_z = -std::numeric_limits<double>::max();
    const auto end = tree->end_leafs();
    for (auto it = tree->begin_leafs(); it != end; ++it) {
        if (!tree->isNodeOccupied(*it))
            continue;
        ++occupied_count;
        min_z = std::min(min_z, it.getZ());
        max_z = std::max(max_z, it.getZ());
    }

    std::vector<VoxelVertex> verts;
    if (occupied_count > 0) {
        constexpr size_t kMaxRenderedVoxels = 120000;
        const size_t stride = occupied_count > kMaxRenderedVoxels
            ? (occupied_count + kMaxRenderedVoxels - 1) / kMaxRenderedVoxels
            : 1;
        const size_t reserve_voxels = std::min(occupied_count, kMaxRenderedVoxels);
        verts.reserve(reserve_voxels * 36);

        auto mix = [](float a, float b, float t) {
            return a + (b - a) * t;
        };
        auto color_for_z = [&](double z, float out[4]) {
            const double span = std::max(1e-6, max_z - min_z);
            float t = static_cast<float>(std::clamp((z - min_z) / span, 0.0, 1.0));
            const float low[3] = {0.10f, 0.38f, 0.92f};
            const float mid[3] = {0.10f, 0.72f, 0.48f};
            const float high[3] = {0.96f, 0.52f, 0.14f};
            const bool upper = t > 0.5f;
            const float local_t = upper ? (t - 0.5f) * 2.0f : t * 2.0f;
            const float* a = upper ? mid : low;
            const float* b = upper ? high : mid;
            out[0] = mix(a[0], b[0], local_t);
            out[1] = mix(a[1], b[1], local_t);
            out[2] = mix(a[2], b[2], local_t);
            out[3] = 1.0f;
        };
        auto push_vertex = [&](float x, float y, float z,
                               float nx, float ny, float nz,
                               const float color[4]) {
            VoxelVertex v{};
            v.pos[0] = x; v.pos[1] = y; v.pos[2] = z;
            v.normal[0] = nx; v.normal[1] = ny; v.normal[2] = nz;
            v.color[0] = color[0]; v.color[1] = color[1];
            v.color[2] = color[2]; v.color[3] = color[3];
            verts.push_back(v);
        };
        auto add_face = [&](float nx, float ny, float nz,
                            const QVector3D& a, const QVector3D& b,
                            const QVector3D& c, const QVector3D& d,
                            const float color[4]) {
            push_vertex(a.x(), a.y(), a.z(), nx, ny, nz, color);
            push_vertex(b.x(), b.y(), b.z(), nx, ny, nz, color);
            push_vertex(c.x(), c.y(), c.z(), nx, ny, nz, color);
            push_vertex(c.x(), c.y(), c.z(), nx, ny, nz, color);
            push_vertex(d.x(), d.y(), d.z(), nx, ny, nz, color);
            push_vertex(a.x(), a.y(), a.z(), nx, ny, nz, color);
        };
        auto add_cube = [&](double cx, double cy, double cz, double size,
                            const float color[4]) {
            const float h = static_cast<float>(size * 0.48);
            const float x0 = static_cast<float>(cx) - h;
            const float x1 = static_cast<float>(cx) + h;
            const float y0 = static_cast<float>(cy) - h;
            const float y1 = static_cast<float>(cy) + h;
            const float z0 = static_cast<float>(cz) - h;
            const float z1 = static_cast<float>(cz) + h;

            add_face( 0,  0,  1, {x0, y0, z1}, {x1, y0, z1}, {x1, y1, z1}, {x0, y1, z1}, color);
            add_face( 0,  0, -1, {x0, y1, z0}, {x1, y1, z0}, {x1, y0, z0}, {x0, y0, z0}, color);
            add_face( 1,  0,  0, {x1, y0, z0}, {x1, y1, z0}, {x1, y1, z1}, {x1, y0, z1}, color);
            add_face(-1,  0,  0, {x0, y0, z1}, {x0, y1, z1}, {x0, y1, z0}, {x0, y0, z0}, color);
            add_face( 0,  1,  0, {x0, y1, z1}, {x1, y1, z1}, {x1, y1, z0}, {x0, y1, z0}, color);
            add_face( 0, -1,  0, {x0, y0, z0}, {x1, y0, z0}, {x1, y0, z1}, {x0, y0, z1}, color);
        };

        size_t visited_occupied = 0;
        size_t rendered_voxels = 0;
        for (auto it = tree->begin_leafs(); it != end; ++it) {
            if (!tree->isNodeOccupied(*it))
                continue;
            if ((visited_occupied++ % stride) != 0)
                continue;
            if (rendered_voxels >= kMaxRenderedVoxels)
                break;
            float color[4];
            color_for_z(it.getZ(), color);
            add_cube(it.getX(), it.getY(), it.getZ(), it.getSize(), color);
            ++rendered_voxels;
        }

        if (occupied_count > kMaxRenderedVoxels) {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 5000,
                "3D map viewer: rendering %zu/%zu occupied voxels (sampled for UI safety)",
                rendered_voxels, occupied_count);
        }
    }

    {
        std::lock_guard<std::mutex> lk(voxel_mutex_);
        pending_voxels_ = std::move(verts);
        voxels_dirty_ = true;
    }
    update();
}

void UrdfViewer::buildFloorTexture()
{
    QImage img; double res, ox, oy;
    {
        std::lock_guard<std::mutex> lk(map_mutex_);
        if (!map_dirty_ || map_img_.isNull()) return;
        img = map_img_; res = map_res_; ox = map_ox_; oy = map_oy_;
        map_dirty_ = false;
    }

    if (floor_tex_ == 0) glGenTextures(1, &floor_tex_);
    glBindTexture(GL_TEXTURE_2D, floor_tex_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, img.width(), img.height(), 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, img.constBits());

    // Quad covering the map rect in the map XY plane at local z=0; drawFloor()
    // translates it down to map_floor_z_ (the model's lowest point) at draw time.
    const float x0 = ox, y0 = oy;
    const float x1 = ox + img.width() * res, y1 = oy + img.height() * res;
    const float quad[] = {
        // pos                uv
        x0, y0, 0.0f,         0.0f, 0.0f,
        x1, y0, 0.0f,         1.0f, 0.0f,
        x1, y1, 0.0f,         1.0f, 1.0f,
        x0, y0, 0.0f,         0.0f, 0.0f,
        x1, y1, 0.0f,         1.0f, 1.0f,
        x0, y1, 0.0f,         0.0f, 1.0f,
    };
    floor_vertex_count_ = 6;
    if (floor_vao_ == 0) glGenVertexArrays(1, &floor_vao_);
    if (floor_vbo_ == 0) glGenBuffers(1, &floor_vbo_);
    glBindVertexArray(floor_vao_);
    glBindBuffer(GL_ARRAY_BUFFER, floor_vbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quad), quad, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float),
                          (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);
}

void UrdfViewer::setFollowOrientation(bool on)
{
    follow_orient_ = on;
    if (on && !orient_sub_) {
        // Default to the raw ZED VIO odom — full 6-DOF (roll/pitch/yaw) so the
        // twin shows tilt/climb. The EKF /odometry/filtered + /odom/wheel are
        // planar (yaw only). Overridable without a rebuild:
        //   ros2 run gui gui --ros-args -p twin_odom_topic:=/odometry/filtered
        // Namespace follows the ZED camera_name (default 'zed'), not the model.
        std::string topic = "/zed/zed_node/odom";
        if (!node_->has_parameter("twin_odom_topic"))
            node_->declare_parameter("twin_odom_topic", topic);
        topic = node_->get_parameter("twin_odom_topic").as_string();
        // best_effort subscriber connects to both reliable and best_effort pubs.
        orient_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            topic, rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::SharedPtr m) { onOdom(m); });
        RCLCPP_INFO(node_->get_logger(),
                    "twin: following robot attitude from odom '%s' "
                    "(override with -p twin_odom_topic:=...)", topic.c_str());
    }
}

void UrdfViewer::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::lock_guard<std::mutex> lk(orient_mutex_);
    orient_q_[0] = msg->pose.pose.orientation.x;
    orient_q_[1] = msg->pose.pose.orientation.y;
    orient_q_[2] = msg->pose.pose.orientation.z;
    orient_q_[3] = msg->pose.pose.orientation.w;
    orient_time_s_ = node_->now().seconds();
}

void UrdfViewer::drawFloor(const QMatrix4x4& view, const QMatrix4x4& projection)
{
    if (!(floor_tex_ && floor_vao_ && floor_vertex_count_ > 0)) return;
    QMatrix4x4 model;
    model.translate(0.0f, 0.0f, map_floor_z_);   // drop to the model's lowest point
    glDisable(GL_BLEND);
    tex_shader_->bind();
    tex_shader_->setUniformValue("model", model);
    tex_shader_->setUniformValue("view", view);
    tex_shader_->setUniformValue("projection", projection);
    tex_shader_->setUniformValue("tex", 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, floor_tex_);
    glBindVertexArray(floor_vao_);
    glDrawArrays(GL_TRIANGLES, 0, floor_vertex_count_);
    glBindVertexArray(0);
    tex_shader_->release();
    glEnable(GL_BLEND);
}

void UrdfViewer::buildVoxelBuffer()
{
    std::vector<VoxelVertex> verts;
    {
        std::lock_guard<std::mutex> lk(voxel_mutex_);
        if (!voxels_dirty_)
            return;
        verts = std::move(pending_voxels_);
        voxels_dirty_ = false;
    }

    if (voxel_vao_ == 0) glGenVertexArrays(1, &voxel_vao_);
    if (voxel_vbo_ == 0) glGenBuffers(1, &voxel_vbo_);

    glBindVertexArray(voxel_vao_);
    glBindBuffer(GL_ARRAY_BUFFER, voxel_vbo_);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(verts.size() * sizeof(VoxelVertex)),
                 verts.empty() ? nullptr : verts.data(),
                 GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(VoxelVertex),
                          reinterpret_cast<void*>(offsetof(VoxelVertex, pos)));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(VoxelVertex),
                          reinterpret_cast<void*>(offsetof(VoxelVertex, normal)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, sizeof(VoxelVertex),
                          reinterpret_cast<void*>(offsetof(VoxelVertex, color)));
    glEnableVertexAttribArray(2);
    glBindVertexArray(0);

    voxel_vertex_count_ = static_cast<int>(verts.size());
    have_voxels_ = voxel_vertex_count_ > 0;
}

void UrdfViewer::drawVoxels(const QMatrix4x4& view, const QMatrix4x4& projection,
                            const QVector3D& eye)
{
    if (!(voxel_shader_ && voxel_vao_ && voxel_vertex_count_ > 0))
        return;

    glDisable(GL_BLEND);
    voxel_shader_->bind();
    voxel_shader_->setUniformValue("view", view);
    voxel_shader_->setUniformValue("projection", projection);
    voxel_shader_->setUniformValue("lightDir", QVector3D(0.5f, 0.3f, 1.0f));
    voxel_shader_->setUniformValue("viewPos", eye);
    glBindVertexArray(voxel_vao_);
    glDrawArrays(GL_TRIANGLES, 0, voxel_vertex_count_);
    glBindVertexArray(0);
    voxel_shader_->release();
    glEnable(GL_BLEND);
}

void UrdfViewer::pollBaseTransform()
{
    if (!tf_buffer_) return;
    static const char* bases[] = {"base_footprint", "base_link"};
    for (const char* bf : bases) {
        try {
            auto tf = tf_buffer_->lookupTransform("map", bf, tf2::TimePointZero);
            const auto& t = tf.transform.translation;
            const auto& q = tf.transform.rotation;
            QMatrix4x4 m;
            m.translate(static_cast<float>(t.x), static_cast<float>(t.y),
                        static_cast<float>(t.z));
            m.rotate(QQuaternion(static_cast<float>(q.w), static_cast<float>(q.x),
                                 static_cast<float>(q.y), static_cast<float>(q.z)));
            base_transform_ = m;
            have_base_ = true;
            return;
        } catch (const tf2::TransformException&) {
            // try the next candidate base frame
        }
    }
}

// ── URDF parsing (ROS thread) ───────────────────────────────────────────────

void UrdfViewer::onRobotDescription(const std_msgs::msg::String::SharedPtr msg)
{
    parseUrdf(msg->data);
}

void UrdfViewer::onJointStates(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
        auto it = joints_.find(msg->name[i]);
        if (it != joints_.end())
            it->second.value = msg->position[i];
    }
}

void UrdfViewer::parseUrdf(const std::string& urdf_xml)
{
    urdf::Model model;
    if (!model.initString(urdf_xml)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to parse URDF");
        return;
    }

    std::map<std::string, std::vector<GeomDesc>> geoms;
    std::map<std::string, JointData> jdata;
    std::map<std::string, std::vector<std::string>> children;

    // Process links
    for (const auto& [name, link] : model.links_) {
        std::vector<GeomDesc> link_geoms;

        // Gather visuals — use visual_array, fall back to single visual
        std::vector<urdf::VisualSharedPtr> vis_list = link->visual_array;
        if (vis_list.empty() && link->visual)
            vis_list.push_back(link->visual);

        for (const auto& vis : vis_list) {
            if (!vis || !vis->geometry) continue;
            GeomDesc gd;

            // Origin
            auto& o = vis->origin;
            double r, p, y;
            o.rotation.getRPY(r, p, y);
            gd.visual_origin.translate(
                static_cast<float>(o.position.x),
                static_cast<float>(o.position.y),
                static_cast<float>(o.position.z));
            gd.visual_origin.rotate(static_cast<float>(qRadiansToDegrees(y)), 0, 0, 1);
            gd.visual_origin.rotate(static_cast<float>(qRadiansToDegrees(p)), 0, 1, 0);
            gd.visual_origin.rotate(static_cast<float>(qRadiansToDegrees(r)), 1, 0, 0);

            // Color
            if (vis->material) {
                gd.color = QVector4D(
                    static_cast<float>(vis->material->color.r),
                    static_cast<float>(vis->material->color.g),
                    static_cast<float>(vis->material->color.b),
                    static_cast<float>(vis->material->color.a));
            }

            // Geometry
            auto geom = vis->geometry;
            switch (geom->type) {
            case urdf::Geometry::BOX: {
                auto box = std::static_pointer_cast<urdf::Box>(geom);
                gd.type = GeomDesc::BOX;
                gd.dims[0] = static_cast<float>(box->dim.x);
                gd.dims[1] = static_cast<float>(box->dim.y);
                gd.dims[2] = static_cast<float>(box->dim.z);
                break;
            }
            case urdf::Geometry::CYLINDER: {
                auto cyl = std::static_pointer_cast<urdf::Cylinder>(geom);
                gd.type = GeomDesc::CYLINDER;
                gd.dims[0] = static_cast<float>(cyl->radius);
                gd.dims[1] = static_cast<float>(cyl->length);
                break;
            }
            case urdf::Geometry::SPHERE: {
                auto sph = std::static_pointer_cast<urdf::Sphere>(geom);
                gd.type = GeomDesc::SPHERE;
                gd.dims[0] = static_cast<float>(sph->radius);
                break;
            }
            case urdf::Geometry::MESH: {
                auto mesh = std::static_pointer_cast<urdf::Mesh>(geom);
                gd.type = GeomDesc::MESH;
                gd.mesh_uri = mesh->filename;
                gd.dims[0] = static_cast<float>(mesh->scale.x);
                gd.dims[1] = static_cast<float>(mesh->scale.y);
                gd.dims[2] = static_cast<float>(mesh->scale.z);
                break;
            }
            default: continue;
            }
            link_geoms.push_back(std::move(gd));
        }
        geoms[name] = std::move(link_geoms);
    }

    // Process joints
    for (const auto& [name, joint] : model.joints_) {
        JointData jd;
        jd.parent_link = joint->parent_link_name;
        jd.child_link = joint->child_link_name;
        jd.type = joint->type;

        auto& o = joint->parent_to_joint_origin_transform;
        double r, p, y;
        o.rotation.getRPY(r, p, y);
        jd.origin.translate(
            static_cast<float>(o.position.x),
            static_cast<float>(o.position.y),
            static_cast<float>(o.position.z));
        jd.origin.rotate(static_cast<float>(qRadiansToDegrees(y)), 0, 0, 1);
        jd.origin.rotate(static_cast<float>(qRadiansToDegrees(p)), 0, 1, 0);
        jd.origin.rotate(static_cast<float>(qRadiansToDegrees(r)), 1, 0, 0);

        jd.axis = QVector3D(
            static_cast<float>(joint->axis.x),
            static_cast<float>(joint->axis.y),
            static_cast<float>(joint->axis.z));

        jdata[name] = std::move(jd);
        children[joint->parent_link_name].push_back(name);
    }

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        pending_geoms_ = std::move(geoms);
        joints_ = std::move(jdata);
        link_children_ = std::move(children);
        root_link_ = model.getRoot()->name;
        urdf_parsed_ = true;
        needs_rebuild_ = true;
    }

    RCLCPP_INFO(node_->get_logger(), "URDF loaded: %s (%zu links, %zu joints)",
                model.getName().c_str(), model.links_.size(), model.joints_.size());
}

// ── GL resource building (main thread) ──────────────────────────────────────

void UrdfViewer::buildGLResources()
{
    std::map<std::string, std::vector<GeomDesc>> geom_copy;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        geom_copy = std::move(pending_geoms_);
    }

    cleanupGLResources();

    for (auto& [link_name, descs] : geom_copy) {
        LinkData ld;
        for (auto& gd : descs) {
            RenderObject obj;
            switch (gd.type) {
            case GeomDesc::BOX:
                obj = createBox(gd.dims[0], gd.dims[1], gd.dims[2]);
                break;
            case GeomDesc::CYLINDER:
                obj = createCylinder(gd.dims[0], gd.dims[1]);
                break;
            case GeomDesc::SPHERE:
                obj = createSphere(gd.dims[0]);
                break;
            case GeomDesc::MESH:
                obj = loadMesh(gd.mesh_uri, gd.dims[0], gd.dims[1], gd.dims[2]);
                break;
            }
            obj.color = gd.color;
            obj.visual_origin = gd.visual_origin;
            ld.visuals.push_back(std::move(obj));
        }
        links_[link_name] = std::move(ld);
    }
}

void UrdfViewer::cleanupGLResources()
{
    for (auto& [name, ld] : links_) {
        for (auto& obj : ld.visuals) {
            if (obj.vao) glDeleteVertexArrays(1, &obj.vao);
            if (obj.vbo) glDeleteBuffers(1, &obj.vbo);
        }
    }
    links_.clear();
}

// ── Forward kinematics ──────────────────────────────────────────────────────

void UrdfViewer::computeFKRecursive(
    const std::string& link_name,
    const QMatrix4x4& parent_tf,
    const std::map<std::string, JointData>& joints,
    const std::map<std::string, std::vector<std::string>>& children)
{
    auto link_it = links_.find(link_name);
    if (link_it != links_.end())
        link_it->second.world_transform = parent_tf;

    auto ch_it = children.find(link_name);
    if (ch_it == children.end()) return;

    for (const auto& joint_name : ch_it->second) {
        auto jt = joints.find(joint_name);
        if (jt == joints.end()) continue;

        const auto& jd = jt->second;
        QMatrix4x4 tf = parent_tf * jd.origin;

        // Apply joint value
        if (jd.type == 1 || jd.type == 2) { // REVOLUTE or CONTINUOUS
            QMatrix4x4 rot;
            rot.rotate(static_cast<float>(qRadiansToDegrees(jd.value)), jd.axis);
            tf *= rot;
        } else if (jd.type == 3) { // PRISMATIC
            QMatrix4x4 trans;
            trans.translate(jd.axis * static_cast<float>(jd.value));
            tf *= trans;
        }

        computeFKRecursive(jd.child_link, tf, joints, children);
    }
}

// ── Geometry generators ─────────────────────────────────────────────────────

void UrdfViewer::uploadMesh(RenderObject& obj, const std::vector<Vertex>& verts)
{
    obj.vertex_count = static_cast<int>(verts.size());

    // Local-space bounding box — used in map mode to drop the floor under the
    // model's lowest world point.
    if (!verts.empty()) {
        QVector3D mn(verts[0].pos[0], verts[0].pos[1], verts[0].pos[2]);
        QVector3D mx = mn;
        for (const auto& v : verts) {
            mn.setX(std::min(mn.x(), v.pos[0]));
            mn.setY(std::min(mn.y(), v.pos[1]));
            mn.setZ(std::min(mn.z(), v.pos[2]));
            mx.setX(std::max(mx.x(), v.pos[0]));
            mx.setY(std::max(mx.y(), v.pos[1]));
            mx.setZ(std::max(mx.z(), v.pos[2]));
        }
        obj.aabb_min = mn;
        obj.aabb_max = mx;
    }

    glGenVertexArrays(1, &obj.vao);
    glGenBuffers(1, &obj.vbo);
    glBindVertexArray(obj.vao);
    glBindBuffer(GL_ARRAY_BUFFER, obj.vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(verts.size() * sizeof(Vertex)),
                 verts.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          reinterpret_cast<void*>(offsetof(Vertex, pos)));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          reinterpret_cast<void*>(offsetof(Vertex, normal)));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);
}

UrdfViewer::RenderObject UrdfViewer::createBox(float sx, float sy, float sz)
{
    float hx = sx * 0.5f, hy = sy * 0.5f, hz = sz * 0.5f;
    // clang-format off
    std::vector<Vertex> v;
    auto quad = [&](float nx, float ny, float nz,
                    float ax, float ay, float az,
                    float bx, float by, float bz,
                    float cx, float cy, float cz,
                    float dx, float dy, float dz) {
        Vertex va{{ax,ay,az},{nx,ny,nz}}, vb{{bx,by,bz},{nx,ny,nz}};
        Vertex vc{{cx,cy,cz},{nx,ny,nz}}, vd{{dx,dy,dz},{nx,ny,nz}};
        v.insert(v.end(), {va, vb, vc, vc, vd, va});
    };
    quad( 0, 0, 1, -hx,-hy, hz,  hx,-hy, hz,  hx, hy, hz, -hx, hy, hz);
    quad( 0, 0,-1, -hx, hy,-hz,  hx, hy,-hz,  hx,-hy,-hz, -hx,-hy,-hz);
    quad( 1, 0, 0,  hx,-hy,-hz,  hx, hy,-hz,  hx, hy, hz,  hx,-hy, hz);
    quad(-1, 0, 0, -hx,-hy, hz, -hx, hy, hz, -hx, hy,-hz, -hx,-hy,-hz);
    quad( 0, 1, 0, -hx, hy, hz,  hx, hy, hz,  hx, hy,-hz, -hx, hy,-hz);
    quad( 0,-1, 0, -hx,-hy,-hz,  hx,-hy,-hz,  hx,-hy, hz, -hx,-hy, hz);
    // clang-format on
    RenderObject obj;
    uploadMesh(obj, v);
    return obj;
}

UrdfViewer::RenderObject UrdfViewer::createCylinder(float radius, float length, int seg)
{
    float hz = length * 0.5f;
    std::vector<Vertex> v;

    for (int i = 0; i < seg; ++i) {
        float a0 = 2.0f * M_PI * i / seg;
        float a1 = 2.0f * M_PI * (i + 1) / seg;
        float c0 = cosf(a0), s0 = sinf(a0);
        float c1 = cosf(a1), s1 = sinf(a1);
        float x0 = radius * c0, y0 = radius * s0;
        float x1 = radius * c1, y1 = radius * s1;

        // Side
        Vertex sa{{x0,y0, hz},{c0,s0,0}}, sb{{x1,y1, hz},{c1,s1,0}};
        Vertex sc{{x1,y1,-hz},{c1,s1,0}}, sd{{x0,y0,-hz},{c0,s0,0}};
        v.insert(v.end(), {sa, sb, sc, sc, sd, sa});

        // Top cap
        Vertex tc{{0,0,hz},{0,0,1}}, ta{{x0,y0,hz},{0,0,1}}, tb{{x1,y1,hz},{0,0,1}};
        v.insert(v.end(), {tc, ta, tb});

        // Bottom cap
        Vertex bc{{0,0,-hz},{0,0,-1}}, ba{{x1,y1,-hz},{0,0,-1}}, bb{{x0,y0,-hz},{0,0,-1}};
        v.insert(v.end(), {bc, ba, bb});
    }

    RenderObject obj;
    uploadMesh(obj, v);
    return obj;
}

UrdfViewer::RenderObject UrdfViewer::createSphere(float radius, int rings, int sectors)
{
    std::vector<Vertex> v;
    for (int r = 0; r < rings; ++r) {
        float phi0 = M_PI * r / rings - M_PI / 2;
        float phi1 = M_PI * (r + 1) / rings - M_PI / 2;
        for (int s = 0; s < sectors; ++s) {
            float th0 = 2.0f * M_PI * s / sectors;
            float th1 = 2.0f * M_PI * (s + 1) / sectors;

            auto pt = [&](float phi, float th) -> Vertex {
                float cp = cosf(phi), sp = sinf(phi);
                float ct = cosf(th), st = sinf(th);
                float x = cp * ct, y = cp * st, z = sp;
                return {{radius*x, radius*y, radius*z}, {x, y, z}};
            };

            Vertex a = pt(phi0, th0), b = pt(phi0, th1);
            Vertex c = pt(phi1, th1), d = pt(phi1, th0);
            v.insert(v.end(), {a, b, c, c, d, a});
        }
    }

    RenderObject obj;
    uploadMesh(obj, v);
    return obj;
}

UrdfViewer::RenderObject UrdfViewer::loadMesh(const std::string& uri,
                                               float sx, float sy, float sz)
{
    std::string path = resolveUri(uri);
    if (path.empty())
        return createBox(0.05f, 0.05f, 0.05f);

    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(path,
        aiProcess_Triangulate | aiProcess_GenNormals | aiProcess_JoinIdenticalVertices);

    if (!scene || !scene->mRootNode || scene->mNumMeshes == 0) {
        RCLCPP_WARN(node_->get_logger(), "Could not load mesh: %s", path.c_str());
        return createBox(0.05f, 0.05f, 0.05f);
    }

    std::vector<Vertex> verts;
    for (unsigned int m = 0; m < scene->mNumMeshes; ++m) {
        const aiMesh* mesh = scene->mMeshes[m];
        for (unsigned int f = 0; f < mesh->mNumFaces; ++f) {
            const aiFace& face = mesh->mFaces[f];
            for (unsigned int i = 0; i < face.mNumIndices; ++i) {
                unsigned int idx = face.mIndices[i];
                Vertex v;
                v.pos[0] = mesh->mVertices[idx].x * sx;
                v.pos[1] = mesh->mVertices[idx].y * sy;
                v.pos[2] = mesh->mVertices[idx].z * sz;
                if (mesh->HasNormals()) {
                    v.normal[0] = mesh->mNormals[idx].x;
                    v.normal[1] = mesh->mNormals[idx].y;
                    v.normal[2] = mesh->mNormals[idx].z;
                } else {
                    v.normal[0] = 0; v.normal[1] = 0; v.normal[2] = 1;
                }
                verts.push_back(v);
            }
        }
    }

    RenderObject obj;
    uploadMesh(obj, verts);
    return obj;
}

// ── Grid ─────────────────────────────────────────────────────────────────────

void UrdfViewer::createGrid()
{
    std::vector<Vertex> v;
    float extent = 3.0f;
    float step = 0.25f;
    Vertex zero{};
    for (float i = -extent; i <= extent; i += step) {
        Vertex a = zero, b = zero, c = zero, d = zero;
        a.pos[0] = i; a.pos[1] = -extent;
        b.pos[0] = i; b.pos[1] =  extent;
        c.pos[0] = -extent; c.pos[1] = i;
        d.pos[0] =  extent; d.pos[1] = i;
        v.insert(v.end(), {a, b, c, d});
    }
    grid_vertex_count_ = static_cast<int>(v.size());

    glGenVertexArrays(1, &grid_vao_);
    glGenBuffers(1, &grid_vbo_);
    glBindVertexArray(grid_vao_);
    glBindBuffer(GL_ARRAY_BUFFER, grid_vbo_);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(v.size() * sizeof(Vertex)),
                 v.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          reinterpret_cast<void*>(offsetof(Vertex, pos)));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          reinterpret_cast<void*>(offsetof(Vertex, normal)));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);
}
