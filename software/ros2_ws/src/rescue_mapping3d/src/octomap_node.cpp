// rescue_mapping3d — on-robot volumetric mapper.
//
// Subscribes the ZED2 point cloud LOCALLY on the Jetson, transforms it into the
// map frame via the existing TF tree (map->odom->base_footprint), voxel-
// downsamples it, inserts it into an octomap::OcTree, and publishes ONLY the
// compressed binary octree (octomap_msgs/Octomap, tens-to-hundreds of KB) on a
// low-rate LATCHED topic. The raw cloud (~35 MB/s) never gets a remote
// subscriber, so it never crosses the network — the previous 3D-mapping attempts
// died because the cloud was published over DDS to the workstation.
//
// This replaces octomap_server2 (not in apt for Humble) with a small purpose-
// built node over the installed liboctomap, so the bandwidth guardrails are
// structural rather than configuration.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <rescue_interfaces/srv/save_map.hpp>
#include <rescue_interfaces/srv/load_map.hpp>

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>
#include <unordered_set>

class OctomapNode : public rclcpp::Node {
public:
    OctomapNode() : rclcpp::Node("octomap_node")
    {
        cloud_topic_   = declare_parameter<std::string>(
            "cloud_topic", "/zed/zed_node/point_cloud/cloud_registered");
        target_frame_  = declare_parameter<std::string>("target_frame", "map");
        resolution_    = declare_parameter<double>("resolution", 0.10);   // octree leaf, m
        leaf_size_     = declare_parameter<double>("leaf_size", 0.05);     // pre-filter, m
        max_range_     = declare_parameter<double>("max_range", 5.0);      // m (ZED is noisy far out)
        min_range_     = declare_parameter<double>("min_range", 0.3);      // m
        insert_period_ = declare_parameter<double>("insert_period", 0.2);  // s (cap insert rate)
        publish_period_ = declare_parameter<double>("publish_period", 1.0); // s (octree out)
        const double p_hit  = declare_parameter<double>("prob_hit", 0.7);
        const double p_miss = declare_parameter<double>("prob_miss", 0.4);
        const double cl_min = declare_parameter<double>("clamp_min", 0.12);
        const double cl_max = declare_parameter<double>("clamp_max", 0.97);

        tree_ = std::make_unique<octomap::OcTree>(resolution_);
        tree_->setProbHit(p_hit);
        tree_->setProbMiss(p_miss);
        tree_->setClampingThresMin(cl_min);
        tree_->setClampingThresMax(cl_max);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // LATCHED, low-rate: a late-joining workstation gets the latest octree on
        // connect, and it never contends with telemetry.
        auto map_qos = rclcpp::QoS(1).reliable().transient_local();
        octree_pub_ = create_publisher<octomap_msgs::msg::Octomap>("/robot/map3d", map_qos);

        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic_, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::PointCloud2::SharedPtr m) { onCloud(m); });

        pub_timer_ = create_wall_timer(
            std::chrono::duration<double>(publish_period_),
            [this]() { publishOctree(); });

        // Named-map save/load. The octree (with the octomap lib) is written/read
        // here as <maps_dir>/<name>/map3d.bt; map_manager forwards the GUI's
        // /robot/maps/{save,load} 3-D requests to these. Service callbacks run on
        // the same single-threaded executor as onCloud(), so tree_ access is
        // serialized (no extra locking needed).
        const char* home = std::getenv("HOME");
        maps_dir_ = declare_parameter<std::string>(
            "maps_dir", std::string(home ? home : ".") + "/maps");
        save_srv_ = create_service<rescue_interfaces::srv::SaveMap>(
            "/robot/map3d/save",
            [this](const std::shared_ptr<rescue_interfaces::srv::SaveMap::Request> req,
                   std::shared_ptr<rescue_interfaces::srv::SaveMap::Response> resp) {
                onSaveMap(req, resp);
            });
        load_srv_ = create_service<rescue_interfaces::srv::LoadMap>(
            "/robot/map3d/load",
            [this](const std::shared_ptr<rescue_interfaces::srv::LoadMap::Request> req,
                   std::shared_ptr<rescue_interfaces::srv::LoadMap::Response> resp) {
                onLoadMap(req, resp);
            });

        RCLCPP_INFO(get_logger(),
            "octomap_node: cloud=%s target=%s res=%.2fm leaf=%.2fm range=[%.1f,%.1f] "
            "insert<=%.0fHz octree@%.2fHz (latched)",
            cloud_topic_.c_str(), target_frame_.c_str(), resolution_, leaf_size_,
            min_range_, max_range_, 1.0 / insert_period_, 1.0 / publish_period_);
    }

private:
    void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Cap insertion rate independently of the cloud rate (CPU guard).
        const rclcpp::Time now = this->now();
        if (last_insert_.nanoseconds() != 0 &&
            (now - last_insert_).seconds() < insert_period_)
            return;

        // Transform cloud_frame -> target_frame (map). Sensor origin = the cloud
        // frame origin expressed in the target frame.
        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = tf_buffer_->lookupTransform(
                target_frame_, msg->header.frame_id, tf2::TimePointZero);
        } catch (const tf2::TransformException& e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "no TF %s <- %s (%s); is SLAM running?",
                target_frame_.c_str(), msg->header.frame_id.c_str(), e.what());
            return;
        }
        last_insert_ = now;

        const auto& q = tf.transform.rotation;
        const auto& tr = tf.transform.translation;
        const tf2::Matrix3x3 R(tf2::Quaternion(q.x, q.y, q.z, q.w));
        const tf2::Vector3 t(tr.x, tr.y, tr.z);
        const octomap::point3d sensor_origin(
            static_cast<float>(tr.x), static_cast<float>(tr.y), static_cast<float>(tr.z));

        const double inv_leaf = 1.0 / std::max(1e-3, leaf_size_);
        const double max_r2 = max_range_ * max_range_;
        const double min_r2 = min_range_ * min_range_;

        std::unordered_set<uint64_t> seen;       // voxel pre-filter (range in cloud frame)
        octomap::Pointcloud scan;                // points in the target (map) frame

        sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iy(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iz(*msg, "z");
        for (; ix != ix.end(); ++ix, ++iy, ++iz) {
            const float x = *ix, y = *iy, z = *iz;
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
            const double r2 = double(x) * x + double(y) * y + double(z) * z;
            if (r2 < min_r2 || r2 > max_r2) continue;

            // Voxel downsample by quantizing in the (compact) cloud frame.
            const int64_t gx = static_cast<int64_t>(std::floor(x * inv_leaf));
            const int64_t gy = static_cast<int64_t>(std::floor(y * inv_leaf));
            const int64_t gz = static_cast<int64_t>(std::floor(z * inv_leaf));
            const uint64_t key = (static_cast<uint64_t>(gx & 0x1FFFFF)) |
                                 (static_cast<uint64_t>(gy & 0x1FFFFF) << 21) |
                                 (static_cast<uint64_t>(gz & 0x1FFFFF) << 42);
            if (!seen.insert(key).second) continue;

            const tf2::Vector3 pg = R * tf2::Vector3(x, y, z) + t;
            scan.push_back(static_cast<float>(pg.x()),
                           static_cast<float>(pg.y()),
                           static_cast<float>(pg.z()));
        }

        if (scan.size() > 0)
            tree_->insertPointCloud(scan, sensor_origin, max_range_,
                                    /*lazy_eval=*/false, /*discretize=*/true);
    }

    void publishOctree()
    {
        if (tree_->size() == 0) return;
        octomap_msgs::msg::Octomap msg;
        if (!octomap_msgs::binaryMapToMsg(*tree_, msg)) return;  // binary = small
        msg.header.frame_id = target_frame_;
        msg.header.stamp = this->now();
        octree_pub_->publish(msg);
    }

    std::string map3dPath(const std::string& name) const
    {
        return maps_dir_ + "/" + name + "/map3d.bt";
    }

    void onSaveMap(const std::shared_ptr<rescue_interfaces::srv::SaveMap::Request> req,
                   std::shared_ptr<rescue_interfaces::srv::SaveMap::Response> resp)
    {
        if (req->name.empty()) {
            resp->success = false;
            resp->message = "empty map name";
            return;
        }
        if (tree_->size() == 0) {
            resp->success = false;
            resp->message = "octree is empty — nothing to save (is 3-D mapping running?)";
            return;
        }
        const std::string path = map3dPath(req->name);
        std::error_code ec;
        std::filesystem::create_directories(std::filesystem::path(path).parent_path(), ec);
        if (!tree_->writeBinary(path)) {
            resp->success = false;
            resp->message = "octree writeBinary failed: " + path;
            RCLCPP_ERROR(get_logger(), "%s", resp->message.c_str());
            return;
        }
        resp->success = true;
        resp->path = path;
        resp->message = "saved 3-D octree (" + std::to_string(tree_->size()) + " nodes) to " + path;
        RCLCPP_INFO(get_logger(), "%s", resp->message.c_str());
    }

    void onLoadMap(const std::shared_ptr<rescue_interfaces::srv::LoadMap::Request> req,
                   std::shared_ptr<rescue_interfaces::srv::LoadMap::Response> resp)
    {
        const std::string path = map3dPath(req->name);
        if (!std::filesystem::exists(path)) {
            resp->success = false;
            resp->message = "no 3-D map at " + path;
            return;
        }
        auto loaded = std::make_unique<octomap::OcTree>(resolution_);
        if (!loaded->readBinary(path)) {
            resp->success = false;
            resp->message = "octree readBinary failed: " + path;
            RCLCPP_ERROR(get_logger(), "%s", resp->message.c_str());
            return;
        }
        tree_ = std::move(loaded);
        publishOctree();   // push the loaded map to any live 3-D viewer
        resp->success = true;
        resp->message = "loaded 3-D octree (" + std::to_string(tree_->size()) + " nodes) from " + path;
        RCLCPP_INFO(get_logger(), "%s", resp->message.c_str());
    }

    std::string cloud_topic_, target_frame_;
    double resolution_, leaf_size_, max_range_, min_range_, insert_period_, publish_period_;
    std::unique_ptr<octomap::OcTree> tree_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octree_pub_;
    rclcpp::TimerBase::SharedPtr pub_timer_;
    rclcpp::Time last_insert_{0, 0, RCL_ROS_TIME};
    std::string maps_dir_;
    rclcpp::Service<rescue_interfaces::srv::SaveMap>::SharedPtr save_srv_;
    rclcpp::Service<rescue_interfaces::srv::LoadMap>::SharedPtr load_srv_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OctomapNode>());
    rclcpp::shutdown();
    return 0;
}
