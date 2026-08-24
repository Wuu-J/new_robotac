#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <deque>
#include <filesystem>
#include "pgos/commons.h"
#include "pgos/simple_pgo.h"
#include "interface/srv/save_maps.hpp"
#include <pcl/io/io.h>
#include <fstream>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

struct NodeConfig
{
    std::string cloud_topic = "/lio/body_cloud";
    std::string odom_topic = "/lio/odom";
    std::string map_frame = "map";
    std::string local_frame = "lidar";
};

struct CloudMsg
{
    double t;
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
};
struct OdomMsg
{
    double t;
    nav_msgs::msg::Odometry::ConstSharedPtr pose;
};
struct NodeState
{
    // 手动按时间戳配对（替代 message_filters ApproximateTime 同步器——
    // 实测该同步器运行约 35s 后静默饿死，导致关键帧停录、merged 缺大半）
    std::mutex message_mutex;
    std::deque<CloudMsg> cloud_deque;
    std::deque<OdomMsg> odom_deque;
};

class PGONode : public rclcpp::Node
{
public:
    PGONode() : Node("pgo_node")
    {
        RCLCPP_INFO(this->get_logger(), "PGO node started");
        loadParameters();
        m_pgo = std::make_shared<SimplePGO>(m_pgo_config);
        rclcpp::QoS qos = rclcpp::QoS(10);
        m_cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            m_node_config.cloud_topic, qos, std::bind(&PGONode::cloudCB, this, std::placeholders::_1));
        m_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            m_node_config.odom_topic, qos, std::bind(&PGONode::odomCB, this, std::placeholders::_1));
        m_loop_marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/pgo/loop_markers", 10000);
        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        m_timer = this->create_wall_timer(50ms, std::bind(&PGONode::timerCB, this));
        m_save_map_srv = this->create_service<interface::srv::SaveMaps>("/pgo/save_maps", std::bind(&PGONode::saveMapsCB, this, std::placeholders::_1, std::placeholders::_2));
    }

    void loadParameters()
    {
        this->declare_parameter("config_path", "");
        std::string config_path;
        this->get_parameter<std::string>("config_path", config_path);
        YAML::Node config = YAML::LoadFile(config_path);
        if (!config)
        {
            RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE!");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());
        m_node_config.cloud_topic = config["cloud_topic"].as<std::string>();
        m_node_config.odom_topic = config["odom_topic"].as<std::string>();
        m_node_config.map_frame = config["map_frame"].as<std::string>();
        m_node_config.local_frame = config["local_frame"].as<std::string>();

        m_pgo_config.key_pose_delta_deg = config["key_pose_delta_deg"].as<double>();
        m_pgo_config.key_pose_delta_trans = config["key_pose_delta_trans"].as<double>();
        m_pgo_config.loop_search_radius = config["loop_search_radius"].as<double>();
        m_pgo_config.loop_time_tresh = config["loop_time_tresh"].as<double>();
        m_pgo_config.loop_score_tresh = config["loop_score_tresh"].as<double>();
        m_pgo_config.loop_submap_half_range = config["loop_submap_half_range"].as<int>();
        m_pgo_config.submap_resolution = config["submap_resolution"].as<double>();
        m_pgo_config.min_loop_detect_duration = config["min_loop_detect_duration"].as<double>();
    }
    void cloudCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg)
    {
        std::lock_guard<std::mutex> lock(m_state.message_mutex);
        CloudMsg cm;
        cm.t = cloud_msg->header.stamp.sec + cloud_msg->header.stamp.nanosec * 1e-9;
        cm.cloud = cloud_msg;
        m_state.cloud_deque.push_back(cm);
        while (m_state.cloud_deque.size() > 60)
            m_state.cloud_deque.pop_front();
    }

    void odomCB(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg)
    {
        std::lock_guard<std::mutex> lock(m_state.message_mutex);
        OdomMsg om;
        om.t = odom_msg->header.stamp.sec + odom_msg->header.stamp.nanosec * 1e-9;
        om.pose = odom_msg;
        m_state.odom_deque.push_back(om);
        while (m_state.odom_deque.size() > 120)
            m_state.odom_deque.pop_front();
    }

    void sendBroadCastTF(builtin_interfaces::msg::Time &time)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = m_node_config.map_frame;
        transformStamped.child_frame_id = m_node_config.local_frame;
        transformStamped.header.stamp = time;
        Eigen::Quaterniond q(m_pgo->offsetR());
        V3D t = m_pgo->offsetT();
        transformStamped.transform.translation.x = t.x();
        transformStamped.transform.translation.y = t.y();
        transformStamped.transform.translation.z = t.z();
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        m_tf_broadcaster->sendTransform(transformStamped);
    }

    void publishLoopMarkers(builtin_interfaces::msg::Time &time)
    {
        if (m_loop_marker_pub->get_subscription_count() == 0)
            return;
        if (m_pgo->historyPairs().size() == 0)
            return;

        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker nodes_marker;
        visualization_msgs::msg::Marker edges_marker;
        nodes_marker.header.frame_id = m_node_config.map_frame;
        nodes_marker.header.stamp = time;
        nodes_marker.ns = "pgo_nodes";
        nodes_marker.id = 0;
        nodes_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        nodes_marker.action = visualization_msgs::msg::Marker::ADD;
        nodes_marker.pose.orientation.w = 1.0;
        nodes_marker.scale.x = 0.3;
        nodes_marker.scale.y = 0.3;
        nodes_marker.scale.z = 0.3;
        nodes_marker.color.r = 1.0;
        nodes_marker.color.g = 0.8;
        nodes_marker.color.b = 0.0;
        nodes_marker.color.a = 1.0;

        edges_marker.header.frame_id = m_node_config.map_frame;
        edges_marker.header.stamp = time;
        edges_marker.ns = "pgo_edges";
        edges_marker.id = 1;
        edges_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        edges_marker.action = visualization_msgs::msg::Marker::ADD;
        edges_marker.pose.orientation.w = 1.0;
        edges_marker.scale.x = 0.1;
        edges_marker.color.r = 0.0;
        edges_marker.color.g = 0.8;
        edges_marker.color.b = 0.0;
        edges_marker.color.a = 1.0;

        std::vector<KeyPoseWithCloud> &poses = m_pgo->keyPoses();
        std::vector<std::pair<size_t, size_t>> &pairs = m_pgo->historyPairs();
        for (size_t i = 0; i < pairs.size(); i++)
        {
            size_t i1 = pairs[i].first;
            size_t i2 = pairs[i].second;
            geometry_msgs::msg::Point p1, p2;
            p1.x = poses[i1].t_global.x();
            p1.y = poses[i1].t_global.y();
            p1.z = poses[i1].t_global.z();

            p2.x = poses[i2].t_global.x();
            p2.y = poses[i2].t_global.y();
            p2.z = poses[i2].t_global.z();

            nodes_marker.points.push_back(p1);
            nodes_marker.points.push_back(p2);
            edges_marker.points.push_back(p1);
            edges_marker.points.push_back(p2);
        }

        marker_array.markers.push_back(nodes_marker);
        marker_array.markers.push_back(edges_marker);
        m_loop_marker_pub->publish(marker_array);
    }

    void timerCB()
    {
        CloudWithPose cp;
        {
            std::lock_guard<std::mutex> lock(m_state.message_mutex);
            // 手动配对：取最早的 cloud，在 odom 缓冲里找 |dt|<50ms 的最近者。
            // LIO 在同一回调里发布两者且时间戳一致，此配对等价于精确同步。
            while (!m_state.cloud_deque.empty())
            {
                CloudMsg &cm = m_state.cloud_deque.front();
                bool found = false;
                for (auto &om : m_state.odom_deque)
                {
                    if (std::abs(om.t - cm.t) < 0.05)
                    {
                        cp.cloud = CloudType::Ptr(new CloudType);
                        pcl::fromROSMsg(*cm.cloud, *cp.cloud);
                        cp.pose.setTime(cm.cloud->header.stamp.sec, cm.cloud->header.stamp.nanosec);
                        cp.pose.r = Eigen::Quaterniond(om.pose->pose.pose.orientation.w,
                                                       om.pose->pose.pose.orientation.x,
                                                       om.pose->pose.pose.orientation.y,
                                                       om.pose->pose.pose.orientation.z)
                                        .toRotationMatrix();
                        cp.pose.t = V3D(om.pose->pose.pose.position.x, om.pose->pose.pose.position.y,
                                        om.pose->pose.pose.position.z);
                        found = true;
                        break;
                    }
                }
                m_state.cloud_deque.pop_front();
                if (found)
                    break;
            }
            // 清理远早于当前 cloud 的 odom
            while (!m_state.cloud_deque.empty() && m_state.odom_deque.size() > 2
                   && m_state.odom_deque.front().t < m_state.cloud_deque.front().t - 0.2)
                m_state.odom_deque.pop_front();
        }
        if (!cp.cloud)
            return;
        // 位姿非有限（LIO 退化产物）时跳过，防 NaN 进入 GTSAM 因子图死循环
        if (!cp.pose.t.allFinite() || !cp.pose.r.allFinite())
            return;
        builtin_interfaces::msg::Time cur_time;
        cur_time.sec = cp.pose.sec;
        cur_time.nanosec = cp.pose.nsec;
        if (!m_pgo->addKeyPose(cp))
        {

            sendBroadCastTF(cur_time);
            // 诊断：每 5 秒报一次"收到配对帧但不是关键帧"
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "timerCB: non-key pose (kf=%zu), cur (%.2f, %.2f)",
                                 m_pgo->keyPoses().size(), cp.pose.t.x(), cp.pose.t.y());
            return;
        }

        // 诊断：每 10 个关键帧报一次（验证录制是否持续推进）
        static size_t kf_log_counter = 0;
        if (m_pgo->keyPoses().size() % 10 == 0 || ++kf_log_counter % 50 == 0)
        {
            RCLCPP_INFO(this->get_logger(), "keyframes: %zu, latest (%.2f, %.2f)",
                        m_pgo->keyPoses().size(), cp.pose.t.x(), cp.pose.t.y());
        }

        m_pgo->searchForLoopPairs();

        m_pgo->smoothAndUpdate();

        sendBroadCastTF(cur_time);

        publishLoopMarkers(cur_time);
    }

    void saveMapsCB(const std::shared_ptr<interface::srv::SaveMaps::Request> request, std::shared_ptr<interface::srv::SaveMaps::Response> response)
    {
        if (!std::filesystem::exists(request->file_path))
        {
            response->success = false;
            response->message = request->file_path + " IS NOT EXISTS!";
            return;
        }

        if (m_pgo->keyPoses().size() == 0)
        {
            response->success = false;
            response->message = "NO POSES!";
            return;
        }

        std::filesystem::path p_dir(request->file_path);
        std::filesystem::path patches_dir = p_dir / "patches";
        std::filesystem::path poses_txt_path = p_dir / "poses.txt";
        std::filesystem::path map_path = p_dir / "map.pcd";

        if (request->save_patches)
        {
            if (std::filesystem::exists(patches_dir))
            {
                std::filesystem::remove_all(patches_dir);
            }

            std::filesystem::create_directories(patches_dir);

            if (std::filesystem::exists(poses_txt_path))
            {
                std::filesystem::remove(poses_txt_path);
            }
            RCLCPP_INFO(this->get_logger(), "Patches Path: %s", patches_dir.string().c_str());
        }
        RCLCPP_INFO(this->get_logger(), "SAVE MAP TO %s", map_path.string().c_str());

        std::ofstream txt_file(poses_txt_path);

        CloudType::Ptr ret(new CloudType);
        for (size_t i = 0; i < m_pgo->keyPoses().size(); i++)
        {

            CloudType::Ptr body_cloud = m_pgo->keyPoses()[i].body_cloud;
            if (request->save_patches)
            {
                std::string patch_name = std::to_string(i) + ".pcd";
                std::filesystem::path patch_path = patches_dir / patch_name;
                pcl::io::savePCDFileBinary(patch_path.string(), *body_cloud);
                Eigen::Quaterniond q(m_pgo->keyPoses()[i].r_global);
                V3D t = m_pgo->keyPoses()[i].t_global;
                txt_file << patch_name << " " << t.x() << " " << t.y() << " " << t.z() << " " << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << std::endl;
            }
            CloudType::Ptr world_cloud(new CloudType);
            pcl::transformPointCloud(*body_cloud, *world_cloud, m_pgo->keyPoses()[i].t_global, Eigen::Quaterniond(m_pgo->keyPoses()[i].r_global));
            *ret += *world_cloud;
        }
        txt_file.close();
        pcl::io::savePCDFileBinary(map_path.string(), *ret);
        response->success = true;
        response->message = "SAVE SUCCESS!";
    }

private:
    NodeConfig m_node_config;
    Config m_pgo_config;
    NodeState m_state;
    std::shared_ptr<SimplePGO> m_pgo;
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_loop_marker_pub;
    rclcpp::Service<interface::srv::SaveMaps>::SharedPtr m_save_map_srv;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_cloud_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PGONode>());
    rclcpp::shutdown();
    return 0;
}