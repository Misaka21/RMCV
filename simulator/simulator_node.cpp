//
// Simulator Node - ROS2模拟器接入实现
// 使用 message_filters 同步图像和云台姿态
//

#include "simulator_node.hpp"

#include <atomic>
#include <chrono>
#include <mutex>
#include <queue>
#include <thread>

#include <Eigen/Geometry>
#include <fmt/format.h>
#include <opencv2/core/mat.hpp>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

// Project
#include "hardware/hardware_node.hpp"
#include "plugin/debug/logger.hpp"
#include "plugin/param/static_config.hpp"
#include "plugin/stats/fps_stats.hpp"
#include "umt/umt.hpp"

namespace simulator {

using namespace std::chrono_literals;
using TimePoint = std::chrono::steady_clock::time_point;

// 同步后的帧数据
struct SyncedData {
    cv::Mat image;
    Eigen::Quaterniond q_gimbal;
    TimePoint timestamp;
    double ros_time_sec = 0;  // ROS 时间戳 (秒)
};

// ============================================================================
// ROS2 Synchronized Subscriber Node
// ============================================================================

class SimulatorSubscriber : public rclcpp::Node {
public:
    using ImageMsg = sensor_msgs::msg::Image;
    using PoseMsg = geometry_msgs::msg::PoseStamped;
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<ImageMsg, PoseMsg>;

    explicit SimulatorSubscriber(const SimulatorConfig& config)
        : Node("simulator_subscriber")
        , config_(config)
    {
        // QoS 设置
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        qos_profile.depth = 10;

        // 创建 message_filters 订阅器
        image_sub_.subscribe(this, config.image_topic, qos_profile);
        gimbal_sub_.subscribe(this, config.gimbal_pose_topic, qos_profile);

        // 创建同步器 (允许 50ms 时间差)
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), image_sub_, gimbal_sub_);

        sync_->registerCallback(std::bind(&SimulatorSubscriber::on_sync, this,
            std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_logger(), "Subscribed (synced) to %s and %s",
            config.image_topic.c_str(), config.gimbal_pose_topic.c_str());
    }

    // 获取同步后的数据
    bool get_synced(SyncedData& out) {
        std::lock_guard<std::mutex> lk(queue_mutex_);
        if (data_queue_.empty()) return false;
        out = std::move(data_queue_.front());
        data_queue_.pop();
        return true;
    }

    size_t queue_size() const {
        std::lock_guard<std::mutex> lk(queue_mutex_);
        return data_queue_.size();
    }

private:
    void on_sync(const ImageMsg::ConstSharedPtr& img_msg,
                 const PoseMsg::ConstSharedPtr& pose_msg)
    {
        try {
            SyncedData data;

            // 图像转换 (使用 toCvShare 避免拷贝)
            auto cv_ptr = cv_bridge::toCvShare(img_msg, "bgr8");
            data.image = cv_ptr->image.clone();  // 必须clone，因为msg会被释放

            // 四元数
            auto& q = pose_msg->pose.orientation;
            data.q_gimbal = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized();

            // 使用 ROS 消息时间戳
            auto& stamp = img_msg->header.stamp;
            data.ros_time_sec = stamp.sec + stamp.nanosec * 1e-9;
            data.timestamp = std::chrono::steady_clock::now();

            // 入队
            {
                std::lock_guard<std::mutex> lk(queue_mutex_);
                // 限制队列大小，丢弃旧帧
                while (data_queue_.size() >= 5) {
                    data_queue_.pop();
                }
                data_queue_.push(std::move(data));
            }

            sync_count_++;
            if (sync_count_ == 1) {
                RCLCPP_INFO(get_logger(), "First synced frame! ros_time=%.3f, q=(%.3f, %.3f, %.3f, %.3f)",
                    data.ros_time_sec, q.x, q.y, q.z, q.w);
            }

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
        }
    }

    SimulatorConfig config_;

    message_filters::Subscriber<ImageMsg> image_sub_;
    message_filters::Subscriber<PoseMsg> gimbal_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    mutable std::mutex queue_mutex_;
    std::queue<SyncedData> data_queue_;

    std::atomic<int> sync_count_{0};
};

// ============================================================================
// Config Loading
// ============================================================================

SimulatorConfig load_config(const std::string& filename) {
    SimulatorConfig cfg;
    try {
        auto toml = static_param::parse_file(filename);

        cfg.image_topic = static_param::get_param<std::string>(toml, "Simulator", "image_topic");
        cfg.gimbal_pose_topic = static_param::get_param<std::string>(toml, "Simulator", "gimbal_pose_topic");

        cfg.robot_id = static_cast<uint8_t>(
            static_param::get_param<int64_t>(toml, "Simulator.serial", "robot_id"));
        cfg.enemy_color = static_cast<uint8_t>(
            static_param::get_param<int64_t>(toml, "Simulator.serial", "enemy_color"));
        cfg.bullet_speed = static_cast<float>(
            static_param::get_param<double>(toml, "Simulator.serial", "bullet_speed"));
        cfg.aim_mode = static_cast<uint8_t>(
            static_param::get_param<int64_t>(toml, "Simulator.serial", "aim_mode"));
        cfg.allow_fire = static_param::get_param<bool>(toml, "Simulator.serial", "allow_fire");

    } catch (const std::exception& e) {
        debug::print(debug::PrintMode::WARNING, "Simulator",
            "Config load failed, using defaults: {}", e.what());
    }
    return cfg;
}

// ============================================================================
// Main Entry
// ============================================================================

void start_simulator_node() {
    if (debug::get_session_path().empty()) {
        debug::init_session();
    }

    debug::print(debug::PrintMode::INFO, "Simulator", "Starting simulator node...");

    // 初始化ROS2
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }

    // 加载配置
    auto config = load_config("simulator.toml");
    debug::print(debug::PrintMode::INFO, "Simulator", "image: {}", config.image_topic);
    debug::print(debug::PrintMode::INFO, "Simulator", "gimbal: {}", config.gimbal_pose_topic);
    debug::print(debug::PrintMode::INFO, "Simulator", "enemy_color: {}", config.enemy_color);

    // 创建订阅节点
    auto node = std::make_shared<SimulatorSubscriber>(config);

    // UMT发布
    umt::Publisher<hardware::SyncFrame> pub("sync_frame");
    auto running = umt::BasicObjManager<bool>::find_or_create("hardware_running", false);

    // 统计
    stats::FpsStats fps_stats("Simulator", "synced");

    int frame_id = 0;

    // spin线程 (使用 MultiThreadedExecutor 提高性能)
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() {
        executor.spin();
    });

    debug::print(debug::PrintMode::INFO, "Simulator", "Waiting for synced data...");

    // 主循环
    while (rclcpp::ok()) {
        SyncedData data;

        if (!node->get_synced(data)) {
            std::this_thread::sleep_for(1ms);
            continue;
        }

        // 构建SyncFrame
        hardware::SyncFrame frame;
        frame.image = std::move(data.image);
        frame.frame_id = frame_id++;
        frame.timestamp = data.timestamp;

        // 四元数转欧拉角 (ZYX顺序)
        Eigen::Vector3d euler = data.q_gimbal.toRotationMatrix().eulerAngles(2, 1, 0);
        constexpr double R2D = 180.0 / M_PI;

        frame.serial_data.yaw = static_cast<float>(euler[0] * R2D);
        frame.serial_data.pitch = static_cast<float>(euler[1] * R2D);
        frame.serial_data.roll = static_cast<float>(euler[2] * R2D);
        frame.serial_valid = true;

        // 填充模拟串口数据
        frame.serial_data.robot_id = config.robot_id;
        frame.serial_data.enemy_color = config.enemy_color;
        frame.serial_data.bullet_speed = config.bullet_speed;
        frame.serial_data.aim_mode = config.aim_mode;
        frame.serial_data.allow_fire = config.allow_fire;

        // 发布
        pub.push(frame);
        running->get() = true;

        fps_stats.update(0, true);
    }

    executor.cancel();
    rclcpp::shutdown();
    if (spin_thread.joinable()) spin_thread.join();
}

} // namespace simulator
