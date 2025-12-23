//
// Simulator Node - ROS2模拟器接入实现
//

#include "simulator_node.hpp"

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

#include <Eigen/Geometry>
#include <fmt/format.h>
#include <opencv2/core/mat.hpp>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.h>

// Project
#include "hardware/hardware_node.hpp"
#include "plugin/debug/logger.hpp"
#include "plugin/param/static_config.hpp"
#include "plugin/stats/fps_stats.hpp"
#include "umt/umt.hpp"

namespace simulator {

using namespace std::chrono_literals;
using TimePoint = std::chrono::steady_clock::time_point;

// ============================================================================
// ROS2 Subscriber Node
// ============================================================================

class SimulatorSubscriber : public rclcpp::Node {
public:
    explicit SimulatorSubscriber(const SimulatorConfig& config)
        : Node("simulator_subscriber")
        , config_(config)
        , gimbal_quat_(Eigen::Quaterniond::Identity())
    {
        // 订阅图像
        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            config.image_topic, 10,
            [this](sensor_msgs::msg::Image::SharedPtr msg) {
                on_image(msg);
            });

        // 订阅云台姿态
        gimbal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            config.gimbal_pose_topic, 10,
            [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                on_gimbal_pose(msg);
            });

        RCLCPP_INFO(get_logger(), "Subscribed to %s, %s",
            config.image_topic.c_str(), config.gimbal_pose_topic.c_str());
    }

    // 获取最新图像
    bool get_image(cv::Mat& out, TimePoint& ts) {
        std::lock_guard<std::mutex> lk(image_mutex_);
        if (!image_valid_) return false;
        out = latest_image_.clone();
        ts = image_ts_;
        image_valid_ = false;  // 标记已读取
        return true;
    }

    // 获取云台四元数
    Eigen::Quaterniond get_gimbal_quat() {
        std::lock_guard<std::mutex> lk(gimbal_mutex_);
        return gimbal_quat_;
    }

    bool has_gimbal() const { return gimbal_valid_.load(); }

private:
    void on_image(const sensor_msgs::msg::Image::SharedPtr& msg) {
        try {
            // 模拟器发布的是rgb8，需要转换为bgr8供OpenCV使用
            auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            std::lock_guard<std::mutex> lk(image_mutex_);
            latest_image_ = cv_ptr->image;
            image_ts_ = std::chrono::steady_clock::now();
            image_valid_ = true;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
        }
    }

    void on_gimbal_pose(const geometry_msgs::msg::PoseStamped::SharedPtr& msg) {
        auto& q = msg->pose.orientation;
        std::lock_guard<std::mutex> lk(gimbal_mutex_);
        gimbal_quat_ = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized();
        gimbal_valid_.store(true);
    }

    SimulatorConfig config_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gimbal_sub_;

    std::mutex image_mutex_;
    cv::Mat latest_image_;
    TimePoint image_ts_;
    bool image_valid_ = false;

    std::mutex gimbal_mutex_;
    Eigen::Quaterniond gimbal_quat_;
    std::atomic<bool> gimbal_valid_{false};
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

    // spin线程
    std::thread spin_thread([node]() {
        rclcpp::spin(node);
    });

    debug::print(debug::PrintMode::LOG, "Simulator", "Waiting for data from simulator...");

    // 主循环
    while (rclcpp::ok()) {
        cv::Mat image;
        TimePoint ts;

        if (!node->get_image(image, ts)) {
            std::this_thread::sleep_for(1ms);
            continue;
        }

        // 构建SyncFrame
        hardware::SyncFrame frame;
        frame.image = image;
        frame.frame_id = frame_id++;
        frame.timestamp = ts;

        // 四元数转欧拉角
        if (node->has_gimbal()) {
            auto q = node->get_gimbal_quat();
            Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);  // ZYX
            constexpr double R2D = 180.0 / M_PI;

            frame.serial_data.yaw = static_cast<float>(euler[0] * R2D);
            frame.serial_data.pitch = static_cast<float>(euler[1] * R2D);
            frame.serial_data.roll = static_cast<float>(euler[2] * R2D);
            frame.serial_valid = true;
        }

        // 填充模拟串口数据
        frame.serial_data.robot_id = config.robot_id;
        frame.serial_data.enemy_color = config.enemy_color;
        frame.serial_data.bullet_speed = config.bullet_speed;
        frame.serial_data.aim_mode = config.aim_mode;
        frame.serial_data.allow_fire = config.allow_fire;

        // 发布
        pub.push(frame);
        running->get() = true;

        fps_stats.update(0, frame.serial_valid);
    }

    rclcpp::shutdown();
    if (spin_thread.joinable()) spin_thread.join();
}

} // namespace simulator
