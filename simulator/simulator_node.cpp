//
// Simulator Node - 共享内存 + ROS2 混合方案
// 图像: 从共享内存读取 (高速)
// 云台姿态: 从 ROS2 读取 (数据量小)
//

#include "simulator_node.hpp"

#include <atomic>
#include <chrono>
#include <mutex>
#include <queue>
#include <thread>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <Eigen/Geometry>
#include <fmt/format.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

// ROS2 (仅用于云台姿态)
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

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
// 共享内存读取器 (和模拟器的 shm.rs 对应)
// ============================================================================

constexpr const char* SHM_NAME = "/daedalus_image";

#pragma pack(push, 1)
struct ImageHeader {
    uint32_t width;
    uint32_t height;
    uint64_t frame_id;
    uint64_t timestamp_ns;
    uint32_t data_size;
    uint32_t ready;
};
#pragma pack(pop)

class ShmReader {
public:
    ShmReader() : ptr_(nullptr), size_(0), last_frame_id_(0) {}

    ~ShmReader() { close(); }

    bool open() {
        int fd = shm_open(SHM_NAME, O_RDONLY, 0666);
        if (fd < 0) {
            return false;
        }

        // 先映射头部获取大小
        void* header_ptr = mmap(nullptr, sizeof(ImageHeader), PROT_READ, MAP_SHARED, fd, 0);
        if (header_ptr == MAP_FAILED) {
            ::close(fd);
            return false;
        }

        auto* header = static_cast<ImageHeader*>(header_ptr);
        size_ = sizeof(ImageHeader) + header->data_size;
        munmap(header_ptr, sizeof(ImageHeader));

        // 映射完整内存
        ptr_ = static_cast<uint8_t*>(mmap(nullptr, size_, PROT_READ, MAP_SHARED, fd, 0));
        ::close(fd);

        if (ptr_ == MAP_FAILED) {
            ptr_ = nullptr;
            return false;
        }

        return true;
    }

    void close() {
        if (ptr_) {
            munmap(ptr_, size_);
            ptr_ = nullptr;
        }
    }

    bool isOpen() const { return ptr_ != nullptr; }

    // 读取新帧，返回 BGR 图像
    bool readFrame(cv::Mat& image, uint64_t& timestamp_ns) {
        if (!ptr_) return false;

        auto* header = reinterpret_cast<const ImageHeader*>(ptr_);
        std::atomic_thread_fence(std::memory_order_acquire);

        if (header->ready != 1) return false;

        uint64_t frame_id = header->frame_id;
        if (frame_id <= last_frame_id_) return false;

        uint32_t width = header->width;
        uint32_t height = header->height;
        uint32_t data_size = header->data_size;
        timestamp_ns = header->timestamp_ns;

        // 读取 RGB 图像数据
        const uint8_t* data_ptr = ptr_ + sizeof(ImageHeader);
        cv::Mat rgb(height, width, CV_8UC3);
        std::memcpy(rgb.data, data_ptr, data_size);

        // 检查数据一致性
        std::atomic_thread_fence(std::memory_order_acquire);
        if (header->frame_id != frame_id) {
            return false;
        }

        // RGB -> BGR
        cv::cvtColor(rgb, image, cv::COLOR_RGB2BGR);

        last_frame_id_ = frame_id;
        return true;
    }

    std::pair<uint32_t, uint32_t> getSize() const {
        if (!ptr_) return {0, 0};
        auto* header = reinterpret_cast<const ImageHeader*>(ptr_);
        return {header->width, header->height};
    }

private:
    uint8_t* ptr_;
    size_t size_;
    uint64_t last_frame_id_;
};

// ============================================================================
// ROS2 云台姿态订阅器
// ============================================================================

class GimbalSubscriber : public rclcpp::Node {
public:
    using PoseMsg = geometry_msgs::msg::PoseStamped;

    explicit GimbalSubscriber(const std::string& topic)
        : Node("gimbal_subscriber")
    {
        sub_ = create_subscription<PoseMsg>(
            topic, 10,
            [this](const PoseMsg::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(mutex_);
                auto& q = msg->pose.orientation;
                q_gimbal_ = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized();
                has_data_ = true;
            });

        RCLCPP_INFO(get_logger(), "Subscribed to %s", topic.c_str());
    }

    bool getQuaternion(Eigen::Quaterniond& q) {
        std::lock_guard<std::mutex> lk(mutex_);
        if (!has_data_) return false;
        q = q_gimbal_;
        return true;
    }

private:
    rclcpp::Subscription<PoseMsg>::SharedPtr sub_;
    std::mutex mutex_;
    Eigen::Quaterniond q_gimbal_{Eigen::Quaterniond::Identity()};
    bool has_data_{false};
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

        cfg.bullet_speed = static_cast<float>(
            static_param::get_param<double>(toml, "Simulator.serial", "bullet_speed"));
        cfg.aim_mode = static_cast<uint8_t>(
            static_param::get_param<int64_t>(toml, "Simulator.serial", "aim_mode"));
        cfg.aiming_lock = static_param::get_param<bool>(toml, "Simulator.serial", "aiming_lock");

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

    debug::print(debug::PrintMode::INFO, "Simulator", "Starting simulator node (SHM mode)...");

    // 初始化ROS2 (仅用于云台姿态)
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }

    // 加载配置
    auto config = load_config("simulator.toml");
    debug::print(debug::PrintMode::INFO, "Simulator", "gimbal: {}", config.gimbal_pose_topic);
    debug::print(debug::PrintMode::INFO, "Simulator", "aim_mode: {}", config.aim_mode);

    // 创建共享内存读取器
    ShmReader shm_reader;
    debug::print(debug::PrintMode::INFO, "Simulator", "Opening shared memory: {}", SHM_NAME);

    while (!shm_reader.open()) {
        debug::print(debug::PrintMode::WARNING, "Simulator",
            "Waiting for shared memory... Is the simulator running?");
        std::this_thread::sleep_for(1s);
    }

    auto [width, height] = shm_reader.getSize();
    debug::print(debug::PrintMode::INFO, "Simulator",
        "Shared memory connected! Image size: {}x{}", width, height);

    // 创建云台姿态订阅器
    auto gimbal_node = std::make_shared<GimbalSubscriber>(config.gimbal_pose_topic);

    // UMT发布
    umt::Publisher<hardware::SyncFrame> pub("sync_frame");
    auto running = umt::BasicObjManager<bool>::find_or_create("hardware_running", false);

    // 统计
    stats::FpsStats fps_stats("Simulator", "synced");

    int frame_id = 0;

    // ROS2 spin线程
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(gimbal_node);
    std::thread spin_thread([&executor]() {
        executor.spin();
    });

    debug::print(debug::PrintMode::INFO, "Simulator", "Running... (SHM + ROS2)");

    // 主循环
    while (rclcpp::ok()) {
        cv::Mat image;
        uint64_t timestamp_ns;

        // 从共享内存读取图像
        if (!shm_reader.readFrame(image, timestamp_ns)) {
            // 没有新帧，短暂等待
            std::this_thread::yield();
            continue;
        }

        // 获取云台姿态
        Eigen::Quaterniond q_gimbal;
        if (!gimbal_node->getQuaternion(q_gimbal)) {
            // 还没收到云台数据，使用单位四元数
            q_gimbal = Eigen::Quaterniond::Identity();
        }

        // 构建SyncFrame
        hardware::SyncFrame frame;
        frame.image = std::move(image);
        frame.frame_id = frame_id++;
        frame.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();

        // 四元数转欧拉角 (ZYX顺序) - 新协议输出弧度
        Eigen::Vector3d euler = q_gimbal.toRotationMatrix().eulerAngles(2, 1, 0);

        frame.serial_data.yaw = static_cast<float>(euler[0]);
        frame.serial_data.pitch = static_cast<float>(euler[1]);
        frame.serial_data.roll = static_cast<float>(euler[2]);
        frame.serial_valid = true;

        // 填充模拟串口数据
        frame.serial_data.bullet_speed = config.bullet_speed;
        frame.serial_data.aim_mode = config.aim_mode;
        frame.serial_data.aiming_lock = config.aiming_lock;

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
