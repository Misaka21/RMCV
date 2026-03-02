//
// 相机-IMU 时间戳标定测试程序
//
// 核心思想:
//   相机和串口分别记录独立的时间戳 (steady_clock)
//   通过最小化静止目标在世界系下的位置方差来标定 delta_t
//
// 与之前版本的区别:
//   之前: 从 hardware_node 获取已同步的 SyncFrame (错误!)
//   现在: 直接分别启动相机和串口，记录独立时间戳 (正确!)
//
// 使用方法:
//   1. 将相机对准一个静止的装甲板
//   2. 按 's' 开始采集 (默认5秒)，期间晃动云台
//   3. 采集完成后自动标定
//   4. 结果显示 delta_t，写入 hardware.toml [TimeSync] delta_t_us
//

#include <atomic>
#include <chrono>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

#include <fmt/format.h>
#include <fmt/color.h>
#include <opencv2/opencv.hpp>

#include "hardware/hik_cam/hik_camera.hpp"
#include "hardware/serial/serial_thread.hpp"
#include "aimer/auto_aim/detector/detector_rv/armor_detector.hpp"
#include "aimer/auto_aim/common/types.hpp"
#include "aimer/common/transformer/transformer.hpp"
#include "plugin/param/static_config.hpp"
#include "plugin/debug/logger.hpp"
#include "umt/umt.hpp"

#include "time_sync.hpp"

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

// ============================================================================
// 配置参数
// ============================================================================
constexpr double COLLECT_DURATION_SEC = 10.0;   // 采集时长(秒)
constexpr size_t MIN_CAM_SAMPLES = 50;         // 最少相机数据点
constexpr size_t MAX_IMU_BUFFER = 50000;        // IMU缓冲区最大容量

// ============================================================================
// 带时间戳的IMU数据 (串口接收时记录)
// ============================================================================
struct TimestampedImu {
    TimePoint recv_time;  // 接收时刻 (steady_clock)
    float yaw;
    float pitch;
    float roll;
};

// ============================================================================
// PnP 辅助函数
// ============================================================================

std::optional<Eigen::Vector3d> armor_to_camera_point(const autoaim::DetectedArmor& armor) {
    if (armor.landmarks.size() != 4) {
        return std::nullopt;
    }

    auto object_points = armor.object_points();
    std::vector<cv::Point2f> image_points(armor.landmarks.begin(), armor.landmarks.end());

    const cv::Mat& camera_matrix = aimer::tf::get_camera_matrix();
    const cv::Mat& dist_coeffs = aimer::tf::get_distort_coeffs();

    cv::Mat rvec, tvec;
    bool success = cv::solvePnP(
        object_points,
        image_points,
        camera_matrix,
        dist_coeffs,
        rvec,
        tvec,
        false,
        cv::SOLVEPNP_IPPE
    );

    if (!success) {
        return std::nullopt;
    }

    return Eigen::Vector3d(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
}

// ============================================================================
// 加载外参
// ============================================================================

time_sync::ExtrinsicParams load_extrinsic_from_yaml(const std::string& yaml_file = "camera.yaml") {
    time_sync::ExtrinsicParams params;

    std::string full_path = std::string(CONFIG_DIR) + "/" + yaml_file;
    cv::FileStorage fs(full_path, cv::FileStorage::READ);

    if (!fs.isOpened()) {
        fmt::print(fmt::fg(fmt::color::yellow),
            "警告: 无法打开 {}, 使用默认外参\n", full_path);
        return params;
    }

    // R_gimbal2imubody: Gimbal → Imu
    std::vector<double> r_gimbal_data;
    fs["R_gimbal2imubody"] >> r_gimbal_data;
    if (r_gimbal_data.size() == 9) {
        params.R_gimbal2imu << r_gimbal_data[0], r_gimbal_data[1], r_gimbal_data[2],
                               r_gimbal_data[3], r_gimbal_data[4], r_gimbal_data[5],
                               r_gimbal_data[6], r_gimbal_data[7], r_gimbal_data[8];
    }

    // R_camera2gimbal: Camera → Gimbal
    std::vector<double> r_cam_data;
    fs["R_camera2gimbal"] >> r_cam_data;
    if (r_cam_data.size() == 9) {
        params.R_cam2gimbal << r_cam_data[0], r_cam_data[1], r_cam_data[2],
                               r_cam_data[3], r_cam_data[4], r_cam_data[5],
                               r_cam_data[6], r_cam_data[7], r_cam_data[8];
    }

    return params;
}

// ============================================================================
// 采集状态机
// ============================================================================

enum class State {
    IDLE,       // 等待开始
    COLLECTING, // 正在采集
    CALIBRATING // 正在标定
};

// ============================================================================
// 全局共享数据
// ============================================================================

std::atomic<bool> g_running{true};
std::atomic<State> g_state{State::IDLE};
TimePoint g_collect_start_time;

// IMU 数据缓冲区 (串口线程写入，主线程读取)
std::mutex g_imu_mutex;
std::deque<TimestampedImu> g_imu_buffer;

// 采集的数据 (只在采集期间写入)
std::mutex g_data_mutex;
std::vector<time_sync::CamSample> g_cam_samples;
std::vector<time_sync::ImuSample> g_imu_samples;

// ============================================================================
// 串口接收线程 (记录独立时间戳)
// ============================================================================

void imu_receiver_thread() {
    try {
        // 启动串口通信 (从配置文件读取)
        serial::start_serial_communication();

        // 等待接收队列就绪
        std::this_thread::sleep_for(100ms);

        // 使用 Subscriber 订阅串口数据
        umt::Subscriber<serial::SerialReceiveData> subscriber("serial_receive");

        while (g_running) {
            try {
                serial::SerialReceiveData data = subscriber.pop_for(10);

                // 使用串口线程记录的时间戳 (关键修复!)
                // 之前错误地在队列处理时才记录时间，导致IMU时间戳偏晚
                TimePoint recv_time = Clock::time_point(
                    std::chrono::microseconds(data.recv_time_us)
                );

                // 存入缓冲区
                // 新协议: yaw/pitch/roll 已经是弧度，转换为角度供时间同步
                constexpr double rad2deg = 180.0 / M_PI;
                double yaw_deg = data.yaw * rad2deg;
                double pitch_deg = data.pitch * rad2deg;
                double roll_deg = data.roll * rad2deg;
                {
                    std::lock_guard lock(g_imu_mutex);
                    g_imu_buffer.push_back({recv_time, static_cast<float>(yaw_deg),
                                            static_cast<float>(pitch_deg), static_cast<float>(roll_deg)});

                    // 限制缓冲区大小
                    while (g_imu_buffer.size() > MAX_IMU_BUFFER) {
                        g_imu_buffer.pop_front();
                    }
                }

                // 如果在采集状态，同时存入采集数据
                if (g_state == State::COLLECTING) {
                    std::lock_guard lock(g_data_mutex);
                    g_imu_samples.push_back(
                        time_sync::ImuSample::from_euler_deg(recv_time,
                            static_cast<float>(yaw_deg), static_cast<float>(pitch_deg), static_cast<float>(roll_deg))
                    );
                }
            } catch (const umt::MessageError_Timeout&) {
                // 超时，继续
            }
        }

    } catch (const std::exception& e) {
        fmt::print(fmt::fg(fmt::color::red), "串口线程异常: {}\n", e.what());
    }

    fmt::print("串口线程退出\n");
}

// ============================================================================
// 主程序
// ============================================================================

int main() {
    fmt::print(fmt::fg(fmt::color::gold),
        "==================================================================\n"
        "              相机-IMU 时间戳标定工具 (v2)\n"
        "==================================================================\n"
        "改进: 分别记录相机和IMU的独立时间戳\n"
        "==================================================================\n"
        "操作:\n"
        "  's' - 开始采集 ({:.0f}秒后自动标定)\n"
        "  'c' - 用当前数据标定 (手动模式)\n"
        "  'r' - 重置数据\n"
        "  'q' - 退出\n"
        "==================================================================\n"
        "标定步骤:\n"
        "  1. 将相机对准静止的装甲板\n"
        "  2. 按 's' 开始采集\n"
        "  3. 采集期间缓慢晃动云台 (yaw和pitch都要动)\n"
        "  4. 等待自动标定完成\n"
        "==================================================================\n\n",
        COLLECT_DURATION_SEC
    );

    // 初始化日志
    debug::init_session("test_time_sync");

    // 加载配置
    auto config = static_param::parse_file("hardware.toml");

    // IMU符号配置 (标定时使用原始数据，不需要修正)
    // 注: 最终运行时由 hardware_node 应用符号修正

    // 初始化TF模块 (加载相机内参)
    if (!aimer::tf::init()) {
        fmt::print(fmt::fg(fmt::color::red), "TF模块初始化失败!\n");
        return 1;
    }

    // 加载外参
    auto extrinsic = load_extrinsic_from_yaml();
    fmt::print("外参加载完成\n");

    // 启动串口线程 (独立记录IMU时间戳)
    std::thread imu_thread(imu_receiver_thread);

    // 等待串口就绪
    std::this_thread::sleep_for(500ms);

    // 检查串口数据
    bool serial_ok = false;
    {
        std::lock_guard lock(g_imu_mutex);
        serial_ok = !g_imu_buffer.empty();
    }
    if (!serial_ok) {
        fmt::print(fmt::fg(fmt::color::yellow),
            "警告: 未收到串口数据，请检查串口连接\n");
    } else {
        fmt::print(fmt::fg(fmt::color::green), "串口数据接收正常\n");
    }

    // 加载相机配置并打开相机
    camera::CameraConfig cam_config;
    cam_config.use_camera_sn = static_param::get_param<bool>(config, "Camera", "use_camera_sn");
    cam_config.camera_sn = static_param::get_param<std::string>(config, "Camera", "camera_sn");
    cam_config.use_mfs_config = static_param::get_param<bool>(config, "Camera", "use_config_from_file");
    std::string mfs_filename = static_param::get_param<std::string>(config, "Camera", "config_file_path");
    cam_config.mfs_config_path = std::string(CONFIG_DIR) + "/" + mfs_filename;
    cam_config.use_runtime_config = static_param::get_param<bool>(config, "Camera", "use_camera_config");

    // 加载相机运行时参数
    auto param_table = static_param::get_param_table(config, "Camera.config");
    for (const auto& [key, value] : param_table) {
        std::visit([&](const auto& v) {
            using T = std::decay_t<decltype(v)>;
            if constexpr (!std::is_same_v<T, std::vector<int64_t>>) {
                cam_config.runtime_params.emplace_back(key, camera::CameraParam(v));
            }
        }, value);
    }

    camera::HikCam cam(cam_config);
    cam.open();
    fmt::print(fmt::fg(fmt::color::green), "相机已打开\n");

    // 创建检测器
    auto detector = autoaim::detector::Detector::from_config(
        autoaim::detector::EnemyColor::RED, "armor_detector.toml");
    fmt::print("检测器已创建\n\n");

    // 主循环
    while (g_running) {
        try {
            // 捕获图像
            cv::Mat& img = cam.capture();
            if (img.empty()) continue;

            // 记录相机时间戳 (关键!)
            TimePoint cam_time = Clock::now();

            // 检测装甲板
            auto armors = detector->detect(img);

            // 采集状态下收集数据
            if (g_state == State::COLLECTING) {
                if (!armors.empty()) {
                    const auto& armor = armors[0];
                    auto point_opt = armor_to_camera_point(armor);

                    if (point_opt.has_value()) {
                        std::lock_guard lock(g_data_mutex);
                        g_cam_samples.emplace_back(cam_time, point_opt.value());
                    }
                }

                // 检查是否采集完成
                double elapsed = std::chrono::duration<double>(Clock::now() - g_collect_start_time).count();
                if (elapsed >= COLLECT_DURATION_SEC) {
                    g_state = State::CALIBRATING;

                    // 复制数据
                    std::vector<time_sync::CamSample> cam_copy;
                    std::vector<time_sync::ImuSample> imu_copy;
                    {
                        std::lock_guard lock(g_data_mutex);
                        cam_copy = g_cam_samples;
                        imu_copy = g_imu_samples;
                    }

                    fmt::print("\n\n采集完成!\n");
                    fmt::print("  相机数据: {} 点\n", cam_copy.size());
                    fmt::print("  IMU数据: {} 点\n", imu_copy.size());

                    if (cam_copy.size() < MIN_CAM_SAMPLES) {
                        fmt::print(fmt::fg(fmt::color::red),
                            "相机数据点不足 (需要 >= {}), 请确保装甲板一直可见!\n",
                            MIN_CAM_SAMPLES);
                        g_state = State::IDLE;
                    } else {
                        fmt::print("\n开始标定...\n\n");
                        auto result = time_sync::calibrate(cam_copy, imu_copy, extrinsic, 100.0, 1.0, true);
                        result.print();

                        // 记录标定结果到日志
                        debug::print("info", "TimeSync", "========== 时间戳标定结果 ==========");
                        debug::print("info", "TimeSync", "状态: {}", result.success ? "成功" : "失败");
                        debug::print("info", "TimeSync", "时间偏移: {:.1f} us ({:.3f} ms)", result.delta_t_us, result.delta_t_us / 1000.0);
                        debug::print("info", "TimeSync", "优化前标准差: {:.3f} mm", result.initial_std * 1000.0);
                        debug::print("info", "TimeSync", "优化后标准差: {:.3f} mm", result.final_std * 1000.0);
                        debug::print("info", "TimeSync", "config/hardware.toml [TimeSync]:");
                        debug::print("info", "TimeSync", "    delta_t_us = {:.0f}", result.delta_t_us);

                        if (result.success) {
                            fmt::print(fmt::fg(fmt::color::green),
                                "\n请将以下配置写入 config/hardware.toml:\n"
                                "[TimeSync]\n"
                                "    delta_t_us = {:.0f}\n\n",
                                result.delta_t_us
                            );

                            // 解释结果
                            if (result.delta_t_us > 0) {
                                fmt::print("含义: 图像比IMU数据晚到达 {:.1f}ms，匹配过去的IMU\n",
                                    result.delta_t_us / 1000.0);
                            } else {
                                fmt::print("含义: 图像比IMU数据早到达 {:.1f}ms，匹配未来的IMU\n",
                                    -result.delta_t_us / 1000.0);
                            }
                        }
                        g_state = State::IDLE;
                    }
                }
            }

            // 可视化
            cv::Mat display = img.clone();

            // 绘制装甲板
            for (const auto& armor : armors) {
                if (armor.landmarks.size() >= 4) {
                    cv::Scalar color = (g_state == State::COLLECTING)
                        ? cv::Scalar(0, 255, 0)   // 采集中: 绿色
                        : cv::Scalar(255, 255, 0); // 待机: 青色

                    for (size_t i = 0; i < 4; ++i) {
                        cv::line(display,
                            armor.landmarks[i],
                            armor.landmarks[(i+1)%4],
                            color, 2);
                    }

                    auto point_opt = armor_to_camera_point(armor);
                    if (point_opt.has_value()) {
                        auto& p = point_opt.value();
                        std::string text = fmt::format("({:.2f}, {:.2f}, {:.2f})", p.x(), p.y(), p.z());
                        cv::putText(display, text, armor.center - cv::Point2f(50, -20),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
                    }
                }
            }

            // 状态显示
            std::string status_text;
            cv::Scalar status_color;

            if (g_state == State::COLLECTING) {
                double elapsed = std::chrono::duration<double>(Clock::now() - g_collect_start_time).count();
                double remaining = COLLECT_DURATION_SEC - elapsed;
                size_t cam_count, imu_count;
                {
                    std::lock_guard lock(g_data_mutex);
                    cam_count = g_cam_samples.size();
                    imu_count = g_imu_samples.size();
                }
                status_text = fmt::format("COLLECTING: {:.1f}s  Cam:{} IMU:{}",
                    remaining, cam_count, imu_count);
                status_color = cv::Scalar(0, 255, 0);

                // 绘制进度条
                double progress = elapsed / COLLECT_DURATION_SEC;
                int bar_width = 300;
                int bar_x = (display.cols - bar_width) / 2;
                int bar_y = display.rows - 50;
                cv::rectangle(display, cv::Point(bar_x, bar_y),
                    cv::Point(bar_x + bar_width, bar_y + 20), cv::Scalar(100, 100, 100), -1);
                cv::rectangle(display, cv::Point(bar_x, bar_y),
                    cv::Point(bar_x + static_cast<int>(bar_width * progress), bar_y + 20),
                    cv::Scalar(0, 255, 0), -1);

            } else if (g_state == State::CALIBRATING) {
                status_text = "CALIBRATING...";
                status_color = cv::Scalar(0, 255, 255);
            } else {
                // 显示IMU缓冲区状态
                size_t imu_buf_size;
                {
                    std::lock_guard lock(g_imu_mutex);
                    imu_buf_size = g_imu_buffer.size();
                }
                status_text = fmt::format("IDLE  IMU buf: {}  Press 's' to start", imu_buf_size);
                status_color = cv::Scalar(255, 255, 255);
            }

            cv::putText(display, status_text, cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2);

            // 提示晃动云台
            if (g_state == State::COLLECTING) {
                cv::putText(display, "Slowly move the gimbal!", cv::Point(10, 60),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 165, 255), 2);
            }

            cv::imshow("Time Sync Calibration v2", display);

            // 处理键盘输入
            int key = cv::waitKey(1) & 0xFF;

            if (key == 'q' || key == 'Q') {
                g_running = false;

            } else if (key == 's' || key == 'S') {
                if (g_state == State::IDLE) {
                    // 检查串口数据
                    size_t imu_buf_size;
                    {
                        std::lock_guard lock(g_imu_mutex);
                        imu_buf_size = g_imu_buffer.size();
                    }

                    if (imu_buf_size == 0) {
                        fmt::print(fmt::fg(fmt::color::red),
                            "错误: 未收到IMU数据，请检查串口连接!\n");
                        continue;
                    }

                    // 开始采集
                    {
                        std::lock_guard lock(g_data_mutex);
                        g_cam_samples.clear();
                        g_imu_samples.clear();
                    }
                    g_collect_start_time = Clock::now();
                    g_state = State::COLLECTING;
                    fmt::print("\n开始采集! 请缓慢晃动云台...\n");
                }

            } else if (key == 'c' || key == 'C') {
                if (g_state == State::IDLE) {
                    // 手动标定
                    std::vector<time_sync::CamSample> cam_copy;
                    std::vector<time_sync::ImuSample> imu_copy;
                    {
                        std::lock_guard lock(g_data_mutex);
                        cam_copy = g_cam_samples;
                        imu_copy = g_imu_samples;
                    }

                    if (cam_copy.size() < MIN_CAM_SAMPLES) {
                        fmt::print(fmt::fg(fmt::color::red),
                            "\n数据点不足! 相机: {} (需要 >= {})\n",
                            cam_copy.size(), MIN_CAM_SAMPLES);
                    } else {
                        fmt::print("\n\n开始标定...\n\n");
                        auto result = time_sync::calibrate(cam_copy, imu_copy, extrinsic, 100.0, 1.0, true);
                        result.print();

                        // 记录标定结果到日志
                        debug::print("info", "TimeSync", "========== 时间戳标定结果 (手动模式) ==========");
                        debug::print("info", "TimeSync", "状态: {}", result.success ? "成功" : "失败");
                        debug::print("info", "TimeSync", "时间偏移: {:.1f} us ({:.3f} ms)", result.delta_t_us, result.delta_t_us / 1000.0);
                        debug::print("info", "TimeSync", "优化前标准差: {:.3f} mm", result.initial_std * 1000.0);
                        debug::print("info", "TimeSync", "优化后标准差: {:.3f} mm", result.final_std * 1000.0);
                        debug::print("info", "TimeSync", "config/hardware.toml [TimeSync]:");
                        debug::print("info", "TimeSync", "    delta_t_us = {:.0f}", result.delta_t_us);

                        if (result.success) {
                            fmt::print(fmt::fg(fmt::color::green),
                                "\n请将以下配置写入 config/hardware.toml:\n"
                                "[TimeSync]\n"
                                "    delta_t_us = {:.0f}\n\n",
                                result.delta_t_us
                            );
                        }
                    }
                }

            } else if (key == 'r' || key == 'R') {
                if (g_state != State::CALIBRATING) {
                    std::lock_guard lock(g_data_mutex);
                    g_cam_samples.clear();
                    g_imu_samples.clear();
                    g_state = State::IDLE;
                    fmt::print("\n数据已重置\n");
                }
            }

        } catch (const std::exception& e) {
            fmt::print(fmt::fg(fmt::color::red), "主循环异常: {}\n", e.what());
            std::this_thread::sleep_for(100ms);
        }
    }

    // 清理
    cv::destroyAllWindows();

    // 等待串口线程退出
    if (imu_thread.joinable()) {
        imu_thread.join();
    }

    fmt::print("\n程序退出\n");
    return 0;
}
