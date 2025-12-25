//
// 弹道可视化测试程序
//
// 功能:
//   按空格发射子弹，可视化弹道轨迹
//   用于验证枪管安装精度
//
// 使用方法:
//   SPACE - 发射子弹 (20Hz)
//   'r'   - 清空所有子弹
//   '+'   - 增加弹速
//   '-'   - 减少弹速
//   'q'   - 退出
//

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

#include <fmt/format.h>
#include <fmt/color.h>
#include <opencv2/opencv.hpp>

#include "hardware/hik_cam/hik_camera.hpp"
#include "hardware/serial/serial_thread.hpp"
#include "aimer/common/transformer/transformer.hpp"
#include "aimer/common/ballistic/projectile_simulator.hpp"
#include "plugin/param/static_config.hpp"
#include "plugin/param/runtime_parameter.hpp"
#include "umt/umt.hpp"

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

// ============================================================================
// 全局状态
// ============================================================================

std::atomic<bool> g_running{true};

// IMU 数据
std::mutex g_imu_mutex;
Eigen::Quaterniond g_q_imu = Eigen::Quaterniond::Identity();
float g_bullet_speed = 15.0f;
bool g_imu_valid = false;

// ============================================================================
// IMU 接收线程
// ============================================================================

void imu_receiver_thread(const std::string& port_name, int baudrate) {
    fmt::print("串口线程启动: {} @ {}\n", port_name, baudrate);

    try {
        serial::start_serial_communication(port_name, baudrate);
        std::this_thread::sleep_for(100ms);

        auto recv_queue = umt::BasicObjManager<serial::ReceiveQueue>::find_or_create("receive_queue");

        while (g_running) {
            serial::ReceiveQueue& queue = recv_queue->get();

            while (!queue.empty()) {
                serial::SerialReceiveData data = queue.front();
                queue.pop();

                // 欧拉角转四元数
                double yaw_rad = data.yaw * M_PI / 180.0;
                double pitch_rad = data.pitch * M_PI / 180.0;
                double roll_rad = data.roll * M_PI / 180.0;

                Eigen::Quaterniond q =
                    Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(pitch_rad, Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(roll_rad, Eigen::Vector3d::UnitX());

                {
                    std::lock_guard lock(g_imu_mutex);
                    g_q_imu = q;
                    g_bullet_speed = data.bullet_speed;
                    g_imu_valid = true;
                }
            }

            std::this_thread::sleep_for(1ms);
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
        "                 弹道可视化测试程序\n"
        "==================================================================\n"
        "用途: 验证枪管安装精度\n"
        "==================================================================\n"
        "操作:\n"
        "  按住 SPACE - 连续发射子弹 (20Hz)\n"
        "  'r'        - 清空所有子弹\n"
        "  '+' / '='  - 增加弹速\n"
        "  '-'        - 减少弹速\n"
        "  'q'        - 退出\n"
        "==================================================================\n\n"
    );

    // 加载配置
    auto config = static_param::parse_file("hardware.toml");
    std::string port_name = static_param::get_param<std::string>(config, "Serial", "port_name");
    int64_t baudrate = static_param::get_param<int64_t>(config, "Serial", "baudrate");

    // 初始化参数系统
    runtime_param::parameter_run("aimer.toml");

    // 初始化 TF 模块
    if (!tf::init()) {
        fmt::print(fmt::fg(fmt::color::red), "TF 模块初始化失败!\n");
        return 1;
    }

    // 启动串口线程
    std::thread imu_thread(imu_receiver_thread, port_name, static_cast<int>(baudrate));
    std::this_thread::sleep_for(500ms);

    // 检查串口
    {
        std::lock_guard lock(g_imu_mutex);
        if (g_imu_valid) {
            fmt::print(fmt::fg(fmt::color::green), "IMU 数据接收正常\n");
        } else {
            fmt::print(fmt::fg(fmt::color::yellow), "警告: 未收到 IMU 数据\n");
        }
    }

    // 加载相机配置
    camera::CameraConfig cam_config;
    cam_config.use_camera_sn = static_param::get_param<bool>(config, "Camera", "use_camera_sn");
    cam_config.camera_sn = static_param::get_param<std::string>(config, "Camera", "camera_sn");
    cam_config.use_mfs_config = static_param::get_param<bool>(config, "Camera", "use_config_from_file");
    std::string mfs_filename = static_param::get_param<std::string>(config, "Camera", "config_file_path");
    cam_config.mfs_config_path = std::string(CONFIG_DIR) + "/" + mfs_filename;
    cam_config.use_runtime_config = static_param::get_param<bool>(config, "Camera", "use_camera_config");

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
    fmt::print(fmt::fg(fmt::color::green), "相机已打开\n\n");

    // 创建弹道模拟器
    ballistic::ProjectileSimulator simulator;
    ballistic::BallisticConfig ballistic_cfg;
    ballistic_cfg.fire_rate_hz = 20.0;
    ballistic_cfg.resistance_k = 0.01;
    ballistic_cfg.max_bullets = 100;
    ballistic_cfg.max_flight_time = 3.0;
    simulator.set_config(ballistic_cfg);

    double manual_bullet_speed = 15.0;
    bool firing = false;
    auto start_time = Clock::now();

    // 主循环
    while (g_running) {
        try {
            cv::Mat& img = cam.capture();
            if (img.empty()) continue;

            cv::Mat display = img.clone();

            // 获取当前时间
            double current_time = std::chrono::duration<double>(
                Clock::now() - start_time).count();

            // 获取 IMU 数据
            Eigen::Quaterniond q_imu;
            float serial_bullet_speed;
            bool imu_ok = false;
            {
                std::lock_guard lock(g_imu_mutex);
                if (g_imu_valid) {
                    q_imu = g_q_imu;
                    serial_bullet_speed = g_bullet_speed;
                    imu_ok = true;
                }
            }

            // 使用串口弹速或手动设置的弹速
            double bullet_speed = (serial_bullet_speed > 1.0)
                ? serial_bullet_speed : manual_bullet_speed;
            simulator.set_bullet_speed(bullet_speed);

            // 如果正在发射
            if (firing && imu_ok) {
                simulator.fire(current_time, q_imu);
            }

            // 清理过期子弹
            simulator.cleanup(current_time);

            // 绘制子弹
            if (imu_ok) {
                simulator.draw(display, current_time, q_imu);
            }

            // 显示状态
            cv::putText(display, fmt::format("Speed: {:.1f} m/s", bullet_speed),
                        cv::Point(display.cols - 200, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);

            cv::putText(display, firing ? "FIRING" : "READY",
                        cv::Point(display.cols - 200, 60),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        firing ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0), 2);

            // 显示 IMU 状态
            if (imu_ok) {
                auto euler = q_imu.toRotationMatrix().eulerAngles(2, 1, 0);
                cv::putText(display,
                    fmt::format("IMU: Y={:.1f} P={:.1f}",
                        euler[0] * 180 / M_PI, euler[1] * 180 / M_PI),
                    cv::Point(display.cols - 200, 90),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);
            } else {
                cv::putText(display, "IMU: No Data",
                    cv::Point(display.cols - 200, 90),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
            }

            // 绘制准星 (屏幕中心)
            int cx = display.cols / 2;
            int cy = display.rows / 2;
            cv::line(display, cv::Point(cx - 20, cy), cv::Point(cx + 20, cy),
                     cv::Scalar(0, 255, 0), 1);
            cv::line(display, cv::Point(cx, cy - 20), cv::Point(cx, cy + 20),
                     cv::Scalar(0, 255, 0), 1);

            cv::imshow("Ballistic Visualization", display);

            // 处理键盘输入
            int key = cv::waitKey(1) & 0xFF;

            if (key == 'q' || key == 'Q' || key == 27) {
                g_running = false;
            } else if (key == ' ') {
                firing = true;
            } else if (key == 'r' || key == 'R') {
                simulator.clear();
                fmt::print("子弹已清空\n");
            } else if (key == '+' || key == '=') {
                manual_bullet_speed += 1.0;
                fmt::print("弹速: {:.1f} m/s\n", manual_bullet_speed);
            } else if (key == '-') {
                manual_bullet_speed = std::max(5.0, manual_bullet_speed - 1.0);
                fmt::print("弹速: {:.1f} m/s\n", manual_bullet_speed);
            } else {
                firing = false;  // 松开空格停止发射
            }

        } catch (const std::exception& e) {
            fmt::print(fmt::fg(fmt::color::red), "异常: {}\n", e.what());
            std::this_thread::sleep_for(100ms);
        }
    }

    cv::destroyAllWindows();

    if (imu_thread.joinable()) {
        imu_thread.join();
    }

    fmt::print("\n程序退出\n");
    return 0;
}
