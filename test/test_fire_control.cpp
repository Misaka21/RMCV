//
// 火控可视化测试程序 (PID 模式)
//
// 功能:
//   可视化火控 PID 模式的瞄准和开火判断
//   - 显示目标位置和预测位置
//   - 显示瞄准角度和开火状态
//   - 反陀螺模式可视化
//
// 使用方法:
//   ./test_fire_control           # 使用实车硬件
//   ./test_fire_control --sim     # 使用模拟器 (需先启动 at_vision_simulator)
//
// 按键:
//   'q' / ESC  - 退出
//   'r'        - 重置火控状态
//

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <thread>

#include <fmt/format.h>
#include <fmt/color.h>
#include <opencv2/opencv.hpp>

#include "aimer/auto_aim/detector/detector_node.hpp"
#include "aimer/auto_aim/predictor/predictor_node.hpp"
#include "aimer/auto_aim/fire_control/fire_controller.hpp"
#include "aimer/auto_aim/fire_control/types.hpp"
#include "aimer/auto_aim/predictor/types.hpp"
#include "aimer/common/transformer/transformer.hpp"
#include "hardware/hardware_node.hpp"
#include "plugin/debug/logger.hpp"
#include "plugin/param/runtime_parameter.hpp"
#include "umt/umt.hpp"

// 模拟器模式时需要
#ifdef SIMULATOR_FOUND
#include "simulator/simulator_node.hpp"
#endif

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

static std::atomic<bool> g_running{true};

void signal_handler(int sig) {
    fmt::print(fmt::fg(fmt::color::yellow), "\n[INFO] Signal {}, exiting...\n", sig);
    g_running = false;

    auto hw_running = umt::BasicObjManager<bool>::find_or_create("hardware_running", false);
    auto det_running = umt::BasicObjManager<bool>::find_or_create("detector_running", false);
    auto pred_running = umt::BasicObjManager<bool>::find_or_create("predictor_running", false);
    hw_running->get() = false;
    det_running->get() = false;
    pred_running->get() = false;
    std::exit(0);
}

// 获取当前时间 (秒)
double get_current_time() {
    static auto start = Clock::now();
    return std::chrono::duration<double>(Clock::now() - start).count();
}

// 从cv::Mat相机矩阵获取内参
struct CameraParams {
    double fx, fy, cx, cy;

    static CameraParams from_matrix(const cv::Mat& K) {
        CameraParams p;
        p.fx = K.at<double>(0, 0);
        p.fy = K.at<double>(1, 1);
        p.cx = K.at<double>(0, 2);
        p.cy = K.at<double>(1, 2);
        return p;
    }
};

// 3D 点投影到图像
cv::Point project_to_image(const Eigen::Vector3d& pos_cam, const CameraParams& K) {
    if (pos_cam.z() < 0.1) return cv::Point(-1, -1);
    double u = K.fx * pos_cam.x() / pos_cam.z() + K.cx;
    double v = K.fy * pos_cam.y() / pos_cam.z() + K.cy;
    return cv::Point(static_cast<int>(u), static_cast<int>(v));
}

// 绘制装甲板
void draw_armor(cv::Mat& img, const Eigen::Vector3d& pos, const cv::Scalar& color,
                const CameraParams& K, const std::string& label = "") {
    cv::Point pt = project_to_image(pos, K);
    if (pt.x < 0 || pt.x >= img.cols || pt.y < 0 || pt.y >= img.rows) return;

    cv::circle(img, pt, 8, color, 2);
    cv::circle(img, pt, 3, color, -1);

    if (!label.empty()) {
        cv::putText(img, label, pt + cv::Point(10, -10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
    }
}

// 绘制准星
void draw_crosshair(cv::Mat& img, const cv::Scalar& color) {
    int cx = img.cols / 2;
    int cy = img.rows / 2;
    cv::line(img, cv::Point(cx - 30, cy), cv::Point(cx + 30, cy), color, 1);
    cv::line(img, cv::Point(cx, cy - 30), cv::Point(cx, cy + 30), color, 1);
    cv::circle(img, cv::Point(cx, cy), 5, color, 1);
}

int main(int argc, char* argv[]) {
    // 检查是否使用模拟器模式
    bool use_simulator = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--sim") == 0 || std::strcmp(argv[i], "-s") == 0) {
            use_simulator = true;
        }
    }

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    debug::init_session("test_fire_control");

    fmt::print(fmt::fg(fmt::color::gold),
        "====================================================================\n"
        "                 火控可视化测试 (PID 模式)\n"
        "====================================================================\n"
        "模式: {}\n"
        "====================================================================\n"
        "操作:\n"
        "  'q' / ESC  - 退出\n"
        "  'r'        - 重置火控状态\n"
        "====================================================================\n\n",
        use_simulator ? "模拟器" : "实车硬件"
    );

    // 启动参数热重载
    std::thread param_thread([]() {
        runtime_param::parameter_run("aimer.toml");
    });
    param_thread.detach();
    runtime_param::wait_for_param("ok");

    // 初始化坐标变换
    tf::init("camera.yaml");
    auto K = CameraParams::from_matrix(tf::get_camera_matrix());

    auto hardware_ready = umt::BasicObjManager<bool>::find_or_create("hardware_running", false);

    // 启动数据源
    std::thread data_source_thread;
    if (use_simulator) {
#ifdef SIMULATOR_FOUND
        fmt::print(fmt::fg(fmt::color::cyan), "[INFO] Starting simulator node...\n");
        fmt::print(fmt::fg(fmt::color::cyan), "[INFO] Make sure at_vision_simulator is running!\n");
        data_source_thread = std::thread([]() {
            simulator::start_simulator_node();
        });
#else
        fmt::print(fmt::fg(fmt::color::red), "[ERROR] Simulator not available (ROS2 not found)\n");
        return 1;
#endif
    } else {
        fmt::print(fmt::fg(fmt::color::cyan), "[INFO] Starting hardware node...\n");
        data_source_thread = std::thread([]() {
            hardware::start_hardware_node();
        });
    }

    // 等待数据源就绪
    fmt::print("[INFO] Waiting for data source...\n");
    while (!hardware_ready->get() && g_running) {
        std::this_thread::sleep_for(100ms);
    }

    if (!g_running) {
        if (data_source_thread.joinable()) data_source_thread.join();
        return 0;
    }

    fmt::print(fmt::fg(fmt::color::green), "[INFO] Data source ready, starting detector...\n");

    // 启动检测器
    std::thread detector_thread([]() {
        autoaim::start_detector_node();
    });

    // 启动预测器
    fmt::print(fmt::fg(fmt::color::green), "[INFO] Starting predictor...\n");
    std::thread predictor_thread([]() {
        autoaim::predictor::start_predictor_node();
    });

    std::this_thread::sleep_for(500ms);

    // 创建火控器 (不启动线程，手动调用)
    autoaim::fire_control::FireController controller;

    // 获取共享数据
    // 订阅 battlefield 消息 (predictor 用 Publisher 发布)
    umt::Subscriber<autoaim::predictor::BattlefieldSnapshot> battlefield_sub("battlefield");
    // 订阅 sync_frame 消息 (hardware 用 Publisher 发布)
    umt::Subscriber<hardware::SyncFrame> sync_frame_sub("sync_frame");

    fmt::print(fmt::fg(fmt::color::green), "[INFO] Fire control test running. Press 'q' to stop.\n\n");

    // 延迟估计器 (简化版)
    autoaim::fire_control::LatencyInfo latency;
    latency.img_to_predict = 0.015;
    latency.predict_to_send = 0.005;
    latency.send_to_control = 0.003;
    latency.control_to_fire = 0.010;
    latency.fire_to_hit = 0.1;

    int frame_count = 0;
    auto last_fps_time = Clock::now();
    double fps = 0;
    int empty_count = 0;

    // 主循环
    autoaim::predictor::BattlefieldSnapshot snapshot;  // 保存最新快照
    int battlefield_recv_count = 0;

    while (g_running) {
        // 获取图像 (从消息队列)
        cv::Mat img;
        try {
            auto frame = sync_frame_sub.pop_for(100);  // 100ms 超时
            if (!frame.image.empty()) {
                img = frame.image.clone();
            }
        } catch (const umt::MessageError_Timeout&) {
            // 超时，继续
        } catch (const umt::MessageError_Stopped&) {
            break;
        }

        // 尝试获取最新的 battlefield (非阻塞)
        try {
            snapshot = battlefield_sub.pop_for(1);  // 1ms 超时，快速检查
            battlefield_recv_count++;
            // DEBUG: 打印收到的 battlefield 信息
            if (battlefield_recv_count <= 5 || battlefield_recv_count % 100 == 0) {
                int valid_count = 0;
                for (int i = 1; i < 9; ++i) {
                    if (snapshot.is_valid(i)) valid_count++;
                }
                fmt::print(fmt::fg(fmt::color::cyan),
                    "[DEBUG] Battlefield #{}: valid_mask=0x{:04x}, valid_count={}, primary={}\n",
                    battlefield_recv_count, snapshot.valid_mask, valid_count, snapshot.primary_target_id);
            }
        } catch (const umt::MessageError_Timeout&) {
            // 没有新数据，用旧的
        } catch (const umt::MessageError_Stopped&) {
            break;
        }

        if (img.empty()) {
            empty_count++;
            if (empty_count % 100 == 0) {
                fmt::print("[DEBUG] No image received ({} times)\n", empty_count);
            }
            continue;
        }
        empty_count = 0;  // 重置

        double current_time = get_current_time();

        // 更新飞行时间
        if (snapshot.get_primary()) {
            const auto* armor = snapshot.get_primary()->get_recommended_armor();
            if (armor) {
                double distance = armor->position.norm();
                double bullet_speed = std::max(static_cast<double>(snapshot.self_state.bullet_speed), 15.0);
                latency.fire_to_hit = distance / bullet_speed;
            }
        }

        // 执行火控
        auto cmd = controller.control(snapshot, current_time, latency);

        // ========== 可视化 ==========

        // 绘制所有目标
        snapshot.for_each_valid([&](int id, const autoaim::predictor::VehicleState& vehicle) {
            cv::Scalar color = (id == cmd.target_id) ? cv::Scalar(0, 255, 0) : cv::Scalar(128, 128, 128);

            // 绘制旋转中心
            draw_armor(img, vehicle.center, cv::Scalar(255, 255, 0), K,
                       fmt::format("ID:{}", id));

            // 绘制各装甲板
            for (int i = 0; i < vehicle.armor_count; ++i) {
                const auto& armor = vehicle.armors[i];
                cv::Scalar armor_color = armor.visible ? color : cv::Scalar(64, 64, 64);

                if (i == vehicle.recommended_armor_idx) {
                    armor_color = cv::Scalar(0, 255, 255);  // 推荐装甲板用黄色
                }

                draw_armor(img, armor.position, armor_color, K);
            }

            // 如果是陀螺，标记
            if (vehicle.spin.active) {
                cv::Point pt = project_to_image(vehicle.center, K);
                if (pt.x >= 0) {
                    std::string spin_label = (vehicle.spin.level == autoaim::predictor::SpinLevel::HIGH)
                        ? "SPIN-H" : "SPIN-L";
                    cv::putText(img, spin_label, pt + cv::Point(-20, 20),
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
                }
            }
        });

        // 绘制火控选择的目标
        const auto& selection = controller.last_selection();
        if (selection.has_target && selection.armor) {
            // 预测位置
            draw_armor(img, selection.predicted_pos, cv::Scalar(0, 255, 0), K, "PRED");
        }

        // 绘制准星
        cv::Scalar crosshair_color = cmd.fire_now ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);
        draw_crosshair(img, crosshair_color);

        // ========== 状态信息 ==========
        int y = 30;
        int dy = 25;

        // 模式
        std::string mode_str = (controller.current_mode() == autoaim::fire_control::ControlMode::MPC)
            ? "MPC" : "PID";
        cv::putText(img, fmt::format("Mode: {}", mode_str),
                    cv::Point(10, y), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
        y += dy;

        // 目标信息
        if (cmd.control_enabled) {
            cv::putText(img, fmt::format("Target: {}", cmd.target_id),
                        cv::Point(10, y), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
            y += dy;

            cv::putText(img, fmt::format("Confidence: {:.2f}", cmd.confidence),
                        cv::Point(10, y), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
            y += dy;

            cv::putText(img, fmt::format("Error: {:.3f} m", cmd.tracking_error),
                        cv::Point(10, y), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
            y += dy;

            // 瞄准角度
            const auto& aim = controller.last_aim();
            cv::putText(img, fmt::format("Aim: Y={:.2f} P={:.2f} deg",
                        aim.yaw * 180 / M_PI, aim.pitch * 180 / M_PI),
                        cv::Point(10, y), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);
            y += dy;

            // 云台状态
            const auto& gimbal = controller.gimbal_state();
            cv::putText(img, fmt::format("Gimbal: Y={:.2f} P={:.2f} deg",
                        gimbal.yaw * 180 / M_PI, gimbal.pitch * 180 / M_PI),
                        cv::Point(10, y), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(200, 200, 200), 2);
            y += dy;
        } else {
            cv::putText(img, "No Target",
                        cv::Point(10, y), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
            y += dy;
        }

        // 开火状态
        y += 10;
        if (cmd.fire_now) {
            cv::putText(img, ">>> FIRE <<<",
                        cv::Point(10, y), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 3);
        } else if (cmd.allow_fire) {
            cv::putText(img, "Ready",
                        cv::Point(10, y), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
        }

        // FPS
        frame_count++;
        auto now = Clock::now();
        double elapsed = std::chrono::duration<double>(now - last_fps_time).count();
        if (elapsed >= 1.0) {
            fps = frame_count / elapsed;
            frame_count = 0;
            last_fps_time = now;
        }
        cv::putText(img, fmt::format("FPS: {:.1f}", fps),
                    cv::Point(img.cols - 120, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);

        // 调试：确认到达这里
        static int show_count = 0;
        if (++show_count <= 3) {
            fmt::print("[DEBUG] Showing frame {} ({}x{})\n", show_count, img.cols, img.rows);
        }

        cv::imshow("Fire Control Test", img);

        // 键盘处理
        int key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 'Q' || key == 27) {
            g_running = false;
        } else if (key == 'r' || key == 'R') {
            controller.reset();
            fmt::print("[INFO] Fire controller reset\n");
        }
    }

    cv::destroyAllWindows();

    // 通知线程退出
    auto hw_running = umt::BasicObjManager<bool>::find_or_create("hardware_running", false);
    auto det_running = umt::BasicObjManager<bool>::find_or_create("detector_running", false);
    auto pred_running = umt::BasicObjManager<bool>::find_or_create("predictor_running", false);
    hw_running->get() = false;
    det_running->get() = false;
    pred_running->get() = false;

    if (data_source_thread.joinable()) data_source_thread.join();
    if (detector_thread.joinable()) detector_thread.join();
    if (predictor_thread.joinable()) predictor_thread.join();

    fmt::print(fmt::fg(fmt::color::green), "[INFO] Done.\n");
    return 0;
}
