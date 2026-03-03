/**
 * @file test_gimbal_step.cpp
 * @brief 云台阶跃响应测试
 *
 * 用法:
 *   ./test_gimbal_step
 *
 * 按键:
 *   T      进入/退出阶跃测试 (进入时锁定当前角度为基准)
 *   A/D    yaw 左/右阶跃
 *   W/S    pitch 上/下阶跃
 *   Q/E    减小/增大阶跃幅度
 *   ESC    退出程序
 *
 * PlotJuggler 通道:
 *   /step/target_yaw    目标 yaw  (deg)
 *   /step/target_pitch  目标 pitch (deg)
 *   /step/actual_yaw    实际 yaw  (deg)
 *   /step/actual_pitch  实际 pitch (deg)
 *   /step/error_yaw     误差 yaw  (deg)
 *   /step/error_pitch   误差 pitch (deg)
 */

#include <chrono>
#include <cmath>
#include <thread>

#include <Eigen/Geometry>
#include <fmt/format.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "aimer/common/math/math.hpp"
#include "aimer/common/fire_control_types.hpp"
#include "aimer/common/transformer/transformer.hpp"
#include "hardware/serial/serial_thread.hpp"
#include "plugin/debug/logger.hpp"
#include "plugin/plotter/plotter.hpp"
#include "umt/umt.hpp"

using SteadyClock = std::chrono::steady_clock;

// 从 q_imu 计算云台 yaw/pitch (与 ypd_to_xyz 同系)
static std::pair<double, double> get_gimbal_angles(const Eigen::Quaterniond& q_imu) {
    const auto& R_g2i = aimer::tf::Transform<
        aimer::tf::Frame::Gimbal, aimer::tf::Frame::Imu>::R_;
    Eigen::Quaterniond q_gimbal(R_g2i.transpose() * q_imu.toRotationMatrix());
    return aimer::math::quat_to_yaw_pitch(q_gimbal);
}

int main() {
    debug::init_session("test_gimbal_step");
    fmt::print("=== 云台阶跃响应测试 ===\n");
    fmt::print("等待串口和参数初始化...\n");

    // TF 初始化
    aimer::tf::init();

    // PlotJuggler
    plotter::init();

    // 串口
    serial::start_serial_communication();
    umt::Subscriber<serial::SerialReceiveData> serial_sub("serial_receive");

    // fire_command (直接写，串口发送线程会读)
    auto fire_cmd = umt::BasicObjManager<::fire_control::FireCommand>::find_or_create("fire_command");

    fmt::print("\n按键说明:\n");
    fmt::print("  T     进入/退出阶跃测试\n");
    fmt::print("  A/D   yaw 左/右 阶跃\n");
    fmt::print("  W/S   pitch 上/下 阶跃\n");
    fmt::print("  Q/E   减小/增大 阶跃幅度\n");
    fmt::print("  ESC   退出\n\n");

    // 状态
    bool test_active = false;
    double target_yaw = 0, target_pitch = 0;
    double actual_yaw = 0, actual_pitch = 0;
    double step_deg = 10.0;

    // 最新串口数据
    serial::SerialReceiveData latest_serial{};

    // 接收线程: 持续读串口，更新 latest_serial
    std::atomic<bool> running{true};
    std::thread recv_thread([&]() {
        while (running) {
            try {
                latest_serial = serial_sub.pop_for(100);
            } catch (const umt::MessageError_Timeout&) {
            } catch (const umt::MessageError_Stopped&) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    });

    // 主循环 (显示 + 键盘 + 发送)
    const int W = 640, H = 360;
    cv::namedWindow("GimbalStep", cv::WINDOW_AUTOSIZE);

    auto loop_time = SteadyClock::now();
    const auto period = std::chrono::milliseconds(2);  // 500Hz 发送

    while (running) {
        // 从串口数据构建 q_imu
        // SerialReceiveData 的 yaw/pitch/roll 是欧拉角 (rad)
        // 需要转为四元数才能用 get_gimbal_angles
        {
            Eigen::AngleAxisd rz(latest_serial.yaw, Eigen::Vector3d::UnitZ());
            Eigen::AngleAxisd ry(latest_serial.pitch, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd rx(latest_serial.roll, Eigen::Vector3d::UnitX());
            Eigen::Quaterniond q_imu = rz * ry * rx;
            auto [yaw, pitch] = get_gimbal_angles(q_imu);
            actual_yaw = yaw;
            actual_pitch = pitch;
        }

        // 发送指令
        if (test_active) {
            ::fire_control::FireCommand cmd;
            cmd.control_enabled = true;
            cmd.yaw = static_cast<float>(target_yaw);
            cmd.pitch = static_cast<float>(target_pitch);
            cmd.allow_fire = false;
            cmd.fire_now = false;
            fire_cmd->get() = cmd;
        } else {
            // 不控制时发 disable
            ::fire_control::FireCommand cmd;
            cmd.control_enabled = false;
            fire_cmd->get() = cmd;
        }

        // PlotJuggler 输出
        {
            double tgt_yaw_deg = target_yaw * 180.0 / M_PI;
            double tgt_pitch_deg = target_pitch * 180.0 / M_PI;
            double act_yaw_deg = actual_yaw * 180.0 / M_PI;
            double act_pitch_deg = actual_pitch * 180.0 / M_PI;

            plotter::begin();
            plotter::add("/step/target_yaw", tgt_yaw_deg);
            plotter::add("/step/target_pitch", tgt_pitch_deg);
            plotter::add("/step/actual_yaw", act_yaw_deg);
            plotter::add("/step/actual_pitch", act_pitch_deg);
            plotter::add("/step/error_yaw", tgt_yaw_deg - act_yaw_deg);
            plotter::add("/step/error_pitch", tgt_pitch_deg - act_pitch_deg);
            plotter::end();
        }

        // 显示 (~30Hz 刷新)
        static int frame_count = 0;
        if (++frame_count % 16 == 0) {
            cv::Mat canvas(H, W, CV_8UC3, cv::Scalar(30, 30, 30));

            int y = 30;
            auto put = [&](const std::string& text, cv::Scalar color = {200, 200, 200}) {
                cv::putText(canvas, text, {20, y},
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv::LINE_AA);
                y += 28;
            };

            put("=== Gimbal Step Response Test ===", {255, 255, 255});
            y += 10;

            if (test_active) {
                put("MODE: ACTIVE  (press T to stop)", {0, 255, 0});
            } else {
                put("MODE: IDLE    (press T to start)", {0, 200, 255});
            }
            put(fmt::format("Step size: {:.1f} deg  (Q/E to adjust)", step_deg), {200, 200, 200});
            y += 10;

            put(fmt::format("Target  yaw: {:+7.2f} deg   pitch: {:+7.2f} deg",
                target_yaw * 57.3, target_pitch * 57.3), {0, 255, 255});
            put(fmt::format("Actual  yaw: {:+7.2f} deg   pitch: {:+7.2f} deg",
                actual_yaw * 57.3, actual_pitch * 57.3), {0, 255, 0});
            put(fmt::format("Error   yaw: {:+7.2f} deg   pitch: {:+7.2f} deg",
                (target_yaw - actual_yaw) * 57.3, (target_pitch - actual_pitch) * 57.3),
                {100, 100, 255});
            y += 10;

            put(fmt::format("Serial  yaw: {:+7.2f}  pitch: {:+7.2f}  roll: {:+7.2f}",
                latest_serial.yaw * 57.3, latest_serial.pitch * 57.3, latest_serial.roll * 57.3),
                {150, 150, 150});
            put(fmt::format("Bullet speed: {:.1f} m/s", latest_serial.bullet_speed),
                {150, 150, 150});
            y += 20;

            // 可视化: 误差仪表
            {
                int cx = W / 2, cy = H - 60;
                int gauge_r = 40;

                // 背景圆
                cv::circle(canvas, {cx, cy}, gauge_r, {60, 60, 60}, -1);
                cv::circle(canvas, {cx, cy}, gauge_r, {100, 100, 100}, 1);

                // 十字线
                cv::line(canvas, {cx - gauge_r, cy}, {cx + gauge_r, cy}, {80, 80, 80}, 1);
                cv::line(canvas, {cx, cy - gauge_r}, {cx, cy + gauge_r}, {80, 80, 80}, 1);

                if (test_active) {
                    // 误差点 (1deg = 4px)
                    double err_yaw = (target_yaw - actual_yaw) * 57.3;
                    double err_pitch = (target_pitch - actual_pitch) * 57.3;
                    double px_per_deg = 4.0;
                    int ex = cx + static_cast<int>(-err_yaw * px_per_deg);   // 左为正yaw
                    int ey = cy + static_cast<int>(err_pitch * px_per_deg);  // 上为正pitch，屏幕y反
                    cv::circle(canvas, {ex, ey}, 4, {0, 0, 255}, -1, cv::LINE_AA);
                }

                cv::putText(canvas, "Error (1div=2.5deg)", {cx - 70, cy + gauge_r + 18},
                    cv::FONT_HERSHEY_SIMPLEX, 0.35, {120, 120, 120}, 1);
            }

            cv::imshow("GimbalStep", canvas);
        }

        // 键盘处理
        int key = cv::waitKey(1);
        if (key == 27) {  // ESC
            break;
        }
        if (key == 't' || key == 'T') {
            test_active = !test_active;
            if (test_active) {
                // 锁定当前角度为基准
                target_yaw = actual_yaw;
                target_pitch = actual_pitch;
                fmt::print("[STEP] 测试开始，基准: yaw={:.2f} pitch={:.2f} deg\n",
                    target_yaw * 57.3, target_pitch * 57.3);
            } else {
                fmt::print("[STEP] 测试结束\n");
            }
        }

        if (test_active) {
            double step = step_deg * M_PI / 180.0;
            if (key == 'a' || key == 'A') {
                target_yaw += step;
                fmt::print("[STEP] yaw +{:.1f}deg → {:.2f}deg\n", step_deg, target_yaw * 57.3);
            }
            if (key == 'd' || key == 'D') {
                target_yaw -= step;
                fmt::print("[STEP] yaw -{:.1f}deg → {:.2f}deg\n", step_deg, target_yaw * 57.3);
            }
            if (key == 'w' || key == 'W') {
                target_pitch += step;
                fmt::print("[STEP] pitch +{:.1f}deg → {:.2f}deg\n", step_deg, target_pitch * 57.3);
            }
            if (key == 's' || key == 'S') {
                target_pitch -= step;
                fmt::print("[STEP] pitch -{:.1f}deg → {:.2f}deg\n", step_deg, target_pitch * 57.3);
            }
        }

        // 阶跃幅度调节 (任何时候都可以调)
        if (key == 'q' || key == 'Q') {
            step_deg = std::max(1.0, step_deg - 1.0);
            fmt::print("[STEP] 阶跃幅度: {:.1f} deg\n", step_deg);
        }
        if (key == 'e' || key == 'E') {
            step_deg = std::min(90.0, step_deg + 1.0);
            fmt::print("[STEP] 阶跃幅度: {:.1f} deg\n", step_deg);
        }

        // 500Hz 节奏
        loop_time += period;
        std::this_thread::sleep_until(loop_time);
    }

    // 停止前发 disable
    {
        ::fire_control::FireCommand cmd;
        cmd.control_enabled = false;
        fire_cmd->get() = cmd;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    running = false;
    recv_thread.join();

    fmt::print("测试结束\n");
    return 0;
}
