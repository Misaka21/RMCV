//
// 地平面可视化测试
// 显示世界坐标系地面网格 + XYZ 坐标轴，验证 IMU 姿态
//

#include <chrono>
#include <thread>
#include <opencv2/opencv.hpp>
#include <fmt/format.h>
#include <fmt/color.h>

#include "hardware/hardware_node.hpp"
#include "aimer/common/robot_state.hpp"
#include "aimer/common/transformer/transformer.hpp"
#include "plugin/param/runtime_parameter.hpp"
#include "umt/umt.hpp"

using namespace std::chrono_literals;

// 绘制地平面网格 + XYZ 坐标轴
void draw_world_ground_grid(
    cv::Mat& img,
    const Eigen::Quaterniond& q_imu,
    double grid_size = 1.0,
    double range = 10.0,
    double ground_z = -0.3
) {
    // 世界坐标系: x前, y左, z上
    // 地面是 z = ground_z 的平面
    for (double x = 0; x <= range; x += grid_size) {
        for (double y = -range; y <= range; y += grid_size) {
            Eigen::Vector3d p_world(x, y, ground_z);
            bool valid = false;
            cv::Point2f pixel = aimer::tf::world_to_pixel(p_world, q_imu, valid);

            if (valid && pixel.x >= 0 && pixel.x < img.cols &&
                pixel.y >= 0 && pixel.y < img.rows) {
                // 前方用绿色，左右用不同亮度
                int brightness = static_cast<int>(255 - std::abs(y) / range * 150);
                cv::Scalar color(0, brightness, 0);

                // X轴上的点用红色标记
                if (std::abs(y) < 0.01) {
                    color = cv::Scalar(0, 0, 255);
                }
                // Y轴上的点用蓝色标记
                if (std::abs(x) < 0.01) {
                    color = cv::Scalar(255, 0, 0);
                }

                cv::circle(img, pixel, 4, color, -1, cv::LINE_AA);

                // 在整米处标注距离
                if (std::abs(y) < 0.01 && static_cast<int>(x) == x) {
                    cv::putText(img, fmt::format("{}m", static_cast<int>(x)),
                                pixel + cv::Point2f(5, -5),
                                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
                }
            }
        }
    }

    // 在地平面末端绘制 Z 轴 (向上的箭头)
    // 起点: (range, 0, ground_z)
    // 终点: (range, 0, ground_z + 2.0)  // 2米高的箭头
    Eigen::Vector3d z_axis_start(range, 0, ground_z);
    Eigen::Vector3d z_axis_end(range, 0, ground_z + 2.0);

    bool valid_start = false, valid_end = false;
    cv::Point2f pixel_start = aimer::tf::world_to_pixel(z_axis_start, q_imu, valid_start);
    cv::Point2f pixel_end = aimer::tf::world_to_pixel(z_axis_end, q_imu, valid_end);

    if (valid_start && valid_end) {
        // 绘制黄色箭头表示 Z 轴 (向上)
        cv::arrowedLine(img, pixel_start, pixel_end, cv::Scalar(0, 255, 255), 3, cv::LINE_AA, 0, 0.2);
        // 标注 "Z"
        cv::putText(img, "Z", pixel_end + cv::Point2f(10, -10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    }
}

int main() {
    fmt::print(fmt::fg(fmt::color::gold),
        "====================================================================\n"
        "                  地平面可视化测试\n"
        "====================================================================\n"
        "说明:\n"
        "  - 绿色网格: 地面 (z = -0.3m)\n"
        "  - 红色轴: X 轴 (向前)\n"
        "  - 蓝色轴: Y 轴 (向左)\n"
        "  - 黄色箭头: Z 轴 (向上)\n"
        "\n"
        "预期行为:\n"
        "  抬头 → 地面向下移动\n"
        "  低头 → 地面向上移动\n"
        "  左转 → 地面逆时针旋转\n"
        "  右转 → 地面顺时针旋转\n"
        "\n"
        "按 ESC 退出\n"
        "====================================================================\n\n"
    );

    // 启动运行时参数加载线程
    std::thread param_thread([]() {
        runtime_param::parameter_run("aimer.toml");
    });
    param_thread.detach();

    // 等待参数加载完成
    runtime_param::wait_for_param("ok");
    fmt::print(fmt::fg(fmt::color::green), "[运行时参数已加载]\n");

    // 初始化坐标变换系统
    if (!aimer::tf::init("camera.yaml")) {
        fmt::print(fmt::fg(fmt::color::red), "[错误] 无法加载 camera.yaml\n");
        return 1;
    }
    fmt::print(fmt::fg(fmt::color::green), "[坐标变换系统已初始化]\n");

    // 启动硬件节点线程
    std::thread hw_thread([]() {
        hardware::start_hardware_node();
    });
    hw_thread.detach();

    // 等待硬件节点启动
    auto hardware_ready = umt::BasicObjManager<bool>::find_or_create("hardware_running", false);
    while (!hardware_ready->get()) {
        fmt::print(fmt::fg(fmt::color::yellow), "\r[等待硬件节点启动...]");
        std::fflush(stdout);
        std::this_thread::sleep_for(500ms);
    }
    fmt::print(fmt::fg(fmt::color::green), "\n[硬件节点已启动]\n");

    // 订阅同步帧
    umt::Subscriber<hardware::SyncFrame> sub("sync_frame");

    // 创建窗口
    const std::string window_name = "Ground Plane Visualization";
    cv::namedWindow(window_name, cv::WINDOW_NORMAL);

    while (true) {
        try {
            auto frame = sub.pop_for(1000);

            // 使用相机图像作为画布（复制一份以免修改原图）
            cv::Mat canvas = frame.image.clone();

            // 如果有串口数据，绘制地平面
            if (frame.serial_valid) {
                // 从串口数据构建 RobotState
                aimer::RobotState state = aimer::RobotState::from_sync_frame(frame);

                // 绘制地平面网格和坐标轴
                draw_world_ground_grid(canvas, state.q_imu, 1.0, 10.0, -0.3);

                // 显示当前姿态角
                auto euler_imu = state.q_imu.toRotationMatrix().eulerAngles(2, 1, 0);
                double yaw = euler_imu[0] * 180.0 / M_PI;
                double pitch = euler_imu[1] * 180.0 / M_PI;
                double roll = euler_imu[2] * 180.0 / M_PI;

                std::string info = fmt::format("IMU: yaw={:.1f} pitch={:.1f} roll={:.1f}",
                                               yaw, pitch, roll);
                cv::putText(canvas, info, cv::Point(10, 30),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);

                // 显示串口原始数据
                std::string serial_info = fmt::format("Serial: yaw={:.1f} pitch={:.1f} roll={:.1f}",
                                                      frame.serial_data.yaw,
                                                      frame.serial_data.pitch,
                                                      frame.serial_data.roll);
                cv::putText(canvas, serial_info, cv::Point(10, 60),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(150, 150, 150), 2);

            } else {
                // 无串口数据时显示提示
                cv::putText(canvas, "Waiting for serial data...", cv::Point(50, 50),
                            cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 2);
            }

            // 显示图像
            cv::imshow(window_name, canvas);

            // 按键处理
            int key = cv::waitKey(1);
            if (key == 27) {  // ESC
                fmt::print(fmt::fg(fmt::color::green), "\n[退出]\n");
                break;
            }

        } catch (const umt::MessageError_Timeout&) {
            fmt::print(fmt::fg(fmt::color::yellow), "\r[超时等待帧...]");
            std::fflush(stdout);
        }
    }

    cv::destroyAllWindows();
    return 0;
}
