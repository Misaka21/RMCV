//
// Detector Helpers - 实现
//

#include "detector_helpers.hpp"

#include <fmt/format.h>
#include <opencv2/imgproc.hpp>

#ifdef RMCV_WITH_OPENCV_HIGHGUI
#include <opencv2/highgui.hpp>
#endif

#include "aimer/common/transformer/transformer.hpp"

namespace autoaim::detector {

namespace {
constexpr const char* DEBUG_WINDOW_NAME = "Detector Debug";
}

// ============================================================================
// 公共绘制
// ============================================================================

void draw_armors(
    cv::Mat& img,
    const std::vector<DetectedArmor>& armors,
    bool colorful
) {
    for (const auto& armor : armors) {
        if (armor.landmarks.size() < 4) continue;

        // 选择颜色
        cv::Scalar draw_color;
        if (colorful) {
            // 用敌方颜色的对立色绘制 (敌方红 → 蓝线, 敌方蓝 → 红线)
            draw_color = (armor.color == EnemyColor::RED)
                             ? cv::Scalar(255, 0, 0)
                             : cv::Scalar(0, 0, 255);
        } else {
            draw_color = cv::Scalar(0, 255, 0);  // 统一绿色
        }

        // 绘制轮廓
        for (size_t i = 0; i < armor.landmarks.size(); ++i) {
            cv::line(img, armor.landmarks[i],
                     armor.landmarks[(i + 1) % armor.landmarks.size()],
                     draw_color, 2, cv::LINE_AA);
        }

        // 绘制中心点
        cv::circle(img, armor.center, 5, cv::Scalar(0, 0, 255), -1);

        // 标注: 数字 类型 置信度
        char type_char = (armor.type == ArmorType::LARGE) ? 'L' : 'S';
        std::string text = fmt::format("{} {} {:.0f}%",
                                       armor_number_to_string(armor.number),
                                       type_char, armor.confidence * 100);
        cv::putText(img, text, armor.center + cv::Point2f(10, -10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2);
    }
}

// ============================================================================
// Web 调试 (轻量)
// ============================================================================

cv::Mat draw_debug_overlay(
    const cv::Mat& image,
    const std::vector<DetectedArmor>& armors,
    float fps,
    float latency_ms
) {
    cv::Mat debug_img = image.clone();

    // 绘制装甲板 (绿色)
    draw_armors(debug_img, armors, false);

    // 统计信息
    std::string info = fmt::format("FPS:{:.0f} Lat:{:.1f}ms Cnt:{}",
                                   fps, latency_ms, armors.size());
    cv::putText(debug_img, info, cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

    return debug_img;
}

// ============================================================================
// 本地调试 (完整)
// ============================================================================

void draw_world_ground_grid(
    cv::Mat& img,
    const Eigen::Quaterniond& q_imu,
    double grid_size,
    double range,
    double ground_z
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

void draw_debug_visualization(
    const cv::Mat& image,
    const aimer::DetectionResult& result,
    const hardware::SyncFrame& frame
) {
    cv::Mat debug_img = image.clone();

    // 绘制世界坐标系地面网格
    if (frame.serial_valid) {
        draw_world_ground_grid(debug_img, result.state.q_imu);
    }

    // 绘制装甲板 (彩色)
    draw_armors(debug_img, result.armors, true);

    // 显示统计信息
    std::string info = fmt::format("Armors: {} Latency: {:.1f}ms",
                                   result.armors.size(), result.latency_ms);
    cv::putText(debug_img, info, cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

    // 显示IMU信息
    if (frame.serial_valid) {
        // 原始串口数据
        std::string serial_info = fmt::format("Serial: yaw={:.1f} pitch={:.1f} roll={:.1f}",
                                              frame.serial_data.yaw,
                                              frame.serial_data.pitch,
                                              frame.serial_data.roll);
        cv::putText(debug_img, serial_info, cv::Point(10, 60),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(150, 150, 150), 2);

        // IMU 四元数转欧拉角 (ZYX顺序)
        auto euler_imu = result.state.q_imu.toRotationMatrix().eulerAngles(2, 1, 0);
        double yaw_imu = euler_imu[0] * 180.0 / M_PI;
        double pitch_imu = euler_imu[1] * 180.0 / M_PI;
        double roll_imu = euler_imu[2] * 180.0 / M_PI;

        std::string imu_info = fmt::format("IMU(q): yaw={:.1f} pitch={:.1f} roll={:.1f}",
                                           yaw_imu, pitch_imu, roll_imu);
        cv::putText(debug_img, imu_info, cv::Point(10, 90),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);

        // Gimbal 坐标系欧拉角 (修正IMU安装偏差后)
        // Gimbal → Imu 的旋转矩阵
        Eigen::Matrix3d R_gimbal2imu =
            aimer::tf::Transform<aimer::tf::Frame::Gimbal, aimer::tf::Frame::Imu>::R_;
        // q_gimbal = R_gimbal2imu.transpose() * q_imu
        Eigen::Quaterniond q_gimbal(R_gimbal2imu.transpose() * result.state.q_imu.toRotationMatrix());

        auto euler_gimbal = q_gimbal.toRotationMatrix().eulerAngles(2, 1, 0);
        double yaw_gimbal = euler_gimbal[0] * 180.0 / M_PI;
        double pitch_gimbal = euler_gimbal[1] * 180.0 / M_PI;
        double roll_gimbal = euler_gimbal[2] * 180.0 / M_PI;

        std::string gimbal_info = fmt::format("Gimbal: yaw={:.1f} pitch={:.1f} roll={:.1f}",
                                              yaw_gimbal, pitch_gimbal, roll_gimbal);
        cv::putText(debug_img, gimbal_info, cv::Point(10, 120),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 150, 0), 2);
    }

    // 显示窗口
#ifdef RMCV_WITH_OPENCV_HIGHGUI
    cv::imshow(DEBUG_WINDOW_NAME, debug_img);
    cv::waitKey(1);
#else
    (void)debug_img;
#endif
}

void close_debug_window() {
#ifdef RMCV_WITH_OPENCV_HIGHGUI
    cv::destroyWindow(DEBUG_WINDOW_NAME);
#endif
}

}  // namespace autoaim::detector
