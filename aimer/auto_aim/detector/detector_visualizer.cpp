//
// Detector Visualizer - 实现
//

#include "detector_visualizer.hpp"

#include <fmt/format.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "aimer/common/transformer/transformer.hpp"

namespace autoaim::detector {

namespace {
constexpr const char* DEBUG_WINDOW_NAME = "Detector Debug";
}

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
            cv::Point2f pixel = tf::world_to_pixel(p_world, q_imu, valid);

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

    // 绘制装甲板
    for (const auto& armor : result.armors) {
        if (armor.landmarks.size() < 4) continue;

        // 用我方颜色绘制 (敌方红色 → 蓝色线)
        cv::Scalar draw_color = (armor.color == EnemyColor::RED)
                                    ? cv::Scalar(255, 0, 0)
                                    : cv::Scalar(0, 0, 255);

        // 绘制轮廓
        for (size_t i = 0; i < armor.landmarks.size(); ++i) {
            cv::line(debug_img, armor.landmarks[i],
                     armor.landmarks[(i + 1) % armor.landmarks.size()],
                     draw_color, 2, cv::LINE_AA);
        }

        // 显示: 数字 类型 置信度%
        char type_char = (armor.type == ArmorType::LARGE) ? 'L' : 'S';
        std::string text = fmt::format("{} {} {:.0f}%",
                                       armor_number_to_string(armor.number),
                                       type_char, armor.confidence * 100);
        cv::putText(debug_img, text, armor.center - cv::Point2f(30, 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2);
    }

    // 显示统计信息
    std::string info = fmt::format("Armors: {} Latency: {:.1f}ms",
                                   result.armors.size(), result.latency_ms);
    cv::putText(debug_img, info, cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

    // 显示IMU信息
    if (frame.serial_valid) {
        std::string imu_info = fmt::format("IMU: yaw={:.1f} pitch={:.1f}",
                                           frame.serial_data.yaw, frame.serial_data.pitch);
        cv::putText(debug_img, imu_info, cv::Point(10, 60),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
    }

    show_debug_window(debug_img);
}

void show_debug_window(const cv::Mat& image) {
    cv::imshow(DEBUG_WINDOW_NAME, image);
    cv::waitKey(1);
}

void close_debug_window() {
    cv::destroyWindow(DEBUG_WINDOW_NAME);
}

}  // namespace autoaim::detector
