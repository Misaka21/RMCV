//
// Detector Node - 检测节点
// 订阅 sync_frame，运行装甲板检测，发布检测结果
//

#ifndef AIMER_AUTOAIM_DETECTOR_NODE_HPP
#define AIMER_AUTOAIM_DETECTOR_NODE_HPP

#include <fmt/format.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "hardware/hardware_node.hpp"  // hardware::SyncFrame
#include "aimer/common/types.hpp"      // aimer::RobotState, aimer::DetectionResult
#include "aimer/common/transformer/transformer.hpp"  // tf::world_to_pixel
#include "common/detector_interface.hpp"
#include "common/types.hpp"            // aimer::autoaim::DetectedArmor

namespace autoaim {

/**
 * @brief 绘制世界坐标系地面网格，用于验证坐标变换是否正确
 * @param img 要绘制的图像
 * @param q_imu IMU四元数
 * @param grid_size 网格间距 (米)
 * @param range 绘制范围 (米)
 * @param ground_z 地面高度 (米，相对于上电时云台位置)
 */
inline void draw_world_ground_grid(
    cv::Mat& img,
    const Eigen::Quaterniond& q_imu,
    double grid_size = 0.5,
    double range = 10.0,
    double ground_z = -0.5  // 假设地面在云台下方30cm
) {
    // 世界坐标系: x前, y左, z上
    // 地面是z = ground_z的平面
    for (double x = 0; x <= range; x += grid_size) {
        for (double y = -range; y <= range; y += grid_size) {
            Eigen::Vector3d p_world(x, y, ground_z);
            bool valid = false;
            cv::Point2f pixel = tf::world_to_pixel(p_world, q_imu, valid);

            if (valid && pixel.x >= 0 && pixel.x < img.cols && pixel.y >= 0 && pixel.y < img.rows) {
                // 前方用绿色，左右用不同亮度
                int brightness = static_cast<int>(255 - std::abs(y) / range * 150);
                cv::Scalar color(0, brightness, 0);

                // X轴上的点用红色标记
                if (std::abs(y) < 0.01) {
                    color = cv::Scalar(0, 0, 255);  // 红色
                }
                // Y轴上的点用蓝色标记 (x=0的情况)
                if (std::abs(x) < 0.01) {
                    color = cv::Scalar(255, 0, 0);  // 蓝色
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

/**
 * @brief 绘制调试可视化
 */
inline void draw_debug_visualization(
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
        if (armor.landmarks.size() >= 4) {
            // 用我方颜色绘制 (敌方红色 → 我方蓝色线)
            cv::Scalar draw_color = (armor.color == detector::EnemyColor::RED)
                ? cv::Scalar(255, 0, 0)   // 敌方红色 → 蓝色线
                : cv::Scalar(0, 0, 255);  // 敌方蓝色 → 红色线

            // 绘制轮廓
            for (size_t i = 0; i < armor.landmarks.size(); ++i) {
                cv::line(
                    debug_img,
                    armor.landmarks[i],
                    armor.landmarks[(i + 1) % armor.landmarks.size()],
                    draw_color,
                    2,
                    cv::LINE_AA
                );
            }

            // 显示: 数字 类型 置信度%
            char type_char = (armor.type == detector::ArmorType::LARGE) ? 'L' : 'S';
            std::string text = fmt::format(
                "{} {} {:.0f}%",
                detector::armor_number_to_string(armor.number),
                type_char,
                armor.confidence * 100
            );
            cv::putText(
                debug_img,
                text,
                armor.center - cv::Point2f(30, 10),
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                cv::Scalar(0, 255, 255),
                2
            );
        }
    }

    // 显示统计信息
    std::string info = fmt::format(
        "Armors: {} Latency: {:.1f}ms",
        result.armors.size(),
        result.latency_ms
    );
    cv::putText(
        debug_img,
        info,
        cv::Point(10, 30),
        cv::FONT_HERSHEY_SIMPLEX,
        0.7,
        cv::Scalar(0, 255, 0),
        2
    );

    // 显示IMU信息
    if (frame.serial_valid) {
        std::string imu_info = fmt::format(
            "IMU: yaw={:.1f} pitch={:.1f}",
            frame.serial_data.yaw,
            frame.serial_data.pitch
        );
        cv::putText(
            debug_img,
            imu_info,
            cv::Point(10, 60),
            cv::FONT_HERSHEY_SIMPLEX,
            0.7,
            cv::Scalar(0, 255, 255),
            2
        );
    }

    cv::Mat display;
    //cv::resize(debug_img, display, cv::Size(960, 720));
    cv::imshow("Detector Debug", debug_img);
    cv::waitKey(1);
}

/**
 * @brief 启动检测节点
 *
 * 订阅: Message<hardware::SyncFrame> "sync_frame"
 * 发布: Message<aimer::DetectionResult> "detections"
 */
void start_detector_node();

}  // namespace autoaim

#endif  // AIMER_AUTOAIM_DETECTOR_NODE_HPP
