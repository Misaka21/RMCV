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
#include "common/detector_interface.hpp"
#include "common/types.hpp"            // aimer::autoaim::DetectedArmor

namespace autoaim {

/**
 * @brief 绘制调试可视化
 */
inline void draw_debug_visualization(
    const cv::Mat& image,
    const aimer::DetectionResult& result,
    const hardware::SyncFrame& frame
) {
    cv::Mat debug_img = image.clone();

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
    cv::resize(debug_img, display, cv::Size(960, 720));
    cv::imshow("Detector Debug", display);
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
