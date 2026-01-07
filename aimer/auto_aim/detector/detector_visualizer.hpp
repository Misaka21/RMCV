//
// Detector Visualizer - 检测器可视化
// 用于本地 OpenCV 窗口调试
//

#ifndef AIMER_AUTOAIM_DETECTOR_VISUALIZER_HPP
#define AIMER_AUTOAIM_DETECTOR_VISUALIZER_HPP

#include <Eigen/Geometry>
#include <opencv2/core/mat.hpp>

#include "aimer/common/types.hpp"
#include "common/types.hpp"
#include "hardware/hardware_node.hpp"

namespace autoaim::detector {

/**
 * @brief 绘制世界坐标系地面网格
 *
 * 用于验证坐标变换是否正确
 *
 * @param img 要绘制的图像
 * @param q_imu IMU四元数
 * @param grid_size 网格间距 (米)
 * @param range 绘制范围 (米)
 * @param ground_z 地面高度 (米，相对于云台)
 */
void draw_world_ground_grid(
    cv::Mat& img,
    const Eigen::Quaterniond& q_imu,
    double grid_size = 0.5,
    double range = 10.0,
    double ground_z = -0.5
);

/**
 * @brief 绘制完整调试可视化
 *
 * 包含: 装甲板轮廓、数字标注、IMU信息、世界坐标网格
 *
 * @param image 原始图像
 * @param result 检测结果
 * @param frame 同步帧数据
 */
void draw_debug_visualization(
    const cv::Mat& image,
    const aimer::DetectionResult& result,
    const hardware::SyncFrame& frame
);

/**
 * @brief 显示调试窗口
 */
void show_debug_window(const cv::Mat& image);

/**
 * @brief 关闭调试窗口
 */
void close_debug_window();

}  // namespace autoaim::detector

#endif  // AIMER_AUTOAIM_DETECTOR_VISUALIZER_HPP
