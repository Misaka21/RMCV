//
// 检测器抽象接口
// 所有检测器实现都必须继承此接口
//

#ifndef AIMER_AUTOAIM_DETECTOR_INTERFACE_HPP
#define AIMER_AUTOAIM_DETECTOR_INTERFACE_HPP

#include <memory>
#include <vector>

#include <opencv2/core.hpp>

#include "types.hpp"

namespace autoaim::detector {

/**
 * @brief 检测器抽象接口
 *
 * 所有检测器（传统/YOLO/...）都必须实现此接口
 * 工厂返回 std::unique_ptr<DetectorInterface>，调用者不需要知道具体实现
 */
class DetectorInterface {
public:
    virtual ~DetectorInterface() = default;

    /**
     * @brief 检测装甲板
     * @param image 输入图像 (BGR格式)
     * @return 检测到的装甲板列表
     */
    virtual std::vector<DetectedArmor> detect(const cv::Mat& image) = 0;

    /**
     * @brief 设置敌方颜色
     * @param color 敌方颜色
     */
    virtual void set_enemy_color(EnemyColor color) = 0;

    /**
     * @brief 获取当前敌方颜色
     */
    virtual EnemyColor get_enemy_color() const = 0;

    /**
     * @brief 获取调试图像 (可选)
     * @return 带有检测结果标注的图像，如果不支持返回空Mat
     */
    virtual cv::Mat debug_image() const { return {}; }
};

// 检测器类型枚举
enum class DetectorType {
    TRADITIONAL,  // 传统检测 (灯条匹配)
    YOLO          // YOLO检测 (神经网络)
};

}  // namespace autoaim::detector

#endif  // AIMER_AUTOAIM_DETECTOR_INTERFACE_HPP
