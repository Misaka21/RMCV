//
// 检测器公共类型定义
// 所有检测器实现都使用这些类型作为输出
//

#ifndef DETECTOR_COMMON_TYPES_HPP
#define DETECTOR_COMMON_TYPES_HPP

#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace autoaim::detector {

// ============================================================================
// 1. 枚举类型
// ============================================================================

// 敌方颜色
enum class EnemyColor { RED, BLUE, WHITE };

// 装甲板类型
enum class ArmorType { SMALL, LARGE, INVALID };

inline std::string armor_type_to_string(ArmorType type) {
    switch (type) {
        case ArmorType::SMALL: return "small";
        case ArmorType::LARGE: return "large";
        default: return "invalid";
    }
}

// ============================================================================
// 2. 装甲板尺寸常量 (单位: m)
// ============================================================================

constexpr double SMALL_ARMOR_WIDTH  = 0.133;  // 小装甲板宽度
constexpr double SMALL_ARMOR_HEIGHT = 0.050;  // 小装甲板高度
constexpr double LARGE_ARMOR_WIDTH  = 0.225;  // 大装甲板宽度
constexpr double LARGE_ARMOR_HEIGHT = 0.050;  // 大装甲板高度

// ============================================================================
// 3. 检测结果结构体 (公共输出类型)
// ============================================================================

/**
 * @brief 检测到的装甲板 - 所有检测器的统一输出类型
 *
 * 无论是传统检测还是YOLO检测，最终都输出这个结构
 * 与内部实现解耦（传统检测器内部可以有Light/Armor，但输出转为DetectedArmor）
 */
struct DetectedArmor {
    // 关键点 (图像坐标)
    // 4点: 左下、左上、右上、右下
    std::vector<cv::Point2f> landmarks;

    // 装甲板中心 (图像坐标)
    cv::Point2f center;

    // 装甲板类型
    ArmorType type = ArmorType::INVALID;

    // 数字识别结果
    std::string number;
    float confidence = 0.0f;

    DetectedArmor() = default;

    /**
     * @brief 获取PnP用的物体坐标系点
     * @return 3D点列表，与landmarks一一对应
     *
     * 物体坐标系: x前(指向相机), y左, z上
     * 点序: 左下、左上、右上、右下
     */
    std::vector<cv::Point3f> object_points() const {
        double w = (type == ArmorType::LARGE) ? LARGE_ARMOR_WIDTH : SMALL_ARMOR_WIDTH;
        double h = (type == ArmorType::LARGE) ? LARGE_ARMOR_HEIGHT : SMALL_ARMOR_HEIGHT;

        return {
            cv::Point3f(0, w / 2, -h / 2),   // 左下
            cv::Point3f(0, w / 2, h / 2),    // 左上
            cv::Point3f(0, -w / 2, h / 2),   // 右上
            cv::Point3f(0, -w / 2, -h / 2)   // 右下
        };
    }

    /**
     * @brief 从内部Armor类型转换 (供传统检测器使用)
     */
    template<typename InternalArmor>
    static DetectedArmor from_internal(const InternalArmor& armor) {
        DetectedArmor result;
        result.landmarks = armor.landmarks();
        result.center = armor.center;
        result.type = armor.type;
        result.number = armor.number;
        result.confidence = armor.confidence;
        return result;
    }
};

} // namespace autoaim::detector

#endif // DETECTOR_COMMON_TYPES_HPP
