//
// 自瞄公共类型定义
// 所有自瞄模块共享的类型
//

#ifndef AUTOAIM_COMMON_TYPES_HPP
#define AUTOAIM_COMMON_TYPES_HPP

#include <string>
#include <unordered_map>
#include <vector>

#include <opencv2/core.hpp>

namespace autoaim {

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
// 2. 数字识别结果
// ============================================================================

// 装甲板数字/标签
enum class ArmorNumber {
    UNKNOWN = -1,
    HERO = 1,        // 1号 (英雄) - 大装甲板
    ENGINEER = 2,    // 2号 (工程)
    INFANTRY_3 = 3,  // 3号 (步兵)
    INFANTRY_4 = 4,  // 4号 (步兵)
    INFANTRY_5 = 5,  // 5号 (步兵)
    OUTPOST = 6,     // 前哨站
    SENTRY = 7,      // 哨兵
    BASE = 8         // 基地 - 大小都有
};

// 字符串 ↔ 枚举映射
inline const std::unordered_map<std::string, ArmorNumber> STR_TO_ARMOR_NUMBER = {
    {"1", ArmorNumber::HERO},
    {"2", ArmorNumber::ENGINEER},
    {"3", ArmorNumber::INFANTRY_3},
    {"4", ArmorNumber::INFANTRY_4},
    {"5", ArmorNumber::INFANTRY_5},
    {"outpost", ArmorNumber::OUTPOST},
    {"sentry", ArmorNumber::SENTRY},
    {"base", ArmorNumber::BASE}
};

inline const std::unordered_map<ArmorNumber, std::string> ARMOR_NUMBER_TO_STR = {
    {ArmorNumber::UNKNOWN, "unknown"},
    {ArmorNumber::HERO, "1"},
    {ArmorNumber::ENGINEER, "2"},
    {ArmorNumber::INFANTRY_3, "3"},
    {ArmorNumber::INFANTRY_4, "4"},
    {ArmorNumber::INFANTRY_5, "5"},
    {ArmorNumber::OUTPOST, "outpost"},
    {ArmorNumber::SENTRY, "sentry"},
    {ArmorNumber::BASE, "base"}
};

inline ArmorNumber string_to_armor_number(const std::string& s) {
    auto it = STR_TO_ARMOR_NUMBER.find(s);
    return (it != STR_TO_ARMOR_NUMBER.end()) ? it->second : ArmorNumber::UNKNOWN;
}

inline std::string armor_number_to_string(ArmorNumber n) {
    auto it = ARMOR_NUMBER_TO_STR.find(n);
    return (it != ARMOR_NUMBER_TO_STR.end()) ? it->second : "unknown";
}

/**
 * @brief 根据数字识别结果修正装甲板类型
 *
 * 规则:
 * - 1号(英雄): 必定大装甲板
 * - base(基地): 保持检测结果 (大小都有)
 * - 其他: 必定小装甲板
 */
inline ArmorType correct_armor_type(ArmorType detected, ArmorNumber number) {
    switch (number) {
        case ArmorNumber::HERO:
            return ArmorType::LARGE;
        case ArmorNumber::BASE:
            return detected;  // 基地大小都有，保持检测结果
        case ArmorNumber::UNKNOWN:
            return detected;  // 未识别，保持检测结果
        default:
            return ArmorType::SMALL;
    }
}

// ============================================================================
// 3. 装甲板尺寸常量 (单位: m)
// ============================================================================

constexpr double SMALL_ARMOR_WIDTH  = 0.133;  // 小装甲板宽度
constexpr double SMALL_ARMOR_HEIGHT = 0.050;  // 小装甲板高度
constexpr double LARGE_ARMOR_WIDTH  = 0.225;  // 大装甲板宽度
constexpr double LARGE_ARMOR_HEIGHT = 0.050;  // 大装甲板高度

// ============================================================================
// 4. 检测结果结构体
// ============================================================================

/**
 * @brief 检测到的装甲板 - 所有检测器的统一输出类型
 */
struct DetectedArmor {
    // 关键点 (图像坐标), 4点: 左下、左上、右上、右下
    std::vector<cv::Point2f> landmarks;

    // 装甲板中心 (图像坐标)
    cv::Point2f center;

    // 装甲板类型 (已根据数字识别修正)
    ArmorType type = ArmorType::INVALID;

    // 装甲板颜色
    EnemyColor color = EnemyColor::WHITE;

    // 数字识别结果
    ArmorNumber number = ArmorNumber::UNKNOWN;
    float confidence = 0.0f;

    DetectedArmor() = default;

    /**
     * @brief 获取PnP用的物体坐标系点
     * 物体坐标系: x前(指向相机), y左, z上
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
     * @brief 从内部Armor类型转换
     */
    template<typename InternalArmor>
    static DetectedArmor from_internal(const InternalArmor& armor) {
        DetectedArmor result;
        result.landmarks = armor.landmarks();
        result.center = armor.center;
        result.color = armor.left_light.color;
        result.confidence = armor.confidence;
        result.number = string_to_armor_number(armor.number);
        result.type = correct_armor_type(armor.type, result.number);
        return result;
    }
};

} // namespace autoaim

#endif // AUTOAIM_COMMON_TYPES_HPP
