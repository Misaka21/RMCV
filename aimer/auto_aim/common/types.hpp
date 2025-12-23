//
// 自瞄公共类型定义
// 所有自瞄模块共享的类型
//

#ifndef AIMER_AUTOAIM_COMMON_TYPES_HPP
#define AIMER_AUTOAIM_COMMON_TYPES_HPP

#include <array>
#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include "aimer/common/math/math.hpp"

namespace autoaim {

// 最大目标数量
constexpr int MAX_ENEMY_NUMBER = 8;

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

// 装甲板数字/标签 (也用作目标ID)
enum class ArmorNumber {
    UNKNOWN = 0,     // 未知 (也作为数组默认索引)
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

// ============================================================================
// 5. 帧级检测结果
// ============================================================================

/**
 * @brief 单帧检测结果 (检测器 → 预测器)
 */
struct DetectionResult {
    cv::Mat img;                           // 原始图像 (可选，调试用)
    Eigen::Quaterniond q_imu;              // IMU 四元数
    double timestamp = 0;                  // 时间戳 (s)
    std::vector<DetectedArmor> armors;     // 检测到的装甲板
    int frame_id = 0;                      // 帧编号
};

// ============================================================================
// 6. PnP 解算后的完整装甲板信息
// ============================================================================

/**
 * @brief 装甲板完整信息 (PnP 后)
 *
 * 保留 2D 检测 + 3D 解算结果，用于:
 * - 重投影误差计算
 * - EKF 残差验证
 * - 调试可视化
 */
struct ArmorInfo {
    // ==================== 原始检测 ====================

    int frame_id = 0;                           // 所属帧
    DetectedArmor detected;                     // 原始 2D 检测
    std::array<cv::Point2f, 4> pts_undistorted; // 畸变矫正后的坐标

    // ==================== PnP 解算结果 ====================

    Eigen::Vector3d pos = Eigen::Vector3d::Zero();   // 世界坐标系位置 (m)
    Eigen::Quaterniond rotation_q = Eigen::Quaterniond::Identity();  // 装甲板朝向
    math::YpdCoord orientation_yp;                   // 装甲板朝向 (yaw, pitch, dist)

    // ==================== 额外信息 ====================

    double z_to_v = 0;        // 装甲板法向与视线夹角 (rad)
    double timestamp = 0;     // 时间戳 (s)

    // ==================== 辅助方法 ====================

    /**
     * @brief 检查数据是否有效
     */
    bool valid() const {
        // 目标编号有效性
        int num = static_cast<int>(detected.number);
        if (num < 0 || num > MAX_ENEMY_NUMBER) {
            return false;
        }
        // 检查 2D 点是否有 NaN/Inf
        for (int i = 0; i < 4; ++i) {
            if (detected.landmarks.size() <= static_cast<size_t>(i)) {
                return false;
            }
            if (std::isnan(detected.landmarks[i].x) || std::isinf(detected.landmarks[i].x) ||
                std::isnan(detected.landmarks[i].y) || std::isinf(detected.landmarks[i].y)) {
                return false;
            }
        }
        // 检查 3D 位置是否有 NaN/Inf
        for (int i = 0; i < 3; ++i) {
            if (std::isnan(pos(i)) || std::isinf(pos(i))) {
                return false;
            }
        }
        return true;
    }

    /**
     * @brief 装甲板面积 (像素)
     */
    float area() const {
        if (detected.landmarks.size() < 4) return 0;
        return math::get_area(detected.landmarks);
    }

    /**
     * @brief 装甲板中心 (图像坐标)
     */
    cv::Point2f center() const {
        return detected.center;
    }

    /**
     * @brief 目标编号 (1-8)
     */
    int target_id() const {
        return static_cast<int>(detected.number);
    }

    /**
     * @brief 3D 距离 (m)
     */
    double distance() const {
        return pos.norm();
    }
};

// ============================================================================
// 7. 装甲板数据 (带颜色判断)
// ============================================================================

/**
 * @brief 装甲板数据实例
 */
struct ArmorData {
    int id;                  // 装甲板 ID
    EnemyColor our_color;    // 我方颜色
    ArmorInfo info;          // 装甲板信息

    ArmorData(int id, EnemyColor color, const ArmorInfo& info)
        : id(id), our_color(color), info(info) {}

    /**
     * @brief 是否被击中 (颜色不匹配)
     */
    bool is_hit() const {
        return our_color != info.detected.color;
    }
};

}  // namespace autoaim

#endif  // AIMER_AUTOAIM_COMMON_TYPES_HPP
