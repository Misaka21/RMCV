//
// 能量机关检测公共类型定义
// 所有能量机关模块共享的类型
//

#ifndef AIMER_AUTOBUFF_COMMON_TYPES_HPP
#define AIMER_AUTOBUFF_COMMON_TYPES_HPP

#include <array>
#include <cmath>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include "aimer/common/robot_state.hpp"

namespace autobuff {

// ============================================================================
// 常量定义
// ============================================================================

constexpr int NUM_SLOTS = 5;           // 能量机关扇叶数
constexpr int TARGET_KEYPOINTS = 5;    // 靶心关键点: 4 gaps + center
constexpr int ARROW_KEYPOINTS = 2;     // 箭头关键点: tip + tail

// ============================================================================
// 能量机关尺寸常量 (单位: m)
// ============================================================================

// R标尺寸
constexpr double R_CENTER_RADIUS = 0.040;       // R标半径 40mm

// 靶心尺寸 (内环到外环)
constexpr double TARGET_INNER_RADIUS = 0.040;   // 靶心内环半径
constexpr double TARGET_OUTER_RADIUS = 0.070;   // 靶心外环半径

// 能量机关旋转半径 (R标到靶心中心)
constexpr double RUNE_RADIUS = 0.700;           // 旋转半径 700mm

// ============================================================================
// 枚举类型
// ============================================================================

// 敌方颜色 (与 autoaim 保持一致但独立定义)
enum class EnemyColor : uint8_t {
    UNKNOWN = 0,
    RED = 1,
    BLUE = 2
};

// 检测状态
enum class DetectionStatus : uint8_t {
    NONE,           // 无检测
    R_ONLY,         // 仅检测到R标
    TARGET_ONLY,    // 仅检测到靶心
    PARTIAL,        // 部分检测 (R标 + 部分靶心)
    COMPLETE        // 完整检测 (R标 + 靶心 + 箭头)
};

// ============================================================================
// 检测结构体
// ============================================================================

/**
 * @brief R标检测结果
 *
 * 能量机关中心的 R 字标识
 */
struct DetectedRCenter {
    cv::Point2f center;                         // 中心点 (必须)
    std::vector<cv::Point2f> landmarks;         // 可选: R字4角点或轮廓点
    Eigen::Vector3d position = Eigen::Vector3d::Zero();  // 3D位置 (相机坐标系)
    bool valid = false;
    float confidence = 0;

    /**
     * @brief 获取PnP用的物体坐标系点
     *
     * 以R标中心为原点，返回4个角点用于PnP
     */
    std::vector<cv::Point3f> object_points() const {
        double r = R_CENTER_RADIUS;
        return {
            cv::Point3f(-r, -r, 0),  // 左上
            cv::Point3f(-r,  r, 0),  // 左下
            cv::Point3f( r,  r, 0),  // 右下
            cv::Point3f( r, -r, 0)   // 右上
        };
    }
};

/**
 * @brief 靶心检测结果
 *
 * 包含5个关键点: center + 4个缺口位置
 * 缺口顺序: top(0), right(1), bottom(2), left(3), center(4)
 *
 *      [0] (top gap)
 *         /\
 *    [3]/    \[1]
 *      \  *  /     * = center
 *    [2]\    /
 *         \/
 */
struct DetectedTarget {
    cv::Point2f center;                         // 中心点 (必须)
    std::vector<cv::Point2f> landmarks;         // 5点: 4 gaps + center
    Eigen::Vector3d position = Eigen::Vector3d::Zero();  // 3D位置 (相机坐标系)

    int slot_id = -1;                           // 槽位ID (0-4)
    double angle_from_center = 0;               // 相对R标的角度 (rad)
    bool is_active = false;                     // true=已击打(亮), false=待击打(暗)
    bool valid = false;
    float confidence = 0;

    /**
     * @brief 获取PnP用的物体坐标系点
     *
     * 以靶心中心为原点，返回关键点
     */
    std::vector<cv::Point3f> object_points() const {
        double r = TARGET_OUTER_RADIUS;
        // 如果有5点关键点，返回对应的3D点
        if (landmarks.size() >= 5) {
            return {
                cv::Point3f(0, -r, 0),   // top gap
                cv::Point3f(r,  0, 0),   // right gap
                cv::Point3f(0,  r, 0),   // bottom gap
                cv::Point3f(-r, 0, 0),   // left gap
                cv::Point3f(0,  0, 0)    // center
            };
        }
        // 否则返回4角点
        return {
            cv::Point3f(-r, -r, 0),
            cv::Point3f(-r,  r, 0),
            cv::Point3f( r,  r, 0),
            cv::Point3f( r, -r, 0)
        };
    }
};

/**
 * @brief 箭头检测结果
 *
 * 箭头指向待打击的靶心
 */
struct DetectedArrow {
    cv::Point2f tip;                            // 箭头尖端 (指向R标方向)
    cv::Point2f tail;                           // 箭头尾部 (指向靶心方向)

    std::vector<cv::Point2f> get_landmarks() const { return {tip, tail}; }

    double direction_angle = 0;                 // 箭头方向角 (rad, 相对水平)
    int target_slot_id = -1;                    // 指向的靶心槽位ID
    bool valid = false;
    float confidence = 0;

    /**
     * @brief 获取箭头中心点
     */
    cv::Point2f center() const {
        return (tip + tail) * 0.5f;
    }
};

// ============================================================================
// 检测结果
// ============================================================================

/**
 * @brief 能量机关检测结果 (检测器 -> 预测器)
 */
struct BuffDetectionResult {
    // 检测到的特征
    DetectedRCenter r_center;
    std::array<DetectedTarget, NUM_SLOTS> targets;
    DetectedArrow arrow;

    // 汇总信息
    int active_slot_id = -1;                    // 待打击靶心的槽位ID (箭头指向)
    int target_count = 0;                       // 检测到的靶心数量
    DetectionStatus status = DetectionStatus::NONE;
    EnemyColor enemy_color = EnemyColor::UNKNOWN;

    // 帧信息
    int frame_id = 0;
    double timestamp = 0;                       // 秒
    float latency_ms = 0;                       // 检测耗时
    aimer::RobotState robot_state;

    // 调试图像 (可选)
    cv::Mat image;

    // ========== 辅助方法 ==========

    bool has_r_center() const { return r_center.valid; }

    bool has_active_target() const {
        return active_slot_id >= 0 && active_slot_id < NUM_SLOTS
               && targets[active_slot_id].valid;
    }

    const DetectedTarget* get_active_target() const {
        if (has_active_target()) {
            return &targets[active_slot_id];
        }
        return nullptr;
    }

    /**
     * @brief 获取所有有效靶心
     */
    std::vector<const DetectedTarget*> get_valid_targets() const {
        std::vector<const DetectedTarget*> result;
        for (const auto& t : targets) {
            if (t.valid) {
                result.push_back(&t);
            }
        }
        return result;
    }

    /**
     * @brief 检测是否完整
     */
    bool is_complete() const {
        return status == DetectionStatus::COMPLETE;
    }

    /**
     * @brief 更新检测状态
     */
    void update_status() {
        target_count = 0;
        for (const auto& t : targets) {
            if (t.valid) target_count++;
        }

        if (!r_center.valid && target_count == 0) {
            status = DetectionStatus::NONE;
        } else if (r_center.valid && target_count == 0) {
            status = DetectionStatus::R_ONLY;
        } else if (!r_center.valid && target_count > 0) {
            status = DetectionStatus::TARGET_ONLY;
        } else if (r_center.valid && arrow.valid && active_slot_id >= 0) {
            status = DetectionStatus::COMPLETE;
        } else {
            status = DetectionStatus::PARTIAL;
        }
    }
};

}  // namespace autobuff

#endif  // AIMER_AUTOBUFF_COMMON_TYPES_HPP
