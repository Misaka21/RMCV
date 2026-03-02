// 能量机关模块公共类型 (2026 规则)
//
// 关键变化:
// - 不再依赖“箭头”指示目标
// - 直接输出“被点亮的装甲模块”集合 (small=1, large=2)
// - slot_id 仅表示 72deg 网格上的槽位编号 (0~4)，本身没有物理身份含义

#ifndef AIMER_AUTOBUFF_COMMON_TYPES_HPP
#define AIMER_AUTOBUFF_COMMON_TYPES_HPP

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include "aimer/common/robot_state.hpp"

namespace autobuff {

// ============================================================================
// 常量定义
// ============================================================================

constexpr int NUM_SLOTS = 5;  // 能量机关扇叶数

// ============================================================================
// 尺寸常量 (单位: m)
// ============================================================================

// R 标 / 中心
constexpr double R_CENTER_RADIUS = 0.040;  // 40mm (仅调试用)

// 扇叶尖端半径 (PnP/调试可用，具体用法由 predictor 决定)
constexpr double TARGET_TIP_RADIUS = 0.095;  // 95mm

// R 标到靶心中心的旋转半径
constexpr double RUNE_RADIUS = 0.700;  // 700mm

// ============================================================================
// 枚举类型
// ============================================================================

// 敌方颜色 (与串口约定一致: 0/1/2)
enum class EnemyColor : uint8_t {
    UNKNOWN = 0,
    RED = 1,
    BLUE = 2
};

// 能量机关模式 (predictor 使用)
enum class BuffMode : uint8_t {
    UNKNOWN = 0,        // 非能量机关模式
    SMALL_ACTIVE = 1,   // 小符 (1块亮)
    LARGE_INACTIVE = 2, // 大符非激活 (0块亮，恒速)
    LARGE_ACTIVE = 3,   // 大符激活 (2块亮，变速)
};

inline std::string buff_mode_name(BuffMode m) {
    switch (m) {
        case BuffMode::UNKNOWN:        return "UNKNOWN";
        case BuffMode::SMALL_ACTIVE:   return "SMALL_ACTIVE";
        case BuffMode::LARGE_INACTIVE: return "LARGE_INACTIVE";
        case BuffMode::LARGE_ACTIVE:   return "LARGE_ACTIVE";
    }
    return "?";
}

// 旋转方向
enum class RotateDir : int8_t {
    UNKNOWN = 0,
    CW = -1,    // 顺时针
    CCW = 1,    // 逆时针
};

// 双车协同角色
enum class CoopRole : uint8_t {
    DISABLED = 0,     // 不协同，打最优目标
    CCW_FIRST = 1,    // 打逆时针方向第 1 个 lit slot
    CCW_SECOND = 2,   // 打逆时针方向第 2 个 lit slot
};

// 推理后端类型
enum class DetectorBackend : uint8_t {
    TRADITIONAL = 0,
    OPENVINO = 1,
    TENSORRT = 2,
};

// 检测状态 (面向 pipeline 的粗粒度状态)
enum class DetectionStatus : uint8_t {
    NONE = 0,        // 无检测
    R_ONLY = 1,      // 仅检测到 R 标
    TARGETS_ONLY = 2,// 仅检测到靶心
    PARTIAL = 3,     // R + 部分靶心
    COMPLETE = 4     // R + 靶心 (足以预测/火控)
};

// ============================================================================
// 检测结构体
// ============================================================================

/**
 * @brief R 标检测结果
 */
struct DetectedRCenter {
    cv::Point2f center{};
    std::vector<cv::Point2f> landmarks;  // 可选: 角点/轮廓点，用于调试或PnP扩展
    bool valid = false;
    float confidence = 0.f;
};

/**
 * @brief 单个扇叶靶心检测结果
 *
 * 注意: 这里的 is_lit 表示“被点亮(可打)” (对应 2026 规则)。
 * slot_id 仅表示 72deg 网格编号 (0~4)，用于构造刚体点集 / 相位展开，不代表物理身份。
 */
struct DetectedTarget {
    cv::Point2f center{};

    // 可选关键点: [center, top, right, bottom, left]
    // - Inactive(暗) 通常能得到 4 个 fan tips
    // - Active(亮) 可能只有 center
    std::vector<cv::Point2f> landmarks;

    // 6 个关键点 (从 YOLO sp25 模型):
    // kpt[0-3]: 扇叶四角 (左上逆时针)
    // kpt[4]:   扇叶中心
    // kpt[5]:   内侧尖端 (指向R标方向)
    std::array<cv::Point2f, 6> keypoints{};
    uint8_t keypoint_count = 0;

    int slot_id = -1;     // 0~4

    // 相对 R 标的角度 (rad)，采用数学坐标系 (x 右, y 上)，范围 (-pi, pi]
    double angle = 0.0;

    bool is_lit = false;  // 是否“被点亮”(可打)
    bool valid = false;
    float confidence = 0.f;
};

inline bool is_valid_slot(int id) { return id >= 0 && id < NUM_SLOTS; }

// ============================================================================
// 帧级检测结果 (detector -> predictor)
// ============================================================================

struct BuffDetectionResult {
    DetectedRCenter r_center;
    std::array<DetectedTarget, NUM_SLOTS> targets{};

    // bit i = 1 表示 slot i 被点亮 (可打)
    uint8_t lit_mask = 0;

    int target_count = 0;
    int lit_count = 0;

    DetectionStatus status = DetectionStatus::NONE;
    EnemyColor enemy_color = EnemyColor::UNKNOWN;
    DetectorBackend backend = DetectorBackend::TRADITIONAL;

    int frame_id = 0;
    double timestamp = 0.0;     // 秒 (steady_clock)
    float latency_ms = 0.0f;
    aimer::RobotState robot_state;

    // 原图像 (可选，调试/录制用)
    cv::Mat image;

    // ========= helpers =========

    bool has_r_center() const { return r_center.valid; }

    void update_summary() {
        target_count = 0;
        lit_count = 0;
        lit_mask = 0;
        for (int i = 0; i < NUM_SLOTS; ++i) {
            if (!targets[i].valid) continue;
            target_count++;
            if (targets[i].is_lit) {
                lit_count++;
                lit_mask |= static_cast<uint8_t>(1u << i);
            }
        }

        if (!r_center.valid && target_count == 0) {
            status = DetectionStatus::NONE;
        } else if (r_center.valid && target_count == 0) {
            status = DetectionStatus::R_ONLY;
        } else if (!r_center.valid && target_count > 0) {
            status = DetectionStatus::TARGETS_ONLY;
        } else if (r_center.valid && target_count > 0) {
            status = DetectionStatus::COMPLETE;
        } else {
            status = DetectionStatus::PARTIAL;
        }
    }

    std::vector<int> get_lit_slots() const {
        std::vector<int> ids;
        for (int i = 0; i < NUM_SLOTS; ++i) {
            if ((lit_mask & (1u << i)) != 0) ids.push_back(i);
        }
        return ids;
    }

    const DetectedTarget* get_slot(int slot_id) const {
        if (!is_valid_slot(slot_id)) return nullptr;
        if (!targets[slot_id].valid) return nullptr;
        return &targets[slot_id];
    }
};

}  // namespace autobuff

#endif  // AIMER_AUTOBUFF_COMMON_TYPES_HPP
