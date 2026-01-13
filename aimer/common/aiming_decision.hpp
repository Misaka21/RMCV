//
// 瞄准决策结果 - 目标决策模块的抽象输出
//

#ifndef AIMER_COMMON_AIMING_DECISION_HPP
#define AIMER_COMMON_AIMING_DECISION_HPP

#include <cstdint>

#include <Eigen/Core>

namespace aimer {

// ============================================================================
// 目标类型 (抽象)
// ============================================================================

enum class TargetType : uint8_t {
    UNKNOWN = 0,
    ARMOR = 1,           // 装甲板 (自瞄)
    ENERGY_BLADE = 2,    // 能量机关扇叶
};

inline const char* target_type_name(TargetType type) {
    switch (type) {
        case TargetType::ARMOR: return "ARMOR";
        case TargetType::ENERGY_BLADE: return "ENERGY_BLADE";
        default: return "UNKNOWN";
    }
}

// ============================================================================
// AimingDecision - 目标决策输出
// ============================================================================

/**
 * @brief 瞄准决策结果 - 目标决策模块的输出
 *
 * 这是抽象类型，不依赖 predictor 的具体实现 (BattlefieldSnapshot 等)
 * 顶层火控只看到这个类型，不知道背后的数据来源
 *
 * 数据流:
 *   AutoAimDecider 内部访问 BattlefieldSnapshot
 *   → 目标选择、弹道解算、开火判断
 *   → 输出 AimingDecision
 */
struct AimingDecision {
    // ==================== 有效性 ====================
    bool valid = false;

    // ==================== 目标信息 (抽象) ====================
    int target_id = -1;                      // 目标 ID
    TargetType target_type = TargetType::UNKNOWN;

    // ==================== 瞄准点 ====================
    Eigen::Vector3d aim_point = Eigen::Vector3d::Zero();     // 预测后的瞄准位置 (世界坐标)
    Eigen::Vector3d aim_velocity = Eigen::Vector3d::Zero();  // 瞄准点速度 (用于速度前馈)

    // ==================== 弹道解算结果 ====================
    double aim_yaw = 0;         // 解算后的 yaw (rad)
    double aim_pitch = 0;       // 解算后的 pitch (rad, 含重力补偿)
    double aim_distance = 0;    // 瞄准距离 (m)
    double fly_time = 0;        // 子弹飞行时间 (s)

    // ==================== 云台规划 (MPC/PID 输出) ====================
    double yaw = 0;             // 规划 yaw (rad)
    double pitch = 0;           // 规划 pitch (rad)
    double yaw_vel = 0;         // yaw 速度前馈 (rad/s)
    double pitch_vel = 0;       // pitch 速度前馈 (rad/s)
    double yaw_acc = 0;         // yaw 加速度前馈 (rad/s², MPC only)
    double pitch_acc = 0;       // pitch 加速度前馈 (rad/s², MPC only)

    // ==================== 开火决策 ====================
    bool allow_fire = false;    // 视野内允许射击
    bool fire_now = false;      // 当前时刻可以开火 (误差足够小)

    // ==================== 置信度和跟踪质量 ====================
    double confidence = 0;      // 目标置信度 (0~1)
    double tracking_error = 0;  // 跟踪误差 (rad)

    // ==================== 时间戳 ====================
    double timestamp = 0;       // 决策时刻 (秒, steady_clock)
    int frame_id = 0;           // 帧 ID

    // ==================== 辅助方法 ====================

    AimingDecision() = default;

    // 创建无目标的决策
    static AimingDecision no_target() {
        AimingDecision d;
        d.valid = false;
        return d;
    }
};

}  // namespace aimer

#endif  // AIMER_COMMON_AIMING_DECISION_HPP
