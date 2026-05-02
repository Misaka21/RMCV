/**
 * @file types.hpp
 * @brief 预测器模块的类型定义
 *
 * 数据流:
 *   检测器 → ArmorObservation → EnemyState(筛选) → EKF → TargetState → BattlefieldSnapshot → 火控
 *
 * 火控插值:
 *   double dt = now - snapshot.timestamp;
 *   pos' = pos + vel * dt;
 *   phase' = phase + omega * dt;
 */

#ifndef __AIMER_AUTO_AIM_PREDICTOR_TYPES_HPP__
#define __AIMER_AUTO_AIM_PREDICTOR_TYPES_HPP__

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include "aimer/auto_aim/common/types.hpp"  // ArmorType, ArmorNumber, DetectedArmor
#include "aimer/common/math/math.hpp"
#include "aimer/common/robot_state.hpp"

namespace autoaim::predictor {

// ==================== 从 common 导入的类型 ====================

using EnemyType = ArmorNumber;  // 目标类型别名

// ==================== 常量定义 ====================

constexpr int MAX_TARGETS = 9;             // 数组大小 (索引 0-8, 目标编号 1-8)
constexpr int MAX_ARMORS_PER_TARGET = 4;   // 每车最多装甲板数

// 装甲板物理尺寸工具函数 (尺寸定义在 common/types.hpp)
namespace armor_size {
    inline constexpr double width(ArmorType type) {
        return (type == ArmorType::LARGE) ? LARGE_ARMOR_WIDTH : SMALL_ARMOR_WIDTH;
    }

    inline constexpr double height(ArmorType type) {
        return (type == ArmorType::LARGE) ? LARGE_ARMOR_HEIGHT : SMALL_ARMOR_HEIGHT;
    }
}

// ==================== 陀螺等级 ====================

enum class SpinLevel {
    NONE = 0,   // 静止: |ω| < 1 rad/s
    LOW = 1,    // 低速陀螺: 1 < |ω| < 3 rad/s
    HIGH = 2    // 高速陀螺: |ω| > 3 rad/s
};

// ==================== 观测量索引 (ArmorObservation.z) ====================

namespace obs {
    enum Idx {
        YAW = 0,       // 装甲板方位角 (rad)
        PITCH = 1,     // 装甲板俯仰角 (rad)
        DIST = 2,      // 装甲板距离 (m)
        ARMOR_YAW = 3  // 装甲板朝向 (rad)
    };
}

// ============================================================================
// 输入数据结构
// ============================================================================

/**
 * @brief 单次装甲板观测 (检测器+PnP → 预测器)
 *
 * 由 DetectedArmor + PnP 解算得到，作为 EnemyState 的输入
 */
struct ArmorObservation {
    // 观测向量 [yaw, pitch, dist, armor_yaw]
    Eigen::Vector4d z = Eigen::Vector4d::Zero();

    // 3D 位置 (世界坐标系)
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();

    // 目标信息
    int target_id = 0;      // 目标编号 (机器人编号 1-8)
    int armor_num = 4;      // 该目标装甲板总数 (3 或 4)
    // 注意: 装甲板 ID (0-3) 由 ArmorIdentifier 分配，存储在 ArmorData 中
    ArmorType type = ArmorType::SMALL;
    EnemyColor color = EnemyColor::GRAY;  // 敌方颜色

    // 检测器置信度
    float confidence = 0;   // 来自检测器的分类置信度

    // 时间和角度
    double timestamp = 0;   // 时间戳 (s)
    Eigen::Quaterniond q_imu = Eigen::Quaterniond::Identity();  // 当前帧 IMU 姿态
    double z_to_v = 0;      // 装甲板法向与视线夹角 (三分法优化后)
    double z_to_v_raw = 0;  // 装甲板法向与视线夹角 (三分法优化前)
    double orientation_pitch = 0;  // 装甲板法向在世界系下的 pitch

    // 图像信息 (可选，调试用)
    std::vector<cv::Point2f> pts;  // 四角点 (原始像素坐标)
    std::array<cv::Point2f, 4> pus;  // 畸变矫正后的四角点，用于三分法重投影比较
    cv::Point2f center_2d;         // 图像中心

    bool valid = false;

    // ==================== 辅助方法 ====================

    double distance() const { return pos.norm(); }

    /**
     * @brief 从 DetectedArmor + PnP 结果构造
     */
    static ArmorObservation from_detection(
        const DetectedArmor& det,
        const Eigen::Vector3d& position,
        const Eigen::Vector4d& obs_z,
        double z_to_v_value,
        double z_to_v_raw_value,
        double timestamp,
        const std::array<cv::Point2f, 4>& pus_in = {}
    ) {
        ArmorObservation obs;
        obs.z = obs_z;
        obs.pos = position;
        obs.target_id = static_cast<int>(det.number);
        obs.type = det.type;
        obs.color = det.color;
        obs.z_to_v = z_to_v_value;
        obs.z_to_v_raw = z_to_v_raw_value;
        obs.timestamp = timestamp;
        obs.pts = det.landmarks;
        obs.pus = pus_in;
        obs.confidence = det.confidence;
        obs.center_2d = det.center;
        obs.valid = true;

        // 根据目标类型设置装甲板数
        obs.armor_num = (det.number == ArmorNumber::OUTPOST) ? 3 : 4;

        return obs;
    }
};

// ============================================================================
// 输出数据结构 (EKF 滤波后)
// ============================================================================

/**
 * @brief 单个装甲板滤波状态 (ArmorMotion 内部输出)
 *
 * 仅作为单装甲板滤波器的中间结果；对火控输出使用 TargetState。
 */
struct ArmorState {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();  // 用于火控插值

    double yaw = 0;            // 装甲板朝向 (INWARD, 从装甲板指向车心/旋转中心)
    double z_to_v = 0;         // 相对相机的夹角，越小越正对

    int id = 0;                // 装甲板编号 0-3
    ArmorType type = ArmorType::SMALL;

    double score = 0;          // 打击评分 (0~1)
    bool visible = false;      // 当前帧是否可见
    double last_seen = 0;      // 上次看到的时间

    // 装甲板物理尺寸
    double width() const { return armor_size::width(type); }
    double height() const { return armor_size::height(type); }

    // 插值预测
    Eigen::Vector3d predict_position(double dt) const {
        return position + velocity * dt;
    }
};

/**
 * @brief 陀螺运动状态
 */
struct SpinState {
    bool active = false;
    SpinLevel level = SpinLevel::NONE;

    double omega = 0;          // 角速度 (rad/s)，正值为逆时针
    double phase = 0;          // 当前相位 (rad)，即车体朝向角 θ
    double radius = 0;         // 陀螺半径 (m)
    double radius_2 = 0;       // 第二半径 (四装甲板时，奇数装甲板用)
    double dz = 0;             // 高度差 (m)，奇数装甲板 z = zc + dz

    // 插值预测
    double predict_phase(double dt) const {
        return phase + omega * dt;
    }

    /**
     * @brief 陀螺等级消抖阈值 (°/s)
     */
    struct SpinThresholds {
        double top1_activate = 60.0;
        double top1_deactivate = 40.0;
        double top2_activate = 200.0;
        double top2_deactivate = 150.0;
    };

    /**
     * @brief 更新陀螺等级 (带迟滞消抖)
     *
     * @param new_omega 新的角速度 (rad/s)
     * @param thresholds 阈值 (°/s)，由调用者从运行时参数读取
     */
    void update_level(double new_omega, const SpinThresholds& thresholds) {
        omega = new_omega;
        double w_deg = std::abs(omega) * 180.0 / M_PI;  // 转换为度/秒

        switch (level) {
            case SpinLevel::NONE:
                if (w_deg > thresholds.top1_activate) {
                    level = SpinLevel::LOW;
                    active = true;
                }
                break;

            case SpinLevel::LOW:
                if (w_deg < thresholds.top1_deactivate) {
                    level = SpinLevel::NONE;
                    active = false;
                } else if (w_deg > thresholds.top2_activate) {
                    level = SpinLevel::HIGH;
                }
                break;

            case SpinLevel::HIGH:
                if (w_deg < thresholds.top2_deactivate) {
                    level = SpinLevel::LOW;
                }
                break;
        }
    }

    /**
     * @brief 重置状态
     */
    void reset() {
        active = false;
        level = SpinLevel::NONE;
        omega = 0;
        phase = 0;
        radius = 0;
        radius_2 = 0;
        dz = 0;
    }
};

/**
 * @brief 单个目标的紧凑火控状态 (RV 风格目标包 + 最小火控元数据)
 *
 * yaw 统一为 INWARD yaw:
 * - yaw 表示 slot 0 装甲板“从装甲板指向车心/旋转中心”的方向。
 * - 预测装甲板位置使用 armor = center - r * dir(inward_yaw)。
 * - z_to_v 使用 inward_yaw - view_yaw，不再做 OUTWARD 的 π 修正。
 *
 * armors[] 不再存完整 ArmorState；火控通过 helper 按预测时刻计算装甲板位置、
 * 速度、朝向和可见性。这样 BattlefieldSnapshot 只携带有效目标 vector，
 * 避免每帧固定携带 9 辆车 × 4 块板的大结构。
 */
struct TargetState {
    // ==================== 基础信息 ====================

    int target_id = -1;
    EnemyType enemy_type = EnemyType::UNKNOWN;
    bool valid = false;
    bool tracking = false;

    // ==================== RV 风格核心状态 ====================

    Eigen::Vector3d position = Eigen::Vector3d::Zero();  // 目标中心
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();

    double yaw = 0;       // slot 0 装甲板 INWARD yaw
    double v_yaw = 0;     // 角速度 (rad/s)
    double radius_1 = 0;  // 主半径
    double radius_2 = 0;  // 次半径
    double dz = 0;        // 高度差
    int armor_count = 4;  // RV armors_num

    // 陀螺状态
    SpinState spin;

    // ==================== 最小火控元数据 ====================

    uint8_t visible_mask = 0;
    uint8_t detected_mask = 0;

    std::array<int, MAX_ARMORS_PER_TARGET> armor_ids = {0, 1, 2, 3};
    std::array<ArmorType, MAX_ARMORS_PER_TARGET> armor_types = {
        ArmorType::SMALL, ArmorType::SMALL, ArmorType::SMALL, ArmorType::SMALL
    };
    std::array<double, MAX_ARMORS_PER_TARGET> armor_radii = {0.0, 0.0, 0.0, 0.0};
    std::array<double, MAX_ARMORS_PER_TARGET> armor_z_offsets = {0.0, 0.0, 0.0, 0.0};
    std::array<Eigen::Vector3d, MAX_ARMORS_PER_TARGET> armor_position_offsets = {
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()
    };
    std::array<Eigen::Vector3d, MAX_ARMORS_PER_TARGET> armor_velocity_offsets = {
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()
    };
    std::array<double, MAX_ARMORS_PER_TARGET> armor_z_to_v = {0.0, 0.0, 0.0, 0.0};
    std::array<double, MAX_ARMORS_PER_TARGET> armor_last_seen = {0.0, 0.0, 0.0, 0.0};
    std::array<double, MAX_ARMORS_PER_TARGET> armor_scores = {0.0, 0.0, 0.0, 0.0};

    // ==================== 置信度 ====================

    double confidence = 0;     // 整体置信度 (0~1)
    double position_std = 0;   // 位置标准差 (m)
    double velocity_std = 0;   // 速度标准差 (m/s)

    // ==================== 推荐目标 ====================

    int recommended_armor_idx = -1;

    // ==================== 时间戳 ====================

    double timestamp = 0;
    int frame_count = 0;

    // ==================== 辅助方法 ====================

    bool armor_index_valid(int armor_idx) const {
        return armor_idx >= 0 && armor_idx < armor_count
            && armor_idx < MAX_ARMORS_PER_TARGET;
    }

    double armor_step() const {
        return armor_count > 0 ? (2.0 * M_PI / armor_count) : 0.0;
    }

    int armor_id(int armor_idx) const {
        return armor_index_valid(armor_idx) ? armor_ids[armor_idx] : -1;
    }

    ArmorType armor_type(int armor_idx) const {
        return armor_index_valid(armor_idx) ? armor_types[armor_idx] : ArmorType::SMALL;
    }

    double armor_width(int armor_idx) const {
        return armor_size::width(armor_type(armor_idx));
    }

    double armor_height(int armor_idx) const {
        return armor_size::height(armor_type(armor_idx));
    }

    bool armor_visible(int armor_idx) const {
        return armor_index_valid(armor_idx)
            && (visible_mask & (1u << armor_idx)) != 0;
    }

    bool armor_detected(int armor_idx) const {
        return armor_index_valid(armor_idx)
            && (detected_mask & (1u << armor_idx)) != 0;
    }

    void set_armor_visible(int armor_idx, bool visible) {
        if (!armor_index_valid(armor_idx)) return;
        if (visible) {
            visible_mask |= (1u << armor_idx);
        } else {
            visible_mask &= ~(1u << armor_idx);
        }
    }

    void set_armor_detected(int armor_idx, bool detected) {
        if (!armor_index_valid(armor_idx)) return;
        if (detected) {
            detected_mask |= (1u << armor_idx);
        } else {
            detected_mask &= ~(1u << armor_idx);
        }
    }

    double armor_score(int armor_idx) const {
        return armor_index_valid(armor_idx) ? armor_scores[armor_idx] : 0.0;
    }

    double armor_last_seen_time(int armor_idx) const {
        return armor_index_valid(armor_idx) ? armor_last_seen[armor_idx] : 0.0;
    }

    int best_armor_idx() const {
        if (recommended_armor_idx >= 0 && recommended_armor_idx < armor_count) {
            return recommended_armor_idx;
        }
        int best_idx = -1;
        double best_score = -1;
        for (int i = 0; i < armor_count; ++i) {
            if (armor_scores[i] > best_score) {
                best_score = armor_scores[i];
                best_idx = i;
            }
        }
        return best_idx;
    }

    Eigen::Vector3d predict_center(double dt) const {
        return position + velocity * dt;
    }

    double armor_yaw(int armor_idx, double dt) const {
        if (!armor_index_valid(armor_idx)) {
            return yaw + v_yaw * dt;
        }
        return aimer::math::reduced_angle(yaw + v_yaw * dt + armor_idx * armor_step());
    }

    Eigen::Vector3d predict_armor_position(int armor_idx, double dt) const {
        if (!armor_index_valid(armor_idx)) {
            return Eigen::Vector3d::Zero();
        }

        if (!spin.active || std::abs(v_yaw) < 0.1) {
            return position + armor_position_offsets[armor_idx]
                + (velocity + armor_velocity_offsets[armor_idx]) * dt;
        }

        const Eigen::Vector3d center = predict_center(dt);
        const double r = armor_radii[armor_idx];
        const double armor_inward_yaw = armor_yaw(armor_idx, dt);

        return Eigen::Vector3d(
            center.x() - r * std::cos(armor_inward_yaw),
            center.y() - r * std::sin(armor_inward_yaw),
            center.z() + armor_z_offsets[armor_idx]
        );
    }

    Eigen::Vector3d predict_armor_velocity(int armor_idx, double dt) const {
        if (!armor_index_valid(armor_idx)) {
            return Eigen::Vector3d::Zero();
        }
        if (!spin.active || std::abs(v_yaw) < 0.1) {
            return velocity + armor_velocity_offsets[armor_idx];
        }

        const Eigen::Vector3d center = predict_center(dt);
        const Eigen::Vector3d armor_pos = predict_armor_position(armor_idx, dt);
        const Eigen::Vector3d offset = armor_pos - center;
        return velocity + Eigen::Vector3d(
            -v_yaw * offset.y(),
            +v_yaw * offset.x(),
            0.0
        );
    }

    double predicted_z_to_v(int armor_idx, double dt) const {
        if (!armor_index_valid(armor_idx)) {
            return 0.0;
        }
        if (!spin.active || std::abs(v_yaw) < 0.1) {
            return armor_z_to_v[armor_idx];
        }
        const Eigen::Vector3d pos = predict_armor_position(armor_idx, dt);
        const double view_yaw = std::atan2(pos.y(), pos.x());
        return aimer::math::reduced_angle(armor_yaw(armor_idx, dt) - view_yaw);
    }
};

/**
 * @brief 战场快照 (Predictor → 火控)
 */
struct BattlefieldSnapshot {
    std::vector<TargetState> targets;

    uint16_t valid_mask = 0;       // 哪些目标有效
    uint16_t detected_mask = 0;   // 当前帧检测到哪些

    int primary_target_id = -1;   // 主目标编号

    // 检测时刻的自身状态 (火控用)
    aimer::RobotState self_state;

    // 时间戳
    double timestamp = 0;          // 图像时间戳 (img_t)
    double predict_timestamp = 0;  // 预测完成时间戳 (predict_t)
    int frame_id = 0;

    // ==================== 辅助方法 ====================

    bool is_valid(int id) const {
        return id >= 0 && id < MAX_TARGETS && (valid_mask & (1 << id)) != 0;
    }

    bool is_detected(int id) const {
        return id >= 0 && id < MAX_TARGETS && (detected_mask & (1 << id)) != 0;
    }

    const TargetState* get_primary() const {
        return find_target(primary_target_id);
    }

    const TargetState* find_target(int id) const {
        if (id < 0 || !is_valid(id)) {
            return nullptr;
        }
        auto it = std::find_if(targets.begin(), targets.end(),
            [id](const TargetState& target) {
                return target.target_id == id && target.valid;
            });
        return it != targets.end() ? &(*it) : nullptr;
    }

    TargetState* find_target(int id) {
        if (id < 0 || !is_valid(id)) {
            return nullptr;
        }
        auto it = std::find_if(targets.begin(), targets.end(),
            [id](const TargetState& target) {
                return target.target_id == id && target.valid;
            });
        return it != targets.end() ? &(*it) : nullptr;
    }

    void add_target(const TargetState& target) {
        if (!target.valid || target.target_id < 0 || target.target_id >= MAX_TARGETS) {
            return;
        }
        targets.push_back(target);
        set_valid(target.target_id, true);
    }

    void set_valid(int id, bool valid) {
        if (id >= 0 && id < MAX_TARGETS) {
            if (valid) valid_mask |= (1 << id);
            else valid_mask &= ~(1 << id);
        }
    }

    void set_detected(int id, bool detected) {
        if (id >= 0 && id < MAX_TARGETS) {
            if (detected) detected_mask |= (1 << id);
            else detected_mask &= ~(1 << id);
        }
    }

    template<typename Func>
    void for_each_valid(Func&& func) const {
        for (const auto& target : targets) {
            if (target.valid && is_valid(target.target_id)) {
                func(target.target_id, target);
            }
        }
    }

    void clear() {
        targets.clear();
        valid_mask = 0;
        detected_mask = 0;
        primary_target_id = -1;
    }
};

/**
 * @brief Predictor 可视化帧 (Predictor → Visualizer)
 *
 * 与 BattlefieldSnapshot 解耦，避免控制链路携带调试图像。
 */
struct PredictorDebugFrame {
    cv::Mat image;                 // 可视化原图/叠加图
    Eigen::Quaterniond q_imu = Eigen::Quaterniond::Identity();
    int frame_id = -1;
    double timestamp = 0;
    float detect_latency_ms = 0;
    float predict_latency_ms = 0;
};

// ==================== 辅助函数 ====================

inline Eigen::Vector3d ypd_to_xyz(double yaw, double pitch, double dist) {
    return aimer::math::ypd_to_xyz(aimer::math::YpdCoord{yaw, pitch, dist});
}

inline Eigen::Vector3d xyz_to_ypd(const Eigen::Vector3d& xyz) {
    return aimer::math::xyz_to_ypd(xyz).to_vec();
}

// normalize_angle 已移到 math/math.hpp

}  // namespace autoaim::predictor

#endif  // __AIMER_AUTO_AIM_PREDICTOR_TYPES_HPP__
