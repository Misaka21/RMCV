/**
 * @file types.hpp
 * @brief 预测器模块的类型定义
 *
 * 数据流:
 *   检测器 → ArmorObservation → EnemyState(筛选) → EKF → VehicleState → BattlefieldSnapshot → 火控
 *
 * 火控插值:
 *   double dt = now - snapshot.timestamp;
 *   pos' = pos + vel * dt;
 *   phase' = phase + omega * dt;
 */

#ifndef __AIMER_AUTO_AIM_PREDICTOR_TYPES_HPP__
#define __AIMER_AUTO_AIM_PREDICTOR_TYPES_HPP__

#include <array>
#include <cstdint>
#include <functional>
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
 * @brief 单个装甲板滤波状态 (EKF 输出)
 *
 * 作为 VehicleState 的组成部分
 */
struct ArmorState {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();  // 用于火控插值

    double yaw = 0;            // 装甲板朝向 (rad)
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
 * @brief 单车整体状态 (火控需要的完整信息)
 */
struct VehicleState {
    // ==================== 基础信息 ====================

    int target_id = -1;
    EnemyType enemy_type = EnemyType::UNKNOWN;
    bool valid = false;

    // ==================== 运动状态 ====================

    // 旋转中心
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();

    // 陀螺状态
    SpinState spin;

    // 装甲板 (最多4块)
    std::array<ArmorState, MAX_ARMORS_PER_TARGET> armors;
    int armor_count = 4;

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

    const ArmorState* get_recommended_armor() const {
        if (recommended_armor_idx >= 0 && recommended_armor_idx < armor_count) {
            return &armors[recommended_armor_idx];
        }
        const ArmorState* best = nullptr;
        double best_score = -1;
        for (int i = 0; i < armor_count; ++i) {
            if (armors[i].score > best_score) {
                best_score = armors[i].score;
                best = &armors[i];
            }
        }
        return best;
    }

    Eigen::Vector3d predict_center(double dt) const {
        return center + velocity * dt;
    }

    Eigen::Vector3d predict_armor_position(int armor_idx, double dt) const {
        if (armor_idx < 0 || armor_idx >= armor_count) {
            return Eigen::Vector3d::Zero();
        }

        if (!spin.active || std::abs(spin.omega) < 0.1) {
            return armors[armor_idx].predict_position(dt);
        }

        // 陀螺：用存储位置 + 绕 z 轴旋转
        // 注意: armors[] 是按相对顺序填充的 (armors[0] = tracked)
        // 不能用 armor_idx % 2 判断物理属性，必须直接使用存储的位置
        Eigen::Vector3d new_center = predict_center(dt);

        // 从存储的 armors[i].position 获取相对中心的偏移 (motion 模型已正确计算)
        Eigen::Vector3d offset = armors[armor_idx].position - center;

        // 绕 z 轴旋转 omega * dt
        double rot = spin.omega * dt;
        double cos_rot = std::cos(rot);
        double sin_rot = std::sin(rot);

        return Eigen::Vector3d(
            new_center.x() + offset.x() * cos_rot - offset.y() * sin_rot,
            new_center.y() + offset.x() * sin_rot + offset.y() * cos_rot,
            new_center.z() + offset.z()  // z 不随旋转变化
        );
    }
};

/**
 * @brief 战场快照 (Predictor → 火控)
 */
struct BattlefieldSnapshot {
    std::array<VehicleState, MAX_TARGETS> vehicles;

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

    const VehicleState* get_primary() const {
        if (primary_target_id >= 0 && is_valid(primary_target_id)) {
            return &vehicles[primary_target_id];
        }
        return nullptr;
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
        for (int i = 0; i < MAX_TARGETS; ++i) {
            if (is_valid(i)) func(i, vehicles[i]);
        }
    }

    void clear() {
        valid_mask = 0;
        detected_mask = 0;
        primary_target_id = -1;
        for (auto& v : vehicles) v.valid = false;
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
