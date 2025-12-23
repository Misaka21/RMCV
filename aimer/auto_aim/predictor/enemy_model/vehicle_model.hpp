/**
 * @file vehicle_model.hpp
 * @brief 车辆运动模型 (4块装甲板)
 *
 * 适用: 英雄、工程、步兵、哨兵
 *
 * 职责:
 * - 观测消抖
 * - 装甲板 ID 分配 (ArmorIdentifier)
 * - 匀速 EKF 滤波 (ArmorModel)
 * - (TODO) 整车 EKF 滤波 (VehicleEkf)
 */

#ifndef __AIMER_AUTO_AIM_PREDICTOR_ENEMY_MODEL_VEHICLE_MODEL_HPP__
#define __AIMER_AUTO_AIM_PREDICTOR_ENEMY_MODEL_VEHICLE_MODEL_HPP__

#include "aimer/auto_aim/predictor/enemy_state/armor_identifier.hpp"
#include "enemy_model.hpp"
#include "motion/armor_model.hpp"

namespace autoaim::predictor {

/**
 * @brief 车辆运动模型
 *
 * 数据流:
 *   observations → filter() → ArmorIdentifier → ArmorModel → VehicleState
 */
class VehicleModel : public EnemyModelInterface {
public:
    VehicleModel(int target_id, EnemyType enemy_type);

    void update(const std::vector<ArmorObservation>& observations, double timestamp) override;
    VehicleState predict(double timestamp) const override;
    bool alive() const override;
    void reset() override;
    int target_id() const override { return target_id_; }
    const char* type_name() const override { return "Vehicle"; }

private:
    // ==================== 消抖过滤 ====================

    /**
     * @brief 过滤无效观测
     */
    std::vector<ArmorObservation> filter(
        const std::vector<ArmorObservation>& raw,
        const std::vector<ArmorObservation>& last
    ) const;

    // ==================== 数据 ====================

    int target_id_;
    EnemyType enemy_type_;
    bool initialized_ = false;
    double last_update_time_ = 0;
    int frame_count_ = 0;

    // 装甲板 ID 分配器 (职责: 跨帧匹配)
    ArmorIdentifier identifier_;

    // 装甲板运动模型 (职责: 匀速 EKF 滤波)
    ArmorModel armor_model_;

    // 上一帧观测 (用于消抖)
    std::vector<ArmorObservation> prev_armors_;
    double prev_timestamp_ = 0;

    // 陀螺状态
    SpinState spin_;

    // ==================== 消抖参数 ====================

    static constexpr double EXISTING_ARMOR_AREA_RATIO = 0.30;
    static constexpr double NEW_ARMOR_AREA_RATIO = 0.40;
    static constexpr double EXISTING_ARMOR_DISTANCE = 0.5;  // 位置匹配阈值 (m)
    static constexpr double JUMP_DISTANCE_LIMIT = 1.2;
    static constexpr double MIN_DIST = 0.5;
    static constexpr double MAX_DIST = 15.0;
    static constexpr double NEW_ARMOR_MAX_DIST = 10.0;
    static constexpr double MAX_Z_TO_V = 1.2;
    static constexpr double LOST_TIMEOUT = 0.5;
    static constexpr double ARMOR_CREDIT_TIME = 0.1;
};

}  // namespace autoaim::predictor

#endif  // __AIMER_AUTO_AIM_PREDICTOR_ENEMY_MODEL_VEHICLE_MODEL_HPP__
