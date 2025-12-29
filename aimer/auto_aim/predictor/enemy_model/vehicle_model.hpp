/**
 * @file vehicle_model.hpp
 * @brief 车辆运动模型 (4块装甲板)
 *
 * 适用: 英雄、工程、步兵、哨兵
 *
 * 职责:
 * - 观测消抖
 * - 装甲板 ID 分配 (ArmorIdentifier)
 * - 匀速 EKF 滤波 (ArmorMotion)
 * - (TODO) 整车 EKF 滤波 (VehicleEkf)
 */

#ifndef __AIMER_AUTO_AIM_PREDICTOR_ENEMY_MODEL_VEHICLE_MODEL_HPP__
#define __AIMER_AUTO_AIM_PREDICTOR_ENEMY_MODEL_VEHICLE_MODEL_HPP__

#include "aimer/auto_aim/predictor/enemy_state/armor_identifier.hpp"
#include "enemy_model.hpp"
#include "motion/armor_motion.hpp"
#include "motion/spin_motion.hpp"
#include "motion/lmtd_motion.hpp"

namespace autoaim::predictor {

/**
 * @brief 车辆运动模型
 *
 * 数据流:
 *   observations → filter() → ArmorIdentifier → ArmorMotion → VehicleState
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

    /**
     * @brief 绘制调试信息
     *
     * 绘制内容:
     * - 检测到的装甲板四角点 (绿色)
     * - ArmorMotion 滤波位置 (蓝色圆圈)
     * - SpinMotion 预测的所有装甲板 (黄色, 陀螺模式)
     * - 旋转中心 (红色十字, 陀螺模式)
     */
    void draw(cv::Mat& img, const Eigen::Quaterniond& q_imu, double timestamp) const override;

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
    ArmorMotion armor_motion_;

    // 整车旋转模型 (职责: 陀螺 EKF 滤波)
    SpinMotion spin_motion_;

    // LMTD 整车旋转模型 (替代 SpinMotion, 内部处理跳变)
    LmtdMotion lmtd_motion_;
    bool use_lmtd_ = true;  // 是否使用 LMTD 模型

    // 上一帧观测 (用于消抖)
    std::vector<ArmorObservation> prev_armors_;
    double prev_timestamp_ = 0;

    // 陀螺状态
    SpinState spin_;

    // 跳变检测 (上层负责检测，通知 SpinMotion)
    int last_tracking_id_ = -1;

    // 敌方颜色 (用于绘图)
    EnemyColor enemy_color_ = EnemyColor::GRAY;
};

}  // namespace autoaim::predictor

#endif  // __AIMER_AUTO_AIM_PREDICTOR_ENEMY_MODEL_VEHICLE_MODEL_HPP__
