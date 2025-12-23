/**
 * @file outpost_model.hpp
 * @brief 前哨站运动模型 (3块小装甲板)
 *
 * 特点:
 * - 固定位置旋转
 * - 3块装甲板均匀分布 (120°间隔)
 * - 转速相对固定
 */

#ifndef __AIMER_AUTO_AIM_PREDICTOR_ENEMY_MODEL_OUTPOST_MODEL_HPP__
#define __AIMER_AUTO_AIM_PREDICTOR_ENEMY_MODEL_OUTPOST_MODEL_HPP__

#include "enemy_model.hpp"

namespace autoaim::predictor {

/**
 * @brief 前哨站运动模型
 */
class OutpostModel : public EnemyModelInterface {
public:
    OutpostModel(int target_id, EnemyType enemy_type);

    void update(const std::vector<ArmorObservation>& observations, double timestamp) override;
    VehicleState predict(double timestamp) const override;
    bool alive() const override;
    void reset() override;
    int target_id() const override { return target_id_; }
    const char* type_name() const override { return "Outpost"; }

private:
    int target_id_;
    EnemyType enemy_type_;
    bool initialized_ = false;
    double last_update_time_ = 0;

    // 前哨站固定参数
    Eigen::Vector3d center_ = Eigen::Vector3d::Zero();
    double radius_ = 0.2765;  // 半径 (m), 553mm / 2
    double omega_ = 0;        // 角速度 (rad/s)
    double phase_ = 0;        // 当前相位 (rad)

    // 装甲板数
    static constexpr int ARMOR_NUM = 3;
    static constexpr double ARMOR_ANGLE_STEP = 2.0 * M_PI / 3.0;  // 120°

    static constexpr double LOST_TIMEOUT = 0.5;
};

}  // namespace autoaim::predictor

#endif  // __AIMER_AUTO_AIM_PREDICTOR_ENEMY_MODEL_OUTPOST_MODEL_HPP__
