/**
 * @file base_model.hpp
 * @brief 基地运动模型 (固定建筑)
 *
 * 特点:
 * - 位置固定，不移动
 * - 简单位置滤波即可
 */

#ifndef __AIMER_AUTO_AIM_PREDICTOR_ENEMY_MODEL_BASE_MODEL_HPP__
#define __AIMER_AUTO_AIM_PREDICTOR_ENEMY_MODEL_BASE_MODEL_HPP__

#include "enemy_model.hpp"

namespace autoaim::predictor {

/**
 * @brief 基地运动模型
 */
class BaseModel : public EnemyModelInterface {
public:
    BaseModel(int target_id, EnemyType enemy_type);

    void update(const std::vector<ArmorObservation>& observations, double timestamp) override;
    VehicleState predict(double timestamp) const override;
    bool alive() const override;
    void reset() override;
    int target_id() const override { return target_id_; }
    const char* type_name() const override { return "Base"; }

private:
    int target_id_;
    EnemyType enemy_type_;
    bool initialized_ = false;
    double last_update_time_ = 0;

    // 基地位置 (固定)
    Eigen::Vector3d position_ = Eigen::Vector3d::Zero();

    // 简单低通滤波系数
    static constexpr double FILTER_ALPHA = 0.3;

    static constexpr double LOST_TIMEOUT = 1.0;  // 基地可以更长
};

}  // namespace autoaim::predictor

#endif  // __AIMER_AUTO_AIM_PREDICTOR_ENEMY_MODEL_BASE_MODEL_HPP__
