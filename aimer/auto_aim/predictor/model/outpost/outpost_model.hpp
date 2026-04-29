/**
 * @file outpost_model.hpp
 * @brief 前哨站运动模型 (EnemyModelInterface 实现)
 *
 * 特点:
 * - 使用 EKF 滤波 (OutpostMotion)
 * - 支持盲区预测 (装甲板不可见时仍可预测)
 * - 3块装甲板，120°间隔
 * - 固定转速 |ω| = 0.8π rad/s
 */

#ifndef __AIMER_AUTO_AIM_PREDICTOR_ENEMY_MODEL_OUTPOST_MODEL_HPP__
#define __AIMER_AUTO_AIM_PREDICTOR_ENEMY_MODEL_OUTPOST_MODEL_HPP__

#include "../enemy_model.hpp"
#include "outpost_ekf.hpp"
#include "aimer/auto_aim/predictor/observer/armor_tracker.hpp"

namespace autoaim::predictor {

/**
 * @brief 前哨站运动模型
 *
 * 使用 OutpostMotion (EKF) 实现盲区预测:
 * - 即使没有观测，也能通过固定 ω 外推相位
 * - 学习每个槽位的高度差
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
    void draw(cv::Mat& img, const Eigen::Quaterniond& q_imu, double timestamp) const override;

private:
    std::vector<ArmorObservation> filter_observations(
        const std::vector<ArmorObservation>& observations,
        double timestamp
    );

    int target_id_;
    EnemyType enemy_type_;
    bool initialized_ = false;
    double last_update_time_ = 0;
    double last_active_filter_z_ = 1e9;
    double last_active_filter_time_ = -1e9;

    // EKF 运动模型
    OutpostMotion motion_;

    // 装甲板 ID 分配器
    ArmorIdentifier identifier_;

    // 参数
    static constexpr int ARMOR_NUM = 3;
    static constexpr double LOST_TIMEOUT = 1.0;  // 盲区容忍时间更长

    // 帧计数
    int frame_count_ = 0;
};

}  // namespace autoaim::predictor

#endif  // __AIMER_AUTO_AIM_PREDICTOR_ENEMY_MODEL_OUTPOST_MODEL_HPP__
