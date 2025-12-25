/**
 * @file latency_estimator.hpp
 * @brief 延迟估计器
 *
 * 时间线:
 *   img → predict → send → control → fire → hit
 *    │       │        │        │        │      │
 *    │←─────→│←──────→│←──────→│←──────→│←────→│
 *    │ 计算  │  滤波  │  静态  │  静态  │ 计算 │
 *
 * 用法:
 *   double img_to_predict = now - snapshot.timestamp;
 *   double predict_to_send = estimator.update_predict_to_send(latency, now);
 */

#ifndef __AIMER_AUTO_AIM_FIRE_CONTROL_COMMON_LATENCY_ESTIMATOR_HPP__
#define __AIMER_AUTO_AIM_FIRE_CONTROL_COMMON_LATENCY_ESTIMATOR_HPP__

#include <algorithm>

#include "aimer/common/filter/single_filter.hpp"

namespace autoaim::fire_control {

/**
 * @brief 延迟估计器
 *
 * 只负责滤波，时间戳从 snapshot 读取
 */
class LatencyEstimator {
public:
    LatencyEstimator() = default;

    /**
     * @brief 更新 predict_to_send 延迟估计
     *
     * @param latency 本次观测的延迟 (s)
     * @param t 当前时间 (s)
     * @return 滤波后的延迟
     */
    double update_predict_to_send(double latency, double t) {
        // 限制在合理范围
        latency = std::clamp(latency, 0.0001, 0.1);
        // Q = 1.0 (过程噪声), R = 3000.0 (观测噪声，越大越平滑)
        predict_to_send_filter_.update(latency, t, {1.0}, {3000.0});
        return get_predict_to_send();
    }

    /**
     * @brief 获取滤波后的 predict_to_send 延迟
     */
    double get_predict_to_send() const {
        auto x = predict_to_send_filter_.get_x();
        double value = x[0];
        return (value > 0) ? value : default_predict_to_send_;
    }

    /**
     * @brief 计算 img → prediction 延迟 (用于位置预测)
     *
     * = img_to_predict + predict_to_send + send_to_control + fire_to_hit
     * (忽略 control_to_fire，参考 rm.cv.fans 设计)
     */
    double get_prediction_latency(
        double img_to_predict,
        double send_to_control,
        double distance,
        double bullet_speed
    ) const {
        double fire_to_hit = distance / std::max(bullet_speed, 10.0);
        return img_to_predict + get_predict_to_send() + send_to_control + fire_to_hit;
    }

    /**
     * @brief 计算 img → hit 延迟 (用于反陀螺打击时机)
     *
     * = img_to_predict + predict_to_send + send_to_control + control_to_fire + fire_to_hit
     */
    double get_hit_latency(
        double img_to_predict,
        double send_to_control,
        double control_to_fire,
        double distance,
        double bullet_speed
    ) const {
        double fire_to_hit = distance / std::max(bullet_speed, 10.0);
        return img_to_predict + get_predict_to_send() + send_to_control
             + control_to_fire + fire_to_hit;
    }

    void set_default_predict_to_send(double value) {
        default_predict_to_send_ = value;
    }

private:
    mutable filter::SingleFilter<1> predict_to_send_filter_;
    double default_predict_to_send_ = 0.002;  // 2ms
};

}  // namespace autoaim::fire_control

#endif  // __AIMER_AUTO_AIM_FIRE_CONTROL_COMMON_LATENCY_ESTIMATOR_HPP__
