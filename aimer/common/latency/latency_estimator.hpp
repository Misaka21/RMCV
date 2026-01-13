/**
 * @file latency_estimator.hpp
 * @brief 通用延迟估计器
 *
 * 时间线:
 *   img → predict → send → control → fire → hit
 *    │       │        │        │        │      │
 *    │←─────→│←──────→│←──────→│←──────→│←────→│
 *    │ 计算  │  滤波  │  静态  │  静态  │ 计算 │
 *
 * 用法:
 *   estimator.update_predict_to_send(current_time - predict_timestamp, current_time);
 *   LatencyInfo latency = estimator.build(img_to_predict, distance, bullet_speed);
 */

#ifndef __AIMER_COMMON_LATENCY_LATENCY_ESTIMATOR_HPP__
#define __AIMER_COMMON_LATENCY_LATENCY_ESTIMATOR_HPP__

#include <algorithm>

#include "aimer/common/filter/single_filter.hpp"
#include "aimer/common/fire_control_types.hpp"
#include "plugin/param/runtime_parameter.hpp"

namespace aimer {

/**
 * @brief 通用延迟估计器
 *
 * 负责:
 *   1. 滤波 predict_to_send 延迟
 *   2. 构建完整的 LatencyInfo
 *
 * 可用于 autoaim 和 autobuff
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
     * @brief 构建完整的延迟信息
     *
     * @param img_to_predict 图像到预测完成的延迟 (s)
     * @param distance 目标距离 (m)
     * @param bullet_speed 弹速 (m/s)
     * @param param_prefix 参数前缀 (如 "AutoAim.FireControl" 或 "AutoBuff.FireControl")
     * @return LatencyInfo 完整延迟信息
     */
    ::fire_control::LatencyInfo build(
        double img_to_predict,
        double distance,
        double bullet_speed,
        const std::string& param_prefix = "AutoAim.FireControl"
    ) const {
        ::fire_control::LatencyInfo latency;

        // img_to_predict: 直接传入
        latency.img_to_predict = img_to_predict;

        // predict_to_send: 卡尔曼滤波
        latency.predict_to_send = get_predict_to_send();

        // send_to_control: 运行时参数
        latency.send_to_control = runtime_param::get_param<double>(
            param_prefix + ".Latency.send_to_control"
        );

        // control_to_fire: 运行时参数
        latency.control_to_fire = runtime_param::get_param<double>(
            param_prefix + ".Latency.control_to_fire"
        );

        // fire_to_hit: 根据目标距离计算
        latency.bullet_speed = std::max(bullet_speed, 10.0);
        latency.fire_to_hit = distance / latency.bullet_speed;

        return latency;
    }

private:
    mutable aimer::filter::SingleFilter<1> predict_to_send_filter_;
    double default_predict_to_send_ = 0.002;  // 2ms
};

}  // namespace aimer

#endif  // __AIMER_COMMON_LATENCY_LATENCY_ESTIMATOR_HPP__
