/**
 * @file autoaim_decider.cpp
 * @brief 自瞄目标决策器实现
 */

#include "autoaim_decider.hpp"

#include "plugin/debug/logger.hpp"

namespace autoaim::target_decision {

AutoAimDecider::AutoAimDecider() {
    // 从 UMT 获取 Predictor 输出
    battlefield_ = umt::BasicObjManager<predictor::BattlefieldSnapshot>::find_or_create("battlefield");

    debug::print(debug::PrintMode::INFO, "AutoAimDecider", "Initialized");
}

::fire_control::FireCommand AutoAimDecider::decide(double current_time) {
    // 1. 从 UMT 读取 BattlefieldSnapshot (内部细节)
    const auto& snapshot = battlefield_->get();

    // 2. 检测新帧，更新延迟估计
    if (snapshot.frame_id != last_frame_id_ && snapshot.predict_timestamp > 0) {
        last_frame_id_ = snapshot.frame_id;
        double predict_to_send = current_time - snapshot.predict_timestamp;
        latency_estimator_.update_predict_to_send(predict_to_send, current_time);
    }

    // 3. 构建延迟信息
    fire_control::LatencyInfo latency = latency_estimator_.build(snapshot, current_time);

    // 4. 调用现有的火控逻辑，直接返回 FireCommand
    return fire_controller_.control(snapshot, current_time, latency);
}

aimer::AimMode AutoAimDecider::current_mode() const {
    // 从 BattlefieldSnapshot 获取当前模式
    const auto& snapshot = battlefield_->get();
    return snapshot.self_state.aim_mode;
}

void AutoAimDecider::reset() {
    fire_controller_.reset();
    last_frame_id_ = -1;
    debug::print(debug::PrintMode::INFO, "AutoAimDecider", "Reset");
}

}  // namespace autoaim::target_decision
