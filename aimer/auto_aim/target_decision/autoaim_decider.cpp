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

aimer::AimingDecision AutoAimDecider::decide(double current_time) {
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

    // 4. 调用现有的火控逻辑
    ::fire_control::FireCommand cmd = fire_controller_.control(snapshot, current_time, latency);

    // 5. 转换为 AimingDecision
    return to_aiming_decision(
        cmd,
        fire_controller_.last_selection(),
        fire_controller_.last_aim(),
        snapshot.timestamp,
        snapshot.frame_id
    );
}

void AutoAimDecider::reset() {
    fire_controller_.reset();
    last_frame_id_ = -1;
    debug::print(debug::PrintMode::INFO, "AutoAimDecider", "Reset");
}

aimer::AimMode AutoAimDecider::current_mode() const {
    // 从 BattlefieldSnapshot 获取当前模式
    const auto& snapshot = battlefield_->get();
    return snapshot.self_state.aim_mode;
}

aimer::AimingDecision AutoAimDecider::to_aiming_decision(
    const ::fire_control::FireCommand& cmd,
    const fire_control::TargetSelection& selection,
    const fire_control::AimResult& aim,
    double timestamp,
    int frame_id
) {
    aimer::AimingDecision decision;

    decision.valid = cmd.control_enabled;
    decision.target_id = cmd.target_id;
    decision.target_type = aimer::TargetType::ARMOR;

    // 瞄准点 (从 selection 获取)
    if (selection.has_target && selection.vehicle) {
        decision.aim_point = selection.predicted_pos;
        decision.aim_velocity = selection.vehicle->velocity;
    }

    // 弹道解算结果
    decision.aim_yaw = aim.yaw;
    decision.aim_pitch = aim.pitch;
    decision.aim_distance = aim.distance;
    decision.fly_time = aim.fly_time;

    // 云台规划
    decision.yaw = cmd.yaw;
    decision.pitch = cmd.pitch;
    decision.yaw_vel = cmd.yaw_vel;
    decision.pitch_vel = cmd.pitch_vel;
    decision.yaw_acc = cmd.yaw_acc;
    decision.pitch_acc = cmd.pitch_acc;

    // 开火决策
    decision.allow_fire = cmd.allow_fire;
    decision.fire_now = cmd.fire_now;

    // 置信度
    decision.confidence = cmd.confidence;
    decision.tracking_error = cmd.tracking_error;

    // 时间戳
    decision.timestamp = timestamp;
    decision.frame_id = frame_id;

    return decision;
}

}  // namespace autoaim::target_decision
