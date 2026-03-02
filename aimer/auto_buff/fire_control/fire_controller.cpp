// auto_buff fire controller implementation (2026)

#include "fire_controller.hpp"

#include <cmath>

#include "plugin/param/runtime_parameter.hpp"

namespace autobuff::fire_control {

namespace {

constexpr int MAX_LOST_COUNT = 50;

}  // namespace

void FireController::reset() {
    last_time_ = 0.0;
    lost_count_ = 0;
}

double FireController::normalize_angle(double a) {
    while (a > M_PI) a -= 2 * M_PI;
    while (a < -M_PI) a += 2 * M_PI;
    return a;
}

::fire_control::FireCommand FireController::control(
    const autobuff::predictor::BuffSnapshot& snapshot,
    double current_time,
    const ::fire_control::LatencyInfo& latency)
{
    // 1) 更新云台状态
    double dt = (last_time_ > 0.0) ? (current_time - last_time_) : ::fire_control::DEFAULT_CONTROL_DT;
    gimbal_state_.update(snapshot.self_state.q_imu, dt);
    last_time_ = current_time;

    // 2) 快照有效性检查
    if (!snapshot.valid) {
        if (++lost_count_ > MAX_LOST_COUNT) reset();
        return no_target_command();
    }
    lost_count_ = 0;

    // 3) 构建候选列表
    auto candidates = ranker_.build(snapshot, latency, gimbal_state_);
    if (candidates.empty()) return no_target_command();

    // 4) 协同策略选择目标
    int chosen_slot = coop_.select(snapshot, candidates);

    // 5) 查找对应候选
    const SlotAimCandidate* chosen = nullptr;
    for (const auto& c : candidates) {
        if (c.slot_id == chosen_slot) {
            chosen = &c;
            break;
        }
    }
    if (!chosen || !chosen->ballistic_valid) return no_target_command();

    // 6) 开火判断 (在使用点直接读取参数，不缓存)
    double fire_threshold = runtime_param::get_param<double>("AutoBuff.FireControl.fire_threshold");
    double min_conf = runtime_param::get_param<double>("AutoBuff.FireControl.min_confidence");
    if (fire_threshold <= 0.0) fire_threshold = 0.02;
    if (min_conf <= 0.0) min_conf = 0.30;

    bool fire_now = snapshot.self_state.allow_fire
        && (static_cast<double>(chosen->confidence) >= min_conf)
        && (chosen->tracking_error <= fire_threshold)
        && chosen->ballistic_valid;

    // 7) 构建指令
    ::fire_control::FireCommand cmd;
    cmd.control_enabled = true;
    cmd.yaw = static_cast<float>(chosen->aim.yaw);
    cmd.pitch = static_cast<float>(chosen->aim.pitch);
    cmd.allow_fire = true;
    cmd.fire_now = fire_now;
    cmd.target_id = chosen->slot_id;
    cmd.tracking_error = static_cast<float>(chosen->tracking_error);
    cmd.confidence = chosen->confidence;
    return cmd;
}

::fire_control::FireCommand FireController::no_target_command() const {
    ::fire_control::FireCommand cmd;
    cmd.control_enabled = false;
    cmd.allow_fire = false;
    cmd.fire_now = false;
    cmd.target_id = -1;
    return cmd;
}

}  // namespace autobuff::fire_control
