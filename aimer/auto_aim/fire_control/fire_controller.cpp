/**
 * @file fire_controller.cpp
 * @brief 火控调度器实现
 */

#include "fire_controller.hpp"

#include "mpc/mpc_controller.hpp"
#include "pid/pid_controller.hpp"
#include "plugin/param/runtime_parameter.hpp"

namespace autoaim::fire_control {

FireController::FireController()
{
    // 延迟初始化策略 (在第一次调用时根据配置创建)
}

FireController::~FireController() = default;

void FireController::ensure_strategy_initialized()
{
    // 读取模式配置
    bool use_mpc = runtime_param::get_param<bool>("AutoAim.FireControl.use_mpc");
    ControlMode target_mode = use_mpc ? ControlMode::MPC : ControlMode::PID;

    // 检查是否需要切换模式
    if (current_strategy_ && current_mode_ == target_mode) {
        return;  // 模式未变，无需处理
    }

    // 切换模式
    current_mode_ = target_mode;

    if (target_mode == ControlMode::MPC) {
        // 确保 MPC 策略已创建
        if (!mpc_strategy_) {
            mpc_strategy_ = std::make_unique<MpcController>();
        }
        current_strategy_ = mpc_strategy_.get();
    } else {
        // 确保 PID 策略已创建
        if (!pid_strategy_) {
            pid_strategy_ = std::make_unique<PidController>();
        }
        current_strategy_ = pid_strategy_.get();
    }
}

FireCommand FireController::control(
    const predictor::BattlefieldSnapshot& snapshot,
    double current_time,
    const LatencyInfo& latency
)
{
    ensure_strategy_initialized();

    if (!current_strategy_) {
        // 不应该发生，但做防御性处理
        FireCommand cmd;
        cmd.control_enabled = false;
        return cmd;
    }

    return current_strategy_->process(snapshot, current_time, latency);
}

void FireController::reset()
{
    if (mpc_strategy_) {
        mpc_strategy_->reset();
    }
    if (pid_strategy_) {
        pid_strategy_->reset();
    }
}

const char* FireController::strategy_name() const
{
    if (current_strategy_) {
        return current_strategy_->name();
    }
    return "None";
}

const TargetSelection& FireController::last_selection() const
{
    static TargetSelection empty;
    if (current_strategy_) {
        return current_strategy_->last_selection();
    }
    return empty;
}

const AimResult& FireController::last_aim() const
{
    static AimResult empty;
    if (current_strategy_) {
        return current_strategy_->last_aim();
    }
    return empty;
}

}  // namespace autoaim::fire_control
