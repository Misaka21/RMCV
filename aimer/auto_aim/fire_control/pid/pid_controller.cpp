/**
 * @file pid_controller.cpp
 * @brief PID 模式火控策略实现
 */

#include "pid_controller.hpp"

#include <cmath>

#include "aimer/auto_aim/fire_control/target_selector/target_selector.hpp"
#include "aimer/auto_aim/fire_control/trajectory/trajectory_solver.hpp"
#include "aimer/common/types.hpp"
#include "plugin/param/runtime_parameter.hpp"

namespace autoaim::fire_control {

PidController::PidController()
{
    target_selector_ = std::make_unique<TargetSelector>();
    trajectory_solver_ = std::make_unique<TrajectorySolver>();
}

PidController::~PidController() = default;

void PidController::reset()
{
    target_selector_->clear_target();
    last_selection_ = {};
    last_spin_aim_ = {};
    last_aim_ = {};
    lost_count_ = 0;
}

FireCommand PidController::process(
    const predictor::BattlefieldSnapshot& snapshot,
    double current_time
)
{
    // 计算时间差
    double dt = current_time - snapshot.timestamp;

    // 从 IMU 获取当前云台姿态
    extract_euler(snapshot.self_state.q_imu, current_yaw_, current_pitch_);
    last_time_ = current_time;

    // 阶段1: 目标选择
    TargetSelection selection = select_target(snapshot, dt);
    last_selection_ = selection;

    if (!selection.has_target || !selection.vehicle) {
        lost_count_++;
        if (lost_count_ > MAX_LOST_COUNT) {
            reset();
        }
        return no_target_command();
    }
    lost_count_ = 0;

    // 预估飞行时间 (用于反陀螺预测)
    double distance = selection.predicted_pos.norm();
    double fly_time = trajectory_solver_->estimate_fly_time(
        distance,
        snapshot.self_state.bullet_speed
    );

    // 阶段2: 反陀螺瞄准
    SpinAimResult spin_aim = compute_spin_aim(*selection.vehicle, fly_time);
    last_spin_aim_ = spin_aim;

    if (!spin_aim.valid) {
        return no_target_command();
    }

    // 阶段3: 弹道解算
    AimResult aim = solve_trajectory(
        spin_aim.target_pos,
        snapshot.self_state.bullet_speed
    );
    last_aim_ = aim;

    if (!aim.valid) {
        return no_target_command();
    }

    // 阶段4: 开火判断
    const predictor::ArmorState* armor = nullptr;
    if (spin_aim.armor_idx >= 0 && spin_aim.armor_idx < selection.vehicle->armor_count) {
        armor = &selection.vehicle->armors[spin_aim.armor_idx];
    }

    bool fire = check_fire_condition(
        aim,
        spin_aim,
        armor,
        selection.vehicle->confidence
    );

    // 生成指令
    return generate_command(selection, spin_aim, aim, fire);
}

TargetSelection PidController::select_target(
    const predictor::BattlefieldSnapshot& snapshot,
    double dt
)
{
    return target_selector_->select(snapshot, dt);
}

SpinAimResult PidController::compute_spin_aim(
    const predictor::VehicleState& vehicle,
    double fly_time
)
{
    return spin_aim_.compute(vehicle, fly_time);
}

AimResult PidController::solve_trajectory(
    const Eigen::Vector3d& target_pos,
    double bullet_speed
)
{
    return trajectory_solver_->solve(target_pos, bullet_speed);
}

bool PidController::check_fire_condition(
    const AimResult& aim,
    const SpinAimResult& spin_aim,
    const predictor::ArmorState* armor,
    double confidence
) const
{
    // 检查置信度
    double min_confidence = runtime_param::get_param<double>(
        "AutoAim.FireControl.min_confidence"
    );
    if (confidence < min_confidence) {
        return false;
    }

    // INDIRECT 模式: 检查是否到达开火时机
    if (spin_aim.mode == AimMode::INDIRECT) {
        double fire_advance = runtime_param::get_param<double>(
            "AutoAim.FireControl.PID.fire_advance"
        );
        double control_to_fire = runtime_param::get_param<double>(
            "AutoAim.FireControl.Latency.control_to_fire"
        );
        // 开火时机 = 装甲板出现时间 - 指令到出膛延迟 - 提前量
        // 即: time_to_fire <= control_to_fire + fire_advance 时开火
        if (spin_aim.time_to_fire > control_to_fire + fire_advance) {
            return false;
        }
    }

    // 角度误差
    double yaw_err = aim.yaw - current_yaw_;
    double pitch_err = aim.pitch - current_pitch_;
    double distance = aim.distance;

    // 将角度误差转换为落点偏移距离 (米)
    double hit_offset_yaw = distance * std::tan(std::abs(yaw_err));
    double hit_offset_pitch = distance * std::tan(std::abs(pitch_err));

    // 装甲板尺寸
    double armor_width = armor ? armor->width() : SMALL_ARMOR_WIDTH;
    double armor_height = armor ? armor->height() : SMALL_ARMOR_HEIGHT;

    // 考虑装甲板朝向
    double cos_inclined = std::abs(std::cos(spin_aim.z_to_v));

    // 开火判断: 落点偏移 < 装甲板有效区域
    double error_rate = runtime_param::get_param<double>(
        "AutoAim.FireControl.error_rate"
    );
    bool yaw_ok = hit_offset_yaw < (armor_width / 2.0) * cos_inclined * error_rate;
    bool pitch_ok = hit_offset_pitch < (armor_height / 2.0) * error_rate;

    return yaw_ok && pitch_ok;
}

FireCommand PidController::generate_command(
    const TargetSelection& selection,
    const SpinAimResult& spin_aim,
    const AimResult& aim,
    bool fire
)
{
    FireCommand cmd;
    cmd.control_enabled = true;

    // 云台控制 (仅位置，无前馈)
    cmd.yaw = static_cast<float>(aim.yaw);
    cmd.yaw_vel = 0;
    cmd.yaw_acc = 0;

    cmd.pitch = static_cast<float>(aim.pitch);
    cmd.pitch_vel = 0;
    cmd.pitch_acc = 0;

    // 射击控制
    cmd.allow_fire = true;
    cmd.fire_now = fire;

    // 调试信息
    cmd.target_id = selection.target_id;
    cmd.confidence = static_cast<float>(
        selection.vehicle ? selection.vehicle->confidence : 0
    );

    // 计算跟踪误差
    double yaw_err = aim.yaw - current_yaw_;
    double pitch_err = aim.pitch - current_pitch_;
    double hit_offset_yaw = aim.distance * std::tan(std::abs(yaw_err));
    double hit_offset_pitch = aim.distance * std::tan(std::abs(pitch_err));
    cmd.tracking_error = static_cast<float>(
        std::hypot(hit_offset_yaw, hit_offset_pitch)
    );

    return cmd;
}

FireCommand PidController::no_target_command()
{
    FireCommand cmd;
    cmd.control_enabled = false;
    cmd.allow_fire = false;
    cmd.fire_now = false;
    cmd.target_id = -1;
    return cmd;
}

void PidController::extract_euler(
    const Eigen::Quaterniond& q,
    double& yaw,
    double& pitch
)
{
    // 从四元数提取 yaw 和 pitch (ZYX 顺序)
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
    yaw = euler[0];
    pitch = euler[1];
}

}  // namespace autoaim::fire_control
