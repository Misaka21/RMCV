/**
 * @file fire_controller.cpp
 * @brief 火控主类实现
 */

#include "fire_controller.hpp"

#include <cmath>

#include "planner/gimbal_planner.hpp"
#include "target_selector/target_selector.hpp"
#include "trajectory/trajectory_solver.hpp"
#include "plugin/param/runtime_parameter.hpp"

namespace autoaim::fire_control {

FireController::FireController()
{
    // 创建子组件 (参数在内部直接调用 get_param)
    target_selector_ = std::make_unique<TargetSelector>();
    trajectory_solver_ = std::make_unique<TrajectorySolver>();
    gimbal_planner_ = std::make_unique<GimbalPlanner>();
}

FireController::~FireController() = default;

void FireController::reset()
{
    target_selector_->clear_target();
    last_selection_ = {};
    last_plan_ = {};
    last_aim_ = {};
    lost_count_ = 0;
}

FireCommand FireController::control(
    const predictor::BattlefieldSnapshot& snapshot,
    double current_time
)
{
    // 计算时间差 (用于插值)
    double dt = current_time - snapshot.timestamp;

    // 从 IMU 获取当前云台姿态
    extract_euler(snapshot.self_state.q_imu, current_yaw_, current_pitch_);

    // 计算云台速度 (简单差分)
    if (last_time_ > 0) {
        double time_diff = current_time - last_time_;
        if (time_diff > 0.001) {
            // 这里简化处理，实际应该从 IMU 获取角速度
            // current_yaw_vel_ = ...
            // current_pitch_vel_ = ...
        }
    }
    last_time_ = current_time;

    // 阶段1: 目标选择
    TargetSelection selection = select_target(snapshot, dt);
    last_selection_ = selection;

    if (!selection.has_target) {
        lost_count_++;
        if (lost_count_ > MAX_LOST_COUNT) {
            reset();
        }
        return no_target_command();
    }
    lost_count_ = 0;

    // 阶段2: 弹道解算 (验证目标可达)
    AimResult aim = solve_trajectory(
        selection.predicted_pos,
        snapshot.self_state.bullet_speed
    );
    last_aim_ = aim;

    if (!aim.valid) {
        return no_target_command();
    }

    // 读取模式配置
    bool use_mpc = runtime_param::get_param<bool>("AutoAim.FireControl.use_mpc");

    if (use_mpc) {
        // ========== MPC 模式: 完整轨迹规划 ==========
        // 阶段3: MPC 规划
        GimbalPlan plan = plan_gimbal(
            *selection.vehicle,
            snapshot.self_state,
            snapshot.self_state.bullet_speed
        );
        last_plan_ = plan;

        if (!plan.valid) {
            return no_target_command();
        }

        // 阶段4: 射击决策 (统一使用物理判断)
        // MPC 提供了考虑开火延迟后的目标角度
        double yaw_err = plan.target_yaw - current_yaw_;
        double pitch_err = plan.target_pitch - current_pitch_;
        bool fire = check_fire_condition(
            yaw_err, pitch_err, aim.distance,
            selection.armor, selection.vehicle->confidence
        );

        // 生成指令 (含速度/加速度前馈)
        return generate_command(selection, plan, fire);

    } else {
        // ========== 简化模式: 仅位置跟踪 ==========
        double yaw_err = aim.yaw - current_yaw_;
        double pitch_err = aim.pitch - current_pitch_;
        bool fire = check_fire_condition(
            yaw_err, pitch_err, aim.distance,
            selection.armor, selection.vehicle->confidence
        );

        // 计算跟踪误差 (调试用)
        double hit_offset_yaw = aim.distance * std::tan(std::abs(yaw_err));
        double hit_offset_pitch = aim.distance * std::tan(std::abs(pitch_err));
        double tracking_error = std::hypot(hit_offset_yaw, hit_offset_pitch);

        // 生成简化指令 (仅位置，无前馈)
        return generate_simple_command(selection, aim, fire, tracking_error);
    }
}

TargetSelection FireController::select_target(
    const predictor::BattlefieldSnapshot& snapshot,
    double dt
)
{
    return target_selector_->select(snapshot, dt);
}

AimResult FireController::solve_trajectory(
    const Eigen::Vector3d& target_pos,
    double bullet_speed
)
{
    return trajectory_solver_->solve(target_pos, bullet_speed);
}

GimbalPlan FireController::plan_gimbal(
    const predictor::VehicleState& target,
    const aimer::RobotState& self_state,
    double bullet_speed
)
{
    return gimbal_planner_->plan(
        target,
        current_yaw_,
        current_pitch_,
        current_yaw_vel_,
        current_pitch_vel_,
        bullet_speed
    );
}

bool FireController::check_fire_condition(
    double yaw_err,
    double pitch_err,
    double distance,
    const predictor::ArmorState* armor,
    double confidence
) const
{
    // 检查置信度
    double min_confidence = runtime_param::get_param<double>("AutoAim.FireControl.min_confidence");
    if (confidence < min_confidence) {
        return false;
    }

    // 将角度误差转换为落点偏移距离 (米)
    double hit_offset_yaw = distance * std::tan(std::abs(yaw_err));
    double hit_offset_pitch = distance * std::tan(std::abs(pitch_err));

    // 装甲板尺寸 (使用 common/types.hpp 中的常量)
    double armor_width = armor ? armor->width() : SMALL_ARMOR_WIDTH;
    double armor_height = armor ? armor->height() : SMALL_ARMOR_HEIGHT;

    // 考虑装甲板朝向 (z_to_v 是装甲板法向与视线夹角)
    double cos_inclined = armor ? std::abs(std::cos(armor->z_to_v)) : 1.0;

    // 开火判断: 落点偏移 < 装甲板有效区域
    double error_rate = runtime_param::get_param<double>("AutoAim.FireControl.error_rate");
    bool yaw_ok = hit_offset_yaw < (armor_width / 2.0) * cos_inclined * error_rate;
    bool pitch_ok = hit_offset_pitch < (armor_height / 2.0) * error_rate;

    return yaw_ok && pitch_ok;
}

FireCommand FireController::generate_command(
    const TargetSelection& selection,
    const GimbalPlan& plan,
    bool fire
)
{
    FireCommand cmd;
    cmd.control_enabled = true;

    // 云台控制
    cmd.yaw = static_cast<float>(plan.yaw.position);
    cmd.yaw_vel = static_cast<float>(plan.yaw.velocity);
    cmd.yaw_acc = static_cast<float>(plan.yaw.acceleration);

    cmd.pitch = static_cast<float>(plan.pitch.position);
    cmd.pitch_vel = static_cast<float>(plan.pitch.velocity);
    cmd.pitch_acc = static_cast<float>(plan.pitch.acceleration);

    // 射击控制
    cmd.allow_fire = true;
    cmd.fire_now = fire;

    // 调试信息
    cmd.target_id = selection.target_id;
    cmd.tracking_error = static_cast<float>(plan.fire_error);  // 使用开火误差
    cmd.confidence = static_cast<float>(selection.vehicle ? selection.vehicle->confidence : 0);

    return cmd;
}

FireCommand FireController::generate_simple_command(
    const TargetSelection& selection,
    const AimResult& aim,
    bool fire,
    double tracking_error
)
{
    FireCommand cmd;
    cmd.control_enabled = true;

    // 云台控制 (仅位置，无前馈)
    cmd.yaw = static_cast<float>(aim.yaw);
    cmd.yaw_vel = 0;   // 无速度前馈
    cmd.yaw_acc = 0;   // 无加速度前馈

    cmd.pitch = static_cast<float>(aim.pitch);
    cmd.pitch_vel = 0;
    cmd.pitch_acc = 0;

    // 射击控制
    cmd.allow_fire = true;
    cmd.fire_now = fire;

    // 调试信息
    cmd.target_id = selection.target_id;
    cmd.tracking_error = static_cast<float>(tracking_error);
    cmd.confidence = static_cast<float>(selection.vehicle ? selection.vehicle->confidence : 0);

    return cmd;
}

FireCommand FireController::no_target_command()
{
    FireCommand cmd;
    cmd.control_enabled = false;
    cmd.allow_fire = false;
    cmd.fire_now = false;
    cmd.target_id = -1;
    return cmd;
}

void FireController::extract_euler(
    const Eigen::Quaterniond& q,
    double& yaw,
    double& pitch
)
{
    // 从四元数提取 yaw 和 pitch
    // 假设 ZYX 顺序
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
    yaw = euler[0];
    pitch = euler[1];
}

}  // namespace autoaim::fire_control
