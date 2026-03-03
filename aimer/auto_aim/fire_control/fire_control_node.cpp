/**
 * @file fire_control_node.cpp
 * @brief 自瞄火控节点实现
 */

#include "fire_control_node.hpp"

#include <chrono>
#include <thread>

#include "aimer/common/robot_state.hpp"
#include "aimer/common/latency/latency_estimator.hpp"
#include "aimer/common/trajectory/solver_factory.hpp"
#include "fire_controller.hpp"
#include "aimer/auto_aim/predictor/types.hpp"
#include "aimer/common/fire_control_types.hpp"
#include "umt/BasicObjManager.hpp"
#include "plugin/debug/logger.hpp"
#include "plugin/watchdog/watchdog_node.hpp"
#include "plugin/webview/dashboard.hpp"

namespace autoaim::fire_control {

namespace {

// 获取当前时间 (秒)
double get_current_time() {
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration<double>(duration).count();
}

// 从 snapshot 提取延迟构建所需参数
::fire_control::LatencyInfo build_latency(
    const aimer::LatencyEstimator& estimator,
    const predictor::BattlefieldSnapshot& snapshot
) {
    // img_to_predict
    double img_to_predict = (snapshot.predict_timestamp > 0)
        ? (snapshot.predict_timestamp - snapshot.timestamp)
        : 0.015;

    // 目标距离
    double distance = 5.0;
    if (snapshot.get_primary()) {
        const auto* armor = snapshot.get_primary()->get_recommended_armor();
        if (armor) {
            distance = armor->position.norm();
        }
    }

    // 弹速
    double bullet_speed = snapshot.self_state.bullet_speed;

    return estimator.build(img_to_predict, distance, bullet_speed, "AutoAim.FireControl");
}

/**
 * @brief 迭代更新 fire_to_hit (参考 rm.cv.fans filter_to_prediction_time)
 *
 * 问题: 弹道解算需要预测位置 → 预测位置需要 prediction_latency()
 *       → prediction_latency() 需要 fire_to_hit → 鸡生蛋
 *
 * 解决: 迭代收敛，通常 2 次迭代即可
 */
void finalize_latency(
    ::fire_control::LatencyInfo& latency,
    const predictor::BattlefieldSnapshot& snapshot
) {
    const auto* target = snapshot.get_primary();
    if (!target) return;

    constexpr int NUM_ITERATIONS = 2;
    for (int iter = 0; iter < NUM_ITERATIONS; ++iter) {
        double dt = latency.prediction_latency();
        Eigen::Vector3d pos = target->predict_armor_position(
            target->recommended_armor_idx, dt
        );

        ::fire_control::AimResult aim = ::fire_control::trajectory::solve(
            pos, std::max(snapshot.self_state.bullet_speed, 10.0f)
        );

        if (aim.valid) {
            latency.set_fly_time(aim.fly_time);
        }
    }
}

}  // namespace

void fire_control_run(const std::string& /* config_path */) {
    debug::print(debug::PrintMode::INFO, "AutoAimFireControl", "Starting...");

    // 数据源
    auto battlefield = umt::BasicObjManager<predictor::BattlefieldSnapshot>::find_or_create("battlefield");
    auto fire_cmd = umt::BasicObjManager<::fire_control::FireCommand>::find_or_create("fire_command");
    auto fire_debug = umt::BasicObjManager<::fire_control::FireDebugInfo>::find_or_create("fire_debug");
    auto aim_mode_obj = umt::BasicObjManager<uint8_t>::find_or_create("current_aim_mode", 0);
    auto app_running = umt::BasicObjManager<bool>::find_or_create("app_running", true);

    // 自瞄控制器
    FireController controller;

    // 延迟估计器 (通用)
    aimer::LatencyEstimator latency_estimator;

    // 模式跟踪
    aimer::AimMode last_mode = aimer::AimMode::DISABLED;
    int last_frame_id = -1;

    debug::print(debug::PrintMode::INFO, "AutoAimFireControl", "Running at 500Hz");

    // 主循环 (500Hz)
    const auto period = std::chrono::microseconds(2000);  // 2ms
    auto next_time = std::chrono::steady_clock::now();

    while (app_running->get()) {
        watchdog::heartbeat("autoaim_fire_control");

        // 获取战场快照
        const auto& snapshot = battlefield->get();
        double current_time = get_current_time();

        // 统一从 hardware 实时共享对象读取 aim_mode，避免 snapshot 停更导致模式滞后
        aimer::AimMode mode = aimer::to_aim_mode(aim_mode_obj->get());
        aimer::AimMode prev_mode = last_mode;

        // 检测新帧，更新延迟估计
        if (snapshot.frame_id != last_frame_id && snapshot.predict_timestamp > 0) {
            last_frame_id = snapshot.frame_id;
            double predict_to_send = current_time - snapshot.predict_timestamp;
            latency_estimator.update_predict_to_send(predict_to_send, current_time);
        }

        // 模式切换检测
        if (mode != last_mode) {
            debug::print(debug::PrintMode::INFO, "AutoAimFireControl",
                "Mode switch: {} -> {}",
                aimer::aim_mode_name(last_mode),
                aimer::aim_mode_name(mode));

            // 重置控制器状态
            if (mode == aimer::AimMode::AUTOAIM) {
                controller.reset();
            }
            last_mode = mode;
        }

        // 构建延迟信息
        LatencyInfo latency = build_latency(latency_estimator, snapshot);

        // 迭代更新 fire_to_hit (延迟准备在 node 层完成)
        finalize_latency(latency, snapshot);

        // 根据模式处理
        ::fire_control::FireCommand cmd{};
        bool should_write = true;

        switch (mode) {
        case aimer::AimMode::AUTOAIM:
            cmd = controller.control(snapshot, current_time, latency);
            break;

        case aimer::AimMode::ENERGY_SMALL:
        case aimer::AimMode::ENERGY_LARGE:
            // 能量机关模式 - 由 autobuff 模块处理
            cmd.control_enabled = false;
            // 只在进入能量机关模式时写一次 disable，避免持续覆盖 autobuff 的 fire_command
            should_write = (prev_mode != mode);
            break;

        case aimer::AimMode::DISABLED:
        default:
            cmd.control_enabled = false;
            break;
        }

        // 输出控制指令
        if (should_write) {
            fire_cmd->get() = cmd;
        }

        // 写入调试信息 (供 predictor_node 绘制)
        // 只要在 AUTOAIM 模式就写，不要求 control_enabled
        {
            ::fire_control::FireDebugInfo dbg;
            // 诊断字段: 无论何种模式都写，用于确认线程存活和模式
            dbg.fc_mode = static_cast<uint8_t>(mode);
            dbg.fc_heartbeat = current_time;
            dbg.snapshot_valid_mask = snapshot.valid_mask;
            dbg.snapshot_primary_id = snapshot.primary_target_id;
            dbg.snapshot_frame_id = snapshot.frame_id;
            dbg.bullet_speed = snapshot.self_state.bullet_speed;

            // 云台状态: 无论是否有目标都写，用于绘制枪管指向
            {
                const auto& gimbal = controller.gimbal_state();
                dbg.gimbal_yaw = gimbal.yaw;
                dbg.gimbal_pitch = gimbal.pitch;
                dbg.gimbal_yaw_vel = gimbal.yaw_vel;
                dbg.gimbal_pitch_vel = gimbal.pitch_vel;
            }

            // 延迟分解 (ms)
            dbg.latency_img_to_predict = latency.img_to_predict * 1000;
            dbg.latency_predict_to_send = latency.predict_to_send * 1000;
            dbg.latency_send_to_control = latency.send_to_control * 1000;
            dbg.latency_fire_to_hit = latency.fire_to_hit * 1000;
            dbg.latency_total = latency.prediction_latency() * 1000;

            if (mode == aimer::AimMode::AUTOAIM) {
                const auto& aim = controller.last_aim();
                const auto& armor_aim = controller.last_armor_aim();
                const auto& gimbal = controller.gimbal_state();

                dbg.valid = aim.valid;
                dbg.control_enabled = cmd.control_enabled;
                dbg.fire_now = cmd.fire_now;
                dbg.target_id = cmd.target_id;
                dbg.armor_idx = armor_aim.armor_idx;
                dbg.target_pos = armor_aim.target_pos;

                dbg.aim_yaw = aim.yaw;
                dbg.aim_pitch = aim.pitch;
                dbg.cmd_yaw = cmd.yaw;
                dbg.cmd_pitch = cmd.pitch;
                dbg.gimbal_yaw = gimbal.yaw;
                dbg.gimbal_pitch = gimbal.pitch;

                dbg.distance = aim.distance;
                dbg.tracking_error = cmd.tracking_error;
                dbg.fly_time = aim.fly_time;
                dbg.timestamp = current_time;
                dbg.fail_stage = controller.last_fail_stage();
            }
            fire_debug->get() = dbg;
        }

        // 遥测数据
        dashboard::set("fire.yaw", cmd.yaw);
        dashboard::set("fire.pitch", cmd.pitch);
        dashboard::set("fire.control", cmd.control_enabled ? 1 : 0);
        dashboard::set("fire.fire_now", cmd.fire_now ? 1 : 0);

        // 等待下一周期
        next_time += period;
        std::this_thread::sleep_until(next_time);
    }

    debug::print(debug::PrintMode::INFO, "AutoAimFireControl", "Stopped");
}

void start_fire_control_node(const std::string& config_path) {
    std::thread([config_path]() {
        fire_control_run(config_path);
    }).detach();
}

}  // namespace autoaim::fire_control
