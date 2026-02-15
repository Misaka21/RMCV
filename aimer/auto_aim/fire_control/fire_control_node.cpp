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
            pos, snapshot.self_state.bullet_speed
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

        // 获取当前模式
        aimer::AimMode mode = snapshot.self_state.aim_mode;
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
