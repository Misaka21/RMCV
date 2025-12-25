/**
 * @file fire_controller_node.cpp
 * @brief 火控线程节点实现
 */

#include "fire_controller_node.hpp"

#include <chrono>
#include <thread>

#include "fire_controller.hpp"
#include "target_selector/target_selector.hpp"
#include "aimer/auto_aim/predictor/types.hpp"
#include "umt/BasicObjManager.hpp"
#include "plugin/debug/log.hpp"
#include "plugin/param/runtime_parameter.hpp"

namespace autoaim::fire_control {

namespace {

// 获取当前时间 (秒)
double get_current_time()
{
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration<double>(duration).count();
}

// 从 TOML 加载火控配置 (支持热更新)
FireControlConfig load_config(const std::string& /* config_path */)
{
    FireControlConfig config;

    // ==================== 射击阈值 ====================
    config.fire_threshold = runtime_param::get_param<double>("AutoAim.FireControl.fire_threshold");
    config.min_confidence = runtime_param::get_param<double>("AutoAim.FireControl.min_confidence");

    // ==================== 延迟配置 ====================
    config.latency.img_to_predict = runtime_param::get_param<double>("AutoAim.FireControl.Latency.img_to_predict");
    config.latency.predict_to_send = runtime_param::get_param<double>("AutoAim.FireControl.Latency.predict_to_send");
    config.latency.send_to_control = runtime_param::get_param<double>("AutoAim.FireControl.Latency.send_to_control");
    config.latency.control_to_fire = runtime_param::get_param<double>("AutoAim.FireControl.Latency.control_to_fire");
    config.latency.steady_state_time_constant = runtime_param::get_param<double>("AutoAim.FireControl.Latency.steady_state_time_constant");

    // ==================== MPC 参数 ====================
    // yaw 轴
    config.q_yaw_pos = runtime_param::get_param<double>("AutoAim.FireControl.MPC.q_yaw_pos");
    config.q_yaw_vel = runtime_param::get_param<double>("AutoAim.FireControl.MPC.q_yaw_vel");
    config.r_yaw_acc = runtime_param::get_param<double>("AutoAim.FireControl.MPC.r_yaw_acc");
    config.max_yaw_acc = runtime_param::get_param<double>("AutoAim.FireControl.MPC.max_yaw_acc");

    // pitch 轴
    config.q_pitch_pos = runtime_param::get_param<double>("AutoAim.FireControl.MPC.q_pitch_pos");
    config.q_pitch_vel = runtime_param::get_param<double>("AutoAim.FireControl.MPC.q_pitch_vel");
    config.r_pitch_acc = runtime_param::get_param<double>("AutoAim.FireControl.MPC.r_pitch_acc");
    config.max_pitch_acc = runtime_param::get_param<double>("AutoAim.FireControl.MPC.max_pitch_acc");

    // 求解器
    config.mpc_max_iter = runtime_param::get_param<int>("AutoAim.FireControl.MPC.max_iter");

    // ==================== 弹道参数 ====================
    config.gravity = runtime_param::get_param<double>("AutoAim.FireControl.Trajectory.gravity");
    config.air_resistance_k = runtime_param::get_param<double>("AutoAim.FireControl.Trajectory.air_resistance_k");
    config.trajectory_max_iter = runtime_param::get_param<int>("AutoAim.FireControl.Trajectory.max_iter");

    return config;
}

// 加载目标选择器配置
TargetSelector::Config load_selector_config()
{
    TargetSelector::Config config;

    config.w_area = runtime_param::get_param<double>("AutoAim.FireControl.TargetSelector.w_area");
    config.w_still = runtime_param::get_param<double>("AutoAim.FireControl.TargetSelector.w_still");
    config.w_distance = runtime_param::get_param<double>("AutoAim.FireControl.TargetSelector.w_distance");
    config.w_priority = runtime_param::get_param<double>("AutoAim.FireControl.TargetSelector.w_priority");
    config.w_confidence = runtime_param::get_param<double>("AutoAim.FireControl.TargetSelector.w_confidence");

    config.area_filter_ratio = runtime_param::get_param<double>("AutoAim.FireControl.TargetSelector.area_filter_ratio");
    config.switch_hysteresis = runtime_param::get_param<double>("AutoAim.FireControl.TargetSelector.switch_hysteresis");
    config.switch_armor_hysteresis = runtime_param::get_param<double>("AutoAim.FireControl.TargetSelector.switch_armor_hysteresis");
    config.still_speed_scale = runtime_param::get_param<double>("AutoAim.FireControl.TargetSelector.still_speed_scale");
    config.max_distance = runtime_param::get_param<double>("AutoAim.FireControl.TargetSelector.max_distance");
    config.max_angle = runtime_param::get_param<double>("AutoAim.FireControl.TargetSelector.max_angle");

    return config;
}

}  // namespace

void fire_control_run(const std::string& config_path)
{
    debug::print("info", "FireControl", "Starting fire control node...");

    // 加载配置
    FireControlConfig config = load_config(config_path);
    TargetSelector::Config selector_config = load_selector_config();

    // 创建火控器
    FireController controller(config);
    controller.set_selector_config(selector_config);

    // 获取共享数据
    auto battlefield = umt::BasicObjManager<predictor::BattlefieldSnapshot>::find_or_create("battlefield");
    auto fire_cmd = umt::BasicObjManager<FireCommand>::find_or_create("fire_command");

    debug::print("info", "FireControl", "Fire control node started, running at 100Hz");

    // 主循环 (100Hz)
    const auto period = std::chrono::microseconds(10000);  // 10ms
    auto next_time = std::chrono::steady_clock::now();

    // 配置热更新计数器 (每 100 次循环更新一次 = 1秒)
    int config_update_counter = 0;
    constexpr int CONFIG_UPDATE_INTERVAL = 100;

    while (true) {
        // 获取战场快照
        const auto& snapshot = battlefield->get();

        // 获取当前时间
        double current_time = get_current_time();

        // 执行控制
        FireCommand cmd = controller.control(snapshot, current_time);

        // 输出控制指令
        fire_cmd->get() = cmd;

        // 定期热更新配置
        if (++config_update_counter >= CONFIG_UPDATE_INTERVAL) {
            config_update_counter = 0;
            controller.set_config(load_config(config_path));
            controller.set_selector_config(load_selector_config());
        }

        // 等待下一周期
        next_time += period;
        std::this_thread::sleep_until(next_time);
    }
}

void start_fire_control(const std::string& config_path)
{
    std::thread([config_path]() {
        fire_control_run(config_path);
    }).detach();
}

void background_fire_control_run(const std::string& config_path)
{
    start_fire_control(config_path);
}

}  // namespace autoaim::fire_control
