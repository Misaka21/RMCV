/**
 * @file fire_control_node.cpp
 * @brief 自瞄火控节点实现
 */

#include "fire_control_node.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <limits>
#include <string>
#include <thread>
#include <variant>

#include "aimer/common/robot_state.hpp"
#include "aimer/common/math/math.hpp"
#include "aimer/common/latency/latency_estimator.hpp"
#include "aimer/common/transformer/transformer.hpp"
#include "fire_controller.hpp"
#include "aimer/auto_aim/predictor/types.hpp"
#include "aimer/common/fire_control_types.hpp"
#include "umt/BasicObjManager.hpp"
#include "plugin/debug/logger.hpp"
#include "plugin/param/runtime_parameter.hpp"
#include "plugin/watchdog/watchdog_node.hpp"
#include "plugin/webview/dashboard.hpp"

namespace autoaim::fire_control {

namespace {

constexpr int64_t AIM_MODE_STALE_TIMEOUT_US = 300000;  // 300ms
constexpr int64_t SNAPSHOT_STALE_TIMEOUT_US = 200000;  // 200ms
constexpr double DEFAULT_IMG_TO_PREDICT_LATENCY = 0.015;  // 15ms
constexpr double MIN_IMG_TO_PREDICT_LATENCY = 1e-4;       // 0.1ms
constexpr double MAX_IMG_TO_PREDICT_LATENCY = 0.1;        // 100ms

// 获取当前时间 (秒)
double get_current_time() {
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration<double>(duration).count();
}

int64_t get_current_time_us() {
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(
        now.time_since_epoch()).count();
}

double get_param_or(const std::string& name, double default_value) {
    auto ptr = runtime_param::find_param(name);
    if (ptr != nullptr) {
        if (auto* val = std::get_if<double>(&*ptr)) {
            return *val;
        }
    }
    return default_value;
}

bool get_param_or(const std::string& name, bool default_value) {
    auto ptr = runtime_param::find_param(name);
    if (ptr != nullptr) {
        if (auto* val = std::get_if<bool>(&*ptr)) {
            return *val;
        }
    }
    return default_value;
}

const predictor::VehicleState* choose_latency_target(
    const predictor::BattlefieldSnapshot& snapshot,
    int preferred_target_id
) {
    if (snapshot.is_valid(preferred_target_id)) {
        return &snapshot.vehicles[preferred_target_id];
    }
    return snapshot.get_primary();
}

int choose_latency_armor_idx(
    const predictor::VehicleState& vehicle,
    int preferred_armor_idx
) {
    if (preferred_armor_idx >= 0 && preferred_armor_idx < vehicle.armor_count) {
        return preferred_armor_idx;
    }

    int idx = vehicle.recommended_armor_idx;
    if (idx >= 0 && idx < vehicle.armor_count) {
        return idx;
    }
    if (vehicle.armor_count <= 0) {
        return -1;
    }

    int best_idx = 0;
    double best_score = vehicle.armors[0].score;
    for (int i = 1; i < vehicle.armor_count; ++i) {
        if (vehicle.armors[i].score > best_score) {
            best_score = vehicle.armors[i].score;
            best_idx = i;
        }
    }
    return best_idx;
}

int find_armor_idx_by_id(
    const predictor::VehicleState& vehicle,
    int armor_id
) {
    if (armor_id < 0) {
        return -1;
    }
    for (int i = 0; i < vehicle.armor_count; ++i) {
        if (vehicle.armors[i].id == armor_id) {
            return i;
        }
    }
    return -1;
}

// 从 snapshot 提取延迟构建所需参数
::fire_control::LatencyInfo build_latency(
    const aimer::LatencyEstimator& estimator,
    const predictor::BattlefieldSnapshot& snapshot,
    int preferred_target_id,
    int preferred_armor_idx
) {
    // img_to_predict
    double img_to_predict = DEFAULT_IMG_TO_PREDICT_LATENCY;
    if (snapshot.predict_timestamp > 0.0 && snapshot.timestamp > 0.0) {
        const double measured = snapshot.predict_timestamp - snapshot.timestamp;
        if (std::isfinite(measured)
            && measured >= MIN_IMG_TO_PREDICT_LATENCY
            && measured <= MAX_IMG_TO_PREDICT_LATENCY)
        {
            img_to_predict = measured;
        }
    }

    // 目标距离
    double distance = 5.0;
    if (const auto* target = choose_latency_target(snapshot, preferred_target_id)) {
        int armor_idx = choose_latency_armor_idx(*target, preferred_armor_idx);
        if (armor_idx >= 0) {
            Eigen::Vector3d pos = target->predict_armor_position(armor_idx, 0.0);
            if (pos.squaredNorm() > 1e-9) {
                distance = aimer::tf::world_to_barrel_origin_world(
                    pos, snapshot.self_state.q_imu
                ).norm();
            }
        }
    }

    // 弹速
    double bullet_speed = snapshot.self_state.bullet_speed;

    return estimator.build(img_to_predict, distance, bullet_speed, "AutoAim.FireControl");
}

}  // namespace

void fire_control_run(const std::string& /* config_path */) {
    debug::print(debug::PrintMode::INFO, "AutoAimFireControl", "Starting...");

    // 数据源
    auto battlefield = umt::BasicObjManager<predictor::BattlefieldSnapshot>::find_or_create("battlefield");
    auto fire_cmd = umt::BasicObjManager<::fire_control::FireCommand>::find_or_create("fire_command");
    auto fire_debug = umt::BasicObjManager<::fire_control::FireDebugInfo>::find_or_create("fire_debug");
    auto aim_mode_obj = umt::BasicObjManager<uint8_t>::find_or_create("current_aim_mode", 0);
    auto aim_mode_time_obj = umt::BasicObjManager<int64_t>::find_or_create("current_aim_mode_time_us", 0);
    auto app_running = umt::BasicObjManager<bool>::find_or_create("app_running", true);

    // 自瞄控制器
    FireController controller;

    // 延迟估计器 (通用)
    aimer::LatencyEstimator latency_estimator;

    // 模式跟踪
    aimer::AimMode last_mode = aimer::AimMode::DISABLED;
    bool last_snapshot_stale = true;
    int last_snapshot_frame_id = -1;
    int64_t last_snapshot_update_us = 0;
    int last_predict_to_send_frame_id = -1;
    double last_predict_to_send_predict_ts = -1.0;
    int cached_latency_frame_id = -1;
    int cached_latency_target_id = -1;
    int cached_latency_armor_idx = -2;
    double cached_latency_predict_ts = -1.0;
    LatencyInfo cached_latency{};
    bool has_cached_latency = false;
    double last_fc_log_ts = 0.0;
    bool has_last_cmd = false;
    int last_log_target_id = -1;
    int last_log_armor_idx = -1;
    double last_cmd_yaw = 0.0;
    double last_cmd_pitch = 0.0;
    double last_aim_yaw = 0.0;
    double last_aim_pitch = 0.0;
    int last_fail_stage = -1;

    debug::print(debug::PrintMode::INFO, "AutoAimFireControl", "Running at 500Hz");

    // 主循环 (500Hz)
    const auto period = std::chrono::microseconds(2000);  // 2ms
    auto next_time = std::chrono::steady_clock::now();

    while (app_running->get()) {
        watchdog::heartbeat("autoaim_fire_control");

        // 获取战场快照 (线程安全拷贝，避免 predictor 写入时读到半更新状态)
        const auto snapshot = battlefield->load();
        double current_time = get_current_time();
        int64_t current_time_us = get_current_time_us();

        if (snapshot.frame_id != last_snapshot_frame_id) {
            last_snapshot_frame_id = snapshot.frame_id;
            last_snapshot_update_us = current_time_us;
        }

        int64_t snapshot_age_us = (snapshot.self_state.timestamp_us > 0)
            ? (current_time_us - snapshot.self_state.timestamp_us)
            : std::numeric_limits<int64_t>::max();
        bool snapshot_stale = (snapshot.self_state.timestamp_us <= 0)
            || (snapshot_age_us < 0)
            || (snapshot_age_us > SNAPSHOT_STALE_TIMEOUT_US)
            || (last_snapshot_update_us > 0
                && (current_time_us - last_snapshot_update_us) > SNAPSHOT_STALE_TIMEOUT_US);

        // aim_mode 从 hardware 实时共享对象读取，不依赖可能过期的 snapshot
        aimer::AimMode mode = aimer::to_aim_mode(aim_mode_obj->load());
        int64_t aim_mode_time_us = aim_mode_time_obj->load();
        if (aim_mode_time_us <= 0 ||
            (current_time_us - aim_mode_time_us) > AIM_MODE_STALE_TIMEOUT_US) {
            mode = aimer::AimMode::DISABLED;
        }
        if (mode == aimer::AimMode::AUTOAIM && snapshot_stale && !last_snapshot_stale) {
            controller.reset();
        }
        last_snapshot_stale = snapshot_stale;
        aimer::AimMode prev_mode = last_mode;

        // 模式切换检测
        if (mode != last_mode) {
            debug::print(debug::PrintMode::INFO, "AutoAimFireControl",
                "Mode switch: {} -> {}",
                aimer::aim_mode_name(last_mode),
                aimer::aim_mode_name(mode));

            // 重置控制器状态
            if (mode == aimer::AimMode::AUTOAIM) {
                controller.reset();
                last_predict_to_send_frame_id = -1;
                last_predict_to_send_predict_ts = -1.0;
            }
            last_mode = mode;
        }

        // 构建延迟信息:
        // 优先沿用上一次火控实际选中的目标，避免 primary 与实际打击目标不一致。
        int latency_target_id = snapshot.primary_target_id;
        int latency_armor_idx = -1;
        const auto& last_sel = controller.last_selection();
        const auto& last_armor = controller.last_armor_aim();
        const int last_armor_id = controller.last_armor_id();
        if (last_sel.has_target && snapshot.is_valid(last_sel.target_id)) {
            latency_target_id = last_sel.target_id;
            const auto& latency_vehicle = snapshot.vehicles[latency_target_id];
            if (last_armor_id >= 0) {
                latency_armor_idx = find_armor_idx_by_id(latency_vehicle, last_armor_id);
            }
            if (latency_armor_idx < 0 && last_armor.valid) {
                if (last_armor.armor_idx >= 0 && last_armor.armor_idx < latency_vehicle.armor_count) {
                    latency_armor_idx = last_armor.armor_idx;
                }
            }
        }

        LatencyInfo latency{};
        if (mode == aimer::AimMode::AUTOAIM && !snapshot_stale) {
            const bool reason_no_cache = !has_cached_latency;
            const bool reason_new_frame = snapshot.frame_id != cached_latency_frame_id;
            // 只在新图像帧到达时依据 target/armor 重建，避免同帧 500Hz 循环里的抖动正反馈
            const bool reason_target = reason_new_frame
                && (latency_target_id != cached_latency_target_id);
            const bool reason_armor = reason_new_frame
                && (latency_armor_idx != cached_latency_armor_idx);
            const bool reason_predict_ts =
                std::abs(snapshot.predict_timestamp - cached_latency_predict_ts) > 1e-9;

            bool need_rebuild_latency = reason_no_cache
                || reason_new_frame
                || reason_target
                || reason_armor
                || reason_predict_ts;

            if (need_rebuild_latency) {
                if (get_param_or("AutoAim.FireControl.Debug.latency_detail", false)) {
                    static double last_latency_log_ts = 0.0;
                    const bool same_frame_rebuild = !reason_new_frame;
                    if (same_frame_rebuild || (current_time - last_latency_log_ts) > 0.5) {
                        debug::print(
                            same_frame_rebuild ? debug::PrintMode::INFO : debug::PrintMode::DEBUG,
                            "AutoAimFireControl",
                            "[LAT-REBUILD] frame={} cached_frame={} same_frame={} "
                            "reasons[nc={} nf={} tid={} aid={} pts={}] "
                            "target {}->{} armor {}->{} pred_ts {:.6f}->{:.6f}",
                            snapshot.frame_id,
                            cached_latency_frame_id,
                            same_frame_rebuild ? 1 : 0,
                            reason_no_cache ? 1 : 0,
                            reason_new_frame ? 1 : 0,
                            reason_target ? 1 : 0,
                            reason_armor ? 1 : 0,
                            reason_predict_ts ? 1 : 0,
                            cached_latency_target_id,
                            latency_target_id,
                            cached_latency_armor_idx,
                            latency_armor_idx,
                            cached_latency_predict_ts,
                            snapshot.predict_timestamp
                        );
                        last_latency_log_ts = current_time;
                    }
                }

                latency = build_latency(
                    latency_estimator, snapshot, latency_target_id, latency_armor_idx
                );

                cached_latency = latency;
                cached_latency_frame_id = snapshot.frame_id;
                cached_latency_target_id = latency_target_id;
                cached_latency_armor_idx = latency_armor_idx;
                cached_latency_predict_ts = snapshot.predict_timestamp;
                has_cached_latency = true;
            } else {
                latency = cached_latency;
            }
        } else if (mode == aimer::AimMode::AUTOAIM && snapshot_stale) {
            has_cached_latency = false;
        } else if (has_cached_latency) {
            // 非 AUTOAIM 模式复用上次延迟用于可视化，不再重算弹道。
            latency = cached_latency;
        }

        // 根据模式处理
        ::fire_control::FireCommand cmd{};
        bool should_write = true;

        switch (mode) {
        case aimer::AimMode::AUTOAIM:
            if (snapshot_stale) {
                // 预测停更时立即失能，避免持续复用旧目标
                cmd.control_enabled = false;
            } else {
                cmd = controller.control(snapshot, current_time, latency);
                latency = controller.last_latency();
                if (has_cached_latency) {
                    cached_latency = latency;
                }
            }
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
            // 对齐 rm.cv.fans: predict_to_send 在“预测完成 -> 命令准备发出”时采样，
            // 不需要串口回传；该量用于下一帧延迟估计。
            if (mode == aimer::AimMode::AUTOAIM
                && !snapshot_stale
                && snapshot.predict_timestamp > 0.0
                && snapshot.timestamp > 0.0)
            {
                if (snapshot.frame_id != last_predict_to_send_frame_id
                    || std::abs(snapshot.predict_timestamp - last_predict_to_send_predict_ts) > 1e-9)
                {
                    const double send_ts = get_current_time();
                    const double predict_to_send = send_ts - snapshot.predict_timestamp;
                    latency_estimator.update_predict_to_send(predict_to_send, snapshot.timestamp);
                    last_predict_to_send_frame_id = snapshot.frame_id;
                    last_predict_to_send_predict_ts = snapshot.predict_timestamp;
                }
            }
            fire_cmd->store(cmd);
        }

        // 写入调试信息 (供 visualizer 绘制)
        {
            ::fire_control::FireDebugInfo dbg;
            // 诊断字段: 无论何种模式都写，用于确认线程存活和模式
            dbg.fc_mode = static_cast<uint8_t>(mode);
            dbg.fc_heartbeat = current_time;
            dbg.snapshot_valid_mask = snapshot.valid_mask;
            dbg.snapshot_primary_id = snapshot.primary_target_id;
            dbg.snapshot_frame_id = snapshot.frame_id;
            dbg.bullet_speed = snapshot.self_state.bullet_speed;
            dbg.snapshot_age_ms = (snapshot.self_state.timestamp_us > 0)
                ? static_cast<double>(current_time_us - snapshot.self_state.timestamp_us) / 1000.0
                : -1.0;

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
            dbg.latency_control_to_fire = latency.control_to_fire * 1000;
            dbg.latency_fire_to_hit = latency.fire_to_hit * 1000;
            dbg.latency_total = latency.prediction_latency() * 1000;
            dbg.latency_hit_total = latency.hit_latency() * 1000;
            dbg.prediction_dt = controller.last_prediction_dt();
            dbg.cmd_additional_predict_time = runtime_param::get_param<double>(
                "AutoAim.FireControl.Cmd.additional_predict_time"
            );
            // 偏置配置单位是 deg，debug 字段也保持 deg 直读展示
            dbg.aim_offset_yaw = runtime_param::get_param<double>(
                "AutoAim.FireControl.AimOffset.yaw"
            );
            dbg.aim_offset_pitch = runtime_param::get_param<double>(
                "AutoAim.FireControl.AimOffset.pitch"
            );
            dbg.gate_min_confidence = runtime_param::get_param<double>(
                "AutoAim.FireControl.min_confidence"
            );
            dbg.gate_error_rate = runtime_param::get_param<double>(
                "AutoAim.FireControl.error_rate"
            );
            dbg.gate_allow_fire_ok = snapshot.self_state.allow_fire;
            dbg.gate_rotate_back_ok = true;

            if (mode == aimer::AimMode::AUTOAIM) {
                const auto& aim = controller.last_aim();
                const auto& armor_aim = controller.last_armor_aim();
                const auto& plan = controller.last_plan();

                dbg.valid = aim.valid;
                dbg.control_enabled = cmd.control_enabled;
                dbg.fire_now = cmd.fire_now;
                dbg.target_id = cmd.target_id;
                dbg.armor_idx = armor_aim.armor_idx;
                dbg.armor_id = armor_aim.armor_id;
                dbg.target_pos = armor_aim.target_pos;
                dbg.armor_aim_mode = static_cast<int>(armor_aim.mode);
                dbg.armor_time_to_fire = armor_aim.time_to_fire;
                dbg.selected_armor_z_to_v = armor_aim.z_to_v;

                dbg.aim_yaw = aim.yaw;
                dbg.aim_pitch = aim.pitch;
                dbg.aim_yaw_vel = plan.yaw_vel;
                dbg.aim_pitch_vel = plan.pitch_vel;
                dbg.cmd_yaw = cmd.yaw;
                dbg.cmd_pitch = cmd.pitch;

                dbg.distance = aim.distance;
                dbg.tracking_error = cmd.tracking_error;
                dbg.fly_time = aim.fly_time;
                dbg.timestamp = current_time;
                dbg.fail_stage = controller.last_fail_stage();
                dbg.gate_rotate_back_ok = controller.last_rotate_back_ok();
                dbg.rotate_back_active = controller.last_rotate_back_active();
                dbg.rotate_back_start_ms = controller.last_rotate_back_start() * 1000.0;
                dbg.rotate_back_end_ms = controller.last_rotate_back_end() * 1000.0;
                dbg.rotate_back_cmd_ms = controller.last_rotate_back_command_time() * 1000.0;
                const auto& gate = controller.last_gate_debug();
                dbg.gate_min_confidence = gate.tracking.min_confidence;
                dbg.gate_error_rate = gate.tracking.error_rate;
                dbg.gate_confidence = gate.tracking.confidence;
                dbg.gate_conf_ok = gate.tracking.conf_ok;
                dbg.gate_angle_ok = gate.tracking.angle_ok;
                dbg.gate_yaw_ok = gate.tracking.yaw_ok;
                dbg.gate_pitch_ok = gate.tracking.pitch_ok;
                dbg.gate_allow_fire_ok = gate.allow_fire_ok;
                dbg.gate_swing_ok = gate.swing_ok;
                dbg.gate_out_ok = gate.out_ok;
                dbg.gate_hit_offset_yaw = gate.tracking.hit_offset_yaw;
                dbg.gate_hit_offset_pitch = gate.tracking.hit_offset_pitch;
                dbg.gate_yaw_limit = gate.tracking.yaw_limit;
                dbg.gate_pitch_limit = gate.tracking.pitch_limit;
                dbg.gate_swing_offset_yaw = gate.swing_offset_yaw;
                dbg.gate_swing_offset_pitch = gate.swing_offset_pitch;
                dbg.gate_swing_yaw_limit = gate.swing_yaw_limit;
                dbg.gate_swing_pitch_limit = gate.swing_pitch_limit;
                dbg.gate_out_offset_yaw = gate.out_offset_yaw;
                dbg.gate_out_offset_pitch = gate.out_offset_pitch;
                dbg.gate_out_yaw_limit = gate.out_yaw_limit;
                dbg.gate_out_pitch_limit = gate.out_pitch_limit;

                if (snapshot.is_valid(cmd.target_id)) {
                    const auto& v = snapshot.vehicles[cmd.target_id];
                    dbg.spin_active = v.spin.active;
                    dbg.spin_level = static_cast<int>(v.spin.level);
                    dbg.spin_omega = v.spin.omega;
                    dbg.selected_armor_count = v.armor_count;
                    const double window_rad = get_spin_window_rad(v);
                    const double window_deg = aimer::math::rad2deg(window_rad);
                    if (v.spin.active) {
                        dbg.orientation_window_deg = window_deg;
                    } else {
                        // 非陀螺(ArmorModel)路径不使用 orientation window。
                        dbg.orientation_window_deg = 0.0;
                    }
                    // 与 ArmorAim::compute_spin 对齐：仅陀螺路径启用窗口。
                    dbg.orientation_window_on =
                        v.spin.active && std::abs(dbg.orientation_window_deg) > 1e-6;

                    if (armor_aim.armor_idx >= 0 && armor_aim.armor_idx < v.armor_count) {
                        const auto& armor = v.armors[armor_aim.armor_idx];
                        dbg.selected_armor_visible = armor.visible;
                    } else {
                        dbg.selected_armor_visible = false;
                    }
                }
            }
            fire_debug->store(dbg);
        }

        // 轻量调试日志:
        // - 周期性快照（低频）
        // - 选板切换/失败阶段切换
        // - 橙黄点角度突变（对应可视化“乱飘”）
        if (get_param_or("AutoAim.FireControl.Debug.enable", false)
            && mode == aimer::AimMode::AUTOAIM) {
                const auto& aim = controller.last_aim();
                const auto& armor_aim = controller.last_armor_aim();
                const auto& plan = controller.last_plan();
                const auto& gate = controller.last_gate_debug();
                const auto fail_stage = controller.last_fail_stage();
                const double log_period = std::max(
                    0.1, get_param_or("AutoAim.FireControl.Debug.period_s", 0.5)
                );
            const double jump_thresh = aimer::math::deg2rad(
                get_param_or("AutoAim.FireControl.Debug.jump_deg", 8.0)
            );

            bool target_switch = has_last_cmd
                && ((cmd.target_id != last_log_target_id)
                    || (armor_aim.armor_idx != last_log_armor_idx));
            bool fail_stage_switch = (fail_stage != last_fail_stage);
            bool periodic = (current_time - last_fc_log_ts) >= log_period;

            double cmd_jump_yaw = 0.0;
            double cmd_jump_pitch = 0.0;
            double aim_jump_yaw = 0.0;
            double aim_jump_pitch = 0.0;
            if (has_last_cmd) {
                cmd_jump_yaw = std::abs(aimer::math::angle_diff(cmd.yaw, last_cmd_yaw));
                cmd_jump_pitch = std::abs(cmd.pitch - last_cmd_pitch);
                aim_jump_yaw = std::abs(aimer::math::angle_diff(aim.yaw, last_aim_yaw));
                aim_jump_pitch = std::abs(aim.pitch - last_aim_pitch);
            }

                const bool jump_event = has_last_cmd
                    && (cmd_jump_yaw > jump_thresh
                        || cmd_jump_pitch > jump_thresh
                        || aim_jump_yaw > jump_thresh
                        || aim_jump_pitch > jump_thresh);

                // 橙黄差值诊断：验证 yellow(cmd) 是否等于 orange(aim)+前馈/偏置
                const double add_pred_t = get_param_or(
                    "AutoAim.FireControl.Cmd.additional_predict_time", 0.0
                );
                const double aim_offset_yaw_deg = get_param_or(
                    "AutoAim.FireControl.AimOffset.yaw", 0.0
                );
                const double aim_offset_pitch_deg = get_param_or(
                    "AutoAim.FireControl.AimOffset.pitch", 0.0
                );
                const double aim_offset_yaw = aimer::math::deg2rad(aim_offset_yaw_deg);
                const double aim_offset_pitch = aimer::math::deg2rad(aim_offset_pitch_deg);
                const double cmd_minus_aim_yaw = aimer::math::angle_diff(aim.yaw, cmd.yaw);
                const double cmd_minus_aim_pitch = cmd.pitch - aim.pitch;
                const double expected_cmd_minus_aim_yaw = add_pred_t * plan.yaw_vel + aim_offset_yaw;
                const double expected_cmd_minus_aim_pitch =
                    add_pred_t * plan.pitch_vel + aim_offset_pitch;

                bool spin_active = false;
                int spin_level = -1;
                double spin_omega = 0.0;
                double orientation_window_deg = 0.0;
                bool orientation_window_on = false;
                if (snapshot.is_valid(cmd.target_id)) {
                    const auto& v_dbg = snapshot.vehicles[cmd.target_id];
                    spin_active = v_dbg.spin.active;
                    spin_level = static_cast<int>(v_dbg.spin.level);
                    spin_omega = v_dbg.spin.omega;
                    if (spin_active) {
                        orientation_window_deg = aimer::math::rad2deg(get_spin_window_rad(v_dbg));
                    }
                    orientation_window_on = spin_active && std::abs(orientation_window_deg) > 1e-6;
                }

                if (periodic || target_switch || fail_stage_switch || jump_event) {
                std::string visible = "N/A";
                std::string vis_stat = "N/A";
                int vis_count = 0;
                int armor_count = 0;
                if (snapshot.is_valid(cmd.target_id)
                        && armor_aim.armor_idx >= 0
                    && armor_aim.armor_idx < snapshot.vehicles[cmd.target_id].armor_count) {
                    const auto& v_dbg = snapshot.vehicles[cmd.target_id];
                    armor_count = v_dbg.armor_count;
                    for (int vi = 0; vi < armor_count; ++vi) {
                        if (v_dbg.armors[vi].visible) {
                            ++vis_count;
                        }
                    }
                    vis_stat = fmt::format("{}/{}", vis_count, armor_count);
                    visible = snapshot.vehicles[cmd.target_id].armors[armor_aim.armor_idx].visible
                        ? "Y" : "N";
                }

                const auto pmode = [](autoaim::fire_control::AimMode m) {
                    switch (m) {
                        case autoaim::fire_control::AimMode::DIRECT: return "DIR";
                        case autoaim::fire_control::AimMode::INDIRECT: return "IND";
                        default: return "UNK";
                    }
                };

                const auto marker = jump_event ? "JUMP" :
                    (target_switch ? "SWITCH" :
                    (fail_stage_switch ? "FAIL" : "PERIOD"));
                const auto level = jump_event
                    ? debug::PrintMode::WARNING
                    : (target_switch || fail_stage_switch
                        ? debug::PrintMode::INFO
                        : debug::PrintMode::DEBUG);
                const int prev_tid = has_last_cmd ? last_log_target_id : -1;
                const int prev_aid = has_last_cmd ? last_log_armor_idx : -1;

                debug::print(
                    level,
                    "AutoAimFireControl",
                    "[{}] frame={} {}:{}->{}:{} id:{} mode={} vis={} vcnt={} "
                    "aim=({:.2f},{:.2f}) cmd=({:.2f},{:.2f}) "
                    "dCmdAim=({:+.2f},{:+.2f}) ff=({:+.2f},{:+.2f}) add={:.0f}ms "
                    "dA=({:.2f},{:.2f}) dC=({:.2f},{:.2f}) "
                    "pred_dt={:.1f}ms lat={:.1f}ms fail={} fire={} "
                    "gate=[{}{}{}{}{}{}{}{}] z2v={:.1f}deg ttf={:.1f}ms "
                    "spin=[{} L{} w={:.2f} win={:.1f}{}]",
                    marker,
                    snapshot.frame_id,
                    prev_tid,
                    prev_aid,
                    cmd.target_id,
                    armor_aim.armor_idx,
                    armor_aim.armor_id,
                    pmode(armor_aim.mode),
                    visible,
                    vis_stat,
                    aimer::math::rad2deg(aim.yaw),
                    aimer::math::rad2deg(aim.pitch),
                    aimer::math::rad2deg(cmd.yaw),
                    aimer::math::rad2deg(cmd.pitch),
                    aimer::math::rad2deg(cmd_minus_aim_yaw),
                    aimer::math::rad2deg(cmd_minus_aim_pitch),
                    aimer::math::rad2deg(expected_cmd_minus_aim_yaw),
                    aimer::math::rad2deg(expected_cmd_minus_aim_pitch),
                    add_pred_t * 1000.0,
                    aimer::math::rad2deg(aim_jump_yaw),
                    aimer::math::rad2deg(aim_jump_pitch),
                    aimer::math::rad2deg(cmd_jump_yaw),
                    aimer::math::rad2deg(cmd_jump_pitch),
                    controller.last_prediction_dt() * 1000.0,
                    latency.prediction_latency() * 1000.0,
                    ::fire_control::FireDebugInfo::fail_stage_name(fail_stage),
                    cmd.fire_now ? 1 : 0,
                    gate.tracking.conf_ok ? "+" : "-",
                    gate.tracking.angle_ok ? "+" : "-",
                    gate.tracking.yaw_ok ? "+" : "-",
                    gate.tracking.pitch_ok ? "+" : "-",
                    gate.swing_ok ? "+" : "-",
                    gate.out_ok ? "+" : "-",
                    gate.allow_fire_ok ? "+" : "-",
                    controller.last_rotate_back_ok() ? "+" : "-",
                    aimer::math::rad2deg(armor_aim.z_to_v),
                    armor_aim.time_to_fire * 1000.0,
                    spin_active ? "A" : "-",
                    spin_level,
                    spin_omega,
                    orientation_window_deg,
                    orientation_window_on ? "Y" : "N"
                );
                last_fc_log_ts = current_time;
            }

            has_last_cmd = true;
            last_log_target_id = cmd.target_id;
            last_log_armor_idx = armor_aim.armor_idx;
            last_cmd_yaw = cmd.yaw;
            last_cmd_pitch = cmd.pitch;
            last_aim_yaw = aim.yaw;
            last_aim_pitch = aim.pitch;
            last_fail_stage = fail_stage;
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
    fire_control_run(config_path);
}

}  // namespace autoaim::fire_control
