/**
 * @file visualizer_node.cpp
 * @brief 统一可视化线程
 *
 * 所有 imshow 集中在此线程，支持视图切换:
 *   - "predictor": predictor_debug.image + 战场面板 + 延迟面板
 *   - "firecontrol": predictor_debug.image + 火控 OSD + 目标几何 + 像素标记
 *   - "detector": detector_debug_img 直显
 *
 * 数据源:
 *   BasicObjManager<BattlefieldSnapshot> "battlefield"
 *   BasicObjManager<PredictorDebugFrame> "predictor_debug"
 *   BasicObjManager<FireDebugInfo> "fire_debug"
 *   BasicObjManager<cv::Mat> "detector_debug_img"
 */

#include "visualizer_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>
#include <variant>

#include <fmt/format.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "aimer/auto_aim/common/types.hpp"
#include "aimer/auto_aim/predictor/types.hpp"
#include "aimer/common/fire_control_types.hpp"
#include "aimer/common/math/math.hpp"
#include "aimer/common/robot_state.hpp"
#include "aimer/common/transformer/transformer.hpp"
#include "plugin/debug/logger.hpp"
#include "plugin/param/runtime_parameter.hpp"
#include "umt/umt.hpp"

namespace visualizer {

using SteadyClock = std::chrono::steady_clock;
using autoaim::predictor::BattlefieldSnapshot;
using autoaim::predictor::PredictorDebugFrame;
using autoaim::predictor::SpinLevel;
using autoaim::predictor::TargetState;
using autoaim::predictor::MAX_TARGETS;
using fire_control::FireDebugInfo;

// ==================== 像素投影辅助 ====================

/**
 * @brief 将世界坐标系角度投影到像素 (用于绘制瞄准标记)
 *
 * @param yaw   世界坐标系 yaw (rad)
 * @param pitch 世界坐标系 pitch (rad)
 * @param q_imu IMU 四元数
 * @param valid 输出: 是否在画面内
 * @param ref_dist 参考距离 (投影用，不影响像素方向)
 */
static cv::Point2f angle_to_pixel(
    double yaw, double pitch,
    const Eigen::Quaterniond& q_imu,
    bool& valid,
    double ref_dist = 5.0
) {
    Eigen::Vector3d p_world = aimer::math::ypd_to_xyz(
        aimer::math::YpdCoord{yaw, pitch, ref_dist}
    );
    return aimer::tf::world_to_pixel(p_world, q_imu, valid);
}

static double camera_intrinsic_at(const cv::Mat& K, int r, int c) {
    if (K.type() == CV_32F || K.type() == CV_32FC1) {
        return static_cast<double>(K.at<float>(r, c));
    }
    return K.at<double>(r, c);
}

/**
 * @brief 拷贝到可复用缓冲区，避免每帧 clone 触发重复分配
 */
static cv::Mat copy_to_reused_buffer(const cv::Mat& src, cv::Mat& buffer) {
    if (buffer.empty()
        || buffer.rows != src.rows
        || buffer.cols != src.cols
        || buffer.type() != src.type())
    {
        buffer.create(src.size(), src.type());
    }
    src.copyTo(buffer);
    return buffer;
}

static int choose_debug_target_id(
    const BattlefieldSnapshot& snapshot,
    const FireDebugInfo& dbg
) {
    if (dbg.target_id >= 0 && snapshot.is_valid(dbg.target_id)) {
        return dbg.target_id;
    }
    if (snapshot.primary_target_id >= 0 && snapshot.is_valid(snapshot.primary_target_id)) {
        return snapshot.primary_target_id;
    }
    return -1;
}

static int resolve_selected_armor_idx(
    const TargetState& v,
    const FireDebugInfo& dbg
) {
    if (dbg.armor_id >= 0) {
        for (int i = 0; i < v.armor_count; ++i) {
            if (v.armor_id(i) == dbg.armor_id) {
                return i;
            }
        }
    }
    if (dbg.armor_idx >= 0 && dbg.armor_idx < v.armor_count) {
        return dbg.armor_idx;
    }
    return -1;
}

static double center_cost_deg(
    const Eigen::Vector3d& pos,
    const FireDebugInfo& dbg
) {
    const double yaw = std::atan2(pos.y(), pos.x());
    const double pitch = std::atan2(pos.z(), std::hypot(pos.x(), pos.y()));
    const double dyaw = ::fire_control::GimbalState::normalize_angle(yaw - dbg.gimbal_yaw);
    const double dpitch = pitch - dbg.gimbal_pitch;
    return std::hypot(dyaw, dpitch) * 57.3;
}

static double predicted_armor_z_to_v(
    const TargetState& v,
    int armor_idx,
    double predict_dt
) {
    return v.predicted_z_to_v(armor_idx, predict_dt);
}

static const char* armor_aim_mode_name(int mode) {
    return mode == 1 ? "INDIRECT" : "DIRECT";
}

// ==================== OSD 绘制函数 ====================

/**
 * @brief 左上角火控诊断面板
 */
static void draw_fire_debug_panel(
    cv::Mat& vis,
    const BattlefieldSnapshot& snapshot,
    const FireDebugInfo& dbg
) {
    int lh = 15;
    int tx = 8;
    int ty = 16;
    auto put = [&](const std::string& text, cv::Scalar color = {200, 200, 200}) {
        cv::putText(vis, text, {tx, ty}, cv::FONT_HERSHEY_SIMPLEX,
            0.38, color, 1, cv::LINE_AA);
        ty += lh;
    };

    // 半透明背景
    int panel_h = 520;
    int panel_w = 560;
    int safe_w = std::min(panel_w, vis.cols);
    int safe_h = std::min(panel_h, vis.rows);
    if (safe_w > 0 && safe_h > 0) {
        cv::Mat bg = vis(cv::Rect(0, 0, safe_w, safe_h));
        bg = bg * 0.3;
    }

    // 1. FC 状态 + 模式
    double age = 0;
    if (dbg.fc_heartbeat > 0) {
        auto now = std::chrono::steady_clock::now();
        double now_s = std::chrono::duration<double>(now.time_since_epoch()).count();
        age = now_s - dbg.fc_heartbeat;
    }
    bool alive = (dbg.fc_heartbeat > 0 && age < 0.5);
    const char* mode_name = aimer::aim_mode_name(aimer::to_aim_mode(dbg.fc_mode));
    put(fmt::format("FC: {} | {}", alive ? "ALIVE" : "DEAD", mode_name),
        alive ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255));

    // 2. Stage + Target + Armor + Control
    cv::Scalar stage_color = (dbg.fail_stage == 9) ? cv::Scalar(0, 255, 0)
                           : (dbg.fail_stage == 0) ? cv::Scalar(150, 150, 150)
                                                   : cv::Scalar(0, 128, 255);
    put(fmt::format("Stage: {} | ctrl:{} fire:{} | Tgt:{} Arm:id={} (idx:{})",
        FireDebugInfo::fail_stage_name(dbg.fail_stage),
        dbg.control_enabled ? "ON" : "OFF",
        dbg.fire_now ? "NOW" : "HOLD",
        dbg.target_id, dbg.armor_id, dbg.armor_idx), stage_color);

    // 3. Bullet speed + snapshot info
    put(fmt::format("BS: {:.1f}m/s | valid: 0x{:03x} pri: {} frm: {}",
        dbg.bullet_speed, dbg.snapshot_valid_mask,
        dbg.snapshot_primary_id, dbg.snapshot_frame_id));
    put(fmt::format("Snapshot age: {:.1f}ms | pred_dt: {:.1f}ms",
        dbg.snapshot_age_ms, dbg.prediction_dt * 1000.0), {170, 200, 255});

    // 4. Gimbal 角度 + 角速度
    put(fmt::format("Gimbal: yaw={:+.1f} pitch={:+.1f} deg",
        dbg.gimbal_yaw * 57.3, dbg.gimbal_pitch * 57.3), {180, 220, 255});
    put(fmt::format("  vel: yaw={:+.0f} pitch={:+.0f} deg/s",
        dbg.gimbal_yaw_vel * 57.3, dbg.gimbal_pitch_vel * 57.3), {150, 180, 220});

    // 5. Latency 分解
    put(fmt::format("LatP: img{:.0f} +pred{:.0f} +send{:.0f} +fly{:.0f} ={:.0f}ms",
        dbg.latency_img_to_predict,
        dbg.latency_predict_to_send,
        dbg.latency_send_to_control,
        dbg.latency_fire_to_hit,
        dbg.latency_total),
        cv::Scalar(100, 200, 255));
    put(fmt::format("LatH: +ctrl{:.0f} => {:.0f}ms",
        dbg.latency_control_to_fire,
        dbg.latency_hit_total),
        cv::Scalar(120, 220, 255));

    // 6. Dist + FlyTime
    put(fmt::format("Dist: {:.2f}m  FlyT: {:.1f}ms",
        dbg.distance, dbg.fly_time * 1000), {200, 200, 200});

    // 7. Aim / Cmd 角度
    put(fmt::format("Aim:  y={:+.2f} p={:+.2f} deg",
        dbg.aim_yaw * 57.3, dbg.aim_pitch * 57.3), {0, 165, 255});  // 橙色
    put(fmt::format("AimV: y={:+.1f} p={:+.1f} deg/s",
        dbg.aim_yaw_vel * 57.3, dbg.aim_pitch_vel * 57.3), {60, 190, 255});
    put(fmt::format("Cmd:  y={:+.2f} p={:+.2f} deg",
        dbg.cmd_yaw * 57.3, dbg.cmd_pitch * 57.3), {0, 255, 255});  // 黄色

    // 8. Error + FIRE 状态 (tracking_error 单位是米: 落点偏移距离)
    cv::Scalar fire_color = dbg.fire_now ? cv::Scalar(0, 0, 255) : cv::Scalar(100, 200, 255);
    put(fmt::format("Err: {:.0f}cm  {}",
        dbg.tracking_error * 100, dbg.fire_now ? ">>> FIRE <<<" : "HOLD"),
        fire_color);

    // 9. 选板路径与窗口状态
    put(fmt::format("AimMode: {}  t_fire={:.1f}ms  armor_vis:{}",
        armor_aim_mode_name(dbg.armor_aim_mode),
        dbg.armor_time_to_fire * 1000.0,
        dbg.selected_armor_visible ? "Y" : "N"), cv::Scalar(80, 220, 255));
    put(fmt::format("Spin: active={} lvl={} w={:+.1f}d/s  win:{}({:.1f}deg)",
        dbg.spin_active ? 1 : 0, dbg.spin_level, dbg.spin_omega * 57.3,
        dbg.orientation_window_on ? "ON" : "OFF", dbg.orientation_window_deg),
        cv::Scalar(180, 210, 255));
    put(fmt::format("Selected z_to_v={:+.1f}deg  armor_cnt={}",
        dbg.selected_armor_z_to_v * 57.3, dbg.selected_armor_count), cv::Scalar(170, 210, 255));

    // 10. 指令注入参数
    put(fmt::format("Cmd inject: add_pred={:.1f}ms  off(y,p)=({:+.2f},{:+.2f})deg",
        dbg.cmd_additional_predict_time * 1000.0,
        dbg.aim_offset_yaw, dbg.aim_offset_pitch),
        cv::Scalar(140, 220, 220));

    // 11. 开火门控分解（真实火控判定字段）
    const int tid = choose_debug_target_id(snapshot, dbg);
    if (tid >= 0 && snapshot.is_valid(tid)) {
        put(fmt::format("Gate conf: {:.2f}/{:.2f} [{}]",
            dbg.gate_confidence, dbg.gate_min_confidence, dbg.gate_conf_ok ? "OK" : "BLOCK"),
            dbg.gate_conf_ok ? cv::Scalar(80, 220, 80) : cv::Scalar(0, 0, 255));

        put(fmt::format("Gate angle: {}", dbg.gate_angle_ok ? "OK" : "BLOCK"),
            dbg.gate_angle_ok ? cv::Scalar(80, 220, 80) : cv::Scalar(0, 0, 255));

        put(fmt::format("Gate yaw: {:.1f}/{:.1f}cm [{}]",
            dbg.gate_hit_offset_yaw * 100.0, dbg.gate_yaw_limit * 100.0,
            dbg.gate_yaw_ok ? "OK" : "BLOCK"),
            dbg.gate_yaw_ok ? cv::Scalar(80, 220, 80) : cv::Scalar(0, 0, 255));
        put(fmt::format("Gate pit: {:.1f}/{:.1f}cm [{}]",
            dbg.gate_hit_offset_pitch * 100.0, dbg.gate_pitch_limit * 100.0,
            dbg.gate_pitch_ok ? "OK" : "BLOCK"),
            dbg.gate_pitch_ok ? cv::Scalar(80, 220, 80) : cv::Scalar(0, 0, 255));
        put(fmt::format("Gate swing(y): {:.1f}/{:.1f}cm [{}]",
            dbg.gate_swing_offset_yaw * 100.0, dbg.gate_swing_yaw_limit * 100.0,
            dbg.gate_swing_ok ? "OK" : "BLOCK"),
            dbg.gate_swing_ok ? cv::Scalar(80, 220, 80) : cv::Scalar(0, 0, 255));
        put(fmt::format("Gate out(y): {:.1f}/{:.1f}cm [{}]",
            dbg.gate_out_offset_yaw * 100.0, dbg.gate_out_yaw_limit * 100.0,
            dbg.gate_out_ok ? "OK" : "BLOCK"),
            dbg.gate_out_ok ? cv::Scalar(80, 220, 80) : cv::Scalar(0, 0, 255));

        put(fmt::format("Gate allow_fire: {}",
            dbg.gate_allow_fire_ok ? "OK" : "BLOCK"),
            dbg.gate_allow_fire_ok ? cv::Scalar(80, 220, 80) : cv::Scalar(0, 0, 255));
        if (dbg.rotate_back_active) {
            put(fmt::format("Gate rotate_back: {} | cmd={:.1f}ms in [{:.1f},{:.1f}]",
                dbg.gate_rotate_back_ok ? "OK" : "BLOCK",
                dbg.rotate_back_cmd_ms,
                dbg.rotate_back_start_ms,
                dbg.rotate_back_end_ms),
                dbg.gate_rotate_back_ok ? cv::Scalar(80, 220, 80) : cv::Scalar(0, 0, 255));
        } else {
            put(fmt::format("Gate rotate_back: {} | inactive",
                dbg.gate_rotate_back_ok ? "OK" : "BLOCK"),
                dbg.gate_rotate_back_ok ? cv::Scalar(80, 220, 80) : cv::Scalar(0, 0, 255));
        }

        put(fmt::format("Gate sum: C{} A{} Y{} P{} S{} O{} F{} R{}",
            dbg.gate_conf_ok ? "+" : "-",
            dbg.gate_angle_ok ? "+" : "-",
            dbg.gate_yaw_ok ? "+" : "-",
            dbg.gate_pitch_ok ? "+" : "-",
            dbg.gate_swing_ok ? "+" : "-",
            dbg.gate_out_ok ? "+" : "-",
            dbg.gate_allow_fire_ok ? "+" : "-",
            dbg.gate_rotate_back_ok ? "+" : "-"),
            (dbg.gate_conf_ok && dbg.gate_angle_ok && dbg.gate_yaw_ok
             && dbg.gate_pitch_ok && dbg.gate_swing_ok && dbg.gate_out_ok
             && dbg.gate_allow_fire_ok && dbg.gate_rotate_back_ok)
                ? cv::Scalar(80, 220, 80)
                : cv::Scalar(0, 0, 255));
    }
}

/**
 * @brief 右下角战场信息面板
 */
static void draw_battlefield_panel(cv::Mat& vis, const BattlefieldSnapshot& snapshot) {
    // 收集有效目标数
    int n_tracked = 0;
    snapshot.for_each_valid([&](int, const TargetState&) { n_tracked++; });

    int lh = 15;
    int panel_h = 20 + n_tracked * 2 * lh + (n_tracked > 0 ? 4 : 0);
    int panel_w = 360;

    // 右下角定位
    int panel_x = vis.cols - panel_w - 4;
    int panel_y = vis.rows - panel_h - 4;
    if (panel_x < 0) panel_x = 0;
    if (panel_y < 0) panel_y = 0;

    int safe_w = std::min(panel_w, vis.cols - panel_x);
    int safe_h = std::min(panel_h, vis.rows - panel_y);
    if (safe_w > 0 && safe_h > 0) {
        cv::Mat bg = vis(cv::Rect(panel_x, panel_y, safe_w, safe_h));
        bg = bg * 0.3;
    }

    int tx = panel_x + 8;
    int ty = panel_y + 16;
    auto put = [&](const std::string& text, cv::Scalar color = {200, 200, 200}) {
        cv::putText(vis, text, {tx, ty}, cv::FONT_HERSHEY_SIMPLEX,
            0.38, color, 1, cv::LINE_AA);
        ty += lh;
    };

    put(fmt::format("Battlefield  {} tracked  bs={:.1f}m/s",
        n_tracked, snapshot.self_state.bullet_speed), {255, 255, 255});

    snapshot.for_each_valid([&](int id, const TargetState& v) {
        bool is_pri = (id == snapshot.primary_target_id);
        cv::Scalar color = is_pri ? cv::Scalar(0, 255, 255) : cv::Scalar(180, 180, 180);

        std::string type_str = armor_number_to_string(v.enemy_type);
        if (type_str != "sentry" && type_str != "outpost" && type_str != "base") {
            type_str = "Inf" + type_str;
        }

        // 可见装甲板
        std::string arm_str;
        for (int a = 0; a < v.armor_count; ++a) {
            if (v.armor_visible(a)) {
                if (!arm_str.empty()) arm_str += ",";
                arm_str += std::to_string(a);
            }
        }
        if (arm_str.empty()) arm_str = "-";

        const char* spin_label = "NONE";
        if (v.spin.level == SpinLevel::LOW) spin_label = "LOW";
        else if (v.spin.level == SpinLevel::HIGH) spin_label = "HIGH";

        put(fmt::format("{}#{} {}  {:.1f}m  arm[{}]{}",
            is_pri ? ">" : " ", id, type_str, v.position.norm(),
            arm_str, is_pri ? " [PRI]" : ""), color);

        put(fmt::format("  spin:{} w={:+.0f}d/s r={:.2f}m  v=({:.1f},{:.1f})",
            spin_label, v.spin.omega * 57.3, v.spin.radius,
            v.velocity.x(), v.velocity.y()), {150, 150, 150});
    });
}

/**
 * @brief 右上角选中目标逐装甲板面板
 */
static void draw_selected_target_panel(
    cv::Mat& vis,
    const BattlefieldSnapshot& snapshot,
    const FireDebugInfo& dbg
) {
    const int tid = choose_debug_target_id(snapshot, dbg);
    if (tid < 0 || !snapshot.is_valid(tid)) {
        return;
    }

    const auto* target = snapshot.find_target(tid);
    if (target == nullptr) {
        return;
    }
    const auto& v = *target;
    const double pred_dt = std::max(0.0, dbg.prediction_dt);
    auto get_param_or = [](const std::string& name, double default_value) {
        auto ptr = runtime_param::find_param(name);
        if (ptr != nullptr) {
            if (auto* val = std::get_if<double>(&*ptr)) {
                return *val;
            }
        }
        return default_value;
    };
    const double top0_window_deg = (v.armor_count == 4)
        ? get_param_or("AutoAim.FireControl.PID.top0_max_orientation_angle_armors4", 58.8888)
        : get_param_or("AutoAim.FireControl.PID.top0_max_orientation_angle_armors_other", 75.0);
    const double top1_window_deg = get_param_or(
        "AutoAim.FireControl.PID.top1_max_orientation_angle", 0.0
    );
    const double top2_window_deg = get_param_or(
        "AutoAim.FireControl.PID.top2_max_orientation_angle", 0.0
    );
    double max_angle_deg = 0.0;
    if (v.spin.active) {
        if (v.spin.level == SpinLevel::LOW) {
            max_angle_deg = top1_window_deg;
        } else if (v.spin.level == SpinLevel::HIGH) {
            max_angle_deg = top2_window_deg;
        } else {
            max_angle_deg = top0_window_deg;
        }
    }
    const double max_angle = max_angle_deg * M_PI / 180.0;
    const bool use_window = v.spin.active && max_angle >= 0.0;

    const int lh = 15;
    const int rows = 4 + std::max(0, v.armor_count);
    const int panel_h = 12 + rows * lh + 6;
    const int panel_w = 510;
    const int panel_x = std::max(0, vis.cols - panel_w - 6);
    const int panel_y = 6;
    const int safe_w = std::min(panel_w, vis.cols - panel_x);
    const int safe_h = std::min(panel_h, vis.rows - panel_y);
    if (safe_w <= 0 || safe_h <= 0) {
        return;
    }

    cv::Mat bg = vis(cv::Rect(panel_x, panel_y, safe_w, safe_h));
    bg = bg * 0.3;

    int tx = panel_x + 8;
    int ty = panel_y + 16;
    auto put = [&](const std::string& text, cv::Scalar color = {220, 220, 220}) {
        cv::putText(vis, text, {tx, ty}, cv::FONT_HERSHEY_SIMPLEX,
            0.38, color, 1, cv::LINE_AA);
        ty += lh;
    };

    const char* spin_label = "NONE";
    if (v.spin.level == SpinLevel::LOW) spin_label = "LOW";
    else if (v.spin.level == SpinLevel::HIGH) spin_label = "HIGH";
    const int sel_idx = resolve_selected_armor_idx(v, dbg);
    put(fmt::format("Selected T{}  pri={}  arm_sel=id:{} (idx:{})  conf={:.2f}",
        tid, snapshot.primary_target_id, dbg.armor_id, sel_idx, v.confidence),
        cv::Scalar(0, 255, 255));
    put(fmt::format("AimMode={}  pred_dt={:.1f}ms  t_fire={:.1f}ms",
        armor_aim_mode_name(dbg.armor_aim_mode), pred_dt * 1000.0, dbg.armor_time_to_fire * 1000.0),
        cv::Scalar(100, 220, 255));
    put(fmt::format("spin:{} active={} w={:+.1f}d/s  center={:.2f}m",
        spin_label, v.spin.active ? 1 : 0, v.spin.omega * 57.3, v.position.norm()), cv::Scalar(180, 180, 255));
    put(fmt::format("ori_window:{}  max={:.1f}deg  frame={} mask=0x{:03x}",
        use_window ? "ON" : "OFF", max_angle * 57.3, snapshot.frame_id, snapshot.valid_mask),
        cv::Scalar(160, 220, 255));
    put("A#: flags(* sel, V vis, W in-win) | z_to_v(pred) | center_cost | score | dist",
        cv::Scalar(140, 140, 140));

    for (int i = 0; i < v.armor_count; ++i) {
        const Eigen::Vector3d pos = v.predict_armor_position(i, pred_dt);
        const double z_to_v = predicted_armor_z_to_v(v, i, pred_dt);
        const bool in_window = (!use_window) || (std::abs(z_to_v) <= max_angle);
        const double cc_deg = center_cost_deg(pos, dbg);
        const double dist = pos.norm();
        const bool selected = (i == sel_idx);

        cv::Scalar color = selected ? cv::Scalar(0, 80, 255)
                       : (v.armor_visible(i) ? cv::Scalar(120, 220, 120)
                                             : cv::Scalar(150, 150, 150));
        put(fmt::format("A{}(id={}): {}{}{}  z={:+5.1f}deg  c={:5.1f}deg  s={:.2f}  d={:.2f}m",
            i,
            v.armor_id(i),
            selected ? "*" : " ",
            v.armor_visible(i) ? "V" : "-",
            in_window ? "W" : "-",
            z_to_v * 57.3, cc_deg, v.armor_score(i), dist),
            color);
    }
}

/**
 * @brief 左下角延迟信息面板 (pipeline + 条形图)
 */
static void draw_latency_panel(cv::Mat& vis, const BattlefieldSnapshot& snapshot,
                                const PredictorDebugFrame& predictor_dbg,
                                const FireDebugInfo& dbg) {
    int panel_x = 10;
    int panel_y = vis.rows - 110;
    int panel_w = 230;
    int panel_h = 100;

    if (panel_y < 0 || panel_x + panel_w > vis.cols || panel_y + panel_h > vis.rows) return;

    cv::Mat roi = vis(cv::Rect(panel_x, panel_y, panel_w, panel_h));
    roi = roi * 0.4;

    int text_x = panel_x + 8;
    int text_y = panel_y + 18;
    int line_h = 16;
    auto draw_line = [&](const std::string& label, float ms, cv::Scalar color) {
        std::string text = fmt::format("{}: {:.1f}ms", label, ms);
        cv::putText(vis, text, cv::Point(text_x, text_y),
            cv::FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv::LINE_AA);
        text_y += line_h;
    };

    // Pipeline 延迟 (来自 snapshot)
    auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(
        SteadyClock::now().time_since_epoch()).count();
    float total_pipeline_ms = (now_us - snapshot.self_state.timestamp_us) / 1000.0f;

    draw_line("Detect", predictor_dbg.detect_latency_ms, cv::Scalar(100, 200, 255));
    draw_line("Predict", predictor_dbg.predict_latency_ms, cv::Scalar(100, 255, 200));
    draw_line("Pipeline", total_pipeline_ms, cv::Scalar(0, 255, 255));

    // 火控延迟分解 (来自 FireDebugInfo)
    if (dbg.fc_heartbeat > 0) {
        draw_line("FC pred", static_cast<float>(dbg.latency_total),
            cv::Scalar(255, 200, 100));
        draw_line("FC hit", static_cast<float>(dbg.latency_hit_total),
            cv::Scalar(120, 220, 255));
    }

    // 堆叠条形图
    int bar_y = text_y + 5;
    int bar_h = 10;
    float max_ms = std::max(50.0f, total_pipeline_ms);
    float scale = (panel_w - 20) / max_ms;

    cv::rectangle(vis,
        cv::Point(text_x, bar_y),
        cv::Point(text_x + static_cast<int>(max_ms * scale), bar_y + bar_h),
        cv::Scalar(50, 50, 50), -1);

    int detect_w = static_cast<int>(predictor_dbg.detect_latency_ms * scale);
    cv::rectangle(vis,
        cv::Point(text_x, bar_y),
        cv::Point(text_x + detect_w, bar_y + bar_h),
        cv::Scalar(100, 150, 255), -1);

    int predict_w = static_cast<int>(predictor_dbg.predict_latency_ms * scale);
    cv::rectangle(vis,
        cv::Point(text_x + detect_w, bar_y),
        cv::Point(text_x + detect_w + predict_w, bar_y + bar_h),
        cv::Scalar(100, 255, 150), -1);
}

/**
 * @brief 像素标记 (与 rm.cv.fans 绘制语义对齐)
 *
 * 白色空心圆: 图像中心
 * 绿色空心圆: 相机光轴
 * 蓝色空心圆: 当前枪管前向点 (gimbal_yaw / gimbal_pitch)
 * 橙色空心圆: aim 点 (debug_aim)
 * 黄色空心圆: cmd 点 (debug_cmd 反算回角度)
 * 红色箭头:   从橙点出发，方向由 aim 角速度 (aim_yaw_vel / aim_pitch_vel) 决定
 * 红色文字:   SHOOT_CMD
 */
static void draw_pixel_markers(
    cv::Mat& vis,
    const FireDebugInfo& dbg,
    const Eigen::Quaterniond& q_imu,
    bool frame_aligned
) {
    int cx = vis.cols / 2;
    int cy = vis.rows / 2;

    // 白色空心圆: 图像中心
    cv::circle(vis, cv::Point(cx, cy), 6, cv::Scalar(220, 220, 220), 3, cv::LINE_AA);

    // 绿色空心圆: 相机光轴 (内参主点, 固定位置)
    const cv::Mat& K = aimer::tf::get_camera_matrix();
    if (!K.empty() && K.rows >= 3 && K.cols >= 3) {
        int opt_cx = static_cast<int>(camera_intrinsic_at(K, 0, 2));
        int opt_cy = static_cast<int>(camera_intrinsic_at(K, 1, 2));
        cv::circle(vis, cv::Point(opt_cx, opt_cy), 6, cv::Scalar(0, 180, 0), 3, cv::LINE_AA);
    }

    if (!frame_aligned) {
        cv::putText(
            vis,
            "WARN: FC/Predictor frame mismatch",
            {20, std::max(30, vis.rows - 20)},
            cv::FONT_HERSHEY_SIMPLEX,
            0.5,
            cv::Scalar(0, 0, 255),
            1,
            cv::LINE_AA
        );
        return;
    }

    if (dbg.fail_stage != 9) return;  // 以下仅在瞄准有效时绘制

    double ref_dist = dbg.distance > 0 ? dbg.distance : 5.0;

    // 蓝色空心圆: 当前枪管前向点 (gimbal_yaw/pitch)
    bool gimbal_valid = false;
    cv::Point2f gimbal_px = angle_to_pixel(
        dbg.gimbal_yaw, dbg.gimbal_pitch, q_imu, gimbal_valid, ref_dist
    );
    if (gimbal_valid) {
        cv::circle(vis, gimbal_px, 10, cv::Scalar(255, 120, 40), 3, cv::LINE_AA);
        cv::putText(vis, "G", gimbal_px + cv::Point2f(8, -8),
            cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 120, 40), 1, cv::LINE_AA);
    }

    // 橙色空心圆: 预测瞄准点 (aim_yaw/pitch)
    bool aim_valid = false;
    cv::Point2f aim_px = angle_to_pixel(
        dbg.aim_yaw, dbg.aim_pitch, q_imu, aim_valid, ref_dist
    );
    if (aim_valid) {
        cv::circle(vis, aim_px, 10, cv::Scalar(0, 105, 255), 4, cv::LINE_AA);
        cv::putText(vis, "A", aim_px + cv::Point2f(8, -8),
            cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 105, 255), 1, cv::LINE_AA);
    }

    // 黄色空心圆: 实际发送角 (cmd_yaw/pitch, 含延迟补偿)
    bool cmd_valid = false;
    cv::Point2f cmd_px = angle_to_pixel(
        dbg.cmd_yaw, dbg.cmd_pitch, q_imu, cmd_valid, ref_dist
    );
    if (cmd_valid) {
        cv::circle(vis, cmd_px, 10, cv::Scalar(0, 255, 255), 4, cv::LINE_AA);
        cv::putText(vis, "C", cmd_px + cv::Point2f(8, -8),
            cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
    }

    // A/C/G 三点距离 (仅调试语义，不参与控制)
    int py = 72;
    if (gimbal_valid && aim_valid) {
        cv::putText(vis, fmt::format("d(G,A)={:.1f}px", cv::norm(gimbal_px - aim_px)),
            cv::Point(vis.cols - 260, py), cv::FONT_HERSHEY_SIMPLEX,
            0.45, cv::Scalar(255, 170, 90), 1, cv::LINE_AA);
        py += 16;
    }
    if (gimbal_valid && cmd_valid) {
        cv::putText(vis, fmt::format("d(G,C)={:.1f}px", cv::norm(gimbal_px - cmd_px)),
            cv::Point(vis.cols - 260, py), cv::FONT_HERSHEY_SIMPLEX,
            0.45, cv::Scalar(255, 230, 90), 1, cv::LINE_AA);
        py += 16;
    }
    if (aim_valid && cmd_valid) {
        cv::putText(vis, fmt::format("d(A,C)={:.1f}px", cv::norm(aim_px - cmd_px)),
            cv::Point(vis.cols - 260, py), cv::FONT_HERSHEY_SIMPLEX,
            0.45, cv::Scalar(120, 210, 255), 1, cv::LINE_AA);
    }

    // 红色箭头: 用“角度前向一步后的像素位置”减“当前像素位置”得到方向。
    // 这样可自动适配本项目 yaw(左正)/pitch(上正) 约定，避免与像素坐标(y下正)符号冲突。
    if (aim_valid) {
        constexpr double arrow_dt = 0.06;  // 仅可视化缩放，不影响控制
        bool next_valid = false;
        cv::Point2f next_px = angle_to_pixel(
            dbg.aim_yaw + dbg.aim_yaw_vel * arrow_dt,
            dbg.aim_pitch + dbg.aim_pitch_vel * arrow_dt,
            q_imu,
            next_valid,
            ref_dist
        );
        if (next_valid) {
            cv::Point2f d = next_px - aim_px;
            const float v_norm = std::hypot(d.x, d.y);
            if (v_norm > 2.0f) {
                cv::arrowedLine(
                    vis,
                    aim_px,
                    next_px,
                    cv::Scalar(0, 0, 255),
                    2,
                    cv::LINE_AA,
                    0,
                    0.15
                );
            }
        }
    }

    // SHOOT_CMD 状态文字
    if (dbg.fire_now) {
        cv::putText(vis, "SHOOT_CMD", cv::Point(vis.cols - 300, 50),
            cv::FONT_HERSHEY_TRIPLEX, 1.5, cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
    }
}

/**
 * @brief 叠加选中目标的中心/速度/逐装甲板像素标记
 */
static void draw_target_geometry_markers(
    cv::Mat& vis,
    const BattlefieldSnapshot& snapshot,
    const FireDebugInfo& dbg,
    const Eigen::Quaterniond& q_imu
) {
    const int tid = choose_debug_target_id(snapshot, dbg);
    if (tid < 0 || !snapshot.is_valid(tid)) {
        return;
    }
    const auto* target = snapshot.find_target(tid);
    if (target == nullptr) {
        return;
    }
    const auto& v = *target;
    const double pred_dt = std::max(0.0, dbg.prediction_dt);
    const Eigen::Vector3d center_world = v.predict_center(pred_dt);

    bool center_valid = false;
    const cv::Point2f center_px = aimer::tf::world_to_pixel(
        center_world, q_imu, center_valid
    );
    if (center_valid) {
        cv::drawMarker(vis, center_px, cv::Scalar(255, 0, 255),
            cv::MARKER_TILTED_CROSS, 16, 2, cv::LINE_AA);

        Eigen::Vector3d tip_world = center_world + v.velocity * 0.15;
        bool tip_valid = false;
        const cv::Point2f tip_px = aimer::tf::world_to_pixel(tip_world, q_imu, tip_valid);
        if (tip_valid) {
            cv::arrowedLine(vis, center_px, tip_px, cv::Scalar(255, 0, 255),
                2, cv::LINE_AA, 0, 0.25);
        }
    }

    const int sel_idx = resolve_selected_armor_idx(v, dbg);
    for (int i = 0; i < v.armor_count; ++i) {
        Eigen::Vector3d pos = v.predict_armor_position(i, pred_dt);
        bool valid = false;
        cv::Point2f px = aimer::tf::world_to_pixel(pos, q_imu, valid);
        if (!valid) continue;

        const bool selected = (i == sel_idx);
        cv::Scalar color = selected ? cv::Scalar(0, 80, 255)
                       : (v.armor_visible(i) ? cv::Scalar(50, 230, 50)
                                             : cv::Scalar(120, 120, 120));
        int radius = selected ? 9 : 6;
        cv::circle(vis, px, radius, color, 2, cv::LINE_AA);
        cv::putText(vis,
            fmt::format("A{}(id={}){} z{:+.0f}",
                i, v.armor_id(i), v.armor_visible(i) ? "V" : "-",
                predicted_armor_z_to_v(v, i, pred_dt) * 57.3),
            px + cv::Point2f(8, -6),
            cv::FONT_HERSHEY_SIMPLEX, 0.35, color, 1, cv::LINE_AA);
    }
}

// ==================== 主循环 ====================

void start_visualizer_node() {
    debug::print(debug::PrintMode::INFO, "Visualizer", "Starting visualizer node...");

    auto battlefield = umt::BasicObjManager<BattlefieldSnapshot>::find_or_create("battlefield");
    auto predictor_debug = umt::BasicObjManager<PredictorDebugFrame>::find_or_create("predictor_debug");
    auto fire_debug = umt::BasicObjManager<FireDebugInfo>::find_or_create("fire_debug");
    auto detector_debug_img = umt::BasicObjManager<cv::Mat>::find_or_create("detector_debug_img");
    auto running = umt::BasicObjManager<bool>::find_or_create("app_running", true);

    int last_frame_id = -1;
    auto last_render_time = SteadyClock::now();
    bool window_created = false;
    bool window_backend_available = true;
    bool window_backend_warned = false;
    const char* WINDOW_NAME = "RMCV";
    cv::Mat vis_buffer;

    debug::print(debug::PrintMode::INFO, "Visualizer", "Visualizer node started");

    while (running->get()) {
        bool show_window = false;
        try {
            show_window = runtime_param::get_param<bool>("Visualizer.show_window");
        } catch (const std::exception& e) {
            debug::print(debug::PrintMode::ERROR, "Visualizer",
                         "Read Visualizer.show_window failed: {}", e.what());
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        if (!show_window || !window_backend_available) {
            if (window_created) {
                try {
                    cv::destroyWindow(WINDOW_NAME);
                } catch (...) {
                    // ignore shutdown exceptions
                }
                window_created = false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        try {
            // 读取视图模式
            std::string view = runtime_param::get_param<std::string>("Visualizer.view");
            bool is_detector_view = (view == "detector");
            bool is_firecontrol_view = (view == "firecontrol");
            if (!is_detector_view && !is_firecontrol_view && view != "predictor") {
                // 兜底: 非法值按 predictor 处理，避免窗口空白
                view = "predictor";
            }

            cv::Mat vis;

            if (is_detector_view) {
                // 检测器视图: 直显 detector_debug_img
                const auto& det_img = detector_debug_img->get();
                if (det_img.empty()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    continue;
                }
                vis = copy_to_reused_buffer(det_img, vis_buffer);

            } else {
                // 默认: predictor / firecontrol 视图 (线程安全拷贝)
                const auto snapshot = battlefield->load();
                const auto predictor_dbg = predictor_debug->load();
                const auto dbg = fire_debug->load();

                // 只在新帧到来时重绘，避免同帧高频刷新导致抖动
                if (predictor_dbg.frame_id == last_frame_id || predictor_dbg.image.empty()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    continue;
                }

                last_frame_id = predictor_dbg.frame_id;
                last_render_time = SteadyClock::now();

                // 拷贝到复用缓冲区再叠加 OSD，减少重复分配抖动
                vis = copy_to_reused_buffer(predictor_dbg.image, vis_buffer);

                // 公共面板
                draw_battlefield_panel(vis, snapshot);
                draw_latency_panel(vis, snapshot, predictor_dbg, dbg);

                // firecontrol 专用面板/标记
                if (is_firecontrol_view) {
                    draw_fire_debug_panel(vis, snapshot, dbg);
                    draw_selected_target_panel(vis, snapshot, dbg);

                    if (dbg.fc_heartbeat > 0) {
                        const bool frame_aligned =
                            (dbg.snapshot_frame_id >= 0 && dbg.snapshot_frame_id == predictor_dbg.frame_id);
                        draw_pixel_markers(vis, dbg, predictor_dbg.q_imu, frame_aligned);
                        if (frame_aligned) {
                            draw_target_geometry_markers(vis, snapshot, dbg, predictor_dbg.q_imu);
                        }
                    }
                }
            }

            // 显示
            cv::imshow(WINDOW_NAME, vis);
            window_created = true;
            cv::waitKey(1);
        } catch (const cv::Exception& e) {
            if (!window_backend_warned) {
                debug::print(debug::PrintMode::WARNING, "Visualizer",
                             "OpenCV window backend unavailable, disable window: {}", e.what());
                window_backend_warned = true;
            }
            window_backend_available = false;
            if (window_created) {
                try {
                    cv::destroyWindow(WINDOW_NAME);
                } catch (...) {
                    // ignore shutdown exceptions
                }
                window_created = false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        } catch (const std::exception& e) {
            debug::print(debug::PrintMode::ERROR, "Visualizer",
                         "Visualizer loop exception: {}", e.what());
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        } catch (...) {
            debug::print(debug::PrintMode::ERROR, "Visualizer",
                         "Visualizer loop unknown exception");
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    if (window_created) {
        try {
            cv::destroyWindow(WINDOW_NAME);
        } catch (...) {
            // ignore shutdown exceptions
        }
    }

    debug::print(debug::PrintMode::INFO, "Visualizer", "Visualizer node stopped");
}

}  // namespace visualizer
