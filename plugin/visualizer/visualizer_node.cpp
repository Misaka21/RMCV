/**
 * @file visualizer_node.cpp
 * @brief 统一可视化线程
 *
 * 所有 imshow 集中在此线程，支持视图切换:
 *   - "predictor": snapshot.debug_img + 火控 OSD + 战场面板 + 延迟面板 + 像素标记
 *   - "detector":  detector_debug_img 直显
 *
 * 数据源:
 *   BasicObjManager<BattlefieldSnapshot> "battlefield"
 *   BasicObjManager<FireDebugInfo> "fire_debug"
 *   BasicObjManager<cv::Mat> "detector_debug_img"
 */

#include "visualizer_node.hpp"

#include <chrono>
#include <thread>

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
using autoaim::predictor::SpinLevel;
using autoaim::predictor::VehicleState;
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

// ==================== OSD 绘制函数 ====================

/**
 * @brief 左上角火控诊断面板 (~9行)
 */
static void draw_fire_debug_panel(cv::Mat& vis, const FireDebugInfo& dbg) {
    int lh = 15;
    int tx = 8;
    int ty = 16;
    auto put = [&](const std::string& text, cv::Scalar color = {200, 200, 200}) {
        cv::putText(vis, text, {tx, ty}, cv::FONT_HERSHEY_SIMPLEX,
            0.38, color, 1, cv::LINE_AA);
        ty += lh;
    };

    // 半透明背景
    int panel_h = 150;
    int panel_w = 380;
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

    // 2. Stage + Target + Armor
    cv::Scalar stage_color = (dbg.fail_stage == 9) ? cv::Scalar(0, 255, 0)
                           : (dbg.fail_stage == 0) ? cv::Scalar(150, 150, 150)
                                                   : cv::Scalar(0, 128, 255);
    put(fmt::format("Stage: {} | Tgt: {} Arm: {}",
        FireDebugInfo::fail_stage_name(dbg.fail_stage),
        dbg.target_id, dbg.armor_idx), stage_color);

    // 3. Bullet speed + snapshot info
    put(fmt::format("BS: {:.1f}m/s | valid: 0x{:03x} pri: {} frm: {}",
        dbg.bullet_speed, dbg.snapshot_valid_mask,
        dbg.snapshot_primary_id, dbg.snapshot_frame_id));

    // 4. Gimbal 角度 + 角速度
    put(fmt::format("Gimbal: yaw={:+.1f} pitch={:+.1f} deg",
        dbg.gimbal_yaw * 57.3, dbg.gimbal_pitch * 57.3), {180, 220, 255});
    put(fmt::format("  vel: yaw={:+.0f} pitch={:+.0f} deg/s",
        dbg.gimbal_yaw_vel * 57.3, dbg.gimbal_pitch_vel * 57.3), {150, 180, 220});

    // 5. Latency 分解
    put(fmt::format("Lat: img{:.0f} +pred{:.0f} +send{:.0f} +fly{:.0f} ={:.0f}ms",
        dbg.latency_img_to_predict,
        dbg.latency_predict_to_send,
        dbg.latency_send_to_control,
        dbg.latency_fire_to_hit,
        dbg.latency_total),
        cv::Scalar(100, 200, 255));

    // 以下仅在有效瞄准时显示
    if (dbg.fail_stage == 9) {
        // 6. Dist + FlyTime
        put(fmt::format("Dist: {:.2f}m  FlyT: {:.1f}ms",
            dbg.distance, dbg.fly_time * 1000), {200, 200, 200});

        // 7. Aim / Cmd 角度
        put(fmt::format("Aim:  y={:+.2f} p={:+.2f} deg",
            dbg.aim_yaw * 57.3, dbg.aim_pitch * 57.3), {0, 165, 255});  // 橙色
        put(fmt::format("Cmd:  y={:+.2f} p={:+.2f} deg",
            dbg.cmd_yaw * 57.3, dbg.cmd_pitch * 57.3), {0, 255, 255});  // 黄色

        // 8. Error + FIRE 状态 (tracking_error 单位是米: 落点偏移距离)
        cv::Scalar fire_color = dbg.fire_now ? cv::Scalar(0, 0, 255) : cv::Scalar(100, 200, 255);
        put(fmt::format("Err: {:.0f}cm  {}",
            dbg.tracking_error * 100, dbg.fire_now ? ">>> FIRE <<<" : "HOLD"),
            fire_color);
    }
}

/**
 * @brief 右下角战场信息面板
 */
static void draw_battlefield_panel(cv::Mat& vis, const BattlefieldSnapshot& snapshot) {
    // 收集有效目标数
    int n_tracked = 0;
    snapshot.for_each_valid([&](int, const VehicleState&) { n_tracked++; });

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

    snapshot.for_each_valid([&](int id, const VehicleState& v) {
        bool is_pri = (id == snapshot.primary_target_id);
        cv::Scalar color = is_pri ? cv::Scalar(0, 255, 255) : cv::Scalar(180, 180, 180);

        std::string type_str = armor_number_to_string(v.enemy_type);
        if (type_str != "sentry" && type_str != "outpost" && type_str != "base") {
            type_str = "Inf" + type_str;
        }

        // 可见装甲板
        std::string arm_str;
        for (int a = 0; a < v.armor_count; ++a) {
            if (v.armors[a].visible) {
                if (!arm_str.empty()) arm_str += ",";
                arm_str += std::to_string(a);
            }
        }
        if (arm_str.empty()) arm_str = "-";

        const char* spin_label = "NONE";
        if (v.spin.level == SpinLevel::LOW) spin_label = "LOW";
        else if (v.spin.level == SpinLevel::HIGH) spin_label = "HIGH";

        put(fmt::format("{}#{} {}  {:.1f}m  arm[{}]{}",
            is_pri ? ">" : " ", id, type_str, v.center.norm(),
            arm_str, is_pri ? " [PRI]" : ""), color);

        put(fmt::format("  spin:{} w={:+.0f}d/s r={:.2f}m  v=({:.1f},{:.1f})",
            spin_label, v.spin.omega * 57.3, v.spin.radius,
            v.velocity.x(), v.velocity.y()), {150, 150, 150});
    });
}

/**
 * @brief 左下角延迟信息面板 (pipeline + 条形图)
 */
static void draw_latency_panel(cv::Mat& vis, const BattlefieldSnapshot& snapshot,
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

    draw_line("Detect", snapshot.detect_latency_ms, cv::Scalar(100, 200, 255));
    draw_line("Predict", snapshot.predict_latency_ms, cv::Scalar(100, 255, 200));
    draw_line("Pipeline", total_pipeline_ms, cv::Scalar(0, 255, 255));

    // 火控延迟分解 (来自 FireDebugInfo)
    if (dbg.fc_heartbeat > 0) {
        draw_line(fmt::format("FC total"), static_cast<float>(dbg.latency_total),
            cv::Scalar(255, 200, 100));
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

    int detect_w = static_cast<int>(snapshot.detect_latency_ms * scale);
    cv::rectangle(vis,
        cv::Point(text_x, bar_y),
        cv::Point(text_x + detect_w, bar_y + bar_h),
        cv::Scalar(100, 150, 255), -1);

    int predict_w = static_cast<int>(snapshot.predict_latency_ms * scale);
    cv::rectangle(vis,
        cv::Point(text_x + detect_w, bar_y),
        cv::Point(text_x + detect_w + predict_w, bar_y + bar_h),
        cv::Scalar(100, 255, 150), -1);
}

/**
 * @brief 像素标记 (枪管、瞄准点、目标位置等)
 */
static void draw_pixel_markers(cv::Mat& vis, const FireDebugInfo& dbg,
                                const Eigen::Quaterniond& q_imu) {
    int cx = vis.cols / 2;
    int cy = vis.rows / 2;

    // 白色十字: 图像中心
    cv::drawMarker(vis, cv::Point(cx, cy), cv::Scalar(255, 255, 255),
        cv::MARKER_CROSS, 20, 1, cv::LINE_AA);

    // 绿色实心圆: 枪管当前指向 (gimbal_yaw/pitch)
    bool gimbal_valid = false;
    cv::Point2f gimbal_px = angle_to_pixel(dbg.gimbal_yaw, dbg.gimbal_pitch,
        q_imu, gimbal_valid);
    if (gimbal_valid) {
        cv::circle(vis, gimbal_px, 5, cv::Scalar(0, 255, 0), -1, cv::LINE_AA);
    }

    if (dbg.fail_stage != 9) return;  // 以下仅在瞄准有效时绘制

    // 红色圆环: 目标装甲板位置
    bool target_valid = false;
    cv::Point2f target_px = aimer::tf::world_to_pixel(dbg.target_pos, q_imu, target_valid);
    if (target_valid) {
        cv::circle(vis, target_px, 12, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    }

    // 橙色实心圆: 期望瞄准 (aim_yaw/pitch)
    bool aim_valid = false;
    cv::Point2f aim_px = angle_to_pixel(dbg.aim_yaw, dbg.aim_pitch,
        q_imu, aim_valid, dbg.distance > 0 ? dbg.distance : 5.0);
    if (aim_valid) {
        cv::circle(vis, aim_px, 5, cv::Scalar(0, 165, 255), -1, cv::LINE_AA);
    }

    // 黄色实心圆: 发送角 (cmd_yaw/pitch)
    bool cmd_valid = false;
    cv::Point2f cmd_px = angle_to_pixel(dbg.cmd_yaw, dbg.cmd_pitch,
        q_imu, cmd_valid, dbg.distance > 0 ? dbg.distance : 5.0);
    if (cmd_valid) {
        cv::circle(vis, cmd_px, 5, cv::Scalar(0, 255, 255), -1, cv::LINE_AA);
    }

    // 红色箭头: aim → cmd 补偿方向
    if (aim_valid && cmd_valid) {
        double dx = cmd_px.x - aim_px.x;
        double dy = cmd_px.y - aim_px.y;
        double len = std::sqrt(dx * dx + dy * dy);
        if (len > 3.0) {
            cv::arrowedLine(vis, aim_px, cmd_px,
                cv::Scalar(0, 0, 255), 1, cv::LINE_AA, 0, 0.3);
        }
    }
}

// ==================== 主循环 ====================

void start_visualizer_node() {
    debug::print(debug::PrintMode::INFO, "Visualizer", "Starting visualizer node...");

    auto battlefield = umt::BasicObjManager<BattlefieldSnapshot>::find_or_create("battlefield");
    auto fire_debug = umt::BasicObjManager<FireDebugInfo>::find_or_create("fire_debug");
    auto detector_debug_img = umt::BasicObjManager<cv::Mat>::find_or_create("detector_debug_img");
    auto running = umt::BasicObjManager<bool>::find_or_create("app_running", true);

    int last_frame_id = -1;
    bool window_created = false;
    bool window_backend_available = true;
    bool window_backend_warned = false;
    const char* WINDOW_NAME = "RMCV";

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

            cv::Mat vis;

            if (view == "detector") {
                // 检测器视图: 直显 detector_debug_img
                const auto& det_img = detector_debug_img->get();
                if (det_img.empty()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    continue;
                }
                vis = det_img.clone();

            } else {
                // 默认: predictor 视图
                const auto& snapshot = battlefield->get();

                // 去重: 同一帧不重复渲染
                if (snapshot.frame_id == last_frame_id || snapshot.debug_img.empty()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    continue;
                }
                last_frame_id = snapshot.frame_id;

                // clone 一份用于 OSD 叠加
                vis = snapshot.debug_img.clone();

                // 读取火控调试信息
                const auto& dbg = fire_debug->get();

                // OSD 面板
                draw_fire_debug_panel(vis, dbg);
                draw_battlefield_panel(vis, snapshot);
                draw_latency_panel(vis, snapshot, dbg);

                // 像素标记 (枪管、瞄准点等)
                if (dbg.fc_heartbeat > 0) {
                    draw_pixel_markers(vis, dbg, snapshot.self_state.q_imu);
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
