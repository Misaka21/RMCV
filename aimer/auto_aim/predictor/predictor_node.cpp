/**
 * @file predictor_node.cpp
 * @brief 预测器节点
 *
 * 订阅: Message<DetectionResult> "detections"
 * 输出: BasicObjManager<BattlefieldSnapshot> "battlefield"
 */

#include <chrono>
#include <thread>

#include <fmt/format.h>
#include <opencv2/imgproc.hpp>

#include <opencv2/highgui.hpp>

#include "aimer/common/types.hpp"
#include "aimer/common/math/math.hpp"
#include "aimer/common/transformer/transformer.hpp"
#include "aimer/common/fire_control_types.hpp"
#include "aimer/common/robot_state.hpp"
#include "enemy_predictor.hpp"
#include "plugin/debug/logger.hpp"
#include "plugin/param/runtime_parameter.hpp"
#include "plugin/stats/fps_stats.hpp"
#include "plugin/watchdog/watchdog_node.hpp"
#include "plugin/webview/dashboard.hpp"
#include "plugin/plotter/plotter.hpp"
#include "umt/umt.hpp"

namespace autoaim::predictor {

using SteadyClock = std::chrono::steady_clock;

/**
 * @brief 启动预测器节点
 */
void start_predictor_node() {
    debug::print(debug::PrintMode::INFO, "PredictorNode", "Starting predictor node...");

    // 创建预测器
    EnemyPredictor predictor;

    // 设置 UMT
    umt::Subscriber<DetectionResult> sub("detections");
    auto battlefield = umt::BasicObjManager<BattlefieldSnapshot>::find_or_create("battlefield");
    umt::Publisher<cv::Mat> vis_pub("predictor_vis");  // 可视化帧 (供录制)
    umt::Publisher<cv::Mat> pub_debug("/predictor/debug");  // Web 调试图像
    auto running = umt::BasicObjManager<bool>::find_or_create("app_running", true);

    stats::FpsStats stats("PredictorNode", "tracked");

    debug::print(debug::PrintMode::INFO, "PredictorNode", "Predictor node started");

    while (running->get()) {
        watchdog::heartbeat("predictor");
        try {
            auto detection = sub.pop_for(1000);

            // 非自瞄模式时跳过预测
            if (detection.state.aim_mode != aimer::AimMode::AUTOAIM) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            // 使用相机帧时间戳（从 SyncFrame 传递过来，微秒转秒）
            double timestamp = detection.state.timestamp_us / 1e6;

            // DEBUG: 输入装甲板数量
            if (!detection.armors.empty()) {
                //fmt::print(fmt::fg(fmt::color::magenta),
                //    "[DEBUG] Input: {} armors\n", detection.armors.size());
            }

            // 运行预测
            auto predict_start = SteadyClock::now();
            auto snapshot = predictor.predict(detection, timestamp);
            auto predict_end = SteadyClock::now();

            float latency = std::chrono::duration_cast<std::chrono::microseconds>(
                predict_end - predict_start).count() / 1000.0f;

            // 设置预测完成时间戳 (供火控计算 predict_to_send 延迟)
            auto predict_time_since_epoch = predict_end.time_since_epoch();
            snapshot.predict_timestamp = std::chrono::duration<double>(predict_time_since_epoch).count();

            // clone 原图像给火控调试用
            if (!detection.img.empty()) {
                snapshot.debug_img = detection.img.clone();
            }

            // 写入共享对象 (火控通过 BasicObjManager 读取)
            battlefield->get() = snapshot;

            // 输出云台状态到 PlotJuggler
            // q_imu 是 IMU 原始姿态，需要修正 R_gimbal2imubody 得到真正的云台角度
            {
                // Gimbal → Imu 的旋转修正
                const auto& R_g2i = aimer::tf::Transform<
                    aimer::tf::Frame::Gimbal, aimer::tf::Frame::Imu>::R_;
                // q_gimbal = R_g2i^T * q_imu (从 Imu 坐标系转到 Gimbal 坐标系)
                Eigen::Quaterniond q_gimbal(R_g2i.transpose() * snapshot.self_state.q_imu.toRotationMatrix());
                auto [yaw, pitch] = aimer::math::quat_to_yaw_pitch(q_gimbal);
                plotter::begin();
                plotter::add("/gimbal/yaw", yaw * 57.3);
                plotter::add("/gimbal/pitch", pitch * 57.3);
                plotter::end();
            }

            // 统计
            int tracked = 0;
            for (int i = 1; i < MAX_TARGETS; ++i) {
                if (snapshot.is_valid(i)) tracked++;
            }
            stats.update(latency, tracked > 0);  // tick + print_if_needed

            // 更新 Dashboard 数据
            dashboard::set("predictor.latency_ms", latency);
            dashboard::set("predictor.tracked_count", tracked);
            dashboard::set("predictor.fps", stats.last_fps);

            // 可视化 (在有 Web 订阅者或 show_window 时执行)
            bool show_window = runtime_param::get_param<bool>("AutoAim.Predictor.show_window");
            bool has_sub = pub_debug.has_subscriber();
            if ((has_sub || show_window) && !detection.img.empty()) {
                cv::Mat vis = detection.img.clone();
                predictor.draw(vis, detection.state.q_imu, timestamp);

                // 火控调试叠加层
                bool draw_fire_debug = runtime_param::get_param<bool>("AutoAim.Predictor.draw_fire_debug");
                if (draw_fire_debug) {
                    auto fire_debug_obj = umt::BasicObjManager<::fire_control::FireDebugInfo>::find_or_create("fire_debug");
                    const auto& fd = fire_debug_obj->get();
                    const auto& q_imu = detection.state.q_imu;

                    // 角度→像素投影 lambda
                    auto angle_to_pixel = [&](double yaw, double pitch, double dist) -> std::pair<cv::Point2f, bool> {
                        auto p = aimer::math::ypd_to_xyz(aimer::math::YpdCoord{yaw, pitch, dist});
                        bool ok;
                        auto px = aimer::tf::world_to_pixel(p, q_imu, ok);
                        return {px, ok};
                    };

                    // 白色十字: 图像中心
                    {
                        int cx = vis.cols / 2, cy = vis.rows / 2;
                        int sz = 12;
                        cv::line(vis, {cx - sz, cy}, {cx + sz, cy}, {255, 255, 255}, 1, cv::LINE_AA);
                        cv::line(vis, {cx, cy - sz}, {cx, cy + sz}, {255, 255, 255}, 1, cv::LINE_AA);
                    }

                    // 绿色实心: 枪管当前指向 (用云台 yaw/pitch 投影)
                    // 使用较大距离 (100m) 消除枪管-相机平移带来的近距离视差
                    {
                        auto [barrel_px, barrel_ok] = angle_to_pixel(
                            fd.gimbal_yaw, fd.gimbal_pitch, 100.0);
                        if (barrel_ok) {
                            cv::circle(vis, barrel_px, 5, {0, 255, 0}, -1, cv::LINE_AA);
                        }
                    }

                    // 左上角诊断面板
                    {
                        bool fc_alive = fd.fc_heartbeat > 0;
                        const char* mode_name = aimer::aim_mode_name(
                            aimer::to_aim_mode(fd.fc_mode));
                        const char* stage_name = ::fire_control::FireDebugInfo::fail_stage_name(
                            fd.fail_stage);

                        // 面板参数
                        int panel_w = 380;
                        int panel_h = fd.valid ? 160 : 100;
                        int panel_x = 5;
                        int panel_y = 5;

                        // 半透明背景
                        int safe_w = std::min(panel_w, vis.cols - panel_x);
                        int safe_h = std::min(panel_h, vis.rows - panel_y);
                        if (safe_w > 0 && safe_h > 0) {
                            cv::Mat roi = vis(cv::Rect(panel_x, panel_y, safe_w, safe_h));
                            roi = roi * 0.3;
                        }

                        int tx = panel_x + 6;
                        int ty = panel_y + 14;
                        int lh = 15;
                        auto put = [&](const std::string& text, cv::Scalar color = {200, 200, 200}) {
                            cv::putText(vis, text, {tx, ty}, cv::FONT_HERSHEY_SIMPLEX,
                                0.40, color, 1, cv::LINE_AA);
                            ty += lh;
                        };

                        // 第1行: 火控线程状态 + 模式
                        put(fmt::format("FC: {}  Mode: {}",
                            fc_alive ? "ALIVE" : "DEAD", mode_name),
                            fc_alive ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255));

                        // 第2行: 管线状态
                        cv::Scalar stage_color = (fd.fail_stage == 9)
                            ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 100, 255);
                        put(fmt::format("Stage: {}  Target: #{}  Armor: [{}]",
                            stage_name, fd.snapshot_primary_id, fd.armor_idx),
                            stage_color);

                        // 第3行: 弹速 + snapshot
                        put(fmt::format("Bullet: {:.1f}m/s  Snap: frm={} mask={:#06x}",
                            fd.bullet_speed, fd.snapshot_frame_id, fd.snapshot_valid_mask));

                        // 第4行: 云台角度 + 角速度
                        put(fmt::format("Gimbal: Y={:+.1f}  P={:+.1f}  Yv={:+.0f}  Pv={:+.0f} deg/s",
                            fd.gimbal_yaw * 57.3, fd.gimbal_pitch * 57.3,
                            fd.gimbal_yaw_vel * 57.3, fd.gimbal_pitch_vel * 57.3));

                        // 第5行: 延迟分解
                        put(fmt::format("Latency: {:.1f}ms (img={:.1f} pred={:.1f} send={:.1f} fly={:.1f})",
                            fd.latency_total, fd.latency_img_to_predict,
                            fd.latency_predict_to_send, fd.latency_send_to_control,
                            fd.latency_fire_to_hit));

                        if (fd.valid) {
                            ty += 3;

                            // 第6行: 距离 + 飞行时间
                            put(fmt::format("Dist: {:.2f}m  FlyTime: {:.1f}ms",
                                fd.distance, fd.fly_time * 1000));

                            // 第7行: 瞄准角 (AimResult)
                            put(fmt::format("Aim:  Y={:+.2f}  P={:+.2f} deg",
                                fd.aim_yaw * 57.3, fd.aim_pitch * 57.3),
                                {0, 165, 255});

                            // 第8行: 发送角 (FireCommand)
                            put(fmt::format("Cmd:  Y={:+.2f}  P={:+.2f} deg",
                                fd.cmd_yaw * 57.3, fd.cmd_pitch * 57.3),
                                {0, 255, 255});

                            // 第9行: 跟踪误差 + 开火状态
                            std::string fire_str = fd.fire_now ? "  >>> FIRE <<<" : "";
                            put(fmt::format("Error: {:.2f} deg{}",
                                fd.tracking_error * 57.3, fire_str),
                                fd.fire_now ? cv::Scalar(0, 0, 255) : cv::Scalar(100, 200, 255));
                        }
                    }

                    // 右下角战场信息面板
                    {
                        // 收集有效目标信息
                        struct VehInfo {
                            int id;
                            std::string type_name;
                            double dist;
                            double omega_dps;     // 角速度 (deg/s)
                            double vx, vy;        // 中心速度
                            double radius;
                            int visible_count;
                            std::string armor_ids;
                            bool is_primary;
                            const char* spin_label;
                        };
                        std::vector<VehInfo> vehs;

                        snapshot.for_each_valid([&](int id, const predictor::VehicleState& v) {
                            VehInfo info;
                            info.id = id;
                            info.type_name = autoaim::armor_number_to_string(v.enemy_type);
                            info.dist = v.center.norm();
                            info.omega_dps = v.spin.omega * 57.3;
                            info.vx = v.velocity.x();
                            info.vy = v.velocity.y();
                            info.radius = v.spin.radius;
                            info.is_primary = (id == snapshot.primary_target_id);

                            // 陀螺等级
                            switch (v.spin.level) {
                                case predictor::SpinLevel::NONE: info.spin_label = "NONE"; break;
                                case predictor::SpinLevel::LOW:  info.spin_label = "LOW";  break;
                                case predictor::SpinLevel::HIGH: info.spin_label = "HIGH"; break;
                                default: info.spin_label = "?"; break;
                            }

                            // 可见装甲板
                            info.visible_count = 0;
                            std::string ids;
                            for (int a = 0; a < v.armor_count; ++a) {
                                if (v.armors[a].visible) {
                                    info.visible_count++;
                                    if (!ids.empty()) ids += ",";
                                    ids += std::to_string(a);
                                }
                            }
                            info.armor_ids = ids.empty() ? "-" : ids;
                            vehs.push_back(info);
                        });

                        if (!vehs.empty()) {
                            // 每个目标2行，加标题1行
                            int line_count = 1 + static_cast<int>(vehs.size()) * 2;
                            int lh = 15;
                            int panel_h = line_count * lh + 12;
                            int panel_w = 340;
                            int panel_x = vis.cols - panel_w - 5;
                            int panel_y = vis.rows - panel_h - 5;

                            // 半透明背景
                            int safe_x = std::max(0, panel_x);
                            int safe_y = std::max(0, panel_y);
                            int safe_w = std::min(panel_w, vis.cols - safe_x);
                            int safe_h = std::min(panel_h, vis.rows - safe_y);
                            if (safe_w > 0 && safe_h > 0) {
                                cv::Mat roi = vis(cv::Rect(safe_x, safe_y, safe_w, safe_h));
                                roi = roi * 0.3;
                            }

                            int tx = panel_x + 6;
                            int ty = panel_y + 14;
                            auto put = [&](const std::string& text, cv::Scalar color = {200, 200, 200}) {
                                cv::putText(vis, text, {tx, ty}, cv::FONT_HERSHEY_SIMPLEX,
                                    0.38, color, 1, cv::LINE_AA);
                                ty += lh;
                            };

                            put(fmt::format("Battlefield  ({} tracked)", vehs.size()), {255, 255, 255});

                            for (const auto& v : vehs) {
                                cv::Scalar color = v.is_primary ? cv::Scalar(0, 255, 255) : cv::Scalar(180, 180, 180);
                                // 第1行: 类型 + 距离 + 可见装甲板
                                put(fmt::format("{}#{} {:.1f}m  arm[{}] {}{}",
                                    v.is_primary ? ">" : " ",
                                    v.id, v.dist, v.armor_ids,
                                    v.type_name == "sentry" ? "Sentry" :
                                    v.type_name == "outpost" ? "Outpost" :
                                    v.type_name == "base" ? "Base" :
                                    "Inf" + v.type_name,
                                    v.is_primary ? " [PRI]" : ""),
                                    color);
                                // 第2行: 陀螺状态 + 速度
                                put(fmt::format("  spin:{} w={:+.0f}d/s r={:.2f}m  vel=({:.1f},{:.1f})",
                                    v.spin_label, v.omega_dps, v.radius, v.vx, v.vy),
                                    {150, 150, 150});
                            }
                        }
                    }

                    if (fd.valid) {
                        double dist = std::max(fd.distance, 1.0);

                        // 橙色实心: 期望瞄准 (AimResult)
                        cv::Point2f aim_px;
                        bool aim_ok;
                        std::tie(aim_px, aim_ok) = angle_to_pixel(fd.aim_yaw, fd.aim_pitch, dist);

                        // 黄色实心: 发送角 (FireCommand)
                        cv::Point2f cmd_px;
                        bool cmd_ok;
                        std::tie(cmd_px, cmd_ok) = angle_to_pixel(fd.cmd_yaw, fd.cmd_pitch, dist);

                        // 红色圆环: 目标装甲板位置
                        {
                            bool tgt_ok;
                            auto tgt_px = aimer::tf::world_to_pixel(fd.target_pos, q_imu, tgt_ok);
                            if (tgt_ok) {
                                cv::circle(vis, tgt_px, 14, {0, 0, 255}, 2, cv::LINE_AA);
                            }
                        }

                        if (aim_ok) {
                            cv::circle(vis, aim_px, 6, {0, 165, 255}, -1, cv::LINE_AA);  // 橙色 BGR
                        }

                        if (cmd_ok) {
                            cv::circle(vis, cmd_px, 5, {0, 255, 255}, -1, cv::LINE_AA);  // 黄色 BGR
                        }

                        // 红色箭头: 橙色→黄色 (预测补偿方向)
                        if (aim_ok && cmd_ok) {
                            double dx = cmd_px.x - aim_px.x;
                            double dy = cmd_px.y - aim_px.y;
                            if (dx * dx + dy * dy > 4.0) {  // 至少2px
                                cv::arrowedLine(vis, aim_px, cmd_px, {0, 0, 255}, 2, cv::LINE_AA, 0, 0.3);
                            }
                        }
                    }
                }

                // 左下角延迟信息面板
                auto now = SteadyClock::now();
                auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(
                    now.time_since_epoch()).count();
                int64_t exposure_us = detection.state.timestamp_us;
                float total_latency_ms = (now_us - exposure_us) / 1000.0f;
                float detect_latency_ms = detection.latency_ms;
                float predict_latency_ms = latency;

                int panel_x = 10;
                int panel_y = vis.rows - 90;
                int panel_w = 200;
                int panel_h = 80;
                cv::Mat roi = vis(cv::Rect(panel_x, panel_y, panel_w, panel_h));
                roi = roi * 0.4;

                int text_x = panel_x + 8;
                int text_y = panel_y + 18;
                int line_h = 16;
                auto draw_latency_line = [&](const std::string& label, float ms, cv::Scalar color) {
                    std::string text = fmt::format("{}: {:.1f}ms", label, ms);
                    cv::putText(vis, text, cv::Point(text_x, text_y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv::LINE_AA);
                    text_y += line_h;
                };

                draw_latency_line("Detect", detect_latency_ms, cv::Scalar(100, 200, 255));
                draw_latency_line("Predict", predict_latency_ms, cv::Scalar(100, 255, 200));
                draw_latency_line("Total", total_latency_ms, cv::Scalar(0, 255, 255));

                int bar_y = text_y + 5;
                int bar_h = 10;
                float max_ms = std::max(50.0f, total_latency_ms);
                float scale = (panel_w - 20) / max_ms;

                cv::rectangle(vis,
                    cv::Point(text_x, bar_y),
                    cv::Point(text_x + static_cast<int>(max_ms * scale), bar_y + bar_h),
                    cv::Scalar(50, 50, 50), -1);

                int detect_w = static_cast<int>(detect_latency_ms * scale);
                cv::rectangle(vis,
                    cv::Point(text_x, bar_y),
                    cv::Point(text_x + detect_w, bar_y + bar_h),
                    cv::Scalar(100, 150, 255), -1);

                int predict_w = static_cast<int>(predict_latency_ms * scale);
                cv::rectangle(vis,
                    cv::Point(text_x + detect_w, bar_y),
                    cv::Point(text_x + detect_w + predict_w, bar_y + bar_h),
                    cv::Scalar(100, 255, 150), -1);

                vis_pub.push(vis);

                if (has_sub) {
                    pub_debug.push(vis);
                }
                if (show_window) {
                    cv::imshow("Predictor", vis);
                    cv::waitKey(1);
                }
            }

        } catch (const umt::MessageError_Timeout&) {
            // 超时，继续
        } catch (const umt::MessageError_Stopped&) {
            // 没有 publisher，等待 detector 启动
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } catch (const std::exception& e) {
            debug::print(debug::PrintMode::ERROR, "PredictorNode",
                "Exception: {}", e.what());
        }
    }

    debug::print(debug::PrintMode::INFO, "PredictorNode", "Predictor node stopped");
}

}  // namespace autoaim::predictor
