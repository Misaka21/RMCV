/**
 * @file predictor_node.cpp
 * @brief 预测器节点
 *
 * 订阅: Message<DetectionResult> "detections"
 * 发布: Message<BattlefieldSnapshot> "battlefield"
 */

#include <chrono>
#include <thread>

#include <fmt/format.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "aimer/common/types.hpp"
#include "aimer/common/transformer/transformer.hpp"
#include "enemy_predictor.hpp"
#include "plugin/debug/logger.hpp"
#include "plugin/param/runtime_parameter.hpp"
#include "plugin/stats/fps_stats.hpp"
#include "plugin/watchdog/watchdog_node.hpp"
#include "plugin/webview/dashboard.hpp"
#include "umt/umt.hpp"

namespace autoaim::predictor {

using SteadyClock = std::chrono::steady_clock;

/**
 * @brief 绘制基于 z_to_v 优化的装甲板框
 */
void draw_armor_by_z_to_v(
    cv::Mat& img,
    const Eigen::Vector3d& pos_world,
    double z_to_v,  // 优化后的朝向角
    ArmorType type,
    int target_id,
    const Eigen::Quaterniond& q_imu,
    const cv::Scalar& color,
    int thickness = 2
) {
    // 装甲板尺寸
    double w = (type == ArmorType::LARGE) ? LARGE_ARMOR_WIDTH : SMALL_ARMOR_WIDTH;
    double h = (type == ArmorType::LARGE) ? LARGE_ARMOR_HEIGHT : SMALL_ARMOR_HEIGHT;

    // 装甲板俯仰角 (弧度): -15° 表示装甲板上沿向后倾斜
    double pitch = -15.0 * M_PI / 180.0;

    // 相机 Z 轴在世界 XY 平面的投影 (归一化)
    Eigen::Vector3d camera_z_world = tf::vector<tf::Frame::Camera, tf::Frame::World>(
        Eigen::Vector3d(0, 0, 1), q_imu
    );
    Eigen::Vector2d camera_z_i2(camera_z_world.x(), camera_z_world.y());
    double norm = camera_z_i2.norm();
    if (norm > 1e-6) camera_z_i2 /= norm;
    else camera_z_i2 = Eigen::Vector2d(1.0, 0.0);

    // 装甲板法向量在世界 XY 平面的方向 = 相机前向旋转 z_to_v
    Eigen::Vector2d radius_norm = math::rotate(camera_z_i2, z_to_v);

    // 装甲板 X 轴 (水平方向，垂直于法向量)
    Eigen::Vector2d x_2d = math::rotate(radius_norm, M_PI / 2);
    Eigen::Vector3d x_axis(x_2d.x(), x_2d.y(), 0.0);

    // 装甲板 Y 轴 (竖直方向，考虑俯仰角)
    // 与 rm.cv.fans 的 radial_armor_corners 一致
    Eigen::Vector3d y_axis(
        -radius_norm.x() * std::sin(pitch),
        -radius_norm.y() * std::sin(pitch),
        std::cos(pitch)
    );

    // 四角点 (世界坐标系): 左上、左下、右下、右上 (逆时针)
    std::array<Eigen::Vector3d, 4> corners = {
        pos_world + x_axis * (w / 2) + y_axis * (h / 2),  // LT
        pos_world + x_axis * (w / 2) - y_axis * (h / 2),  // LB
        pos_world - x_axis * (w / 2) - y_axis * (h / 2),  // RB
        pos_world - x_axis * (w / 2) + y_axis * (h / 2)   // RT
    };

    // 投影到图像
    std::array<cv::Point2f, 4> pts;
    bool all_valid = true;
    for (int i = 0; i < 4; ++i) {
        bool valid = false;
        pts[i] = tf::world_to_pixel(corners[i], q_imu, valid);
        if (!valid) all_valid = false;
    }

    if (!all_valid) return;

    // 画矩形框
    for (int i = 0; i < 4; ++i) {
        cv::line(img, pts[i], pts[(i + 1) % 4], color, thickness, cv::LINE_AA);
    }

    // DEBUG: 标记角点编号
    const char* labels[] = {"0:LT", "1:LB", "2:RB", "3:RT"};
    for (int i = 0; i < 4; ++i) {
        cv::putText(img, labels[i], pts[i] + cv::Point2f(3, -3),
                    cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 255), 1);
    }
}

/**
 * @brief 打印观测调试信息
 */
void print_observations(const ArmorObservationTable& table) {
    for (int target_id : table.get_target_ids()) {
        const auto& obs_list = table.get(target_id);
        for (const auto& obs : obs_list) {
            if (!obs.valid) continue;

            // 计算面积
            double area = 0;
            if (obs.pts.size() >= 4) {
                // 四边形面积
                area = std::abs(
                    (obs.pts[0].x - obs.pts[2].x) * (obs.pts[1].y - obs.pts[3].y) -
                    (obs.pts[1].x - obs.pts[3].x) * (obs.pts[0].y - obs.pts[2].y)
                ) / 2.0;
            }

            // 球坐标 (yaw, pitch, dist)
            double yaw_deg = obs.z[0] * 180.0 / M_PI;
            double pitch_deg = obs.z[1] * 180.0 / M_PI;
            double dist = obs.z[2];

            // 装甲板朝向 (armor_yaw)
            double armor_yaw_deg = obs.z[3] * 180.0 / M_PI;

            // 装甲板类型
            const char* type_str = (obs.type == ArmorType::LARGE) ? "L" : "S";

            fmt::print(fmt::fg(fmt::color::cyan),
                "number: {} type: {} area: {:.1f}k\n"
                "ypd: {:.1f}|{:.1f}|{:.3f}\n"
                "xyz: {:.3f}|{:.3f}|{:.3f}\n"
                "armor_yaw: {:.1f} deg\n"
                "z_to_v: {:.1f} → {:.1f} deg (raw → fit)\n\n",
                target_id, type_str, area / 1000.0,
                yaw_deg, pitch_deg, dist,
                obs.pos.x(), obs.pos.y(), obs.pos.z(),
                armor_yaw_deg,
                obs.z_to_v_raw * 180.0 / M_PI, obs.z_to_v * 180.0 / M_PI
            );
        }
    }
}

/**
 * @brief 打印预测结果调试信息
 */
void print_predictions(const BattlefieldSnapshot& snapshot) {
    for (int i = 1; i < MAX_TARGETS; ++i) {
        if (!snapshot.is_valid(i)) continue;

        const auto& v = snapshot.vehicles[i];
        fmt::print(fmt::fg(fmt::color::yellow),
            "[T{}] center: ({:.3f}, {:.3f}, {:.3f}) vel: ({:.2f}, {:.2f}, {:.2f}) armors: {}\n",
            i,
            v.center.x(), v.center.y(), v.center.z(),
            v.velocity.x(), v.velocity.y(), v.velocity.z(),
            v.armor_count
        );
    }
}

/**
 * @brief 绘制预测结果可视化
 */
void draw_prediction(
    cv::Mat& img,
    const BattlefieldSnapshot& snapshot,
    const ArmorObservationTable& table,
    const Eigen::Quaterniond& q_imu
) {
    // ========== 1. 绘制原始观测 + 优化后装甲板框 ==========
    for (int target_id : table.get_target_ids()) {
        const auto& obs_list = table.get(target_id);
        int obs_idx = 0;
        for (const auto& obs : obs_list) {
            if (!obs.valid) continue;

            bool valid = false;
            cv::Point2f obs_px = tf::world_to_pixel(obs.pos, q_imu, valid);
            if (!valid) continue;

            // 蓝色小圆圈标记观测位置
            cv::circle(img, obs_px, 5, cv::Scalar(255, 100, 0), 2, cv::LINE_AA);

            // 绘制优化后的装甲板框 (黄色)
            draw_armor_by_z_to_v(img, obs.pos, obs.z_to_v, obs.type, target_id,
                                 q_imu, cv::Scalar(0, 255, 255), 2);

            // 显示 armor_yaw 和 z_to_v 信息
            // 根据位置决定文字放在左侧还是右侧，避免重叠
            bool on_left = obs_px.x > img.cols / 2;
            float x_offset = on_left ? -150 : 10;
            float y_offset = -20 + obs_idx * 50;  // 每个装甲板往下偏移

            cv::Point2f text_pos = obs_px + cv::Point2f(x_offset, y_offset);

            // 边界检查
            if (text_pos.x < 5) text_pos.x = 5;
            if (text_pos.x > img.cols - 150) text_pos.x = img.cols - 150;
            if (text_pos.y < 15) text_pos.y = 15;
            if (text_pos.y > img.rows - 35) text_pos.y = img.rows - 35;

            double armor_yaw_deg = obs.z[3] * 180.0 / M_PI;
            double z_to_v_raw_deg = obs.z_to_v_raw * 180.0 / M_PI;
            double z_to_v_deg = obs.z_to_v * 180.0 / M_PI;

            // 背景框
            cv::rectangle(img, text_pos + cv::Point2f(-2, -12),
                          text_pos + cv::Point2f(145, 30), cv::Scalar(0, 0, 0, 180), -1);

            cv::putText(img, fmt::format("T{} armor_yaw: {:.1f}", target_id, armor_yaw_deg),
                        text_pos, cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 255), 1);
            cv::putText(img, fmt::format("z_to_v: {:.1f} -> {:.1f}", z_to_v_raw_deg, z_to_v_deg),
                        text_pos + cv::Point2f(0, 14), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 200, 0), 1);

            ++obs_idx;
        }
    }

    // ========== 2. 绘制 EKF 预测结果 + 完整信息 ==========
    for (int i = 1; i < MAX_TARGETS; ++i) {
        if (!snapshot.is_valid(i)) continue;

        const auto& vehicle = snapshot.vehicles[i];

        // 绘制各装甲板
        for (int j = 0; j < vehicle.armor_count; ++j) {
            const auto& armor = vehicle.armors[j];

            bool valid = false;
            cv::Point2f armor_px = tf::world_to_pixel(armor.position, q_imu, valid);
            if (!valid) continue;
            if (armor_px.x < 0 || armor_px.x >= img.cols ||
                armor_px.y < 0 || armor_px.y >= img.rows) continue;

            // 绿色圆圈 (推荐目标实心，其他空心)
            bool is_best = (j == vehicle.recommended_armor_idx);
            cv::Scalar circle_color = is_best ? cv::Scalar(0, 255, 0) : cv::Scalar(100, 255, 100);
            int thickness = is_best ? -1 : 2;
            cv::circle(img, armor_px, 8, circle_color, thickness, cv::LINE_AA);

            // 计算信息
            double dist = armor.position.norm();
            double vel = armor.velocity.norm();
            const char* type_str = (armor.type == ArmorType::LARGE) ? "L" : "S";

            // ===== 信息卡片 (右侧) =====
            cv::Point2f text_pos = armor_px + cv::Point2f(15, -35);
            int y_offset = 0;

            auto draw_text = [&](const std::string& text, cv::Scalar color) {
                cv::putText(img, text, text_pos + cv::Point2f(1, y_offset + 1),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
                cv::putText(img, text, text_pos + cv::Point2f(0, y_offset),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv::LINE_AA);
                y_offset += 14;
            };

            // number: 3 id: 4
            draw_text(fmt::format("number: {} id: {}", i, armor.id), cv::Scalar(255, 255, 255));
            // type: S
            draw_text(fmt::format("type: {}", type_str), cv::Scalar(200, 200, 200));
            // dist: 1.09m
            draw_text(fmt::format("dist: {:.2f}m", dist), cv::Scalar(100, 255, 255));
            // vel: 0.00m/s
            draw_text(fmt::format("vel: {:.2f}m/s", vel), cv::Scalar(0, 255, 0));
            // xyz: -0.15|-1.02|-0.14
            draw_text(fmt::format("xyz: {:.2f}|{:.2f}|{:.2f}",
                armor.position.x(), armor.position.y(), armor.position.z()),
                cv::Scalar(150, 255, 150));
        }
    }

    // ========== 3. 顶部状态栏 ==========
    int valid_count = 0;
    int detected_count = 0;
    for (int i = 1; i < MAX_TARGETS; ++i) {
        if (snapshot.is_valid(i)) valid_count++;
        if (snapshot.is_detected(i)) detected_count++;
    }

    // 背景条
    cv::rectangle(img, cv::Point(0, 0), cv::Point(img.cols, 25), cv::Scalar(0, 0, 0), -1);

    std::string status = fmt::format(
        "Predictor | Detected: {} | Tracking: {} | Frame: {}",
        detected_count, valid_count, snapshot.frame_id
    );
    cv::putText(img, status, cv::Point(10, 18),
        cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);

    // 图例
    int legend_x = img.cols - 200;
    cv::circle(img, cv::Point(legend_x, 12), 5, cv::Scalar(255, 100, 0), 2);
    cv::putText(img, "Obs", cv::Point(legend_x + 10, 16),
        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 100, 0), 1);

    cv::circle(img, cv::Point(legend_x + 60, 12), 5, cv::Scalar(0, 255, 255), -1);
    cv::putText(img, "EKF", cv::Point(legend_x + 70, 16),
        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 255), 1);

    cv::circle(img, cv::Point(legend_x + 120, 12), 5, cv::Scalar(0, 255, 0), -1);
    cv::putText(img, "Pred", cv::Point(legend_x + 130, 16),
        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1);
}

/**
 * @brief 启动预测器节点
 */
void start_predictor_node() {
    debug::print(debug::PrintMode::INFO, "PredictorNode", "Starting predictor node...");

    // 创建预测器
    EnemyPredictor predictor;

    // 设置 UMT
    umt::Subscriber<DetectionResult> sub("detections");
    umt::Publisher<BattlefieldSnapshot> pub("battlefield");
    umt::Publisher<cv::Mat> vis_pub("predictor_vis");  // 可视化帧 (供录制)
    umt::Publisher<cv::Mat> pub_debug("/predictor/debug");  // Web 调试图像
    auto running = umt::BasicObjManager<bool>::find_or_create("app_running", true);

    stats::FpsStats stats("PredictorNode", "tracked");

    debug::print(debug::PrintMode::INFO, "PredictorNode", "Predictor node started");

    while (running->get()) {
        watchdog::heartbeat("predictor");
        try {
            auto detection = sub.pop_for(1000);

            // 使用相机帧时间戳（从 SyncFrame 传递过来，微秒转秒）
            double timestamp = detection.state.timestamp_us / 1e6;

            // DEBUG: 输入装甲板数量
            if (!detection.armors.empty()) {
                fmt::print(fmt::fg(fmt::color::magenta),
                    "[DEBUG] Input: {} armors\n", detection.armors.size());
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

            // 发布结果
            pub.push(snapshot);

            // 调试输出
            const auto& table = predictor.get_observation_table();

            // DEBUG: 观测表状态
            if (!detection.armors.empty()) {
                int tracking = 0;
                for (int i = 1; i < MAX_TARGETS; ++i) {
                    if (snapshot.is_valid(i)) tracking++;
                }
                fmt::print(fmt::fg(fmt::color::magenta),
                    "[DEBUG] Table: {} observations, Tracking: {}\n",
                    table.total_count(), tracking);
            }

            if (table.total_count() > 0) {
                fmt::print("\n========== Frame {} ==========\n", detection.frame_id);
                print_observations(table);
                print_predictions(snapshot);
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
                draw_prediction(vis, snapshot, table, detection.state.q_imu);
                predictor.draw(vis, detection.state.q_imu, timestamp);

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
