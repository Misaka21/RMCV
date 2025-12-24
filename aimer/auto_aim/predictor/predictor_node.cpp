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
#include "plugin/stats/fps_stats.hpp"
#include "umt/umt.hpp"

namespace autoaim::predictor {

using SteadyClock = std::chrono::steady_clock;

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

            // 装甲板类型
            const char* type_str = (obs.type == ArmorType::LARGE) ? "L" : "S";

            fmt::print(fmt::fg(fmt::color::cyan),
                "number: {} type: {} area: {:.1f}k\n"
                "ypd: {:.1f}|{:.1f}|{:.3f}\n"
                "xyz: {:.3f}|{:.3f}|{:.3f}\n"
                "z_to_v: {:.3f} rad ({:.1f} deg)\n\n",
                target_id, type_str, area / 1000.0,
                yaw_deg, pitch_deg, dist,
                obs.pos.x(), obs.pos.y(), obs.pos.z(),
                obs.z_to_v, obs.z_to_v * 180.0 / M_PI
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
    // ========== 1. 绘制原始观测 (蓝色小圆圈，不显示文字) ==========
    for (int target_id : table.get_target_ids()) {
        const auto& obs_list = table.get(target_id);
        for (const auto& obs : obs_list) {
            if (!obs.valid) continue;

            bool valid = false;
            cv::Point2f obs_px = tf::world_to_pixel(obs.pos, q_imu, valid);
            if (!valid) continue;

            // 蓝色小圆圈标记观测位置
            cv::circle(img, obs_px, 5, cv::Scalar(255, 100, 0), 2, cv::LINE_AA);
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
    auto running = umt::BasicObjManager<bool>::find_or_create("predictor_running", true);

    stats::FpsStats stats("PredictorNode", "tracked");

    debug::print(debug::PrintMode::INFO, "PredictorNode", "Predictor node started");

    while (running->get()) {
        try {
            auto detection = sub.pop_for(1000);

            // 使用相机帧时间戳（从 SyncFrame 传递过来）
            double timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                detection.state.timestamp.time_since_epoch()
            ).count() / 1e9;

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

            // 可视化 (如果有图像)
            if (!detection.img.empty()) {
                cv::Mat vis = detection.img.clone();
                draw_prediction(vis, snapshot, table, detection.state.q_imu);
                // 调用各模型的 draw 方法 (绘制 X 和 □)
                predictor.draw(vis, detection.state.q_imu, timestamp);

                // 发布可视化帧供录制
                vis_pub.push(vis);

                cv::imshow("Predictor", vis);
                cv::waitKey(1);
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
