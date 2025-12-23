/**
 * @file predictor_node.cpp
 * @brief 预测器节点
 *
 * 订阅: Message<aimer::DetectionResult> "detections"
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
 * @brief 转换检测结果格式
 */
autoaim::DetectionResult convert_detection(const aimer::DetectionResult& det) {
    autoaim::DetectionResult result;
    result.q_imu = det.state.q_imu;
    result.timestamp = SteadyClock::now().time_since_epoch().count() / 1e9;  // 用当前时间
    result.armors = det.armors;
    result.frame_id = det.frame_id;
    result.img = det.img;  // 传递图像
    return result;
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
    // ========== 1. 绘制原始观测 (蓝色) ==========
    for (int target_id : table.get_target_ids()) {
        const auto& obs_list = table.get(target_id);
        for (const auto& obs : obs_list) {
            if (!obs.valid) continue;

            // 在图像上绘制观测位置
            bool valid = false;
            cv::Point2f obs_px = tf::world_to_pixel(obs.pos, q_imu, valid);
            if (!valid) continue;

            // 绘制观测点 (蓝色圆圈)
            cv::circle(img, obs_px, 6, cv::Scalar(255, 100, 0), 2, cv::LINE_AA);

            // 计算面积
            double area = 0;
            if (obs.pts.size() >= 4) {
                area = std::abs(
                    (obs.pts[0].x - obs.pts[2].x) * (obs.pts[1].y - obs.pts[3].y) -
                    (obs.pts[1].x - obs.pts[3].x) * (obs.pts[0].y - obs.pts[2].y)
                ) / 2.0;
            }

            // 球坐标
            double yaw_deg = obs.z[0] * 180.0 / M_PI;
            double pitch_deg = obs.z[1] * 180.0 / M_PI;
            double dist = obs.z[2];

            // 绘制观测信息 (右侧)
            const char* type_str = (obs.type == ArmorType::LARGE) ? "L" : "S";
            int y_offset = 0;
            cv::Point2f text_pos = obs_px + cv::Point2f(15, -30);

            // 编号和类型
            cv::putText(img, fmt::format("#{} {}", target_id, type_str),
                text_pos + cv::Point2f(0, y_offset),
                cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 200, 100), 1, cv::LINE_AA);
            y_offset += 15;

            // 面积
            cv::putText(img, fmt::format("A:{:.1f}k", area / 1000.0),
                text_pos + cv::Point2f(0, y_offset),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
            y_offset += 13;

            // YPD (球坐标)
            cv::putText(img, fmt::format("Y:{:.1f} P:{:.1f}", yaw_deg, pitch_deg),
                text_pos + cv::Point2f(0, y_offset),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(100, 255, 255), 1, cv::LINE_AA);
            y_offset += 13;

            // 距离
            cv::putText(img, fmt::format("D:{:.2f}m", dist),
                text_pos + cv::Point2f(0, y_offset),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(100, 255, 255), 1, cv::LINE_AA);
            y_offset += 13;

            // XYZ (世界坐标)
            cv::putText(img, fmt::format("({:.2f},{:.2f},{:.2f})",
                obs.pos.x(), obs.pos.y(), obs.pos.z()),
                text_pos + cv::Point2f(0, y_offset),
                cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(150, 255, 150), 1, cv::LINE_AA);
        }
    }

    // ========== 2. 绘制 EKF 预测结果 (黄色/绿色) ==========
    for (int i = 1; i < MAX_TARGETS; ++i) {
        if (!snapshot.is_valid(i)) continue;

        const auto& vehicle = snapshot.vehicles[i];

        // 绘制车辆中心 (黄色)
        bool valid = false;
        cv::Point2f center_px = tf::world_to_pixel(vehicle.center, q_imu, valid);
        if (valid && center_px.x >= 0 && center_px.x < img.cols &&
            center_px.y >= 0 && center_px.y < img.rows) {

            // 中心点 (黄色实心圆)
            cv::circle(img, center_px, 10, cv::Scalar(0, 255, 255), -1, cv::LINE_AA);
            cv::circle(img, center_px, 10, cv::Scalar(0, 200, 200), 2, cv::LINE_AA);

            // 速度方向箭头 (绿色)
            double vel_scale = 0.3;  // 0.3秒后位置
            Eigen::Vector3d vel_end = vehicle.center + vehicle.velocity * vel_scale;
            cv::Point2f vel_px = tf::world_to_pixel(vel_end, q_imu, valid);
            if (valid) {
                cv::arrowedLine(img, center_px, vel_px, cv::Scalar(0, 255, 0), 2, cv::LINE_AA, 0, 0.3);
            }

            // 标注预测信息 (左侧)
            cv::Point2f text_pos = center_px + cv::Point2f(-120, -20);
            int y_offset = 0;

            // 目标编号
            cv::putText(img, fmt::format("T{} [EKF]", i),
                text_pos + cv::Point2f(0, y_offset),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
            y_offset += 16;

            // 距离
            double dist = vehicle.center.norm();
            cv::putText(img, fmt::format("dist: {:.2f}m", dist),
                text_pos + cv::Point2f(0, y_offset),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
            y_offset += 14;

            // 速度
            double speed = vehicle.velocity.norm();
            cv::putText(img, fmt::format("vel: {:.2f}m/s", speed),
                text_pos + cv::Point2f(0, y_offset),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
            y_offset += 14;

            // 位置
            cv::putText(img, fmt::format("xyz: ({:.2f},{:.2f},{:.2f})",
                vehicle.center.x(), vehicle.center.y(), vehicle.center.z()),
                text_pos + cv::Point2f(0, y_offset),
                cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
        }

        // 绘制各装甲板预测位置 (绿色小圆) + 信息
        for (int j = 0; j < vehicle.armor_count; ++j) {
            const auto& armor = vehicle.armors[j];
            cv::Point2f armor_px = tf::world_to_pixel(armor.position, q_imu, valid);
            if (valid && armor_px.x >= 0 && armor_px.x < img.cols &&
                armor_px.y >= 0 && armor_px.y < img.rows) {
                // 推荐击打的用绿色实心，其他用绿色空心
                bool is_best = (j == vehicle.recommended_armor_idx);
                cv::Scalar color = is_best ? cv::Scalar(0, 255, 0) : cv::Scalar(100, 255, 100);
                int thickness = is_best ? -1 : 2;
                cv::circle(img, armor_px, 10, color, thickness, cv::LINE_AA);

                // 在圆圈下方显示: "T{target_id}.{armor_id} v={vel}"
                double vel = armor.velocity.norm();
                std::string info = fmt::format("T{}.{} v:{:.1f}", i, armor.id, vel);
                cv::Point2f text_pos = armor_px + cv::Point2f(-30, 25);
                // 黑色描边
                cv::putText(img, info, text_pos + cv::Point2f(1, 1),
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
                // 黄色文字
                cv::putText(img, info, text_pos,
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
            }
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

    // 获取相机内参 (从 transformer 模块)
    const cv::Mat& camera_matrix = tf::get_camera_matrix();
    const cv::Mat& dist_coeffs = tf::get_distort_coeffs();

    // 创建预测器
    EnemyPredictor predictor;
    predictor.set_camera_params(camera_matrix, dist_coeffs);

    // 设置 UMT
    umt::Subscriber<aimer::DetectionResult> sub("detections");
    umt::Publisher<BattlefieldSnapshot> pub("battlefield");
    auto running = umt::BasicObjManager<bool>::find_or_create("predictor_running", true);

    stats::FpsStats stats("PredictorNode", "tracked");

    debug::print(debug::PrintMode::INFO, "PredictorNode", "Predictor node started");

    while (running->get()) {
        try {
            auto detection = sub.pop_for(1000);
            // 即使没有检测到装甲板也要调用 predict，让 EKF 衰减

            // 转换格式
            auto det = convert_detection(detection);

            // DEBUG: 输入装甲板数量
            if (!det.armors.empty()) {
                fmt::print(fmt::fg(fmt::color::magenta),
                    "[DEBUG] Input: {} armors\n", det.armors.size());
            }

            // 运行预测
            auto predict_start = SteadyClock::now();
            auto snapshot = predictor.predict(det);
            auto predict_end = SteadyClock::now();

            float latency = std::chrono::duration_cast<std::chrono::microseconds>(
                predict_end - predict_start).count() / 1000.0f;

            // 发布结果
            pub.push(snapshot);

            // 调试输出
            const auto& table = predictor.get_observation_table();

            // DEBUG: 观测表状态
            if (!det.armors.empty()) {
                int tracking = 0;
                for (int i = 1; i < MAX_TARGETS; ++i) {
                    if (snapshot.is_valid(i)) tracking++;
                }
                fmt::print(fmt::fg(fmt::color::magenta),
                    "[DEBUG] Table: {} observations, Tracking: {}\n",
                    table.total_count(), tracking);
            }

            if (table.total_count() > 0) {
                fmt::print("\n========== Frame {} ==========\n", det.frame_id);
                print_observations(table);
                print_predictions(snapshot);
            }

            // 统计
            int tracked = 0;
            for (int i = 1; i < MAX_TARGETS; ++i) {
                if (snapshot.is_valid(i)) tracked++;
            }
            stats.tick(latency, tracked > 0);

            // 可视化 (如果有图像)
            if (!det.img.empty()) {
                cv::Mat vis = det.img.clone();
                draw_prediction(vis, snapshot, table, det.q_imu);
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
