/**
 * @file predictor_node.cpp
 * @brief 预测器节点
 *
 * 订阅: Message<aimer::DetectionResult> "detections"
 * 发布: Message<BattlefieldSnapshot> "battlefield"
 */

#include <chrono>

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
                "xyz: {:.3f}|{:.3f}|{:.3f}\n\n",
                target_id, type_str, area / 1000.0,
                yaw_deg, pitch_deg, dist,
                obs.pos.x(), obs.pos.y(), obs.pos.z()
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
    const Eigen::Quaterniond& q_imu
) {
    // 遍历所有有效车辆
    for (int i = 1; i < MAX_TARGETS; ++i) {
        if (!snapshot.is_valid(i)) continue;

        const auto& vehicle = snapshot.vehicles[i];

        // 绘制车辆中心
        bool valid = false;
        cv::Point2f center_px = tf::world_to_pixel(vehicle.center, q_imu, valid);
        if (valid && center_px.x >= 0 && center_px.x < img.cols &&
            center_px.y >= 0 && center_px.y < img.rows) {
            // 中心点
            cv::circle(img, center_px, 8, cv::Scalar(0, 255, 255), -1, cv::LINE_AA);

            // 速度方向
            Eigen::Vector3d vel_end = vehicle.center + vehicle.velocity * 0.5;  // 0.5秒后位置
            cv::Point2f vel_px = tf::world_to_pixel(vel_end, q_imu, valid);
            if (valid) {
                cv::arrowedLine(img, center_px, vel_px, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
            }

            // 标注目标编号和距离
            double dist = vehicle.center.norm();
            std::string text = fmt::format("T{} {:.1f}m", i, dist);
            cv::putText(img, text, center_px + cv::Point2f(10, -10),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
        }

        // 绘制各装甲板
        for (int j = 0; j < vehicle.armor_count; ++j) {
            const auto& armor = vehicle.armors[j];
            cv::Point2f armor_px = tf::world_to_pixel(armor.position, q_imu, valid);
            if (valid && armor_px.x >= 0 && armor_px.x < img.cols &&
                armor_px.y >= 0 && armor_px.y < img.rows) {
                // 推荐击打的用绿色，其他用蓝色
                cv::Scalar color = (j == vehicle.recommended_armor_idx)
                    ? cv::Scalar(0, 255, 0)
                    : cv::Scalar(255, 100, 0);
                cv::circle(img, armor_px, 5, color, -1, cv::LINE_AA);
            }
        }
    }

    // 显示统计信息
    int valid_count = 0;
    for (int i = 1; i < MAX_TARGETS; ++i) {
        if (snapshot.is_valid(i)) valid_count++;
    }
    std::string info = fmt::format("Predictor: {} targets tracked", valid_count);
    cv::putText(img, info, cv::Point(10, 90),
        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);
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
            if (detection.empty()) continue;

            // 转换格式
            auto det = convert_detection(detection);

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
                draw_prediction(vis, snapshot, det.q_imu);
                cv::imshow("Predictor", vis);
                cv::waitKey(1);
            }

        } catch (const umt::MessageError_Timeout&) {
            // 超时，继续
        } catch (const std::exception& e) {
            debug::print(debug::PrintMode::ERROR, "PredictorNode",
                "Exception: {}", e.what());
        }
    }

    debug::print(debug::PrintMode::INFO, "PredictorNode", "Predictor node stopped");
}

}  // namespace autoaim::predictor
