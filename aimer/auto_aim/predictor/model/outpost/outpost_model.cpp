/**
 * @file outpost_model.cpp
 * @brief 前哨站运动模型实现 (使用 EKF)
 */

#include "outpost_model.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include <fmt/format.h>
#include <opencv2/imgproc.hpp>

#include "aimer/common/fire_control_types.hpp"
#include "aimer/common/math/math.hpp"
#include "aimer/common/transformer/transformer.hpp"
#include "plugin/param/runtime_parameter.hpp"
#include "umt/BasicObjManager.hpp"

namespace autoaim::predictor {

namespace {

constexpr double OUTPOST_ARMOR_PITCH = 15.0 * M_PI / 180.0;

double get_param_or(const std::string& name, double default_value)
{
    auto ptr = runtime_param::find_param(name);
    if (ptr != nullptr) {
        if (auto* val = std::get_if<double>(&*ptr)) {
            return *val;
        }
    }
    return default_value;
}

double get_draw_predict_dt()
{
    auto fd = umt::BasicObjManager<::fire_control::FireDebugInfo>::find("fire_debug");
    if (fd) {
        const auto& dbg = fd->get();
        if (dbg.fc_heartbeat > 0 && dbg.prediction_dt > 0.001) {
            return dbg.prediction_dt + dbg.latency_control_to_fire / 1000.0;
        }
    }
    return get_param_or("AutoAim.Predictor.EKF.draw_predict_dt", 0.020);
}

void draw_armor_rect(
    cv::Mat& img,
    const Eigen::Vector3d& center,
    double yaw,
    ArmorType type,
    const Eigen::Quaterniond& q_imu,
    const cv::Scalar& color,
    int thickness
) {
    const double w = (type == ArmorType::LARGE) ? 0.225 : 0.133;
    const double h = 0.050;

    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);

    const Eigen::Vector3d x_axis(-sin_yaw, cos_yaw, 0.0);
    const Eigen::Vector3d y_axis(
        -cos_yaw * std::sin(OUTPOST_ARMOR_PITCH),
        -sin_yaw * std::sin(OUTPOST_ARMOR_PITCH),
        std::cos(OUTPOST_ARMOR_PITCH)
    );

    const std::array<Eigen::Vector3d, 4> corners = {
        center + x_axis * (w / 2.0) + y_axis * (h / 2.0),
        center + x_axis * (w / 2.0) - y_axis * (h / 2.0),
        center - x_axis * (w / 2.0) - y_axis * (h / 2.0),
        center - x_axis * (w / 2.0) + y_axis * (h / 2.0)
    };

    std::array<cv::Point2f, 4> pts;
    bool all_valid = true;
    for (int i = 0; i < 4; ++i) {
        bool valid = false;
        pts[i] = aimer::tf::world_to_pixel(corners[i], q_imu, valid);
        all_valid = all_valid && valid;
    }
    if (!all_valid) return;

    for (int i = 0; i < 4; ++i) {
        cv::line(img, pts[i], pts[(i + 1) % 4], color, thickness, cv::LINE_AA);
    }
}

void draw_armor_by_z_to_v(
    cv::Mat& img,
    const Eigen::Vector3d& pos_world,
    double z_to_v,
    ArmorType type,
    const Eigen::Quaterniond& q_imu,
    const cv::Scalar& color,
    int thickness
) {
    const double w = (type == ArmorType::LARGE) ? 0.225 : 0.133;
    const double h = 0.055;

    Eigen::Vector3d camera_z_world =
        aimer::tf::vector<aimer::tf::Frame::Camera, aimer::tf::Frame::World>(
            Eigen::Vector3d(0, 0, 1), q_imu
        );
    Eigen::Vector2d camera_z_2d(camera_z_world.x(), camera_z_world.y());
    if (camera_z_2d.norm() > 1e-6) {
        camera_z_2d.normalize();
    } else {
        camera_z_2d = Eigen::Vector2d(1.0, 0.0);
    }

    const Eigen::Vector2d normal_2d = aimer::math::rotate(camera_z_2d, z_to_v);
    const Eigen::Vector2d x_2d = aimer::math::rotate(normal_2d, M_PI / 2.0);
    const Eigen::Vector3d x_axis(x_2d.x(), x_2d.y(), 0.0);
    const Eigen::Vector3d y_axis(
        -normal_2d.x() * std::sin(OUTPOST_ARMOR_PITCH),
        -normal_2d.y() * std::sin(OUTPOST_ARMOR_PITCH),
        std::cos(OUTPOST_ARMOR_PITCH)
    );

    const std::array<Eigen::Vector3d, 4> corners = {
        pos_world + x_axis * (w / 2.0) + y_axis * (h / 2.0),
        pos_world + x_axis * (w / 2.0) - y_axis * (h / 2.0),
        pos_world - x_axis * (w / 2.0) - y_axis * (h / 2.0),
        pos_world - x_axis * (w / 2.0) + y_axis * (h / 2.0)
    };

    std::array<cv::Point2f, 4> pts;
    bool all_valid = true;
    for (int i = 0; i < 4; ++i) {
        bool valid = false;
        pts[i] = aimer::tf::world_to_pixel(corners[i], q_imu, valid);
        all_valid = all_valid && valid;
    }
    if (!all_valid) return;

    for (int i = 0; i < 4; ++i) {
        cv::line(img, pts[i], pts[(i + 1) % 4], color, thickness, cv::LINE_AA);
    }
}

}  // namespace

OutpostModel::OutpostModel(int target_id, EnemyType enemy_type)
    : target_id_(target_id)
    , enemy_type_(enemy_type) {}

void OutpostModel::update(const std::vector<ArmorObservation>& observations, double timestamp) {
    ++frame_count_;

    // 盲区处理: 没有观测也更新时间戳 (用于 alive() 判断)
    // 但不 reset，让 motion_ 继续预测
    if (observations.empty()) {
        if (initialized_ && (timestamp - last_update_time_) > LOST_TIMEOUT) {
            reset();
        }
        return;
    }

    last_update_time_ = timestamp;

    // ArmorIdentifier 分配 ID
    identifier_.update(observations, timestamp, frame_count_);
    auto armors_with_id = identifier_.get_active_armors(frame_count_);
    if (armors_with_id.empty()) return;

    // 选择最正对的装甲板 (过滤顶部装甲板)
    const ArmorData* best = nullptr;
    double best_abs_z_to_v = 1e9;
    for (const auto& armor : armors_with_id) {
        // 过滤顶部装甲板: pitch > 45° (朝上)
        if (armor.observation.z[obs::PITCH] > outpost_model::TOP_ARMOR_PITCH_THRESHOLD) {
            continue;
        }
        double abs_z_to_v = std::abs(armor.z_to_v());
        if (abs_z_to_v < best_abs_z_to_v) {
            best_abs_z_to_v = abs_z_to_v;
            best = &armor;
        }
    }

    if (!best) return;

    // 更新 EKF 运动模型
    motion_.update(*best, timestamp);
    initialized_ = motion_.valid();
}

VehicleState OutpostModel::predict(double timestamp) const {
    VehicleState vs;
    vs.target_id = target_id_;
    vs.enemy_type = enemy_type_;
    vs.valid = initialized_;

    if (!initialized_) return vs;

    // 计算预测时间差
    double dt = timestamp - last_update_time_;
    const double visible_time_window = get_param_or("AutoAim.Predictor.visible_time_window", 0.04);
    const bool visible_fresh = dt <= visible_time_window;

    // 从 EKF 模型获取预测
    Eigen::Vector3d center = motion_.predict_center(dt);
    Eigen::Vector3d velocity = motion_.get_velocity();
    double omega = motion_.get_omega();
    double theta = motion_.get_theta() + omega * dt;

    vs.center = center;
    vs.velocity = velocity;
    vs.timestamp = timestamp;
    vs.armor_count = ARMOR_NUM;

    // 陀螺状态
    vs.spin.active = true;
    vs.spin.omega = omega;
    vs.spin.phase = theta;
    vs.spin.radius = outpost_model::RADIUS;
    vs.spin.level = SpinLevel::HIGH;  // 前哨站始终高速

    // 生成 3 块装甲板位置
    for (int i = 0; i < ARMOR_NUM; ++i) {
        Eigen::Vector3d armor_pos = motion_.predict_armor_pos(i, dt);

        vs.armors[i].id = i;
        vs.armors[i].type = ArmorType::SMALL;
        vs.armors[i].position = armor_pos;
        vs.armors[i].velocity = velocity;  // 继承中心速度
        vs.armors[i].visible = visible_fresh;
        vs.armors[i].last_seen = last_update_time_;

        // 装甲板朝向 = 中心到装甲板的方向
        double armor_theta = theta + i * (2.0 * M_PI / 3.0);
        vs.armors[i].yaw = armor_theta;

        // 评分: 当前槽位得分最高
        vs.armors[i].score = (i == motion_.get_current_slot()) ? 1.0 : 0.5;
    }

    // 推荐装甲板 = 当前追踪的槽位
    vs.recommended_armor_idx = motion_.get_current_slot();

    return vs;
}

bool OutpostModel::alive() const {
    return initialized_;
}

void OutpostModel::reset() {
    initialized_ = false;
    motion_.reset();
    identifier_.reset();
    frame_count_ = 0;
}

void OutpostModel::draw(cv::Mat& img, const Eigen::Quaterniond& q_imu, double timestamp) const {
    if (!initialized_ || !motion_.valid()) return;

    const double time_since_update = timestamp - last_update_time_;
    const bool data_fresh = time_since_update < 0.1;
    const double draw_dt = get_draw_predict_dt();

    const cv::Scalar color_detected(0, 255, 0);
    const cv::Scalar color_z_to_v(0, 255, 255);
    const cv::Scalar color_predicted(0, 255, 255);
    const cv::Scalar color_center(255, 0, 255);
    const cv::Scalar color_current(0, 80, 255);

    std::vector<ArmorData> active_armors;
    if (data_fresh) {
        active_armors = identifier_.get_active_armors(frame_count_);

        for (const auto& armor : active_armors) {
            const auto& obs = armor.observation;
            if (obs.pts.size() < 4) continue;

            cv::line(img, obs.pts[0], obs.pts[2], color_detected, 2, cv::LINE_AA);
            cv::line(img, obs.pts[1], obs.pts[3], color_detected, 2, cv::LINE_AA);
            draw_armor_by_z_to_v(img, obs.pos, obs.z_to_v, obs.type, q_imu, color_z_to_v, 2);
        }
    }

    const int current_slot = motion_.get_current_slot();
    const double omega = motion_.get_omega();
    const double theta_predicted = motion_.get_theta() + omega * draw_dt;

    for (int i = 0; i < ARMOR_NUM; ++i) {
        const Eigen::Vector3d pos = motion_.predict_armor_pos(i, draw_dt);
        const double armor_yaw = theta_predicted + i * (2.0 * M_PI / ARMOR_NUM);
        const bool current = (i == current_slot);
        const cv::Scalar color = current ? color_current : color_predicted;
        const int thickness = current ? 3 : 1;

        draw_armor_rect(img, pos, armor_yaw, ArmorType::SMALL, q_imu, color, thickness);

        bool valid = false;
        const cv::Point2f pt = aimer::tf::world_to_pixel(pos, q_imu, valid);
        if (valid) {
            cv::putText(
                img,
                fmt::format("O{}{}", i, current ? "*" : ""),
                pt + cv::Point2f(8, -8),
                cv::FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                1,
                cv::LINE_AA
            );
        }
    }

    const Eigen::Vector3d center = motion_.predict_center(draw_dt);
    const Eigen::Vector3d velocity = motion_.get_velocity();
    bool center_valid = false;
    const cv::Point2f center_px = aimer::tf::world_to_pixel(center, q_imu, center_valid);
    if (center_valid) {
        const int s = 12;
        cv::line(
            img, center_px - cv::Point2f(s, 0), center_px + cv::Point2f(s, 0),
            color_center, 2, cv::LINE_AA
        );
        cv::line(
            img, center_px - cv::Point2f(0, s), center_px + cv::Point2f(0, s),
            color_center, 2, cv::LINE_AA
        );

        std::vector<cv::Point> diamond = {
            cv::Point(center_px.x, center_px.y - s - 3),
            cv::Point(center_px.x + s + 3, center_px.y),
            cv::Point(center_px.x, center_px.y + s + 3),
            cv::Point(center_px.x - s - 3, center_px.y)
        };
        cv::polylines(img, diamond, true, color_center, 2, cv::LINE_AA);

        const Eigen::Vector3d tip_world = center + velocity * 0.15;
        bool tip_valid = false;
        const cv::Point2f tip_px = aimer::tf::world_to_pixel(tip_world, q_imu, tip_valid);
        if (tip_valid) {
            cv::arrowedLine(img, center_px, tip_px, color_center, 2, cv::LINE_AA, 0, 0.25);
        }

        const double theta_deg = motion_.get_theta() * 180.0 / M_PI;
        const double omega_deg = omega * 180.0 / M_PI;
        const std::array<std::string, 5> lines = {
            fmt::format("[T{}] OUTPOST | EKF ON", target_id_),
            fmt::format("slot:{} | w:{:+.0f} d/s | th:{:.0f}", current_slot, omega_deg, theta_deg),
            fmt::format("r:{:.2f} | dt:{:.1f}ms | lost:{:.1f}ms",
                outpost_model::RADIUS, draw_dt * 1000.0, time_since_update * 1000.0),
            fmt::format("pos: ({:.2f}, {:.2f}, {:.2f})", center.x(), center.y(), center.z()),
            fmt::format("vel: ({:.2f}, {:.2f}, {:.2f})", velocity.x(), velocity.y(), velocity.z())
        };

        const cv::Point2f text_base = center_px + cv::Point2f(-100, -105);
        for (size_t i = 0; i < lines.size(); ++i) {
            cv::putText(
                img,
                lines[i],
                text_base + cv::Point2f(0, static_cast<float>(i) * 16.0f),
                cv::FONT_HERSHEY_SIMPLEX,
                0.45,
                color_center,
                1,
                cv::LINE_AA
            );
        }
    }

    if (!data_fresh || active_armors.empty()) return;

    float avg_x = 0.0f;
    for (const auto& armor : active_armors) {
        avg_x += armor.observation.center_2d.x;
    }
    avg_x /= static_cast<float>(active_armors.size());

    for (size_t idx = 0; idx < active_armors.size(); ++idx) {
        const auto& armor = active_armors[idx];
        const auto& obs = armor.observation;
        if (obs.pts.size() < 4) continue;

        const double area_k = aimer::math::get_area(obs.pts) / 1000.0;
        const std::array<std::string, 5> lines = {
            fmt::format("outpost obs:{} | id:{}", idx, armor.id),
            fmt::format("z_to_v:{:.1f} | area:{:.1f}k", obs.z_to_v * 57.3, area_k),
            fmt::format("x:{:.2f} | y:{:.2f} | z:{:.2f}", obs.pos.x(), obs.pos.y(), obs.pos.z()),
            fmt::format("yaw:{:.1f} | pitch:{:.1f}", obs.z[obs::YAW] * 57.3, obs.z[obs::PITCH] * 57.3),
            fmt::format("dist:{:.2f} | slot:{}", obs.z[obs::DIST], current_slot)
        };

        cv::Point2f base;
        if (active_armors.size() == 1) {
            base = obs.center_2d + cv::Point2f(-100, 70);
        } else if (obs.center_2d.x < avg_x) {
            base = obs.center_2d + cv::Point2f(-160, 70);
        } else {
            base = obs.center_2d + cv::Point2f(20, 70);
        }

        for (size_t line_idx = 0; line_idx < lines.size(); ++line_idx) {
            cv::putText(
                img,
                lines[line_idx],
                base + cv::Point2f(0, static_cast<float>(line_idx) * 18.0f),
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                color_detected,
                1,
                cv::LINE_AA
            );
        }
    }
}

}  // namespace autoaim::predictor
