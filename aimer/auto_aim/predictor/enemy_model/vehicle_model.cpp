/**
 * @file vehicle_model.cpp
 * @brief 车辆运动模型实现
 *
 * 过滤逻辑参考 rm.cv.fans/aimer/auto_aim/predictor/enemy/enemy_state.cpp
 */

#include "vehicle_model.hpp"

#include <algorithm>
#include <cmath>

#include <fmt/color.h>
#include <opencv2/imgproc.hpp>

#include "aimer/common/math/math.hpp"
#include "aimer/common/transformer/transformer.hpp"
#include "plugin/param/runtime_parameter.hpp"

namespace autoaim::predictor {

// ============================================================================
// 辅助函数: 读取运行时参数
// ============================================================================

namespace {

// 默认参数值
constexpr double DEFAULT_EXISTING_ARMOR_AREA_RATIO = 0.30;
constexpr double DEFAULT_NEW_ARMOR_AREA_RATIO = 0.40;
constexpr double DEFAULT_JUMP_DISTANCE_LIMIT = 1.2;
constexpr double DEFAULT_NEW_ARMOR_MAX_DIST = 10.0;
constexpr double DEFAULT_LOST_TIMEOUT = 0.5;
constexpr double DEFAULT_ARMOR_CREDIT_TIME = 0.1;

// 辅助函数: 安全读取运行时参数，找不到时返回默认值
double get_double_param(const std::string& name, double default_val) {
    auto ptr = runtime_param::find_param(name);
    if (ptr != nullptr) {
        if (auto* val = std::get_if<double>(&*ptr)) {
            return *val;
        }
    }
    return default_val;
}

double get_existing_armor_area_ratio() {
    return get_double_param("AutoAim.Predictor.existing_armor_area_ratio", DEFAULT_EXISTING_ARMOR_AREA_RATIO);
}

double get_new_armor_area_ratio() {
    return get_double_param("AutoAim.Predictor.new_armor_area_ratio", DEFAULT_NEW_ARMOR_AREA_RATIO);
}

double get_jump_distance_limit() {
    return get_double_param("AutoAim.Predictor.jump_distance_limit", DEFAULT_JUMP_DISTANCE_LIMIT);
}

double get_new_armor_max_distance() {
    return get_double_param("AutoAim.Predictor.new_armor_max_distance", DEFAULT_NEW_ARMOR_MAX_DIST);
}

double get_lost_timeout() {
    return get_double_param("AutoAim.Predictor.lost_timeout", DEFAULT_LOST_TIMEOUT);
}

double get_armor_credit_time() {
    return get_double_param("AutoAim.Predictor.armor_credit_time", DEFAULT_ARMOR_CREDIT_TIME);
}

double get_draw_predict_dt() {
    return get_double_param("AutoAim.Predictor.EKF.draw_predict_dt", 0.020);
}

}  // namespace

// ============================================================================
// VehicleModel 实现
// ============================================================================

VehicleModel::VehicleModel(int target_id, EnemyType enemy_type)
    : target_id_(target_id),
      enemy_type_(enemy_type),
      armor_motion_(get_armor_credit_time()),
      spin_motion_(enemy_type == EnemyType::OUTPOST ? 3 : 4) {}

void VehicleModel::update(const std::vector<ArmorObservation>& observations, double timestamp) {
    ++frame_count_;

    // 1. 消抖过滤 (参考 rm.cv.fans screened_armors)
    auto filtered = filter(observations, prev_armors_);

    if (filtered.empty()) {
        if (initialized_ && (timestamp - last_update_time_) > get_lost_timeout()) {
            reset();
        }
        return;
    }

    last_update_time_ = timestamp;

    // 2. ArmorIdentifier: ID 分配
    identifier_.update(filtered, timestamp, frame_count_);

    // 3. ArmorMotion: EKF 滤波
    auto armors_with_id = identifier_.get_active_armors(frame_count_);
    armor_motion_.update(armors_with_id, timestamp);

    // 4. SpinMotion: 整车 EKF 滤波
    if (!armors_with_id.empty()) {
        // 获取当前最佳装甲板 (按 z_to_v 排序后的第一个)
        const auto& best_armor = armors_with_id[0];
        int current_id = best_armor.id;

        // 检测跳变: ID 变化 + 角度匹配验证
        if (last_tracking_id_ >= 0 && current_id != last_tracking_id_ && spin_motion_.valid()) {
            // 计算跳变索引 (用角度匹配)
            double theta_pred = spin_motion_.get_theta();
            double armor_yaw = best_armor.observation.z[obs::ARMOR_YAW];

            int armor_num = (enemy_type_ == EnemyType::OUTPOST) ? 3 : 4;
            double angle_step = 2.0 * M_PI / armor_num;

            int best_index = 0;
            double min_diff = std::abs(math::angle_diff(theta_pred, armor_yaw));

            for (int i = 1; i < armor_num; ++i) {
                double possible_theta = theta_pred + i * angle_step;
                double diff = std::abs(math::angle_diff(possible_theta, armor_yaw));
                if (diff < min_diff) {
                    min_diff = diff;
                    best_index = i;
                }
            }

            // 通知 SpinMotion 发生跳变
            if (best_index > 0) {
                spin_motion_.notify_jump(best_index, best_armor);
            }
        }

        // 更新追踪 ID
        last_tracking_id_ = current_id;

        // 更新 SpinMotion
        spin_motion_.update(armors_with_id, timestamp);
    }

    // DEBUG: 输出装甲板数量和 ID
    if (armors_with_id.size() > 1 || armor_motion_.size() > 1) {
        fmt::print(fmt::fg(fmt::color::orange),
            "[T{}] obs:{} filtered:{} active:{} filters:{}\n",
            target_id_, observations.size(), filtered.size(),
            armors_with_id.size(), armor_motion_.size());
        for (const auto& a : armors_with_id) {
            fmt::print("  armor id={} pos=({:.2f},{:.2f},{:.2f})\n",
                a.id, a.pos().x(), a.pos().y(), a.pos().z());
        }
    }

    if (!initialized_) {
        initialized_ = true;
    }

    // 5. 更新陀螺状态
    spin_.omega = spin_motion_.get_omega();
    spin_.phase = spin_motion_.get_theta();
    spin_.radius = spin_motion_.get_radius();
    spin_.update_level(spin_.omega);

    // 6. 更新敌方颜色 (用于绘图)
    if (!filtered.empty()) {
        enemy_color_ = filtered[0].color;
    }

    prev_armors_ = filtered;
    prev_timestamp_ = timestamp;
}

/**
 * @brief 过滤无效观测 (参考 rm.cv.fans EnemyState::screened_armors)
 *
 * 过滤规则:
 * 1. 颜色消抖: 新目标灰色丢弃，已跟踪目标灰色保留（灯条闪烁）
 * 2. 面积过小: 已存在装甲板 < max_area * 0.30, 新装甲板 < max_area * 0.40
 * 3. 距离过远: 新装甲板 > 10.0m
 * 4. 跳变过大: 相邻帧位置跳变 > 1.2m
 *
 * 注意: 不检查 MIN_DIST 和 z_to_v (与 rm.cv.fans 保持一致)
 */
std::vector<ArmorObservation> VehicleModel::filter(
    const std::vector<ArmorObservation>& raw,
    const std::vector<ArmorObservation>& last
) const {
    std::vector<ArmorObservation> result;

    if (raw.empty()) return result;

    // 读取运行时参数
    const double existing_area_ratio = get_existing_armor_area_ratio();
    const double new_area_ratio = get_new_armor_area_ratio();
    const double jump_limit = get_jump_distance_limit();
    const double new_max_dist = get_new_armor_max_distance();

    // 找最大面积
    double max_area = 0;
    for (const auto& a : raw) {
        if (!a.valid) continue;
        double area = math::get_area(a.pts);
        if (area > max_area) max_area = area;
    }

    for (const auto& a : raw) {
        if (!a.valid) continue;

        // 规则0: 颜色消抖
        // - 灰色 (GRAY) + 新目标 → 丢弃（防止误跟踪）
        // - 灰色 (GRAY) + 已跟踪目标 → 保留（灯条闪烁消抖）
        if (a.color == EnemyColor::GRAY && !initialized_) {
            continue;
        }

        double area = math::get_area(a.pts);

        // 找到与上一帧最近的距离
        double closest = 1e9;
        for (const auto& o : last) {
            double d = (a.pos - o.pos).norm();
            if (d < closest) closest = d;
        }

        // 规则1: 相邻帧跳变过大
        if (!last.empty() && closest > jump_limit) {
            continue;
        }

        // 判断是否是已存在的装甲板 (用位置匹配，因为 ID 还未分配)
        bool is_existing = !last.empty() && closest < 0.5;

        // 规则2: 面积过小
        if (is_existing) {
            if (area < max_area * existing_area_ratio) {
                continue;
            }
        } else {
            // 新装甲板
            if (area < max_area * new_area_ratio) {
                continue;
            }
            // 规则3: 新装甲板距离过远
            if (a.distance() > new_max_dist) {
                continue;
            }
        }

        result.push_back(a);
    }

    // 按 z_to_v 排序 (正对的优先)
    std::sort(result.begin(), result.end(), [](const auto& a, const auto& b) {
        return a.z_to_v < b.z_to_v;
    });

    // 最多保留 4 块装甲板
    if (result.size() > 4) {
        result.resize(4);
    }

    return result;
}

VehicleState VehicleModel::predict(double timestamp) const {
    VehicleState vs;
    vs.target_id = target_id_;
    vs.enemy_type = enemy_type_;
    vs.valid = initialized_;
    vs.timestamp = timestamp;
    vs.frame_count = frame_count_;
    vs.spin = spin_;

    if (!initialized_) return vs;

    double dt = timestamp - last_update_time_;

    // 用于置信度计算的变量
    double best_score = 0;

    // 根据陀螺等级选择模型
    if (spin_motion_.get_spin_level() >= SpinLevel::LOW && spin_motion_.valid()) {
        // ========== 陀螺模式: 用 SpinMotion ==========
        vs.center = spin_motion_.predict_center(dt);
        vs.velocity = spin_motion_.get_velocity();

        // 获取当前追踪装甲板的 ID (从 ArmorMotion)
        int tracking_id = 0;
        const auto* best_filter = armor_motion_.get_best(timestamp);
        if (best_filter) {
            tracking_id = best_filter->id();
        }

        // 预测所有装甲板位置
        int armor_num = (enemy_type_ == EnemyType::OUTPOST) ? 3 : 4;
        vs.armor_count = armor_num;

        double local_best_score = -1;
        int best_idx = -1;

        for (int i = 0; i < armor_num; ++i) {
            auto& as = vs.armors[i];
            as.id = (i == 0) ? tracking_id : -1;  // 只有当前追踪的有 ID
            as.position = spin_motion_.predict_armor_pos(i, dt);
            as.velocity = vs.velocity;  // 近似用中心速度
            as.yaw = spin_motion_.get_theta() + i * (2.0 * M_PI / armor_num);
            as.visible = (i == 0);  // 只有当前追踪的可见
            as.last_seen = last_update_time_;

            // 评分: 越正对越好 (用 cos(装甲板朝向 - 视线方向))
            double armor_yaw = as.yaw;
            double view_yaw = std::atan2(as.position.y(), as.position.x());
            double angle_diff = std::abs(math::reduced_angle(armor_yaw - view_yaw - M_PI));
            as.score = std::cos(angle_diff);

            if (as.score > local_best_score) {
                local_best_score = as.score;
                best_idx = i;
            }
        }

        vs.recommended_armor_idx = best_idx;
        best_score = std::max(0.0, local_best_score);

    } else {
        // ========== 普通模式: 用 ArmorMotion ==========
        auto armor_states = armor_motion_.get_armor_states(timestamp);

        vs.armor_count = static_cast<int>(std::min(armor_states.size(), size_t(MAX_ARMORS_PER_TARGET)));

        double local_best_score = -1;
        int best_idx = -1;
        Eigen::Vector3d center_sum = Eigen::Vector3d::Zero();
        Eigen::Vector3d vel_sum = Eigen::Vector3d::Zero();
        int valid_count = 0;

        for (int i = 0; i < vs.armor_count; ++i) {
            vs.armors[i] = armor_states[i];

            center_sum += armor_states[i].position;
            vel_sum += armor_states[i].velocity;
            ++valid_count;

            if (armor_states[i].score > local_best_score) {
                local_best_score = armor_states[i].score;
                best_idx = i;
            }
        }

        if (valid_count > 0) {
            vs.center = center_sum / valid_count;
            vs.velocity = vel_sum / valid_count;
        }

        vs.recommended_armor_idx = best_idx;
        best_score = std::max(0.0, local_best_score);
    }

    // ========== 计算置信度 ==========
    // 综合考虑: 装甲板朝向分数 + 时间衰减 + 连续观测帧数
    // 1. 时间衰减: 距上次观测越久越不可靠
    double time_decay = std::exp(-dt / 0.5);  // 0.5秒时间常数，0.5s后衰减到37%
    // 2. 帧数因子: 连续观测帧数越多越可靠
    double frame_factor = std::min(1.0, static_cast<double>(frame_count_) / 5.0);  // 5帧后满分
    // 3. 综合置信度
    vs.confidence = best_score * time_decay * frame_factor;

    return vs;
}

bool VehicleModel::alive() const {
    return initialized_;
}

void VehicleModel::reset() {
    initialized_ = false;
    identifier_.reset();
    armor_motion_.reset();
    spin_motion_.reset();
    prev_armors_.clear();
    spin_.reset();
    frame_count_ = 0;
    last_tracking_id_ = -1;
}

// ============================================================================
// 绘图
// ============================================================================

namespace {

// 绘制装甲板矩形框 (口字形)
void draw_armor_rect(cv::Mat& img, const Eigen::Vector3d& center, double yaw,
                     ArmorType type, const Eigen::Quaterniond& q_imu,
                     const cv::Scalar& color, int thickness = 2) {
    // 装甲板尺寸
    double w = (type == ArmorType::LARGE) ? 0.225 : 0.133;
    double h = 0.050;

    // 装甲板俯仰角: -15° 表示装甲板上沿向后倾斜
    constexpr double pitch = -15.0 * M_PI / 180.0;

    // 法向量在 XY 平面的投影 (用于计算 x_axis 和 y_axis)
    double cos_yaw = std::cos(yaw);
    double sin_yaw = std::sin(yaw);

    // 装甲板 X 轴 (水平方向，垂直于法向量)
    Eigen::Vector3d x_axis(-sin_yaw, cos_yaw, 0);

    // 装甲板 Y 轴 (竖直方向，考虑俯仰角)
    Eigen::Vector3d y_axis(
        -cos_yaw * std::sin(pitch),
        -sin_yaw * std::sin(pitch),
        std::cos(pitch)
    );

    // 四个角点: 左上、左下、右下、右上 (逆时针)
    std::array<Eigen::Vector3d, 4> corners = {
        center + x_axis * (w / 2) + y_axis * (h / 2),  // 左上
        center + x_axis * (w / 2) - y_axis * (h / 2),  // 左下
        center - x_axis * (w / 2) - y_axis * (h / 2),  // 右下
        center - x_axis * (w / 2) + y_axis * (h / 2)   // 右上
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

    // 画矩形 (口字形)
    for (int i = 0; i < 4; ++i) {
        cv::line(img, pts[i], pts[(i + 1) % 4], color, thickness);
    }
}

}  // namespace

void VehicleModel::draw(cv::Mat& img, const Eigen::Quaterniond& q_imu, double timestamp) const {
    if (!initialized_) return;

    // 颜色定义
    const cv::Scalar COLOR_DETECTED(0, 255, 0);    // 绿色: 检测到的
    const cv::Scalar COLOR_FILTERED(255, 200, 0);  // 蓝色: 滤波后的
    const cv::Scalar COLOR_CENTER(0, 0, 255);      // 红色: 旋转中心

    // SpinMotion 预测颜色: 未使用灰色，使用时用我方颜色
    cv::Scalar COLOR_SPIN;
    bool spin_active = spin_motion_.get_spin_level() >= SpinLevel::LOW && spin_motion_.valid();
    if (!spin_active) {
        COLOR_SPIN = cv::Scalar(128, 128, 128);  // 灰色: SpinMotion 未使用
    } else {
        // 我方颜色 (与敌方相反)
        if (enemy_color_ == EnemyColor::RED) {
            COLOR_SPIN = cv::Scalar(255, 0, 0);  // 蓝色 (BGR)
        } else if (enemy_color_ == EnemyColor::BLUE) {
            COLOR_SPIN = cv::Scalar(0, 0, 255);  // 红色 (BGR)
        } else {
            COLOR_SPIN = cv::Scalar(0, 255, 255);  // 黄色: 未知颜色
        }
    }

    // 1. 绘制检测到的装甲板 (X 形状: 对角线连接)
    for (const auto& obs : prev_armors_) {
        if (obs.pts.size() >= 4) {
            // X 形状: 连接对角线
            cv::line(img, obs.pts[0], obs.pts[2], COLOR_DETECTED, 2);  // 左上-右下
            cv::line(img, obs.pts[1], obs.pts[3], COLOR_DETECTED, 2);  // 左下-右上
            // 标注 target_id
            cv::putText(img, std::to_string(target_id_),
                        obs.center_2d + cv::Point2f(10, -10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, COLOR_DETECTED, 2);
        }
    }

    // 2. 绘制 ArmorMotion 滤波后的位置 (口字形)
    double draw_dt = get_draw_predict_dt();
    auto armor_states = armor_motion_.get_armor_states(timestamp);  // 用原始时间获取
    for (auto& as : armor_states) {
        // 手动外推位置
        Eigen::Vector3d predicted_pos = as.position + as.velocity * draw_dt;
        draw_armor_rect(img, predicted_pos, as.yaw, as.type, q_imu, COLOR_FILTERED, 2);
        // 标注 armor_id
        bool valid = false;
        cv::Point2f pt = tf::world_to_pixel(predicted_pos, q_imu, valid);
        if (valid) {
            cv::putText(img, "A" + std::to_string(as.id),
                        pt + cv::Point2f(15, 5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, COLOR_FILTERED, 1);
        }
    }

    // 3. 绘制 SpinMotion 预测 (口字形) - EKF 滤波后的
    // 即使未激活也绘制，方便调试
    if (spin_motion_.valid()) {
        int armor_num = (enemy_type_ == EnemyType::OUTPOST) ? 3 : 4;
        double theta = spin_motion_.get_theta() + spin_motion_.get_omega() * draw_dt;

        // 绘制所有装甲板预测位置
        for (int i = 0; i < armor_num; ++i) {
            Eigen::Vector3d pos = spin_motion_.predict_armor_pos(i, draw_dt);
            double armor_yaw = theta + i * (2.0 * M_PI / armor_num);

            // 当前追踪的装甲板用粗线
            int thickness = (i == 0) ? 3 : 1;
            draw_armor_rect(img, pos, armor_yaw, ArmorType::SMALL, q_imu, COLOR_SPIN, thickness);

            // 标注序号
            bool valid = false;
            cv::Point2f pt = tf::world_to_pixel(pos, q_imu, valid);
            if (valid) {
                cv::putText(img, std::to_string(i), pt + cv::Point2f(-5, 5),
                            cv::FONT_HERSHEY_SIMPLEX, 0.4, COLOR_SPIN, 1);
            }
        }

        // 绘制旋转中心
        Eigen::Vector3d center = spin_motion_.predict_center(draw_dt);
        bool valid = false;
        cv::Point2f pt = tf::world_to_pixel(center, q_imu, valid);
        if (valid) {
            // 十字标记
            int s = 15;
            cv::line(img, pt - cv::Point2f(s, 0), pt + cv::Point2f(s, 0), COLOR_CENTER, 2);
            cv::line(img, pt - cv::Point2f(0, s), pt + cv::Point2f(0, s), COLOR_CENTER, 2);
            // 标注角速度和状态
            std::string state_str = spin_active ? "ON" : "OFF";
            cv::putText(img, fmt::format("w={:.1f} {}", spin_motion_.get_omega(), state_str),
                        pt + cv::Point2f(20, 0),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, COLOR_CENTER, 1);
        }
    }

    // 4. 绘制从观测反推的装甲板 (洋红色) - 无滤波滞后
    // 用途: 直观验证几何参数 (r, another_r, dz) 估计是否准确
    if (spin_motion_.valid() && !prev_armors_.empty()) {
        const cv::Scalar COLOR_GEOMETRY(255, 0, 255);  // 洋红色: 几何反推

        // 取最正对的观测装甲板
        const auto& best_obs = prev_armors_[0];  // prev_armors_ 已按 z_to_v 排序
        double obs_armor_yaw = best_obs.z[obs::ARMOR_YAW];

        // 从观测反推所有装甲板
        auto all_armors = spin_motion_.compute_all_armors_from_observation(best_obs.pos, obs_armor_yaw);

        int armor_num = (enemy_type_ == EnemyType::OUTPOST) ? 3 : 4;
        for (int i = 0; i < armor_num && i < static_cast<int>(all_armors.size()); ++i) {
            double armor_yaw = obs_armor_yaw + i * (2.0 * M_PI / armor_num);

            // idx=0 是观测装甲板本身，用虚线; 其他用实线
            int thickness = (i == 0) ? 1 : 2;
            draw_armor_rect(img, all_armors[i], armor_yaw, ArmorType::SMALL, q_imu, COLOR_GEOMETRY, thickness);

            // 标注序号 (带 G 前缀表示 Geometry)
            bool valid = false;
            cv::Point2f pt = tf::world_to_pixel(all_armors[i], q_imu, valid);
            if (valid) {
                cv::putText(img, "G" + std::to_string(i), pt + cv::Point2f(10, -10),
                            cv::FONT_HERSHEY_SIMPLEX, 0.4, COLOR_GEOMETRY, 1);
            }
        }

        // 绘制从观测反推的中心
        double r0 = spin_motion_.get_radius();
        Eigen::Vector3d obs_center(
            best_obs.pos.x() + r0 * std::cos(obs_armor_yaw),
            best_obs.pos.y() + r0 * std::sin(obs_armor_yaw),
            best_obs.pos.z() - spin_motion_.get_dz()
        );
        bool valid = false;
        cv::Point2f pt = tf::world_to_pixel(obs_center, q_imu, valid);
        if (valid) {
            // 菱形标记
            int s = 10;
            std::vector<cv::Point> diamond = {
                cv::Point(pt.x, pt.y - s),
                cv::Point(pt.x + s, pt.y),
                cv::Point(pt.x, pt.y + s),
                cv::Point(pt.x - s, pt.y)
            };
            cv::polylines(img, diamond, true, COLOR_GEOMETRY, 2);

            // 标注几何参数
            cv::putText(img, fmt::format("r={:.2f}/{:.2f}", r0, spin_motion_.get_another_radius()),
                        pt + cv::Point2f(15, -5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, COLOR_GEOMETRY, 1);
            cv::putText(img, fmt::format("dz={:.2f}/{:.2f}", spin_motion_.get_dz(), spin_motion_.get_another_dz()),
                        pt + cv::Point2f(15, 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, COLOR_GEOMETRY, 1);
        }
    }
}

}  // namespace autoaim::predictor
