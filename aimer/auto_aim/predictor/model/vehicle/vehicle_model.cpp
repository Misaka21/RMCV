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

#include "aimer/common/fire_control_types.hpp"
#include "aimer/common/math/math.hpp"
#include "aimer/common/transformer/transformer.hpp"
#include "aimer/auto_aim/predictor/observer/armor_observer.hpp"
#include "plugin/param/runtime_parameter.hpp"
#include "plugin/rerun/rmcv_rerun.hpp"
#include "umt/BasicObjManager.hpp"

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

double get_existing_armor_distance() {
    return get_double_param("AutoAim.Predictor.existing_armor_distance", 0.5);
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
    // 优先使用火控全链路击打时间: prediction_dt + control_to_fire
    // prediction_dt = img_age + send_to_control + fire_to_hit (火控位置预测用)
    // control_to_fire = 控制器→出膛 (ms, 需转换为 s)
    auto fd = umt::BasicObjManager<::fire_control::FireDebugInfo>::find("fire_debug");
    if (fd) {
        const auto& dbg = fd->get();
        if (dbg.fc_heartbeat > 0 && dbg.prediction_dt > 0.001) {
            return dbg.prediction_dt + dbg.latency_control_to_fire / 1000.0;
        }
    }
    // 火控未启动时 fallback
    return get_double_param("AutoAim.Predictor.EKF.draw_predict_dt", 0.020);
}

bool get_use_double_z_fit() {
    auto ptr = runtime_param::find_param("AutoAim.Predictor.use_double_z_fit");
    if (ptr != nullptr) {
        if (auto* val = std::get_if<bool>(&*ptr)) {
            return *val;
        }
    }
    return true;
}

}  // namespace

// ============================================================================
// VehicleModel 实现
// ============================================================================

VehicleModel::VehicleModel(int target_id, EnemyType enemy_type)
    : target_id_(target_id),
      enemy_type_(enemy_type),
      armor_motion_(get_armor_credit_time()),
      motion_(enemy_type == EnemyType::OUTPOST ? 3 : 4) {}

void VehicleModel::update(const std::vector<ArmorObservation>& observations, double timestamp) {
    ++frame_count_;

    // 1. 消抖过滤 (参考 rm.cv.fans screened_armors)
    auto filtered = filter(observations, prev_armors_);

    // 1.5 车辆模型专用双板拟合:
    // observer 不做目标类型约束，这里只对 vehicle 链路启用重投影联合拟合。
    if (get_use_double_z_fit()) {
        ArmorObserver::apply_double_z_fit(filtered);
        std::sort(filtered.begin(), filtered.end(), [](const auto& a, const auto& b) {
            return std::abs(a.z_to_v) < std::abs(b.z_to_v);
        });
    }

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
    std::sort(armors_with_id.begin(), armors_with_id.end(), [](const auto& a, const auto& b) {
        return std::abs(a.z_to_v()) < std::abs(b.z_to_v());
    });
    armor_motion_.update(armors_with_id, timestamp);

    // 4. 整车 EKF 滤波: 使用统一接口
    if (!armors_with_id.empty()) {
        motion_.update(armors_with_id, timestamp);
        last_tracking_id_ = motion_.get_tracked_id();
    }

    if (!initialized_) {
        initialized_ = true;
    }

    // 5. 更新陀螺状态
    spin_.omega = motion_.get_omega();
    // motion 的 theta 是当前追踪装甲板的角度，需要转换为车体角度
    // 车体角度 = 装甲板角度 - tracked_id × 2π/N
    int tracked_id = motion_.get_tracked_id();
    int armor_num = motion_.armor_num();
    spin_.phase = aimer::math::reduced_angle(
        motion_.get_theta() - tracked_id * (2.0 * M_PI / armor_num));
    spin_.radius = motion_.get_radius();
    spin_.radius_2 = motion_.get_another_radius();
    spin_.dz = motion_.get_dz();
    SpinState::SpinThresholds spin_thresholds;
    spin_thresholds.top1_activate = runtime_param::get_param<double>("AutoAim.Predictor.Motion.top1_activate_w");
    spin_thresholds.top1_deactivate = runtime_param::get_param<double>("AutoAim.Predictor.Motion.top1_deactivate_w");
    spin_thresholds.top2_activate = runtime_param::get_param<double>("AutoAim.Predictor.Motion.top2_activate_w");
    spin_thresholds.top2_deactivate = runtime_param::get_param<double>("AutoAim.Predictor.Motion.top2_deactivate_w");
    spin_.update_level(motion_.get_omega(), spin_thresholds);  // 更新陀螺等级 (带迟滞消抖)
    spin_.active = (spin_.level >= SpinLevel::LOW) && motion_.valid();

    // 6. 更新敌方颜色 (用于绘图)
    if (!filtered.empty()) {
        enemy_color_ = filtered[0].color;
    }

    prev_armors_ = filtered;
    prev_timestamp_ = timestamp;

    // ========== 输出到 Rerun ==========
    if (rr::enabled()) {
        std::string prefix = fmt::format("target_{}", target_id_);

        // ========== 1. 观测层 obs ==========
        rr::scalar(prefix + "/obs_num", static_cast<int>(filtered.size()));
        for (size_t i = 0; i < filtered.size(); ++i) {
            const auto& o = filtered[i];
            std::string obs_prefix = fmt::format("{}/obs/armor_{}", prefix, i);
            rr::scalar(obs_prefix + "/x", o.pos.x());
            rr::scalar(obs_prefix + "/y", o.pos.y());
            rr::scalar(obs_prefix + "/z", o.pos.z());
            rr::scalar(obs_prefix + "/yaw", o.z[obs::ARMOR_YAW] * 57.3);
            rr::scalar(obs_prefix + "/z_to_v", o.z_to_v * 57.3);
        }

        // ========== 2. 单装甲板滤波 armor_model ==========
        auto armor_states = armor_motion_.get_armor_states(timestamp);
        rr::scalar(prefix + "/armor_num", static_cast<int>(armor_states.size()));
        for (size_t i = 0; i < armor_states.size(); ++i) {
            const auto& state = armor_states[i];
            std::string armor_prefix = fmt::format("{}/armor_model/armor_{}", prefix, i);
            rr::scalar(armor_prefix + "/x", state.position.x());
            rr::scalar(armor_prefix + "/y", state.position.y());
            rr::scalar(armor_prefix + "/z", state.position.z());
            rr::scalar(armor_prefix + "/vx", state.velocity.x());
            rr::scalar(armor_prefix + "/vy", state.velocity.y());
            rr::scalar(armor_prefix + "/vz", state.velocity.z());
        }

        // ========== 3. 整车滤波 vehicle ==========
        std::string veh_prefix = prefix + "/vehicle";
        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
        double theta = 0, omega = 0, radius = 0, radius_2 = 0, dz = 0;
        bool vehicle_valid = motion_.valid();

        if (vehicle_valid) {
            center = motion_.predict_center(0);
            velocity = motion_.get_velocity();
            theta = motion_.get_theta();
            omega = motion_.get_omega();
            radius = motion_.get_radius();
            radius_2 = motion_.get_another_radius();
            dz = motion_.get_dz();
        }

        rr::scalar(veh_prefix + "/valid", vehicle_valid);
        rr::scalar(veh_prefix + "/x", center.x());
        rr::scalar(veh_prefix + "/y", center.y());
        rr::scalar(veh_prefix + "/z", center.z());
        rr::scalar(veh_prefix + "/vx", velocity.x());
        rr::scalar(veh_prefix + "/vy", velocity.y());
        rr::scalar(veh_prefix + "/vz", velocity.z());
        rr::scalar(veh_prefix + "/theta", theta * 57.3);
        rr::scalar(veh_prefix + "/omega", omega);
        rr::scalar(veh_prefix + "/r", radius);
        rr::scalar(veh_prefix + "/r2", radius_2);
        rr::scalar(veh_prefix + "/dz", dz);
        rr::scalar(veh_prefix + "/spin_level", static_cast<int>(spin_.level));

        // ========== 4. EKF 内部状态 ==========
        if (vehicle_valid) {
            motion_.log_state(veh_prefix + "/ekf");
        }

        // ========== 5. 3D 可视化 ==========
        // 观测装甲板
        std::vector<Eigen::Vector3d> obs_positions;
        for (const auto& o : filtered) {
            obs_positions.push_back(o.pos);
        }
        if (!obs_positions.empty()) {
            rr::points3d(prefix + "/obs_3d", obs_positions, 0, 255, 0, 0.03f);
        }

        // 预测装甲板 + 旋转中心
        if (vehicle_valid) {
            auto predicted_armors = motion_.compute_all_armors(0);
            rr::points3d(prefix + "/predicted_3d", predicted_armors, 0, 128, 255, 0.03f);
            rr::points3d(prefix + "/center", {center}, 255, 0, 0, 0.05f);
            rr::arrows3d(prefix + "/velocity", {center}, {velocity}, 255, 255, 0);
        }
    }
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
    const double existing_dist = get_existing_armor_distance();
    const double jump_limit = get_jump_distance_limit();
    const double new_max_dist = get_new_armor_max_distance();

    // 找最大面积
    double max_area = 0;
    for (const auto& a : raw) {
        if (!a.valid) continue;
        double area = aimer::math::get_area(a.pts);
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

        double area = aimer::math::get_area(a.pts);

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
        bool is_existing = !last.empty() && closest < existing_dist;

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
        return std::abs(a.z_to_v) < std::abs(b.z_to_v);
    });

    // 最多保留 4 块装甲板
    if (result.size() > 4) {
        result.resize(4);
    }

    return result;
}

TargetState VehicleModel::predict(double timestamp) const {
    TargetState vs;
    vs.target_id = target_id_;
    vs.enemy_type = enemy_type_;
    vs.valid = initialized_;
    vs.tracking = initialized_;
    vs.timestamp = timestamp;
    vs.frame_count = frame_count_;
    vs.spin = spin_;
    vs.armor_count = motion_.armor_num();

    if (!initialized_) return vs;

    double dt = timestamp - last_update_time_;

    // 用于置信度计算的变量
    double best_score = 0;

    // 根据陀螺等级选择模型
    bool spin_active = (spin_.level >= SpinLevel::LOW && motion_.valid());

    if (spin_active) {
        // ========== 陀螺模式 ==========
        vs.position = motion_.predict_center(dt);
        vs.velocity = motion_.get_velocity();

        // 预测所有装甲板位置
        int armor_num = motion_.armor_num();
        vs.armor_count = armor_num;

        double local_best_score = -1;
        int best_idx = -1;

        double omega = motion_.get_omega();
        int tracked_id = motion_.get_tracked_id();
        if (armor_num > 0) {
            tracked_id = ((tracked_id % armor_num) + armor_num) % armor_num;
        }
        const double step = armor_num > 0 ? (2.0 * M_PI / armor_num) : 0.0;
        vs.yaw = aimer::math::reduced_angle(motion_.get_theta() + omega * dt - tracked_id * step);
        vs.v_yaw = omega;
        vs.radius_1 = motion_.get_radius();
        vs.radius_2 = motion_.get_another_radius();
        vs.dz = motion_.get_dz();
        vs.spin.phase = vs.yaw;
        vs.spin.omega = omega;
        vs.spin.radius = vs.radius_1;
        vs.spin.radius_2 = vs.radius_2;
        vs.spin.dz = vs.dz;

        const bool fresh_visible = dt <= get_armor_credit_time();

        for (int i = 0; i < armor_num; ++i) {
            int abs_id = (tracked_id + i) % armor_num;
            const bool use_l_h = (armor_num == 4) && (abs_id % 2 == 1);

            Eigen::Vector3d armor_pos = motion_.predict_armor_pos(i, dt);
            Eigen::Vector3d offset = armor_pos - vs.position;
            Eigen::Vector3d tangent_vel(
                -omega * offset.y(),
                +omega * offset.x(),
                0
            );
            Eigen::Vector3d armor_vel = vs.velocity + tangent_vel;

            vs.armor_ids[abs_id] = abs_id;
            vs.armor_types[abs_id] = correct_armor_type(ArmorType::SMALL, enemy_type_);
            vs.armor_radii[abs_id] = use_l_h ? vs.radius_2 : vs.radius_1;
            vs.armor_z_offsets[abs_id] = use_l_h ? vs.dz : 0.0;
            vs.armor_position_offsets[abs_id] = offset;
            vs.armor_velocity_offsets[abs_id] = armor_vel - vs.velocity;
            vs.armor_last_seen[abs_id] = last_update_time_;

            // 评分: 越正对越好 (用 cos(装甲板朝向 - 视线方向))
            double armor_yaw = vs.armor_yaw(abs_id, 0);
            double view_yaw = std::atan2(armor_pos.y(), armor_pos.x());
            // 保留符号: z_to_v 的方向信息会被 INDIRECT 选板逻辑使用
            double angle_diff = aimer::math::reduced_angle(armor_yaw - view_yaw);
            double score = std::cos(std::abs(angle_diff));
            vs.armor_z_to_v[abs_id] = angle_diff;
            vs.armor_scores[abs_id] = score;

            // 可见性: 必须在“新鲜观测窗口”内，防止纯预测板长期被当作可见
            bool visible = fresh_visible &&
                           ((i == 0) ||
                            (prev_armors_.size() >= 2 && score > 0.5));
            vs.set_armor_visible(abs_id, visible);
            vs.set_armor_detected(abs_id, visible);

            if (score > local_best_score) {
                local_best_score = score;
                best_idx = abs_id;
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
            center_sum += armor_states[i].position;
            vel_sum += armor_states[i].velocity;
            ++valid_count;

            if (armor_states[i].score > local_best_score) {
                local_best_score = armor_states[i].score;
                best_idx = i;
            }
        }

        if (valid_count > 0) {
            vs.position = center_sum / valid_count;
            vs.velocity = vel_sum / valid_count;
        }

        for (int i = 0; i < vs.armor_count; ++i) {
            const auto& armor = armor_states[i];
            vs.armor_ids[i] = armor.id;
            vs.armor_types[i] = correct_armor_type(armor.type, enemy_type_);
            vs.armor_position_offsets[i] = armor.position - vs.position;
            vs.armor_velocity_offsets[i] = armor.velocity - vs.velocity;
            vs.armor_z_to_v[i] = armor.z_to_v;
            vs.armor_last_seen[i] = armor.last_seen;
            vs.armor_scores[i] = armor.score;
            vs.set_armor_visible(i, armor.visible);
            vs.set_armor_detected(i, armor.visible);
        }

        if (best_idx >= 0 && best_idx < vs.armor_count) {
            // ArmorMotion 的单板 EKF 只滤位置，yaw 仍是面向相机的旧启发式。
            // 在 VehicleModel 输出边界转换成 TargetState 统一的 INWARD yaw。
            const double best_armor_yaw_inward =
                aimer::math::reduced_angle(armor_states[best_idx].yaw + M_PI);
            vs.yaw = aimer::math::reduced_angle(
                best_armor_yaw_inward - best_idx * vs.armor_step());
        }
        vs.v_yaw = 0.0;

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
    motion_.reset();
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
        pts[i] = aimer::tf::world_to_pixel(corners[i], q_imu, valid);
        if (!valid) all_valid = false;
    }

    if (!all_valid) return;

    // 画矩形 (口字形)
    for (int i = 0; i < 4; ++i) {
        cv::line(img, pts[i], pts[(i + 1) % 4], color, thickness);
    }
}

// 根据 z_to_v (三分法优化后的朝向角) 绘制装甲板框
// z_to_v: 相对于相机视线的角度 (0=正对, 正值=顺时针偏转)
void draw_armor_by_z_to_v(cv::Mat& img, const Eigen::Vector3d& pos_world,
                          double z_to_v, ArmorType type,
                          const Eigen::Quaterniond& q_imu,
                          const cv::Scalar& color, int thickness = 2) {
    double w = (type == ArmorType::LARGE) ? 0.225 : 0.133;
    double h = 0.055;
    constexpr double pitch = -15.0 * M_PI / 180.0;

    // 相机 Z 轴在世界 XY 平面的投影 (归一化)
    Eigen::Vector3d camera_z_world = aimer::tf::vector<aimer::tf::Frame::Camera, aimer::tf::Frame::World>(
        Eigen::Vector3d(0, 0, 1), q_imu);
    Eigen::Vector2d camera_z_2d(camera_z_world.x(), camera_z_world.y());
    double norm = camera_z_2d.norm();
    if (norm > 1e-6) camera_z_2d /= norm;
    else camera_z_2d = Eigen::Vector2d(1.0, 0.0);

    // 装甲板法向量 = 相机前向旋转 z_to_v
    Eigen::Vector2d normal_2d = aimer::math::rotate(camera_z_2d, z_to_v);

    // 装甲板 X 轴 (水平，垂直于法向量)
    Eigen::Vector2d x_2d = aimer::math::rotate(normal_2d, M_PI / 2);
    Eigen::Vector3d x_axis(x_2d.x(), x_2d.y(), 0.0);

    // 装甲板 Y 轴 (竖直，考虑俯仰角)
    Eigen::Vector3d y_axis(
        -normal_2d.x() * std::sin(pitch),
        -normal_2d.y() * std::sin(pitch),
        std::cos(pitch));

    // 四角点
    std::array<Eigen::Vector3d, 4> corners = {
        pos_world + x_axis * (w / 2) + y_axis * (h / 2),
        pos_world + x_axis * (w / 2) - y_axis * (h / 2),
        pos_world - x_axis * (w / 2) - y_axis * (h / 2),
        pos_world - x_axis * (w / 2) + y_axis * (h / 2)
    };

    // 投影并绘制
    std::array<cv::Point2f, 4> pts;
    bool all_valid = true;
    for (int i = 0; i < 4; ++i) {
        bool valid = false;
        pts[i] = aimer::tf::world_to_pixel(corners[i], q_imu, valid);
        if (!valid) all_valid = false;
    }
    if (!all_valid) return;

    for (int i = 0; i < 4; ++i) {
        cv::line(img, pts[i], pts[(i + 1) % 4], color, thickness);
    }
}

// 颜色转字符串
const char* enemy_color_str(EnemyColor color) {
    switch (color) {
        case EnemyColor::RED: return "R";
        case EnemyColor::BLUE: return "B";
        default: return "G";
    }
}

}  // namespace

void VehicleModel::draw(cv::Mat& img, const Eigen::Quaterniond& q_imu, double timestamp) const {
    if (!initialized_) return;

    // 计算距离上次更新的时间，用于判断数据是否过时
    double time_since_update = timestamp - last_update_time_;
    bool data_fresh = time_since_update < 0.1;  // 100ms 内认为数据新鲜

    // 预测时间 (用于绘图外推)
    double draw_dt = get_draw_predict_dt();

    // 颜色定义
    const cv::Scalar COLOR_DETECTED(0, 255, 0);    // 绿色: 检测到的
    const cv::Scalar COLOR_FILTERED(255, 200, 0);  // 蓝色: 滤波后的
    const cv::Scalar COLOR_CENTER(255, 0, 255);    // 紫色: 旋转中心

    // 整车模型预测颜色: 未使用灰色，使用时用我方颜色
    cv::Scalar COLOR_SPIN;
    bool spin_valid = motion_.valid();
    SpinLevel spin_level = spin_.level;
    bool spin_active = spin_level >= SpinLevel::LOW && spin_valid;
    if (!spin_active) {
        COLOR_SPIN = cv::Scalar(128, 128, 128);  // 灰色: 未使用
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

    // 1. 绘制检测到的装甲板 (X形状 + z_to_v重投影框)
    // 文字在最后绘制，避免被其他线覆盖
    const cv::Scalar COLOR_Z_TO_V(0, 255, 255);  // 黄色: z_to_v 重投影
    if (data_fresh) {
        auto active_armors = identifier_.get_active_armors(frame_count_);

        for (size_t idx = 0; idx < active_armors.size(); ++idx) {
            const auto& armor = active_armors[idx];
            const auto& obs = armor.observation;

            if (obs.pts.size() >= 4) {
                // X 形状: 连接对角线 (原始检测)
                cv::line(img, obs.pts[0], obs.pts[2], COLOR_DETECTED, 2);
                cv::line(img, obs.pts[1], obs.pts[3], COLOR_DETECTED, 2);

                // 三分法优化后的 z_to_v 重投影装甲板框 (黄色)
                draw_armor_by_z_to_v(img, obs.pos, obs.z_to_v, obs.type, q_imu, COLOR_Z_TO_V, 2);
            }
        }

        // 如果是双装甲板，显示两者夹角差 (应该接近 90°)
        if (active_armors.size() == 2) {
            double z0 = active_armors[0].observation.z_to_v;
            double z1 = active_armors[1].observation.z_to_v;
            double angle_diff = std::abs(aimer::math::angle_diff(z0, z1)) * 180.0 / M_PI;

            // 在两块装甲板中间显示夹角差
            cv::Point2f mid = (active_armors[0].observation.center_2d +
                               active_armors[1].observation.center_2d) * 0.5f;
            cv::Scalar color = (std::abs(angle_diff - 90.0) < 10.0)
                ? cv::Scalar(0, 255, 0)    // 绿色: 接近 90°
                : cv::Scalar(0, 165, 255); // 橙色: 偏离 90°
            cv::putText(img, fmt::format("D:{:.0f}", angle_diff),
                        mid + cv::Point2f(0, -20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
        }
    }

    // 2. 绘制 ArmorMotion 滤波后的位置 (空心圈，近大远小)
    // ArmorMotion 没有 yaw 信息，侧向时画口字形不准，改用空心圈
    // 只有数据新鲜时才绘制
    if (data_fresh) {
        auto armor_states = armor_motion_.get_armor_states(timestamp);  // 用原始时间获取
        for (auto& as : armor_states) {
            // 手动外推位置
            Eigen::Vector3d predicted_pos = as.position + as.velocity * draw_dt;
            bool valid = false;
            cv::Point2f pt = aimer::tf::world_to_pixel(predicted_pos, q_imu, valid);
            if (valid) {
                // 近大远小: 半径 = base_size / distance
                double distance = predicted_pos.norm();
                int radius = static_cast<int>(50.0 / std::max(distance, 0.5));  // 1m处50px，2m处25px
                radius = std::clamp(radius, 5, 100);  // 限制范围
                cv::circle(img, pt, radius, COLOR_FILTERED, 2);
                // 标注 armor_id
                cv::putText(img, "A" + std::to_string(as.id),
                            pt + cv::Point2f(radius + 5, 5),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, COLOR_FILTERED, 1);
            }
        }
    }

    // 3. 绘制整车模型预测 (口字形) - EKF 滤波后的
    // 即使未激活也绘制，方便调试
    // EKF 预测可以继续绘制，因为它有自己的时间预测能力
    if (spin_valid) {
        int armor_num = motion_.armor_num();
        double omega = motion_.get_omega();
        double theta = motion_.get_theta();
        // 注意: predict_armor_pos 内部已经对 theta 做了 draw_dt 外推
        // 这里也要做同样的外推，保持一致
        double theta_predicted = theta + omega * draw_dt;

        // 绘制所有装甲板预测位置
        for (int i = 0; i < armor_num; ++i) {
            Eigen::Vector3d pos = motion_.predict_armor_pos(i, draw_dt);

            // draw_armor_rect 需要装甲板朝向 (INWARD)
            double armor_yaw = theta_predicted + i * (2.0 * M_PI / armor_num);

            // 当前追踪的装甲板用粗线
            int thickness = (i == 0) ? 3 : 1;
            draw_armor_rect(img, pos, armor_yaw, ArmorType::SMALL, q_imu, COLOR_SPIN, thickness);

            // 标注序号
            bool valid = false;
            cv::Point2f pt = aimer::tf::world_to_pixel(pos, q_imu, valid);
            if (valid) {
                cv::putText(img, std::to_string(i), pt + cv::Point2f(-5, 5),
                            cv::FONT_HERSHEY_SIMPLEX, 0.4, COLOR_SPIN, 1);
            }
        }

        // 绘制旋转中心 (紫色十字 + 菱形)
        Eigen::Vector3d center = motion_.predict_center(draw_dt);
        Eigen::Vector3d velocity = motion_.get_velocity();
        double r0 = motion_.get_radius();
        double r2 = motion_.get_another_radius();
        double dz = motion_.get_dz();
        double theta_deg = motion_.get_theta() * 180.0 / M_PI;
        double omega_deg = omega * 180.0 / M_PI;
        int tracked_id = motion_.get_tracked_id();

        bool valid = false;
        cv::Point2f pt = aimer::tf::world_to_pixel(center, q_imu, valid);
        if (valid) {
            // 十字 + 菱形组合标记
            int s = 12;
            // 十字
            cv::line(img, pt - cv::Point2f(s, 0), pt + cv::Point2f(s, 0), COLOR_CENTER, 2);
            cv::line(img, pt - cv::Point2f(0, s), pt + cv::Point2f(0, s), COLOR_CENTER, 2);
            // 菱形
            std::vector<cv::Point> diamond = {
                cv::Point(pt.x, pt.y - s - 3),
                cv::Point(pt.x + s + 3, pt.y),
                cv::Point(pt.x, pt.y + s + 3),
                cv::Point(pt.x - s - 3, pt.y)
            };
            cv::polylines(img, diamond, true, COLOR_CENTER, 2);

            // ========== 详细状态信息 ==========
            // 陀螺等级字符串
            const char* level_str = "NONE";
            if (spin_level == SpinLevel::LOW) level_str = "LOW";
            else if (spin_level == SpinLevel::HIGH) level_str = "HIGH";

            // 敌方类型字符串
            const char* type_str = "UNK";
            switch (enemy_type_) {
                case EnemyType::HERO: type_str = "HERO"; break;
                case EnemyType::ENGINEER: type_str = "ENG"; break;
                case EnemyType::INFANTRY_3:
                case EnemyType::INFANTRY_4:
                case EnemyType::INFANTRY_5:
                    type_str = "INF"; break;
                case EnemyType::SENTRY: type_str = "SENT"; break;
                case EnemyType::OUTPOST: type_str = "OUTP"; break;
                case EnemyType::BASE: type_str = "BASE"; break;
                default: break;
            }

            // 多行状态信息
            std::string model_str = motion_.name();
            std::array<std::string, 6> lines = {
                fmt::format("[T{}] {} | {} {}", target_id_, type_str, model_str, spin_active ? "ON" : "OFF"),
                fmt::format("spin: {} | track_id: {}", level_str, tracked_id),
                fmt::format("w: {:.0f} d/s | th: {:.0f}", omega_deg, theta_deg),
                fmt::format("r: {:.2f}/{:.2f} | dz: {:.2f}", r0, r2, dz),
                fmt::format("pos: ({:.2f}, {:.2f}, {:.2f})", center.x(), center.y(), center.z()),
                fmt::format("vel: ({:.2f}, {:.2f}, {:.2f})", velocity.x(), velocity.y(), velocity.z())
            };

            // 绘制在中心上方
            cv::Point2f text_base = pt + cv::Point2f(-100, -120);
            for (size_t i = 0; i < lines.size(); ++i) {
                cv::putText(img, lines[i], text_base + cv::Point2f(0, i * 16),
                            cv::FONT_HERSHEY_SIMPLEX, 0.45, COLOR_CENTER, 1);
            }
        }
    }

    // 4. 绘制从 EKF 状态生成的装甲板 (洋红色)
    // rm.cv.fans 原版设计: 直接从 EKF 状态生成，不需要从观测反推
    auto active_armors = identifier_.get_active_armors(frame_count_);
    if (spin_valid && data_fresh && !active_armors.empty()) {
        const cv::Scalar COLOR_GEOMETRY(255, 0, 255);  // 洋红色

        // 直接从 EKF 状态生成所有装甲板
        std::vector<Eigen::Vector3d> all_armors = motion_.compute_all_armors(0);

        int armor_num = motion_.armor_num();

        // 用 EKF 的 theta 来画装甲板朝向
        double state_theta_inward = motion_.get_theta();

        for (int i = 0; i < armor_num && i < static_cast<int>(all_armors.size()); ++i) {
            // draw_armor_rect 需要装甲板朝向 (INWARD)
            double armor_yaw = state_theta_inward + i * (2.0 * M_PI / armor_num);

            // idx=0 是当前追踪的装甲板
            int thickness = (i == 0) ? 1 : 2;
            draw_armor_rect(img, all_armors[i], armor_yaw, ArmorType::SMALL, q_imu, COLOR_GEOMETRY, thickness);

            // 标注序号
            bool valid = false;
            cv::Point2f pt = aimer::tf::world_to_pixel(all_armors[i], q_imu, valid);
            if (valid) {
                cv::putText(img, "G" + std::to_string(i), pt + cv::Point2f(10, -10),
                            cv::FONT_HERSHEY_SIMPLEX, 0.4, COLOR_GEOMETRY, 1);
            }
        }
    }

    // 5. 最后绘制装甲板详细信息文字 (在所有线之上)
    if (data_fresh) {
        auto active_armors = identifier_.get_active_armors(frame_count_);

        // 计算所有装甲板的平均 x 坐标 (用于左右分布)
        float avg_x = 0;
        for (const auto& a : active_armors) {
            avg_x += a.observation.center_2d.x;
        }
        if (!active_armors.empty()) avg_x /= active_armors.size();

        for (size_t idx = 0; idx < active_armors.size(); ++idx) {
            const auto& armor = active_armors[idx];
            const auto& obs = armor.observation;

            if (obs.pts.size() >= 4) {
                // 计算面积 (以 k 为单位)
                double area = aimer::math::get_area(obs.pts) / 1000.0;

                // 模式标注
                std::string mode_str = "NORMAL";
                if (spin_active) {
                    mode_str = motion_.name();
                }

                // 5行文字
                std::array<std::string, 5> lines = {
                    fmt::format("number:{} | id:{} | {}", idx, armor.id, mode_str),
                    fmt::format("color:{} | z_to_v:{:.1f}", enemy_color_str(obs.color), obs.z_to_v * 57.3),
                    fmt::format("area:{:.1f}k", area),
                    fmt::format("x:{:.2f} | y:{:.2f} | z:{:.2f}", obs.pos.x(), obs.pos.y(), obs.pos.z()),
                    fmt::format("yaw:{:.1f} | pitch:{:.1f} | dist:{:.2f}",
                        obs.z[obs::YAW] * 57.3, obs.z[obs::PITCH] * 57.3, obs.z[obs::DIST])
                };

                // 根据位置分左右，避免重叠
                bool on_left = (obs.center_2d.x < avg_x);
                cv::Point2f base;
                if (active_armors.size() == 1) {
                    // 单装甲板：放正下方
                    base = obs.center_2d + cv::Point2f(-100, 70);
                } else if (on_left) {
                    // 左边装甲板：文字偏左下
                    base = obs.center_2d + cv::Point2f(-160, 70);
                } else {
                    // 右边装甲板：文字偏右下
                    base = obs.center_2d + cv::Point2f(20, 70);
                }

                for (size_t i = 0; i < lines.size(); ++i) {
                    cv::putText(img, lines[i], base + cv::Point2f(0, i * 18),
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, COLOR_DETECTED, 1);
                }
            }
        }
    }
}

}  // namespace autoaim::predictor
