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
#include "plugin/param/runtime_parameter.hpp"
#include "plugin/plotter/plotter.hpp"
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
    // 优先使用火控实际的 prediction_dt (img→hit 击打时间)
    auto fd = umt::BasicObjManager<::fire_control::FireDebugInfo>::find("fire_debug");
    if (fd) {
        const auto& dbg = fd->get();
        if (dbg.fc_heartbeat > 0 && dbg.prediction_dt > 0.001) {
            return dbg.prediction_dt;
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

// 车辆模型专用双板朝向约束:
// 仅在两块装甲板同时可见时，将 z_to_v 约束到相差 90°，再同步更新 armor_yaw。
void apply_vehicle_double_fit(std::vector<ArmorObservation>& observations) {
    if (observations.size() != 2) return;

    int left_idx = observations[0].z_to_v <= observations[1].z_to_v ? 0 : 1;
    int right_idx = 1 - left_idx;

    auto& left = observations[left_idx];
    auto& right = observations[right_idx];

    double z_left = left.z_to_v;
    double z_right = right.z_to_v;
    double diff = std::abs(aimer::math::angle_diff(z_left, z_right));

    // 先做几何一致性检查，避免错误配对强行拟合
    constexpr double MIN_PAIR_ANGLE = 60.0 * M_PI / 180.0;
    constexpr double MAX_PAIR_ANGLE = 120.0 * M_PI / 180.0;
    if (diff < MIN_PAIR_ANGLE || diff > MAX_PAIR_ANGLE) {
        return;
    }

    // 最小化 (zL'-zL)^2 + (zR'-(zL'+90°))^2 的闭式解
    double z_right_near = aimer::math::get_closest_angle(z_right, z_left + M_PI / 2.0);
    double z_left_fit = 0.5 * (z_left + z_right_near - M_PI / 2.0);
    double z_right_fit = z_left_fit + M_PI / 2.0;

    // 限制单次修正量，防止离群观测引发大跳变
    constexpr double MAX_CORRECTION = 35.0 * M_PI / 180.0;
    if (std::abs(aimer::math::angle_diff(z_left, z_left_fit)) > MAX_CORRECTION
        || std::abs(aimer::math::angle_diff(z_right, z_right_fit)) > MAX_CORRECTION)
    {
        return;
    }

    left.z_to_v = aimer::math::reduced_angle(z_left_fit);
    right.z_to_v = aimer::math::reduced_angle(z_right_fit);

    // camera_yaw = armor_yaw - z_to_v，左右各自估计后取平均再回写
    double cam_yaw_l = left.z[obs::ARMOR_YAW] - z_left;
    double cam_yaw_r = right.z[obs::ARMOR_YAW] - z_right;
    cam_yaw_r = aimer::math::get_closest_angle(cam_yaw_r, cam_yaw_l);
    double cam_yaw = 0.5 * (cam_yaw_l + cam_yaw_r);

    left.z[obs::ARMOR_YAW] = aimer::math::get_closest_angle(
        cam_yaw + left.z_to_v, left.z[obs::ARMOR_YAW]);
    right.z[obs::ARMOR_YAW] = aimer::math::get_closest_angle(
        cam_yaw + right.z_to_v, right.z[obs::ARMOR_YAW]);
}

}  // namespace

// ============================================================================
// VehicleModel 实现
// ============================================================================

VehicleModel::VehicleModel(int target_id, EnemyType enemy_type)
    : target_id_(target_id),
      enemy_type_(enemy_type),
      armor_motion_(get_armor_credit_time()) {}

void VehicleModel::ensure_motion_model() {
    const std::string model = runtime_param::get_param<std::string>("AutoAim.Predictor.motion_model");

    // 模型类型未变化，无需操作
    if (motion_ && model == current_motion_model_) {
        return;
    }

    // 创建或切换模型
    int armor_num = (enemy_type_ == EnemyType::OUTPOST) ? 3 : 4;
    motion_ = create_motion(model, armor_num);
    current_motion_model_ = model;
}

void VehicleModel::update(const std::vector<ArmorObservation>& observations, double timestamp) {
    ++frame_count_;

    // 确保运动模型存在 (支持热切换)
    ensure_motion_model();

    // 1. 消抖过滤 (参考 rm.cv.fans screened_armors)
    auto filtered = filter(observations, prev_armors_);

    // 1.5 车辆模型专用双板拟合:
    // observer 不再做联合拟合，这里只对 vehicle 链路启用 90° 约束。
    if (get_use_double_z_fit()) {
        apply_vehicle_double_fit(filtered);
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
        motion_->update(armors_with_id, timestamp);
        last_tracking_id_ = motion_->get_tracked_id();
    }

    if (!initialized_) {
        initialized_ = true;
    }

    // 5. 更新陀螺状态
    spin_.omega = motion_->get_omega();
    // motion 的 theta 是当前追踪装甲板的角度，需要转换为车体角度
    // 车体角度 = 装甲板角度 - tracked_id × 2π/N
    int tracked_id = motion_->get_tracked_id();
    int armor_num = motion_->armor_num();
    spin_.phase = motion_->get_theta() - tracked_id * (2.0 * M_PI / armor_num);
    spin_.radius = motion_->get_radius();
    spin_.radius_2 = motion_->get_another_radius();
    spin_.dz = motion_->get_dz();
    spin_.update_level(motion_->get_omega());  // 更新陀螺等级 (带迟滞消抖)
    spin_.active = (spin_.level >= SpinLevel::LOW) && motion_->valid();

    // 6. 更新敌方颜色 (用于绘图)
    if (!filtered.empty()) {
        enemy_color_ = filtered[0].color;
    }

    prev_armors_ = filtered;
    prev_timestamp_ = timestamp;

    // ========== 输出到 PlotJuggler ==========
    if (plotter::enabled()) {
        std::string prefix = fmt::format("/target_{}", target_id_);

        plotter::begin();

        // ========== 1. 观测层 obs ==========
        plotter::add(prefix + "/obs_num", static_cast<int>(filtered.size()));
        for (size_t i = 0; i < filtered.size(); ++i) {
            const auto& o = filtered[i];
            std::string obs_prefix = fmt::format("{}/obs/armor_{}", prefix, i);
            plotter::add(obs_prefix + "/x", o.pos.x());
            plotter::add(obs_prefix + "/y", o.pos.y());
            plotter::add(obs_prefix + "/z", o.pos.z());
            plotter::add(obs_prefix + "/yaw", o.z[obs::ARMOR_YAW] * 57.3);
            plotter::add(obs_prefix + "/z_to_v", o.z_to_v * 57.3);
        }

        // ========== 2. 单装甲板滤波 armor_model ==========
        auto armor_states = armor_motion_.get_armor_states(timestamp);
        plotter::add(prefix + "/armor_num", static_cast<int>(armor_states.size()));
        for (size_t i = 0; i < armor_states.size(); ++i) {
            const auto& state = armor_states[i];
            std::string armor_prefix = fmt::format("{}/armor_model/armor_{}", prefix, i);
            plotter::add(armor_prefix + "/x", state.position.x());
            plotter::add(armor_prefix + "/y", state.position.y());
            plotter::add(armor_prefix + "/z", state.position.z());
            plotter::add(armor_prefix + "/vx", state.velocity.x());
            plotter::add(armor_prefix + "/vy", state.velocity.y());
            plotter::add(armor_prefix + "/vz", state.velocity.z());
        }

        // ========== 3. 整车滤波 vehicle ==========
        std::string veh_prefix = prefix + "/vehicle";
        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
        double theta = 0, omega = 0, radius = 0, radius_2 = 0, dz = 0;
        bool vehicle_valid = motion_->valid();

        if (vehicle_valid) {
            center = motion_->predict_center(0);
            velocity = motion_->get_velocity();
            theta = motion_->get_theta();
            omega = motion_->get_omega();
            radius = motion_->get_radius();
            radius_2 = motion_->get_another_radius();
            dz = motion_->get_dz();
        }

        plotter::add(veh_prefix + "/valid", vehicle_valid ? 1 : 0);
        plotter::add(veh_prefix + "/x", center.x());
        plotter::add(veh_prefix + "/y", center.y());
        plotter::add(veh_prefix + "/z", center.z());
        plotter::add(veh_prefix + "/vx", velocity.x());
        plotter::add(veh_prefix + "/vy", velocity.y());
        plotter::add(veh_prefix + "/vz", velocity.z());
        plotter::add(veh_prefix + "/theta", theta * 57.3);
        plotter::add(veh_prefix + "/omega", omega);
        plotter::add(veh_prefix + "/r", radius);
        plotter::add(veh_prefix + "/r2", radius_2);
        plotter::add(veh_prefix + "/dz", dz);
        plotter::add(veh_prefix + "/spin_level", static_cast<int>(spin_.level));

        plotter::end();
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

VehicleState VehicleModel::predict(double timestamp) const {
    VehicleState vs;
    vs.target_id = target_id_;
    vs.enemy_type = enemy_type_;
    vs.valid = initialized_;
    vs.timestamp = timestamp;
    vs.frame_count = frame_count_;
    vs.spin = spin_;

    if (!initialized_ || !motion_) return vs;

    double dt = timestamp - last_update_time_;

    // 用于置信度计算的变量
    double best_score = 0;

    // 根据陀螺等级选择模型
    bool spin_active = (spin_.level >= SpinLevel::LOW && motion_->valid());

    if (spin_active) {
        // ========== 陀螺模式 ==========
        vs.center = motion_->predict_center(dt);
        vs.velocity = motion_->get_velocity();

        // 预测所有装甲板位置
        int armor_num = motion_->armor_num();
        vs.armor_count = armor_num;

        double local_best_score = -1;
        int best_idx = -1;

        double theta = motion_->get_theta();
        double omega = spin_.omega;
        int tracked_id = motion_->get_tracked_id();
        const bool fresh_visible = dt <= get_armor_credit_time();

        for (int i = 0; i < armor_num; ++i) {
            auto& as = vs.armors[i];
            // 统一语义: ArmorState.id 表示当前数组中的装甲板索引
            as.id = i;

            as.position = motion_->predict_armor_pos(i, dt);
            Eigen::Vector3d offset = as.position - vs.center;
            Eigen::Vector3d tangent_vel(
                -omega * offset.y(),
                +omega * offset.x(),
                0
            );
            as.velocity = vs.velocity + tangent_vel;
            as.yaw = theta + i * (2.0 * M_PI / armor_num);
            as.last_seen = last_update_time_;

            // 评分: 越正对越好 (用 cos(装甲板朝向 - 视线方向))
            double armor_yaw = as.yaw;
            double view_yaw = std::atan2(as.position.y(), as.position.x());
            // 保留符号: z_to_v 的方向信息会被 INDIRECT 选板逻辑使用
            double angle_diff = aimer::math::reduced_angle(armor_yaw - view_yaw - M_PI);
            as.score = std::cos(std::abs(angle_diff));

            // 装甲板类型和朝向角
            as.type = correct_armor_type(ArmorType::SMALL, enemy_type_);
            as.z_to_v = angle_diff;

            // 可见性: 必须在“新鲜观测窗口”内，防止纯预测板长期被当作可见
            as.visible = fresh_visible &&
                         ((i == tracked_id) ||
                          (prev_armors_.size() >= 2 && as.score > 0.5));

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
    if (motion_) {
        motion_->reset();
    }
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
    if (!initialized_ || !motion_) return;

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
    bool spin_valid = motion_->valid();
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
        int armor_num = motion_->armor_num();
        double omega = motion_->get_omega();
        double theta = motion_->get_theta();
        // 注意: predict_armor_pos 内部已经对 theta 做了 draw_dt 外推
        // 这里也要做同样的外推，保持一致
        double theta_predicted = theta + omega * draw_dt;

        // 绘制所有装甲板预测位置
        for (int i = 0; i < armor_num; ++i) {
            Eigen::Vector3d pos = motion_->predict_armor_pos(i, draw_dt);

            // draw_armor_rect 需要装甲板朝向 (面朝方向, INWARD)
            // theta_predicted 是 OUTWARD (从中心指向装甲板), 装甲板朝向 = theta + π
            double armor_yaw = theta_predicted + i * (2.0 * M_PI / armor_num) + M_PI;

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
        Eigen::Vector3d center = motion_->predict_center(draw_dt);
        Eigen::Vector3d velocity = motion_->get_velocity();
        double r0 = motion_->get_radius();
        double r2 = motion_->get_another_radius();
        double dz = motion_->get_dz();
        double theta_deg = motion_->get_theta() * 180.0 / M_PI;
        double omega_deg = omega * 180.0 / M_PI;
        int tracked_id = motion_->get_tracked_id();

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
            std::string model_str = motion_->name();
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
        std::vector<Eigen::Vector3d> all_armors = motion_->compute_all_armors(0);

        int armor_num = motion_->armor_num();

        // 用 EKF 的 theta 来画装甲板朝向
        double state_theta_outward = motion_->get_theta();

        for (int i = 0; i < armor_num && i < static_cast<int>(all_armors.size()); ++i) {
            // draw_armor_rect 需要装甲板朝向 (INWARD)
            // OUTWARD + M_PI = INWARD
            double armor_yaw = state_theta_outward + i * (2.0 * M_PI / armor_num) + M_PI;

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
                    mode_str = motion_->name();
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
