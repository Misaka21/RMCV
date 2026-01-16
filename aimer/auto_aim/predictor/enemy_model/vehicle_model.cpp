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
#include "plugin/plotter/plotter.hpp"

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
      spin_motion_(enemy_type == EnemyType::OUTPOST ? 3 : 4),
      lmtd_motion_(enemy_type == EnemyType::OUTPOST ? 3 : 4),
      sp_motion_(enemy_type == EnemyType::OUTPOST ? 3 : 4) {}

void VehicleModel::update(const std::vector<ArmorObservation>& observations, double timestamp) {
    ++frame_count_;
    // 模型选择: spin (旧), lmtd, sp (新)
    const std::string motion_model = runtime_param::get_param<std::string>("AutoAim.Predictor.motion_model");
    const bool use_lmtd = (motion_model == "lmtd");
    const bool use_sp = (motion_model == "sp");

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

    // 4. 整车 EKF 滤波: SpinMotion, LmtdMotion 或 SpMotion
    if (!armors_with_id.empty()) {
        if (use_sp) {
            // ========== SP 模式: 11维状态，无状态交换 ==========
            sp_motion_.update(armors_with_id, timestamp);
            last_tracking_id_ = sp_motion_.get_tracked_id();
        } else if (use_lmtd) {
            // ========== LMTD 模式: 内部处理跳变 ==========
            lmtd_motion_.update(armors_with_id, timestamp);

            // 更新 last_tracking_id_ (用于上层逻辑)
            last_tracking_id_ = lmtd_motion_.get_tracked_id();
        } else {
            // ========== SpinMotion 模式: 内部处理跳变 ==========
            // SpinMotion::update() 内部调用 detect_and_handle_jump()
            // 顺序: predict → 跳变检测 → 观测更新
            spin_motion_.update(armors_with_id, timestamp);

            // 更新追踪 ID (用于上层逻辑)
            last_tracking_id_ = armors_with_id[0].id;
        }
    }

    // DEBUG: 输出装甲板数量和 ID
    //if (armors_with_id.size() > 1 || armor_motion_.size() > 1) {
    //    fmt::print(fmt::fg(fmt::color::orange),
    //        "[T{}] obs:{} filtered:{} active:{} filters:{}\n",
    //        target_id_, observations.size(), filtered.size(),
    //        armors_with_id.size(), armor_motion_.size());
    //    for (const auto& a : armors_with_id) {
    //        fmt::print("  armor id={} pos=({:.2f},{:.2f},{:.2f})\n",
    //            a.id, a.pos().x(), a.pos().y(), a.pos().z());
    //    }
    //}

    if (!initialized_) {
        initialized_ = true;
    }

    // 5. 更新陀螺状态
    if (use_sp) {
        spin_.omega = sp_motion_.get_omega();
        spin_.phase = sp_motion_.get_theta();
        spin_.radius = sp_motion_.get_radius();
        spin_.radius_2 = sp_motion_.get_another_radius();
        spin_.level = sp_motion_.get_spin_level();
        spin_.active = (spin_.level >= SpinLevel::LOW) && sp_motion_.valid();
    } else if (use_lmtd) {
        spin_.omega = lmtd_motion_.get_omega();
        // LmtdMotion 的 theta 是当前追踪装甲板的角度，需要转换为车体角度
        // 车体角度 = 装甲板角度 - tracked_id × 2π/N
        int tracked_id = lmtd_motion_.get_tracked_id();
        constexpr int armor_num = 4;
        spin_.phase = lmtd_motion_.get_theta() - tracked_id * (2.0 * M_PI / armor_num);
        spin_.radius = lmtd_motion_.get_radius();
        spin_.radius_2 = lmtd_motion_.get_another_radius();
        spin_.level = lmtd_motion_.get_spin_level();
        spin_.active = (spin_.level >= SpinLevel::LOW) && lmtd_motion_.valid();
    } else {
        spin_.omega = spin_motion_.get_omega();
        // SpinMotion 的 theta 也是当前追踪装甲板的角度
        // 但 SpinMotion 没有 get_tracked_id，暂时不转换 (此分支已很少使用)
        spin_.phase = spin_motion_.get_theta();
        spin_.radius = spin_motion_.get_radius();
        spin_.radius_2 = spin_motion_.get_another_radius();
        spin_.update_level(spin_.omega);
        spin_.active = (spin_.level >= SpinLevel::LOW) && spin_motion_.valid();
    }

    // 6. 更新敌方颜色 (用于绘图)
    if (!filtered.empty()) {
        enemy_color_ = filtered[0].color;
    }

    prev_armors_ = filtered;
    prev_timestamp_ = timestamp;

    // ========== 输出到 PlotJuggler ==========
    {
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
        bool vehicle_valid = false;

        if (use_sp && sp_motion_.valid()) {
            vehicle_valid = true;
            center = sp_motion_.predict_center(0);
            velocity = sp_motion_.get_velocity();
            theta = sp_motion_.get_theta();
            omega = sp_motion_.get_omega();
            radius = sp_motion_.get_radius();
            radius_2 = sp_motion_.get_another_radius();
            dz = sp_motion_.get_dz();
        } else if (use_lmtd && lmtd_motion_.valid()) {
            vehicle_valid = true;
            center = lmtd_motion_.predict_center(0);
            velocity = lmtd_motion_.get_center_velocity();
            theta = lmtd_motion_.get_theta();
            omega = lmtd_motion_.get_omega();
            radius = lmtd_motion_.get_radius();
            radius_2 = lmtd_motion_.get_another_radius();
            dz = lmtd_motion_.get_dz();
        } else if (spin_motion_.valid()) {
            vehicle_valid = true;
            center = spin_motion_.predict_center(0);
            velocity = spin_motion_.get_velocity();
            theta = spin_motion_.get_theta();
            omega = spin_motion_.get_omega();
            radius = spin_motion_.get_radius();
            radius_2 = spin_motion_.get_another_radius();
            dz = spin_motion_.get_dz();
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
    // 模型选择
    const std::string motion_model = runtime_param::get_param<std::string>("AutoAim.Predictor.motion_model");
    const bool use_lmtd = (motion_model == "lmtd");
    const bool use_sp = (motion_model == "sp");

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
    bool spin_active = false;
    if (use_sp) {
        spin_active = (sp_motion_.get_spin_level() >= SpinLevel::LOW && sp_motion_.valid());
    } else if (use_lmtd) {
        spin_active = (lmtd_motion_.get_spin_level() >= SpinLevel::LOW && lmtd_motion_.valid());
    } else {
        spin_active = (spin_motion_.get_spin_level() >= SpinLevel::LOW && spin_motion_.valid());
    }

    if (spin_active) {
        // ========== 陀螺模式 ==========
        if (use_sp) {
            vs.center = sp_motion_.predict_center(dt);
            vs.velocity = sp_motion_.get_velocity();
        } else if (use_lmtd) {
            vs.center = lmtd_motion_.predict_center(dt);
            vs.velocity = lmtd_motion_.get_center_velocity();
        } else {
            vs.center = spin_motion_.predict_center(dt);
            vs.velocity = spin_motion_.get_velocity();
        }

        // 获取当前追踪装甲板的 ID
        int tracking_id = 0;
        if (use_sp) {
            tracking_id = sp_motion_.get_tracked_id();
        } else if (use_lmtd) {
            tracking_id = lmtd_motion_.get_tracked_id();
        } else {
            const auto* best_filter = armor_motion_.get_best(timestamp);
            if (best_filter) {
                tracking_id = best_filter->id();
            }
        }

        // 预测所有装甲板位置
        int armor_num = (enemy_type_ == EnemyType::OUTPOST) ? 3 : 4;
        vs.armor_count = armor_num;

        double local_best_score = -1;
        int best_idx = -1;

        double theta = 0;
        if (use_sp) {
            theta = sp_motion_.get_theta();
        } else if (use_lmtd) {
            theta = lmtd_motion_.get_theta();
        } else {
            theta = spin_motion_.get_theta();
        }

        for (int i = 0; i < armor_num; ++i) {
            auto& as = vs.armors[i];
            as.id = (i == 0) ? tracking_id : -1;  // 只有当前追踪的有 ID

            if (use_sp) {
                as.position = sp_motion_.predict_armor_pos(i, dt);
            } else if (use_lmtd) {
                as.position = lmtd_motion_.predict_armor_pos(i, dt);
            } else {
                as.position = spin_motion_.predict_armor_pos(i, dt);
            }

            as.velocity = vs.velocity;  // 近似用中心速度
            as.yaw = theta + i * (2.0 * M_PI / armor_num);
            as.visible = (i == 0);  // 只有当前追踪的可见
            as.last_seen = last_update_time_;

            // 评分: 越正对越好 (用 cos(装甲板朝向 - 视线方向))
            double armor_yaw = as.yaw;
            double view_yaw = std::atan2(as.position.y(), as.position.x());
            double angle_diff = std::abs(aimer::math::reduced_angle(armor_yaw - view_yaw - M_PI));
            as.score = std::cos(angle_diff);

            // 装甲板类型和朝向角
            as.type = correct_armor_type(ArmorType::SMALL, enemy_type_);
            as.z_to_v = angle_diff;

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
    lmtd_motion_.reset();
    sp_motion_.reset();
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
    // 模型选择
    const std::string motion_model = runtime_param::get_param<std::string>("AutoAim.Predictor.motion_model");
    const bool use_lmtd = (motion_model == "lmtd");
    const bool use_sp = (motion_model == "sp");

    // 计算距离上次更新的时间，用于判断数据是否过时
    double time_since_update = timestamp - last_update_time_;
    bool data_fresh = time_since_update < 0.1;  // 100ms 内认为数据新鲜

    // 预测时间 (用于绘图外推)
    double draw_dt = get_draw_predict_dt();

    // 颜色定义
    const cv::Scalar COLOR_DETECTED(0, 255, 0);    // 绿色: 检测到的
    const cv::Scalar COLOR_FILTERED(255, 200, 0);  // 蓝色: 滤波后的
    const cv::Scalar COLOR_CENTER(0, 0, 255);      // 红色: 旋转中心

    // SpinMotion/LmtdMotion/SpMotion 预测颜色: 未使用灰色，使用时用我方颜色
    cv::Scalar COLOR_SPIN;
    bool spin_valid = false;
    SpinLevel spin_level = SpinLevel::NONE;
    if (use_sp) {
        spin_valid = sp_motion_.valid();
        spin_level = sp_motion_.get_spin_level();
    } else if (use_lmtd) {
        spin_valid = lmtd_motion_.valid();
        spin_level = lmtd_motion_.get_spin_level();
    } else {
        spin_valid = spin_motion_.valid();
        spin_level = spin_motion_.get_spin_level();
    }
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

    // 3. 绘制 SpinMotion/LmtdMotion/SpMotion 预测 (口字形) - EKF 滤波后的
    // 即使未激活也绘制，方便调试
    // EKF 预测可以继续绘制，因为它有自己的时间预测能力
    if (spin_valid) {
        int armor_num = (enemy_type_ == EnemyType::OUTPOST) ? 3 : 4;
        double omega = 0, theta = 0;
        if (use_sp) {
            omega = sp_motion_.get_omega();
            theta = sp_motion_.get_theta();
        } else if (use_lmtd) {
            omega = lmtd_motion_.get_omega();
            theta = lmtd_motion_.get_theta();
        } else {
            omega = spin_motion_.get_omega();
            theta = spin_motion_.get_theta();
        }
        // 注意: predict_armor_pos 内部已经对 theta 做了 draw_dt 外推
        // 这里也要做同样的外推，保持一致
        double theta_predicted = theta + omega * draw_dt;

        // 绘制所有装甲板预测位置
        for (int i = 0; i < armor_num; ++i) {
            Eigen::Vector3d pos;
            if (use_sp) {
                pos = sp_motion_.predict_armor_pos(i, draw_dt);
            } else if (use_lmtd) {
                pos = lmtd_motion_.predict_armor_pos(i, draw_dt);
            } else {
                pos = spin_motion_.predict_armor_pos(i, draw_dt);
            }

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

        // 绘制旋转中心
        Eigen::Vector3d center;
        if (use_sp) {
            center = sp_motion_.predict_center(draw_dt);
        } else if (use_lmtd) {
            center = lmtd_motion_.predict_center(draw_dt);
        } else {
            center = spin_motion_.predict_center(draw_dt);
        }
        bool valid = false;
        cv::Point2f pt = aimer::tf::world_to_pixel(center, q_imu, valid);
        if (valid) {
            // 十字标记
            int s = 15;
            cv::line(img, pt - cv::Point2f(s, 0), pt + cv::Point2f(s, 0), COLOR_CENTER, 2);
            cv::line(img, pt - cv::Point2f(0, s), pt + cv::Point2f(0, s), COLOR_CENTER, 2);
            // 标注角速度和状态
            std::string model_str = use_sp ? "SP" : (use_lmtd ? "L" : "S");  // SP=SpMotion, L=LMTD, S=SpinMotion
            std::string state_str = spin_active ? "ON" : "OFF";
            cv::putText(img, fmt::format("w={:.1f} {}{}", omega, model_str, state_str),
                        pt + cv::Point2f(20, 0),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, COLOR_CENTER, 1);
        }
    }

    // 4. 绘制从 EKF 状态生成的装甲板 (洋红色)
    // rm.cv.fans 原版设计: 直接从 EKF 状态生成，不需要从观测反推
    // 参考 lmtd_top_model.cpp 的 draw_armors (第 492-514 行)
    auto active_armors = identifier_.get_active_armors(frame_count_);
    if (spin_valid && data_fresh && !active_armors.empty()) {
        const cv::Scalar COLOR_GEOMETRY(255, 0, 255);  // 洋红色

        // rm.cv.fans 原版: 直接从 EKF 状态生成所有装甲板
        std::vector<Eigen::Vector3d> all_armors;
        if (use_sp) {
            for (int i = 0; i < ((enemy_type_ == EnemyType::OUTPOST) ? 3 : 4); ++i) {
                all_armors.push_back(sp_motion_.predict_armor_pos(i, 0));
            }
        } else if (use_lmtd) {
            for (int i = 0; i < ((enemy_type_ == EnemyType::OUTPOST) ? 3 : 4); ++i) {
                all_armors.push_back(lmtd_motion_.predict_armor_pos(i, 0));
            }
        } else {
            const auto& best_armor = active_armors[0];
            double obs_theta_outward = best_armor.observation.z[obs::ARMOR_YAW] + M_PI;
            all_armors = spin_motion_.compute_all_armors_from_observation(best_armor.pos(), obs_theta_outward);
        }

        int armor_num = (enemy_type_ == EnemyType::OUTPOST) ? 3 : 4;

        // rm.cv.fans 原版: 用 EKF 的 theta 来画装甲板朝向
        double state_theta_outward = 0;
        if (use_sp) {
            state_theta_outward = sp_motion_.get_theta();
        } else if (use_lmtd) {
            state_theta_outward = lmtd_motion_.get_theta();
        }
        double obs_armor_yaw = active_armors[0].observation.z[obs::ARMOR_YAW];

        for (int i = 0; i < armor_num && i < static_cast<int>(all_armors.size()); ++i) {
            // draw_armor_rect 需要装甲板朝向 (INWARD)
            // SP/LMTD: OUTWARD + M_PI = INWARD
            double armor_yaw;
            if (use_sp || use_lmtd) {
                armor_yaw = state_theta_outward + i * (2.0 * M_PI / armor_num) + M_PI;
            } else {
                armor_yaw = obs_armor_yaw + i * (2.0 * M_PI / armor_num);
            }

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

        // 绘制中心和参数
        double r0 = 0, dz = 0, another_r = 0;
        Eigen::Vector3d center;
        if (use_sp) {
            r0 = sp_motion_.get_radius();
            dz = sp_motion_.get_dz();
            another_r = sp_motion_.get_another_radius();
            center = sp_motion_.predict_center(0);
        } else if (use_lmtd) {
            r0 = lmtd_motion_.get_radius();
            dz = lmtd_motion_.get_dz();
            another_r = lmtd_motion_.get_another_radius();
            center = lmtd_motion_.predict_center(0);
        } else {
            r0 = spin_motion_.get_radius();
            dz = spin_motion_.get_dz();
            another_r = spin_motion_.get_another_radius();
            center = spin_motion_.predict_center(0);
        }

        bool valid = false;
        cv::Point2f pt = aimer::tf::world_to_pixel(center, q_imu, valid);
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
            cv::putText(img, fmt::format("r={:.2f}/{:.2f}", r0, another_r),
                        pt + cv::Point2f(15, -5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, COLOR_GEOMETRY, 1);

            // dz 和双装甲板标记
            std::string dz_str = fmt::format("dz={:.2f}", dz);
            if (prev_armors_.size() == 2) {
                dz_str += " [2]";  // 标记双装甲板
            }
            if (use_sp || use_lmtd) {
                cv::putText(img, dz_str,
                            pt + cv::Point2f(15, 10),
                            cv::FONT_HERSHEY_SIMPLEX, 0.4, COLOR_GEOMETRY, 1);
            } else {
                cv::putText(img, fmt::format("dz={:.2f}/{:.2f}", dz, spin_motion_.get_another_dz()),
                            pt + cv::Point2f(15, 10),
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
                    if (use_sp) mode_str = "SP";
                    else if (use_lmtd) mode_str = "LMTD";
                    else mode_str = "SPIN";
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
