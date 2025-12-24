/**
 * @file armor_model.cpp
 * @brief 装甲板运动模型实现 - YPD坐标系 EKF
 */

#include "armor_model.hpp"

#include <cmath>

#include "plugin/param/runtime_parameter.hpp"

namespace autoaim::predictor {

// ============================================================================
// EKF 参数 (运行时读取)
// ============================================================================

namespace {

double get_q_pos() {
    return runtime_param::get_param<double>("AutoAim.Predictor.EKF.q_pos");
}

double get_q_vel() {
    return runtime_param::get_param<double>("AutoAim.Predictor.EKF.q_vel");
}

double get_r_angle() {
    return runtime_param::get_param<double>("AutoAim.Predictor.EKF.r_angle");
}

double get_r_dis_1m() {
    return runtime_param::get_param<double>("AutoAim.Predictor.EKF.r_dis_1m");
}

}  // namespace

// ============================================================================
// FilterThread
// ============================================================================

FilterThread::FilterThread(const ArmorData& armor, double timestamp, double credit_time)
    : armor_(armor), last_update_time_(timestamp), credit_time_(credit_time) {
    // XYZ → YPD
    math::YpdCoord ypd = math::xyz_to_ypd(armor.pos());

    // 初始化状态: [yaw, 0, pitch, 0, dis, 0]
    VectorX x0;
    x0 << ypd.yaw, 0, ypd.pitch, 0, ypd.dis, 0;
    ekf_.init(x0);
}

void FilterThread::update(const ArmorData& armor, double timestamp) {
    double dt = timestamp - last_update_time_;
    if (dt <= 0) return;

    // XYZ → YPD
    math::YpdCoord ypd = math::xyz_to_ypd(armor.pos());

    // 读取运行时参数
    double q_pos = get_q_pos();
    double q_vel = get_q_vel();
    double r_angle = get_r_angle();
    double r_dis_1m = get_r_dis_1m();

    // 构建噪声矩阵
    MatrixXX Q = MatrixXX::Zero();
    Q(0, 0) = Q(2, 2) = Q(4, 4) = q_pos * dt;
    Q(1, 1) = Q(3, 3) = Q(5, 5) = q_vel * dt;

    MatrixYY R = MatrixYY::Zero();
    R(0, 0) = R(1, 1) = r_angle;
    R(2, 2) = r_dis_1m * ypd.dis * ypd.dis;

    // 预测
    YpdCVPredict predict_func(dt);
    ekf_.predict_forward(predict_func, Q);

    // 处理角度±π跨越
    VectorX x = ekf_.get_x();
    double yaw_close = math::get_closest_angle(ypd.yaw, x[0]);
    double pitch_close = math::get_closest_angle(ypd.pitch, x[2]);

    // 观测更新
    VectorY y;
    y << yaw_close, pitch_close, ypd.dis;

    YpdDirectMeasure measure_func;
    ekf_.update_forward(measure_func, y, R);

    // 归一化角度
    VectorX x_new = ekf_.get_x();
    x_new[0] = math::normalize_angle(x_new[0]);
    x_new[2] = math::normalize_angle(x_new[2]);
    ekf_.set_x(x_new);

    // 保存
    armor_ = armor;
    last_update_time_ = timestamp;
}

bool FilterThread::credit(double current_time) const {
    return (current_time - last_update_time_) <= credit_time_;
}

math::YpdCoord FilterThread::predict_ypd(double timestamp) const {
    double dt = timestamp - last_update_time_;
    YpdCVPredict predict_func(dt);
    auto res = ekf_.predict(predict_func);
    return math::YpdCoord(res.x_p[0], res.x_p[2], res.x_p[4]);
}

math::YpdCoord FilterThread::predict_ypd_v(double timestamp) const {
    double dt = timestamp - last_update_time_;
    YpdCVPredict predict_func(dt);
    auto res = ekf_.predict(predict_func);
    return math::YpdCoord(res.x_p[1], res.x_p[3], res.x_p[5]);
}

Eigen::Vector3d FilterThread::predict_pos(double timestamp) const {
    return math::ypd_to_xyz(predict_ypd(timestamp));
}

Eigen::Vector3d FilterThread::predict_vel(double timestamp) const {
    math::YpdCoord ypd = predict_ypd(timestamp);
    math::YpdCoord ypd_v = predict_ypd_v(timestamp);

    // 雅可比矩阵 ∂xyz/∂ypd
    double cy = std::cos(ypd.yaw), sy = std::sin(ypd.yaw);
    double cp = std::cos(ypd.pitch), sp = std::sin(ypd.pitch);
    double d = ypd.dis;

    Eigen::Matrix3d J;
    J(0, 0) = -d * cp * sy;  J(0, 1) = -d * sp * cy;  J(0, 2) = cp * cy;
    J(1, 0) =  d * cp * cy;  J(1, 1) = -d * sp * sy;  J(1, 2) = cp * sy;
    J(2, 0) =  0;            J(2, 1) =  d * cp;       J(2, 2) = sp;

    Eigen::Vector3d v_ypd(ypd_v.yaw, ypd_v.pitch, ypd_v.dis);
    return J * v_ypd;
}

ArmorState FilterThread::get_armor_state(double timestamp) const {
    ArmorState as;
    as.id = armor_.id;
    as.type = armor_.type();
    as.z_to_v = armor_.z_to_v();
    as.position = predict_pos(timestamp);
    as.velocity = predict_vel(timestamp);
    as.visible = true;
    as.last_seen = last_update_time_;

    // 装甲板朝向: 用预测的方位角 + π (面向相机时法向与视线相反)
    math::YpdCoord ypd = predict_ypd(timestamp);
    as.yaw = ypd.yaw + M_PI;

    // 评分: z_to_v 越小越好
    as.score = std::max(0.0, 1.0 - std::abs(armor_.z_to_v()));

    return as;
}

// ============================================================================
// ArmorModel
// ============================================================================

ArmorModel::ArmorModel(double credit_time) : credit_time_(credit_time) {}

void ArmorModel::update(const std::vector<ArmorData>& armors, double timestamp) {
    // 更新或创建滤波器
    for (const auto& armor : armors) {
        auto it = filters_.find(armor.id);
        if (it == filters_.end()) {
            filters_.emplace(armor.id, FilterThread(armor, timestamp, credit_time_));
        } else {
            it->second.update(armor, timestamp);
        }
    }

    // 清理超时滤波器
    for (auto it = filters_.begin(); it != filters_.end();) {
        if (!it->second.credit(timestamp)) {
            it = filters_.erase(it);
        } else {
            ++it;
        }
    }
}

std::vector<ArmorState> ArmorModel::get_armor_states(double timestamp) const {
    std::vector<ArmorState> result;
    for (const auto& [id, filter] : filters_) {
        if (filter.credit(timestamp)) {
            result.push_back(filter.get_armor_state(timestamp));
        }
    }
    return result;
}

const FilterThread* ArmorModel::get_best(double timestamp) const {
    const FilterThread* best = nullptr;
    double best_z_to_v = 1e9;

    for (const auto& [id, filter] : filters_) {
        if (!filter.credit(timestamp)) continue;

        double z_to_v = std::abs(filter.armor().z_to_v());
        if (z_to_v < best_z_to_v) {
            best_z_to_v = z_to_v;
            best = &filter;
        }
    }

    return best;
}

void ArmorModel::reset() {
    filters_.clear();
}

}  // namespace autoaim::predictor
