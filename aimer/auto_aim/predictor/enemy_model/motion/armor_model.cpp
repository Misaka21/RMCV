/**
 * @file armor_model.cpp
 * @brief 装甲板运动模型实现
 */

#include "armor_model.hpp"

#include <cmath>

namespace autoaim::predictor {

// ============================================================================
// FilterThread
// ============================================================================

FilterThread::FilterThread(const ArmorData& armor, double timestamp, double credit_time)
    : armor_(armor), last_update_time_(timestamp), credit_time_(credit_time) {
    // 初始化状态
    x_(0) = armor.pos().x();
    x_(1) = 0;
    x_(2) = armor.pos().y();
    x_(3) = 0;
    x_(4) = armor.pos().z();
    x_(5) = 0;

    // 初始协方差
    P_.setIdentity();
    P_(0, 0) = P_(2, 2) = P_(4, 4) = 0.1;   // 位置
    P_(1, 1) = P_(3, 3) = P_(5, 5) = 1.0;   // 速度
}

void FilterThread::predict(double dt) {
    if (dt <= 0) return;

    // 状态转移矩阵
    Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
    F(0, 1) = dt;
    F(2, 3) = dt;
    F(4, 5) = dt;

    // 预测状态
    x_ = F * x_;

    // 过程噪声
    Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
    Q(0, 0) = Q(2, 2) = Q(4, 4) = q_pos_ * dt;
    Q(1, 1) = Q(3, 3) = Q(5, 5) = q_vel_ * dt;

    // 预测协方差
    P_ = F * P_ * F.transpose() + Q;
}

void FilterThread::correct(const Eigen::Vector3d& z_meas) {
    // 观测矩阵
    Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
    H(0, 0) = 1;
    H(1, 2) = 1;
    H(2, 4) = 1;

    // 观测噪声 (距离越远噪声越大)
    double dist = z_meas.norm();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * r_base_ * dist * dist;

    // 卡尔曼增益
    Eigen::Matrix3d S = H * P_ * H.transpose() + R;
    Eigen::Matrix<double, 6, 3> K = P_ * H.transpose() * S.inverse();

    // 残差
    Eigen::Vector3d y = z_meas - H * x_;

    // 更新状态
    x_ = x_ + K * y;

    // 更新协方差 (Joseph 形式更稳定)
    Eigen::Matrix<double, 6, 6> I_KH = Eigen::Matrix<double, 6, 6>::Identity() - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
}

void FilterThread::update(const ArmorData& armor, double timestamp) {
    double dt = timestamp - last_update_time_;

    // 预测
    predict(dt);

    // 更新
    correct(armor.pos());

    // 保存
    armor_ = armor;
    last_update_time_ = timestamp;
}

bool FilterThread::credit(double current_time) const {
    return (current_time - last_update_time_) <= credit_time_;
}

Eigen::Vector3d FilterThread::predict_pos(double timestamp) const {
    double dt = timestamp - last_update_time_;
    return Eigen::Vector3d(
        x_(0) + x_(1) * dt,
        x_(2) + x_(3) * dt,
        x_(4) + x_(5) * dt
    );
}

Eigen::Vector3d FilterThread::predict_vel(double /*timestamp*/) const {
    return Eigen::Vector3d(x_(1), x_(3), x_(5));
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
            // 新建滤波器
            filters_.emplace(armor.id, FilterThread(armor, timestamp, credit_time_));
        } else {
            // 更新现有滤波器
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
        result.push_back(filter.get_armor_state(timestamp));
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
